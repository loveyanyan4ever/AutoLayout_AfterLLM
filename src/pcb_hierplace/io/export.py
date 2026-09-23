"""Append-only pose/outline edits followed by full reparse and rule validation."""
from __future__ import annotations
from dataclasses import replace
from pathlib import Path
import json
import os
import tempfile
import zipfile
import numpy as np
from .epro2 import ProjectArchive,parse_project,SCALE
from ..core.schema import PlacementError,Status,file_hash,finite_number
from ..core.geometry import bbox,rotation,angle_equal
from ..constraints.validator import validate_placement
from ..reporting import write_json


def compare_geometry(snapshot,readback,eps):
    """Native readback must preserve actual bodies and every pad, not only net counts."""
    for c in readback.components:
        old=snapshot.by_ref[c.ref]
        if c.footprint!=old.footprint or c.locked!=old.locked or len(c.body)!=len(old.body) or not np.allclose(c.body,old.body,atol=eps,rtol=0):
            raise PlacementError('ROUNDTRIP_GEOMETRY',f'{c.ref} 封装/本体/锁定状态改变')
        op={p.id:p for p in old.pads}; npads={p.id:p for p in c.pads}
        if set(op)!=set(npads): raise PlacementError('ROUNDTRIP_GEOMETRY',f'{c.ref} 焊盘集合改变')
        for pid,p in npads.items():
            q=op[pid]
            if ((p.number,p.net,p.layer)!=(q.number,q.net,q.layer) or abs(p.hole_mm-q.hole_mm)>eps
                or len(p.polygon)!=len(q.polygon) or not np.allclose(p.polygon,q.polygon,atol=eps,rtol=0)
                or not np.allclose(p.center,q.center,atol=eps,rtol=0)
                or len(p.hole_polygon)!=len(q.hole_polygon)
                or (p.hole_polygon and not np.allclose(p.hole_polygon,q.hole_polygon,atol=eps,rtol=0))
                or ((p.hole_center is None)!=(q.hole_center is None))
                or (p.hole_center is not None and not np.allclose(p.hole_center,q.hole_center,atol=eps,rtol=0))):
                raise PlacementError('ROUNDTRIP_GEOMETRY',f'{c.ref}.{p.number} 焊盘几何/连接改变')


def _publish_pair(project_tmp,project_path,report_tmp,report_path):
    """Publish two already-validated files without replacing any existing path.

    A directory cannot atomically commit two names. On a failed second publish,
    remove only the link we created, leaving pre-existing/racing files intact.
    Both temporaries live in the destination directory, so hard links are local.
    """
    published=[]
    try:
        for src,dst in [(project_tmp,project_path),(report_tmp,report_path)]:
            os.link(src,dst)
            published.append((src,dst))
    except OSError as e:
        for src,dst in reversed(published):
            try:
                if os.path.samestat(os.stat(src),os.stat(dst)): os.unlink(dst)
            except FileNotFoundError:
                pass
        raise PlacementError('EXPORT_COMMIT',f'输出发布失败，未覆盖已有文件: {e}') from e


def export_project(problem,record,target):
    state=record.state; snapshot=problem.design
    if not record.validation.model_feasible or not validate_placement(problem,state).model_feasible:
        raise PlacementError('EXPORT_NOT_FEASIBLE','禁止正式导出非法候选')
    out=Path(target).resolve(); source=Path(snapshot.source_path).resolve()
    sidecar=out.with_suffix('.export.json')
    if out==source or os.path.lexists(out) or os.path.lexists(sidecar):
        raise PlacementError('EXPORT_OVERWRITE','工程与 .export.json 必须都使用不存在的新文件路径')
    if file_hash(source)!=snapshot.source_hash: raise PlacementError('SOURCE_CHANGED','源工程已更改，不能写回旧布局')
    arc=ProjectArchive(source); records={str(r.outer['id']):r for r in arc.of(snapshot.board_id,'COMPONENT')}
    scale=SCALE[snapshot.source_unit]; ox,oy=snapshot.origin; appended=[]
    def line(outer,inner):
        try:
            return json.dumps(outer,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'||'+json.dumps(inner,ensure_ascii=False,separators=(',',':'),allow_nan=False)+'|\n'
        except ValueError as e:
            raise PlacementError('EXPORT_NONFINITE','导出记录包含非有限数值，未发布输出') from e
    updated=[];updated_attributes=[]
    clock=max([getattr(arc,'max_ticket',0)]+[r.outer.get('ticket',0) for r in arc.records.values()])
    def newer(rec):
        nonlocal clock
        clock+=1
        outer=dict(rec.outer);outer['ticket']=clock
        return outer
    moved={}
    def set_position(payload,x,y):
        # Some v3 documents retain both spellings. Keep every existing alias
        # synchronized so the next import cannot see conflicting coordinates.
        for primary,alias,value in [('x','positionX',x),('y','positionY',y)]:
            keys=[key for key in (primary,alias) if key in payload] or [alias]
            for key in keys: payload[key]=round(float(value),9)
    for c in snapshot.components:
        p=state.by_ref[c.ref]
        if problem.motions[c.ref]=='fixed_pose': continue
        if (abs(p.x-c.source_pose.x)<=1e-12 and abs(p.y-c.source_pose.y)<=1e-12 and angle_equal(p.angle,c.source_pose.angle)):
            continue
        rec=records[c.uuid]; payload=dict(rec.inner)
        set_position(payload,p.x/scale+ox,-p.y/scale+oy)
        payload['angle']=round((-p.angle)%360,9)
        outer=newer(rec)
        appended.append(line(outer,payload)); updated.append(c.ref)
        moved[c.uuid]=(c.source_pose,p)
    # PCB instance attributes have world positions, independent of COMPONENT.
    # Preserve their local relationship (and their key/value/style) under motion.
    for rec in arc.of(snapshot.board_id,'ATTR'):
        a=rec.inner; pair=moved.get(str(a.get('parentId')))
        if pair is None: continue
        xkey='x' if 'x' in a else 'positionX';ykey='y' if 'y' in a else 'positionY'
        if a.get(xkey) is None or a.get(ykey) is None: continue
        old,new=pair;delta=new.angle-old.angle
        ax=finite_number(a[xkey],f'ATTR {rec.outer.get("id")}.{xkey}')
        ay=finite_number(a[ykey],f'ATTR {rec.outer.get("id")}.{ykey}')
        for axis,value in [('x',ax),('y',ay)]:
            alias='position'+axis.upper()
            if axis in a and alias in a and finite_number(a[alias],f'ATTR {alias}')!=value:
                raise PlacementError('ATTR_POSITION',f'ATTR {rec.outer.get("id")}: {axis}/{alias} 相互冲突')
        offset=np.array([(ax-ox)*scale-old.x,-(ay-oy)*scale-old.y])
        position=rotation(delta)@offset+[new.x,new.y]
        payload=dict(a)
        set_position(payload,position[0]/scale+ox,-position[1]/scale+oy)
        if a.get('angle') is not None:
            payload['angle']=round((finite_number(a['angle'],f'ATTR {rec.outer.get("id")}.angle')-delta)%360,9)
        appended.append(line(newer(rec),payload));updated_attributes.append(str(rec.outer.get('id')))
    from .epro2 import decode_id
    outline_layers=set()
    for rec in arc.of(snapshot.board_id,'LAYER'):
        if rec.inner.get('layerType')=='OUTLINE':
            lid=decode_id(rec.outer.get('id'));outline_layers.add(lid[-1] if isinstance(lid,list) else lid)
    def on_outline(r):
        lid=decode_id(r.inner.get('layerId'));lid=lid[-1] if isinstance(lid,list) else lid
        return r.inner.get('polyType')=='BOARD_OUTLINE' or lid in outline_layers
    outlines=[r for r in arc.of(snapshot.board_id,'POLY') if on_outline(r)]
    if problem.estimated:
        if len(outlines)>1:
            raise PlacementError('OUTLINE_WRITE_UNSUPPORTED','估算模式暂不替换多记录板框；固定模式可原样保留',Status.UNSUPPORTED_FEATURE)
        if outlines:
            outer=newer(outlines[0])
            payload=dict(outlines[0].inner)
        else:
            clock+=1
            outer={'type':'POLY','id':'pcb_hierplace_outline_'+state.digest[:12],'ticket':clock}
            native=snapshot.adapter=='easyeda_pro_v3'
            if native and len(outline_layers)!=1:
                raise PlacementError('OUTLINE_LAYER','原生板框导出需要唯一 OUTLINE 层')
            payload={'polyType':'NORMAL' if native else 'BOARD_OUTLINE',
                     'layerId':next(iter(outline_layers)) if outline_layers else 11,
                     'width':.1/scale if native else 0.,'locked':False,'zIndex':0}
            if native: payload.update(partitionId='',groupID=0,netName='')
        path=[]
        points=list(state.outline)+[state.outline[0]]
        for i,(x,y) in enumerate(points):
            if i: path.append('L')
            path.extend([round(x/scale+ox,9),round(-y/scale+oy,9)])
        payload['path']=path; appended.append(line(outer,payload))
    lines=list(arc.lines); at=arc.insert_at[snapshot.board_id]
    if at and not lines[at-1].endswith(('\n','\r')): lines[at-1]+='\n'
    raw=''.join(lines[:at]+appended+lines[at:]).encode('utf-8')
    if arc.bom: raw=b'\xef\xbb\xbf'+raw
    out.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(suffix='.epro2',dir=out.parent); os.close(fd)
    report_tmp=None
    try:
        with zipfile.ZipFile(tmp,'w',zipfile.ZIP_DEFLATED) as z:
            for name,data in arc.members.items(): z.writestr(name,raw if name==arc.log_name else data)
        readback=parse_project(tmp,snapshot.board_id,{'source_unit':snapshot.source_unit,'geometry':problem.cfg['geometry']})
        if {(c.uuid,c.ref) for c in snapshot.components}!={(c.uuid,c.ref) for c in readback.components}:
            raise PlacementError('EXPORT_OBJECTS','写回器件集合不一致',Status.NUMERICAL_FAILURE)
        if snapshot.nets!=readback.nets: raise PlacementError('EXPORT_NETLIST','完整网络连接变化',Status.NUMERICAL_FAILURE)
        compare_geometry(snapshot,readback,problem.eps)
        for c in readback.components:
            want=state.by_ref[c.ref]; got=c.source_pose
            if max(abs(want.x-got.x),abs(want.y-got.y))>problem.eps or abs((want.angle-got.angle+180)%360-180)>1e-7:
                raise PlacementError('EXPORT_POSE',f'{c.ref}: 写回位姿不一致',Status.NUMERICAL_FAILURE)
        if max(abs(a-b) for a,b in zip(bbox(state.outline),bbox(readback.outline)))>problem.eps:
            raise PlacementError('EXPORT_OUTLINE','写回板框不一致',Status.NUMERICAL_FAILURE)
        actual=replace(state,poses=tuple(c.source_pose for c in readback.components))
        validation=validate_placement(problem,actual)
        if not validation.model_feasible:
            raise PlacementError('EXPORT_VALIDATION','写回后几何未通过约束检查',Status.NUMERICAL_FAILURE,
                                 validation.as_dict())
        # New records must belong only to the selected board; other document records stay identical.
        roundtrip=ProjectArchive(tmp)
        before_other={k:r.inner for k,r in arc.records.items() if k[0]!=snapshot.board_id}
        after_other={k:r.inner for k,r in roundtrip.records.items() if k[0]!=snapshot.board_id}
        if before_other!=after_other: raise PlacementError('EXPORT_OTHER_BOARD','非目标文档改变',Status.NUMERICAL_FAILURE)
        # Preserve every non-log ZIP payload, not just the selected object model.
        if set(roundtrip.members)!=set(arc.members) or any(
            data!=roundtrip.members[name] for name,data in arc.members.items() if name!=arc.log_name):
            raise PlacementError('EXPORT_PAYLOAD','非日志附件发生变化',Status.NUMERICAL_FAILURE)
        report={'status':'FEASIBLE','updated_components':updated,'updated_attributes':updated_attributes,
                'board_written':problem.estimated,'output_hash':file_hash(tmp),
                'roundtrip_validation':'PASSED','native_open_validation':'PENDING',
                'preserved_original_records':True,'preserved_non_log_members':True,
                'safety_signoff':'NOT_PERFORMED'}
        fd,report_tmp=tempfile.mkstemp(suffix='.export.json',dir=out.parent);os.close(fd)
        write_json(report_tmp,report)
        _publish_pair(tmp,out,report_tmp,sidecar)
    finally:
        if os.path.exists(tmp): os.unlink(tmp)
        if report_tmp is not None and os.path.exists(report_tmp): os.unlink(report_tmp)
    return report
