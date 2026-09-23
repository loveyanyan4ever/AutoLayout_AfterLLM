"""Conservative adapter for the log structure in the supplied reference archive.

Source: X right/Y down; numeric source rotation maps to -angle internally.
Only explicit source units are accepted. Native-editor interoperability is pending.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import json
import math
import zipfile
import numpy as np
from ..core.schema import Component, Pad, Pose, Net, DesignSnapshot, PlacementError, Status, file_hash,finite_number
from ..core.geometry import rect, bbox, union_bbox, circle_envelope, rotate, is_rectangle, validate_convex
from .native_geometry import pad_geometry,source_body,layer_type_map

SCALE={'mil':.0254,'mm':1.,'0.01inch':.254,'0.01mm':.01}
METADATA_RECORDS={'ACTIVE_LAYER','PRIMITIVE','LAYER_PHYS','RULE_TEMPLATE','RULE','RULE_SELECTOR',
                  'SILK_OPTS','PREFERENCE','PANELIZE','LAYER_FILL','ELE_PLACEHOLDER'}
SAFE_PCB={'META','CANVAS','LAYER','COMPONENT','ATTR','PAD_NET','NET','POLY','STRING'}|METADATA_RECORDS
SAFE_FP={'META','CANVAS','LAYER','ATTR','PAD','POLY','STRING','FILL','VIA','NET'}|METADATA_RECORDS
ROUTED={'LINE','TRACK','VIA','COPPER','COPPERAREA','POUR','FILL','REGION'}
# Explicit non-copper layers; the layer table must agree, never infer by number.
DOCUMENT_LAYERS={'TOP_SILK','BOT_SILK','TOP_PASTE_MASK','BOT_PASTE_MASK','TOP_SOLDER_MASK','BOT_SOLDER_MASK',
                 'DOCUMENT','COMPONENT_SHAPE','COMPONENT_MARKING','PIN_SOLDERING','PIN_FLOATING','COMPONENT_MODEL'}


def strict_json(value,location):
    """Python's JSON extension accepts NaN/Infinity; native JSON does not.
    Reject them even in attributes/metadata outside the layout geometry model.
    """
    def reject(constant):
        raise PlacementError('NONFINITE_JSON',f'{location}: 非标准 JSON 常量 {constant}')
    def number(token):
        result=float(token)
        if not math.isfinite(result):
            raise PlacementError('NONFINITE_JSON',f'{location}: 数值超出有限浮点范围 {token}')
        return result
    return json.loads(value,parse_constant=reject,parse_float=number)


def opaque_id(value):
    return json.dumps(value,sort_keys=True,separators=(',',':')) if not isinstance(value,str) else value


def decode_id(value):
    if isinstance(value,str) and value.startswith('['):
        try: return json.loads(value)
        except ValueError: return value
    return value


@dataclass
class Record:
    doc: str
    outer: dict
    inner: dict | None
    line: int
    client: str = ''


class ProjectArchive:
    def __init__(self,path):
        self.path=str(Path(path).resolve())
        try:
            with zipfile.ZipFile(path) as z:
                if len(set(z.namelist()))!=len(z.namelist()):
                    raise PlacementError('DUPLICATE_ZIP_ENTRY','工程包含重复压缩条目')
                self.members={name:z.read(name) for name in z.namelist()}
        except (OSError,zipfile.BadZipFile) as e:
            raise PlacementError('EPRO2_READ',str(e)) from e
        logs=[n for n in self.members if n.lower().endswith('.epru')]
        if len(logs)!=1:
            raise PlacementError('LOG_COUNT','首版要求恰好一个 epru 日志；不猜测选取',Status.UNSUPPORTED_FEATURE)
        self.log_name=logs[0]
        raw=self.members[self.log_name]
        self.bom=raw.startswith(b'\xef\xbb\xbf')
        try: self.lines=raw.decode('utf-8-sig').splitlines(keepends=True)
        except UnicodeError as e: raise PlacementError('LOG_ENCODING',str(e)) from e
        self.docs={}; self.records={}; self.insert_at={}; self.tombstones={}; self.doc_headers={};self.max_ticket=0
        self.project_metadata={}
        if 'project2.json' in self.members:
            try:self.project_metadata=strict_json(self.members['project2.json'].decode('utf-8-sig'),'project2.json')
            except (ValueError,UnicodeError) as e:raise PlacementError('PROJECT_METADATA','project2.json 无法识别') from e
        doc=None;client=''
        for i,line in enumerate(self.lines):
            if not line.strip(): continue
            if '||' not in line:
                raise PlacementError('LOG_RECORD',f'不支持的日志行 {i+1}')
            a,b=line.strip().split('||',1)
            try:
                head=strict_json(a,f'日志行 {i+1} 外层'); payload=b[:-1] if b.endswith('|') else b
                # Official v3 atomic deletion: empty inner data. Native exports
                # contain both ||""| and the equivalent empty delimiter |||.
                body=strict_json(payload,f'日志行 {i+1} 内层') if payload else None
                if body=='':body=None
                if body is None and payload not in ('','""'):
                    raise PlacementError('LOG_PAYLOAD','JSON null 不等同于已定义空字符串墓碑',Status.UNSUPPORTED_FEATURE)
            except ValueError as e:
                raise PlacementError('LOG_RECORD',f'日志行 {i+1} 不是已支持的 JSON 记录') from e
            if not isinstance(head,dict) or (body is not None and not isinstance(body,dict)):
                raise PlacementError('LOG_PAYLOAD',f'日志行 {i+1} 负载类型不支持',Status.UNSUPPORTED_FEATURE)
            typ=head.get('type','')
            if not isinstance(typ,str) or not typ:raise PlacementError('LOG_TYPE','记录缺少有效类型')
            current_ticket=head.get('ticket',0)
            if isinstance(current_ticket,bool) or not isinstance(current_ticket,int):
                raise PlacementError('TICKET','ticket 必须为整数')
            self.max_ticket=max(self.max_ticket,current_ticket)
            if typ=='DELETE_DOC':
                flag=(body or {}).get('isDelete')
                if (not isinstance(body,dict) or set(body)!={'isDelete'} or
                    not isinstance(flag,(bool,int)) or flag not in (0,1,False,True)):
                    raise PlacementError('DELETE_SEMANTICS','DELETE_DOC 需要明确 isDelete 布尔标记',Status.UNSUPPORTED_FEATURE)
            elif 'DELETE' in typ.upper() or any(head.get(k) or (body or {}).get(k) for k in ('deleted','isDeleted')):
                raise PlacementError('DELETE_SEMANTICS','检测到未验证的删除语义，停止解析',Status.UNSUPPORTED_FEATURE)
            if typ=='DOCHEAD':
                if body is None:raise PlacementError('DOCHEAD','文档头不得为空')
                doc=body.get('uuid')
                if not doc or not body.get('docType'): raise PlacementError('DOCHEAD','缺少文档类型/UUID')
                if doc in self.docs and self.docs[doc]!=body['docType']:
                    raise PlacementError('DOCHEAD','同 UUID 文档类型冲突')
                self.docs[doc]=body['docType']; self.insert_at[doc]=i+1
                client=str(body.get('client',''));self.doc_headers[doc]=dict(body)
                continue
            if not doc: raise PlacementError('DOCHEAD','记录出现在 DOCHEAD 之前')
            self.insert_at[doc]=i+1
            key=(doc,typ,opaque_id(head.get('id')))
            ticket=head.get('ticket',0)
            if isinstance(ticket,bool) or not isinstance(ticket,int): raise PlacementError('TICKET','ticket 必须为整数')
            prev=self.records.get(key)
            if prev and ticket==prev.outer.get('ticket',0) and body!=prev.inner:
                if not client or not prev.client or client==prev.client:
                    raise PlacementError('TICKET_CONFLICT',f'同 ticket/client 不同负载: {key}')
            if prev is None or ticket>prev.outer.get('ticket',0) or (ticket==prev.outer.get('ticket',0) and client<prev.client):
                self.records[key]=Record(doc,head,body,i,client)
        self.tombstones={k:r for k,r in self.records.items() if r.inner is None}
        self.records={k:r for k,r in self.records.items() if r.inner is not None}
        self.deleted_docs=set()
        for d in self.docs:
            markers=[r for (rd,typ,_),r in self.records.items() if rd==d and typ=='DELETE_DOC']
            if len(markers)>1:raise PlacementError('DELETE_SEMANTICS','同文档存在多个 DELETE_DOC 身份，不能猜测',Status.UNSUPPORTED_FEATURE)
            if markers and markers[0].inner['isDelete']:self.deleted_docs.add(d)
        self.all_doc_types=dict(self.docs)
        self.docs={d:t for d,t in self.docs.items() if d not in self.deleted_docs}

    def of(self,doc,typ=None):
        if doc in self.deleted_docs:return []
        return [r for (d,t,_),r in self.records.items() if d==doc and t!='DELETE_DOC' and (typ is None or t==typ)]

    def boards(self):
        return [{'board_id':d,'components':len(self.of(d,'COMPONENT')),
                 'title':next((r.inner.get('title','') for r in self.of(d,'META')),'')}
                for d,t in self.docs.items() if t=='PCB']


def parse_path(path):
    if not isinstance(path,list): raise PlacementError('OUTLINE_PATH','仅支持数值/命令数组板框',Status.UNSUPPORTED_FEATURE)
    points=[]; i=0
    while i<len(path):
        if isinstance(path[i],str):
            if path[i]=='L': i+=1
            elif path[i]=='Z':
                if points and points[-1]!=points[0]: points.append(points[0])
                i+=1; continue
            else: raise PlacementError('OUTLINE_CURVE',f'首版不支持板框命令 {path[i]}',Status.UNSUPPORTED_FEATURE)
        if i+1>=len(path) or not all(isinstance(v,(float,int)) for v in path[i:i+2]):
            raise PlacementError('OUTLINE_PATH','板框路径坐标不完整')
        points.append((float(path[i]),float(path[i+1]))); i+=2
    return points


def stitch(paths,eps=1e-6):
    pending=[list(p) for p in paths if len(p)>=2]; loops=[]
    while pending:
        chain=pending.pop(0)
        while np.linalg.norm(np.array(chain[-1])-chain[0])>eps:
            found=False
            for i,p in enumerate(pending):
                if np.linalg.norm(np.array(chain[-1])-p[0])<=eps:
                    chain.extend(p[1:]); pending.pop(i); found=True; break
                if np.linalg.norm(np.array(chain[-1])-p[-1])<=eps:
                    chain.extend(list(reversed(p))[1:]); pending.pop(i); found=True; break
            if not found: raise PlacementError('OPEN_OUTLINE','板框未闭合；不能使用凸包替代')
        loops.append(chain[:-1])
    return loops


def _layer(value):
    value=decode_id(value)
    if isinstance(value,list): value=value[-1]
    return {1:'top',2:'bottom',12:'both'}.get(value,'unknown')


def layer_types(arc,doc):
    return layer_type_map(arc.of(doc,'LAYER'))


def source_position(payload,axis):
    aliases=(axis,'position'+axis.upper())
    present=[finite_number(payload[k],'component '+k) for k in aliases if k in payload]
    if not present:raise PlacementError('COMPONENT_POSITION',f'缺少器件 {aliases[0]}/{aliases[1]}')
    if len(present)==2 and present[0]!=present[1]:
        raise PlacementError('COMPONENT_POSITION',f'器件位置字段 {aliases[0]}/{aliases[1]} 相互冲突')
    return present[0]


def is_outline_record(record,layers):
    return record.outer['type']=='POLY' and (record.inner.get('polyType')=='BOARD_OUTLINE' or
                                             layers.get(record.inner.get('layerId'))=='OUTLINE')


def orphan_pour_count(arc,board_id):
    """Only detached derived results can be excluded from the layout model.
    Official POURED data belongs to a POUR target. No result is deleted from
    the raw archive. A target in any active/tombstoned record is not guessed.
    """
    rows=arc.of(board_id,'POURED')
    if not rows:return 0
    if any(r.outer['type'] in ROUTED for r in arc.of(board_id)):
        raise PlacementError('ROUTED_INPUT_UNSUPPORTED','POURED 与活动导体同时存在，不能忽略结果',Status.UNSUPPORTED_FEATURE)
    ids={str(r.outer.get('id')) for r in [*arc.records.values(),*arc.tombstones.values()]}
    for r in rows:
        rid=decode_id(r.outer.get('id'))
        if not isinstance(rid,list) or len(rid)!=2 or rid[0]!='POURED':
            raise PlacementError('POURED_TARGET','不能确定覆铜结果所属边框',Status.UNSUPPORTED_FEATURE)
        target=r.inner.get('targetId',rid[1])
        if str(target)!=str(rid[1]) or str(target) in ids:
            raise PlacementError('ROUTED_INPUT_UNSUPPORTED','存在或无法排除 POURED 的父铜区',Status.UNSUPPORTED_FEATURE)
        if set(r.inner)-{'targetId','pourFill'} or not isinstance(r.inner.get('pourFill'),list):
            raise PlacementError('POURED_PAYLOAD','覆铜结果结构尚未适配',Status.UNSUPPORTED_FEATURE)
    return len(rows)


def parse_project(path,board_id,adapter_options):
    arc=ProjectArchive(path)
    unit=adapter_options.get('source_unit')
    if unit not in SCALE: raise PlacementError('SOURCE_UNIT_REQUIRED','必须明确 --source-unit 或 input.source_unit')
    boards=arc.boards()
    if board_id is None:
        if len(boards)!=1: raise PlacementError('BOARD_SELECTION_REQUIRED','多板工程必须提供 board_id',details={'boards':boards})
        board_id=boards[0]['board_id']
    if board_id not in {b['board_id'] for b in boards}: raise PlacementError('BOARD_SELECTION','未知 board_id')
    scale=SCALE[unit]; layers=layer_types(arc,board_id);orphaned_pours=orphan_pour_count(arc,board_id)
    for r in arc.of(board_id):
        typ=r.outer['type']
        if typ=='POURED':continue  # proven detached derived cache, preserved verbatim
        if typ in ROUTED:
            raise PlacementError('ROUTED_INPUT_UNSUPPORTED',f'发现布线/导体记录 {typ}',Status.UNSUPPORTED_FEATURE)
        if typ not in SAFE_PCB:
            raise PlacementError('UNKNOWN_PCB_RECORD',f'未验证 PCB 记录类型 {typ}',Status.UNSUPPORTED_FEATURE)
        if typ=='PANELIZE' and r.inner.get('on'):
            raise PlacementError('PANELIZE_UNSUPPORTED','活动拼板尚未适配',Status.UNSUPPORTED_FEATURE)
        if typ=='LAYER_FILL' and r.inner.get('fill'):
            raise PlacementError('ROUTED_INPUT_UNSUPPORTED','存在活动层填充',Status.UNSUPPORTED_FEATURE)
        if typ=='POLY' and not is_outline_record(r,layers) and r.inner.get('polyType') not in ('SILK','DOCUMENT') and layers.get(r.inner.get('layerId')) not in DOCUMENT_LAYERS:
            raise PlacementError('UNKNOWN_PCB_POLY','非板框 POLY 需显式适配其铜/禁区语义',Status.UNSUPPORTED_FEATURE)
    canvas=arc.of(board_id,'CANVAS')
    if len(canvas)!=1: raise PlacementError('CANVAS','需要唯一 CANVAS')
    origin=(finite_number(canvas[0].inner.get('originX',0),'canvas originX'),
            finite_number(canvas[0].inner.get('originY',0),'canvas originY'))
    def world(x,y): return ((float(x)-origin[0])*scale,-(float(y)-origin[1])*scale)
    outline_records=[r for r in arc.of(board_id,'POLY') if is_outline_record(r,layers)]
    loops=stitch([parse_path(r.inner.get('path')) for r in outline_records])
    if len(loops)>1: raise PlacementError('OUTLINE_HOLES','多环/内孔板框尚未支持',Status.UNSUPPORTED_FEATURE)
    outline=tuple(world(x,y) for x,y in loops[0]) if loops else ()
    if outline and not is_rectangle(outline):
        raise PlacementError('NONRECTANGULAR_OUTLINE','首版要求轴对齐矩形板框',Status.UNSUPPORTED_FEATURE)
    attrs={}
    for r in arc.of(board_id,'ATTR'):
        a=r.inner; attrs.setdefault(str(a.get('parentId')), {})[a.get('key')]=a.get('value')
    pin_net={}
    for r in arc.of(board_id,'PAD_NET'):
        rid=decode_id(r.outer.get('id'))
        if not isinstance(rid,list) or len(rid)!=4 or rid[0]!='PAD_NET' or any(not isinstance(v,str) or not v for v in rid[1:]):
            raise PlacementError('PAD_NET_ID','PAD_NET 必须包含类型、器件UUID、焊盘号、稳定pad ID')
        k=tuple(rid[1:]);net=r.inner.get('padNet') or ''
        if not isinstance(net,str):raise PlacementError('PAD_NET_VALUE','网络名必须为字符串')
        if k in pin_net and net!=pin_net[k]:raise PlacementError('PAD_NET_CONFLICT',f'焊盘网络冲突: {k}')
        pin_net[k]=net
    geometry=adapter_options.get('geometry',{})
    overrides=geometry.get('body_overrides',{})
    approved=set(geometry.get('approved_pad_envelopes',[]))
    components=[]; allpins={}
    seen=set()
    for r in arc.of(board_id,'COMPONENT'):
        if not isinstance(r.outer.get('id'),str) or not r.outer['id']:
            raise PlacementError('COMPONENT_ID','器件 UUID 必须为非空字符串')
        a=attrs.get(str(r.outer['id']),{}); payload=r.inner
        ref=a.get('Designator'); fp=a.get('Footprint'); uid=str(r.outer['id'])
        if not ref or ref in seen: raise PlacementError('COMPONENT_ID','器件位号缺失或重复，禁止丢件')
        seen.add(ref)
        if not fp and a.get('Device'):
            device=a['Device'];metas=arc.of(device,'META')
            if arc.docs.get(device)=='DEVICE' and len(metas)==1:
                fp=metas[0].inner.get('attributes',{}).get('Footprint')
        if arc.docs.get(fp)!='FOOTPRINT': raise PlacementError('MISSING_FOOTPRINT',f'{ref} 缺少封装文档')
        layer=_layer(payload.get('layerId'))
        if layer!='top': raise PlacementError('COMPONENT_SIDE',f'{ref} 不是首版支持的顶面器件',Status.UNSUPPORTED_FEATURE)
        fp_layers=layer_types(arc,fp)
        for fr in arc.of(fp):
            if fr.outer['type'] not in SAFE_FP:
                raise PlacementError('UNKNOWN_FOOTPRINT_RECORD',f'{fp}: {fr.outer["type"]}',Status.UNSUPPORTED_FEATURE)
            if fr.outer['type']=='POLY' and fr.inner.get('polyType') not in ('SILK','DOCUMENT','COURTYARD','ASSEMBLY') and fp_layers.get(fr.inner.get('layerId')) not in DOCUMENT_LAYERS:
                raise PlacementError('UNKNOWN_FOOTPRINT_POLY',f'{fp}: 非文档几何需要适配',Status.UNSUPPORTED_FEATURE)
            if fr.outer['type']=='FILL' and (fr.inner.get('netName') or fp_layers.get(fr.inner.get('layerId')) not in DOCUMENT_LAYERS):
                raise PlacementError('FOOTPRINT_COPPER_FILL',f'{fp}: 铜填充需独立网络建模',Status.UNSUPPORTED_FEATURE)
        pads=[]; identifiers=set()
        for pr in arc.of(fp,'PAD'):
            if not isinstance(pr.outer.get('id'),str) or not pr.outer['id'] or pr.inner.get('num') in (None,''):
                raise PlacementError('PAD_NUMBER',f'{ref} 焊盘身份缺失')
            p=pr.inner; num=str(p.get('num','')); pad_id=str(pr.outer['id'])
            if not num or not pad_id or pad_id in identifiers: raise PlacementError('PAD_NUMBER',f'{ref} 焊盘身份缺失/重复')
            identifiers.add(pad_id)
            poly,center,source,hole_mm,hole_poly,hole_center=pad_geometry(p,scale,geometry.get('polygon_path_frame','reject'))
            net=pin_net.get((uid,num,pad_id),'')
            if _layer(p.get('layerId'))=='unknown':
                raise PlacementError('PAD_LAYER',f'{ref}.{num} 未识别焊盘层',Status.UNSUPPORTED_FEATURE)
            pads.append(Pad(pad_id,num,poly,center,net,_layer(p.get('layerId')),hole_mm,source,hole_poly,hole_center))
            allpins.setdefault(net,[]).append((ref,pad_id)) if net else None
        for vr in arc.of(fp,'VIA'):
            if not isinstance(vr.outer.get('id'),str) or not vr.outer['id']:
                raise PlacementError('FOOTPRINT_VIA_ID',f'{ref}: VIA ID 缺失')
            v=vr.inner;vid=str(vr.outer['id'])
            if v.get('netName') or v.get('ruleName'):
                raise PlacementError('FOOTPRINT_VIA_NET',f'{ref}: 有网络或盲埋规则的封装VIA尚未适配',Status.UNSUPPORTED_FEATURE)
            vp={'centerX':v.get('centerX'),'centerY':v.get('centerY'),'padAngle':0,
                'defaultPad':{'padType':'CIRCLE','width':v.get('viaDiameter'),'height':v.get('viaDiameter')},
                'hole':v.get('holeDiameter',0)}
            poly,center,source,hm,hp,hc=pad_geometry(vp,scale)
            if vid in identifiers:raise PlacementError('FOOTPRINT_PAD_VIA_ID','PAD/VIA 的稳定身份发生冲突')
            identifiers.add(vid)
            # Native PAD_NET uses a footprint VIA's id for both number and id.
            net=pin_net.get((uid,vid,vid),'')
            pads.append(Pad(vid,vid,poly,center,net,'both',hm,'footprint_via_'+source,hp,hc))
            allpins.setdefault(net,[]).append((ref,vid)) if net else None
        override=overrides.get(ref,overrides.get(fp))
        if override is not None:
            if isinstance(override,list) and len(override)==4 and all(isinstance(v,(int,float)) for v in override):
                body=rect(override)
            else: body=tuple(tuple(x) for x in override)
            body_source='explicit_override_mm'
        elif geometry.get('source_body_policy','explicit_only')=='source_courtyard_or_assembly' and (native_body:=source_body(arc,fp,scale)):
            body,body_source=native_body
        elif ref in approved or fp in approved:
            if not pads: raise PlacementError('MISSING_BODY',f'{ref}: 无 pad，无法批准包络')
            b=union_bbox([bbox(p.polygon) for p in pads]+[bbox(p.hole_polygon) for p in pads if p.hole_polygon])
            m=finite_number(geometry.get('pad_envelope_margin_mm',0),'pad_envelope_margin_mm')
            if m<0:raise PlacementError('PAD_ENVELOPE_MARGIN','包络边距不能为负')
            body=rect((b[0]-m,b[1]-m,b[2]+m,b[3]+m)); body_source='user_approved_pad_envelope'
        else:
            raise PlacementError('BODY_GEOMETRY_REQUIRED',f'{ref}: 提供 geometry.body_overrides 或明确 approved_pad_envelopes',
                                 details={'ref':ref,'footprint':fp})
        validate_convex(body)
        x,y=world(source_position(payload,'x'),source_position(payload,'y'))
        locked=payload.get('locked',False)
        if not isinstance(locked,(bool,int)) or locked not in (False,True,0,1):raise PlacementError('COMPONENT_LOCK','locked 必须为布尔值或0/1')
        components.append(Component(uid,ref,str(fp),Pose(ref,x,y,(-finite_number(payload.get('angle',0),'component angle'))%360,layer),
                                    tuple(body),tuple(pads),bool(locked),body_source))
    known_pins={(c.uuid,p.number,p.id) for c in components for p in c.pads}
    if set(pin_net)-known_pins:
        raise PlacementError('UNKNOWN_PAD_NET','PAD_NET 引用了未解析的器件或焊盘',
                             details={'unknown_pad_net_identities':sorted(set(pin_net)-known_pins)})
    if not components: raise PlacementError('EMPTY_BOARD','所选板无器件')
    version=str(arc.project_metadata.get('editorVersion','unknown'))
    adapter='easyeda_pro_v3' if version.startswith('3.') else 'reference_v3'
    return DesignSnapshot(str(board_id),str(Path(path).resolve()),file_hash(path),adapter,unit,origin,
                          outline,tuple(sorted(components,key=lambda c:c.ref)),
                          tuple(Net(k,tuple(sorted(v))) for k,v in sorted(allpins.items())),
                          (('native_open_validation','PENDING'),('source_editor_version',version),
                           ('atomic_tombstones',str(len(arc.tombstones))),('orphaned_pour_results_preserved',str(orphaned_pours)),
                           ('deleted_documents',str(len(arc.deleted_docs))),
                           ('polygon_path_frame',geometry.get('polygon_path_frame','reject'))))
