"""Compile explicit user rules into one model shared by solver and checker."""
from __future__ import annotations
from dataclasses import dataclass
import copy
import math
import numpy as np
from ..core.schema import PlacementError, Status, finite_number
from ..core.geometry import rect, bbox, union_bbox, angle_equal, signed_distance, transform, circle_envelope


def hole_geometry(pad):
    """Local drill envelope, independent of the copper pad's offset."""
    polygon=getattr(pad,'hole_polygon',())
    if polygon: return polygon
    if pad.hole_mm>0:
        center=getattr(pad,'hole_center',None)
        if center is None: center=pad.center
        return circle_envelope(*center,pad.hole_mm/2)
    return ()


@dataclass(frozen=True)
class Problem:
    design: object
    assignments: object
    cfg: dict
    motions: dict
    angles: dict
    pad_domains: dict
    groups: dict
    ref_group: dict
    rules: dict
    bridges: dict
    fixed: tuple
    special: tuple
    electrical_domains: tuple
    isolation: dict
    obstacles: tuple
    estimated: bool
    reference_length_mm: float

    @property
    def gap(self): return float(self.cfg['rules']['mechanical_gap_mm'])
    @property
    def eps(self): return float(self.cfg['geometry']['epsilon_mm'])
    @property
    def margin(self): return float(self.cfg['geometry']['solve_margin_mm'])
    @property
    def domains(self):
        result=dict(self.assignments.domains)
        for ref,rule in self.rules.items():
            if rule.get('conductive_class') in self.electrical_domains:
                result[ref]=rule['conductive_class']
        return result

    def copper_gap(self,a,b,same_net=False):
        if a!=b and tuple(sorted((a,b))) in self.isolation:
            return self.isolation[tuple(sorted((a,b)))]['copper_clearance_mm']
        return 0. if same_net else self.cfg['rules']['default_pad_clearance_mm']

    def occupancy(self,ref):
        c=self.design.by_ref[ref]
        boxes=[bbox(c.body)]+[bbox(p.polygon) for p in c.pads]
        boxes.extend(bbox(hole_geometry(p)) for p in c.pads if hole_geometry(p))
        rule=self.rules[ref]
        for key in ('keepout_geometry','insertion_keepout'):
            if rule.get(key) is not None: boxes.append(tuple(rule[key]))
        if ref in self.bridges and self.bridges[ref].get('keepout_geometry') is not None:
            boxes.append(tuple(self.bridges[ref]['keepout_geometry']))
        return rect(union_bbox(boxes))


def _vector(value,name):
    if not isinstance(value,(list,tuple)) or len(value)!=2:
        raise PlacementError('RULE_VECTOR',f'{name} 需要二维向量')
    a=np.array([finite_number(v,name) for v in value])
    if np.linalg.norm(a)<1e-12: raise PlacementError('RULE_VECTOR',f'{name} 不得为零向量')
    return a/np.linalg.norm(a)


def compile_constraints(design,assignments,cfg):
    cfg=copy.deepcopy(cfg)
    defs=cfg['rules']['domains']; domains=dict(assignments.domains)
    for k,d in defs.items():
        if d.get('kind') not in ('electrical','mechanical','bridge'):
            raise PlacementError('DOMAIN_KIND',f'{k}: 未知域类型')
    ed=tuple(sorted(k for k,d in defs.items() if d['kind']=='electrical'))
    if not 1<=len(ed)<=2: raise PlacementError('DOMAIN_COUNT','首版支持 1～2 个主电气域',Status.UNSUPPORTED_FEATURE)
    isolation={}
    for p in cfg['rules']['isolation_pairs']:
        ds=p.get('domains',[])
        if len(ds)!=2 or len(set(ds))!=2 or not set(ds)<=set(ed):
            raise PlacementError('ISOLATION_PAIR','隔离对必须是两个已声明电气域')
        key=tuple(sorted(ds))
        if key in isolation: raise PlacementError('ISOLATION_PAIR','隔离对重复')
        for field in ['copper_clearance_mm','creepage_mm','routing_reserve_mm']:
            if finite_number(p.get(field),field)<0: raise PlacementError('RULE_RANGE',field)
        isolation[key]=p
    if len(ed)==2 and tuple(ed) not in isolation:
        raise PlacementError('ISOLATION_RULE_REQUIRED','两个主域之间必须提供隔离规则')
    overrides=cfg['rules']['component_rules']
    unknown=set(overrides)-set(design.by_ref)
    if unknown: raise PlacementError('UNKNOWN_COMPONENT_RULE',f'未知器件规则 {sorted(unknown)}')
    unlock=set(cfg['rules']['unlock_source_locked'])
    if unlock-set(design.by_ref): raise PlacementError('UNKNOWN_UNLOCK','解锁列表包含未知器件')
    motions={}; angles={}; pads={}; rules={}; bridges={}; fixed=[]; special=[]
    for c in design.components:
        r=copy.deepcopy(overrides.get(c.ref,{}))
        mode=r.get('mode','fixed_pose' if c.locked and c.ref not in unlock else 'free')
        if mode not in ('fixed_pose','free','edge_slide','edge_choice','edge_band','corner','region_bounded'):
            raise PlacementError('MOTION_MODE',f'{c.ref}: {mode}')
        if c.locked and c.ref not in unlock and mode!='fixed_pose':
            raise PlacementError('LOCK_CONFLICT',f'{c.ref} 源锁定；显式加入 unlock_source_locked 才可解除',Status.CONSTRAINT_CONFLICT)
        if mode=='fixed_pose' and r.get('pose','source')!='source':
            raise PlacementError('FIXED_POSE','fixed_pose 首版使用源位姿；修改机械坐标须在源工程中明确设置')
        aa=r.get('allowed_angles',[c.source_pose.angle])
        if not isinstance(aa,list) or not aa: raise PlacementError('ANGLES',f'{c.ref}: 允许角度不能为空')
        aa=tuple(sorted(set(finite_number(v,c.ref+'.allowed_angles')%360 for v in aa)))
        if mode=='fixed_pose' and not any(angle_equal(c.source_pose.angle,x) for x in aa):
            raise PlacementError('FIXED_ANGLE',f'{c.ref} 固定角度不在允许集合',Status.CONSTRAINT_CONFLICT)
        if mode=='corner':
            corners=r.get('allowed_corners')
            if not isinstance(corners,list) or not corners or not set(corners)<={'bottom_left','bottom_right','top_left','top_right'}:
                raise PlacementError('CORNER_RULE',f'{c.ref}: 需要有效 allowed_corners')
            if finite_number(r.get('corner_inset_mm'),c.ref+'.corner_inset_mm')<0:
                raise PlacementError('CORNER_RULE',f'{c.ref}: corner_inset_mm 不得为负')
        if mode=='edge_band':
            edges=r.get('allowed_edges')
            if not isinstance(edges,list) or not edges or not set(edges)<={'left','right','bottom','top'}:
                raise PlacementError('EDGE_RULE',f'{c.ref}: 需要有效 allowed_edges')
            if finite_number(r.get('edge_offset_mm'),c.ref+'.edge_offset_mm')<0:
                raise PlacementError('EDGE_RULE',f'{c.ref}: edge_offset_mm 不得为负')
            if not any(angle_equal(c.source_pose.angle,a) for a in aa):
                raise PlacementError('EDGE_ANGLE',f'{c.ref}: edge_band 必须保留源角度',Status.CONSTRAINT_CONFLICT)
            aa=(c.source_pose.angle%360,)
            if r.get('allow_body_overhang',False):
                raise PlacementError('EDGE_RULE',f'{c.ref}: edge_band 完整占位必须在板内')
        if mode!='edge_band' and (mode.startswith('edge') or 'allowed_edges' in r):
            for key in ['allowed_edges','edge_segment_mm','local_mating_point_mm','local_outward_vector','edge_offset_mm','insertion_keepout']:
                if r.get(key) is None: raise PlacementError('EDGE_RULE_REQUIRED',f'{c.ref}.{key} 必须提供')
            if not set(r['allowed_edges'])<=set(['left','right','bottom','top']) or not r['allowed_edges']:
                raise PlacementError('EDGE_RULE',f'{c.ref}: 板边无效')
            _vector(r['local_outward_vector'],c.ref+'.outward')
            if len(r['local_mating_point_mm'])!=2 or len(r['edge_segment_mm'])!=2:
                raise PlacementError('EDGE_RULE',f'{c.ref}: 基准点/边段无效')
            if r['edge_segment_mm'][1]<r['edge_segment_mm'][0]: raise PlacementError('EDGE_RULE','边段区间反向')
            rect(r['insertion_keepout'])
        kind=defs[domains[c.ref]]['kind']
        if kind=='bridge':
            name=r.get('bridge_template')
            template=cfg['rules']['bridge_templates'].get(name)
            if not template: raise PlacementError('BRIDGE_TEMPLATE_REQUIRED',f'{c.ref}: 需要 bridge_template')
            pair=tuple(template.get('between',[]))
            if len(pair)!=2 or set(pair)!=set(ed) or set(defs[domains[c.ref]].get('between',[]))!=set(pair):
                raise PlacementError('BRIDGE_TEMPLATE',f'{c.ref}: 模板域与桥接标签不一致')
            _vector(template.get('axis_local'),c.ref+'.axis_local')
            if template.get('barrier_point_mm') is None: raise PlacementError('BRIDGE_TEMPLATE','需要 barrier_point_mm')
            if len(template['barrier_point_mm'])!=2:
                raise PlacementError('BRIDGE_TEMPLATE','barrier_point_mm 需要二维局部坐标')
            for value in template['barrier_point_mm']: finite_number(value,'barrier_point_mm')
            if template.get('keepout_geometry') is not None: rect(template['keepout_geometry'])
            if set(template.get('pad_domains',{}))!={p.number for p in c.pads}:
                raise PlacementError('BRIDGE_PAD_DOMAINS',f'{c.ref}: 模板必须覆盖所有真实焊盘号')
            if set(template['pad_domains'].values())!=set(ed): raise PlacementError('BRIDGE_PAD_DOMAINS','桥接两侧域不完整')
            if mode=='fixed_pose':
                fixed.append(c.ref)
            else: special.append(c.ref)
            if template.get('allowed_angles'):
                aa=tuple(x for x in aa if any(angle_equal(x,v) for v in template['allowed_angles']))
                if not aa: raise PlacementError('BRIDGE_ANGLE','桥接模板与器件允许角度无交集')
            bridges[c.ref]=copy.deepcopy(template)
            pds=template['pad_domains']
        else:
            pds=r.get('pad_domains',{})
            if set(pds)-{p.number for p in c.pads}: raise PlacementError('PAD_DOMAIN','未知焊盘编号')
            if kind=='mechanical':
                cls=r.get('conductive_class')
                if cls not in ('insulating',*ed):
                    raise PlacementError('MECHANICAL_CONDUCTIVITY_REQUIRED',f'{c.ref}: conductive_class 为 insulating 或电气域 ID')
                if c.pads and cls=='insulating': raise PlacementError('MECHANICAL_CONDUCTIVITY','有铜 pad 的机械件不能标 insulating')
                if mode not in ('fixed_pose','corner'):
                    raise PlacementError('MECHANICAL_TEMPLATE_REQUIRED','机械孔须固定或使用明确的 corner 机械模板',Status.UNSUPPORTED_FEATURE)
            if mode=='fixed_pose': fixed.append(c.ref)
            elif mode.startswith('edge') or mode=='corner': special.append(c.ref)
        for p in c.pads:
            d=pds.get(p.number,domains[c.ref])
            if kind=='mechanical': d=r['conductive_class']
            if d not in ed: raise PlacementError('PAD_DOMAIN',f'{c.ref}.{p.number} 没有电气域')
            # The effective pad assignment, rather than the CSV label alone,
            # determines whether a component crosses an isolation interface.
            if kind=='electrical' and d!=domains[c.ref]:
                raise PlacementError('BRIDGE_TEMPLATE_REQUIRED',
                    f'{c.ref}.{p.number} 跨出器件所属电气域；须声明桥接域及完整桥接模板',
                    details={'component':c.ref,'component_domain':domains[c.ref],
                             'pad_number':p.number,'pad_domain':d})
            pads[(c.ref,p.id)]=d
        if mode=='region_bounded' and kind!='bridge' and r.get('region_id') not in cfg['rules']['regions']:
            raise PlacementError('REGION_REQUIRED',f'{c.ref}: 未声明 region_id')
        for key in ('keepout_geometry','insertion_keepout'):
            if r.get(key) is not None: rect(r[key])
        rules[c.ref]=r; motions[c.ref]=mode; angles[c.ref]=aa
    for net in design.nets:
        ds={pads[k] for k in net.pins}
        if len(ds)>1: raise PlacementError('NET_CROSSES_ISOLATION',f'网络 {net.id} 跨互相隔离的域: {sorted(ds)}')
    for k,box in cfg['rules']['regions'].items(): rect(box)
    obs=[]
    for o in cfg['rules']['obstacles']:
        if not o.get('id') or o.get('bbox_mm') is None: raise PlacementError('OBSTACLE','障碍需要 id 和 bbox_mm')
        rect(o['bbox_mm'])
        layers=o.get('layers',['top','bottom'])
        if not set(layers)<=set(['top','bottom']): raise PlacementError('OBSTACLE_LAYER','障碍适用层无效')
        obs.append(copy.deepcopy(o))
    if len({o['id'] for o in obs})!=len(obs): raise PlacementError('OBSTACLE_ID','障碍 ID 重复')
    def endpoint(value):
        if not isinstance(value,str) or ':' not in value: raise PlacementError('PIN_REFERENCE','引脚格式 ref:pad_number')
        ref,num=value.split(':',1)
        matches=[p for p in design.by_ref[ref].pads if p.number==num] if ref in design.by_ref else []
        if not matches:
            raise PlacementError('PIN_REFERENCE',f'未知引脚 {value}')
        if len(matches)!=1:
            raise PlacementError('PIN_REFERENCE_AMBIGUOUS',
                f'{value} 对应多个物理焊盘，距离/顺序规则不能静默选择其中一个',
                details={'endpoint':value,'pad_ids':sorted(p.id for p in matches)})
    for r in cfg['rules']['distance_constraints']:
        endpoint(r.get('a')); endpoint(r.get('b'))
        key='max_mm' if r.get('hard',True) else 'target_mm'
        if finite_number(r.get(key),key)<0 or finite_number(r.get('weight',1),'weight')<0:
            raise PlacementError('RULE_RANGE','距离和权重不得为负')
    for r in cfg['rules']['order_constraints']:
        endpoint(r.get('a')); endpoint(r.get('b'))
        if r.get('axis') not in ('x','y') or not r.get('hard',True):
            raise PlacementError('ORDER_RULE','首版顺序规则仅支持 x/y 硬关系')
        finite_number(r.get('min_separation_mm',0),'min_separation_mm')
    for k,w in cfg['rules']['net_weights'].items():
        if k not in {n.id for n in design.nets} or finite_number(w,'net_weight')<0:
            raise PlacementError('NET_WEIGHT',f'网络权重无效 {k}')
    groups={}; rg={}
    for ref,cid in assignments.clusters:
        if ref in fixed or ref in special: continue
        groups.setdefault(cid,[]).append(ref); rg[ref]=cid
    mode=cfg['board']['mode']; estimated=mode=='estimated' or mode=='auto' and not design.outline_present
    if mode=='fixed' and not design.outline_present: raise PlacementError('OUTLINE_REQUIRED','fixed 需要有效源板框')
    if mode=='estimated' and design.outline_present and not cfg['board']['replace_existing_outline']:
        raise PlacementError('OUTLINE_REPLACEMENT_REQUIRED','已有板框需显式 replace_existing_outline=true')
    if estimated:
        for k in ('max_width_mm','max_height_mm'):
            if cfg['board']['estimated'][k] is None: raise PlacementError('DECISIONS_REQUIRED',f'估算模式需要 board.estimated.{k}')
    if design.outline:
        b=bbox(design.outline); reference_length=math.hypot(b[2]-b[0],b[3]-b[1])
    else:
        occupied=sum((bbox(c.body)[2]-bbox(c.body)[0])*(bbox(c.body)[3]-bbox(c.body)[1]) for c in design.components)
        reference_length=math.sqrt(2*occupied/cfg['board']['estimated']['target_utilization'])
    problem=Problem(design,assignments,cfg,motions,angles,pads,
                    {k:tuple(sorted(v)) for k,v in groups.items()},rg,rules,bridges,
                    tuple(sorted(fixed)),tuple(sorted(special)),ed,isolation,tuple(obs),estimated,max(reference_length,1.))
    # Known fixed-fixed mechanical conflicts are never excluded from the input checks.
    for i,ref in enumerate(problem.fixed):
        c=design.by_ref[ref]; a=transform(c.body,c.source_pose)
        for other in problem.fixed[i+1:]:
            d=design.by_ref[other]
            if signed_distance(a,transform(d.body,d.source_pose))<problem.gap-problem.eps:
                raise PlacementError('FIXED_COLLISION',f'固定件 {ref}/{other} 冲突',Status.CONSTRAINT_CONFLICT)
    return problem
