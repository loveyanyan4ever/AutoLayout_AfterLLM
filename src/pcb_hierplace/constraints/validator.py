"""Independent geometric validation, including immutable and electrical intent checks."""
from __future__ import annotations
import math
import numpy as np
from ..core.schema import Violation, ValidationReport,Pad
from ..core.geometry import (bbox,rect,area,transform,rotation,signed_distance,inside_box,
                             pad_world,pad_center,angle_equal,is_rectangle)
from .compiler import hole_geometry

NORMALS={'left':np.array([-1.,0.]),'right':np.array([1.,0.]),
         'bottom':np.array([0.,-1.]),'top':np.array([0.,1.])}


def edge_matches(rule,pose,board,eps):
    point=rotation(pose.angle)@np.asarray(rule['local_mating_point_mm'])+[pose.x,pose.y]
    vec=rotation(pose.angle)@np.asarray(rule['local_outward_vector'])
    vec=vec/np.linalg.norm(vec)
    for edge in rule['allowed_edges']:
        axis=0 if edge in ('left','right') else 1
        coord={'left':board[0],'right':board[2],'bottom':board[1],'top':board[3]}[edge]
        offset=float(rule['edge_offset_mm'])*NORMALS[edge][axis]
        lo,hi=rule['edge_segment_mm']
        # edge segment coordinates are offsets measured from bottom/left of board.
        along=point[1-axis]-board[1-axis]
        if abs(point[axis]-(coord+offset))<=eps and lo-eps<=along<=hi+eps and np.dot(vec,NORMALS[edge])>=1-1e-7:
            return True
    return False


def corner_matches(rule,pose,board,occupancy,eps):
    """Match a board corner using the rotated complete local occupancy."""
    box=bbox(transform(occupancy,pose)); inset=float(rule['corner_inset_mm'])
    if not inside_box(rect(box),board,eps): return False
    for corner in rule['allowed_corners']:
        x=(box[0]-board[0]) if corner.endswith('left') else (board[2]-box[2])
        y=(box[1]-board[1]) if corner.startswith('bottom') else (board[3]-box[3])
        if abs(x-inset)<=eps and abs(y-inset)<=eps: return True
    return False


def edge_band_matches(rule,pose,board,occupancy,eps):
    """Match an inset board edge without inferring a mating direction."""
    box=bbox(transform(occupancy,pose)); offset=float(rule['edge_offset_mm'])
    if not inside_box(rect(box),board,eps): return False
    distances={'left':box[0]-board[0],'right':board[2]-box[2],
               'bottom':box[1]-board[1],'top':board[3]-box[3]}
    return any(abs(distances[edge]-offset)<=eps for edge in rule['allowed_edges'])


def pin(problem,state,endpoint):
    ref,num=endpoint.split(':',1)
    c=problem.design.by_ref[ref]
    pad=next(p for p in c.pads if p.number==num)
    return pad_center(pad,state.by_ref[ref])


def _bbox_separation(a,b):
    """Lower bound on polygon separation; zero never rules out penetration."""
    return math.hypot(max(a[0]-b[2],b[0]-a[2],0.),
                      max(a[1]-b[3],b[1]-a[3],0.))


def validate_placement(problem,state,require_regions=True,reference=None):
    v=[]; refs=problem.design.by_ref; poses=state.by_ref; eps=problem.eps
    def bad(rule,objects,message,actual=None,required=None):
        v.append(Violation(rule,tuple(objects),message,actual,required))
    if len(poses)!=len(state.poses) or set(poses)!=set(refs):
        bad('OBJECT_SET',set(poses)^set(refs),'器件缺失/重复/多余')
        return ValidationReport(tuple(v))
    if any(not all(math.isfinite(x) for x in (p.x,p.y,p.angle)) for p in state.poses):
        bad('FINITE',[],'坐标/角度非有限'); return ValidationReport(tuple(v))
    if not state.outline or not is_rectangle(state.outline):
        bad('OUTLINE',[],'需要有效矩形板框'); return ValidationReport(tuple(v))
    board=bbox(state.outline)
    if not problem.estimated and state.outline!=problem.design.outline:
        bad('FIXED_OUTLINE',[],'固定板框几何被改变')
    regions=dict(state.regions); domain_regions=dict(state.domain_regions)
    if require_regions:
        if set(domain_regions)!=set(problem.electrical_domains):
            bad('DOMAIN_SET',[],'隔离域区域集合不完整或含未知域')
        for domain,box in domain_regions.items():
            if not inside_box(rect(box),board,eps): bad('DOMAIN_BOARD',[domain],'域区域超出板框')
        for pair,rule in problem.isolation.items():
            if all(d in domain_regions for d in pair):
                distance=signed_distance(rect(domain_regions[pair[0]]),rect(domain_regions[pair[1]]))
                required=rule['copper_clearance_mm']+rule['routing_reserve_mm']
                if distance<required-eps: bad('ISOLATION_RESERVE',pair,'隔离区域间预留不足',distance,required)
        for cid,box in regions.items():
            if cid not in problem.groups: bad('CLUSTER_SET',[cid],'未知簇区域')
            elif not inside_box(rect(box),domain_regions.get(problem.domains[problem.groups[cid][0]],board),eps):
                bad('CLUSTER_DOMAIN',[cid],'簇区域越出隔离域')
        for i,(cid,box) in enumerate(sorted(regions.items())):
            for other,other_box in sorted(regions.items())[i+1:]:
                if signed_distance(rect(box),rect(other_box)) < -eps:
                    bad('CLUSTER_REGION_OVERLAP',[cid,other],'硬簇区域相交')
    bodies={r:transform(refs[r].body,p) for r,p in poses.items()}
    body_bounds={r:bbox(poly) for r,poly in bodies.items()}
    pad_polys={r:[(pad,pad_world(pad,poses[r])) for pad in c.pads] for r,c in refs.items()}
    holes={r:[(pad,transform(hole_geometry(pad),poses[r])) for pad in c.pads if hole_geometry(pad)]
           for r,c in refs.items()}
    copper=[]
    for ref,c in refs.items():
        p=poses[ref]; r=problem.rules[ref]; motion=problem.motions[ref]
        if p.layer!=c.source_pose.layer: bad('LAYER',[ref],'安装面改变')
        if not any(angle_equal(p.angle,a) for a in problem.angles[ref]): bad('ANGLE',[ref],'方向不在允许集合')
        if motion=='fixed_pose' and (abs(p.x-c.source_pose.x)>eps or abs(p.y-c.source_pose.y)>eps or not angle_equal(p.angle,c.source_pose.angle)):
            bad('FIXED_POSE',[ref],'固定位置或角度改变')
        has_edge=motion!='edge_band' and (motion.startswith('edge') or 'allowed_edges' in r)
        overhang=has_edge and r.get('allow_body_overhang',False)
        if not overhang and not inside_box(bodies[ref],board,eps): bad('BOARD_BODY',[ref],'本体出板')
        if has_edge and not edge_matches(r,p,board,eps): bad('EDGE',[ref],'接口基准/方向/边段不满足')
        if motion=='corner' and not corner_matches(r,p,board,problem.occupancy(ref),eps):
            bad('CORNER',[ref],'完整占位不满足所选板角及内缩距离')
        if motion=='edge_band':
            if not angle_equal(p.angle,c.source_pose.angle): bad('EDGE_BAND_ANGLE',[ref],'沿边预布局改变源角度')
            if not edge_band_matches(r,p,board,problem.occupancy(ref),eps):
                bad('EDGE_BAND',[ref],'完整占位不满足板边内缩距离')
        if motion=='region_bounded' and ref not in problem.bridges:
            box=problem.cfg['rules']['regions'][r['region_id']]
            if not inside_box(transform(problem.occupancy(ref),p),box,eps): bad('REGION',[ref],'完整占位不在指定区域')
        domain=problem.domains[ref]
        if domain in problem.electrical_domains:
            if domain not in domain_regions:
                if require_regions: bad('DOMAIN_REGION',[ref],'缺少域区域')
            else:
                body=bodies[ref]
                if overhang:
                    bb=bbox(body); clipped=(max(bb[0],board[0]),max(bb[1],board[1]),min(bb[2],board[2]),min(bb[3],board[3]))
                    body=rect(clipped) if clipped[0]<clipped[2] and clipped[1]<clipped[3] else None
                if body is not None and not inside_box(body,domain_regions[domain],eps):
                    bad('DOMAIN_REGION',[ref],'本体在板内的部分越出域区域')
        cid=problem.ref_group.get(ref)
        if cid:
            if cid not in regions:
                if require_regions: bad('CLUSTER_REGION',[ref],'缺少硬簇区域')
            elif not inside_box(transform(problem.occupancy(ref),p),regions[cid],eps): bad('CLUSTER_REGION',[ref],'完整占位超出簇区域')
        for pad,poly in pad_polys[ref]:
            pd=problem.pad_domains[(ref,pad.id)]
            if not inside_box(poly,board,eps): bad('BOARD_PAD',[ref,pad.number],'焊盘出板')
            if pd in domain_regions and not inside_box(poly,domain_regions[pd],eps): bad('PAD_DOMAIN',[ref,pad.number],'焊盘不在所属域侧')
            copper.append((ref,pad,poly,pd))
        for pad,poly in holes[ref]:
            if not inside_box(poly,board,eps): bad('BOARD_HOLE',[ref,pad.number],'钻孔出板')
            pd=problem.pad_domains[(ref,pad.id)]
            if pd in domain_regions and not inside_box(poly,domain_regions[pd],eps):
                bad('HOLE_DOMAIN',[ref,pad.number],'钻孔不在所属域侧')
        # A metal body remains conductive when a footprint also contains pads.
        # Its contact net is unspecified, so no same-net exemption is inferred.
        if r.get('conductive_class') in problem.electrical_domains:
            pad=Pad('__body_conductor__','metal_body',c.body,(0.,0.))
            copper.append((ref,pad,bodies[ref],r['conductive_class']))
        for o in problem.obstacles:
            if p.layer in o.get('layers',['top','bottom']):
                dist=min(signed_distance(poly,rect(o['bbox_mm'])) for poly in
                         [bodies[ref]]+[poly for _,poly in pad_polys[ref]+holes[ref]])
                if dist<problem.gap-eps: bad('OBSTACLE',[ref,o['id']],'侵入固定障碍/间距',dist,problem.gap)
        if ref in problem.bridges and domain_regions:
            t=problem.bridges[ref]; a,b=t['between']
            if a in domain_regions and b in domain_regions:
                ba,bb=domain_regions[a],domain_regions[b]
                n=np.array([(bb[0]+bb[2]-ba[0]-ba[2])/2,(bb[1]+bb[3]-ba[1]-ba[3])/2]); n/=np.linalg.norm(n)
                axis=rotation(p.angle)@np.asarray(t['axis_local'],float); axis/=np.linalg.norm(axis)
                if np.dot(axis,n)<1-1e-7: bad('BRIDGE_AXIS',[ref],'桥接隔离轴方向错误')
                k=int(np.argmax(abs(n)))
                boundary=(ba[k+2]+bb[k])/2 if n[k]>0 else (bb[k+2]+ba[k])/2
                barrier=rotation(p.angle)@np.asarray(t['barrier_point_mm'])+[p.x,p.y]
                if abs(barrier[k]-boundary)>eps: bad('BRIDGE_BARRIER',[ref],'桥接隔离基准偏离隔离带中线')
    sorted_refs=sorted(refs)
    for i,ref in enumerate(sorted_refs):
        for other in sorted_refs[i+1:]:
            lower=_bbox_separation(body_bounds[ref],body_bounds[other])
            if lower>0 and lower>=problem.gap-eps: continue
            dist=signed_distance(bodies[ref],bodies[other])
            if dist<problem.gap-eps: bad('MECHANICAL_GAP',[ref,other],'本体重叠或机械间距不足',dist,problem.gap)
    for ref,pad_holes in holes.items():
        for pad,poly in pad_holes:
            hb=bbox(poly)
            for other in refs:
                if other==ref: continue  # A drill is allowed within its own footprint.
                occupied=[bodies[other]]+[q for _,q in pad_polys[other]+holes[other]]
                for q in occupied:
                    lower=_bbox_separation(hb,bbox(q))
                    if lower>0 and lower>=problem.gap-eps: continue
                    distance=signed_distance(poly,q)
                    if distance<problem.gap-eps:
                        bad('HOLE_KEEPOUT',[ref+':'+pad.number,other],'钻孔与其他器件占位间距不足',distance,problem.gap)
                        break
    # Directional keepouts also exclude copper that extends beyond a component body.
    for ref in refs:
        keepouts=[problem.rules[ref].get(k) for k in ('keepout_geometry','insertion_keepout')]
        if ref in problem.bridges: keepouts.append(problem.bridges[ref].get('keepout_geometry'))
        for box in filter(lambda x:x is not None,keepouts):
            poly=transform(rect(box),poses[ref])
            for other in refs:
                occupied=[bodies[other]]+[poly for _,poly in pad_polys[other]+holes[other]]
                if other!=ref and min(signed_distance(poly,q) for q in occupied) < -eps:
                    bad('KEEPOUT',[ref,other],'侵入器件局部禁区')
            for obstacle in problem.obstacles:
                if poses[ref].layer in obstacle.get('layers',['top','bottom']) and signed_distance(poly,rect(obstacle['bbox_mm'])) < -eps:
                    bad('KEEPOUT_OBSTACLE',[ref,obstacle['id']],'器件局部禁区与固定障碍相交')
    minimum=math.inf
    copper_bounds=[bbox(poly) for _,_,poly,_ in copper]
    for i,(r,p,poly,d) in enumerate(copper):
        for j in range(i+1,len(copper)):
            s,q,other,e=copper[j]
            same_net=bool(p.net) and p.net==q.net
            cross=d!=e
            if r==s and not cross: continue  # footprint-internal ordinary pad rules belong to library validation
            if same_net and not cross: continue
            gap=problem.copper_gap(d,e,same_net)
            lower=_bbox_separation(copper_bounds[i],copper_bounds[j])
            # Keep exact cross-domain minimum reporting: skip only when the
            # pair can neither violate its rule nor improve the minimum seen so far.
            if lower>0 and lower>=gap-eps and (not cross or lower>=minimum): continue
            dist=signed_distance(poly,other)
            if cross: minimum=min(minimum,dist)
            if dist<gap-eps: bad('COPPER_CLEARANCE',[r+':'+p.number,s+':'+q.number],'导体间距不足',dist,gap)
    for r in problem.cfg['rules']['distance_constraints']:
        distance=float(np.linalg.norm(pin(problem,state,r['a'])-pin(problem,state,r['b'])))
        if r.get('hard',True) and distance>r['max_mm']+eps:
            bad(r.get('id','PIN_DISTANCE'),[r['a'],r['b']],'引脚距离超过上限',distance,r['max_mm'])
    for r in problem.cfg['rules']['order_constraints']:
        k=0 if r['axis']=='x' else 1
        delta=float(pin(problem,state,r['b'])[k]-pin(problem,state,r['a'])[k])
        if delta<r.get('min_separation_mm',0)-eps: bad(r.get('id','PIN_ORDER'),[r['a'],r['b']],'引脚顺序违反',delta,r.get('min_separation_mm',0))
    if reference is not None:
        def same_regions(actual,expected):
            aa,bb=dict(actual),dict(expected)
            return (len(aa)==len(actual) and len(bb)==len(expected) and set(aa)==set(bb)
                    and all(len(aa[k])==len(bb[k]) and
                            all(abs(x-y)<=eps for x,y in zip(aa[k],bb[k])) for k in aa))
        if bbox(state.outline)!=bbox(reference.outline):
            bad('REFINE_OUTLINE',[],'整体微调改变已冻结板框')
        if not same_regions(state.domain_regions,reference.domain_regions):
            bad('REFINE_DOMAIN_REGIONS',[],'整体微调改变已冻结隔离域区域')
        if not same_regions(state.regions,reference.regions):
            bad('REFINE_CLUSTER_REGIONS',[],'整体微调改变已冻结硬簇区域')
        rr=reference.by_ref; trust=problem.cfg['optimization']['stage_c']['trust_radius_mm']
        if len(rr)!=len(reference.poses) or set(rr)!=set(poses):
            bad('REFERENCE_OBJECT_SET',[],'微调参考对象集合不完整或重复')
            return ValidationReport(tuple(v),None if math.isinf(minimum) else float(minimum))
        frozen=set(problem.fixed)|set(problem.special)
        for ref,p in poses.items():
            if np.linalg.norm([p.x-rr[ref].x,p.y-rr[ref].y])>trust+eps: bad('TRUST_RADIUS',[ref],'整体微调位移超限')
            if not angle_equal(p.angle,rr[ref].angle): bad('REFINE_ANGLE',[ref],'微调改变方向')
            if ref in frozen and (abs(p.x-rr[ref].x)>eps or abs(p.y-rr[ref].y)>eps
                                  or not angle_equal(p.angle,rr[ref].angle) or p.layer!=rr[ref].layer):
                bad('REFINE_FIXED_OBJECT',[ref],'整体微调改变已冻结固定/特殊器件')
        for cid,members in problem.groups.items():
            def extent(st):
                pp=np.concatenate([transform(problem.occupancy(r),st.by_ref[r]) for r in members])
                b=bbox(pp); return (b[2]-b[0])*(b[3]-b[1])
            limit=extent(reference)*(1+problem.cfg['optimization']['stage_c']['cluster_area_growth_ratio'])
            if extent(state)>limit+eps: bad('CLUSTER_GROWTH',members,'簇面积增长超限',extent(state),limit)
    return ValidationReport(tuple(v),None if math.isinf(minimum) else float(minimum))
