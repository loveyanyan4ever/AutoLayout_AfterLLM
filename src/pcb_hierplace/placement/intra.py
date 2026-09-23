"""Stage A/A-prime: legal, non-stretchable shape candidates with real terminals."""
from __future__ import annotations
from dataclasses import dataclass,asdict,replace
import hashlib
import math
import numpy as np
from ..core.schema import Pose,PlacementState,digest
from ..core.geometry import bbox,union_bbox,transform,rect,signed_distance,pad_world,pad_center,angle_equal
from ..opt.projected import optimize
from ..opt.objectives import exact_hpwl
from ..constraints.compiler import hole_geometry
from .common import Block,translation_bounds,seed_pack,constraint_projector,objective_for,expand


@dataclass(frozen=True)
class ShapeCandidate:
    cluster_id: str
    id: str
    members: tuple[Pose,...]
    box: tuple
    internal_hpwl: float
    ports: tuple
    seed: int
    stats: dict

    @property
    def area(self): return (self.box[2]-self.box[0])*(self.box[3]-self.box[1])


def stable_seed(seed,*parts):
    return int(hashlib.sha256(repr((seed,parts)).encode()).hexdigest()[:8],16)


def local_valid(problem,poses):
    refs=problem.design.by_ref; eps=problem.eps; gap=problem.gap
    byref={p.ref:p for p in poses}
    if len(byref)!=len(poses) or not set(byref)<=set(refs): return False
    # Geometry is transformed exactly once per pose.  Each cached shape keeps
    # its world AABB; a positive AABB separation is a lower bound on the true
    # polygon distance, so only nearby pairs need the exact convex routine.
    def shape(poly): return (poly,bbox(poly))
    def separation(a,b):
        return math.hypot(max(a[0]-b[2],b[0]-a[2],0.),max(a[1]-b[3],b[1]-a[3],0.))
    def violates(a,b,required):
        lower=separation(a[1],b[1])
        if lower>0 and lower>=required-eps: return False
        return signed_distance(a[0],b[0])<required-eps
    bodies={}; pads={}; holes={}; keepouts={}; copper={}; occupied={}; bounds={}
    for p in poses:
        a=refs[p.ref]
        if not all(math.isfinite(x) for x in (p.x,p.y,p.angle)): return False
        if p.layer!=a.source_pose.layer or not any(angle_equal(p.angle,x) for x in problem.angles[p.ref]):
            return False
        bodies[p.ref]=shape(transform(a.body,p)); pads[p.ref]=[]; holes[p.ref]=[]; copper[p.ref]=[]
        for pad in a.pads:
            world=shape(pad_world(pad,p)); pads[p.ref].append(world)
            copper[p.ref].append((world,pad.net,problem.pad_domains[(p.ref,pad.id)]))
            hole=hole_geometry(pad)
            if hole: holes[p.ref].append(shape(transform(hole,p)))
        rule=problem.rules[p.ref]; conductor=rule.get('conductive_class')
        if conductor in problem.electrical_domains:
            copper[p.ref].append((bodies[p.ref],None,conductor))
        boxes=[rule.get(k) for k in ('keepout_geometry','insertion_keepout')]
        if p.ref in problem.bridges: boxes.append(problem.bridges[p.ref].get('keepout_geometry'))
        keepouts[p.ref]=[shape(transform(rect(box),p)) for box in boxes if box is not None]
        occupied[p.ref]=[bodies[p.ref]]+pads[p.ref]+holes[p.ref]
        bounds[p.ref]=union_bbox([entry[1] for entry in occupied[p.ref]+keepouts[p.ref]])
    max_gap=max([gap,problem.cfg['rules']['default_pad_clearance_mm']]+
                [rule['copper_clearance_mm'] for rule in problem.isolation.values()])
    def copper_invalid(left,right,same_component=False):
        for i,(a,net_a,domain_a) in enumerate(left):
            for b,net_b,domain_b in (right[i+1:] if same_component else right):
                cross=domain_a!=domain_b
                if same_component and not cross: continue
                same_net=bool(net_a) and net_a==net_b
                if same_net and not cross: continue
                if violates(a,b,problem.copper_gap(domain_a,domain_b,same_net)): return True
        return False
    for i,p in enumerate(poses):
        ref=p.ref
        # Bridge pads on different domains still require internal isolation.
        if copper_invalid(copper[ref],copper[ref],same_component=True): return False
        for q in poses[i+1:]:
            other=q.ref
            lower=separation(bounds[ref],bounds[other])
            if lower>0 and lower>=max_gap-eps: continue
            if violates(bodies[ref],bodies[other],gap): return False
            if any(violates(hole,geometry,gap) for hole in holes[ref] for geometry in occupied[other]): return False
            # Hole/hole was already checked in the first direction.
            if any(violates(hole,geometry,gap) for hole in holes[other]
                   for geometry in [bodies[ref]]+pads[ref]): return False
            if any(violates(zone,geometry,0.) for zone in keepouts[ref] for geometry in occupied[other]): return False
            if any(violates(zone,geometry,0.) for zone in keepouts[other] for geometry in occupied[ref]): return False
            if copper_invalid(copper[ref],copper[other]): return False
    for r in problem.cfg['rules']['distance_constraints']:
        if not r.get('hard',True): continue
        endpoints=[]
        for key in ('a','b'):
            ref,num=r[key].split(':',1)
            if ref not in byref: break
            pad=next(p for p in refs[ref].pads if p.number==num)
            endpoints.append(pad_center(pad,byref[ref]))
        if len(endpoints)==2 and np.linalg.norm(endpoints[0]-endpoints[1])>r['max_mm']+problem.eps: return False
    return True


def make_shape(problem,cid,poses,seed,stats):
    bb=union_bbox([bbox(transform(problem.occupancy(p.ref),p)) for p in poses])
    cx,cy=(bb[0]+bb[2])/2,(bb[1]+bb[3])/2
    members=tuple(Pose(p.ref,p.x-cx,p.y-cy,p.angle,p.layer) for p in poses)
    byref={p.ref:p for p in members}; ports=[]; internal=0.
    refs=problem.design.by_ref
    for net in problem.design.nets:
        local=[]
        for ref,pid in net.pins:
            if ref in byref:
                pad=next(p for p in refs[ref].pads if p.id==pid)
                xy=pad_center(pad,byref[ref]); local.append(xy)
                if any(r not in byref for r,_ in net.pins): ports.append((net.id,ref,pid,float(xy[0]),float(xy[1])))
        if len(local)>=2:
            q=np.asarray(local); internal+=float(np.ptp(q[:,0])+np.ptp(q[:,1]))
    box=(bb[0]-cx,bb[1]-cy,bb[2]-cx,bb[3]-cy)
    ident=digest({'cid':cid,'members':[asdict(p) for p in members],'box':box,'ports':ports})[:16]
    return ShapeCandidate(cid,ident,members,box,internal,tuple(ports),seed,stats)


def select_shapes(candidates,max_keep):
    # Keep port-distinct alternatives; Pareto filter only within equivalent port arrangements.
    unique={c.id:c for c in candidates}
    cands=list(unique.values()); front=[]
    def signature(c):
        return tuple((p[0],p[1],round(p[3],2),round(p[4],2)) for p in c.ports)
    for c in cands:
        if not any(signature(o)==signature(c) and o.area<=c.area and o.internal_hpwl<=c.internal_hpwl
                   and (o.area<c.area-1e-9 or o.internal_hpwl<c.internal_hpwl-1e-9) for o in cands if o.id!=c.id):
            front.append(c)
    front.sort(key=lambda c:(c.area,c.internal_hpwl,c.id))
    if len(front)<=max_keep: return front
    # Shape spectrum and low-area seed; real Pareto filtering precedes capacity selection.
    chosen=[front[0]]
    ordered=sorted(front,key=lambda c:(c.box[2]-c.box[0])/max(c.box[3]-c.box[1],1e-12))
    for idx in np.linspace(0,len(ordered)-1,max_keep).round().astype(int):
        if ordered[idx] not in chosen: chosen.append(ordered[idx])
    return chosen[:max_keep]


def solve_intra(problem,cid,budget,seed,feedback=None,region=None,mechanical=None):
    refs=problem.groups[cid]; cfg=problem.cfg['optimization']; comps=problem.design.by_ref
    source=feedback.by_ref if feedback is not None else {r:c.source_pose for r,c in comps.items()}
    if mechanical is not None:
        source.update({p.ref:p for p in mechanical.poses})
    center=np.mean([[source[r].x,source[r].y] for r in refs],axis=0)
    if region is not None: center=np.array([(region[0]+region[2])/2,(region[1]+region[3])/2])
    local_source=tuple(Pose(r,source[r].x-center[0],source[r].y-center[1],source[r].angle,source[r].layer) for r in refs)
    anchor_refs=set(problem.fixed)|set(problem.special)
    anchors=tuple(Pose(r,source[r].x-center[0],source[r].y-center[1],source[r].angle,source[r].layer)
                  for r in sorted(anchor_refs) if r not in refs)
    candidates=[]
    if local_valid(problem,local_source+anchors):
        candidates.append(make_shape(problem,cid,local_source,seed,{'origin':'source'}))
    ext=PlacementState(tuple(Pose(r,p.x-center[0],p.y-center[1],p.angle,p.layer) for r,p in source.items()),())
    total=sum((bbox(problem.occupancy(r))[2]-bbox(problem.occupancy(r))[0]+problem.gap)*
              (bbox(problem.occupancy(r))[3]-bbox(problem.occupancy(r))[1]+problem.gap) for r in refs)
    aspects=cfg['shape_aspects'] if region is None else [(region[2]-region[0])/(region[3]-region[1])]
    for k,aspect in enumerate(aspects):
        if budget.expired: break
        sd=stable_seed(seed,cid,k,'feedback' if feedback else 'initial'); rng=np.random.default_rng(sd)
        if region is None:
            width=math.sqrt(total/.55*aspect); height=math.sqrt(total/.55/aspect)
        else: width=region[2]-region[0]; height=region[3]-region[1]
        angles={r:(source[r].angle if k==0 and any(angle_equal(source[r].angle,a) for a in problem.angles[r])
                   else problem.angles[r][0 if k==0 else int(rng.integers(len(problem.angles[r])))]) for r in refs}
        widths=[]; heights=[]
        for r in refs:
            b=bbox(transform(problem.occupancy(r),Pose(r,0,0,angles[r])))
            widths.append(b[2]-b[0]); heights.append(b[3]-b[1])
        if region is None:
            width=max(width,max(widths)+4*problem.margin); height=max(height,max(heights)+4*problem.margin)
        region_local=(-width/2,-height/2,width/2,height/2)
        for attempt in range(3 if region is None else 1):
            blocks=[]
            for r in refs:
                pose=Pose(r,0.,0.,angles[r]); box=bbox(transform(problem.occupancy(r),pose))
                bounds=translation_bounds(box,region_local,problem.margin)
                initial=(source[r].x-center[0],source[r].y-center[1])
                blocks.append(Block(r,(pose,),box,problem.domains[r],bounds,initial))
            for p in anchors:
                local=Pose(p.ref,0.,0.,p.angle,p.layer)
                box=bbox(transform(problem.occupancy(p.ref),local))
                blocks.append(Block('anchor:'+p.ref,(local,),box,problem.domains[p.ref],
                                    (p.x,p.y,p.x,p.y),(p.x,p.y),True))
            for obstacle in problem.obstacles:
                if 'top' in obstacle.get('layers',['top','bottom']):
                    ob=obstacle['bbox_mm']
                    box=(ob[0]-center[0],ob[1]-center[1],ob[2]-center[0],ob[3]-center[1])
                    blocks.append(Block('obstacle:'+obstacle['id'],(),box,'MECHANICAL',
                                        (0.,0.,0.,0.),(0.,0.),True))
            z0=seed_pack(problem,blocks,rng,randomize=attempt>0)
            if z0 is None:
                if region is None: region_local=tuple(x*1.2 for x in region_local)
                continue
            projector,graph=constraint_projector(problem,blocks,z0)
            if projector is None: continue
            objective=objective_for(problem,blocks,max(width,height),external=ext,compact=.4)
            def decode(z): return tuple(p for p in expand(blocks,z,()).poses if p.ref in refs)
            result=optimize(objective,z0,projector,cfg['iterations_a'],cfg['learning_rate_mm'],
                            cfg['gamma_start_mm'],cfg['gamma_end_mm'],budget,
                            accept=lambda z:local_valid(problem,expand(blocks,z,()).poses),optimizer=cfg['optimizer'])
            if result.z is not None:
                candidates.append(make_shape(problem,cid,decode(result.z),sd,{**result.stats,'graph':graph})); break
    if region is not None:
        candidates=[c for c in candidates if c.box[2]-c.box[0]<=region[2]-region[0]+problem.eps and c.box[3]-c.box[1]<=region[3]-region[1]+problem.eps]
    return select_shapes(candidates,problem.cfg['clusters']['max_candidates'])
