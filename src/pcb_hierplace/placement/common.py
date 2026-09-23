"""Shared rigid-block geometry, packing seeds and linear constraint graphs."""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from scipy import sparse
from ..core.schema import Pose,PlacementState
from ..core.geometry import bbox,union_bbox,transform,pad_center
from ..opt.objectives import Objective
from ..opt.projected import Projector


@dataclass(frozen=True)
class Block:
    key: str
    members: tuple[Pose,...]  # positions relative to block translation
    box: tuple
    domain: str
    bounds: tuple  # minX,minY,maxX,maxY for the block reference point
    initial: tuple
    fixed: bool=False


def translation_bounds(box,region,margin=0.):
    return (region[0]-box[0]+margin,region[1]-box[1]+margin,
            region[2]-box[2]-margin,region[3]-box[3]-margin)


def expand(blocks,z,outline,domains=(),tag='',regions=None):
    pos=np.asarray(z).reshape(-1,2); poses=[]; rr=[]
    for i,b in enumerate(blocks):
        x,y=pos[i]
        poses.extend(Pose(p.ref,float(x+p.x),float(y+p.y),p.angle,p.layer) for p in b.members)
        if b.key.startswith('cluster:'):
            rr.append((b.key[8:],(float(x+b.box[0]),float(y+b.box[1]),float(x+b.box[2]),float(y+b.box[3]))))
    return PlacementState(tuple(sorted(poses,key=lambda p:p.ref)),tuple(outline),
                          tuple(rr if regions is None else regions),tuple(domains),tag)


def member_map(blocks):
    return {p.ref:(i,p) for i,b in enumerate(blocks) for p in b.members}


def objective_for(problem,blocks,length_scale,external=None,compact=0.,anchor=None,anchor_weight=0.):
    mapping=member_map(blocks); nets=[]; pinmap={}
    for c in problem.design.components:
        for pad in c.pads:
            if c.ref in mapping:
                i,pose=mapping[c.ref]; xy=pad_center(pad,pose); pinmap[(c.ref,pad.id)]=(i,float(xy[0]),float(xy[1]))
            elif external and c.ref in external.by_ref:
                xy=pad_center(pad,external.by_ref[c.ref]); pinmap[(c.ref,pad.id)]=(-1,float(xy[0]),float(xy[1]))
    for net in problem.design.nets:
        pins=[pinmap[k] for k in net.pins if k in pinmap]
        if len(pins)>=2 and any(p[0]>=0 for p in pins):
            nets.append((pins,float(problem.cfg['rules']['net_weights'].get(net.id,1.))))
    distances=[]
    for rule in problem.cfg['rules']['distance_constraints']:
        if rule.get('hard',True): continue
        ps=[]
        for name in ('a','b'):
            ref,num=rule[name].split(':',1)
            pad=next(p for p in problem.design.by_ref[ref].pads if p.number==num)
            ps.append(pinmap.get((ref,pad.id)))
        if all(p is not None for p in ps): distances.append((ps[0],ps[1],rule['target_mm'],rule.get('weight',1.)))
    compact_ids=tuple(i for i,b in enumerate(blocks) if b.members and not b.fixed)
    parents=dict(getattr(problem.assignments,'parent_clusters',()))
    parent_groups={}
    for i,b in enumerate(blocks):
        if b.key.startswith('cluster:') and b.key[8:] in parents and not b.fixed:
            parent_groups.setdefault(parents[b.key[8:]],[]).append(i)
    return Objective(nets,problem.reference_length_mm,compact,anchor,anchor_weight,distances,
                     compact_indices=compact_ids,
                     compact_groups=tuple(tuple(ids) for _,ids in sorted(parent_groups.items()) if len(ids)>1),
                     parent_compact_weight=problem.cfg['clusters'].get('parent_compact_weight',.05))


def pair_gap(problem,a,b):
    gap=max(problem.gap,problem.cfg['rules']['default_pad_clearance_mm'])
    key=tuple(sorted((a.domain,b.domain)))
    if key in problem.isolation: gap=max(gap,problem.isolation[key]['copper_clearance_mm'])
    return gap+problem.margin


def separation_options(a,b,pa,pb,gap):
    # Each option describes x_j-x_i >= required (or equivalent on Y).
    result=[]
    for axis in (0,1):
        req=a.box[axis+2]-b.box[axis]+gap
        result.append((req-(pb[axis]-pa[axis]),axis,1,req))
        req=b.box[axis+2]-a.box[axis]+gap
        result.append((req-(pa[axis]-pb[axis]),axis,-1,req))
    return sorted(result,key=lambda x:(x[0],x[1],x[2]))


def seed_pack(problem,blocks,rng,randomize=False):
    n=len(blocks); pos=np.array([b.initial for b in blocks],float)
    placed=[i for i,b in enumerate(blocks) if b.fixed]
    order=[i for i,b in enumerate(blocks) if not b.fixed]
    order.sort(key=lambda i:-(blocks[i].box[2]-blocks[i].box[0])*(blocks[i].box[3]-blocks[i].box[1]))
    if randomize: rng.shuffle(order)
    for i in order:
        b=blocks[i]; lo=np.array(b.bounds[:2]); hi=np.array(b.bounds[2:])
        if np.any(lo>hi): return None
        xs={float(lo[0]),float(hi[0]),float(np.clip(pos[i,0],lo[0],hi[0]))}
        ys={float(lo[1]),float(hi[1]),float(np.clip(pos[i,1],lo[1],hi[1]))}
        for j in placed:
            a=blocks[j]; g=pair_gap(problem,a,b)
            xs.update((pos[j,0]+a.box[2]-b.box[0]+g,pos[j,0]+a.box[0]-b.box[2]-g))
            ys.update((pos[j,1]+a.box[3]-b.box[1]+g,pos[j,1]+a.box[1]-b.box[3]-g))
        candidates=[(x,y) for x in xs if lo[0]-1e-9<=x<=hi[0]+1e-9 for y in ys if lo[1]-1e-9<=y<=hi[1]+1e-9]
        candidates.sort(key=lambda p:((p[0]-pos[i,0])**2+(p[1]-pos[i,1])**2,p))
        found=None
        for p in candidates:
            if all(separation_options(blocks[j],b,pos[j],p,pair_gap(problem,blocks[j],b))[0][0]<=1e-8 for j in placed):
                found=p; break
        if found is None: return None
        pos[i]=found; placed.append(i)
    return pos.ravel()


def constraint_projector(problem,blocks,seed):
    n=len(blocks); pos=np.asarray(seed).reshape(-1,2)
    rows=[]; cols=[]; data=[]; lower=[]; upper=[]; graph=[]
    def row(entries,lo,hi):
        k=len(lower)
        for col,val in entries: rows.append(k); cols.append(col); data.append(val)
        lower.append(lo); upper.append(hi)
    for i,b in enumerate(blocks):
        for a in (0,1):
            lo,hi=(b.initial[a],b.initial[a]) if b.fixed else (b.bounds[a],b.bounds[a+2])
            if lo>hi: return None,[]
            row([(2*i+a,1.)],lo,hi)
    for i,a in enumerate(blocks):
        for j in range(i+1,n):
            b=blocks[j]
            if not a.members and not b.members: continue
            # Fixed objects that are the same physical source are never duplicated.
            _,axis,sign,req=separation_options(a,b,pos[i],pos[j],pair_gap(problem,a,b))[0]
            row([(2*j+axis,float(sign)),(2*i+axis,float(-sign))],req,np.inf)
            graph.append({'a':a.key,'b':b.key,'axis':'xy'[axis],'direction':sign,'distance':req})
    mapping=member_map(blocks)
    for rule in problem.cfg['rules']['order_constraints']:
        endpoints=[]
        for key in ('a','b'):
            ref,num=rule[key].split(':',1)
            if ref not in mapping: endpoints=[]; break
            i,p=mapping[ref]; pad=next(t for t in problem.design.by_ref[ref].pads if t.number==num)
            endpoints.append((i,pad_center(pad,p)))
        if len(endpoints)==2:
            (i,pa),(j,pb)=endpoints; axis=0 if rule['axis']=='x' else 1
            row([(2*j+axis,1.),(2*i+axis,-1.)],rule.get('min_separation_mm',0)-(pb[axis]-pa[axis]),np.inf)
    distances=[]
    for rule in problem.cfg['rules']['distance_constraints']:
        if not rule.get('hard',True): continue
        endpoints=[]
        for key in ('a','b'):
            ref,num=rule[key].split(':',1)
            if ref not in mapping: endpoints=[]; break
            i,p=mapping[ref]
            pad=next(t for t in problem.design.by_ref[ref].pads if t.number==num)
            endpoints.append((i,pad_center(pad,p)))
        if len(endpoints)==2:
            (i,pa),(j,pb)=endpoints
            offset=pb-pa
            distances.append((i,j,float(offset[0]),float(offset[1]),float(rule['max_mm'])))
    A=sparse.csc_matrix((data,(rows,cols)),shape=(len(lower),2*n))
    return Projector(A,lower,upper,eps=problem.eps*.1,distance_constraints=distances),graph


def obstacle_blocks(problem,poses):
    blocks=[]
    for pose in poses:
        c=problem.design.by_ref[pose.ref]
        local=Pose(pose.ref,0.,0.,pose.angle,pose.layer)
        box=bbox(transform(problem.occupancy(pose.ref),local))
        blocks.append(Block('anchor:'+pose.ref,(local,),box,problem.domains[pose.ref],
                            (pose.x,pose.y,pose.x,pose.y),(pose.x,pose.y),True))
    for o in problem.obstacles:
        if 'top' in o.get('layers',['top','bottom']):
            blocks.append(Block('obstacle:'+o['id'],(),tuple(o['bbox_mm']),'MECHANICAL',(0.,0.,0.,0.),(0.,0.),True))
    return blocks
