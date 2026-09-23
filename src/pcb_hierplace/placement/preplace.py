"""Finite mechanical and two-domain topology candidates. No source locks are moved."""
from __future__ import annotations
from dataclasses import dataclass
import itertools
import math
import numpy as np
from ..core.schema import Pose,PlacementState
from ..core.geometry import rect,bbox,area,rotation,transform,inside_box,signed_distance,pad_world
from ..constraints.validator import NORMALS,edge_matches,corner_matches,edge_band_matches


@dataclass(frozen=True)
class PreplacementCandidate:
    outline: tuple
    poses: tuple
    domain_regions: tuple
    topology: str


def board_candidates(problem):
    if not problem.estimated:
        yield problem.design.outline; return
    cfg=problem.cfg['board']['estimated']
    occupied=sum(area(problem.occupancy(c.ref)) for c in problem.design.components)
    gap=max([p['copper_clearance_mm']+p['routing_reserve_mm'] for p in problem.isolation.values()]+[0.])
    fixed_boxes=[bbox(transform(problem.occupancy(r),problem.design.by_ref[r].source_pose)) for r in problem.fixed]
    xmin=min([b[0] for b in fixed_boxes]+[0.]); ymin=min([b[1] for b in fixed_boxes]+[0.])
    minw=max([b[2]-xmin for b in fixed_boxes]+[cfg['min_width_mm']])
    minh=max([b[3]-ymin for b in fixed_boxes]+[cfg['min_height_mm']])
    for aspect in cfg['aspect_candidates']:
        # Reserve a conservative strip estimate and update dimensions per trial.
        guess=occupied/cfg['target_utilization']
        for k in range(cfg['size_trials']):
            s=cfg['growth_factor']**k
            w=max(minw,math.sqrt(guess*aspect)*s+gap)
            h=max(minh,math.sqrt(guess/aspect)*s+gap)
            if w>cfg['max_width_mm'] or h>cfg['max_height_mm']: break
            yield rect((xmin,ymin,xmin+w,ymin+h))


def domain_candidates(problem,outline):
    b=bbox(outline); ds=problem.electrical_domains
    if len(ds)==1:
        yield tuple([(ds[0],b)]),'single'; return
    gap=problem.isolation[tuple(ds)]['copper_clearance_mm']+problem.isolation[tuple(ds)]['routing_reserve_mm']+2*problem.margin
    for axis in (0,1):
        for reverse in (False,True):
            first,second=ds[::-1] if reverse else ds
            for fraction in problem.cfg['optimization']['domain_split_fractions']:
                split=b[axis]+(b[axis+2]-b[axis])*fraction
                low=list(b); high=list(b)
                low[axis+2]=split-gap/2; high[axis]=split+gap/2
                if low[axis+2]<=low[axis] or high[axis+2]<=high[axis]: continue
                yield tuple(sorted([(first,tuple(low)),(second,tuple(high))])),f'{axis}:{int(reverse)}:{fraction}'


def _mechanical_ok(problem,pose,placed,board,regions):
    c=problem.design.by_ref[pose.ref]; rule=problem.rules[pose.ref]
    body=transform(c.body,pose); occ=transform(problem.occupancy(pose.ref),pose)
    motion=problem.motions[pose.ref]
    if motion=='corner' and not corner_matches(rule,pose,board,problem.occupancy(pose.ref),problem.eps): return False
    if motion=='edge_band' and not edge_band_matches(rule,pose,board,problem.occupancy(pose.ref),problem.eps): return False
    has_edge=motion in ('edge_slide','edge_choice') or ('allowed_edges' in rule and motion!='edge_band')
    overhang=has_edge and rule.get('allow_body_overhang',False)
    if has_edge and not edge_matches(rule,pose,board,problem.eps): return False
    if not overhang and not inside_box(body,board,problem.eps): return False
    d=problem.domains[pose.ref]
    if d in regions:
        target=body
        if overhang:
            bb=bbox(body); clipped=(max(bb[0],board[0]),max(bb[1],board[1]),min(bb[2],board[2]),min(bb[3],board[3]))
            target=rect(clipped) if clipped[0]<clipped[2] and clipped[1]<clipped[3] else None
        if target is not None and not inside_box(target,regions[d],problem.eps): return False
    for p in c.pads:
        poly=pad_world(p,pose); pd=problem.pad_domains[(c.ref,p.id)]
        if not inside_box(poly,board,problem.eps) or not inside_box(poly,regions[pd],problem.eps): return False
    for other in placed:
        oo=transform(problem.occupancy(other.ref),other)
        if signed_distance(occ,oo)<max(problem.gap,problem.cfg['rules']['default_pad_clearance_mm'])+problem.margin: return False
    for o in problem.obstacles:
        if 'top' in o.get('layers',['top','bottom']) and signed_distance(occ,rect(o['bbox_mm']))<problem.gap+problem.margin: return False
    return True


def corner_candidates(problem,ref,board):
    """Place the whole occupied envelope inside a requested board corner."""
    rule=problem.rules[ref]; inset=rule['corner_inset_mm']
    source=problem.design.by_ref[ref].source_pose
    for corner in rule['allowed_corners']:
        for theta in problem.angles[ref]:
            bounds=bbox(np.asarray(problem.occupancy(ref))@rotation(theta).T)
            x=(board[0]+inset-bounds[0]) if corner.endswith('left') else (board[2]-inset-bounds[2])
            y=(board[1]+inset-bounds[1]) if corner.startswith('bottom') else (board[3]-inset-bounds[3])
            yield Pose(ref,float(x),float(y),theta,source.layer)


def edge_band_candidates(problem,ref,board):
    """Near-edge placement for vertical mating interfaces; no planar mating claim."""
    rule=problem.rules[ref]; offset=rule['edge_offset_mm']
    source=problem.design.by_ref[ref].source_pose
    for edge in rule['allowed_edges']:
        axis=0 if edge in ('left','right') else 1
        for theta in problem.angles[ref]:
            bb=bbox(np.asarray(problem.occupancy(ref))@rotation(theta).T)
            fixed=(board[axis]+offset-bb[axis]) if edge in ('left','bottom') else (board[axis+2]-offset-bb[axis+2])
            lo=board[1-axis]-bb[1-axis]+problem.margin
            hi=board[3-axis]-bb[3-axis]-problem.margin
            if lo>hi: continue
            for along in np.linspace(lo,hi,problem.cfg['optimization']['preplace_samples']):
                xy=np.zeros(2);xy[axis]=fixed;xy[1-axis]=along
                yield Pose(ref,float(xy[0]),float(xy[1]),theta,source.layer)


def edge_candidates(problem,ref,board):
    rule=problem.rules[ref]; source=problem.design.by_ref[ref].source_pose
    for edge in rule['allowed_edges']:
        a=0 if edge in ('left','right') else 1
        coord={'left':board[0],'right':board[2],'bottom':board[1],'top':board[3]}[edge]
        lo,hi=rule['edge_segment_mm']; hi=min(hi,board[3-a]-board[1-a])
        if lo>hi: continue
        for theta in problem.angles[ref]:
            n=rotation(theta)@np.asarray(rule['local_outward_vector']); n/=np.linalg.norm(n)
            if np.dot(n,NORMALS[edge])<1-1e-7: continue
            off=rotation(theta)@np.asarray(rule['local_mating_point_mm'])
            samples=np.linspace(lo,hi,problem.cfg['optimization']['preplace_samples'])
            for s in samples:
                point=np.zeros(2); point[a]=coord+rule['edge_offset_mm']*NORMALS[edge][a]; point[1-a]=board[1-a]+s
                x,y=point-off
                yield Pose(ref,float(x),float(y),theta,source.layer)


def bridge_candidates(problem,ref,board,regions):
    t=problem.bridges[ref]; a,b=t['between']; ba,bb=regions[a],regions[b]
    ca=np.array([(ba[0]+ba[2])/2,(ba[1]+ba[3])/2]); cb=np.array([(bb[0]+bb[2])/2,(bb[1]+bb[3])/2])
    axis=int(np.argmax(abs(cb-ca))); sign=1 if cb[axis]>ca[axis] else -1
    boundary=(ba[axis+2]+bb[axis])/2 if sign>0 else (bb[axis+2]+ba[axis])/2
    n=np.zeros(2); n[axis]=sign
    for theta in problem.angles[ref]:
        direction=rotation(theta)@np.asarray(t['axis_local']); direction/=np.linalg.norm(direction)
        if np.dot(direction,n)<1-1e-7: continue
        offset=rotation(theta)@np.asarray(t['barrier_point_mm'])
        for fraction in np.linspace(.2,.8,problem.cfg['optimization']['preplace_samples']):
            p=np.zeros(2); p[axis]=boundary; p[1-axis]=board[1-axis]+fraction*(board[3-axis]-board[1-axis])
            x,y=p-offset
            yield Pose(ref,float(x),float(y),theta)


def preplace(problem,outline,budget):
    board=bbox(outline)
    for dr,topology in domain_candidates(problem,outline):
        if budget.expired: return
        regions=dict(dr); placed=[]; valid=True
        for ref in problem.fixed:
            p=problem.design.by_ref[ref].source_pose
            if not _mechanical_ok(problem,p,placed,board,regions): valid=False; break
            placed.append(p)
        if not valid: continue
        beam=[placed]
        for ref in sorted(problem.special,key=lambda r:(len(problem.angles[r]),r)):
            motion=problem.motions[ref]
            if ref in problem.bridges: generated=bridge_candidates(problem,ref,board,regions)
            elif motion=='corner': generated=corner_candidates(problem,ref,board)
            elif motion=='edge_band': generated=edge_band_candidates(problem,ref,board)
            else: generated=edge_candidates(problem,ref,board)
            candidates=list(generated)
            nextbeam=[]
            for items in beam:
                for p in candidates:
                    if _mechanical_ok(problem,p,items,board,regions): nextbeam.append(items+[p])
            source=problem.design.by_ref[ref].source_pose
            nextbeam.sort(key=lambda items:sum((p.x-problem.design.by_ref[p.ref].source_pose.x)**2+(p.y-problem.design.by_ref[p.ref].source_pose.y)**2 for p in items))
            beam=nextbeam[:problem.cfg['optimization']['beam_width']]
            if not beam: break
        for items in beam:
            yield PreplacementCandidate(tuple(outline),tuple(items),dr,topology)
