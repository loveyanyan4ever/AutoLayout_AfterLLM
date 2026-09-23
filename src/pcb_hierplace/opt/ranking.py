"""Exact metrics and lexicographic selection, never cross-schedule loss comparison."""
from __future__ import annotations
from dataclasses import replace
import math
import numpy as np
from ..core.schema import CandidateRecord
from ..core.geometry import bbox,transform,union_bbox,pad_center,angle_equal
from ..constraints.validator import validate_placement,pin


def metrics(problem,state):
    poses=state.by_ref; refs=problem.design.by_ref
    weighted=unweighted=0.
    for net in problem.design.nets:
        q=[]
        for ref,pid in net.pins:
            if ref not in poses: continue
            pad=next(p for p in refs[ref].pads if p.id==pid)
            q.append(pad_center(pad,poses[ref]))
        if len(q)>1:
            qq=np.asarray(q); length=float(np.ptp(qq[:,0])+np.ptp(qq[:,1]))
            weighted+=problem.cfg['rules']['net_weights'].get(net.id,1.)*length; unweighted+=length
    b=bbox(state.outline) if state.outline else (0,0,0,0)
    intent=0.
    for r in problem.cfg['rules']['distance_constraints']:
        if not r.get('hard',True):
            distance=np.linalg.norm(pin(problem,state,r['a'])-pin(problem,state,r['b']))
            intent+=r.get('weight',1.)*max(0.,distance-r['target_mm'])**2
    compact=0.
    for members in problem.groups.values():
        box=union_bbox([bbox(transform(problem.occupancy(r),poses[r])) for r in members])
        compact+=(box[2]-box[0])*(box[3]-box[1])
    parent_members={}
    for cid,parent in problem.assignments.parent_clusters:
        parent_members.setdefault(parent,[]).extend(problem.groups.get(cid,()))
    parent_compact=0.
    for members in parent_members.values():
        if not members: continue
        box=union_bbox([bbox(transform(problem.occupancy(r),poses[r])) for r in members])
        parent_compact+=(box[2]-box[0])*(box[3]-box[1])
    displacement=sum((poses[r].x-c.source_pose.x)**2+(poses[r].y-c.source_pose.y)**2 for r,c in refs.items())
    rotations=sum(not angle_equal(poses[r].angle,c.source_pose.angle) for r,c in refs.items())
    return {'hpwl_weighted_mm':float(weighted),'hpwl_mm':float(unweighted),
            'intent_penalty':float(intent),'cluster_extent_area_mm2':float(compact),
            'parent_extent_area_mm2':float(parent_compact),
            'board_area_mm2':float((b[2]-b[0])*(b[3]-b[1])),
            'displacement_squared_mm2':float(displacement),'rotation_changes':rotations,
            'components':len(poses),'clusters':len(problem.groups)}


def evaluate(problem,state,reference=None):
    report=validate_placement(problem,state,reference=reference)
    return CandidateRecord(state,report,metrics(problem,state))


def priority_keys(problem):
    area_first=problem.estimated and problem.cfg['optimization']['priority_policy'] in ('auto','feasibility_intent_area_wirelength_displacement')
    keys=(['intent_penalty','board_area_mm2','hpwl_weighted_mm'] if area_first else
          ['intent_penalty','hpwl_weighted_mm','cluster_extent_area_mm2'])
    if problem.assignments.parent_clusters: keys.append('parent_extent_area_mm2')
    return keys+['displacement_squared_mm2','rotation_changes']


def rank(problem,record):
    return tuple(record.metrics[k] for k in priority_keys(problem))+(record.state.digest,)


def choose(problem,old,new):
    if new is None or not new.validation.model_feasible: return old
    keys=priority_keys(problem)
    if old is None:
        return replace(new,priority_reference=tuple((k,new.metrics[k]) for k in keys))
    if new is old: return old
    tolerance=problem.cfg['optimization']['higher_priority_relative_tolerance']
    # Persist the reference across accepted candidates: relative slack cannot compound.
    reference=dict(old.priority_reference) or {k:old.metrics[k] for k in keys}
    next_reference={}
    for i,k in enumerate(keys):
        a,b=reference[k],new.metrics[k]
        guard=1e-9+tolerance*abs(a)
        if b>a+guard: return old
        next_reference[k]=min(a,b)
        if b<a-guard:
            next_reference.update({q:new.metrics[q] for q in keys[i+1:]})
            return replace(new,priority_reference=tuple(next_reference.items()))
    if new.state.digest<old.state.digest:
        return replace(new,priority_reference=tuple(next_reference.items()))
    return old
