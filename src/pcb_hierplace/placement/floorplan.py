"""Stage B: discrete rigid shapes/topologies with continuous projected gradients."""
from __future__ import annotations
from dataclasses import replace
from functools import lru_cache
import itertools
import math
import numpy as np
from ..core.schema import Pose,StageResult,Status
from ..core.geometry import rotation,angle_equal,bbox,transform,union_bbox
from ..constraints.validator import validate_placement
from ..opt.projected import optimize
from ..opt.ranking import evaluate,choose,rank
from .common import (Block,obstacle_blocks,translation_bounds,seed_pack,constraint_projector,
                     objective_for,expand)
from .intra import stable_seed


def rotated_shape(problem,shape,angle):
    members=[]
    for p in shape.members:
        theta=(p.angle+angle)%360
        if not any(angle_equal(theta,a) for a in problem.angles[p.ref]): return None
        x,y=rotation(angle)@np.array([p.x,p.y])
        members.append(Pose(p.ref,float(x),float(y),theta,p.layer))
    box=union_bbox([bbox(transform(problem.occupancy(p.ref),p)) for p in members])
    return tuple(members),box


def build_blocks(problem,library,preplacement,choices):
    domains=dict(preplacement.domain_regions); blocks=obstacle_blocks(problem,preplacement.poses)
    board=bbox(preplacement.outline)
    for cid in sorted(problem.groups):
        shape,theta=choices[cid]
        rotated=rotated_shape(problem,shape,theta)
        if rotated is None: return None
        members,box=rotated; domain=problem.domains[problem.groups[cid][0]]
        bounds=list(translation_bounds(box,domains[domain],problem.margin))
        for p in members:
            if problem.motions[p.ref]=='region_bounded':
                rr=problem.cfg['rules']['regions'][problem.rules[p.ref]['region_id']]
                mb=bbox(transform(problem.occupancy(p.ref),p)); rb=translation_bounds(mb,rr,problem.margin)
                bounds=[max(bounds[0],rb[0]),max(bounds[1],rb[1]),min(bounds[2],rb[2]),min(bounds[3],rb[3])]
        initial=np.mean([[problem.design.by_ref[p.ref].source_pose.x-p.x,
                          problem.design.by_ref[p.ref].source_pose.y-p.y] for p in members],axis=0)
        blocks.append(Block('cluster:'+cid,members,box,domain,tuple(bounds),tuple(initial)))
    return blocks


def solve_floorplan(problem,library,preplacement,budget,seed,trials=3):
    cfg=problem.cfg['optimization']; options={}; stats=[]; best=None
    for cid,shapes in library.items():
        options[cid]=[(s,a) for s in shapes for a in (0.,90.,180.,270.) if rotated_shape(problem,s,a) is not None]
        if not options[cid]: return StageResult(Status.NO_FEASIBLE_FOUND,termination_reason='no_legal_shape_orientation')
    dims=bbox(preplacement.outline); length=math.hypot(dims[2]-dims[0],dims[3]-dims[1])
    for attempt in range(trials):
        if budget.expired: break
        rng=np.random.default_rng(stable_seed(seed,preplacement.topology,attempt))
        choices={cid:values[0 if attempt==0 else int(rng.integers(len(values)))] for cid,values in options.items()}
        blocks=build_blocks(problem,library,preplacement,choices)
        if blocks is None: continue
        if not blocks: return StageResult(Status.INPUT_INVALID,termination_reason='no_objects')
        z0=seed_pack(problem,blocks,rng,randomize=attempt>0)
        if z0 is None:
            stats.append({'attempt':attempt,'reason':'packing_seed_failed'}); continue
        projector,graph=constraint_projector(problem,blocks,z0)
        if projector is None: continue
        def decode(z):
            s=expand(blocks,z,preplacement.outline,preplacement.domain_regions,'B')
            return replace(s,choices=tuple((cid,f'{s.id}@{a}') for cid,(s,a) in sorted(choices.items())))
        @lru_cache(maxsize=8)
        def checked(key):
            return evaluate(problem,decode(np.frombuffer(key,dtype=np.float64)))
        def candidate(z): return checked(np.asarray(z,dtype=np.float64).tobytes())
        objective=objective_for(problem,blocks,length)
        initial=candidate(z0)
        if initial.validation.model_feasible: best=choose(problem,best,initial)
        result=optimize(objective,z0,projector,cfg['iterations_b'],cfg['learning_rate_mm'],
                        cfg['gamma_start_mm'],cfg['gamma_end_mm'],budget,
                        accept=lambda z:candidate(z).validation.model_feasible,
                        score=lambda z:rank(problem,candidate(z)),optimizer=cfg['optimizer'])
        stats.append({'attempt':attempt,**result.stats,'graph':graph,
                      'initial_violations':[v.rule for v in initial.validation.violations]})
        if result.z is not None: best=choose(problem,best,candidate(result.z))
    return StageResult(Status.FEASIBLE if best else Status.NO_FEASIBLE_FOUND,best,
                       termination_reason='budget' if budget.expired else 'candidate_budget',
                       stats={'attempts':stats,'topology':preplacement.topology})
