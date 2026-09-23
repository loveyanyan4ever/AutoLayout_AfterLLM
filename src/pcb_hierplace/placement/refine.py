"""Stage C: exact-checked trust-region refinement; orientations and board frozen."""
from __future__ import annotations
from functools import lru_cache
import math
import numpy as np
from ..core.schema import Pose,StageResult,Status
from ..core.geometry import bbox,transform
from ..opt.projected import optimize
from ..opt.ranking import evaluate,choose,rank
from ..constraints.validator import validate_placement
from .common import Block,obstacle_blocks,translation_bounds,constraint_projector,objective_for,expand


def refine(problem,record,budget):
    if not record.validation.model_feasible:
        return StageResult(Status.INPUT_INVALID,termination_reason='C_requires_feasible_input')
    state=record.state; poses=state.by_ref; regions=dict(state.regions); domains=dict(state.domain_regions)
    frozen=set(problem.fixed)|set(problem.special)
    blocks=obstacle_blocks(problem,[poses[r] for r in sorted(frozen)])
    cfg=problem.cfg['optimization']; board=bbox(state.outline)
    # Axis-aligned inscribed trust box implies the configured Euclidean trust ball.
    radius=cfg['stage_c']['trust_radius_mm']/math.sqrt(2)
    for ref in sorted(set(poses)-frozen):
        p=poses[ref]; local=Pose(ref,0.,0.,p.angle,p.layer)
        box=bbox(transform(problem.occupancy(ref),local))
        bounds=list(translation_bounds(box,regions[problem.ref_group[ref]],problem.margin*.1))
        db=translation_bounds(box,domains[problem.domains[ref]],problem.margin)
        bounds=[max(bounds[0],db[0],p.x-radius),max(bounds[1],db[1],p.y-radius),
                min(bounds[2],db[2],p.x+radius),min(bounds[3],db[3],p.y+radius)]
        # Original cluster extents are exact; do not shrink them by a new margin.
        bounds[0]=min(bounds[0],p.x); bounds[1]=min(bounds[1],p.y)
        bounds[2]=max(bounds[2],p.x); bounds[3]=max(bounds[3],p.y)
        blocks.append(Block(ref,(local,),box,problem.domains[ref],tuple(bounds),(p.x,p.y)))
    z0=np.array([b.initial for b in blocks]).ravel()
    projector,graph=constraint_projector(problem,blocks,z0)
    if projector is None: return StageResult(Status.FEASIBLE,record,termination_reason='C_no_projection_region')
    length=math.hypot(board[2]-board[0],board[3]-board[1])
    objective=objective_for(problem,blocks,length,anchor=z0.reshape(-1,2)-z0.reshape(-1,2).mean(axis=0),anchor_weight=.1)
    def decode(z): return expand(blocks,z,state.outline,state.domain_regions,'C',regions=state.regions)
    @lru_cache(maxsize=8)
    def checked(key):
        return evaluate(problem,decode(np.frombuffer(key,dtype=np.float64)),reference=state)
    def candidate(z): return checked(np.asarray(z,dtype=np.float64).tobytes())
    higher_keys=['intent_penalty']
    tol=cfg['higher_priority_relative_tolerance']
    def acceptable(z):
        record_trial=candidate(z)
        return record_trial.validation.model_feasible and all(record_trial.metrics[k]<=record.metrics[k]+tol*abs(record.metrics[k])+1e-9 for k in higher_keys)
    result=optimize(objective,z0,projector,cfg['iterations_c'],cfg['learning_rate_mm'],
                    cfg['gamma_start_mm'],cfg['gamma_end_mm'],budget,accept=acceptable,
                    score=lambda z:rank(problem,candidate(z)),optimizer=cfg['optimizer'])
    best=record
    if result.z is not None: best=choose(problem,best,candidate(result.z))
    return StageResult(Status.FEASIBLE,best,termination_reason='improved' if best is not record else 'rollback_to_legal_input',
                       stats={**result.stats,'graph':graph})
