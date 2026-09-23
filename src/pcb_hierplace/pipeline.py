"""Explicit stage orchestration with bounded A/B feedback and legal incumbents."""
from __future__ import annotations
from dataclasses import asdict,replace
from pathlib import Path
import copy
import math
import shutil
import time
from .config import check_config
from .core.schema import Budget,PlacementError,Status,Pose,StageResult,digest,file_hash
from .core.geometry import bbox,rect
from .io.epro2 import parse_project
from .io.cluster_csv import load_assignments
from .constraints.compiler import compile_constraints
from .constraints.validator import validate_placement
from .placement.intra import solve_intra
from .placement.preplace import preplace,board_candidates
from .placement.floorplan import solve_floorplan
from .placement.refine import refine
from .opt.ranking import evaluate,choose,metrics
from .reporting import write_json,layout_svg,environment


def load_problem(cfg):
    check_config(cfg)
    design=parse_project(cfg['input']['project'],cfg['input']['board_id'],
                         {'source_unit':cfg['input']['source_unit'],'geometry':cfg['geometry']})
    assignments=load_assignments(design,cfg['input']['voltage_domains'],cfg['input']['functional_clusters'],cfg)
    return compile_constraints(design,assignments,cfg)


def _quantize(problem,record,reference=None):
    grid=problem.cfg['geometry']['output_grid_mm']; state=record.state
    frozen=set(problem.fixed)|set(problem.special)
    poses=tuple(p if p.ref in frozen else Pose(p.ref,round(p.x/grid)*grid,round(p.y/grid)*grid,p.angle,p.layer)
                for p in state.poses)
    trial=evaluate(problem,replace(state,poses=poses,tag=state.tag+'-grid'),reference=reference)
    # Grid is an output preference, never permission to break constraints or priority.
    return choose(problem,record,trial)


def run(cfg,out,progress=print):
    root=Path(out)
    if root.exists() and any(root.iterdir()): raise PlacementError('OUTPUT_EXISTS','输出目录非空，请选择新目录以保留旧结果')
    root.mkdir(parents=True,exist_ok=True)
    t0=time.monotonic(); stages={}; best=None; stage_times={}
    def log(s):
        progress(s)
        with (root/'pipeline.log').open('a',encoding='utf-8') as f: f.write(s+'\n')
    write_json(root/'effective_config.json',cfg)
    try:
        problem=load_problem(cfg)
        stored_cfg=copy.deepcopy(cfg)
        (root/'inputs').mkdir()
        for key,name in [('project','source.epro2'),('voltage_domains','voltage_domains.csv'),
                         ('functional_clusters','functional_clusters.csv')]:
            shutil.copyfile(cfg['input'][key],root/'inputs'/name)
            stored_cfg['input'][key]='inputs/'+name
        write_json(root/'effective_config.json',stored_cfg)
        budget=Budget(cfg['optimization']['time_budget_s'])
        write_json(root/'normalized_design.json',problem.design)
        write_json(root/'assignments.json',problem.assignments)
        write_json(root/'compiled_constraints.json',{'motions':problem.motions,'angles':problem.angles,
             'groups':problem.groups,'fixed':problem.fixed,'special':problem.special,
             'pad_domains':[(list(k),v) for k,v in problem.pad_domains.items()],
             'estimated':problem.estimated,'reference_length_mm':problem.reference_length_mm,'rules':cfg['rules']})
        write_json(root/'input_diagnostics.json',{'status':'VALIDATED','changes':problem.assignments.changes,
             'native_editor_status':'PENDING','body_geometry':{c.ref:c.body_source for c in problem.design.components}})
        log(f'P0: {len(problem.design.components)} 器件, {len(problem.groups)} 可动功能簇, {len(problem.fixed)} 固定对象')
        stage_times['P0']=time.monotonic()-t0; stage_start=time.monotonic()
        preliminary=None
        for provisional_outline in board_candidates(problem):
            preliminary=next(preplace(problem,provisional_outline,budget),None)
            if preliminary is not None or budget.expired: break
        if preliminary is None:
            raise PlacementError('MECHANICAL_PREPLACEMENT_FAILED',
                '已配置的板框候选及简单两域分区中未找到机械预布局',Status.NO_FEASIBLE_FOUND)
        write_json(root/'preplacement_initial.json',preliminary)
        stage_times['P1']=time.monotonic()-stage_start; stage_start=time.monotonic()
        preliminary_poses={p.ref:p for p in preliminary.poses}
        preliminary_state=replace(problem.design.source_state(preliminary.outline),
            poses=tuple(preliminary_poses.get(c.ref,c.source_pose) for c in problem.design.components),
            domain_regions=preliminary.domain_regions,tag='P1-partial-placement')
        log(f'P1: 机械预布局与域拓扑 {preliminary.topology} 已生成')
        library={}
        # Domain dimensions are unknown during initial local shape generation.
        for cid in sorted(problem.groups):
            shapes=solve_intra(problem,cid,budget,cfg['optimization']['seed'],mechanical=preliminary)
            library[cid]=shapes
            write_json(root/'stageA'/f'{digest(cid)[:12]}.json',{'cluster_id':cid,'candidates':shapes})
            log(f'A: {cid}: {len(shapes)} 个内部合法形状')
            if not shapes:
                raise PlacementError('NO_LEGAL_SHAPE',f'{cid} 在预算内无合法形状',Status.NO_FEASIBLE_FOUND)
        stages['A']={'shape_counts':{k:len(v) for k,v in library.items()}}
        stage_times['A']=time.monotonic()-stage_start; stage_start=time.monotonic()
        trials=0; bstats=[]; budget_max=cfg['optimization']['max_topology_candidates']; best_pre=None
        initial_outline=None
        for outline in board_candidates(problem):
            if budget.expired or trials>=budget_max: break
            initial_outline=initial_outline or outline
            for pp in preplace(problem,outline,budget):
                if budget.expired or trials>=budget_max: break
                remaining=min(3,budget_max-trials)
                result=solve_floorplan(problem,library,pp,budget,cfg['optimization']['seed']+trials,trials=remaining)
                trials+=remaining; bstats.append(result.stats)
                old=best; best=choose(problem,best,result.best_feasible)
                if best is not old: best_pre=pp
                log(f'B: topology {pp.topology}, candidates {trials}/{budget_max}, feasible={result.best_feasible is not None}')
                # Fixed-board search continues to explore until its configured budget.
                if problem.estimated and result.best_feasible is not None: break
            if problem.estimated and best is not None: break
        stages['B']={'trials':trials,'attempts':bstats}
        write_json(root/'stageB.json',stages['B'])
        if best is None:
            raise PlacementError('SEARCH_EXHAUSTED','当前形状/分区/拓扑搜索未找到可行布局，未证明全问题无解',Status.NO_FEASIBLE_FOUND,
                                 {'trials':trials,'time_remaining_s':budget.remaining})
        best_b=best
        stage_times['B']=time.monotonic()-stage_start; stage_start=time.monotonic()
        # Explicit shrinking loop; each size change regenerates mechanical candidates.
        if problem.estimated and not budget.expired:
            base=bbox(best.state.outline); factor=cfg['board']['estimated']['shrink_factor']
            for shrink in range(3):
                w=(base[2]-base[0])*factor; h=(base[3]-base[1])*factor
                dims=cfg['board']['estimated']
                if w<dims['min_width_mm'] or h<dims['min_height_mm']: break
                outline=rect((base[0],base[1],base[0]+w,base[1]+h))
                improvement=False
                for i,pp in enumerate(preplace(problem,outline,budget)):
                    if i>=2 or budget.expired: break
                    result=solve_floorplan(problem,library,pp,budget,cfg['optimization']['seed']+shrink,trials=2)
                    updated=choose(problem,best,result.best_feasible)
                    if updated is not best: best=updated; best_pre=pp; improvement=True
                if not improvement: break
                base=bbox(best.state.outline)
        stage_times['board_shrink']=time.monotonic()-stage_start; stage_start=time.monotonic()
        # A-prime: port feedback inside assigned regions, then rebuild B geometry/graphs.
        feedback_stats=[]
        for iteration in range(cfg['optimization']['max_ab_feedback_rounds']):
            if budget.expired or best_pre is None: break
            newlib={}; regions=dict(best.state.regions)
            for cid in sorted(problem.groups):
                new=solve_intra(problem,cid,budget,cfg['optimization']['seed']+iteration+1,
                                feedback=best.state,region=regions[cid])
                newlib[cid]=new or library[cid]
            result=solve_floorplan(problem,newlib,best_pre,budget,cfg['optimization']['seed']+100+iteration,trials=2)
            updated=choose(problem,best,result.best_feasible)
            feedback_stats.append({'iteration':iteration,'improved':updated is not best,'solver':result.stats})
            if updated is best: break
            best=updated; library=newlib
        stages['A_prime']=feedback_stats
        stage_times['A_prime']=time.monotonic()-stage_start; stage_start=time.monotonic()
        before_c=best
        log('C: 在冻结的板框、分区和簇区域中微调' if cfg['optimization']['enable_stage_c'] else 'C: 按显式实验配置跳过')
        result=refine(problem,best,budget) if cfg['optimization']['enable_stage_c'] else StageResult(
            Status.FEASIBLE,best,termination_reason='disabled_by_explicit_experiment_config')
        best=choose(problem,best,result.best_feasible)
        stages['C']={'reason':result.termination_reason,'stats':result.stats}
        stage_times['C']=time.monotonic()-stage_start
        c_record=result.best_feasible or before_c
        best=_quantize(problem,best,reference=before_c.state)
        final=evaluate(problem,best.state)
        if not final.validation.model_feasible:
            raise PlacementError('FINAL_VALIDATION','最终独立验证失败',Status.NUMERICAL_FAILURE)
        source=problem.design.source_state(best.state.outline if not problem.design.outline else None)
        report={'status':str(Status.FEASIBLE),'source':{'metrics':metrics(problem,source),
            'validation':validate_placement(problem,source,require_regions=False).as_dict()},
            'mechanical_preplacement':{'metrics':metrics(problem,preliminary_state),
                'validation':validate_placement(problem,preliminary_state,require_regions=False).as_dict(),
                'note':'仅预布固定件、接口和桥接器件；普通器件仍在源位置，不代表全板合法初解'},
            'best_B':best_b.metrics,'before_C':before_c.metrics,'final':final.metrics,
            'C':{'metrics':c_record.metrics,'validation':c_record.validation.as_dict()},
            'validation':final.validation.as_dict(),'native_open_validation':'PENDING',
            'stages':stages,'stage_elapsed_s':stage_times,'elapsed_s':time.monotonic()-t0,'termination_reason':'budget' if budget.expired else 'completed',
            'scope':'single-top-side rectangular-board, explicit convex geometry and 2D copper spacing',
            'output_grid_applied':best.state.tag.endswith('-grid')}
        write_json(root/'placements.json',{'status':'FEASIBLE','units':'mm','frame':'board_local_y_up',
            'rotation':'ccw_degrees','source_hash':problem.design.source_hash,
            'rule_id':cfg['rules']['id'],'object_uuids':{c.ref:c.uuid for c in problem.design.components},'state':best.state})
        write_json(root/'report.json',report)
        manifest={'schema_version':1,'source_hash':problem.design.source_hash,
                  'domains_hash':file_hash(cfg['input']['voltage_domains']),
                  'clusters_hash':file_hash(cfg['input']['functional_clusters']),
                  'config_hash':digest(stored_cfg),'placement_hash':best.state.digest,
                  'environment':environment(),'seed':cfg['optimization']['seed'],
                  'rule_id':cfg['rules']['id'],'adapter':problem.design.adapter,
                  'units':'mm','frame':'board_local_y_up','rotation':'ccw_degrees'}
        write_json(root/'run_manifest.json',manifest)
        layout_svg(problem,source,best.state,True,root/'layout.svg')
        if cfg['export']['emit_epro2']:
            from .io.export import export_project
            export_project(problem,best,root/'board_placed.epro2')
        log(f'FEASIBLE: HPWL={best.metrics["hpwl_mm"]:.4f} mm, {time.monotonic()-t0:.2f}s')
        return report
    except PlacementError as e:
        write_json(root/'report.json',{**e.as_dict(),'stages':stages,'elapsed_s':time.monotonic()-t0})
        write_json(root/'input_diagnostics.json',e.as_dict())
        if 'problem' in locals() and 'initial_outline' in locals() and initial_outline:
            diagnostic=problem.design.source_state(initial_outline)
            layout_svg(problem,diagnostic,diagnostic,False,root/'layout.svg')
        log(f'{e.status}: {e.code}: {e}')
        raise
    except Exception as e:
        import traceback
        (root/'exception.log').write_text(traceback.format_exc(),encoding='utf-8')
        wrapped=PlacementError('UNEXPECTED_FAILURE',f'{type(e).__name__}: {e}',Status.NUMERICAL_FAILURE)
        write_json(root/'report.json',{**wrapped.as_dict(),'stages':stages,'elapsed_s':time.monotonic()-t0})
        raise wrapped from e
