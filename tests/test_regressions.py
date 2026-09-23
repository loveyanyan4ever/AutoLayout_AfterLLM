from dataclasses import replace,asdict
import copy
import csv
import json
from pathlib import Path
import numpy as np
import pytest
from pcb_hierplace.config import DEFAULTS,load_config
from pcb_hierplace.core.schema import (Pose,Pad,Component,Net,DesignSnapshot,Assignments,PlacementState,
                                       PlacementError,Budget,digest)
from pcb_hierplace.core.geometry import rect,bbox
from pcb_hierplace.constraints.compiler import compile_constraints
from pcb_hierplace.constraints.validator import validate_placement
from pcb_hierplace.pipeline import load_problem,run
from pcb_hierplace.demo import create_demo
from pcb_hierplace.cli import load_run,main
from pcb_hierplace.io.export import export_project
from pcb_hierplace.placement.intra import solve_intra
from pcb_hierplace.opt.objectives import Objective


def tiny_problem(distance=3.9,domains=('HV','LV'),locked=False):
    c=[]
    for i,ref in enumerate(['A','B']):
        pad=Pad('pad','1',rect((-.5,-.5,.5,.5)),(0,0),'net'+ref)
        c.append(Component(ref,ref,'box',Pose(ref,4+i*distance,5),rect((-.5,-.5,.5,.5)),(pad,),locked))
    d=DesignSnapshot('b','','','synthetic','mm',(0,0),rect((0,0,20,20)),tuple(c),
                     tuple(Net('net'+x.ref,((x.ref,'pad'),)) for x in c))
    cfg=copy.deepcopy(DEFAULTS)
    cfg['board']['mode']='fixed'
    cfg['rules'].update(mechanical_gap_mm=.05,default_pad_clearance_mm=.05,
        domains={'HV':{'kind':'electrical'},'LV':{'kind':'electrical'}},
        isolation_pairs=[{'domains':['HV','LV'],'copper_clearance_mm':3.,'creepage_mm':3.,'routing_reserve_mm':0.}])
    a=Assignments(tuple(zip(['A','B'],domains)),(('A','ca'),('B','cb')),(('A','ca'),('B','cb')))
    return compile_constraints(d,a,cfg)


def test_copper_29_mm_does_not_meet_30_mm():
    p=tiny_problem()
    report=validate_placement(p,p.design.source_state(),require_regions=False)
    assert report.min_copper_distance_mm==pytest.approx(2.9)
    assert any(v.rule=='COPPER_CLEARANCE' for v in report.violations)


@pytest.mark.parametrize('distance',[.98,1.02])
def test_shallow_overlap_and_short_gap_are_both_invalid(distance):
    p=tiny_problem(distance,('HV','HV'))
    report=validate_placement(p,p.design.source_state(),require_regions=False)
    assert any(v.rule=='MECHANICAL_GAP' for v in report.violations)


def test_fixed_fixed_conflict_is_input_conflict():
    with pytest.raises(PlacementError) as e:tiny_problem(.98,('HV','HV'),locked=True)
    assert e.value.code=='FIXED_COLLISION'


def test_coincident_wirelength_gradient_remains_finite():
    o=Objective([([(0,0,0),(1,0,0)],1)],1.)
    value,g,_=o.evaluate(np.zeros(4),.1)
    assert np.isfinite(value) and np.isfinite(g).all()


def test_a_does_not_mutate_source_rotations(problem):
    before=digest(asdict(problem.design))
    cid=next(iter(problem.groups))
    solve_intra(problem,cid,Budget(10),7)
    assert digest(asdict(problem.design))==before


def test_preserve_45_degree_allowlist(fixture_config):
    # Source modification represents an explicitly declared original 45-degree part.
    from test_contracts import mutate_archive
    def change(s):
        out=[]
        for line in s.splitlines():
            head,body=line.split('||',1);h=json.loads(head);b=json.loads(body[:-1])
            if h.get('type')=='COMPONENT' and h.get('id')=='id_R1':b['angle']=-45.
            out.append(json.dumps(h)+'||'+json.dumps(b)+'|')
        return '\n'.join(out)+'\n'
    mutate_archive(fixture_config['input']['project'],change)
    fixture_config['rules']['component_rules']['R1']={'allowed_angles':[45.]}
    p=load_problem(fixture_config)
    shapes=solve_intra(p,'sense',Budget(10),0)
    assert shapes and all(s.members[0].angle==45. for s in shapes)


def test_cross_domain_cluster_error_and_explicit_split(fixture_config):
    p=Path(fixture_config['input']['functional_clusters'])
    rows=list(csv.DictReader(p.read_text().splitlines()))
    for row in rows:
        if row['designator']=='U1':row['cluster_id']='switch'
    with p.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=['designator','cluster_id']);w.writeheader();w.writerows(rows)
    with pytest.raises(PlacementError) as e:load_problem(fixture_config)
    assert e.value.code=='CROSS_DOMAIN_CLUSTER'
    fixture_config['clusters']['cross_domain_policy']='split_by_domain'
    problem=load_problem(fixture_config)
    assert problem.assignments.changes


def test_estimated_outline_is_written_and_reread(tmp_path):
    create_demo(tmp_path/'fixture',estimated=True)
    cfg=load_config(tmp_path/'fixture'/'constraints.yaml')
    cfg['optimization'].update(iterations_a=5,iterations_b=6,iterations_c=5,
        max_topology_candidates=3,max_ab_feedback_rounds=0,time_budget_s=60)
    report=run(cfg,tmp_path/'run',progress=lambda _:None)
    p,r=load_run(tmp_path/'run')
    assert not p.design.outline_present and r.validation.model_feasible
    exported=export_project(p,r,tmp_path/'estimated.epro2')
    assert exported['board_written'] and exported['roundtrip_validation']=='PASSED'


def test_fixed_no_space_does_not_expand(fixture_config,tmp_path):
    fixture_config['rules']['regions']={'tiny':[10,10,10.1,10.1]}
    fixture_config['rules']['component_rules']['Q1'].update(mode='region_bounded',region_id='tiny')
    fixture_config['optimization'].update(iterations_a=3,iterations_b=3,iterations_c=3,
        max_topology_candidates=3,max_ab_feedback_rounds=0,time_budget_s=20)
    with pytest.raises(PlacementError) as e:run(fixture_config,tmp_path/'run',progress=lambda _:None)
    assert str(e.value.status)=='NO_FEASIBLE_FOUND'
    report=json.loads((tmp_path/'run'/'report.json').read_text())
    assert report['status']=='NO_FEASIBLE_FOUND'
    assert not (tmp_path/'run'/'placements.json').exists()


def test_grow_limit_uses_full_area(solved):
    _,p,r,_=solved
    # Actual implementation must reject a displaced member when 0% growth is configured.
    q=replace(p,cfg=copy.deepcopy(p.cfg));q.cfg['optimization']['stage_c']['cluster_area_growth_ratio']=0.
    ref=q.groups['switch'][0]
    st=replace(r.state,poses=tuple(replace(x,x=x.x+10) if x.ref==ref else x for x in r.state.poses))
    assert any(v.rule=='CLUSTER_GROWTH' for v in validate_placement(q,st,reference=r.state).violations)


def test_same_seed_deterministic_and_baseline_frozen(tmp_path):
    create_demo(tmp_path/'fixture')
    cfg=load_config(tmp_path/'fixture'/'constraints.yaml')
    cfg['optimization'].update(iterations_a=3,iterations_b=3,iterations_c=3,
        max_topology_candidates=3,max_ab_feedback_rounds=0,time_budget_s=60)
    run(cfg,tmp_path/'a',progress=lambda _:None);run(cfg,tmp_path/'b',progress=lambda _:None)
    a=json.loads((tmp_path/'a'/'placements.json').read_text());b=json.loads((tmp_path/'b'/'placements.json').read_text())
    assert a==b
