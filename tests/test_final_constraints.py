from dataclasses import replace
import copy
import json
from pathlib import Path
import numpy as np
import pytest
from pcb_hierplace.constraints.compiler import compile_constraints
from pcb_hierplace.constraints.validator import validate_placement
from pcb_hierplace.core.geometry import rect,bbox
from pcb_hierplace.core.schema import Pose,Pad,Assignments,PlacementError
from pcb_hierplace.io.export import export_project,compare_geometry
from pcb_hierplace.io.epro2 import ProjectArchive
from pcb_hierplace.pipeline import load_problem,run
from pcb_hierplace.cli import load_run,main
from test_regressions import tiny_problem
from test_contracts import mutate_archive


def test_fixed_interface_still_checks_outward_direction(problem):
    cfg=copy.deepcopy(problem.cfg)
    cfg['rules']['component_rules']['JH'].update(mode='fixed_pose',local_outward_vector=[1.,0.])
    p=compile_constraints(problem.design,problem.assignments,cfg)
    report=validate_placement(p,p.design.source_state(),require_regions=False)
    assert any(v.rule=='EDGE' and v.objects==('JH',) for v in report.violations)


def test_no_pad_metal_body_participates_in_copper_clearance():
    p=tiny_problem()
    cfg=copy.deepcopy(p.cfg)
    cfg['rules']['domains']['MECHANICAL']={'kind':'mechanical'}
    cfg['rules']['component_rules']['A']={'mode':'fixed_pose','conductive_class':'HV'}
    d=replace(p.design,components=tuple(replace(c,pads=()) if c.ref=='A' else c for c in p.design.components),
              nets=tuple(n for n in p.design.nets if n.id!='netA'))
    a=replace(p.assignments,domains=(('A','MECHANICAL'),('B','LV')))
    q=compile_constraints(d,a,cfg)
    violations=validate_placement(q,d.source_state(),require_regions=False).violations
    assert q.domains['A']=='HV'
    assert any(v.rule=='COPPER_CLEARANCE' and 'A:metal_body' in v.objects for v in violations)


def test_explicit_interface_overhang_does_not_waive_pad_containment(solved):
    _,p,r,_=solved
    cfg=copy.deepcopy(p.cfg);cfg['rules']['component_rules']['JH']['allow_body_overhang']=True
    d=replace(p.design,components=tuple(replace(c,body=rect((-3,-3,2,3))) if c.ref=='JH' else c for c in p.design.components))
    q=compile_constraints(d,p.assignments,cfg)
    assert validate_placement(q,r.state).model_feasible
    cfg['rules']['component_rules']['JH']['allow_body_overhang']=False
    q=compile_constraints(d,p.assignments,cfg)
    assert any(v.rule=='BOARD_BODY' for v in validate_placement(q,r.state).violations)
    j=d.by_ref['JH'];pad=replace(j.pads[0],polygon=rect((-4,-.5,-3,.5)),center=(-3.5,0))
    d=replace(d,components=tuple(replace(c,pads=(pad,)+c.pads[1:]) if c.ref=='JH' else c for c in d.components))
    cfg['rules']['component_rules']['JH']['allow_body_overhang']=True
    q=compile_constraints(d,p.assignments,cfg)
    assert any(v.rule=='BOARD_PAD' for v in validate_placement(q,r.state).violations)


def test_obstacle_blocks_outlying_pad():
    p=tiny_problem(10,('HV','HV'));cfg=copy.deepcopy(p.cfg)
    c=p.design.by_ref['A'];pad=replace(c.pads[0],polygon=rect((2,-.5,3,.5)),center=(2.5,0))
    d=replace(p.design,components=(replace(c,pads=(pad,)),p.design.components[1]))
    cfg['rules']['obstacles']=[{'id':'cutout','bbox_mm':[6,4.5,7,5.5]}]
    q=compile_constraints(d,p.assignments,cfg)
    assert any(v.rule=='OBSTACLE' and v.objects==('A','cutout') for v in validate_placement(q,d.source_state(),require_regions=False).violations)


def test_bridge_barrier_must_stay_on_interface(solved):
    _,p,r,_=solved
    pose=r.state.by_ref['ISO1'];t=p.bridges['ISO1']
    from pcb_hierplace.core.geometry import rotation
    delta=rotation(pose.angle)@np.asarray(t['axis_local'])*.01
    state=replace(r.state,poses=tuple(replace(x,x=x.x+delta[0],y=x.y+delta[1]) if x.ref=='ISO1' else x for x in r.state.poses))
    assert any(v.rule=='BRIDGE_BARRIER' for v in validate_placement(p,state).violations)


def test_area_growth_uses_44_square_mm_limit():
    p=tiny_problem(9,('HV','HV'));cfg=copy.deepcopy(p.cfg)
    cfg['optimization']['stage_c'].update(trust_radius_mm=10.,cluster_area_growth_ratio=.1)
    d=replace(p.design,components=tuple(replace(c,source_pose=Pose(c.ref,3+i*9,3+i*3)) for i,c in enumerate(p.design.components)))
    a=replace(p.assignments,clusters=(('A','one'),('B','one')))
    q=compile_constraints(d,a,cfg);reference=d.source_state()
    state=replace(reference,poses=tuple(replace(x,x=x.x+1.1) if x.ref=='B' else x for x in reference.poses))
    violations=validate_placement(q,state,require_regions=False,reference=reference).violations
    growth=next(v for v in violations if v.rule=='CLUSTER_GROWTH')
    assert growth.required==pytest.approx(44.)
    assert growth.actual==pytest.approx(44.4)


def test_multiboard_export_preserves_unselected_records(fixture_config,tmp_path):
    def second_board(s):
        return s+'{"type":"DOCHEAD"}||{"uuid":"other_board","docType":"PCB"}|\n'+\
            '{"type":"META","id":"meta","ticket":1}||{"title":"未选中板"}|\n'
    mutate_archive(fixture_config['input']['project'],second_board)
    arc=ProjectArchive(fixture_config['input']['project'])
    assert len(arc.boards())==2
    cfg=copy.deepcopy(fixture_config);cfg['input']['board_id']=None
    with pytest.raises(PlacementError) as e:load_problem(cfg)
    assert e.value.code=='BOARD_SELECTION_REQUIRED'
    fixture_config['optimization'].update(iterations_a=3,iterations_b=3,iterations_c=3,
        max_topology_candidates=3,max_ab_feedback_rounds=0)
    run(fixture_config,tmp_path/'run',progress=lambda _:None)
    p,r=load_run(tmp_path/'run');export_project(p,r,tmp_path/'result.epro2')
    after=ProjectArchive(tmp_path/'result.epro2')
    assert [x.inner for x in arc.of('other_board')]==[x.inner for x in after.of('other_board')]


def test_native_geometry_change_is_not_hidden_by_same_netlist(problem):
    c=problem.design.by_ref['Q1'];pad=replace(c.pads[0],polygon=rect((-5,-5,5,5)))
    changed=replace(problem.design,components=tuple(replace(x,pads=(pad,)+x.pads[1:]) if x.ref=='Q1' else x for x in problem.design.components))
    assert changed.nets==problem.design.nets
    with pytest.raises(PlacementError) as e:compare_geometry(problem.design,changed,problem.eps)
    assert e.value.code=='ROUNDTRIP_GEOMETRY'


def test_noninteractive_decisions_do_not_wait(fixture_config,tmp_path):
    assert main(['configure','--project',fixture_config['input']['project'],'--out',str(tmp_path/'rules.yaml')])==0
    assert main(['validate','--config',str(tmp_path/'rules.yaml')])==2


def test_relative_priority_budget_cannot_drift(solved):
    from pcb_hierplace.opt.ranking import choose
    _,p,r,_=solved
    cfg=copy.deepcopy(p.cfg);cfg['optimization']['higher_priority_relative_tolerance']=.01
    q=replace(p,cfg=cfg)
    base=replace(r,metrics={**r.metrics,'hpwl_weighted_mm':100.,'cluster_extent_area_mm2':100.},priority_reference=())
    current=choose(q,None,base)
    first=replace(base,metrics={**base.metrics,'hpwl_weighted_mm':100.9,'cluster_extent_area_mm2':90.})
    current=choose(q,current,first)
    assert current.metrics['cluster_extent_area_mm2']==90.
    second=replace(base,metrics={**base.metrics,'hpwl_weighted_mm':101.8,'cluster_extent_area_mm2':80.})
    assert choose(q,current,second) is current


def test_string_false_is_not_a_boolean_permission(fixture_config):
    fixture_config['rules']['component_rules']['JH']['allow_body_overhang']='false'
    with pytest.raises(PlacementError) as e:load_problem(fixture_config)
    assert e.value.code=='CONFIG_TYPE'
