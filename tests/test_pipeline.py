from dataclasses import replace
import json
import pytest
from pcb_hierplace.core.schema import Pose,PlacementError
from pcb_hierplace.constraints.validator import validate_placement
from pcb_hierplace.core.geometry import bbox
from pcb_hierplace.io.export import export_project
from pcb_hierplace.opt.ranking import evaluate,choose
from pcb_hierplace.cli import main


def test_full_dual_domain_pipeline(solved):
    root,p,r,report=solved
    assert report['status']=='FEASIBLE'
    assert r.validation.model_feasible
    assert r.state.outline==p.design.outline
    assert r.validation.min_copper_distance_mm>=3-p.eps
    assert r.validation.creepage_status=='NOT_EVALUATED'
    assert report['native_open_validation']=='PENDING'
    assert r.metrics['hpwl_weighted_mm']<=report['before_C']['hpwl_weighted_mm']+1e-7
    for ref in p.fixed:
        a=r.state.by_ref[ref];b=p.design.by_ref[ref].source_pose
        assert a.x==pytest.approx(b.x,abs=1e-7)
        assert a.y==pytest.approx(b.y,abs=1e-7)
        assert a.angle==b.angle


def test_independent_recheck_after_invalid_move(solved):
    _,p,r,_=solved
    poses=tuple(replace(x,x=-1.) if x.ref=='Q1' else x for x in r.state.poses)
    v=validate_placement(p,replace(r.state,poses=poses))
    assert not v.model_feasible
    assert any(x.rule=='BOARD_BODY' for x in v.violations)


def test_edge_direction_cannot_be_traded_for_wirelength(solved):
    _,p,r,_=solved
    state=replace(r.state,poses=tuple(replace(x,angle=180.) if x.ref=='JH' else x for x in r.state.poses))
    bad=evaluate(p,state)
    assert any(v.rule=='EDGE' for v in bad.validation.violations)
    assert choose(p,r,bad) is r


def test_bridge_swapped_sides_rejected(solved):
    _,p,r,_=solved
    state=replace(r.state,poses=tuple(replace(x,angle=(x.angle+180)%360) if x.ref=='ISO1' else x for x in r.state.poses))
    assert any(v.rule=='PAD_DOMAIN' for v in validate_placement(p,state).violations)


def test_cluster_region_checks_whole_body(solved):
    _,p,r,_=solved
    poses=tuple(replace(x,x=dict(r.state.regions)[p.ref_group['C1']][2]) if x.ref=='C1' else x for x in r.state.poses)
    assert any(v.rule=='CLUSTER_REGION' for v in validate_placement(p,replace(r.state,poses=poses)).violations)


def test_export_roundtrip_and_source_unchanged(solved,tmp_path):
    _,p,r,_=solved
    from pcb_hierplace.core.schema import file_hash
    before=file_hash(p.design.source_path)
    report=export_project(p,r,tmp_path/'placed.epro2')
    assert report['roundtrip_validation']=='PASSED'
    assert report['native_open_validation']=='PENDING'
    assert before==file_hash(p.design.source_path)


def test_invalid_export_refused(solved,tmp_path):
    _,p,r,_=solved
    state=replace(r.state,poses=tuple(replace(x,x=-5.) if x.ref=='Q1' else x for x in r.state.poses))
    with pytest.raises(PlacementError):export_project(p,evaluate(p,state),tmp_path/'bad.epro2')
    assert not (tmp_path/'bad.epro2').exists()


def test_fixed_outline_change_rejected(solved):
    _,p,r,_=solved
    state=replace(r.state,outline=((0,0),(61,0),(61,40),(0,40)))
    assert any(v.rule=='FIXED_OUTLINE' for v in validate_placement(p,state).violations)


def test_cli_evaluate_revalidates(solved):
    root,_,_,_=solved
    assert main(['evaluate','--run',str(root/'run')])==0
