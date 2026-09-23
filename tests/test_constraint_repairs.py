"""Regression cases from the read-only review, with independently set distances."""
import copy
from dataclasses import replace

import pytest

from pcb_hierplace.config import DEFAULTS
from pcb_hierplace.constraints.compiler import compile_constraints
from pcb_hierplace.constraints.validator import validate_placement,corner_matches,edge_band_matches
from pcb_hierplace.core.geometry import rect,bbox,transform
from pcb_hierplace.core.schema import (
    Assignments, Component, DesignSnapshot, Net, Pad, PlacementError, Pose,
)
from pcb_hierplace.io.cluster_csv import load_assignments, read_assignment


def _config(domains):
    cfg=copy.deepcopy(DEFAULTS)
    cfg['board']['mode']='fixed'
    cfg['rules'].update(mechanical_gap_mm=.05,default_pad_clearance_mm=1.,domains=domains)
    if 'HV' in domains and 'LV' in domains:
        cfg['rules']['isolation_pairs']=[{
            'domains':['HV','LV'],'copper_clearance_mm':3.,'creepage_mm':3.,'routing_reserve_mm':0.,
        }]
    return cfg


def _design(components):
    nets={}
    for c in components:
        for p in c.pads:
            if p.net:nets.setdefault(p.net,[]).append((c.ref,p.id))
    return DesignSnapshot('test','','','synthetic','mm',(0,0),rect((0,0,20,10)),tuple(components),
                          tuple(Net(name,tuple(pins)) for name,pins in nets.items()))


def _assignments(domains):
    groups=tuple((ref,'group_'+ref) for ref,_ in domains)
    return Assignments(tuple(domains),groups,groups)


@pytest.mark.parametrize('has_pad',[False,True])
@pytest.mark.parametrize('distance,valid',[(.5,False),(1.,True),(1.2,True)])
def test_conductive_body_clearance_independent_of_pad_presence(has_pad,distance,valid):
    own_pad=Pad('p','1',rect((-.1,-.1,.1,.1)),(0,0),'metal_contact')
    a=Component('A','A','fp',Pose('A',5,5),rect((-1,-1,1,1)),(own_pad,) if has_pad else ())
    # Metal right boundary is x=6; B pad left boundary is x=6+distance.
    b=Component('B','B','fp',Pose('B',6.1+distance,5),rect((-.1,-.1,.1,.1)),
                (Pad('p','1',rect((-.1,-.1,.1,.1)),(0,0),'other_net'),))
    d=_design([a,b]);cfg=_config({'HV':{'kind':'electrical'},'MECHANICAL':{'kind':'mechanical'}})
    cfg['rules']['component_rules']={'A':{'mode':'fixed_pose','conductive_class':'HV'},'B':{'mode':'fixed_pose'}}
    p=compile_constraints(d,_assignments([('A','MECHANICAL'),('B','HV')]),cfg)
    state=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),))
    report=validate_placement(p,state)
    assert report.model_feasible is valid
    if not valid:
        violation=next(v for v in report.violations if 'A:metal_body' in v.objects)
        assert violation.rule=='COPPER_CLEARANCE'
        assert violation.actual==pytest.approx(distance)
        assert violation.required==1.


@pytest.mark.parametrize('pad_domains',[{'1':'HV','2':'LV'},{'1':'LV','2':'LV'}])
def test_ordinary_domain_cannot_bypass_bridge_template(pad_domains):
    pads=(Pad('p1','1',rect((-.1,-.1,.1,.1)),(0,0),'nHV'),
          Pad('p2','2',rect((9.9,-.1,10.1,.1)),(10,0),'nLV'))
    d=_design([Component('A','A','fp',Pose('A',5,5),rect((-.5,-.5,.5,.5)),pads)])
    cfg=_config({'HV':{'kind':'electrical'},'LV':{'kind':'electrical'}})
    cfg['rules']['component_rules']['A']={'mode':'fixed_pose','pad_domains':pad_domains}
    with pytest.raises(PlacementError) as e:
        compile_constraints(d,_assignments([('A','HV')]),cfg)
    assert e.value.code=='BRIDGE_TEMPLATE_REQUIRED'


def test_explicit_complete_bridge_remains_valid():
    pads=(Pad('p1','1',rect((-5.1,-.1,-4.9,.1)),(-5,0),'nHV'),
          Pad('p2','2',rect((4.9,-.1,5.1,.1)),(5,0),'nLV'))
    d=_design([Component('A','A','fp',Pose('A',10,5),rect((-5.2,-.5,5.2,.5)),pads)])
    cfg=_config({'HV':{'kind':'electrical'},'LV':{'kind':'electrical'},
                 'BRIDGE':{'kind':'bridge','between':['HV','LV']}})
    cfg['rules']['component_rules']['A']={'mode':'fixed_pose','bridge_template':'iso'}
    cfg['rules']['bridge_templates']['iso']={
        'between':['HV','LV'],'axis_local':[1,0],'barrier_point_mm':[0,0],
        'pad_domains':{'1':'HV','2':'LV'},'allowed_angles':[0],
    }
    p=compile_constraints(d,_assignments([('A','BRIDGE')]),cfg)
    state=replace(d.source_state(),domain_regions=(('HV',(0,0,8.5,10)),('LV',(11.5,0,20,10))))
    assert validate_placement(p,state).model_feasible


def _csv_design():
    return _design([Component(r,r,'fp',Pose(r,2+i*5,5),rect((-.1,-.1,.1,.1)),())
                    for i,r in enumerate(['A','B','C'])])


@pytest.mark.parametrize('content,code',[
    ('designator,domain_id,domain_id\nA,HV,LV\nB,HV,LV\nC,HV,LV\n','CSV_HEADER'),
    ('designator,domain_id\nA,HV,LV\nB,HV\nC,HV\n','CSV_COLUMNS'),
    ('designator,domain_id,note\nA,HV\nB,HV,\nC,HV,\n','CSV_COLUMNS'),
])
def test_csv_rejects_ambiguous_header_and_row_width(tmp_path,content,code):
    path=tmp_path/'domains.csv';path.write_text(content)
    with pytest.raises(PlacementError) as e:read_assignment(path,'domain_id',_csv_design())
    assert e.value.code==code


def test_csv_keeps_physical_line_number_and_quoted_multiline_note(tmp_path):
    path=tmp_path/'domains.csv'
    path.write_text('# comment\n\ndesignator,domain_id,note\nA,HV,"first\n# part of note"\nB,HV,ok\nC,HV,ok\n')
    assert read_assignment(path,'domain_id',_csv_design())=={'A':'HV','B':'HV','C':'HV'}
    path.write_text('# comment\n\ndesignator,domain_id\n\nA,HV,extra\n')
    with pytest.raises(PlacementError) as e:read_assignment(path,'domain_id',_csv_design())
    assert e.value.code=='CSV_COLUMNS'
    assert e.value.details['line']==5


def test_split_cluster_id_cannot_merge_existing_cluster(tmp_path):
    voltage=tmp_path/'v.csv';voltage.write_text('designator,domain_id\nA,HV\nB,LV\nC,HV\n')
    functional=tmp_path/'f.csv';functional.write_text('designator,cluster_id\nA,x\nB,x\nC,x::HV\n')
    cfg=_config({'HV':{'kind':'electrical'},'LV':{'kind':'electrical'}})
    cfg['clusters']['cross_domain_policy']='split_by_domain'
    with pytest.raises(PlacementError) as e:load_assignments(_csv_design(),voltage,functional,cfg)
    assert e.value.code=='CLUSTER_ID_COLLISION'
    functional.write_text('designator,cluster_id\nA,x\nB,x\nC,z\n')
    a=load_assignments(_csv_design(),voltage,functional,cfg)
    assert dict(a.clusters)=={'A':'x::HV','B':'x::LV','C':'z'}
    assert dict(a.logical_clusters)=={'A':'x','B':'x','C':'z'}


@pytest.mark.parametrize('field,rule',[
    ('outline','REFINE_OUTLINE'),
    ('domain_regions','REFINE_DOMAIN_REGIONS'),
    ('regions','REFINE_CLUSTER_REGIONS'),
])
def test_refine_checks_frozen_structures_even_when_changed_state_is_feasible(field,rule):
    a=Component('A','A','fp',Pose('A',5,5),rect((-.5,-.5,.5,.5)),())
    d=_design([a]);cfg=_config({'HV':{'kind':'electrical'}})
    cfg['board']['mode']='estimated';cfg['board']['replace_existing_outline']=True
    cfg['board']['estimated'].update(max_width_mm=30,max_height_mm=30)
    p=compile_constraints(d,_assignments([('A','HV')]),cfg)
    reference=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),),regions=(('group_A',(4,4,6,6)),))
    changes={'outline':rect((0,0,20,11)),
             'domain_regions':(('HV',(0,0,20,9)),),
             'regions':(('group_A',(3.9,4,6,6)),)}
    state=replace(reference,**{field:changes[field]})
    assert validate_placement(p,state).model_feasible
    report=validate_placement(p,state,reference=reference)
    assert not report.model_feasible
    assert rule in {v.rule for v in report.violations}
    assert validate_placement(p,reference,reference=reference).model_feasible


def test_refine_special_pose_is_frozen_even_within_trust_radius():
    a=Component('A','A','fp',Pose('A',5,5),rect((-.5,-.5,.5,.5)),())
    d=_design([a]);p=compile_constraints(d,_assignments([('A','HV')]),_config({'HV':{'kind':'electrical'}}))
    # A selected mechanical candidate must be fixed during C, independently of
    # its source locking policy or the solver's own choice of variables.
    p=replace(p,special=('A',),groups={},ref_group={})
    reference=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),))
    state=replace(reference,poses=(replace(a.source_pose,x=5.1),))
    assert validate_placement(p,state).model_feasible
    report=validate_placement(p,state,reference=reference)
    assert 'REFINE_FIXED_OBJECT' in {v.rule for v in report.violations}
    assert 'TRUST_RADIUS' not in {v.rule for v in report.violations}


def test_corner_template_respects_rotated_complete_occupancy():
    c=Component('H','H','fp',Pose('H',2,2,90),rect((-1,-2,3,1)),())
    d=_design([c]);cfg=_config({'HV':{'kind':'electrical'},'M':{'kind':'mechanical'}})
    rule={'mode':'corner','allowed_corners':['bottom_left'],'corner_inset_mm':1.,'conductive_class':'insulating'}
    cfg['rules']['component_rules']['H']=rule
    p=compile_constraints(d,_assignments([('H','M')]),cfg)
    assert p.special==('H',) and not p.fixed and not p.groups
    assert corner_matches(rule,c.source_pose,(0,0,20,10),p.occupancy('H'),p.eps)
    assert not corner_matches(rule,replace(c.source_pose,x=2.1),(0,0,20,10),p.occupancy('H'),p.eps)
    st=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),))
    assert validate_placement(p,st).model_feasible
    moved=replace(st,poses=(replace(c.source_pose,x=2.1),))
    assert 'CORNER' in {v.rule for v in validate_placement(p,moved).violations}


def test_edge_band_uses_pad_extent_and_preserves_source_angle():
    c=Component('J','J','fp',Pose('J',16,5),rect((-1,-1,1,1)),
                (Pad('p','1',rect((2,-.5,3,.5)),(2.5,0),'net'),))
    d=_design([c]);cfg=_config({'HV':{'kind':'electrical'}})
    rule={'mode':'edge_band','allowed_edges':['right'],'edge_offset_mm':1.,'allowed_angles':[0,90]}
    cfg['rules']['component_rules']['J']=rule
    p=compile_constraints(d,_assignments([('J','HV')]),cfg)
    assert p.special==('J',) and p.angles['J']==(0.,)
    assert edge_band_matches(rule,c.source_pose,(0,0,20,10),p.occupancy('J'),p.eps)
    assert not edge_band_matches(rule,replace(c.source_pose,x=18),(0,0,20,10),p.occupancy('J'),p.eps)
    state=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),))
    assert validate_placement(p,state).model_feasible
    turned=replace(state,poses=(replace(c.source_pose,angle=90),))
    assert 'EDGE_BAND_ANGLE' in {v.rule for v in validate_placement(p,turned).violations}
    rule['allowed_angles']=[90]
    with pytest.raises(PlacementError) as e:compile_constraints(d,_assignments([('J','HV')]),cfg)
    assert e.value.code=='EDGE_ANGLE'


def test_offset_drill_is_independent_of_copper_and_blocks_other_bodies():
    pad=Pad('p','1',rect((-2.5,-.5,-1.5,.5)),(-2,0),'net',hole_mm=1.,
            hole_polygon=rect((1.5,-.5,2.5,.5)),hole_center=(2,0))
    a=Component('A','A','fp',Pose('A',5,5),rect((-.25,-.25,.25,.25)),(pad,))
    b=Component('B','B','fp',Pose('B',7.6,5),rect((-.05,-.05,.05,.05)),())
    d=_design([a,b]);cfg=_config({'HV':{'kind':'electrical'}})
    cfg['rules']['mechanical_gap_mm']=.2
    cfg['rules']['component_rules']={r:{'mode':'fixed_pose'} for r in ('A','B')}
    p=compile_constraints(d,_assignments([('A','HV'),('B','HV')]),cfg)
    assert bbox(p.occupancy('A'))==(-2.5,-.5,2.5,.5)
    assert p.design.by_ref['A'].pads[0].center==(-2,0)  # Drill offset must not move copper.
    state=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),))
    report=validate_placement(p,state)
    v=next(v for v in report.violations if v.rule=='HOLE_KEEPOUT')
    assert v.actual==pytest.approx(.05)
    assert not any(v.rule=='MECHANICAL_GAP' for v in report.violations)
    # Move the other part clear, then isolate hole-only board/obstacle checks.
    b=replace(b,source_pose=replace(b.source_pose,y=8))
    d=_design([a,b]);p=compile_constraints(d,_assignments([('A','HV'),('B','HV')]),cfg)
    state=replace(d.source_state(),domain_regions=(('HV',(0,0,20,10)),))
    assert validate_placement(p,state).model_feasible
    outside=replace(state,poses=(replace(a.source_pose,x=18),b.source_pose))
    assert 'BOARD_HOLE' in {v.rule for v in validate_placement(p,outside).violations}
    cfg['rules']['obstacles']=[{'id':'drill_only','bbox_mm':[7.2,4.8,7.3,5.2]}]
    p=compile_constraints(d,_assignments([('A','HV'),('B','HV')]),cfg)
    assert 'OBSTACLE' in {v.rule for v in validate_placement(p,state).violations}


def test_parent_function_groups_survive_physical_split_without_becoming_domains(tmp_path):
    voltage=tmp_path/'v.csv';voltage.write_text('designator,domain_id\nA,HV\nB,LV\nC,HV\n')
    functional=tmp_path/'f.csv'
    functional.write_text('designator,cluster_id,parent_cluster_id\nA,x,P\nB,x,P\nC,z,Q\n')
    cfg=_config({'HV':{'kind':'electrical'},'LV':{'kind':'electrical'}})
    cfg['clusters']['cross_domain_policy']='split_by_domain'
    a=load_assignments(_csv_design(),voltage,functional,cfg)
    assert dict(a.parent_clusters)=={'x::HV':'P','x::LV':'P','z':'Q'}
    assert dict(a.domains)=={'A':'HV','B':'LV','C':'HV'}
    functional.write_text('designator,cluster_id,parent_cluster_id\nA,x,P\nB,x,Q\nC,z,Q\n')
    with pytest.raises(PlacementError) as e:load_assignments(_csv_design(),voltage,functional,cfg)
    assert e.value.code=='PARENT_CLUSTER_CONFLICT'


@pytest.mark.parametrize('overlap',[False,True])
def test_broadphase_matches_brute_force_clearance_and_minimum(monkeypatch,overlap):
    import pcb_hierplace.constraints.validator as validator
    components=[]
    for i in range(12):
        x=1.5+(i%6)*3.;y=2.+(i//6)*5.
        if overlap and i==1:x=1.6
        ref=f'C{i}'
        pad=Pad('p','1',rect((-.4,-.15,.4,.15)),(0,0),'net'+ref)
        components.append(Component(ref,ref,'fp',Pose(ref,x,y,(i%3)*30.),rect((-.5,-.4,.5,.4)),(pad,)))
    d=_design(components);cfg=_config({'HV':{'kind':'electrical'},'LV':{'kind':'electrical'}})
    cfg['rules']['mechanical_gap_mm']=.15
    p=compile_constraints(d,_assignments([(c.ref,'HV' if i%2==0 else 'LV') for i,c in enumerate(components)]),cfg)
    accelerated=validator.validate_placement(p,d.source_state(),require_regions=False).as_dict()
    monkeypatch.setattr(validator,'_bbox_separation',lambda a,b:0.)
    brute=validator.validate_placement(p,d.source_state(),require_regions=False).as_dict()
    assert accelerated==brute


@pytest.mark.parametrize('kind,fields',[
    ('distance_constraints',{'hard':True,'max_mm':10.}),
    ('distance_constraints',{'hard':False,'target_mm':2.}),
    ('order_constraints',{'axis':'x','hard':True,'min_separation_mm':0.}),
])
def test_number_based_rule_rejects_ambiguous_pads_but_keeps_imported_geometry(kind,fields):
    pads=(Pad('p1a','1',rect((-.6,-.1,-.4,.1)),(-.5,0),'nA'),
          Pad('p1b','1',rect((.4,-.1,.6,.1)),(.5,0),'nA'),
          Pad('p2','2',rect((-.1,.4,.1,.6)),(0,.5),'nA2'))
    a=Component('A','A','fp',Pose('A',5,5),rect((-1,-1,1,1)),pads)
    b=Component('B','B','fp',Pose('B',10,5),rect((-.5,-.5,.5,.5)),
                (Pad('p1','1',rect((-.1,-.1,.1,.1)),(0,0),'nB'),))
    d=_design([a,b]);assignments=_assignments([('A','HV'),('B','HV')])
    cfg=_config({'HV':{'kind':'electrical'}})
    ordinary=compile_constraints(d,assignments,cfg)
    assert [p.id for p in ordinary.design.by_ref['A'].pads]==['p1a','p1b','p2']
    # A unique number is still usable in a footprint containing other repeated numbers.
    cfg['rules'][kind]=[{'a':'A:2','b':'B:1',**fields}]
    assert compile_constraints(d,assignments,cfg).design.nets==d.nets
    cfg['rules'][kind][0]['a']='A:1'
    with pytest.raises(PlacementError) as e:compile_constraints(d,assignments,cfg)
    assert e.value.code=='PIN_REFERENCE_AMBIGUOUS'
    assert e.value.details=={'endpoint':'A:1','pad_ids':['p1a','p1b']}
