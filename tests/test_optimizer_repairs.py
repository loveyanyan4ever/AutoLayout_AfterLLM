"""Regression cases for candidate legality, hard-distance feasibility and objectives."""
from dataclasses import replace
import copy
import numpy as np
import pytest
from pcb_hierplace.config import DEFAULTS
from pcb_hierplace.core.schema import Pose, Pad, Component, Net, DesignSnapshot, Assignments, Budget
from pcb_hierplace.core.geometry import rect
from pcb_hierplace.constraints.compiler import compile_constraints
from pcb_hierplace.constraints.validator import validate_placement
from pcb_hierplace.placement.intra import solve_intra, local_valid
from pcb_hierplace.placement.preplace import preplace
from pcb_hierplace.placement.floorplan import solve_floorplan
from pcb_hierplace.placement.common import Block, objective_for
from pcb_hierplace.opt.projected import Projector, optimize


def small_problem(distance=10., one_cluster=False):
    components=[]
    for i,ref in enumerate(('A','B')):
        pad=Pad('pad','1',rect((-.5,-.5,.5,.5)),(0.,0.),'net'+ref)
        components.append(Component(ref,ref,'box',Pose(ref,4+i*distance,5),
                                    rect((-.5,-.5,.5,.5)),(pad,)))
    design=DesignSnapshot('b','','','synthetic','mm',(0,0),rect((0,0,20,20)),tuple(components),
                          tuple(Net('net'+c.ref,((c.ref,'pad'),)) for c in components))
    cfg=copy.deepcopy(DEFAULTS)
    cfg['board']['mode']='fixed'
    cfg['rules'].update(mechanical_gap_mm=.05,default_pad_clearance_mm=.05,
                        domains={'HV':{'kind':'electrical'}},isolation_pairs=[])
    cfg['optimization'].update(shape_aspects=[1.],iterations_a=4,iterations_b=4,iterations_c=4)
    clusters=(('A','c'),('B','c' if one_cluster else 'd'))
    assignments=Assignments((('A','HV'),('B','HV')),clusters,clusters)
    return compile_constraints(design,assignments,cfg)


def test_source_angles_outside_allowlist_are_repaired_before_stage_b():
    original=small_problem(2.,one_cluster=True)
    cfg=copy.deepcopy(original.cfg)
    cfg['rules']['component_rules']={'A':{'allowed_angles':[90.]},'B':{'allowed_angles':[0.]}}
    problem=compile_constraints(original.design,original.assignments,cfg)
    assert not local_valid(problem,tuple(c.source_pose for c in problem.design.components))
    shapes=solve_intra(problem,'c',Budget(10),0)
    assert shapes
    assert all({p.ref:p.angle for p in shape.members}=={'A':90.,'B':0.} for shape in shapes)
    pp=next(preplace(problem,problem.design.outline,Budget(10)))
    result=solve_floorplan(problem,{'c':shapes},pp,Budget(10),0,trials=1)
    assert result.best_feasible is not None
    assert validate_placement(problem,result.best_feasible.state).model_feasible
    assert original.design.by_ref['A'].source_pose.angle==0.


def test_stage_b_repairs_initial_hard_pin_distance():
    original=small_problem()
    cfg=copy.deepcopy(original.cfg)
    cfg['rules']['distance_constraints']=[{'a':'A:1','b':'B:1','hard':True,'max_mm':2.}]
    problem=compile_constraints(original.design,original.assignments,cfg)
    library={cid:solve_intra(problem,cid,Budget(10),0) for cid in problem.groups}
    pp=next(preplace(problem,problem.design.outline,Budget(10)))
    result=solve_floorplan(problem,library,pp,Budget(10),0,trials=1)
    assert result.best_feasible is not None
    state=result.best_feasible.state
    assert validate_placement(problem,state).model_feasible
    a,b=state.by_ref['A'],state.by_ref['B']
    assert np.hypot(a.x-b.x,a.y-b.y)<=2.+problem.eps
    assert result.stats['attempts'][0]['iterations']>0
    assert problem.cfg['rules']['distance_constraints'][0]['max_mm']==2.


def test_distance_projection_matches_offset_circle_solution():
    projector=Projector(np.eye(4),[0,0,-10,-10],[0,0,10,10],
                        distance_constraints=[(0,1,1.,-1.,2.)])
    actual=projector.project(np.array([0.,0.,4.,3.]))
    expected=2*np.array([5.,2.])/np.sqrt(29)-np.array([1.,-1.])
    assert actual is not None
    np.testing.assert_allclose(actual[2:],expected,atol=2e-7)
    assert np.linalg.norm(actual[2:]+[1,-1])<=2.+projector.eps


def test_distance_projection_handles_intersection_of_two_circles():
    projector=Projector(np.eye(6),[-10,-10,-1,0,1,0],[10,10,-1,0,1,0],
                        distance_constraints=[(0,1,0.,0.,1.5),(0,2,0.,0.,1.5)])
    actual=projector.project(np.array([0.,3.,-1.,0.,1.,0.]))
    assert actual is not None
    np.testing.assert_allclose(actual[:2],[0.,np.sqrt(1.25)],atol=3e-7)
    assert np.linalg.norm(actual[:2]-actual[2:4])<=1.5+projector.eps
    assert np.linalg.norm(actual[:2]-actual[4:])<=1.5+projector.eps


def test_impossible_hard_distance_is_not_relaxed():
    A=np.vstack((np.eye(4),[-1,0,1,0]))
    projector=Projector(A,[-10,-10,-10,-10,3.],[10,10,10,10,np.inf],
                        distance_constraints=[(0,1,0.,0.,2.)])
    assert projector.project(np.array([0.,0.,4.,0.])) is None
    rigid=Projector(np.eye(2),[-10,-10],[10,10],distance_constraints=[(0,0,3.,0.,2.)])
    assert rigid.project(np.zeros(2)) is None
    assert rigid.last_status=='rigid_distance_infeasible'


def test_zero_distance_is_exact_pin_alignment():
    projector=Projector(np.eye(4),[0,0,-10,-10],[0,0,10,10],
                        distance_constraints=[(0,1,2.,-1.,0.)])
    actual=projector.project(np.array([0.,0.,4.,3.]))
    assert actual is not None
    np.testing.assert_allclose(actual,[0.,0.,-2.,1.],atol=projector.eps)


def test_cluster_compactness_ignores_obstacle_variables_and_translation():
    problem=small_problem(2.,one_cluster=True)
    members=[Block(ref,(Pose(ref,0,0),),(-.5,-.5,.5,.5),'HV',(-20,-20,20,20),(2*i,0))
             for i,ref in enumerate(('A','B'))]
    obstacle=Block('obstacle:far',(),(100,100,101,101),'MECHANICAL',(0,0,0,0),(0,0),True)
    plain=objective_for(problem,members,20.,compact=.4)
    with_obstacle=objective_for(problem,members+[obstacle],20.,compact=.4)
    value0,gradient0,_=plain.evaluate(np.array([0.,0.,2.,0.]),1.)
    value1,gradient1,_=with_obstacle.evaluate(np.array([0.,0.,2.,0.,0.,0.]),1.)
    value2,gradient2,_=with_obstacle.evaluate(np.array([10.,0.,12.,0.,0.,0.]),1.)
    assert value0==pytest.approx(value1,abs=1e-15)
    assert value1==pytest.approx(value2,abs=1e-15)
    np.testing.assert_allclose(gradient0,gradient1[:4],atol=1e-15)
    np.testing.assert_allclose(gradient1,gradient2,atol=1e-15)
    np.testing.assert_array_equal(gradient1[4:],np.zeros(2))


def test_adam_accept_reject_accept_preserves_moments_and_step_count():
    class Quadratic:
        length_scale=1.
        def evaluate(self,z,gamma):
            d=z-np.array([0.,1.])
            return float(d@d),2*d,{'quadratic':float(d@d)}
    class ControlledProjector:
        last_status='controlled'; projections=0
        def project(self,z):
            self.projections+=1
            # Initial projection, first accepted step, eight rejected trials,
            # then one accepted step.  The outer learning-rate schedule stays
            # identical to the implementation under test.
            return None if 3<=self.projections<=10 else np.asarray(z).copy()
    obj=Quadratic(); z0=np.array([2.,4.]); lr=.4
    g0=obj.evaluate(z0,1.)[1]; m1=.1*g0; v1=.001*g0*g0
    z1=z0-lr*(m1/.1)/(np.sqrt(v1/.001)+1e-8)
    g1=obj.evaluate(z1,1.)[1]; m2=.9*m1+.1*g1; v2=.999*v1+.001*g1*g1
    expected=z1-lr*.1*(m2/(1-.9**2))/(np.sqrt(v2/(1-.999**2))+1e-8)
    result=optimize(obj,z0,ControlledProjector(),3,lr,1.,1.,Budget(10))
    assert [row['accepted'] for row in result.stats['history']]==[True,False,True]
    assert result.stats['rejected']==1
    np.testing.assert_allclose(result.z,expected,rtol=0,atol=1e-12)


def test_parent_cluster_compactness_and_gradient():
    problem=small_problem()
    parented=replace(problem,assignments=replace(problem.assignments,parent_clusters=(('c','power'),('d','power'))))
    blocks=[Block('cluster:'+cid,(Pose(ref,0,0),),(-.5,-.5,.5,.5),'HV',(-20,-20,20,20),(0,0))
            for cid,ref in [('c','A'),('d','B')]]
    obstacle=Block('obstacle:far',(),(100,100,101,101),'MECHANICAL',(0,0,0,0),(0,0),True)
    plain=objective_for(problem,blocks,20.)
    grouped=objective_for(parented,blocks+[obstacle],20.)
    z=np.array([1.,2.,6.,7.,0.,0.])
    assert plain.evaluate(z[:4],1.)[2]['parent_compact']==0.
    value,gradient,parts=grouped.evaluate(z,1.)
    assert parts['parent_compact']>0 and grouped.compact_groups==((0,1),)
    moved=z.copy();moved[:4]+=10
    assert grouped.evaluate(moved,1.)[0]==pytest.approx(value,abs=1e-15)
    numerical=[]
    for k in range(len(z)):
        a=z.copy();b=z.copy();a[k]+=1e-5;b[k]-=1e-5
        numerical.append((grouped.evaluate(a,1.)[0]-grouped.evaluate(b,1.)[0])/2e-5)
    np.testing.assert_allclose(gradient,numerical,rtol=1e-5,atol=1e-10)
    np.testing.assert_array_equal(gradient[4:],np.zeros(2))


def test_distance_cut_cache_never_waives_original_circles():
    projector=Projector(np.eye(6),[-10,-10,-1,0,1,0],[10,10,-1,0,1,0],
                        distance_constraints=[(0,1,0.,0.,1.5),(0,2,0.,0.,1.5)])
    for theta in np.linspace(0,2*np.pi,12,endpoint=False):
        target=np.array([5*np.cos(theta),5*np.sin(theta),-1.,0.,1.,0.])
        actual=projector.project(target)
        assert actual is not None, projector.last_status
        assert np.linalg.norm(actual[:2]-[-1.,0.])<=1.5+2*projector.eps
        assert np.linalg.norm(actual[:2]-[1.,0.])<=1.5+2*projector.eps


@pytest.mark.parametrize('other_geometry',['body','pad','hole'])
def test_local_holes_reject_other_footprint_occupancy(other_geometry):
    from pcb_hierplace.core.geometry import circle_envelope
    original=small_problem()
    a,b=original.design.components
    apad=replace(a.pads[0],hole_mm=1.,hole_center=(5.,0.),hole_polygon=circle_envelope(5.,0.,.5))
    a=replace(a,pads=(apad,))
    if other_geometry=='body':
        b=replace(b,source_pose=replace(b.source_pose,x=9.))
    elif other_geometry=='pad':
        b=replace(b,pads=(replace(b.pads[0],polygon=rect((-5.5,-.5,-4.5,.5)),center=(-5.,0.)),))
    else:
        b=replace(b,pads=(replace(b.pads[0],hole_mm=1.,hole_center=(-5.,0.),
                                  hole_polygon=circle_envelope(-5.,0.,.5)),))
    design=replace(original.design,components=(a,b))
    problem=compile_constraints(design,original.assignments,original.cfg)
    assert not local_valid(problem,tuple(c.source_pose for c in design.components))
    violations=validate_placement(problem,design.source_state(),require_regions=False).violations
    assert any(v.rule=='HOLE_KEEPOUT' for v in violations)


def test_local_metal_body_with_pads_is_still_a_conductor():
    original=small_problem(3.)
    a,b=original.design.components
    a=replace(a,body=rect((-2.,-.5,2.,.5)),
              pads=(replace(a.pads[0],polygon=rect((-1.9,-.1,-1.8,.1)),center=(-1.85,0.)),))
    design=replace(original.design,components=(a,b))
    cfg=copy.deepcopy(original.cfg)
    cfg['rules']['default_pad_clearance_mm']=1.
    cfg['rules']['component_rules']['A']={'conductive_class':'HV'}
    problem=compile_constraints(design,original.assignments,cfg)
    assert not local_valid(problem,tuple(c.source_pose for c in design.components))
    violations=validate_placement(problem,design.source_state(),require_regions=False).violations
    assert any(v.rule=='COPPER_CLEARANCE' and 'A:metal_body' in v.objects for v in violations)
    assert not any(v.rule=='MECHANICAL_GAP' for v in violations)


def test_local_broadphase_matches_independent_full_checker():
    from pcb_hierplace.core.geometry import circle_envelope
    original=small_problem()
    a,b=original.design.components
    a=replace(a,pads=(replace(a.pads[0],hole_mm=.6,hole_polygon=circle_envelope(.8,0.,.3)),))
    b=replace(b,pads=(replace(b.pads[0],polygon=rect((-.3,-.2,.9,.2)),center=(.3,0.)),))
    design=replace(original.design,components=(a,b),outline=rect((-100,-100,100,100)))
    cfg=copy.deepcopy(original.cfg)
    cfg['rules']['component_rules']={r:{'allowed_angles':[0.,45.,90.]} for r in ('A','B')}
    problem=compile_constraints(design,original.assignments,cfg)
    rng=np.random.default_rng(41)
    for _ in range(25):
        poses=(Pose('A',0.,0.,float(rng.choice([0.,45.,90.]))),
               Pose('B',float(rng.uniform(-3,3)),float(rng.uniform(-3,3)),float(rng.choice([0.,45.,90.]))))
        state=replace(design.source_state(),poses=poses)
        assert local_valid(problem,poses)==validate_placement(problem,state,require_regions=False).model_feasible


def test_local_broadphase_skips_distant_ellipse_pairs(monkeypatch):
    from pcb_hierplace.core.geometry import circle_envelope
    from pcb_hierplace.placement import intra
    original=small_problem()
    components=[]; domains=[]; groups=[]; nets=[]
    for i in range(50):
        ref=f'U{i}'
        pad=Pad('p','1',circle_envelope(0.,0.,.3),(0.,0.),'n'+ref,
                hole_mm=.2,hole_polygon=circle_envelope(.1,0.,.1))
        components.append(Component(ref,ref,'box',Pose(ref,(i%10)*4.,(i//10)*4.),
                                    rect((-.5,-.5,.5,.5)),(pad,)))
        domains.append((ref,'HV'));groups.append((ref,'large'));nets.append(Net('n'+ref,((ref,'p'),)))
    design=replace(original.design,components=tuple(components),nets=tuple(nets),outline=rect((-1,-1,40,20)))
    assignments=Assignments(tuple(domains),tuple(groups),tuple(groups))
    problem=compile_constraints(design,assignments,original.cfg)
    calls=[]
    exact=intra.signed_distance
    def counted(a,b):
        calls.append(1)
        return exact(a,b)
    monkeypatch.setattr(intra,'signed_distance',counted)
    assert local_valid(problem,tuple(c.source_pose for c in components))
    assert not calls  # Bounding-box lower bounds prove every relevant pair safe.
