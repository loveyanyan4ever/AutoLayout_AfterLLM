import numpy as np
import pytest
from scipy import sparse
from pcb_hierplace.opt.objectives import Objective
from pcb_hierplace.opt.projected import Projector,optimize
from pcb_hierplace.core.schema import Budget


@pytest.mark.parametrize('mode',['wire','compact','anchor','distance','combined'])
def test_analytical_gradients(mode):
    nets=[([(0,.4,-.2),(1,.1,.6),(2,-.2,.7)],1.3),([(0,0,0),(-1,8,5)],.7)]
    anchor=np.array([[-1.,0.],[1.,1.],[0.,-1.]])
    obj=Objective(nets if mode in ('wire','combined') else [],20.,
                  compact=.7 if mode in ('compact','combined') else 0.,
                  anchor_offsets=anchor,anchor_weight=.5 if mode in ('anchor','combined') else 0.,
                  soft_distances=[((0,0,0),(1,.2,.1),.4,2.)] if mode in ('distance','combined') else [])
    z=np.array([1.,2.,4.,6.,8.,3.]); g=obj.evaluate(z,.8)[1]
    numeric=[]
    for k in range(len(z)):
        a=z.copy();b=z.copy();a[k]+=1e-5;b[k]-=1e-5
        numeric.append((obj.evaluate(a,.8)[0]-obj.evaluate(b,.8)[0])/2e-5)
    np.testing.assert_allclose(g,numeric,rtol=1e-4,atol=1e-8)


def test_projection_and_score_atomicity():
    obj=Objective([([(0,0.,0.),(1,0.,0.)],1.)],10.)
    A=np.vstack([np.eye(4),[-1,0,1,0]])
    proj=Projector(A,[0,0,0,0,2],[10,10,10,10,np.inf])
    result=optimize(obj,np.array([1.,1.,8.,1.]),proj,5,.2,2.,.1,Budget(5))
    assert result.z is not None
    assert result.z[2]-result.z[0]>=2-1e-7
    assert result.stats['final_loss']==pytest.approx(obj.evaluate(result.z,.1)[0],abs=1e-12)


def test_projection_rejects_infeasible_subproblem():
    proj=Projector([[1.],[1.]],[1.,-np.inf],[np.inf,0.])
    assert proj.project(np.array([0.])) is None


def test_rejected_steps_restore_all_state():
    obj=Objective([([(0,0.,0.),(-1,0.,0.)],1.)],10.)
    proj=Projector(np.eye(2),[0,0],[10,10]); z=np.array([5.,5.])
    result=optimize(obj,z,proj,3,.5,1.,1.,Budget(5),accept=lambda x:np.linalg.norm(x-z)<1e-8)
    assert result.stats['rejected']==3
    np.testing.assert_allclose(result.z,z,atol=1e-8)
