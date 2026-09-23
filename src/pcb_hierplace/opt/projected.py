"""Projected Adam/GD with transactional state and fixed-evaluator selection."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy import sparse
import osqp
from ..core.schema import PlacementError, Status


class Projector:
    def __init__(self,A,lower,upper,eps=1e-7,distance_constraints=()):
        self.A=sparse.csc_matrix(A); self.lower=np.asarray(lower,float); self.upper=np.asarray(upper,float)
        self.eps=eps; self.n=self.A.shape[1]
        # Each entry means ||p_j-p_i+(dx,dy)||_2 <= radius.  Outer
        # supporting halfspaces preserve the exact radius; a QP result is
        # returned only after checking every original Euclidean constraint.
        self.distances=tuple(distance_constraints)
        self._constant_infeasible=False; self._cut_keys=set(); self._cuts=[]
        equalities=[]; values=[]
        for i,j,dx,dy,radius in self.distances:
            if i==j:
                self._constant_infeasible |= np.hypot(dx,dy)>radius+eps
            elif radius==0:
                for axis,offset in enumerate((dx,dy)):
                    row=np.zeros(self.n); row[2*j+axis]=1.; row[2*i+axis]=-1.
                    equalities.append(row); values.append(-offset)
        if equalities:
            self.A=sparse.vstack((self.A,sparse.csc_matrix(equalities)),format='csc')
            self.lower=np.concatenate((self.lower,values)); self.upper=np.concatenate((self.upper,values))
        self._base_A=self.A; self._base_lower=self.lower; self._base_upper=self.upper
        self.last_status='unstarted'; self.projections=0; self.distance_cuts=0
        self.distance_inner_fallbacks=0
        self._setup()

    def _setup(self):
        self.solver=osqp.OSQP()
        self.solver.setup(P=sparse.eye(self.n,format='csc'),q=np.zeros(self.n),A=self.A,
            l=self.lower,u=self.upper,verbose=False,eps_abs=self.eps*.1 if self.distances else min(self.eps*.05,1e-9),eps_rel=1e-10,
            max_iter=12000,polishing=False,adaptive_rho=bool(self.distances),
            adaptive_rho_interval=50,check_termination=25)

    def _inner_projection(self,target):
        """Numerical fallback to a declared conservative, inscribed 64-gon.

        The circumradius is the actual hard limit (maximum radial loss 0.121%).
        This is a feasible subset, never an enlargement of a distance circle.
        """
        if not self.distances: return None
        rows=[]; bounds=[]; count=64
        for i,j,dx,dy,radius in self.distances:
            if i==j or radius==0: continue
            for angle in (np.arange(count)+.5)*(2*np.pi/count):
                normal=np.array([np.cos(angle),np.sin(angle)])
                row=np.zeros(self.n); row[2*j:2*j+2]+=normal; row[2*i:2*i+2]-=normal
                rows.append(row); bounds.append(radius*np.cos(np.pi/count)-normal@np.array([dx,dy]))
        A=sparse.vstack((self._base_A,sparse.csc_matrix(rows)),format='csc') if rows else self._base_A
        lower=np.concatenate((self._base_lower,np.full(len(rows),-np.inf)))
        upper=np.concatenate((self._base_upper,bounds))
        fallback=Projector(A,lower,upper,eps=self.eps)
        candidate=fallback.project(target)
        self.projections+=fallback.projections; self.distance_inner_fallbacks+=1
        if candidate is not None:
            pos=candidate.reshape(-1,2)
            if all(np.linalg.norm(pos[j]-pos[i]+[dx,dy])<=radius+self.eps
                   for i,j,dx,dy,radius in self.distances):
                self.last_status='solved_conservative_distance_polygon'; return candidate
        self.last_status='distance_fallback_'+fallback.last_status
        return None

    def project(self,z):
        if not np.isfinite(z).all(): return None
        if self._constant_infeasible:
            self.last_status='rigid_distance_infeasible'; return None
        target=np.asarray(z,float); warm=target
        for _ in range(24):
            self.solver.update(q=-target)
            self.solver.warm_start(x=warm,y=np.zeros(len(self.lower)))
            result=self.solver.solve(raise_error=False)
            self.last_status=result.info.status; self.projections+=1
            if result.info.status_val not in (1,2) or result.x is None or not np.isfinite(result.x).all():
                return self._inner_projection(target)
            r=self.A@result.x
            if np.max(np.maximum(self.lower-r,0),initial=0)>self.eps or np.max(np.maximum(r-self.upper,0),initial=0)>self.eps:
                self.last_status='residual_rejected'; return self._inner_projection(target)
            candidate=np.asarray(result.x).copy(); pos=candidate.reshape(-1,2); pending=[]
            for k,(i,j,dx,dy,radius) in enumerate(self.distances):
                delta=pos[j]-pos[i]+[dx,dy]; length=float(np.linalg.norm(delta))
                if length<=radius+self.eps: continue
                if radius==0 or i==j:
                    self.last_status='distance_residual_rejected'; return None
                normal=delta/length; key=(k,*np.round(normal,12))
                if key in self._cut_keys: continue
                row=np.zeros(self.n); row[2*j:2*j+2]+=normal; row[2*i:2*i+2]-=normal
                pending.append((key,row,float(radius-normal@np.array([dx,dy]))))
            if all(np.linalg.norm(pos[j]-pos[i]+[dx,dy])<=radius+self.eps
                   for i,j,dx,dy,radius in self.distances): return candidate
            if not pending:
                self.last_status='distance_cut_stalled'; return self._inner_projection(target)
            for item in pending:
                key=item[0]
                # Nearly parallel tangents make the QP unnecessarily ill
                # conditioned.  Retain only two nearby tangents;
                # the original circle is still checked before every return.
                neighbors=[old for old in self._cuts if old[0][0]==key[0]
                           and np.dot(old[0][1:],key[1:])>=.999]
                remove_keys={old[0] for old in neighbors[:-1]}
                self._cuts=[old for old in self._cuts if old[0] not in remove_keys]
                self._cuts.append(item)
            self.distance_cuts+=len(pending)
            # Old cuts are valid but only accelerate the next projection.  A
            # bounded cache avoids matrix growth across long gradient runs.
            self._cuts=self._cuts[-256:]; self._cut_keys={x[0] for x in self._cuts}
            self.A=sparse.vstack((self._base_A,sparse.csc_matrix([x[1] for x in self._cuts])),format='csc')
            self.lower=np.concatenate((self._base_lower,np.full(len(self._cuts),-np.inf)))
            self.upper=np.concatenate((self._base_upper,[x[2] for x in self._cuts]))
            self._setup(); warm=candidate
        self.last_status='distance_projection_budget'; return self._inner_projection(target)


@dataclass
class OptimizationResult:
    z: np.ndarray | None
    stats: dict


def optimize(objective,z0,projector,iterations,lr,gamma_start,gamma_end,budget,
             accept=None,score=None,optimizer='projected_adam'):
    z=projector.project(z0)
    if z is None: return OptimizationResult(None,{'reason':'projection_failed','solver_status':projector.last_status})
    if accept is not None and not accept(z):
        return OptimizationResult(None,{'reason':'initial_exact_validation_failed'})
    score=score or (lambda x: objective.evaluate(x,gamma_end)[0])
    best=z.copy(); best_score=score(best); m=np.zeros_like(z); v=np.zeros_like(z); t=0
    history=[]; rejected=0; numerical=0; evaluations=0
    for it in range(iterations):
        if budget.expired: break
        frac=it/max(iterations-1,1)
        gamma=gamma_start*(gamma_end/gamma_start)**min(1.,frac/.65)
        value,grad,parts=objective.evaluate(z,gamma)
        evaluations+=1
        if not np.isfinite(value) or not np.isfinite(grad).all(): numerical+=1; break
        norm=np.linalg.norm(grad)
        if norm>10: grad*=10/norm
        new_m=.9*m+.1*grad; new_v=.999*v+.001*grad**2; nt=t+1
        direction=(new_m/(1-.9**nt))/(np.sqrt(new_v/(1-.999**nt))+1e-8) if optimizer=='projected_adam' else grad
        step=lr*(.1+.9*.5*(1+np.cos(np.pi*frac)))
        accepted=False
        for _ in range(8):
            trial=projector.project(z-step*direction)
            if trial is not None and (accept is None or accept(trial)):
                newvalue=objective.evaluate(trial,gamma)[0]
                # Backtracking keeps projected steps stable even when an active face changes.
                if np.isfinite(newvalue) and newvalue<=value+1e-10:
                    z=trial; m=new_m; v=new_v; t=nt; accepted=True; break
            step*=.5
        if not accepted:
            rejected+=1  # z, m, v, t all remain unchanged
        s=score(z)
        if s<best_score:
            best=z.copy(); best_score=s
        if it%max(1,iterations//8)==0 or it==iterations-1:
            cv,_,cp=objective.evaluate(z,gamma_end)
            history.append({'iteration':it,'gamma_search_mm':gamma,'canonical_loss':cv,'parts':cp,
                            'gradient_norm_before_step':float(norm),'accepted':accepted,
                            'reference_length_mm':objective.length_scale})
    final,_,parts=objective.evaluate(best,gamma_end)
    return OptimizationResult(best,{'reason':'budget' if budget.expired else 'completed',
        'iterations':evaluations,'rejected':rejected,'numerical_failures':numerical,'history':history,
        'final_loss':final,'final_parts':parts,'canonical_gamma_mm':gamma_end,
        'projection_calls':projector.projections,
        'distance_projection_cuts':getattr(projector,'distance_cuts',0),
        'distance_inner_polygon_fallbacks':getattr(projector,'distance_inner_fallbacks',0)})
