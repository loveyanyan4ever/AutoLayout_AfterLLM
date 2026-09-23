"""Float64 analytical derivatives. All smooth terms have finite-difference tests."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from scipy.special import logsumexp,softmax


@dataclass
class Objective:
    nets: list  # ([(variable index, x offset, y offset)], weight); -1 means constant
    length_scale: float
    compact: float = 0.
    anchor_offsets: np.ndarray | None = None
    anchor_weight: float = 0.
    soft_distances: list | None = None
    compact_indices: tuple[int, ...] | None = None
    compact_groups: tuple[tuple[int, ...], ...] = ()
    parent_compact_weight: float = 0.

    def evaluate(self,z,gamma):
        pos=np.asarray(z,dtype=np.float64).reshape(-1,2)
        g=np.zeros_like(pos); wire=0.
        weight_sum=max(sum(w for _,w in self.nets),1.)
        norm=self.length_scale*weight_sum
        for pins,w in self.nets:
            if len(pins)<2 or w==0: continue
            idx=np.array([p[0] for p in pins]); offsets=np.array([p[1:] for p in pins],float)
            q=offsets.copy(); mask=idx>=0; q[mask]+=pos[idx[mask]]
            for axis in (0,1):
                v=q[:,axis]/gamma
                wire+=w*gamma*(logsumexp(v)+logsumexp(-v))/norm
                d=w*(softmax(v)-softmax(-v))/norm
                np.add.at(g[:,axis],idx[mask],d[mask])
        shape=anchor=intent=parent_compact=0.
        compact_ids=np.arange(len(pos)) if self.compact_indices is None else np.asarray(self.compact_indices,dtype=int)
        if len(compact_ids) and self.compact:
            members=pos[compact_ids]
            d=members-members.mean(axis=0)
            shape=self.compact*float((d*d).sum())/(len(members)*self.length_scale**2)
            g[compact_ids]+=2*self.compact*d/(len(members)*self.length_scale**2)
        for group in self.compact_groups:
            if len(group)<2 or not self.parent_compact_weight: continue
            ids=np.asarray(group,dtype=int); members=pos[ids]
            d=members-members.mean(axis=0)
            norm=len(members)*self.length_scale**2
            parent_compact+=self.parent_compact_weight*float((d*d).sum())/norm
            g[ids]+=2*self.parent_compact_weight*d/norm
        if self.anchor_offsets is not None and len(pos):
            d=(pos-pos.mean(axis=0))-self.anchor_offsets
            anchor=self.anchor_weight*float((d*d).sum())/(len(pos)*self.length_scale**2)
            g+=2*self.anchor_weight*(d-d.mean(axis=0))/(len(pos)*self.length_scale**2)
        for a,b,target,weight in self.soft_distances or []:
            def get(p):
                i,x,y=p
                return (pos[i] if i>=0 else np.zeros(2))+[x,y]
            d=get(a)-get(b); length=np.sqrt(np.dot(d,d)+1e-16)
            error=max(0.,length-target)
            intent+=weight*error**2/self.length_scale**2
            grad=2*weight*error*d/length/self.length_scale**2
            if a[0]>=0: g[a[0]]+=grad
            if b[0]>=0: g[b[0]]-=grad
        parts={'wire':float(wire),'compact':float(shape),'anchor':float(anchor),'intent':float(intent),
               'parent_compact':float(parent_compact)}
        return sum(parts.values()),g.ravel(),parts


def exact_hpwl(nets,z):
    pos=np.asarray(z).reshape(-1,2); weighted=unweighted=0.
    for pins,w in nets:
        if len(pins)<2: continue
        q=np.array([[x,y]+(pos[i] if i>=0 else np.zeros(2)) for i,x,y in pins])
        value=float(np.ptp(q[:,0])+np.ptp(q[:,1]))
        weighted+=w*value; unweighted+=value
    return weighted,unweighted
