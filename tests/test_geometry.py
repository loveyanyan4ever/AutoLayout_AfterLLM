import numpy as np
import pytest
from pcb_hierplace.core.geometry import rect,transform,bbox,signed_distance,circle_envelope
from pcb_hierplace.core.schema import Pose


@pytest.mark.parametrize('angle,expected',[(0,(10,20,14,22)),(90,(8,20,10,24)),(180,(6,18,10,20))])
def test_asymmetric_reference_point(angle,expected):
    assert bbox(transform(rect((0,0,4,2)),Pose('X',10,20,angle)))==pytest.approx(expected)


@pytest.mark.parametrize('dx,dy,expected',[(1.98,0,-.02),(2.02,0,.02),(3,3,2**.5),(0,0,-2.)])
def test_signed_distance_against_hand_calculation(dx,dy,expected):
    a=rect((-1,-1,1,1)); b=transform(a,Pose('b',dx,dy))
    assert signed_distance(a,b)==pytest.approx(expected)


def test_containment_penetration_not_intersection_width():
    assert signed_distance(rect((-3,-3,3,3)),rect((-1,-1,1,1)))==pytest.approx(-4.)


def test_diagonal_distance_independent_geometry():
    shapely=pytest.importorskip('shapely.geometry')
    a=transform(rect((0,0,4,2)),Pose('a',0,0,45))
    b=transform(rect((-1,-1,1,1)),Pose('b',7,5,23))
    assert signed_distance(a,b)==pytest.approx(shapely.Polygon(a).distance(shapely.Polygon(b)),abs=1e-9)


def test_circle_envelope_is_conservative():
    poly=circle_envelope(0,0,1)
    for angle in np.linspace(0,2*np.pi,200):
        direction=np.array([np.cos(angle),np.sin(angle)])
        assert np.max(np.asarray(poly)@direction)>=1-1e-12
