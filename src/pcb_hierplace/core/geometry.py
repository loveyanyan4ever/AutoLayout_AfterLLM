"""Convex geometry in physical mm. No optimizer loss is used for validation."""
from __future__ import annotations
import math
import numpy as np
from .schema import PlacementError, Status


def rect(box):
    x0, y0, x1, y1 = map(float, box)
    if not all(math.isfinite(x) for x in box) or x1 <= x0 or y1 <= y0:
        raise PlacementError("INVALID_BOX", f"无效矩形: {box}")
    return ((x0, y0), (x1, y0), (x1, y1), (x0, y1))


def rotation(angle):
    a = math.radians(angle)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def transform(poly, pose):
    a = np.asarray(poly, dtype=float) @ rotation(pose.angle).T
    return a + [pose.x, pose.y]


def rotate(poly, angle):
    return np.asarray(poly, dtype=float) @ rotation(angle).T


def bbox(poly):
    p = np.asarray(poly, dtype=float)
    if len(p) == 0 or not np.isfinite(p).all():
        raise PlacementError("INVALID_GEOMETRY", "空几何或非有限坐标")
    return (float(p[:, 0].min()), float(p[:, 1].min()),
            float(p[:, 0].max()), float(p[:, 1].max()))


def union_bbox(boxes):
    a = np.asarray(boxes)
    return (float(a[:, 0].min()), float(a[:, 1].min()),
            float(a[:, 2].max()), float(a[:, 3].max()))


def area(poly):
    p = np.asarray(poly)
    return float(abs(np.dot(p[:, 0], np.roll(p[:, 1], 1)) -
                     np.dot(p[:, 1], np.roll(p[:, 0], 1))) / 2)


def is_rectangle(poly, eps=1e-7):
    p = list(map(tuple, poly))
    if len(p) > 1 and np.linalg.norm(np.array(p[0])-p[-1]) < eps:
        p.pop()
    if len(p) != 4:
        return False
    b = bbox(p)
    corners = rect(b)
    return (all(any(np.linalg.norm(np.array(x)-y) < eps for y in corners) for x in p)
            and abs(area(p)-(b[2]-b[0])*(b[3]-b[1])) < eps)


def inside_box(poly, box, eps=0.):
    b = bbox(poly)
    return b[0] >= box[0]-eps and b[1] >= box[1]-eps and b[2] <= box[2]+eps and b[3] <= box[3]+eps


def circle_envelope(cx, cy, radius, sides=32):
    # Circumscribed, never an inscribed approximation that can miss collisions.
    r = radius / math.cos(math.pi/sides)
    return tuple((cx+r*math.cos(2*math.pi*k/sides), cy+r*math.sin(2*math.pi*k/sides))
                 for k in range(sides))


def _point_segment(p, a, b):
    d = b-a
    t = np.clip(np.dot(p-a, d)/max(float(np.dot(d,d)),1e-30),0,1)
    return float(np.linalg.norm(p-a-t*d))


def signed_distance(poly_a, poly_b):
    """Euclidean separation; negative SAT penetration when convex polygons overlap."""
    a,b = np.asarray(poly_a,float),np.asarray(poly_b,float)
    def axis_rect(p):
        return len(p)==4 and all(abs(p[i,0]-p[(i+1)%4,0])<1e-12 or
                                abs(p[i,1]-p[(i+1)%4,1])<1e-12 for i in range(4))
    if axis_rect(a) and axis_rect(b):
        aa=bbox(a); bb=bbox(b)
        dx=max(bb[0]-aa[2],aa[0]-bb[2],0.)
        dy=max(bb[1]-aa[3],aa[1]-bb[3],0.)
        if dx>0 or dy>0: return math.hypot(dx,dy)
        return -max(0.,min(aa[2]-bb[0],bb[2]-aa[0],aa[3]-bb[1],bb[3]-aa[1]))
    min_depth = math.inf
    separated = False
    for p in (a,b):
        for i in range(len(p)):
            d = p[(i+1)%len(p)]-p[i]
            if np.linalg.norm(d) < 1e-15:
                continue
            n = np.array([-d[1],d[0]])/np.linalg.norm(d)
            pa,pb = a@n,b@n
            if pa.max() < pb.min() or pb.max() < pa.min():
                separated = True
            # Minimum separating translation also handles complete containment.
            min_depth = min(min_depth, pa.max()-pb.min(), pb.max()-pa.min())
    if not separated:
        return -max(0., float(min_depth))
    return min(_point_segment(v,p[i],p[(i+1)%len(p)])
               for q,p in ((a,b),(b,a)) for v in q for i in range(len(p)))


def pad_world(pad, pose):
    return transform(pad.polygon, pose)


def pad_center(pad, pose):
    return rotation(pose.angle) @ np.asarray(pad.center) + [pose.x,pose.y]


def angle_equal(a,b,eps=1e-7):
    return abs((a-b+180)%360-180) <= eps


def validate_simple(poly):
    """Reject self-intersection before any convex-only distance algorithm is used.

    One closing copy of the first vertex is accepted. Other repeated vertices,
    overlapping edges and non-adjacent contacts are invalid polygon boundaries.
    Returns the open ring for callers that need a canonical representation.
    """
    try:
        p=np.asarray(poly,float)
    except (TypeError,ValueError) as e:
        raise PlacementError('INVALID_GEOMETRY','多边形需要有限二维坐标') from e
    if p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or not np.isfinite(p).all():
        raise PlacementError('INVALID_GEOMETRY','多边形需要至少三个有限二维顶点')
    if len(p)>3 and np.array_equal(p[0],p[-1]): p=p[:-1]
    eps=1e-10
    if len(p)<3 or area(p)<1e-12:
        raise PlacementError('INVALID_GEOMETRY','多边形不得退化')
    def cross(a,b,c):
        u=b-a;v=c-a
        return float(u[0]*v[1]-u[1]*v[0])
    def on_segment(a,b,c):
        return (abs(cross(a,b,c))<=eps and
                np.all(c>=np.minimum(a,b)-eps) and np.all(c<=np.maximum(a,b)+eps))
    for i in range(len(p)):
        if any(np.linalg.norm(p[i]-p[j])<=eps for j in range(i)):
            raise PlacementError('INVALID_GEOMETRY','多边形包含重复顶点')
        a,b=p[i],p[(i+1)%len(p)]
        # Adjacent collinear reversals overlap; straight continuation is valid.
        c=p[(i+2)%len(p)]
        if abs(cross(a,b,c))<=eps and np.dot(b-a,c-b)<-eps:
            raise PlacementError('SELF_INTERSECTING_POLYGON','相邻多边形边反向重叠')
        for j in range(i+1,len(p)):
            if j==i+1 or (i==0 and j==len(p)-1): continue
            c,d=p[j],p[(j+1)%len(p)]
            v1,v2,v3,v4=cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)
            proper=(v1>eps and v2<-eps or v1<-eps and v2>eps) and (v3>eps and v4<-eps or v3<-eps and v4>eps)
            if proper or on_segment(a,b,c) or on_segment(a,b,d) or on_segment(c,d,a) or on_segment(c,d,b):
                raise PlacementError('SELF_INTERSECTING_POLYGON','非相邻多边形边相交',Status.UNSUPPORTED_FEATURE)
    return p


def validate_convex(poly):
    p=validate_simple(poly)
    cross=[]
    for i in range(len(p)):
        a=p[(i+1)%len(p)]-p[i]; b=p[(i+2)%len(p)]-p[(i+1)%len(p)]
        cross.append(a[0]*b[1]-a[1]*b[0])
    if min(cross)<-1e-9 and max(cross)>1e-9:
        raise PlacementError("NONCONVEX_BODY","首版局部占位必须为凸多边形",Status.UNSUPPORTED_FEATURE)
