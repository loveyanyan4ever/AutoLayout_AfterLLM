"""Conservative envelopes for explicitly supported EasyEDA Pro v3 shapes.

No source geometry is rewritten. Curves are enclosed by circumscribed polygons,
and a polygon origin ambiguity requires an explicit adapter policy.
"""
from __future__ import annotations
import math
import json
import numpy as np
from ..core.schema import PlacementError,Status,finite_number
from ..core.geometry import rect,bbox,rotate,validate_simple,validate_convex


def layer_type_map(rows):
    result={}
    for row in rows:
        rid=row.outer.get('id')
        if isinstance(rid,str):
            try:rid=json.loads(rid)
            except ValueError:pass
        lid=rid[-1] if isinstance(rid,list) and len(rid)==2 and rid[0]=='LAYER' else rid if isinstance(rid,int) and not isinstance(rid,bool) else row.inner.get('layerId')
        if lid is not None:result[lid]=row.inner.get('layerType')
    return result


def convex_envelope(points):
    p=sorted(set(tuple(map(float,x)) for x in points))
    if len(p)<3 or not np.isfinite(p).all():
        raise PlacementError('INVALID_GEOMETRY','铜包络需要至少三个有限二维点')
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower=[];upper=[]
    for q in p:
        while len(lower)>=2 and cross(lower[-2],lower[-1],q)<=0: lower.pop()
        lower.append(q)
    for q in reversed(p):
        while len(upper)>=2 and cross(upper[-2],upper[-1],q)<=0: upper.pop()
        upper.append(q)
    out=tuple(lower[:-1]+upper[:-1]);validate_convex(out)
    return out


def oval_envelope(width,height,ellipse=False,sides=32):
    """An affine circumscribed circle encloses an ellipse; a capsule is a
    segment Minkowski-summed with a circumscribed disk. Both are outer bounds.
    """
    if min(width,height)<=0 or not all(math.isfinite(x) for x in (width,height)):
        raise PlacementError('PAD_GEOMETRY','焊盘/孔尺寸必须为正有限数值')
    t=np.arange(sides)*2*math.pi/sides
    disk=np.column_stack([np.cos(t),np.sin(t)])/math.cos(math.pi/sides)
    if ellipse: return tuple(map(tuple,disk*[width/2,height/2]))
    r=min(width,height)/2
    offset=np.array([(width-height)/2,0]) if width>=height else np.array([0,(height-width)/2])
    return convex_envelope(np.vstack([disk*r+offset,disk*r-offset]))


def path_points(path,allow_arcs=False):
    """Read one native flat path; ARC is conservatively enclosed by its full
    supporting circle for body AABBs. Copper POLYGON paths currently need L/Z.
    """
    if not isinstance(path,list) or not path or isinstance(path[0],list):
        raise PlacementError('NATIVE_PATH','需要单环平面路径数组',Status.UNSUPPORTED_FEATURE)
    points=[];i=0;last=None;first=None;closed=False;extra=[]
    while i<len(path):
        token=path[i]
        if isinstance(token,str):
            if token=='L': i+=1;continue
            if token=='Z':
                if i!=len(path)-1 or first is None: raise PlacementError('NATIVE_PATH','Z 必须结束已存在路径')
                closed=True;i+=1;continue
            if token=='ARC' and allow_arcs and last is not None and i+3<len(path):
                sweep=finite_number(path[i+1],'arc sweep')
                end=np.array([finite_number(path[i+2],'arc x'),finite_number(path[i+3],'arc y')])
                d=end-last;chord=float(np.linalg.norm(d));a=abs(math.radians(sweep))
                if not 0<a<2*math.pi or chord<=1e-12 or abs(math.sin(a/2))<=1e-12:
                    raise PlacementError('NATIVE_ARC','退化圆弧不支持',Status.UNSUPPORTED_FEATURE)
                radius=chord/(2*abs(math.sin(a/2)))
                height=math.sqrt(max(0.,radius*radius-chord*chord/4))
                normal=np.array([-d[1],d[0]])/chord
                # Either signed arc convention is enclosed; source coordinates
                # are preserved verbatim and only the mechanical AABB uses it.
                for center in ((last+end)/2+height*normal,(last+end)/2-height*normal):
                    extra.extend([(center[0]-radius,center[1]-radius),(center[0]+radius,center[1]+radius)])
                points.append(tuple(end));last=end;i+=4;continue
            raise PlacementError('NATIVE_PATH_COMMAND',f'未支持路径命令 {token}',Status.UNSUPPORTED_FEATURE)
        if i+1>=len(path):raise PlacementError('NATIVE_PATH','路径缺少坐标')
        q=np.array([finite_number(path[i],'path x'),finite_number(path[i+1],'path y')])
        if first is None:first=q
        points.append(tuple(q));last=q;i+=2
    if first is not None and last is not None and np.linalg.norm(first-last)<1e-8:closed=True
    return points,extra,closed


def pad_geometry(payload,scale,polygon_frame='reject'):
    dp=payload.get('defaultPad') or {}
    if not isinstance(dp,dict):raise PlacementError('PAD_GEOMETRY','defaultPad 必须为对象')
    if payload.get('specialPad'):
        raise PlacementError('PAD_SPECIAL','特殊层焊盘尚未适配',Status.UNSUPPORTED_FEATURE)
    cx=finite_number(payload.get('centerX',0),'pad centerX')*scale
    cy=-finite_number(payload.get('centerY',0),'pad centerY')*scale
    angle=-finite_number(payload.get('padAngle',0),'padAngle')
    typ=str(dp.get('padType','')).upper()
    if typ=='POLYGON':
        if polygon_frame not in ('footprint','pad_local','conservative_union'):
            raise PlacementError('POLYGON_FRAME_REQUIRED','POLYGON 须显式声明 polygon_path_frame',Status.UNSUPPORTED_FEATURE)
        pts,_,closed=path_points(dp.get('path'))
        # Native paths can close by returning to the first vertex or implicitly.
        pp=validate_simple(pts)
        pp=np.asarray(pp)*[scale,-scale]
        absolute=pp
        relative=rotate(pp,angle)+[cx,cy]
        if polygon_frame=='footprint': candidate=absolute
        elif polygon_frame=='pad_local':candidate=relative
        else:candidate=np.vstack([absolute,relative])
        poly=convex_envelope(candidate)
        source='source_polygon_convex_envelope_'+polygon_frame
    else:
        w=finite_number(dp.get('width',0),'pad width')*scale
        h=finite_number(dp.get('height',0),'pad height')*scale
        if w<=0 or h<=0:raise PlacementError('PAD_GEOMETRY','焊盘尺寸缺失或非正')
        if typ=='RECT':shape=rect((-w/2,-h/2,w/2,h/2));source='source_rectangle_outer_bound'
        elif typ in ('CIRCLE','ELLIPSE'):
            if typ=='CIRCLE' and abs(w-h)>1e-8:raise PlacementError('PAD_SHAPE','CIRCLE 宽高不一致')
            shape=oval_envelope(w,h,ellipse=True);source='circumscribed_ellipse_32'
        elif typ in ('OVAL','ROUND'):
            shape=oval_envelope(w,h);source='circumscribed_capsule_32'
        else:raise PlacementError('PAD_SHAPE',f'不支持焊盘 {typ}',Status.UNSUPPORTED_FEATURE)
        poly=tuple(map(tuple,rotate(shape,angle)+[cx,cy]))
    validate_convex(poly)
    hole=payload.get('hole');hole_poly=();hole_center=None;hole_mm=0.
    if hole:
        if isinstance(hole,(int,float)):
            hw=hh=finite_number(hole,'hole')*scale;ht='ROUND'
        elif isinstance(hole,dict):
            hw=finite_number(hole.get('diameter',hole.get('width',0)),'hole width')*scale
            hh=finite_number(hole.get('diameter',hole.get('height',hole.get('width',0))),'hole height')*scale
            ht=str(hole.get('holeType','ROUND')).upper()
        else:raise PlacementError('HOLE_GEOMETRY','孔结构未支持',Status.UNSUPPORTED_FEATURE)
        if min(hw,hh)<=0:raise PlacementError('HOLE_GEOMETRY','孔尺寸必须为正')
        if ht in ('ROUND','SLOT'):hshape=oval_envelope(hw,hh)
        elif ht=='RECT':hshape=rect((-hw/2,-hh/2,hw/2,hh/2))
        else:raise PlacementError('HOLE_GEOMETRY',f'孔类型 {ht} 未支持',Status.UNSUPPORTED_FEATURE)
        ox=finite_number(payload.get('padOffsetX',0),'hole offsetX')*scale
        oy=-finite_number(payload.get('padOffsetY',0),'hole offsetY')*scale
        delta=rotate([(ox,oy)],angle)[0]
        hole_center=tuple(np.array([cx,cy])+delta)
        hole_angle=angle-finite_number(payload.get('relativeAngle',0),'hole relativeAngle')
        hole_poly=tuple(map(tuple,rotate(hshape,hole_angle)+hole_center))
        hole_mm=max(hw,hh)
    return tuple(poly),(cx,cy),source,hole_mm,hole_poly,hole_center


def source_body(archive,footprint,scale):
    """Component-shape layer 48 is explicitly named physical component shape
    in the native layer table. Its closed paths yield a conservative body AABB.
    """
    layer_types=layer_type_map(archive.of(footprint,'LAYER'))
    points=[];margin=0.
    for r in archive.of(footprint):
        b=r.inner
        explicit=b.get('polyType') in ('COURTYARD','ASSEMBLY')
        is_shape=layer_types.get(b.get('layerId'))=='COMPONENT_SHAPE'
        if r.outer['type'] not in ('POLY','FILL') or not (explicit or is_shape):continue
        paths=b.get('path')
        if isinstance(paths,list) and paths and isinstance(paths[0],list):rings=paths
        else:rings=[paths]
        for path in rings:
            pp,extra,closed=path_points(path,allow_arcs=True)
            if not closed:raise PlacementError('OPEN_BODY','原生本体外形没有闭合')
            if not extra:validate_simple(pp)
            points.extend(pp+extra)
        margin=max(margin,abs(finite_number(b.get('width',0),'body stroke width'))*scale/2)
    if not points:return None
    a=np.asarray(points)*[scale,-scale];b=bbox(a)
    return rect((b[0]-margin,b[1]-margin,b[2]+margin,b[3]+margin)),'source_component_shape_aabb'
