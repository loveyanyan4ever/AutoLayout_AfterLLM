"""JSON and side-by-side SVG, with explicit invalid-state labels."""
from dataclasses import asdict,is_dataclass
from pathlib import Path
from html import escape
import json
import platform
import importlib.metadata
from .core.geometry import bbox,transform,rect,pad_world


def write_json(path,data):
    p=Path(path); p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False,
                           default=lambda x:asdict(x) if is_dataclass(x) else str(x))+'\n',encoding='utf-8')


def environment():
    versions={}
    for name in ['numpy','scipy','PyYAML','osqp','pytest','shapely']:
        try: versions[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError: versions[name]='not installed'
    return {'python':platform.python_version(),'platform':platform.platform(),'dependencies':versions}


def layout_svg(problem,before,after,valid,target):
    chunks=['<svg xmlns="http://www.w3.org/2000/svg" width="1400" height="680" viewBox="0 0 1400 680">',
            '<rect width="1400" height="680" fill="#f4f6f8"/>',
            '<style>text{font-family:Arial,sans-serif;fill:#172338}</style>']
    palette=['#71c7b8','#a4b9ee','#efc17c','#d7a0d4','#b0c478','#99cbd7']
    cids=sorted(problem.groups)
    colors={c:palette[i%len(palette)] for i,c in enumerate(cids)}
    for panel,(state,title) in enumerate([(before,'SOURCE'),(after,'FEASIBLE PLACEMENT' if valid else 'DIAGNOSTIC — NOT FEASIBLE')]):
        outline=state.outline or after.outline; b=bbox(outline)
        scale=min(610/max(b[2]-b[0],1),520/max(b[3]-b[1],1)); ox=panel*700+45; oy=90
        def point(x,y): return (ox+(x-b[0])*scale,oy+(b[3]-y)*scale)
        def poly(pp,style):
            pts=' '.join(f'{point(x,y)[0]:.3f},{point(x,y)[1]:.3f}' for x,y in pp)
            return f'<polygon points="{pts}" {style}/>'
        chunks.append(f'<text x="{ox}" y="40" font-size="19" font-weight="bold">{escape(title)}</text>')
        chunks.append(poly(outline,'fill="white" stroke="#26354a" stroke-width="2"'))
        for d,r in state.domain_regions:
            chunks.append(poly([(r[0],r[1]),(r[2],r[1]),(r[2],r[3]),(r[0],r[3])],
                               'fill="#eef3fa" fill-opacity="0.7" stroke="#b0b6c0" stroke-dasharray="4 3"'))
            x,y=point(r[0],r[3]); chunks.append(f'<text x="{x+4}" y="{y+15}" font-size="12">{escape(d)}</text>')
        for cid,region in state.regions:
            chunks.append(poly(rect(region),f'fill="none" stroke="{colors.get(cid,"#777")}" stroke-width="2" stroke-dasharray="3 2"'))
        for p in state.poses:
            c=problem.design.by_ref[p.ref]; color=colors.get(problem.ref_group.get(p.ref),'#b4b6bc')
            chunks.append(poly(transform(c.body,p),f'fill="{color}" stroke="#344356" stroke-width="0.8"'))
            for pad in c.pads:
                chunks.append(poly(pad_world(pad,p),'fill="#b87822" fill-opacity="0.8" stroke="#6b481a" stroke-width="0.3"'))
            x,y=point(p.x,p.y)
            chunks.append(f'<text x="{x+2:.2f}" y="{y-2:.2f}" font-size="9">{escape(p.ref)}</text>')
    chunks.append('<text x="45" y="652" font-size="12">mm · body geometry · fixed objects in gray · placement rules only; safety sign-off not performed</text></svg>')
    Path(target).write_text('\n'.join(chunks),encoding='utf-8')
