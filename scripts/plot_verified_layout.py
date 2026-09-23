#!/usr/bin/env python3
"""Render saved PCB layout geometry without importing or modifying the project.

Developer/reporting helper only: requires matplotlib and numpy, neither becomes
an extra application runtime requirement. Run after a FEASIBLE run has finished:

  python plot_verified_layout.py --run path/to/run --out path/to/figures

For an internal non-delivery preview add --draft. Source and result coordinates
are read verbatim in the normalized model's millimetre / Y-up frame. The plot is
not an independent constraint checker or a claim of native editor acceptance.
"""
from __future__ import annotations

import argparse
from io import BytesIO
from collections import Counter
import json
import math
from pathlib import Path
import re
import unicodedata

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.text import Annotation, Text
from matplotlib.lines import Line2D
from matplotlib.patches import Patch, Polygon, Rectangle
import numpy as np

PALETTE = ['#378C99', '#B16BBD', '#78818B', '#D89837', '#55A27A', '#5B7FBB', '#D56F7A', '#8075AE']
INK = '#263447'
PAPER = '#F7F9FC'
COPPER = '#CF8724'
BOARD = '#243A53'


def read_json(path):
    with path.open(encoding='utf-8') as source:
        return json.load(source)


def sort_key(value):
    return [int(part) if part.isdigit() else part for part in re.split(r'(\d+)', value)]


def pick_font():
    for entry in font_manager.fontManager.ttflist:
        if any(name in entry.name.lower() for name in ('noto sans cjk', 'source han sans', 'wenquanyi', 'wqy')):
            return entry.name, True
    return 'DejaVu Sans', False


def label(ref):
    # Fullwidth punctuation has no glyph in the available default font. NFKC
    # changes only display typography for this board (VIN（+24V） -> VIN(+24V));
    # the complete original ref remains the dictionary key and is never dropped.
    return unicodedata.normalize('NFKC', ref)


def transform(poly, pose):
    points = np.asarray(poly, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('Invalid saved polygon; refusing to invent geometry')
    angle = math.radians(float(pose['angle']))
    matrix = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    return points @ matrix.T + [float(pose['x']), float(pose['y'])]


def bounds(points):
    p = np.asarray(points, dtype=float)
    return (float(p[:, 0].min()), float(p[:, 1].min()), float(p[:, 0].max()), float(p[:, 1].max()))


def load_run(directory):
    design = read_json(directory / 'normalized_design.json')
    placement = read_json(directory / 'placements.json')
    assignment = read_json(directory / 'assignments.json')
    compiled = read_json(directory / 'compiled_constraints.json')
    report = read_json(directory / 'report.json')
    if placement.get('status') != 'FEASIBLE' or report.get('status') != 'FEASIBLE':
        raise ValueError('Only completed FEASIBLE runs can be plotted as results')
    if not report.get('validation', {}).get('model_feasible'):
        raise ValueError('The saved report does not certify model feasibility')
    if placement.get('source_hash') != design.get('source_hash'):
        raise ValueError('The saved source hashes disagree')
    if placement.get('units') != 'mm' or placement.get('frame') != 'board_local_y_up':
        raise ValueError('Expected normalized millimetres with Y up')
    if placement.get('rotation') != 'ccw_degrees':
        raise ValueError('Expected counter-clockwise degree rotations')
    state = placement['state']
    components = {c['ref']: c for c in design['components']}
    poses = {p['ref']: p for p in state['poses']}
    if len(components) != len(design['components']) or len(poses) != len(state['poses']) or set(components) != set(poses):
        raise ValueError('Missing or repeated component in saved result')
    clusters = dict(assignment['clusters'])
    parents = dict(assignment.get('parent_clusters', []))
    if set(clusters) != set(components):
        raise ValueError('Assignment coverage differs from the component set')
    parent_of = {ref: parents.get(cid, '(no L2)') for ref, cid in clusters.items()}
    parent_ids = sorted(set(parent_of.values()), key=sort_key)
    colors = {parent: PALETTE[i % len(PALETTE)] for i, parent in enumerate(parent_ids)}
    return dict(design=design, state=state, components=components, report=report,
                compiled=compiled, clusters=clusters, parents=parents, parent_of=parent_of,
                parent_ids=parent_ids, colors=colors, poses=poses)


def all_world_geometry(data, poses):
    points = []
    for ref, c in data['components'].items():
        pose = poses[ref]
        points.extend(transform(c['body'], pose))
        for pad in c['pads']:
            points.extend(transform(pad['polygon'], pose))
            if pad.get('hole_polygon'):
                points.extend(transform(pad['hole_polygon'], pose))
    return np.asarray(points)


def board_dimensions(outline):
    b = bounds(outline)
    return b[2] - b[0], b[3] - b[1]


def add_board(ax, outline, reference=False):
    if not outline:
        return
    ax.add_patch(Polygon(outline, closed=True, facecolor='none' if reference else '#FCFDFE',
                         edgecolor='#78899C' if reference else BOARD,
                         linewidth=1.15 if reference else 1.8,
                         linestyle=(0, (5, 4)) if reference else '-', zorder=0 if not reference else 4))


def add_regions(ax, data, fontsize):
    for cid, box in data['state'].get('regions', []):
        parent = data['parents'].get(cid, '(no L2)')
        color = data['colors'].get(parent, '#6A7A90')
        x0, y0, x1, y1 = map(float, box)
        ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, facecolor=color,
                               edgecolor=color, alpha=.065, linewidth=0, zorder=.3))
        ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, facecolor='none',
                               edgecolor=color, alpha=.75, linewidth=.7,
                               linestyle=(0, (4, 2)), zorder=3.5))
        ax.annotate(cid, (x0, y1), xytext=(2, -2), textcoords='offset points',
                    ha='left', va='top', color=color, fontsize=fontsize,
                    fontweight='bold', zorder=8,
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.90, pad=.55))


def draw_geometry(ax, data, poses, detail, final=False):
    for ref in sorted(data['components'], key=sort_key):
        c = data['components'][ref]
        pose = poses[ref]
        color = data['colors'][data['parent_of'][ref]]
        body = transform(c['body'], pose)
        ax.add_patch(Polygon(body, closed=True, facecolor=color, edgecolor=color,
                             alpha=.34, linewidth=.55 if detail else .42, zorder=2))
        for pad in c['pads']:
            copper = transform(pad['polygon'], pose)
            ax.add_patch(Polygon(copper, closed=True, facecolor=COPPER, edgecolor='#8B6021',
                                 linewidth=.35 if detail else .2, alpha=.83, zorder=3))
            if pad.get('hole_polygon'):
                hole = transform(pad['hole_polygon'], pose)
                ax.add_patch(Polygon(hole, closed=True, facecolor='white', edgecolor='#4B5563',
                                     linewidth=.4 if detail else .25, zorder=4))
        box = bounds(body)
        cx, cy = (box[0]+box[2])/2, (box[1]+box[3])/2
        small = min(box[2]-box[0], box[3]-box[1]) < .78
        if detail:
            fontsize = 6.6 if len(label(ref)) <= 4 else 6.1
        elif final:
            fontsize = 5.6 if len(label(ref)) <= 4 else 5.0
        else:
            fontsize = 4.2 if len(label(ref)) <= 4 else 3.8
        # Labels remain adjacent to their real body; only visual text is offset.
        # The optional thin line makes tiny test-pad labels unambiguous.
        xy = (cx, cy)
        offset = (0, 4.2 if detail else 2.8) if small else (0, 0)
        is_special = ref in data['compiled'].get('special', []) or ref in data['compiled'].get('fixed', [])
        ax.annotate(label(ref), xy, xytext=offset, textcoords='offset points',
                    ha='center', va='bottom' if small else 'center', color='#142231',
                    fontsize=fontsize, fontweight='bold' if is_special else 'normal', zorder=7,
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.77, pad=.27),
                    arrowprops=dict(arrowstyle='-', linewidth=.3, color='#455267') if small else None)


def configure_axes(ax):
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X (mm)', color=INK, fontsize=10)
    ax.set_ylabel('Y (mm)', color=INK, fontsize=10)
    ax.tick_params(axis='both', colors='#4C5D70', labelsize=9)
    ax.grid(color='#DEE5ED', linewidth=.55, alpha=.6, zorder=-2)
    ax.set_axisbelow(True)
    ax.set_facecolor('white')
    for spine in ax.spines.values():
        spine.set_color('#D5DDE6')


def plot_panel(ax, data, final, detail=False):
    poses = data['poses'] if final else {ref:c['source_pose'] for ref,c in data['components'].items()}
    outline = data['state']['outline'] if final else data['design'].get('outline', [])
    reference = not final and not outline
    if reference:
        outline = data['state']['outline']
    add_board(ax, outline, reference=reference)
    if final:
        add_regions(ax, data, 6.6 if detail else 5.0)
    draw_geometry(ax, data, poses, detail, final=final)
    geometry = all_world_geometry(data, poses)
    if outline:
        geometry = np.vstack([geometry, outline])
    x0, y0, x1, y1 = bounds(geometry)
    padding = max(x1-x0, y1-y0) * (.035 if detail else .04)
    ax.set_xlim(x0-padding, x1+padding)
    ax.set_ylim(y0-padding, y1+padding)
    configure_axes(ax)
    return reference


def parent_legend(data):
    counts = Counter(data['parent_of'].values())
    return [Patch(facecolor=data['colors'][parent], edgecolor=data['colors'][parent], alpha=.65,
                  label=f'{parent}  ({counts[parent]} components)') for parent in data['parent_ids']]


def geometry_legend(reference=False):
    result = [Patch(facecolor=COPPER, edgecolor='#8B6021', label='Copper envelope'),
              Patch(facecolor='white', edgecolor='#4B5563', label='Drill / slot'),
              Line2D([0],[0],color='#6B8197',linestyle=(0,(4,2)),linewidth=.9,label='L1 region'),
              Line2D([0],[0],color=BOARD,linewidth=1.8,label='Result board outline')]
    if reference:
        result.append(Line2D([0],[0],color='#78899C',linestyle=(0,(5,4)),linewidth=1.1,
                             label='Result outline shown as reference in source panel'))
    return result


def caption(data):
    metrics = data['report']['final']
    w,h = board_dimensions(data['state']['outline'])
    return f"{len(data['components'])} components   |   Board {w:.2f} x {h:.2f} mm   |   HPWL {metrics['hpwl_mm']:.2f} mm"


def resolve_labels(fig):
    """Greedily separate only display labels; never move geometry or data."""
    fig.canvas.draw()
    renderer=fig.canvas.get_renderer()
    for ax in fig.axes:
        labels=[text for text in ax.texts if isinstance(text,Annotation)]
        # Long and group labels get first choice; ties preserve drawing order.
        labels.sort(key=lambda text:(not text.get_text().startswith('L1-'),-len(text.get_text())))
        occupied=[]
        offsets=[(0,0),(0,5),(0,-5),(5,0),(-5,0),(0,10),(0,-10),
                 (7,7),(-7,7),(7,-7),(-7,-7),(12,0),(-12,0),
                 (0,15),(0,-15),(12,10),(-12,10),(12,-10),(-12,-10)]
        for text in labels:
            origin=np.asarray(text.get_position(),float)
            best=None
            for dx,dy in offsets:
                position=origin+[dx,dy]
                text.set_position(position);text.update_positions(renderer)
                box=Text.get_window_extent(text,renderer=renderer).expanded(1.04,1.16)
                overlap=0.
                for previous in occupied:
                    width=max(0.,min(box.x1,previous.x1)-max(box.x0,previous.x0))
                    height=max(0.,min(box.y1,previous.y1)-max(box.y0,previous.y0))
                    overlap+=width*height
                score=(overlap,dx*dx+dy*dy)
                if best is None or score<best[0]:best=(score,position,box)
                if overlap==0:break
            text.set_position(best[1]);text.update_positions(renderer);occupied.append(best[2])
            if np.linalg.norm(best[1]-origin)>1. and text.arrow_patch is None:
                anchor_px=ax.transData.transform(text.xy)
                end_px=anchor_px+best[1]*fig.dpi/72
                end=ax.transData.inverted().transform(end_px)
                ax.plot([text.xy[0],end[0]],[text.xy[1],end[1]],color='#667788',
                        linewidth=.35,alpha=.75,zorder=6,solid_capstyle='round')


def save_png(fig,path,dpi):
    resolve_labels(fig)
    buffer=BytesIO()
    fig.savefig(buffer,format='png',dpi=dpi,facecolor=fig.get_facecolor())
    payload=buffer.getvalue()
    if not payload.endswith(b'\x00\x00\x00\x00IEND\xaeB`\x82'):
        raise RuntimeError('PNG encoder did not produce a complete image')
    path.write_bytes(payload)


def save_comparison(data, out, dpi, draft, run_name):
    fig, axes = plt.subplots(1,2,figsize=(21,9.5))
    fig.patch.set_facecolor(PAPER)
    fig.subplots_adjust(left=.05,right=.985,bottom=.17,top=.83,wspace=.17)
    source_reference = plot_panel(axes[0],data,False)
    plot_panel(axes[1],data,True)
    axes[0].set_title('Source arrangement' + (' — no source board outline' if source_reference else ''),
                      loc='left',fontsize=12,color=INK,pad=12,fontweight='bold')
    axes[1].set_title('Automatic placement — configured model feasible',loc='left',
                      fontsize=12,color=INK,pad=12,fontweight='bold')
    title = ('INTERNAL PREVIEW | ' + run_name + ' | ') if draft else ''
    fig.suptitle(title+'PCB placement: source and result',x=.05,y=.96,ha='left',fontsize=21,color=INK,fontweight='bold')
    fig.text(.05,.90,caption(data),fontsize=11,color='#4B6279')
    fig.legend(handles=parent_legend(data),loc='lower center',bbox_to_anchor=(.50,.095),
               ncol=min(len(data['parent_ids']),6),frameon=False,fontsize=9,
               title='L2 functional groups (colors do not denote isolation domains)',title_fontsize=9)
    fig.legend(handles=geometry_legend(source_reference),loc='lower center',bbox_to_anchor=(.50,.052),
               ncol=5,frameon=False,fontsize=8.5)
    fig.text(.05,.030,'Both axes use mm; panels have independent display scales. Dashed L1 boxes are saved solver regions.',fontsize=9,color='#526578')
    fig.text(.05,.009,'Model geometry includes conservative envelopes; native editor / DRC verification remains PENDING. No routing is shown.',fontsize=8.5,color='#68798A')
    path=out/'layout_comparison.png'
    save_png(fig,path,dpi)
    plt.close(fig)
    return path


def save_final(data, out, dpi, draft, run_name):
    fig=plt.figure(figsize=(15,12))
    fig.patch.set_facecolor(PAPER)
    ax=fig.add_axes([.065,.135,.735,.73])
    plot_panel(ax,data,True,detail=True)
    title = ('INTERNAL PREVIEW | ' + run_name + ' | ') if draft else ''
    fig.text(.065,.954,title+'Automatic PCB placement',fontsize=22,fontweight='bold',color=INK)
    fig.text(.065,.916,caption(data),fontsize=11,color='#4B6279')
    count_regions=len(data['state'].get('regions',[]))
    fig.text(.065,.885,f'{count_regions} movable L1 regions   |   Physical bodies, copper and drill geometry shown in mm',fontsize=10,color='#4B6279')
    fig.legend(handles=parent_legend(data),loc='upper left',bbox_to_anchor=(.815,.86),
               frameon=False,fontsize=9,title='L2 functional groups',title_fontsize=10,labelspacing=1.1)
    fig.legend(handles=geometry_legend(),loc='upper left',bbox_to_anchor=(.815,.62),
               frameon=False,fontsize=9,title='Geometry key',title_fontsize=10,labelspacing=1.0)
    explanation=('Dashed boxes: saved L1 regions\n\nBold references: preplaced / fixed\nmechanical parts and interfaces\n\nWhite circles/capsules: holes\n\nL2 colors are functional groups,\nnot isolation domains.')
    fig.text(.822,.40,explanation,fontsize=9,color='#526578',va='top',linespacing=1.45)
    fig.text(.065,.082,'Configured 2D model: FEASIBLE   |   Native editor / native DRC: PENDING',fontsize=11,color=INK)
    fig.text(.065,.051,'Source footprint records are preserved. Plotted geometry is the normalized checking model, including conservative envelopes.',fontsize=9,color='#526578')
    fig.text(.065,.031,'L1 mechanical/interface membership is retained even when these preplaced members have no movable region.',fontsize=9,color='#526578')
    path=out/'layout_final.png'
    save_png(fig,path,dpi)
    plt.close(fig)
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--dpi',type=int,default=220)
    parser.add_argument('--draft',action='store_true')
    args=parser.parse_args()
    if args.dpi<100:
        parser.error('--dpi must be at least 100 for readable reference labels')
    data=load_run(args.run)
    args.out.mkdir(parents=True,exist_ok=True)
    font,cjk=pick_font()
    plt.rcParams.update({'font.family':font,'axes.unicode_minus':False,'savefig.transparent':False})
    comparison=save_comparison(data,args.out,args.dpi,args.draft,args.run.name)
    final=save_final(data,args.out,args.dpi,args.draft,args.run.name)
    print(json.dumps({'status':'PLOTTED','run':str(args.run),'draft':args.draft,
                      'files':[str(comparison),str(final)],'components':len(data['components']),
                      'physical_pad_via_objects':sum(len(c['pads']) for c in data['components'].values()),
                      'font':font,'cjk_font_available':cjk,
                      'reference_display_normalization':{ref:label(ref) for ref in data['components'] if ref!=label(ref)},
                      'source_outline_present':bool(data['design'].get('outline')),
                      'native_editor_validation':'PENDING'},ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
