#!/usr/bin/env python3
"""Independent GEOS/Shapely check of a saved hierarchical-placement result.

Native polygons are read with the public format adapter. No optimizer,
constraint compiler, internal validator or signed-distance routine is imported.
This verifies the decoded geometry, not opening the file in the native editor.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import itertools
import json
import math
from pathlib import Path
import zipfile

import shapely
from shapely import affinity
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union

from pcb_hierplace.io.epro2 import parse_project


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local_polygon(points):
    result = Polygon(points)
    if not result.is_valid or result.area <= 0:
        raise ValueError('Invalid or zero-area decoded polygon')
    return result


def world(geometry, pose):
    return affinity.translate(affinity.rotate(geometry, pose['angle'], origin=(0, 0)),
                              xoff=pose['x'], yoff=pose['y'])


def raw_archive(path):
    """Independent log replay, retaining pad IDs and document boundaries."""
    with zipfile.ZipFile(path) as archive:
        members = {name: archive.read(name) for name in archive.namelist()}
    logs = [name for name in members if name.lower().endswith('.epru')]
    if len(logs) != 1:
        raise ValueError('Expected exactly one epru')
    lines = members[logs[0]].decode('utf-8-sig').splitlines(keepends=True)
    records, doc, client, documents = {}, None, '', {}
    for line in lines:
        if not line.strip():
            continue
        head_text, payload = line.strip().split('||', 1)
        head = json.loads(head_text)
        payload = payload[:-1] if payload.endswith('|') else payload
        body = json.loads(payload) if payload else None
        if body == '':
            body = None
        if head['type'] == 'DOCHEAD':
            doc, client = body['uuid'], str(body.get('client', ''))
            documents[doc] = body
            continue
        identifier = head.get('id')
        if not isinstance(identifier, str):
            identifier = json.dumps(identifier, sort_keys=True, separators=(',', ':'))
        key = (doc, head['type'], identifier)
        ticket = head.get('ticket', 0)
        old = records.get(key)
        if old is None or ticket > old[0] or (ticket == old[0] and client < old[1]):
            records[key] = (ticket, client, body)
    return members, logs[0], lines, {k: v[2] for k, v in records.items() if v[2] is not None}, documents


def evaluate_run(run_path, output_path):
    root = Path(run_path)
    read = lambda name: json.loads((root / name).read_text(encoding='utf-8'))
    cfg, norm = read('effective_config.json'), read('normalized_design.json')
    manifest, result = read('run_manifest.json'), read('placements.json')
    compiled, assignments = read('compiled_constraints.json'), read('assignments.json')
    state = result['state']
    source_path = Path(cfg['input']['project'])
    if not source_path.is_absolute():
        source_path = root / source_path
    source_before = sha256(source_path)
    board_id = cfg['input']['board_id']
    readback = parse_project(output_path, board_id, {
        'source_unit': cfg['input']['source_unit'], 'geometry': cfg['geometry']})
    actual = {c.ref: asdict(c) for c in readback.components}
    original = {c['ref']: c for c in norm['components']}
    poses = {ref: c['source_pose'] for ref, c in actual.items()}
    expected = {pose['ref']: pose for pose in state['poses']}
    eps = cfg['geometry']['epsilon_mm']
    violations, checks = [], {}

    def check(name, passed, objects=(), actual_value=None, required=None):
        checks[name] = checks.get(name, 0) + 1
        if not passed:
            entry = {'rule': name, 'objects': list(objects)}
            if actual_value is not None:
                entry['actual'] = actual_value
            if required is not None:
                entry['required'] = required
            violations.append(entry)

    def contains(name, container, geometry, objects):
        check(name, container.buffer(eps).covers(geometry), objects)

    def gap_check(name, a, b, minimum, objects):
        distance = a.distance(b)
        overlap = a.intersection(b).area
        check(name, distance >= minimum - eps and overlap <= eps * eps,
              objects, distance, minimum)
        return distance

    def angle_error(a, b):
        return abs((a - b + 180) % 360 - 180)

    check('SOURCE_HASH', source_before == manifest['source_hash'])
    check('OBJECT_SET', set(original) == set(actual) == set(expected))
    check('COMPONENT_UUID', {c['uuid'] for c in original.values()} ==
          {c['uuid'] for c in actual.values()})
    check('NETLIST_STABLE_PAD_IDS', norm['nets'] == json.loads(json.dumps([asdict(n) for n in readback.nets])))
    outline = local_polygon(state['outline'])
    check('READBACK_OUTLINE', outline.hausdorff_distance(local_polygon(readback.outline)) <= eps)
    if not compiled['estimated']:
        check('FIXED_OUTLINE', outline.hausdorff_distance(local_polygon(norm['outline'])) <= eps)
    board_bounds = outline.bounds
    groups = compiled['groups']
    regions = {name: box(*bounds) for name, bounds in state['regions']}
    domain_regions = {name: box(*bounds) for name, bounds in state['domain_regions']}
    domains = dict(assignments['domains'])
    pad_domains = {tuple(key): domain for key, domain in compiled['pad_domains']}
    electrical = {name for name, definition in cfg['rules']['domains'].items()
                  if definition['kind'] == 'electrical'}
    isolation = {tuple(sorted(rule['domains'])): rule for rule in cfg['rules']['isolation_pairs']}
    rules = cfg['rules']['component_rules']
    body, pads, holes, occupancy, copper = {}, {}, {}, {}, []
    geometry_fields = ['footprint', 'body', 'locked']
    pose_error, moved = 0.0, []
    for ref in sorted(actual):
        component, pose = actual[ref], poses[ref]
        old = original[ref]
        for field in geometry_fields:
            a = json.loads(json.dumps(component[field]))
            check('GEOMETRY_' + field.upper(), a == old[field], [ref])
        old_pads = {pad['id']: pad for pad in old['pads']}
        check('PAD_IDS', set(old_pads) == {pad['id'] for pad in component['pads']}, [ref])
        for pad in component['pads']:
            for field in ('number', 'net', 'layer', 'polygon', 'center', 'hole_mm', 'hole_polygon', 'hole_center'):
                if field in old_pads.get(pad['id'], {}):
                    check('PAD_' + field.upper(), json.loads(json.dumps(pad[field])) == old_pads[pad['id']][field],
                          [ref, pad['id']])
        want = expected[ref]
        error = max(abs(pose['x'] - want['x']), abs(pose['y'] - want['y']))
        pose_error = max(pose_error, error)
        check('READBACK_POSE', error <= eps and angle_error(pose['angle'], want['angle']) <= 1e-7,
              [ref], error, eps)
        check('LAYER', pose['layer'] == old['source_pose']['layer'], [ref])
        check('ANGLE', any(angle_error(pose['angle'], a) < 1e-7 for a in compiled['angles'][ref]), [ref])
        source_pose = old['source_pose']
        changed = math.hypot(pose['x']-source_pose['x'], pose['y']-source_pose['y']) > eps or angle_error(pose['angle'], source_pose['angle']) > 1e-7
        if changed:
            moved.append(ref)
        if compiled['motions'][ref] == 'fixed_pose':
            check('FIXED_POSE', not changed, [ref])
        r = rules.get(ref, {})
        local_body = local_polygon(component['body'])
        body[ref] = world(local_body, pose)
        local_occupied = [local_body]
        pads[ref], holes[ref] = [], []
        for pad in component['pads']:
            local = local_polygon(pad['polygon'])
            geometry = world(local, pose)
            pads[ref].append((pad, geometry))
            copper.append((ref, pad, geometry, pad_domains[(ref, pad['id'])]))
            local_occupied.append(local)
            contains('BOARD_PAD', outline, geometry, [ref, pad['id']])
            if pad_domains[(ref, pad['id'])] in domain_regions:
                contains('PAD_DOMAIN', domain_regions[pad_domains[(ref, pad['id'])]], geometry, [ref, pad['id']])
            hole = None
            if pad.get('hole_polygon'):
                hole = local_polygon(pad['hole_polygon'])
            elif pad['hole_mm'] > 0:
                center = pad.get('hole_center') or pad['center']
                hole = Point(center).buffer(pad['hole_mm']/2, quad_segs=128)
            if hole is not None:
                local_occupied.append(hole)
                geometry = world(hole, pose)
                holes[ref].append((pad, geometry))
                contains('BOARD_HOLE', outline, geometry, [ref, pad['id']])
        for key in ('insertion_keepout', 'keepout_geometry'):
            if r.get(key) is not None:
                local_occupied.append(box(*r[key]))
        template = cfg['rules']['bridge_templates'].get(r.get('bridge_template'), {})
        if template.get('keepout_geometry') is not None:
            local_occupied.append(box(*template['keepout_geometry']))
        occupancy[ref] = world(box(*unary_union(local_occupied).bounds), pose)
        if not r.get('allow_body_overhang', False):
            contains('BOARD_BODY', outline, body[ref], [ref])
        domain = r.get('conductive_class', domains[ref])
        if domain in electrical:
            contains('BODY_DOMAIN', domain_regions[domain], body[ref].intersection(outline), [ref])
        if r.get('conductive_class') in electrical:
            copper.append((ref, {'id': '__body_conductor__', 'net': '', 'number': 'metal_body'}, body[ref], r['conductive_class']))
        motion = compiled['motions'][ref]
        if motion == 'corner':
            b = occupancy[ref].bounds
            inset = r['corner_inset_mm']
            valid = any(abs((b[0]-board_bounds[0] if c.endswith('left') else board_bounds[2]-b[2])-inset) <= eps
                        and abs((b[1]-board_bounds[1] if c.startswith('bottom') else board_bounds[3]-b[3])-inset) <= eps
                        for c in r['allowed_corners'])
            check('CORNER', valid, [ref])
            contains('CORNER_BOARD', outline, occupancy[ref], [ref])
        if motion == 'edge_band':
            b = occupancy[ref].bounds
            edge_gaps = {'left': b[0]-board_bounds[0], 'right': board_bounds[2]-b[2],
                         'bottom': b[1]-board_bounds[1], 'top': board_bounds[3]-b[3]}
            check('EDGE_BAND', any(abs(edge_gaps[e]-r['edge_offset_mm']) <= eps for e in r['allowed_edges']), [ref])
            check('EDGE_BAND_ANGLE', angle_error(pose['angle'], source_pose['angle']) <= 1e-7, [ref])
            contains('EDGE_BAND_BOARD', outline, occupancy[ref], [ref])
        elif motion.startswith('edge') or 'allowed_edges' in r:
            point = world(Point(r['local_mating_point_mm']), pose)
            theta = math.radians(pose['angle'])
            vx, vy = r['local_outward_vector']
            vector = (math.cos(theta)*vx-math.sin(theta)*vy, math.sin(theta)*vx+math.cos(theta)*vy)
            length = math.hypot(*vector)
            matches = []
            for edge in r['allowed_edges']:
                axis = 0 if edge in ('left', 'right') else 1
                normal = {'left': (-1,0), 'right': (1,0), 'bottom': (0,-1), 'top': (0,1)}[edge]
                boundary = {'left': board_bounds[0], 'right': board_bounds[2], 'bottom': board_bounds[1], 'top': board_bounds[3]}[edge]
                p = (point.x, point.y)
                along = p[1-axis] - board_bounds[1-axis]
                matches.append(abs(p[axis] - boundary-r['edge_offset_mm']*normal[axis]) <= eps
                               and r['edge_segment_mm'][0]-eps <= along <= r['edge_segment_mm'][1]+eps
                               and sum(a*b for a,b in zip(vector,normal))/length >= 1-1e-7)
            check('EDGE_MATING', any(matches), [ref])
    check('CLUSTER_SET', set(regions) == set(groups))
    for cid, members in groups.items():
        for ref in members:
            contains('HARD_CLUSTER', regions[cid], occupancy[ref], [cid, ref])
        domain = domains[members[0]]
        if domain in domain_regions:
            contains('CLUSTER_DOMAIN', domain_regions[domain], regions[cid], [cid, domain])
    for a, b in itertools.combinations(regions, 2):
        check('CLUSTER_NONOVERLAP', regions[a].intersection(regions[b]).area <= eps*eps, [a, b])
    for domain, region in domain_regions.items():
        contains('DOMAIN_BOARD', outline, region, [domain])
    for pair, rule in isolation.items():
        gap_check('ISOLATION_RESERVE', domain_regions[pair[0]], domain_regions[pair[1]],
                  rule['copper_clearance_mm']+rule['routing_reserve_mm'], pair)
    min_body = math.inf
    for a, b in itertools.combinations(body, 2):
        min_body = min(min_body, gap_check('BODY_GAP', body[a], body[b], cfg['rules']['mechanical_gap_mm'], [a,b]))
    min_copper, min_isolation, min_hole = math.inf, math.inf, math.inf
    for first, second in itertools.combinations(copper, 2):
        a, pa, ga, da = first
        b, pb, gb, db = second
        same_net = bool(pa['net']) and pa['net'] == pb['net']
        if da == db and (a == b or same_net):
            continue
        pair = tuple(sorted((da, db)))
        required = (isolation[pair]['copper_clearance_mm'] if pair in isolation
                    else 0.0 if same_net else cfg['rules']['default_pad_clearance_mm'])
        distance = gap_check('COPPER_GAP', ga, gb, required, [a+':'+pa['id'], b+':'+pb['id']])
        min_copper = min(min_copper, distance)
        if da != db:
            min_isolation = min(min_isolation, distance)
    for ref, entries in holes.items():
        for pad, hole in entries:
            for other in body:
                if other == ref:
                    continue
                geometries = [body[other]]+[p for _,p in pads[other]+holes[other]]
                for geometry in geometries:
                    min_hole = min(min_hole, gap_check('HOLE_KEEPOUT', hole, geometry,
                        cfg['rules']['mechanical_gap_mm'], [ref+':'+pad['id'], other]))
    for ref, pose in poses.items():
        rule = rules.get(ref, {})
        occupied = [body[ref]]+[p for _,p in pads[ref]+holes[ref]]
        for obstacle in cfg['rules']['obstacles']:
            if pose['layer'] in obstacle.get('layers', ['top','bottom']):
                for geometry in occupied:
                    gap_check('OBSTACLE', geometry, box(*obstacle['bbox_mm']),
                              cfg['rules']['mechanical_gap_mm'], [ref, obstacle['id']])
        template = cfg['rules']['bridge_templates'].get(rule.get('bridge_template'), {})
        keepouts = [rule.get(key) for key in ('keepout_geometry', 'insertion_keepout')]
        keepouts.append(template.get('keepout_geometry'))
        for keepout in filter(lambda value: value is not None, keepouts):
            geometry = world(box(*keepout), pose)
            for other in actual:
                if other != ref:
                    for other_geometry in [body[other]]+[p for _,p in pads[other]+holes[other]]:
                        check('KEEPOUT', geometry.intersection(other_geometry).area <= eps*eps, [ref, other])
        if template:
            a, b = template['between']
            pa, pb = domain_regions[a].centroid, domain_regions[b].centroid
            direction = (pb.x-pa.x, pb.y-pa.y)
            norm_direction = math.hypot(*direction)
            vector = affinity.rotate(Point(template['axis_local']), pose['angle'], origin=(0,0))
            norm_vector = math.hypot(vector.x, vector.y)
            dot = (direction[0]*vector.x+direction[1]*vector.y)/(norm_vector*norm_direction)
            check('BRIDGE_AXIS', dot >= 1-1e-7, [ref], dot, 1.)
            axis = 0 if abs(direction[0]) >= abs(direction[1]) else 1
            bounds_a, bounds_b = domain_regions[a].bounds, domain_regions[b].bounds
            boundary = ((bounds_a[axis+2]+bounds_b[axis])/2 if direction[axis] > 0
                        else (bounds_b[axis+2]+bounds_a[axis])/2)
            barrier = world(Point(template['barrier_point_mm']), pose)
            value = (barrier.x, barrier.y)[axis]
            check('BRIDGE_BARRIER', abs(value-boundary) <= eps, [ref], value, boundary)

    def pin_point(endpoint):
        ref, number = endpoint.split(':', 1)
        pad = next(pad for pad in actual[ref]['pads'] if pad['number'] == number)
        return world(Point(pad['center']), poses[ref])

    for rule in cfg['rules']['distance_constraints']:
        if rule.get('hard', True):
            distance = pin_point(rule['a']).distance(pin_point(rule['b']))
            check('HARD_PIN_DISTANCE', distance <= rule['max_mm']+eps,
                  [rule['a'], rule['b']], distance, rule['max_mm'])
    for rule in cfg['rules']['order_constraints']:
        a, b = pin_point(rule['a']), pin_point(rule['b'])
        separation = b.x-a.x if rule['axis'] == 'x' else b.y-a.y
        check('PIN_ORDER', separation >= rule.get('min_separation_mm', 0)-eps,
              [rule['a'], rule['b']], separation, rule.get('min_separation_mm', 0))
    parent_extents = {}
    for child, parent in assignments.get('parent_clusters', []):
        if child in regions:
            parent_extents.setdefault(parent, {'children': [], 'members': [], 'geometries': []})
            entry = parent_extents[parent]
            entry['children'].append(child)
            entry['members'].extend(groups[child])
            entry['geometries'].extend(occupancy[ref] for ref in groups[child])
    for entry in parent_extents.values():
        extent = box(*unary_union(entry.pop('geometries')).bounds)
        entry['bounds_mm'] = extent.bounds
        entry['area_mm2'] = extent.area
        entry['constraint_type'] = 'SOFT_COMPACTNESS'
    before_members, log_name, old_lines, before_records, before_docs = raw_archive(source_path)
    after_members, after_log, new_lines, after_records, after_docs = raw_archive(output_path)
    check('ZIP_MEMBER_SET', set(before_members) == set(after_members))
    for name, content in before_members.items():
        if name != log_name:
            check('NONLOG_BYTES', content == after_members[name], [name])
    check('LOG_NAME', log_name == after_log)
    check('DOCUMENT_SET', before_docs == after_docs)
    index = 0
    for line in new_lines:
        if index < len(old_lines) and line == old_lines[index]:
            index += 1
    check('ORIGINAL_LOG_LINES_PRESERVED', index == len(old_lines), actual_value=index, required=len(old_lines))
    allowed_changes = {'COMPONENT', 'ATTR', 'POLY', 'POURED'}
    changes = {}
    for key in set(before_records) | set(after_records):
        before, after = before_records.get(key), after_records.get(key)
        if before == after:
            continue
        doc, typ, identifier = key
        changes[typ] = changes.get(typ, 0)+1
        check('RECORD_CHANGE_SCOPE', doc == board_id and typ in allowed_changes, list(key))
        if typ == 'COMPONENT' and before is not None and after is not None:
            check('COMPONENT_PAYLOAD_PRESERVED', {k:v for k,v in before.items() if k not in ('x','y','positionX','positionY','angle')} ==
                  {k:v for k,v in after.items() if k not in ('x','y','positionX','positionY','angle')}, [identifier])
        if typ == 'ATTR' and before is not None and after is not None:
            coordinate_fields = {'x','y','positionX','positionY','angle'}
            check('ATTRIBUTE_PAYLOAD_PRESERVED', {k:v for k,v in before.items() if k not in coordinate_fields} ==
                  {k:v for k,v in after.items() if k not in coordinate_fields}, [identifier])
            parent = next((ref for ref,c in original.items() if c['uuid'] == before.get('parentId')), None)
            if parent:
                xkey = 'x' if 'x' in before else 'positionX'
                ykey = 'y' if 'y' in before else 'positionY'
                scale = {'mm': 1., 'mil': .0254, '0.01inch': .254, '0.01mm': .01}[norm['source_unit']]
                ox, oy = norm['origin']
                if xkey in before and ykey in before:
                    old_pose, new_pose = original[parent]['source_pose'], poses[parent]
                    dx, dy = (before[xkey]-ox)*scale-old_pose['x'], -(before[ykey]-oy)*scale-old_pose['y']
                    expected_point = affinity.rotate(Point(dx,dy), new_pose['angle']-old_pose['angle'], origin=(0,0))
                    actual_point = Point((after[xkey]-ox)*scale-new_pose['x'], -(after[ykey]-oy)*scale-new_pose['y'])
                    check('ATTRIBUTE_RIGID_POSITION', actual_point.distance(expected_point) <= eps, [parent,identifier])
                    if before.get('angle') is not None:
                        check('ATTRIBUTE_RIGID_ANGLE', angle_error(after['angle'], before['angle']-new_pose['angle']+old_pose['angle']) <= 1e-7,
                              [parent,identifier])
    net_filter = lambda records: {key: value for key,value in records.items() if key[1] in ('PAD_NET','NET')}
    check('RAW_NET_AND_STABLE_PAD_IDS', net_filter(before_records) == net_filter(after_records))
    attribute_keys = {key for key in before_records if key[1] == 'ATTR'}
    check('ATTRIBUTE_OBJECT_SET', attribute_keys == {key for key in after_records if key[1] == 'ATTR'})
    # Verify every original world-positioned label, including a label that was
    # accidentally left unchanged while its component moved.
    parent_refs = {c['uuid']: ref for ref,c in original.items()}
    source_scale = {'mm': 1., 'mil': .0254, '0.01inch': .254, '0.01mm': .01}[norm['source_unit']]
    ox, oy = norm['origin']
    for key in sorted(attribute_keys):
        if key[0] != board_id or key not in after_records:
            continue
        before, after = before_records[key], after_records[key]
        parent = parent_refs.get(before.get('parentId'))
        xkey, ykey = ('x' if 'x' in before else 'positionX'), ('y' if 'y' in before else 'positionY')
        if parent is None or before.get(xkey) is None or before.get(ykey) is None:
            continue
        old_pose, new_pose = original[parent]['source_pose'], poses[parent]
        offset = Point((before[xkey]-ox)*source_scale-old_pose['x'],
                       -(before[ykey]-oy)*source_scale-old_pose['y'])
        expected_offset = affinity.rotate(offset, new_pose['angle']-old_pose['angle'], origin=(0,0))
        actual_offset = Point((after[xkey]-ox)*source_scale-new_pose['x'],
                              -(after[ykey]-oy)*source_scale-new_pose['y'])
        check('ALL_ATTRIBUTE_ATTACHMENTS', expected_offset.distance(actual_offset) <= eps, [parent,key[2]])
    check('SOURCE_BYTES_UNCHANGED', source_before == sha256(source_path))
    finite = lambda value: value if math.isfinite(value) else None
    return {
        'status': 'PASSED' if not violations else 'FAILED',
        'method': 'Public native decoder plus independent Shapely/GEOS distances, intersections, containment and raw-log replay; no internal validator called',
        'shapely_version': shapely.__version__, 'source_sha256': source_before,
        'output_sha256': sha256(output_path), 'board_id': board_id,
        'component_count': len(actual), 'pad_count': sum(len(c['pads']) for c in actual.values()),
        'net_count': len(readback.nets), 'changed_components': moved,
        'board_width_mm': board_bounds[2]-board_bounds[0], 'board_height_mm': board_bounds[3]-board_bounds[1],
        'minimum_body_gap_mm': finite(min_body), 'minimum_checked_copper_gap_mm': finite(min_copper),
        'minimum_cross_domain_copper_gap_mm': finite(min_isolation), 'minimum_hole_keepout_mm': finite(min_hole),
        'maximum_epro2_pose_error_mm': pose_error, 'parent_extents': parent_extents,
        'checks_executed': checks, 'checks_total': sum(checks.values()),
        'changed_record_types': changes, 'violations': violations,
        'native_editor_open': 'NOT_PERFORMED', 'creepage_evaluation': 'NOT_PERFORMED',
        'safety_signoff': 'NOT_PERFORMED',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--epro2', required=True)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    report = evaluate_run(args.run, args.epro2)
    Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({key: report[key] for key in ('status', 'checks_total', 'component_count', 'violations')}, ensure_ascii=False))
    return 0 if report['status'] == 'PASSED' else 3


if __name__ == '__main__':
    raise SystemExit(main())
