"""Independent regressions for public commands and multi-document export."""
from dataclasses import replace
import copy
import json
import math
import os
import shutil
import zipfile

import pytest

from pcb_hierplace.cli import main
from pcb_hierplace.core.schema import PlacementError, state_from_dict
from pcb_hierplace.io.epro2 import ProjectArchive, parse_project
from pcb_hierplace.io.export import export_project
from pcb_hierplace.opt.ranking import evaluate
from pcb_hierplace.pipeline import load_problem


def test_evaluate_invalid_geometry_has_nonzero_exit(solved, tmp_path, capsys):
    root, _, _, _ = solved
    run_dir = tmp_path / 'invalid_run'
    shutil.copytree(root / 'run', run_dir)
    payload_path = run_dir / 'placements.json'
    payload = json.loads(payload_path.read_text())
    for pose in payload['state']['poses']:
        if pose['ref'] == 'Q1':
            pose['x'] = -10.0
    payload_path.write_text(json.dumps(payload))
    manifest_path = run_dir / 'run_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    # Keep the provenance internally consistent so geometry, not hash integrity,
    # is what this public CLI test exercises.
    manifest['placement_hash'] = state_from_dict(payload['state']).digest
    manifest_path.write_text(json.dumps(manifest))
    capsys.readouterr()
    assert main(['evaluate', '--run', str(run_dir)]) == 3
    result = json.loads(capsys.readouterr().out)
    assert result['status'] == 'VALIDATION_FAILED'
    assert not result['validation']['model_feasible']
    assert any(v['rule'] == 'BOARD_BODY' for v in result['validation']['violations'])


@pytest.mark.parametrize('expression', ['rules={broken', 'geometry=[1, 2', 'rules=[]',
                                        'geometry=[]', 'rules.isolation_pairs=[1]'])
def test_configure_bad_yaml_is_structured_error(fixture_config, tmp_path, capsys, expression):
    target = tmp_path / 'bad.yaml'
    assert main(['configure', '--project', fixture_config['input']['project'],
                 '--out', str(target), '--set', expression]) == 2
    captured = capsys.readouterr()
    error = json.loads(captured.err)
    assert error['status'] == 'INPUT_INVALID'
    assert 'Traceback' not in captured.err
    assert not target.exists()


def test_export_refuses_existing_sidecar_without_changing_bytes(solved, tmp_path):
    _, problem, record, _ = solved
    target = tmp_path / 'placed.epro2'
    sidecar = target.with_suffix('.export.json')
    sentinel = b'previous independent evaluation\x00\xff\n'
    sidecar.write_bytes(sentinel)
    source_bytes = open(problem.design.source_path, 'rb').read()
    with pytest.raises(PlacementError) as caught:
        export_project(problem, record, target)
    assert caught.value.code == 'EXPORT_OVERWRITE'
    assert not target.exists()
    assert sidecar.read_bytes() == sentinel
    assert open(problem.design.source_path, 'rb').read() == source_bytes


def test_complete_second_board_and_shared_footprints_preserved(solved, tmp_path):
    _, original_problem, record, _ = solved
    archive = ProjectArchive(original_problem.design.source_path)
    log = archive.members[archive.log_name].decode('utf-8-sig')
    lines = log.splitlines(keepends=True)
    start = next(i for i, line in enumerate(lines)
                 if json.loads(line.split('||', 1)[0]).get('type') == 'DOCHEAD'
                 and json.loads(line.split('||', 1)[1][:-2]).get('uuid') == 'pcb_demo')
    duplicate = list(lines[start:])
    head, body = duplicate[0].split('||', 1)
    document = json.loads(body[:-2])
    document['uuid'] = 'pcb_second_complete'
    duplicate[0] = head + '||' + json.dumps(document, separators=(',', ':')) + '|\n'
    # All component IDs, all PAD_NET stable IDs, all attributes, canvas and
    # outline are retained in the second document, which reuses the same
    # footprint documents. IDs are intentionally board scoped.
    second_document_bytes = ''.join(duplicate).encode()
    source = tmp_path / 'two_boards.epro2'
    with zipfile.ZipFile(source, 'w', zipfile.ZIP_DEFLATED) as output:
        for name, data in archive.members.items():
            output.writestr(name, data + second_document_bytes if name == archive.log_name else data)
    original_bytes = source.read_bytes()
    cfg = copy.deepcopy(original_problem.cfg)
    cfg['input']['project'] = str(source)
    problem = load_problem(cfg)
    normalized = {'source_unit': problem.design.source_unit, 'geometry': cfg['geometry']}
    before = parse_project(source, 'pcb_second_complete', normalized)
    assert len(before.components) == len(problem.design.components) == 11
    assert before.nets == problem.design.nets
    assert {c.footprint for c in before.components} == {c.footprint for c in problem.design.components}
    target = tmp_path / 'selected_board_placed.epro2'
    result = export_project(problem, evaluate(problem, record.state), target)
    assert result['roundtrip_validation'] == 'PASSED'
    after = parse_project(target, 'pcb_second_complete', normalized)
    assert after.components == before.components
    assert after.nets == before.nets
    assert after.outline == before.outline
    output_archive = ProjectArchive(target)
    assert output_archive.members[archive.log_name].endswith(second_document_bytes)
    assert {k: r.inner for k, r in archive.records.items() if k[0] != 'pcb_demo'} == {
        k: r.inner for k, r in output_archive.records.items()
        if k[0] not in ('pcb_demo', 'pcb_second_complete')}
    for name, data in archive.members.items():
        if name != archive.log_name:
            assert output_archive.members[name] == data
    assert source.read_bytes() == original_bytes


@pytest.mark.parametrize('coordinate_keys', [('x', 'y'), ('positionX', 'positionY')])
def test_world_attribute_follows_component_and_tickets_exceed_tombstones(solved, tmp_path, coordinate_keys):
    _, original_problem, record, _ = solved
    archive = ProjectArchive(original_problem.design.source_path)
    lines = list(archive.lines)
    rec = next(r for r in archive.of('pcb_demo', 'ATTR') if r.inner['parentId'] == 'id_C1'
               and r.inner['key'] == 'Designator')
    source_component = next(r for r in archive.of('pcb_demo', 'COMPONENT') if r.outer['id'] == 'id_C1')
    a = dict(rec.inner)
    xkey, ykey = coordinate_keys
    a.update({xkey: source_component.inner['x']+20., ykey: source_component.inner['y']-30.,
              'angle': 45., 'fontSize': 7.5, 'visible': True})
    lines[rec.line] = json.dumps(rec.outer)+'||'+json.dumps(a)+'|\n'
    lines.append('{"type":"LINE","id":"qa_deleted","ticket":900000}|||\n')
    source = tmp_path / 'attributes.epro2'
    with zipfile.ZipFile(source, 'w', zipfile.ZIP_DEFLATED) as output:
        for name, data in archive.members.items():
            output.writestr(name, ''.join(lines).encode() if name == archive.log_name else data)
    cfg = copy.deepcopy(original_problem.cfg)
    cfg['input']['project'] = str(source)
    problem = load_problem(cfg)
    target = tmp_path / 'attributes_placed.epro2'
    report = export_project(problem, evaluate(problem, record.state), target)
    after = ProjectArchive(target)
    rewritten = next(r.inner for r in after.of('pcb_demo', 'ATTR') if r.outer['id'] == rec.outer['id'])
    old_pose, new_pose = problem.design.by_ref['C1'].source_pose, record.state.by_ref['C1']
    delta = math.radians(new_pose.angle-old_pose.angle)
    scale = .0254
    dx, dy = 20.*scale, 30.*scale
    expected_x = new_pose.x + math.cos(delta)*dx-math.sin(delta)*dy
    expected_y = new_pose.y + math.sin(delta)*dx+math.cos(delta)*dy
    ox, oy = problem.design.origin
    assert rewritten[xkey] == pytest.approx(expected_x/scale+ox, abs=1e-8)
    assert rewritten[ykey] == pytest.approx(-expected_y/scale+oy, abs=1e-8)
    assert rewritten['angle'] == pytest.approx((45.-new_pose.angle+old_pose.angle)%360, abs=1e-8)
    assert {k:v for k,v in rewritten.items() if k not in (xkey,ykey,'angle')} == {
        k:v for k,v in a.items() if k not in (xkey,ykey,'angle')}
    assert rec.outer['id'] in report['updated_attributes']
    original_lines = set(lines)
    added = [json.loads(line.split('||', 1)[0]) for line in after.lines if line not in original_lines]
    assert added
    assert all(head['ticket'] > 900000 for head in added)
    assert len({head['ticket'] for head in added}) == len(added)


def test_second_publish_failure_rolls_back_only_own_project_link(tmp_path, monkeypatch):
    from pcb_hierplace.io.export import _publish_pair
    project_tmp, report_tmp = tmp_path/'tmp.epro2', tmp_path/'tmp.json'
    project, report = tmp_path/'output.epro2', tmp_path/'output.export.json'
    project_tmp.write_bytes(b'new-project')
    report_tmp.write_bytes(b'new-report')
    real_link = os.link
    calls = []

    def competing_writer(src, dst):
        calls.append(dst)
        if len(calls) == 2:
            report.write_bytes(b'concurrent-evaluation-do-not-overwrite')
            raise FileExistsError('simulated competing report writer')
        return real_link(src, dst)

    monkeypatch.setattr(os, 'link', competing_writer)
    with pytest.raises(PlacementError) as caught:
        _publish_pair(project_tmp, project, report_tmp, report)
    assert caught.value.code == 'EXPORT_COMMIT'
    assert not project.exists()
    assert report.read_bytes() == b'concurrent-evaluation-do-not-overwrite'
    assert project_tmp.read_bytes() == b'new-project'
    assert report_tmp.read_bytes() == b'new-report'
