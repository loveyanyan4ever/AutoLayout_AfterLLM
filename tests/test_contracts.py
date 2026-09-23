from dataclasses import replace
import copy
import json
from pathlib import Path
import zipfile
import pytest
from pcb_hierplace.config import merge_strict,DEFAULTS,check_config
from pcb_hierplace.core.schema import PlacementError
from pcb_hierplace.pipeline import load_problem
from pcb_hierplace.io.epro2 import ProjectArchive,parse_project
from pcb_hierplace.demo import create_demo


def test_unknown_config_is_error():
    with pytest.raises(PlacementError,match='未知配置项'): merge_strict(DEFAULTS,{'optimization':{'lerning_rate':1}})


def test_safety_parameters_never_default(fixture_config):
    fixture_config['rules']['mechanical_gap_mm']=None
    with pytest.raises(PlacementError) as ex: check_config(fixture_config)
    assert ex.value.code=='DECISIONS_REQUIRED'


def test_no_source_unit_guess(fixture_config):
    with pytest.raises(PlacementError) as e:
        parse_project(fixture_config['input']['project'],None,{'geometry':fixture_config['geometry']})
    assert e.value.code=='SOURCE_UNIT_REQUIRED'


def test_source_locks_not_silently_overridden(fixture_config):
    fixture_config['rules']['component_rules']['H1']['mode']='free'
    with pytest.raises(PlacementError) as e:load_problem(fixture_config)
    assert e.value.code=='LOCK_CONFLICT'


def test_bridge_mapping_must_cover_pads(fixture_config):
    fixture_config['rules']['bridge_templates']['iso']['pad_domains'].pop('2')
    with pytest.raises(PlacementError) as e: load_problem(fixture_config)
    assert e.value.code=='BRIDGE_PAD_DOMAINS'


def test_body_geometry_required(fixture_config):
    fixture_config['geometry']['body_overrides']={}
    with pytest.raises(PlacementError) as e: load_problem(fixture_config)
    assert e.value.code=='BODY_GEOMETRY_REQUIRED'


@pytest.mark.parametrize('change',['unknown','duplicate','missing'])
def test_csv_strict(fixture_config,change):
    path=Path(fixture_config['input']['voltage_domains']); lines=path.read_text().splitlines()
    if change=='unknown': lines.append('TYPO,HV')
    elif change=='duplicate':lines.append(lines[1])
    else:lines.pop()
    path.write_text('\n'.join(lines))
    with pytest.raises(PlacementError):load_problem(fixture_config)


def test_units_are_physical(tmp_path):
    from pcb_hierplace.config import load_config
    create_demo(tmp_path/'mm',source_unit='mm');create_demo(tmp_path/'mil',source_unit='mil')
    a=load_problem(load_config(tmp_path/'mm'/'constraints.yaml')).design
    b=load_problem(load_config(tmp_path/'mil'/'constraints.yaml')).design
    for x,y in zip(a.components,b.components):
        assert x.body==y.body
        assert x.source_pose.x==pytest.approx(y.source_pose.x)
        for p,q in zip(x.pads,y.pads): assert p.center==pytest.approx(q.center)


def mutate_archive(path,mutator):
    with zipfile.ZipFile(path) as z: members={n:z.read(n) for n in z.namelist()}
    name=next(n for n in members if n.endswith('.epru'))
    members[name]=mutator(members[name].decode()).encode()
    with zipfile.ZipFile(path,'w') as z:
        for n,d in members.items():z.writestr(n,d)


def test_unknown_delete_does_not_revive_objects(fixture_config):
    path=fixture_config['input']['project']
    mutate_archive(path,lambda s:s+'{"type":"DELETE","id":"id_Q1","ticket":99}||{}|\n')
    with pytest.raises(PlacementError) as e:ProjectArchive(path)
    assert e.value.code=='DELETE_SEMANTICS'


def test_routed_board_refused(fixture_config):
    mutate_archive(fixture_config['input']['project'],lambda s:s+'{"type":"VIA","id":"v","ticket":2}||{"x":1,"y":2}|\n')
    with pytest.raises(PlacementError) as e:load_problem(fixture_config)
    assert e.value.code=='ROUTED_INPUT_UNSUPPORTED'
