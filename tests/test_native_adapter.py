"""Regression cases independent of any private real-board file."""
import json
import math
import zipfile
from pathlib import Path
import numpy as np
import pytest
from pcb_hierplace.core.geometry import validate_convex,validate_simple,bbox
from pcb_hierplace.core.schema import PlacementError
from pcb_hierplace.io.epro2 import ProjectArchive,parse_project,layer_types
from pcb_hierplace.io.native_geometry import pad_geometry,oval_envelope


def archive(tmp_path,lines):
    path=tmp_path/'records.epro2'
    with zipfile.ZipFile(path,'w') as z:z.writestr('records.epru','\n'.join(lines)+'\n')
    return path


def doc(client='a'):
    return json.dumps({'type':'DOCHEAD'})+'||'+json.dumps({'uuid':'b','docType':'PCB','client':client})+'|'


def rec(kind,key,ticket,body):
    return json.dumps({'type':kind,'id':key,'ticket':ticket})+'||'+body+'|'


@pytest.mark.parametrize('empty',['','""'])
def test_highest_ticket_empty_body_is_tombstone_without_revival(tmp_path,empty):
    lines=[doc(),rec('COMPONENT','x',1,'{"x":1}'),rec('COMPONENT','x',4,empty),rec('COMPONENT','x',2,'{"x":2}')]
    path=archive(tmp_path,lines);a=ProjectArchive(path)
    assert a.of('b','COMPONENT')==[] and len(a.tombstones)==1
    assert a.max_ticket==4
    assert a.members[a.log_name].decode()=='\n'.join(lines)+'\n'


def test_higher_ticket_can_restore_an_atomic_record(tmp_path):
    a=ProjectArchive(archive(tmp_path,[doc(),rec('NET','n',4,''),rec('NET','n',5,'{"name":"N"}')]))
    assert a.of('b','NET')[0].inner=={'name':'N'} and not a.tombstones


def test_equal_ticket_uses_document_client_and_is_order_independent(tmp_path):
    lines=[doc('z'),rec('NET','n',9,'{"name":"z"}'),doc('a'),rec('NET','n',9,'{"name":"a"}')]
    assert ProjectArchive(archive(tmp_path,lines)).of('b','NET')[0].inner['name']=='a'
    lines=[doc('a'),rec('NET','n',9,'{"name":"a"}'),doc('z'),rec('NET','n',9,'{"name":"z"}')]
    assert ProjectArchive(archive(tmp_path,lines)).of('b','NET')[0].inner['name']=='a'


def test_equal_ticket_same_client_conflict_is_not_hidden(tmp_path):
    with pytest.raises(PlacementError,match='同 ticket/client'):
        ProjectArchive(archive(tmp_path,[doc(),rec('NET','n',9,'{}'),rec('NET','n',9,'{"name":"x"}')]))


def test_json_null_is_not_mistaken_for_a_defined_tombstone(tmp_path):
    with pytest.raises(PlacementError) as e:ProjectArchive(archive(tmp_path,[doc(),rec('NET','n',9,'null')]))
    assert e.value.code=='LOG_PAYLOAD'


@pytest.mark.parametrize('constant',['NaN','Infinity','-Infinity','1e999'])
def test_nonstandard_json_numeric_constants_are_rejected_in_attributes(tmp_path,constant):
    path=archive(tmp_path,[doc(),rec('ATTR','a',1,'{"key":"Designator","x":'+constant+',"y":1}')])
    with pytest.raises(PlacementError) as e:ProjectArchive(path)
    assert e.value.code=='NONFINITE_JSON' and '日志行 2 内层' in str(e.value)


def test_nonstandard_json_constant_in_project_metadata_is_rejected(tmp_path):
    path=tmp_path/'metadata.epro2'
    with zipfile.ZipFile(path,'w') as z:
        z.writestr('project2.json','{"editorVersion":"3.2.121","custom":NaN}')
        z.writestr('records.epru',doc()+'\n')
    with pytest.raises(PlacementError) as e:ProjectArchive(path)
    assert e.value.code=='NONFINITE_JSON' and 'project2.json' in str(e.value)


def test_literal_nan_text_is_preserved_as_valid_json_string(tmp_path):
    a=ProjectArchive(archive(tmp_path,[doc(),rec('ATTR','a',1,'{"value":"NaN / Infinity"}')]))
    assert a.of('b','ATTR')[0].inner['value']=='NaN / Infinity'


def test_document_deletion_excludes_board_and_later_restore_recovers_it(tmp_path):
    lines=[doc(),rec('COMPONENT','x',1,'{"x":1}'),rec('DELETE_DOC',None,5,'{"isDelete":true}')]
    a=ProjectArchive(archive(tmp_path,lines))
    assert a.boards()==[] and a.of('b')==[] and a.deleted_docs=={'b'}
    assert len(a.records)==2  # original records remain available for lossless export
    a=ProjectArchive(archive(tmp_path,lines+[rec('DELETE_DOC',None,6,'{"isDelete":false}')]))
    assert len(a.boards())==1 and len(a.of('b','COMPONENT'))==1


def test_deleting_one_board_does_not_delete_other_board(tmp_path):
    second=json.dumps({'type':'DOCHEAD'})+'||'+json.dumps({'uuid':'other','docType':'PCB','client':'a'})+'|'
    a=ProjectArchive(archive(tmp_path,[doc(),rec('DELETE_DOC',None,2,'{"isDelete":1}'),second,rec('META','META',1,'{}')]))
    assert [b['board_id'] for b in a.boards()]==['other']


def test_unknown_document_delete_shape_is_rejected(tmp_path):
    with pytest.raises(PlacementError) as e:ProjectArchive(archive(tmp_path,[doc(),rec('DELETE_DOC',None,2,'{"isDelete":"false"}')]))
    assert e.value.code=='DELETE_SEMANTICS'


def mutate_project(path,fn):
    with zipfile.ZipFile(path) as z:members={n:z.read(n) for n in z.namelist()}
    name=next(n for n in members if n.endswith('.epru'));rows=[]
    for line in members[name].decode().splitlines():
        h,b=line.split('||',1);head=json.loads(h);body=json.loads(b[:-1] if b.endswith('|') else b)
        extra=fn(head,body)
        rows.append(json.dumps(head)+'||'+json.dumps(body)+'|')
        if extra:rows.extend(extra)
    members[name]=('\n'.join(rows)+'\n').encode()
    with zipfile.ZipFile(path,'w') as z:
        for n,d in members.items():z.writestr(n,d)


def parsed(cfg):
    return parse_project(cfg['input']['project'],cfg['input']['board_id'],{'source_unit':cfg['input']['source_unit'],'geometry':cfg['geometry']})


@pytest.mark.parametrize('replacement',['does_not_exist','chip_p2'])
def test_full_pad_identity_is_cross_checked(fixture_config,replacement):
    def change(h,b):
        if h['type']=='PAD_NET':
            rid=json.loads(h['id'])
            if rid[1:3]==['id_Q1','1']:rid[3]=replacement;h['id']=json.dumps(rid)
    mutate_project(fixture_config['input']['project'],change)
    with pytest.raises(PlacementError) as e:parsed(fixture_config)
    assert e.value.code=='UNKNOWN_PAD_NET'
    assert e.value.details['unknown_pad_net_identities']==[('id_Q1','1',replacement)]


@pytest.mark.parametrize('rid',[['PAD_NET','id_Q1','1'],['WRONG','id_Q1','1','chip_p1']])
def test_pad_net_structure_is_validated(fixture_config,rid):
    def change(h,b):
        if h['type']=='PAD_NET' and json.loads(h['id'])[1:3]==['id_Q1','1']:h['id']=json.dumps(rid)
    mutate_project(fixture_config['input']['project'],change)
    with pytest.raises(PlacementError) as e:parsed(fixture_config)
    assert e.value.code=='PAD_NET_ID'


def test_repeated_pad_number_preserves_distinct_stable_entities(fixture_config):
    def change(h,b):
        if h['type']=='PAD' and h['id']=='chip_p2':b['num']='1'
        if h['type']=='PAD_NET':
            rid=json.loads(h['id'])
            if rid[3]=='chip_p2':rid[2]='1';h['id']=json.dumps(rid)
    mutate_project(fixture_config['input']['project'],change)
    q=parsed(fixture_config).by_ref['Q1']
    assert [p.number for p in q.pads]==['1','1']
    assert {p.id for p in q.pads}=={'chip_p1','chip_p2'}
    assert {p.net for p in q.pads}=={'H_IN','H_SIGNAL'}


def test_footprint_via_uses_full_instance_pad_net_identity(fixture_config):
    def change(h,b):
        if h['type']=='PAD' and h['id']=='chip_p2':
            return [rec('VIA','v1',10,json.dumps({'centerX':0,'centerY':0,'viaDiameter':10,'holeDiameter':5,'netName':'','ruleName':''}))]
        if h['type']=='COMPONENT' and h['id']=='id_Q1':
            return [rec('PAD_NET',json.dumps(['PAD_NET','id_Q1','v1','v1']),10,'{"padNet":"H_SIGNAL"}')]
    mutate_project(fixture_config['input']['project'],change)
    d=parsed(fixture_config);p=next(p for p in d.by_ref['Q1'].pads if p.id=='v1')
    assert p.number=='v1' and p.net=='H_SIGNAL' and p.layer=='both' and p.hole_polygon
    assert ('Q1','v1') in next(n for n in d.nets if n.id=='H_SIGNAL').pins


def test_orphaned_poured_data_preserved_but_active_parent_rejected(fixture_config):
    def orphan(h,b):
        if h['type']=='COMPONENT' and h['id']=='id_Q1':return [rec('POURED',json.dumps(['POURED','missing-parent']),20,'{"pourFill":[]}')]
    mutate_project(fixture_config['input']['project'],orphan)
    d=parsed(fixture_config)
    assert dict(d.metadata)['orphaned_pour_results_preserved']=='1'
    a=ProjectArchive(fixture_config['input']['project']);assert len(a.of(d.board_id,'POURED'))==1
    def active(h,b):
        if h['type']=='COMPONENT' and h['id']=='id_Q1':return [rec('POUR','missing-parent',20,'{}')]
    mutate_project(fixture_config['input']['project'],active)
    with pytest.raises(PlacementError) as e:parsed(fixture_config)
    assert e.value.code=='ROUTED_INPUT_UNSUPPORTED'


def test_integer_layer_ids_are_supported(tmp_path):
    a=ProjectArchive(archive(tmp_path,[doc(),rec('LAYER',11,1,'{"layerType":"OUTLINE"}')]))
    assert layer_types(a,'b')=={11:'OUTLINE'}


def test_native_outline_is_identified_by_layer(fixture_config):
    def change(h,b):
        if h['type']=='POLY' and b.get('polyType')=='BOARD_OUTLINE':b['polyType']='NORMAL'
    mutate_project(fixture_config['input']['project'],change)
    assert parsed(fixture_config).outline_present


def test_official_position_aliases_are_supported_without_silent_conflict(fixture_config):
    before=parsed(fixture_config)
    def change(h,b):
        if h['type']=='COMPONENT':b['positionX']=b.pop('x');b['positionY']=b.pop('y')
    mutate_project(fixture_config['input']['project'],change)
    assert tuple(c.source_pose for c in parsed(fixture_config).components)==tuple(c.source_pose for c in before.components)
    def conflict(h,b):
        if h['type']=='COMPONENT':b['x']=b['positionX']+1
    mutate_project(fixture_config['input']['project'],conflict)
    with pytest.raises(PlacementError) as e:parsed(fixture_config)
    assert e.value.code=='COMPONENT_POSITION'


@pytest.mark.parametrize('ellipse,width,height',[(True,6,2),(False,6,2),(False,2,6)])
def test_curve_envelopes_contain_analytic_boundary(ellipse,width,height):
    shapely=pytest.importorskip('shapely.geometry')
    p=shapely.Polygon(oval_envelope(width,height,ellipse))
    for t in np.linspace(0,2*np.pi,300):
        v=np.array([math.cos(t),math.sin(t)])
        if ellipse:point=v*[width/2,height/2]
        else:
            point=v*min(width,height)/2
            point+=np.array([(width-height)/2*np.sign(v[0]),0]) if width>=height else np.array([0,(height-width)/2*np.sign(v[1])])
        assert p.buffer(1e-10).covers(shapely.Point(point))


def test_hole_offsets_do_not_move_copper_and_slots_are_retained():
    payload={'centerX':10,'centerY':5,'padAngle':90,'defaultPad':{'padType':'RECT','width':8,'height':6},
             'padOffsetX':2,'padOffsetY':1,'relativeAngle':90,'hole':{'holeType':'SLOT','width':3,'height':1}}
    copper,center,_,_,hole,hcenter=pad_geometry(payload,1)
    assert center==(10,-5) and bbox(copper)==pytest.approx((7,-9,13,-1))
    assert hcenter==pytest.approx((9,-7)) and hole


def test_polygon_ambiguity_requires_policy_and_union_covers_both():
    p={'centerX':4,'centerY':3,'padAngle':0,'defaultPad':{'padType':'POLYGON','path':[0,0,'L',2,0,2,1,0,1,0,0]}}
    with pytest.raises(PlacementError) as e:pad_geometry(p,1)
    assert e.value.code=='POLYGON_FRAME_REQUIRED'
    geom=pytest.importorskip('shapely.geometry')
    a=geom.Polygon(pad_geometry(p,1,'footprint')[0]);b=geom.Polygon(pad_geometry(p,1,'pad_local')[0])
    u=geom.Polygon(pad_geometry(p,1,'conservative_union')[0])
    assert u.covers(a) and u.covers(b)


@pytest.mark.parametrize('poly',[
    [(math.cos(2*math.pi*i/5),math.sin(2*math.pi*i/5)) for i in [0,2,4,1,3]],
    [(0,0),(2,2),(0,2),(2,0)],
    [(0,0),(2,0),(2,2),(0,0),(0,2)],
    [(0,0,1),(2,0,1),(0,2,1)],
])
def test_invalid_polygons_are_rejected_before_sat(poly):
    with pytest.raises(PlacementError):validate_convex(poly)


def test_valid_clockwise_closed_convex_and_concave_simple_polygon():
    validate_convex([(0,0),(0,1),(1,1),(1,0),(0,0)])
    p=[(0,0),(2,0),(1,1),(2,2),(0,2)]
    validate_simple(p)
    with pytest.raises(PlacementError) as e:validate_convex(p)
    assert e.value.code=='NONCONVEX_BODY'


@pytest.mark.parametrize('conflicting_attribute_alias',[False,True])
def test_export_checks_all_position_aliases_for_components_and_attributes(solved,tmp_path,conflicting_attribute_alias):
    import copy
    from pcb_hierplace.pipeline import load_problem
    from pcb_hierplace.opt.ranking import evaluate
    from pcb_hierplace.io.export import export_project
    _,original,record,_=solved
    a=ProjectArchive(original.design.source_path);lines=list(a.lines)
    c=next(r for r in a.of('pcb_demo','COMPONENT') if r.outer['id']=='id_C1')
    attr=next(r for r in a.of('pcb_demo','ATTR') if r.inner.get('parentId')=='id_C1' and r.inner.get('key')=='Designator')
    for r,is_component in [(c,True),(attr,False)]:
        b=dict(r.inner)
        if not is_component:b.update(x=c.inner['x']+20,y=c.inner['y']-30,angle=45.)
        b.update(positionX=b['x'],positionY=b['y'])
        if not is_component and conflicting_attribute_alias:b['positionX']+=1
        lines[r.line]=json.dumps(r.outer)+'||'+json.dumps(b)+'|\n'
    source=tmp_path/'both_aliases.epro2'
    with zipfile.ZipFile(source,'w') as z:
        for n,data in a.members.items():z.writestr(n,''.join(lines).encode() if n==a.log_name else data)
    before=source.read_bytes();cfg=copy.deepcopy(original.cfg);cfg['input']['project']=str(source)
    problem=load_problem(cfg);target=tmp_path/'both_aliases_placed.epro2'
    if conflicting_attribute_alias:
        with pytest.raises(PlacementError) as caught:export_project(problem,evaluate(problem,record.state),target)
        assert caught.value.code=='ATTR_POSITION'
        assert not target.exists() and not target.with_suffix('.export.json').exists()
        assert source.read_bytes()==before
        return
    result=export_project(problem,evaluate(problem,record.state),target)
    after=ProjectArchive(target)
    assert result['roundtrip_validation']=='PASSED'
    for typ,rid in [('COMPONENT','id_C1'),('ATTR',attr.outer['id'])]:
        b=next(r.inner for r in after.of('pcb_demo',typ) if r.outer['id']==rid)
        assert b['x']==b['positionX'] and b['y']==b['positionY']
        assert b['x']!=c.inner['x']
    assert source.read_bytes()==before
