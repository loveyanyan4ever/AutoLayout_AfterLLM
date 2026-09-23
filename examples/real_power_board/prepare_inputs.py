"""Input/evidence converter for the supplied digital-power snapshot only.

The known source has no repeated atomic versions; this small evidence helper
does not replace the application's complete native-log replay adapter. It never
changes project code or input archives.
"""
from pathlib import Path
import csv, json, zipfile, collections, shutil, hashlib
ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
SOURCE=ROOT/'upload'/'ProPrj_数字电源_功率板_2026-09-23.epro2'
if not SOURCE.exists(): SOURCE=OUT/'source.epro2'
CSV_SOURCE={lev:(ROOT/'upload'/f'{lev}.assignments.csv') for lev in ['L1','L2']}
CSV_SOURCE={lev:(p if p.exists() else OUT/f'{lev}.original.csv') for lev,p in CSV_SOURCE.items()}
records={};docs={};doc=None
with zipfile.ZipFile(SOURCE) as z:
    log=next(n for n in z.namelist() if n.endswith('.epru'))
    lines=z.read(log).decode('utf-8-sig').splitlines()
for line_no,line in enumerate(lines,1):
    a,b=line.split('||',1);h=json.loads(a);b=b.removesuffix('|');b=json.loads(b) if b else None
    if h['type']=='DOCHEAD': doc=b['uuid'];docs[doc]=b;continue
    k=(doc,h['type'],h.get('id'));old=records.get(k)
    if not old or h.get('ticket',0)>old[0].get('ticket',0):records[k]=(h,b,line_no)
def of(d,t):
    return [(h,b,n) for (dd,tt,k),(h,b,n) in records.items() if dd==d and tt==t and b is not None]
board=[d for d,b in docs.items() if b['docType']=='PCB'];assert len(board)==1
board=board[0];attrs={}
for h,b,n in of(board,'ATTR'):
    k=(b.get('parentId'),b.get('key')); old=attrs.get(k)
    if not old or h['ticket']>old[0]:attrs[k]=(h['ticket'],b.get('value'))
components={};evidence={};overrides={}
for h,b,n in of(board,'COMPONENT'):
    uid=h['id'];a={key:v[1] for (parent,key),v in attrs.items() if parent==uid};ref=a['Designator'];fp=a.get('Footprint');resolution='component ATTR.Footprint'
    if not fp:
        dev=a['Device'];meta=of(dev,'META');assert len(meta)==1
        fp=meta[0][1]['attributes']['Footprint'];resolution=f'Device {dev} META.attributes.Footprint'
    assert docs[fp]['docType']=='FOOTPRINT'
    components[ref]={'designator':ref,'component_uuid':uid,'footprint_uuid':fp,'footprint_title':of(fp,'META')[0][1].get('title'),
        'footprint_resolution':resolution,'source_position_mm':[b['x']*.0254,-b['y']*.0254],
        'source_angle_internal_deg':(-b.get('angle',0))%360,'source_locked':bool(b.get('locked')),'source_record_line':n}
    if fp in evidence:continue
    shape=[(sh,sb,sn) for sh,sb,sn in of(fp,'POLY') if sb.get('layerId')==48]
    if shape:
        assert len(shape)==1
        sh,sb,sn=shape[0];path=sb['path'];assert not any(isinstance(v,str) and v not in ['L','Z'] for v in path)
        coords=[v for v in path if not isinstance(v,str)];assert len(coords)%2==0
        polygon=[[round(x*.0254,8),round(-y*.0254,8)] for x,y in zip(coords[0::2],coords[1::2])]
        if polygon[-1]==polygon[0]:polygon.pop()
        overrides[fp]=polygon
        evidence[fp]={'basis':'source_component_shape_layer_48','source_poly_id':sh['id'],'source_line':sn,'polygon_mm':polygon,'added_margin_mm':0}
    elif fp=='ab13ac9220710194':
        overrides[fp]=[-4.,-4.,4.,4.]
        evidence[fp]={'basis':'explicit_research_assembly_keepout','bbox_mm':overrides[fp],'source_central_pad_diameter_mm':255.9055*.0254,'note':'M3 footprint has no layer48 body. 8x8mm conductive mechanical envelope is an explicit conservative working assumption; not a measured screw-head outline.'}
    elif fp=='a939a2d556d9b1f3':
        overrides[fp]=[-.35,-.35,.35,.35]
        evidence[fp]={'basis':'explicit_research_testpoint_access_envelope','bbox_mm':overrides[fp],'source_pad_diameter_mm':19.685*.0254,'note':'Bare 0.5mm test pad; 0.7x0.7mm access envelope is a working allowance, not a component body.'}
    else:raise RuntimeError(f'Unresolved body {ref}/{fp}')
with CSV_SOURCE['L1'].open(encoding='utf-8-sig',newline='') as f:l1=list(csv.DictReader(f))
with CSV_SOURCE['L2'].open(encoding='utf-8-sig',newline='') as f:l2=list(csv.DictReader(f))
l1={r['designator']:r for r in l1};l2={r['component_id']:r for r in l2};assert set(l1)==set(l2)==set(components)
def write_csv(name,fields,rows):
    with (OUT/name).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
write_csv('functional_clusters.csv',['designator','cluster_id','parent_cluster_id','component_uuid','cluster_name'],[
    {'designator':r,'cluster_id':l1[r]['cluster'],'parent_cluster_id':l2[r]['cluster_id'],'component_uuid':components[r]['component_uuid'],'cluster_name':l1[r]['function']} for r in sorted(components)])
write_csv('voltage_domains.csv',['designator','domain_id','component_uuid','note'],[
    {'designator':r,'domain_id':'MECH' if r in ['1','2','3','4'] else 'MAIN','component_uuid':components[r]['component_uuid'],
     'note':'Research assumption: conductive unconnected mechanical mounting' if r in ['1','2','3','4'] else 'Research common domain; L2 remains functional hierarchy; no certified HV/LV split supplied'} for r in sorted(components)])
write_csv('l2_functional_groups.csv',['designator','cluster_id','component_uuid','cluster_name'],[
    {'designator':r,'cluster_id':l2[r]['cluster_id'],'component_uuid':components[r]['component_uuid'],'cluster_name':l2[r]['function']} for r in sorted(components)])
fields=['designator','component_uuid','l1_cluster_id','l2_cluster_id','voltage_domain_l1','voltage_domain_l2','isolation_domain_l1','isolation_domain_l2','function_l1','function_l2','evidence_rule_ids_l1','evidence_rule_ids_l2']
hierarchy=[]
for ref in sorted(components):
    a,b=l1[ref],l2[ref];hierarchy.append({'designator':ref,'component_uuid':components[ref]['component_uuid'],'l1_cluster_id':a['cluster'],'l2_cluster_id':b['cluster_id'],
     **{f'{key}_{level}':row[key] for level,row in [('l1',a),('l2',b)] for key in ['voltage_domain','isolation_domain','function']},
     'evidence_rule_ids_l1':';'.join(v for v in [a['evidence_rule_ids'],a.get('','')] if v),'evidence_rule_ids_l2':b['evidence_rule_ids']})
write_csv('hierarchy.csv',fields,hierarchy)
parents={}
for ref in sorted(components):
    cid=l1[ref]['cluster'];pid=l2[ref]['cluster_id'];assert cid not in parents or parents[cid]==pid;parents[cid]=pid
for name,data in [('body_overrides.json',overrides),('body_geometry_evidence.json',evidence),('source_components.json',components),('cluster_parents.json',parents)]:
    (OUT/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if SOURCE.resolve()!=(OUT/'source.epro2').resolve(): shutil.copyfile(SOURCE,OUT/'source.epro2')
for lev in ['L1','L2']:
    if CSV_SOURCE[lev].resolve()!=(OUT/f'{lev}.original.csv').resolve(): shutil.copyfile(CSV_SOURCE[lev],OUT/f'{lev}.original.csv')
manifest={'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),'board_id':board,'component_count':len(components),'unique_footprints':len(evidence),
 'l1_cluster_count':len(parents),'l2_group_count':len(set(parents.values())),'coordinate_scale_mm_per_source_unit':.0254,
 'coordinate_scale_evidence':'C0603 COMPONENT_SHAPE=1.5999968 x 0.7999984mm; testpoint pad=0.499999mm; M3 central pad=6.4999997mm.',
 'normalization_notes':['L1 cluster renamed cluster_id','L2 component_id contains designators and is renamed designator','L1 unnamed final field merged into evidence_rule_ids_l1 with semicolon','Original source/assignments included unchanged','L2 retained as functional parents; NOT interpreted as isolation domain'],
 'assumptions':['MAIN common electrical placement domain for research; no invented galvanic isolation','MECH four conductive mounting items; mechanical material/head dimensions not independently certified','Layer48 source vertex coordinates are evidence without stroke expansion; the runtime source_body uses a conservative AABB including stroke, not a certified physical courtyard. M3 and testpoint envelopes are explicit research allowances.']}
(OUT/'input_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest,ensure_ascii=False,indent=2))
