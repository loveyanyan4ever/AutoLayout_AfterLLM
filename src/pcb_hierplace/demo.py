"""Fully synthetic dual-domain fixture. Distances are test constants, not safety advice."""
from __future__ import annotations
import copy
import csv
import json
from pathlib import Path
import zipfile
import yaml
from .config import DEFAULTS
from .core.schema import PlacementError


def create_demo(destination,estimated=False,source_unit='mil'):
    root=Path(destination)
    if root.exists() and any(root.iterdir()): raise PlacementError('DEMO_EXISTS','示例目录非空，请选新目录')
    root.mkdir(parents=True,exist_ok=True)
    scale={'mil':.0254,'mm':1.}[source_unit]; origin=(1234.,5678.)
    lines=[]
    def line(head,body): lines.append(json.dumps(head,separators=(',',':'))+'||'+json.dumps(body,separators=(',',':'))+'|\n')
    def doc(kind,uid): line({'type':'DOCHEAD'},{'docType':kind,'uuid':uid,'client':'synthetic-test'})
    def rec(kind,uid,body,ticket=1): line({'type':kind,'id':uid,'ticket':ticket},body)
    parts=[
        ('H1','hole',3,3,'MECHANICAL','mechanical',True),
        ('H2','hole',57,37,'MECHANICAL','mechanical',True),
        ('JH','connector',2,20,'HV','input',False),
        ('JL','connector',58,20,'LV','output',False),
        ('ISO1','bridge',30,20,'BRIDGE','isolation',False),
        ('Q1','chip',14,18,'HV','switch',False),
        ('C1','passive',14.1,18.1,'HV','switch',False),
        ('R1','passive',20,28,'HV','sense',False),
        ('U1','chip',44,20,'LV','control',False),
        ('C2','passive',44.1,20.1,'LV','control',False),
        ('R2','passive',48,10,'LV','filter',False),
    ]
    footprints={
        'hole':[],
        'connector':[('1',0,0,.8,.8)],
        'bridge':[('1',-3.2,0,.6,.8),('2',3.2,0,.6,.8)],
        'chip':[('1',-1.8,0,.6,.8),('2',1.8,0,.6,.8)],
        'passive':[('1',-.8,0,.6,.8),('2',.8,0,.6,.8)],
    }
    body={'hole':[-1,-1,1,1],'connector':[-2,-3,2,3],'bridge':[-4,-2,4,2],
          'chip':[-1.5,-1.5,1.5,1.5],'passive':[-.7,-.5,.7,.5]}
    for fp,pads in footprints.items():
        doc('FOOTPRINT',fp); rec('META','META',{'title':fp})
        for num,x,y,w,h in pads:
            rec('PAD',fp+'_p'+num,{'num':num,'centerX':x/scale,'centerY':-y/scale,'padAngle':0,
                'layerId':1,'hole':None,'defaultPad':{'padType':'RECT','width':w/scale,'height':h/scale},
                'specialPad':[],'padOffsetX':0,'padOffsetY':0})
    doc('PCB','pcb_demo'); rec('META','META',{'title':'Synthetic isolation fixture'})
    rec('CANVAS','CANVAS',{'originX':origin[0],'originY':origin[1],'unit':'mm'})
    for n,t in [(1,'TOP'),(2,'BOTTOM'),(11,'OUTLINE'),(12,'MULTI')]:
        rec('LAYER',json.dumps(['LAYER',n]),{'layerType':t,'layerName':t})
    if not estimated:
        path=[]
        for i,(x,y) in enumerate([(0,0),(60,0),(60,40),(0,40),(0,0)]):
            if i:path.append('L')
            path.extend([origin[0]+x/scale,origin[1]-y/scale])
        rec('POLY','outline',{'polyType':'BOARD_OUTLINE','layerId':11,'path':path,'locked':True})
    nets={'JH:1':'H_IN','Q1:1':'H_IN','Q1:2':'H_SIGNAL','C1:1':'H_SIGNAL','C1:2':'H_RETURN',
          'R1:1':'H_SIGNAL','R1:2':'H_RETURN','ISO1:1':'H_SIGNAL',
          'ISO1:2':'L_SIGNAL','U1:1':'L_SIGNAL','U1:2':'L_OUT','C2:1':'L_OUT',
          'C2:2':'L_RETURN','R2:1':'L_OUT','R2:2':'L_RETURN','JL:1':'L_OUT'}
    for ref,fp,x,y,domain,cid,locked in parts:
        rec('COMPONENT','id_'+ref,{'x':origin[0]+x/scale,'y':origin[1]-y/scale,'angle':0,'layerId':1,'locked':locked})
        for key,val in [('Designator',ref),('Footprint',fp)]:
            rec('ATTR',ref+'_'+key,{'parentId':'id_'+ref,'key':key,'value':val})
        for num,*_ in footprints[fp]:
            rec('PAD_NET',json.dumps(['PAD_NET','id_'+ref,num,fp+'_p'+num]),{'padNet':nets.get(ref+':'+num,'')})
    with zipfile.ZipFile(root/'board.epro2','w',zipfile.ZIP_DEFLATED) as z:
        z.writestr('project2.json',json.dumps({'title':'SYNTHETIC TEST ONLY','editorVersion':'reference-fixture'}))
        z.writestr('design.epru',''.join(lines))
    for filename,column,index in [('voltage_domains.csv','domain_id',4),('functional_clusters.csv','cluster_id',5)]:
        with (root/filename).open('w',encoding='utf-8',newline='') as f:
            w=csv.writer(f);w.writerow(['designator',column]);w.writerows((r[0],r[index]) for r in parts)
    cfg=copy.deepcopy(DEFAULTS)
    cfg['input'].update(project='board.epro2',board_id='pcb_demo',source_unit=source_unit,
        voltage_domains='voltage_domains.csv',functional_clusters='functional_clusters.csv')
    cfg['geometry']['body_overrides']=body
    cfg['board']['mode']='estimated' if estimated else 'fixed'
    cfg['board']['estimated'].update(max_width_mm=120,max_height_mm=100)
    cfg['rules'].update(id='synthetic-test-v1',basis='Synthetic geometry test; not a safety standard',
        mechanical_gap_mm=.2,default_pad_clearance_mm=.15,
        domains={'HV':{'kind':'electrical'},'LV':{'kind':'electrical'},'MECHANICAL':{'kind':'mechanical'},
                 'BRIDGE':{'kind':'bridge','between':['HV','LV']}},
        isolation_pairs=[{'domains':['HV','LV'],'copper_clearance_mm':3.,'creepage_mm':3.,'routing_reserve_mm':.4}],
        component_rules={
            'H1':{'mode':'fixed_pose','pose':'source','conductive_class':'insulating'},
            'H2':{'mode':'fixed_pose','pose':'source','conductive_class':'insulating'},
            'JH':{'mode':'edge_slide','allowed_angles':[0.], 'allowed_edges':['left'],'edge_segment_mm':[8.,32.],
                  'local_mating_point_mm':[-2.,0.],'local_outward_vector':[-1.,0.],'edge_offset_mm':0.,
                  'insertion_keepout':[-2.,-3.,2.,3.],'allow_body_overhang':False},
            'JL':{'mode':'edge_slide','allowed_angles':[0.], 'allowed_edges':['right'],'edge_segment_mm':[8.,32.],
                  'local_mating_point_mm':[2.,0.],'local_outward_vector':[1.,0.],'edge_offset_mm':0.,
                  'insertion_keepout':[-2.,-3.,2.,3.],'allow_body_overhang':False},
            'ISO1':{'mode':'region_bounded','region_id':'isolation_interface','bridge_template':'iso', 'allowed_angles':[0.,90.,180.,270.]},
            **{r[0]:{'allowed_angles':[0.,90.,180.,270.]} for r in parts if r[0][0] in ['Q','C','R','U']}
        },
        bridge_templates={'iso':{'pad_domains':{'1':'HV','2':'LV'},'axis_local':[1.,0.],
                                  'between':['HV','LV'],'barrier_point_mm':[0.,0.]}})
    cfg['optimization'].update(iterations_a=25,iterations_b=30,iterations_c=25,max_topology_candidates=9,
        max_ab_feedback_rounds=1,time_budget_s=120,preplace_samples=3,beam_width=2)
    if estimated: cfg['optimization']['priority_policy']='feasibility_intent_area_wirelength_displacement'
    (root/'constraints.yaml').write_text('# SYNTHETIC TEST ONLY: distances are not safety recommendations.\n'+yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False),encoding='utf-8')
    return {'directory':str(root),'config':str(root/'constraints.yaml'),'synthetic':True,'estimated':estimated}
