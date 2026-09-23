"""Strict configuration loading and explicit CLI decisions; no safety defaults."""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path
import yaml
from .core.schema import PlacementError, finite_number


DEFAULTS = {
    "schema_version":1,
    "input":{"project":None,"board_id":None,"voltage_domains":None,
             "functional_clusters":None,"source_unit":None,"adapter":"reference_v3"},
    "board":{"mode":"auto","replace_existing_outline":False,"estimated":{
        "target_utilization":.55,"aspect_candidates":[1.,1.4,1.8],
        "max_width_mm":None,"max_height_mm":None,"min_width_mm":10.,
        "min_height_mm":10.,"growth_factor":1.25,"shrink_factor":.92,"size_trials":8}},
    "geometry":{"unsupported_policy":"error","missing_body_policy":"error",
                "epsilon_mm":1e-6,"output_grid_mm":.001,"solve_margin_mm":.002,
                "side_policy":"single_side","body_overrides":{},
                "source_body_policy":"explicit_only",
                "polygon_path_frame":"reject",
                "approved_pad_envelopes":[],"pad_envelope_margin_mm":0.},
    "rules":{"id":None,"basis":None,"mechanical_gap_mm":None,
             "default_pad_clearance_mm":None,"domains":{},"isolation_pairs":[],
             "creepage_evaluation":"external_required","unlock_source_locked":[],
             "component_rules":{},"bridge_templates":{},"obstacles":[],
             "regions":{},"net_weights":{},"distance_constraints":[],
             "order_constraints":[]},
    "clusters":{"cross_domain_policy":"error","missing_assignment_policy":"error",
                "containment":"hard","anchored_members_policy":"attachment","max_candidates":4,
                "parent_compact_weight":.05},
    "optimization":{"seed":0,"continuous_backend":"numpy_analytic_float64",
                    "projection_backend":"osqp","optimizer":"projected_adam",
                    "rotations":"per_component","allow_layer_change":False,
                    "max_ab_feedback_rounds":3,"max_topology_candidates":40,
                    "time_budget_s":600.,
                    "priority_policy":"auto",
                    "higher_priority_relative_tolerance":0.,"iterations_a":60,
                    "enable_stage_c":True,
                    "iterations_b":100,"iterations_c":80,"learning_rate_mm":.15,
                    "gamma_start_mm":2.,"gamma_end_mm":.1,
                    "shape_aspects":[.7,1.,1.4,2.],"domain_split_fractions":[.4,.5,.6],
                    "preplace_samples":5,"beam_width":6,"stage_c":{
                        "trust_radius_mm":.5,"cluster_area_growth_ratio":.05,
                        "allow_rotation":False,"allow_cross_cluster_move":False}},
    "export":{"require_model_feasible":True,"overwrite_source":False,"emit_epro2":False,
              "native_open_validation":"pending"},
}

DYNAMIC = {"geometry.body_overrides","rules.domains","rules.component_rules",
           "rules.bridge_templates","rules.regions","rules.net_weights"}
ENTRY_KEYS = {
    "rules.domains":{"kind","between"},
    "rules.component_rules":{"mode","pose","allowed_angles","allowed_edges","edge_segment_mm",
       "local_mating_point_mm","local_outward_vector","edge_offset_mm","allow_body_overhang",
       "insertion_keepout","region_id","bridge_template","pad_domains","conductive_class",
       "keepout_geometry","allowed_corners","corner_inset_mm"},
    "rules.bridge_templates":{"pad_domains","axis_local","between","allowed_angles",
       "keepout_geometry","barrier_point_mm"},
}
LIST_KEYS = {
    "rules.isolation_pairs":{"domains","copper_clearance_mm","creepage_mm","routing_reserve_mm"},
    "rules.obstacles":{"id","bbox_mm","layers"},
    "rules.distance_constraints":{"id","a","b","max_mm","target_mm","hard","weight"},
    "rules.order_constraints":{"id","a","b","axis","min_separation_mm","hard"},
}


def merge_strict(base, values, prefix=""):
    if not isinstance(values,dict):
        raise PlacementError("CONFIG_TYPE",f"{prefix or '配置'} 应为映射")
    out=copy.deepcopy(base)
    for key,value in values.items():
        if key not in base:
            raise PlacementError("UNKNOWN_CONFIG_KEY",f"未知配置项 {prefix+key}")
        path=prefix+key
        if path in DYNAMIC:
            if not isinstance(value,dict):
                raise PlacementError("CONFIG_TYPE",f"{path} 应为映射")
            if path in ENTRY_KEYS:
                for name,entry in value.items():
                    if not isinstance(entry,dict) or set(entry)-ENTRY_KEYS[path]:
                        raise PlacementError("UNKNOWN_CONFIG_KEY",f"{path}.{name} 含未知字段或不是映射")
            out[key]=copy.deepcopy(value)
        elif isinstance(base[key],dict):
            out[key]=merge_strict(base[key],value,path+".")
        else:
            if path in LIST_KEYS:
                if not isinstance(value,list) or any(not isinstance(e,dict) or set(e)-LIST_KEYS[path] for e in value):
                    raise PlacementError("UNKNOWN_CONFIG_KEY",f"{path} 列表含未知字段")
            out[key]=copy.deepcopy(value)
    return out


def set_value(data, expression):
    if "=" not in expression:
        raise PlacementError("SET_SYNTAX","--set 使用 path=value（value 为 YAML/JSON 值）")
    path,value=expression.split("=",1)
    keys=path.split(".")
    node=data
    for k in keys[:-1]:
        if k not in node: node[k]={}
        if not isinstance(node[k],dict):
            raise PlacementError("SET_SYNTAX",f"{path}: 父节点不是映射")
        node=node[k]
    if any(not k for k in keys):
        raise PlacementError("SET_SYNTAX", "--set 的配置路径不得包含空字段")
    try:
        parsed=yaml.safe_load(value)
    except yaml.YAMLError as e:
        raise PlacementError("SET_VALUE", f"{path}: 无效的 YAML/JSON 值") from e
    node[keys[-1]]=parsed


def load_config(path, overrides=()):
    p=Path(path)
    try:
        raw=yaml.safe_load(p.read_text(encoding="utf-8-sig")) or {}
        for expr in overrides: set_value(raw,expr)
        cfg=merge_strict(DEFAULTS,raw)
    except (OSError,yaml.YAMLError,ValueError) as e:
        raise PlacementError("CONFIG_READ",str(e)) from e
    for k in ("project","voltage_domains","functional_clusters"):
        if cfg['input'][k]:
            q=Path(cfg['input'][k]).expanduser()
            cfg['input'][k]=str((p.parent/q).resolve()) if not q.is_absolute() else str(q.resolve())
    return cfg


def required_decisions(cfg):
    fields=["input.project","input.voltage_domains","input.functional_clusters","input.source_unit",
            "rules.id","rules.basis","rules.mechanical_gap_mm","rules.default_pad_clearance_mm"]
    missing=[]
    for path in fields:
        node=cfg
        for k in path.split('.'): node=node.get(k) if isinstance(node,dict) else None
        if node is None or node=="": missing.append(path)
    if not cfg['rules']['domains']: missing.append('rules.domains')
    for i,pair in enumerate(cfg['rules']['isolation_pairs']):
        for key in ('copper_clearance_mm','creepage_mm','routing_reserve_mm'):
            if pair.get(key) is None: missing.append(f"rules.isolation_pairs[{i}].{key}")
    if cfg['board']['mode']=='estimated':
        for key in ('max_width_mm','max_height_mm'):
            if cfg['board']['estimated'][key] is None: missing.append('board.estimated.'+key)
    return missing


def check_config(cfg):
    def types(node,template,path=''):
        for key,base in template.items():
            value=node[key];name=path+key
            if isinstance(base,bool) and not isinstance(value,bool):
                raise PlacementError('CONFIG_TYPE',f'{name} 必须为 true/false')
            if isinstance(base,list) and not isinstance(value,list):
                raise PlacementError('CONFIG_TYPE',f'{name} 必须为列表')
            if isinstance(base,dict) and name not in DYNAMIC: types(value,base,name+'.')
    types(cfg,DEFAULTS)
    for ref,rule in cfg['rules']['component_rules'].items():
        if 'allow_body_overhang' in rule and not isinstance(rule['allow_body_overhang'],bool):
            raise PlacementError('CONFIG_TYPE',f'{ref}.allow_body_overhang 必须为 true/false')
    for rule in cfg['rules']['distance_constraints']+cfg['rules']['order_constraints']:
        if 'hard' in rule and not isinstance(rule['hard'],bool):
            raise PlacementError('CONFIG_TYPE','hard 必须为 true/false')
    missing=required_decisions(cfg)
    if missing:
        raise PlacementError('DECISIONS_REQUIRED','请通过配置文件、configure 向导或 --set 提供必填决策',
                             details={'missing':missing})
    choices={
        'input.adapter':['reference_v3','easyeda_pro_v3'], 'input.source_unit':['mil','mm','0.01inch','0.01mm'],
        'board.mode':['auto','fixed','estimated'],
        'geometry.unsupported_policy':['error'],'geometry.missing_body_policy':['error'],
        'geometry.side_policy':['single_side'],
        'geometry.source_body_policy':['explicit_only','source_courtyard_or_assembly'],
        'geometry.polygon_path_frame':['reject','footprint','pad_local','conservative_union'],
        'clusters.cross_domain_policy':['error','split_by_domain','bridge_template'],
        'clusters.missing_assignment_policy':['error'],'clusters.containment':['hard'],
        'clusters.anchored_members_policy':['attachment'],
        'rules.creepage_evaluation':['external_required'],
        'optimization.continuous_backend':['numpy_analytic_float64'],
        'optimization.projection_backend':['osqp'],
        'optimization.optimizer':['projected_adam','projected_gd'],
        'optimization.rotations':['per_component'],
        'optimization.priority_policy':['auto','feasibility_intent_wirelength_shape_displacement',
                                        'feasibility_intent_area_wirelength_displacement'],
        'export.native_open_validation':['pending'],
    }
    for path,allowed in choices.items():
        v=cfg
        for k in path.split('.'): v=v[k]
        if v not in allowed:
            raise PlacementError('CONFIG_ENUM',f'{path} 仅支持 {allowed}, 得到 {v!r}')
    for path,expected in [('optimization.allow_layer_change',False),
          ('optimization.stage_c.allow_rotation',False),('optimization.stage_c.allow_cross_cluster_move',False),
          ('export.require_model_feasible',True),('export.overwrite_source',False)]:
        node=cfg
        for k in path.split('.'): node=node[k]
        if node is not expected: raise PlacementError('UNSUPPORTED_CONFIG',f'{path} 必须为 {expected}')
    # Numeric validation runs recursively; null is only permitted in unneeded templates.
    for section,keys in [('geometry',['epsilon_mm','output_grid_mm','solve_margin_mm']),
                         ('rules',['mechanical_gap_mm','default_pad_clearance_mm']),
                         ('optimization',['time_budget_s','learning_rate_mm','gamma_start_mm','gamma_end_mm'])]:
        for k in keys:
            x=finite_number(cfg[section][k],f'{section}.{k}')
            if x<0 or (section!='rules' and x==0):
                raise PlacementError('CONFIG_RANGE',f'{section}.{k} 超出范围')
    o=cfg['optimization']
    for k in ['seed','max_ab_feedback_rounds','max_topology_candidates','iterations_a','iterations_b',
              'iterations_c','preplace_samples','beam_width']:
        x=o[k]
        if isinstance(x,bool) or not isinstance(x,int) or x<0 or (k not in ['seed','max_ab_feedback_rounds'] and x==0):
            raise PlacementError('CONFIG_RANGE',f'optimization.{k} 必须为有效整数')
    if cfg['geometry']['solve_margin_mm'] < cfg['geometry']['output_grid_mm']:
        raise PlacementError('CONFIG_RANGE','solve_margin_mm 不得小于输出网格')
    for key in ['trust_radius_mm','cluster_area_growth_ratio']:
        if finite_number(o['stage_c'][key],key)<0: raise PlacementError('CONFIG_RANGE',key)
    if not 0<=finite_number(o['higher_priority_relative_tolerance'],'tolerance')<1:
        raise PlacementError('CONFIG_RANGE','高层级相对预算必须在 [0,1)')
    if not 0<cfg['board']['estimated']['target_utilization']<1:
        raise PlacementError('CONFIG_RANGE','target_utilization 必须在 (0,1)')
    if cfg['schema_version']!=1: raise PlacementError('SCHEMA_VERSION','仅支持 schema_version: 1')
    if isinstance(cfg['clusters']['max_candidates'],bool) or not isinstance(cfg['clusters']['max_candidates'],int) or cfg['clusters']['max_candidates']<1:
        raise PlacementError('CONFIG_RANGE','max_candidates 必须为正整数')
    estimated=cfg['board']['estimated']
    for k in ['min_width_mm','min_height_mm','growth_factor','shrink_factor']:
        if finite_number(estimated[k],k)<=0: raise PlacementError('CONFIG_RANGE',k)
    if estimated['growth_factor']<=1 or not 0<estimated['shrink_factor']<1:
        raise PlacementError('CONFIG_RANGE','growth_factor>1 且 0<shrink_factor<1')
    if not isinstance(estimated['size_trials'],int) or estimated['size_trials']<1:
        raise PlacementError('CONFIG_RANGE','size_trials 必须为正整数')
    for key in ['max_width_mm','max_height_mm']:
        if estimated[key] is not None and finite_number(estimated[key],key)<=0:
            raise PlacementError('CONFIG_RANGE',key)
    for name,values,lower,upper in [('aspect_candidates',estimated['aspect_candidates'],0,float('inf')),
            ('shape_aspects',o['shape_aspects'],0,float('inf')),
            ('domain_split_fractions',o['domain_split_fractions'],0,1)]:
        if not isinstance(values,list) or not values or any(not lower<finite_number(v,name)<upper for v in values):
            raise PlacementError('CONFIG_RANGE',name)
    if finite_number(cfg['geometry']['pad_envelope_margin_mm'],'pad_envelope_margin_mm')<0:
        raise PlacementError('CONFIG_RANGE','pad_envelope_margin_mm 不得为负')
    if finite_number(cfg['clusters']['parent_compact_weight'],'parent_compact_weight')<0:
        raise PlacementError('CONFIG_RANGE','parent_compact_weight 不得为负')
    return cfg
