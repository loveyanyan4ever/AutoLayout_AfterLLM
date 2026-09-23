"""CLI decisions are explicit. A noninteractive command never waits on stdin."""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import sys
import yaml
from .config import DEFAULTS,load_config,required_decisions,set_value,merge_strict
from .core.schema import PlacementError,Status,state_from_dict,file_hash,digest
from .io.epro2 import ProjectArchive
from .pipeline import load_problem,run
from .opt.ranking import evaluate
from .reporting import write_json


def parser():
    p=argparse.ArgumentParser(prog='pcb-hierplace',description='约束驱动的 PCB 分层布局；未知设计参数必须由用户明确提供。')
    sub=p.add_subparsers(dest='command',required=True)
    ins=sub.add_parser('inspect',help='只检查工程文档与板清单'); ins.add_argument('--project',required=True)
    con=sub.add_parser('configure',help='创建规则模板，可通过 --interactive 逐项输入设计决策')
    con.add_argument('--project',required=True); con.add_argument('--out',required=True)
    con.add_argument('--source-unit',choices=['mil','mm','0.01inch','0.01mm'])
    con.add_argument('--interactive',action='store_true'); con.add_argument('--set',action='append',default=[])
    demo=sub.add_parser('demo',help='创建明确标记为合成数据的可运行示例'); demo.add_argument('--out',required=True)
    demo.add_argument('--estimated',action='store_true')
    for name in ['validate','run']:
        q=sub.add_parser(name); q.add_argument('--config',required=True); q.add_argument('--set',action='append',default=[])
        if name=='run': q.add_argument('--out',required=True)
    for name in ['evaluate','export']:
        q=sub.add_parser(name); q.add_argument('--run',required=True)
        if name=='export': q.add_argument('--out',required=True)
    q=sub.add_parser('verify-native',help='用户在立创EDA打开并另存后，核对另存文件')
    q.add_argument('--run',required=True);q.add_argument('--project',required=True)
    q.add_argument('--opened-and-saved-in-editor',action='store_true')
    return p


def configure(args):
    cfg=copy.deepcopy(DEFAULTS); cfg['input']['project']=str(Path(args.project).resolve())
    boards=ProjectArchive(args.project).boards()
    if len(boards)==1: cfg['input']['board_id']=boards[0]['board_id']
    cfg['input']['source_unit']=args.source_unit
    for expression in args.set: set_value(cfg,expression)
    cfg=merge_strict(DEFAULTS,cfg)
    if args.interactive:
        if not sys.stdin.isatty(): raise PlacementError('TTY_REQUIRED','--interactive 需要真实终端；自动化请用 --set 或配置文件')
        fields=required_decisions(cfg)+['rules.isolation_pairs','geometry.body_overrides',
                                    'rules.component_rules','rules.bridge_templates']
        if len(boards)!=1: fields.insert(0,'input.board_id')
        for field in fields:
            if '[' in field: continue
            print(f'请输入 {field} 的 YAML/JSON 值；回车保持当前值，不会猜测：')
            answer=input('> ').strip()
            if answer: set_value(cfg,field+'='+answer)
    cfg=merge_strict(DEFAULTS,cfg)
    remaining=required_decisions(cfg)
    out=Path(args.out)
    if out.exists(): raise PlacementError('CONFIG_EXISTS','不覆盖已有配置，请选择新路径')
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text('# 真实规则必须由设计者填写；null 不会被当作 0。\n'+yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False),encoding='utf-8')
    return {'config':str(out),'remaining_decisions':remaining,
            'next':'填写本体几何、器件规则和桥接模板后运行 validate；错误会指出缺失对象。'}


def load_run(root):
    root=Path(root)
    cfg=json.loads((root/'effective_config.json').read_text(encoding='utf-8'))
    manifest=json.loads((root/'run_manifest.json').read_text(encoding='utf-8'))
    if digest(cfg)!=manifest['config_hash']: raise PlacementError('CONFIG_CHANGED','运行配置哈希不一致')
    for key in ['project','voltage_domains','functional_clusters']:
        q=Path(cfg['input'][key])
        cfg['input'][key]=str((root/q).resolve()) if not q.is_absolute() else str(q)
    for key,field in [('project','source_hash'),('voltage_domains','domains_hash'),('functional_clusters','clusters_hash')]:
        if file_hash(cfg['input'][key])!=manifest[field]: raise PlacementError('INPUT_CHANGED',f'{key} 已改变')
    problem=load_problem(cfg)
    payload=json.loads((root/'placements.json').read_text(encoding='utf-8'))
    if payload['status']!='FEASIBLE': raise PlacementError('EXPORT_NOT_FEASIBLE','此运行无可交付布局')
    state=state_from_dict(payload['state'])
    if state.digest!=manifest['placement_hash']: raise PlacementError('PLACEMENT_CHANGED','布局文件哈希不一致')
    return problem,evaluate(problem,state)


def main(argv=None):
    args=parser().parse_args(argv)
    exit_code=0
    try:
        if args.command=='inspect':
            archive=ProjectArchive(args.project)
            version=str(archive.project_metadata.get('editorVersion','unknown'))
            result={'boards':archive.boards(),
                    'adapter':'easyeda_pro_v3' if version.startswith('3.') else 'reference_v3',
                    'source_editor_version':version,'atomic_tombstones':len(archive.tombstones),
                    'validation_scope':'archive_and_board_inventory_only',
                    'native_open_validation':'PENDING'}
        elif args.command=='configure': result=configure(args)
        elif args.command=='demo':
            from .demo import create_demo
            result=create_demo(args.out,estimated=args.estimated)
        elif args.command=='validate':
            problem=load_problem(load_config(args.config,args.set))
            result={'status':'VALIDATED','components':len(problem.design.components),'groups':problem.groups,
                    'source_hash':problem.design.source_hash,'estimated':problem.estimated,
                    'note':'输入规则有效；不表示源布局已合法。'}
        elif args.command=='run': result=run(load_config(args.config,args.set),args.out)
        elif args.command=='evaluate':
            _,record=load_run(args.run)
            feasible=record.validation.model_feasible
            result={'status':'FEASIBLE' if feasible else 'VALIDATION_FAILED',
                    'metrics':record.metrics,'validation':record.validation.as_dict()}
            exit_code=0 if feasible else 3
        elif args.command=='export':
            from .io.export import export_project
            problem,record=load_run(args.run); result=export_project(problem,record,args.out)
        elif args.command=='verify-native':
            from dataclasses import replace
            from .io.epro2 import parse_project
            from .core.geometry import bbox,angle_equal
            from .constraints.validator import validate_placement
            from .io.export import compare_geometry
            problem,record=load_run(args.run)
            readback=parse_project(args.project,problem.design.board_id,
                {'source_unit':problem.design.source_unit,'geometry':problem.cfg['geometry']})
            if readback.nets!=problem.design.nets or {(c.uuid,c.ref) for c in readback.components}!={(c.uuid,c.ref) for c in problem.design.components}:
                raise PlacementError('NATIVE_OBJECTS','编辑器另存后对象/网络连接发生变化')
            compare_geometry(problem.design,readback,problem.eps)
            expected=record.state
            if max(abs(a-b) for a,b in zip(bbox(readback.outline),bbox(expected.outline)))>problem.eps:
                raise PlacementError('NATIVE_OUTLINE','编辑器另存后板框变化')
            for c in readback.components:
                a,b=c.source_pose,expected.by_ref[c.ref]
                if abs(a.x-b.x)>problem.eps or abs(a.y-b.y)>problem.eps or not angle_equal(a.angle,b.angle):
                    raise PlacementError('NATIVE_POSE',f'{c.ref} 另存后位姿变化')
            report=validate_placement(problem,replace(expected,poses=tuple(c.source_pose for c in readback.components)))
            if not report.model_feasible: raise PlacementError('NATIVE_VALIDATION','另存文件未通过约束检查',details=report.as_dict())
            result={'native_open_validation':'PASSED' if args.opened_and_saved_in_editor else 'PENDING',
                    'readback_validation':'PASSED','user_attested_editor_open':args.opened_and_saved_in_editor,
                    'editor_saved_file_hash':file_hash(args.project)}
            write_json(Path(args.run)/'native_validation.json',result)
        print(json.dumps(result if args.command!='run' else {k:result[k] for k in ['status','final','elapsed_s']},ensure_ascii=False,indent=2))
        return exit_code
    except PlacementError as e:
        print(json.dumps(e.as_dict(),ensure_ascii=False,indent=2),file=sys.stderr)
        return 3 if e.status in (Status.NO_FEASIBLE_FOUND,Status.MODEL_INFEASIBLE,Status.PROVEN_INFEASIBLE) else 4 if e.status==Status.NUMERICAL_FAILURE else 2
    except (OSError,ValueError,KeyError,TypeError,yaml.YAMLError) as e:
        print(json.dumps({'status':'INPUT_INVALID','code':'INPUT_READ','message':str(e)},ensure_ascii=False),file=sys.stderr)
        return 2
