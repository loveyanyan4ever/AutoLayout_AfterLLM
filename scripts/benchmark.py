"""Reproducible full / A+B / no-feedback / single-topology experiments.

Uses the same geometry and rules in every variant. Never removes a failed case
from the success-rate denominator. This is a runner, not a performance claim.
"""
import argparse
import copy
import csv
import statistics
from pathlib import Path
from pcb_hierplace.config import load_config
from pcb_hierplace.core.schema import PlacementError
from pcb_hierplace.pipeline import run
from pcb_hierplace.reporting import write_json,environment


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True)
    p.add_argument('--out',required=True)
    p.add_argument('--seeds',type=int,nargs='+',default=[0,1,2,3,4])
    p.add_argument('--variants',nargs='+',choices=['full','ab','without_feedback','single_topology'],default=['full','ab'])
    args=p.parse_args();out=Path(args.out)
    if out.exists() and any(out.iterdir()):p.error('输出目录必须不存在或为空')
    out.mkdir(parents=True,exist_ok=True)
    baseline=load_config(args.config);rows=[]
    for variant in dict.fromkeys(args.variants):
        for seed in dict.fromkeys(args.seeds):
            cfg=copy.deepcopy(baseline);cfg['optimization']['seed']=seed
            if variant in ('ab','without_feedback'):cfg['optimization']['max_ab_feedback_rounds']=0
            if variant=='ab':cfg['optimization']['enable_stage_c']=False
            if variant=='single_topology':cfg['optimization']['max_topology_candidates']=1
            row={'variant':variant,'seed':seed,'status':'','hpwl_mm':None,'weighted_hpwl_mm':None,
                 'board_area_mm2':None,'min_copper_distance_mm':None,'elapsed_s':None,'error_code':''}
            try:
                r=run(cfg,out/f'{variant}_seed_{seed}',progress=lambda s:None)
                row.update(status=r['status'],hpwl_mm=r['final']['hpwl_mm'],
                    weighted_hpwl_mm=r['final']['hpwl_weighted_mm'],board_area_mm2=r['final']['board_area_mm2'],
                    min_copper_distance_mm=r['validation']['min_copper_distance_mm'],elapsed_s=r['elapsed_s'])
            except PlacementError as e:
                row.update(status=str(e.status),error_code=e.code)
            rows.append(row)
            print(f'{variant} seed={seed}: {row["status"]}',flush=True)
    with (out/'results.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    groups={}
    for variant in dict.fromkeys(args.variants):
        all_rows=[r for r in rows if r['variant']==variant];ok=[r for r in all_rows if r['status']=='FEASIBLE']
        values=[r['weighted_hpwl_mm'] for r in ok]
        groups[variant]={'total':len(all_rows),'feasible':len(ok),'success_rate':len(ok)/len(all_rows),
            'hpwl_best_mm':min(values) if values else None,'hpwl_median_mm':statistics.median(values) if values else None,
            'hpwl_population_stdev_mm':statistics.pstdev(values) if values else None}
    write_json(out/'summary.json',{'results':groups,'environment':environment(),
        'note':'HPWL statistics use feasible cases; success-rate denominator includes all attempts. Same wall-time cap is a maximum, not equal consumed time. Flat-gradient and priority-removal baselines are not implemented.'})
    return 0


if __name__=='__main__':raise SystemExit(main())
