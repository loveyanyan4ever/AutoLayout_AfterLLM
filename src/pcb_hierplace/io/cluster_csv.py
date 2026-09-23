"""Strict, complete two-CSV assignment contract."""
import csv
from pathlib import Path
from ..core.schema import Assignments, PlacementError


def read_assignment(path, value_key, design, parent_clusters=None):
    refs=design.by_ref
    result={}
    try:
        allowed={'designator',value_key,'component_uuid','note','cluster_name'}
        if value_key=='cluster_id': allowed.add('parent_cluster_id')
        fields=None
        with Path(path).open(encoding='utf-8-sig',newline='') as source:
            reader=csv.reader(source,strict=True)
            previous_line=0
            for cells in reader:
                rownum=previous_line+1; previous_line=reader.line_num
                if not cells or (len(cells)==1 and not cells[0].strip()) or cells[0].lstrip().startswith('#'):
                    continue
                if fields is None:
                    fields=cells
                    if len(set(fields))!=len(fields):
                        raise PlacementError('CSV_HEADER',f'{path}:{rownum} 表头含重复列')
                    if not {'designator',value_key}<=set(fields):
                        raise PlacementError('CSV_HEADER',f'{path}:{rownum} 需要 designator,{value_key}')
                    if set(fields)-allowed:
                        raise PlacementError('CSV_HEADER',f'{path}:{rownum} 未知/空列 {set(fields)-allowed}')
                    continue
                if len(cells)!=len(fields):
                    raise PlacementError('CSV_COLUMNS',f'{path}:{rownum} 列数应为 {len(fields)}，实际为 {len(cells)}',
                                         details={'line':rownum,'expected':len(fields),'actual':len(cells)})
                row=dict(zip(fields,cells))
                ref=row['designator'].strip(); value=row[value_key].strip()
                if not ref or not value or ref not in refs or ref in result:
                    raise PlacementError('CSV_ASSIGNMENT',f'{path}:{rownum} 位号缺失/未知/重复或标签为空: {ref}')
                if row.get('component_uuid') and row['component_uuid']!=refs[ref].uuid:
                    raise PlacementError('CSV_UUID',f'{path}:{rownum} UUID 与位号冲突')
                if 'parent_cluster_id' in fields:
                    parent=row['parent_cluster_id'].strip()
                    if not parent: raise PlacementError('CSV_PARENT',f'{path}:{rownum} 父功能簇不能为空')
                    if parent_clusters is not None: parent_clusters[ref]=parent
                result[ref]=value
        if fields is None:
            raise PlacementError('CSV_HEADER',f'{path}: 需要 designator,{value_key}')
        if set(result)!=set(refs):
            raise PlacementError('CSV_COVERAGE',f'{path}: 器件未完整覆盖',details={'missing':sorted(set(refs)-set(result))})
    except (OSError,UnicodeError,csv.Error) as e:
        raise PlacementError('CSV_READ',str(e)) from e
    return result


def load_assignments(design, domains_csv, clusters_csv, cfg):
    domains=read_assignment(domains_csv,'domain_id',design)
    parents={}
    logical=read_assignment(clusters_csv,'cluster_id',design,parents)
    declared=cfg['rules']['domains']
    if set(domains.values())-set(declared):
        raise PlacementError('UNKNOWN_DOMAIN',f'未声明的域: {sorted(set(domains.values())-set(declared))}')
    physical=dict(logical); changes=[]
    # Reserve original names as well as generated names. A request to split a
    # cluster never authorizes merging another logical cluster with its child.
    reserved={cid:('logical',cid) for cid in set(logical.values())}
    for cid in sorted(set(logical.values())):
        members=[r for r in logical if logical[r]==cid]
        if parents and len({parents[r] for r in members})!=1:
            raise PlacementError('PARENT_CLUSTER_CONFLICT',f'{cid} 的成员声明了不同父功能簇',
                                 details={'members':members,'parents':sorted({parents[r] for r in members})})
        ds={domains[r] for r in members if declared[domains[r]]['kind']=='electrical'}
        if len(ds)>1:
            if cfg['clusters']['cross_domain_policy']=='split_by_domain':
                for r in members:
                    generated=cid+'::'+domains[r]; owner=('split',cid,domains[r])
                    if generated in reserved and reserved[generated]!=owner:
                        raise PlacementError('CLUSTER_ID_COLLISION',f'{cid} 拆分生成的簇 ID {generated} 与已有簇冲突',
                                             details={'logical_cluster':cid,'domain':domains[r],'generated_id':generated})
                    reserved[generated]=owner; physical[r]=generated
                changes.append(f'{cid} split_by_domain: {sorted(ds)}')
            else:
                raise PlacementError('CROSS_DOMAIN_CLUSTER',f'{cid} 跨隔离域，必须显式选择 split_by_domain',details={'members':members})
    parent_map={physical[r]:parents[r] for r in physical} if parents else {}
    return Assignments(tuple(sorted(domains.items())),tuple(sorted(physical.items())),
                       tuple(sorted(logical.items())),tuple(changes),parent_clusters=tuple(sorted(parent_map.items())))
