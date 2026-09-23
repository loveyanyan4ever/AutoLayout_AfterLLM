import pytest
from pcb_hierplace.demo import create_demo
from pcb_hierplace.config import load_config
from pcb_hierplace.pipeline import load_problem,run
from pcb_hierplace.cli import load_run


@pytest.fixture
def fixture_config(tmp_path):
    create_demo(tmp_path/'fixture')
    return load_config(tmp_path/'fixture'/'constraints.yaml')


@pytest.fixture
def problem(fixture_config):
    return load_problem(fixture_config)


@pytest.fixture(scope='session')
def solved(tmp_path_factory):
    p=tmp_path_factory.mktemp('solved')
    create_demo(p/'fixture')
    cfg=load_config(p/'fixture'/'constraints.yaml')
    cfg['optimization'].update(iterations_a=8,iterations_b=10,iterations_c=8,
        max_topology_candidates=3,max_ab_feedback_rounds=1,time_budget_s=120)
    report=run(cfg,p/'run',progress=lambda _:None)
    problem,record=load_run(p/'run')
    return p,problem,record,report
