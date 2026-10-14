"""Tests for the tote cell model, task checks and KPI computation."""

import mujoco
import numpy as np
import pytest

from tote_cell.build import build_model, tote_names
from tote_cell.metrics import EventLog, summarize
from tote_cell import tasks


@pytest.fixture(scope="module")
def cell():
    model = build_model(with_robot=True)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def test_cell_has_robot_and_totes(cell):
    model, _ = cell
    assert model.nu == 29
    for tote in tote_names():
        model.body(tote)  # raises if missing
    assert len(tote_names()) == 6


def test_robot_home_pose_on_floor(cell):
    model, data = cell
    penetration = min((data.contact[i].dist for i in range(data.ncon)), default=0.0)
    assert penetration > -1e-3
    assert 0.6 < data.body("g1/pelvis").xpos[2] < 0.85
    assert not tasks.robot_fallen(data)


def test_totes_settle_on_stacks():
    model = build_model(with_robot=False)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    start = {t: data.body(t).xpos.copy() for t in tote_names()}
    for _ in range(int(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    for tote in tote_names():
        assert np.linalg.norm(data.body(tote).xpos - start[tote]) < 0.01, tote
        assert tasks.tote_speed(data, tote) < 0.01
    assert sorted(tasks.remaining_on_stacks(model, data)) == sorted(tote_names())


def test_tote_placed_detects_zone():
    model = build_model(with_robot=False)
    data = mujoco.MjData(model)
    tote = "klt4147_2"  # top of stack A
    mujoco.mj_forward(model, data)
    assert not tasks.tote_placed(model, data, tote)

    # Teleport the tote onto the conveyor's place zone and let it settle
    jnt = model.joint(f"{tote}_free")
    zone = data.site("place_zone").xpos
    data.qpos[jnt.qposadr[0]:jnt.qposadr[0] + 3] = [zone[0], zone[1], 0.75]
    for _ in range(int(1.5 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    assert tasks.tote_placed(model, data, tote)
    assert not tasks.tote_dropped(data, tote)


def test_kpis():
    log = EventLog()
    for i in range(10):
        log.add(60.0 * i + 5, "pick_attempt", f"t{i}")
        log.add(60.0 * i + 15, "pick_success", f"t{i}")
        if i != 3:
            log.add(60.0 * i + 40, "place_success", f"t{i}")
    log.add(200.0, "drop", "t3")
    log.add(210.0, "intervention")
    kpi = summarize(log, duration_s=600.0)
    assert kpi["tote_moves_per_hour"] == pytest.approx(54.0)
    assert kpi["pick_success_rate"] == pytest.approx(1.0)
    assert kpi["pick_and_place_success_rate"] == pytest.approx(0.9)
    assert kpi["longest_autonomous_min"] == pytest.approx(390.0 / 60)
    assert kpi["drops"] == 1 and kpi["interventions"] == 1 and kpi["falls"] == 0
