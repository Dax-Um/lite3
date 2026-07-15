from __future__ import annotations

import importlib.util
from pathlib import Path

from lite3_behavior.patrol_events import PatrolEvent, PatrolState
from lite3_behavior.patrol_fsm import PatrolContext, PatrolFSM
from lite3_common.types import Twist2D


ZERO = Twist2D(0.0, 0.0, 0.0)
SCRIPT_PATH = Path(__file__).resolve().parents[3] / "scripts" / "run_patrol_fsm_dry_run.py"


def load_dry_run_script():
    spec = importlib.util.spec_from_file_location("run_patrol_fsm_dry_run", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_start_goes_idle_init_move():
    fsm = PatrolFSM()

    fsm.handle_event(PatrolEvent.PATROL_START)
    assert fsm.state() is PatrolState.INIT
    assert fsm.tick(now=0.0) == ZERO
    assert fsm.state() is PatrolState.MOVE_ALONG_LANE


def test_move_outputs_directional_vx():
    fsm = PatrolFSM(PatrolContext(direction=-1, patrol_speed_mps=0.08))
    fsm.handle_event(PatrolEvent.PATROL_START)
    fsm.tick(now=0.0)

    assert fsm.tick(now=0.1) == Twist2D(-0.08, 0.0, 0.0)


def test_lane_end_outputs_stop_then_shift():
    fsm = started_fsm()

    fsm.handle_event(PatrolEvent.LANE_END)

    assert fsm.state() is PatrolState.END_OF_LANE
    assert fsm.tick(now=0.1) == ZERO
    assert fsm.state() is PatrolState.SHIFT_TO_NEXT_LANE


def test_side_shift_outputs_vy():
    fsm = shifted_fsm()

    assert fsm.tick(now=0.2) == Twist2D(0.0, 0.04, 0.0)


def test_turn_outputs_wz():
    fsm = shifted_fsm()

    fsm.handle_event(PatrolEvent.SIDE_SHIFT_DONE)

    assert fsm.state() is PatrolState.TURN_AROUND
    assert fsm.tick(now=0.3) == Twist2D(0.0, 0.0, 0.15)


def test_turn_done_flips_direction():
    fsm = shifted_fsm()
    fsm.handle_event(PatrolEvent.SIDE_SHIFT_DONE)

    fsm.handle_event(PatrolEvent.TURN_DONE)

    assert fsm.state() is PatrolState.MOVE_ALONG_LANE
    assert fsm.context().lane_index == 1
    assert fsm.context().direction == -1


def test_max_lane_finishes():
    fsm = shifted_fsm(PatrolContext(max_lane_count=1))
    fsm.handle_event(PatrolEvent.SIDE_SHIFT_DONE)

    fsm.handle_event(PatrolEvent.TURN_DONE)

    assert fsm.state() is PatrolState.FINISH
    assert fsm.tick(now=0.4) == ZERO


def test_return_home_interrupts_every_active_state():
    for state in active_states():
        fsm = PatrolFSM()
        force_state(fsm, state)

        fsm.handle_event(PatrolEvent.RETURN_HOME)

        assert fsm.state() is PatrolState.PAUSE_AND_RETURN_HOME
        assert fsm.tick(now=1.0) == ZERO
        assert fsm.state() is PatrolState.RETURN_HOME
        assert fsm.tick(now=1.1) == ZERO


def test_return_done_returns_to_idle():
    fsm = PatrolFSM()
    force_state(fsm, PatrolState.RETURN_HOME)

    fsm.handle_event(PatrolEvent.RETURN_DONE)

    assert fsm.state() is PatrolState.IDLE


def test_emergency_stop_interrupts_every_state():
    for state in PatrolState:
        fsm = PatrolFSM()
        force_state(fsm, state)

        fsm.handle_event(PatrolEvent.EMERGENCY_STOP)

        assert fsm.state() is PatrolState.ERROR
        assert fsm.tick(now=2.0) == ZERO


def test_reset_from_finish_resets_context():
    fsm = shifted_fsm(PatrolContext(max_lane_count=1))
    fsm.handle_event(PatrolEvent.SIDE_SHIFT_DONE)
    fsm.handle_event(PatrolEvent.TURN_DONE)

    fsm.handle_event(PatrolEvent.RESET)

    assert fsm.state() is PatrolState.IDLE
    assert fsm.context().lane_index == 0
    assert fsm.context().direction == 1


def test_dry_run_log_matches_two_lane_sequence():
    script = load_dry_run_script()

    lines = script.run_dry_run().splitlines()

    assert lines[0] == "time,state,lane_index,direction,vx,vy,wz,event"
    assert lines[1:] == [
        "0.0,init,0,1,0.000,0.000,0.000,patrol_start",
        "0.1,move_along_lane,0,1,0.080,0.000,0.000,tick",
        "0.2,end_of_lane,0,1,0.000,0.000,0.000,lane_end",
        "0.3,shift_to_next_lane,0,1,0.000,0.040,0.000,tick",
        "0.4,turn_around,0,1,0.000,0.000,0.150,side_shift_done",
        "0.5,move_along_lane,1,-1,-0.080,0.000,0.000,turn_done",
        "0.6,end_of_lane,1,-1,0.000,0.000,0.000,lane_end",
        "0.7,shift_to_next_lane,1,-1,0.000,0.040,0.000,tick",
        "0.8,turn_around,1,-1,0.000,0.000,0.150,side_shift_done",
        "0.9,finish,2,1,0.000,0.000,0.000,turn_done",
    ]


def started_fsm(context: PatrolContext | None = None) -> PatrolFSM:
    fsm = PatrolFSM(context)
    fsm.handle_event(PatrolEvent.PATROL_START)
    fsm.tick(now=0.0)
    return fsm


def shifted_fsm(context: PatrolContext | None = None) -> PatrolFSM:
    fsm = started_fsm(context)
    fsm.handle_event(PatrolEvent.LANE_END)
    fsm.tick(now=0.1)
    return fsm


def active_states() -> tuple[PatrolState, ...]:
    return (
        PatrolState.INIT,
        PatrolState.MOVE_ALONG_LANE,
        PatrolState.END_OF_LANE,
        PatrolState.SHIFT_TO_NEXT_LANE,
        PatrolState.TURN_AROUND,
        PatrolState.PAUSE_AND_RETURN_HOME,
        PatrolState.RETURN_HOME,
    )


def force_state(fsm: PatrolFSM, state: PatrolState) -> None:
    fsm._state = state
