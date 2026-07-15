"""Mapless patrol finite state machine."""

from __future__ import annotations

from dataclasses import dataclass, replace

from lite3_behavior.patrol_events import PatrolEvent, PatrolState
from lite3_common.types import Twist2D


ZERO_TWIST = Twist2D(0.0, 0.0, 0.0)
ACTIVE_STATES = {
    PatrolState.INIT,
    PatrolState.MOVE_ALONG_LANE,
    PatrolState.END_OF_LANE,
    PatrolState.SHIFT_TO_NEXT_LANE,
    PatrolState.TURN_AROUND,
    PatrolState.PAUSE_AND_RETURN_HOME,
    PatrolState.RETURN_HOME,
}


@dataclass
class PatrolContext:
    lane_index: int = 0
    direction: int = 1
    max_lane_count: int = 2
    lane_spacing_m: float = 0.5
    patrol_speed_mps: float = 0.08
    side_shift_speed_mps: float = 0.04
    turn_speed_radps: float = 0.15


class PatrolFSM:
    def __init__(self, context: PatrolContext | None = None):
        self._initial_context = replace(context) if context is not None else PatrolContext()
        self._context = replace(self._initial_context)
        self._state = PatrolState.IDLE

    def handle_event(self, event: PatrolEvent) -> None:
        if event is PatrolEvent.EMERGENCY_STOP:
            self._state = PatrolState.ERROR
            return

        if event is PatrolEvent.RESET:
            if self._state is PatrolState.FINISH:
                self._context = replace(self._initial_context)
            self._state = PatrolState.IDLE
            return

        if event is PatrolEvent.RETURN_HOME and self._state in ACTIVE_STATES:
            self._state = PatrolState.PAUSE_AND_RETURN_HOME
            return

        if self._state is PatrolState.IDLE and event is PatrolEvent.PATROL_START:
            self._state = PatrolState.INIT
            return

        if self._state is PatrolState.MOVE_ALONG_LANE and event is PatrolEvent.LANE_END:
            self._state = PatrolState.END_OF_LANE
            return

        if self._state is PatrolState.SHIFT_TO_NEXT_LANE and event is PatrolEvent.SIDE_SHIFT_DONE:
            self._state = PatrolState.TURN_AROUND
            return

        if self._state is PatrolState.TURN_AROUND and event is PatrolEvent.TURN_DONE:
            self._context.lane_index += 1
            self._context.direction *= -1
            if self._context.lane_index >= self._context.max_lane_count:
                self._state = PatrolState.FINISH
            else:
                self._state = PatrolState.MOVE_ALONG_LANE
            return

        if self._state is PatrolState.RETURN_HOME and event is PatrolEvent.RETURN_DONE:
            self._state = PatrolState.IDLE

    def tick(self, now: float) -> Twist2D:
        _ = now
        if self._state is PatrolState.INIT:
            self._state = PatrolState.MOVE_ALONG_LANE
            return ZERO_TWIST
        if self._state is PatrolState.MOVE_ALONG_LANE:
            return Twist2D(self._context.patrol_speed_mps * self._context.direction, 0.0, 0.0)
        if self._state is PatrolState.END_OF_LANE:
            self._state = PatrolState.SHIFT_TO_NEXT_LANE
            return ZERO_TWIST
        if self._state is PatrolState.SHIFT_TO_NEXT_LANE:
            return Twist2D(0.0, self._context.side_shift_speed_mps, 0.0)
        if self._state is PatrolState.TURN_AROUND:
            return Twist2D(0.0, 0.0, self._context.turn_speed_radps)
        if self._state is PatrolState.PAUSE_AND_RETURN_HOME:
            self._state = PatrolState.RETURN_HOME
            return ZERO_TWIST
        return ZERO_TWIST

    def state(self) -> PatrolState:
        return self._state

    def context(self) -> PatrolContext:
        return replace(self._context)
