"""Patrol event definitions."""

from enum import Enum


class PatrolState(Enum):
    IDLE = "idle"
    INIT = "init"
    MOVE_ALONG_LANE = "move_along_lane"
    END_OF_LANE = "end_of_lane"
    SHIFT_TO_NEXT_LANE = "shift_to_next_lane"
    TURN_AROUND = "turn_around"
    PAUSE_AND_RETURN_HOME = "pause_and_return_home"
    RETURN_HOME = "return_home"
    FINISH = "finish"
    ERROR = "error"


class PatrolEvent(Enum):
    PATROL_START = "patrol_start"
    LANE_END = "lane_end"
    SIDE_SHIFT_DONE = "side_shift_done"
    TURN_DONE = "turn_done"
    RETURN_HOME = "return_home"
    RETURN_DONE = "return_done"
    EMERGENCY_STOP = "emergency_stop"
    RESET = "reset"
