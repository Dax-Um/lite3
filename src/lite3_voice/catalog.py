"""Immutable allowlist for voice actions; Mongo may select IDs, never parameters."""
from dataclasses import dataclass

@dataclass(frozen=True)
class VoiceAction:
    id: str
    phrases: tuple[str, ...]
    target_state: int | None
    kind: str

ACTIONS = (
    VoiceAction("stand_up", ("stand", "stand up", "get up"), 6, "posture"),
    VoiceAction("sit_down", ("sit", "sit down", "take a seat"), 1, "posture"),
    VoiceAction("move_forward", ("forward", "move forward", "go forward"), 6, "velocity"),
    VoiceAction("move_backward", ("backward", "move backward", "go back"), 6, "velocity"),
    VoiceAction("stop", ("stop", "stop moving", "emergency stop"), None, "simple"),
    VoiceAction("turn_left_full", ("turn left", "spin left", "full left circle"), 6, "turn"),
    VoiceAction("turn_right_full", ("turn right", "spin right", "full right circle"), 6, "turn"),
    VoiceAction("moonwalk", ("moonwalk", "do a moonwalk"), 6, "simple"),
    VoiceAction("hello", ("hello", "say hello", "greet"), None, "simple"),
)
BY_ID = {item.id: item for item in ACTIONS}
