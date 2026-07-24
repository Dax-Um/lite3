"""Deterministic state-transition plans for resolved voice actions."""
from dataclasses import dataclass
from .catalog import BY_ID

SITTING, STANDING = 1, 6

@dataclass(frozen=True)
class PlannedStep:
    kind: str
    action_id: str

def plan(action_id: str, basic_state: int) -> tuple[PlannedStep, ...]:
    action = BY_ID[action_id]
    if action.id == "stand_up":
        return () if basic_state == STANDING else (PlannedStep("posture", "stand_up"),)
    if action.id == "sit_down":
        return () if basic_state == SITTING else (PlannedStep("posture", "sit_down"),)
    steps = []
    if action.target_state == STANDING and basic_state == SITTING:
        steps.append(PlannedStep("posture", "stand_up"))
    if action.target_state == STANDING and basic_state != SITTING and basic_state != STANDING:
        raise ValueError(f"{action_id} requires stable standing or sitting state, got {basic_state}")
    steps.append(PlannedStep(action.kind, action_id))
    return tuple(steps)
