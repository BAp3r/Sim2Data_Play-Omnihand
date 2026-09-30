"""Small explicit transition machine for one continuous two-arm relay."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum


class RelayPhase(str, Enum):
    RESET_SETTLE="reset_settle"; LEFT_APPROACH="left_approach"; LEFT_APPROACH_LOWER="left_approach_lower"; LEFT_CLOSE="left_close"; LEFT_LIFT="left_lift"
    LEFT_TRANSFER="left_transfer"; LEFT_LOWER="left_lower"; LEFT_RELEASE="left_release"; LEFT_RETREAT="left_retreat"
    RELAY_STABLE="relay_stable"; RIGHT_APPROACH="right_approach"; RIGHT_APPROACH_LOWER="right_approach_lower"; RIGHT_CLOSE="right_close"; RIGHT_LIFT="right_lift"
    RIGHT_TRANSFER="right_transfer"; RIGHT_LOWER="right_lower"; RIGHT_RELEASE="right_release"; RIGHT_RETREAT="right_retreat"
    COMPLETE="complete"; FAILED="failed"


@dataclass
class RelayStateMachine:
    phase: RelayPhase = RelayPhase.RESET_SETTLE
    timeout_s: float = 20.0
    elapsed_s: float = 0.0

    _next = {
        RelayPhase.RESET_SETTLE: RelayPhase.LEFT_APPROACH,
        RelayPhase.LEFT_APPROACH: RelayPhase.LEFT_APPROACH_LOWER, RelayPhase.LEFT_APPROACH_LOWER: RelayPhase.LEFT_CLOSE, RelayPhase.LEFT_CLOSE: RelayPhase.LEFT_LIFT,
        RelayPhase.LEFT_LIFT: RelayPhase.LEFT_TRANSFER, RelayPhase.LEFT_TRANSFER: RelayPhase.LEFT_LOWER,
        RelayPhase.LEFT_LOWER: RelayPhase.LEFT_RELEASE, RelayPhase.LEFT_RELEASE: RelayPhase.LEFT_RETREAT,
        RelayPhase.LEFT_RETREAT: RelayPhase.RELAY_STABLE, RelayPhase.RELAY_STABLE: RelayPhase.RIGHT_APPROACH,
        RelayPhase.RIGHT_APPROACH: RelayPhase.RIGHT_APPROACH_LOWER, RelayPhase.RIGHT_APPROACH_LOWER: RelayPhase.RIGHT_CLOSE, RelayPhase.RIGHT_CLOSE: RelayPhase.RIGHT_LIFT,
        RelayPhase.RIGHT_LIFT: RelayPhase.RIGHT_TRANSFER, RelayPhase.RIGHT_TRANSFER: RelayPhase.RIGHT_LOWER,
        RelayPhase.RIGHT_LOWER: RelayPhase.RIGHT_RELEASE, RelayPhase.RIGHT_RELEASE: RelayPhase.RIGHT_RETREAT,
        RelayPhase.RIGHT_RETREAT: RelayPhase.COMPLETE,
    }

    def tick(self, dt: float, gate_passed: bool, *, fatal=False):
        if self.phase in (RelayPhase.COMPLETE, RelayPhase.FAILED): return self.phase
        if dt <= 0 or fatal: self.phase=RelayPhase.FAILED; return self.phase
        self.elapsed_s += dt
        if self.elapsed_s > self.timeout_s: self.phase=RelayPhase.FAILED; return self.phase
        if gate_passed:
            self.phase=self._next[self.phase]; self.elapsed_s=0.0
        return self.phase
