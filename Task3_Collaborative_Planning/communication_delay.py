"""Communication-delay model for reported fleet positions."""
from __future__ import annotations

from collections import defaultdict
from typing import Iterable


class CommunicationDelayManager:
    """Return each agent's position from ``t - communication_delay_steps``.

    The manager stores true positions at every simulator tick. Task allocation
    remains synchronized; only the position state that other agents observe is
    delayed.
    """

    def __init__(self, communication_delay_steps: int = 0):
        self.communication_delay_steps = max(0, int(communication_delay_steps))
        self._history: dict[str, list[tuple[int, tuple[int, int]]]] = defaultdict(list)

    def record_positions(self, time_step: int, positions: dict[str, tuple[int, int]] | Iterable[dict]) -> None:
        if isinstance(positions, dict):
            items = positions.items()
        else:
            items = ((agent["name"], tuple(agent["start"])) for agent in positions)
        for name, pos in items:
            pos_tuple = (int(pos[0]), int(pos[1]))
            history = self._history[name]
            if history and history[-1][0] == time_step:
                history[-1] = (int(time_step), pos_tuple)
            else:
                history.append((int(time_step), pos_tuple))

    def get_reported_position(self, agent_name: str, current_time: int) -> tuple[int, int] | None:
        history = self._history.get(agent_name, [])
        if not history:
            return None
        target_time = int(current_time) - self.communication_delay_steps
        reported = history[0][1]
        for recorded_time, pos in history:
            if recorded_time <= target_time:
                reported = pos
            else:
                break
        return reported

    def get_true_history(self) -> dict[str, list[tuple[int, tuple[int, int]]]]:
        return {name: list(history) for name, history in self._history.items()}

    def get_reported_positions(self, current_time: int) -> dict[str, tuple[int, int]]:
        return {
            agent_name: pos
            for agent_name in self._history
            if (pos := self.get_reported_position(agent_name, current_time)) is not None
        }
