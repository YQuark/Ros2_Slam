from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Tuple


DEFAULT_PRIORITIES: Mapping[str, int] = {
    "teleop": 100,
    "test": 80,
    "nav": 60,
}


@dataclass(frozen=True)
class Command:
    linear_x: float
    angular_z: float


@dataclass(frozen=True)
class SelectedCommand:
    source: str
    command: Command


@dataclass(frozen=True)
class SourceConfig:
    research_sources: Tuple[str, ...] = ()

    def __init__(self, research_sources: Iterable[str] = ()) -> None:
        object.__setattr__(self, "research_sources", tuple(name.strip() for name in research_sources if name.strip()))

    def topic_map(self) -> Dict[str, str]:
        topics = {
            "teleop": "/cmd_vel/teleop",
            "nav": "/cmd_vel/nav",
            "test": "/cmd_vel/test",
        }
        for name in self.research_sources:
            topics[f"research/{name}"] = f"/cmd_vel/research/{name}"
        return topics

    def priorities(self) -> Dict[str, int]:
        priorities = dict(DEFAULT_PRIORITIES)
        for name in self.research_sources:
            priorities[f"research/{name}"] = 40
        return priorities


class CommandMux:
    def __init__(
        self,
        *,
        source_config: SourceConfig,
        linear_limit: float,
        angular_limit: float,
        timeout_sec: float,
    ) -> None:
        self._priorities = source_config.priorities()
        self._linear_limit = abs(float(linear_limit))
        self._angular_limit = abs(float(angular_limit))
        self._timeout_sec = max(float(timeout_sec), 0.0)
        self._commands: Dict[str, tuple[Command, float]] = {}

    def update(self, source: str, command: Command, now_sec: float) -> None:
        if source not in self._priorities:
            return
        self._commands[source] = (command, float(now_sec))

    def select(self, now_sec: float) -> SelectedCommand:
        active = []
        for source, (command, stamp) in self._commands.items():
            if (float(now_sec) - stamp) <= self._timeout_sec:
                active.append((self._priorities[source], stamp, source, command))
        if not active:
            return SelectedCommand(source="idle", command=Command(0.0, 0.0))

        _, _, source, command = max(active, key=lambda item: (item[0], item[1]))
        return SelectedCommand(source=source, command=self._clamp(command))

    def _clamp(self, command: Command) -> Command:
        return Command(
            linear_x=_clamp(command.linear_x, -self._linear_limit, self._linear_limit),
            angular_z=_clamp(command.angular_z, -self._angular_limit, self._angular_limit),
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))
