"""Upper-v3 wheel-slot layout rules.

The wire order is always LF/LR/RF/RR.  A disabled or invalid wheel is absent;
it is never a valid zero-speed sample.
"""

from dataclasses import dataclass
from typing import Iterable, Tuple


LEFT_MASK = 0b0011
RIGHT_MASK = 0b1100
ALL_WHEELS_MASK = 0b1111
DEFAULT_ENABLED_MASK = 0b0110  # M2=left, M3=right


class WheelLayoutError(ValueError):
    """The observation does not contain an eligible wheel on both sides."""


@dataclass(frozen=True)
class WheelLayout:
    enabled_mask: int
    valid_mask: int
    anomaly_mask: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "enabled_mask", int(self.enabled_mask) & ALL_WHEELS_MASK)
        object.__setattr__(self, "valid_mask", int(self.valid_mask) & ALL_WHEELS_MASK)
        object.__setattr__(self, "anomaly_mask", int(self.anomaly_mask) & ALL_WHEELS_MASK)

    @property
    def eligible_mask(self) -> int:
        return self.enabled_mask & self.valid_mask & ~self.anomaly_mask & ALL_WHEELS_MASK

    @property
    def left_indices(self) -> Tuple[int, ...]:
        return self._indices(LEFT_MASK)

    @property
    def right_indices(self) -> Tuple[int, ...]:
        return self._indices(RIGHT_MASK)

    @property
    def complete(self) -> bool:
        return bool(self.left_indices and self.right_indices)

    @property
    def enabled_invalid_count(self) -> int:
        return (self.enabled_mask & ~self.eligible_mask & ALL_WHEELS_MASK).bit_count()

    def aggregate(self, values: Iterable[float]) -> Tuple[float, float]:
        items = tuple(float(value) for value in values)
        if len(items) != 4:
            raise ValueError("exactly four LF/LR/RF/RR values are required")
        if not self.complete:
            raise WheelLayoutError("at least one eligible wheel per side is required")
        return (
            sum(items[index] for index in self.left_indices) / len(self.left_indices),
            sum(items[index] for index in self.right_indices) / len(self.right_indices),
        )

    def pair_disagreement(self, values: Iterable[float]) -> float:
        items = tuple(float(value) for value in values)
        disagreements = []
        for indices in (self.left_indices, self.right_indices):
            if len(indices) >= 2:
                disagreements.append(
                    max(items[i] for i in indices) - min(items[i] for i in indices)
                )
        return max(disagreements, default=0.0)

    def _indices(self, side_mask: int) -> Tuple[int, ...]:
        mask = self.eligible_mask & side_mask
        return tuple(index for index in range(4) if mask & (1 << index))
