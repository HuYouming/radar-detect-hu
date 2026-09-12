"""Length-unit policy for the radar localization pipeline."""

from __future__ import annotations

from typing import Iterable, Optional

import numpy as np


LENGTH_UNIT = "m"


def require_meter_unit(unit: Optional[str], context: str) -> str:
    """Require an explicit meter-based coordinate convention."""
    normalized = LENGTH_UNIT if unit is None else str(unit).strip().lower()
    if normalized not in {"m", "meter", "meters", "米"}:
        raise ValueError(
            f"{context} must use meters ('m'); received unit={unit!r}. "
            "Convert the source data before entering the localization pipeline."
        )
    return LENGTH_UNIT


def validate_meter_vector(values: Iterable[float], context: str) -> np.ndarray:
    """Return a finite float vector, whose unit is defined as meters."""
    vector = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{context} contains non-finite coordinates")
    return vector


def validate_meter_scale(scale: float, context: str) -> float:
    """Reject accidental millimeter-scale vectors at meter-only boundaries."""
    scale = float(scale)
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError(f"{context} must be a positive finite scale")
    return scale
