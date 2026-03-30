"""Common value normalization helpers shared across runtime layers."""

from __future__ import annotations

from typing import Any

_ENABLED_STRINGS = {"1", "true", "on", "yes", "enabled"}


def is_enabled(value: Any) -> bool:
    """Return whether a mixed-type payload value should be treated as enabled."""
    if isinstance(value, str):
        return value.strip().lower() in _ENABLED_STRINGS
    return bool(value)


def enabled_state(value: Any) -> str:
    """Return the Home Assistant-friendly state string for an enabled flag."""
    return "enabled" if is_enabled(value) else "disabled"
