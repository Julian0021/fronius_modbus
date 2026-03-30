"""Shared control-value conversions used by read and write services."""
from __future__ import annotations

from typing import Any


def coerce_scale_factor(
    value: Any,
    *,
    minimum: int = -6,
    maximum: int = 6,
) -> int | None:
    """Return a bounded integer scale factor or ``None`` when unavailable."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None

    scale_factor = round(value)
    if minimum <= scale_factor <= maximum:
        return scale_factor
    return None


def power_factor_raw_to_value(raw_value: Any, power_factor_sf: int | None) -> float | None:
    """Convert a fixed power factor register into a normalized float."""
    if power_factor_sf is None or isinstance(raw_value, bool) or not isinstance(
        raw_value, (int, float)
    ):
        return None

    value = float(raw_value) * (10**power_factor_sf)
    if value < -1 or value > 1:
        return None
    return round(value, max(0, -power_factor_sf))


def power_factor_value_to_raw(value: Any, power_factor_sf: int | None) -> int | None:
    """Convert a normalized power factor value into the register payload."""
    if power_factor_sf is None or isinstance(value, bool) or not isinstance(
        value, (int, float)
    ):
        return None

    numeric_value = float(value)
    if numeric_value < -1 or numeric_value > 1:
        return None

    raw_value = round(numeric_value / (10**power_factor_sf))
    if raw_value < -32768 or raw_value > 32767:
        return None
    return raw_value


def ac_limit_raw_to_percent(raw_value: Any, rate_sf: int | None) -> float | None:
    """Convert the SunSpec AC limit raw register into a percentage."""
    if rate_sf is None or isinstance(raw_value, bool) or not isinstance(
        raw_value, (int, float)
    ):
        return None

    percent = float(raw_value) * (10**rate_sf)
    if percent < 0 or percent > 100:
        return None
    return percent


def ac_limit_raw_to_watts(
    raw_value: Any,
    rate_sf: int | None,
    max_power_w: float | None,
) -> int | None:
    """Convert the AC limit register into watts using the current inverter max power."""
    percent = ac_limit_raw_to_percent(raw_value, rate_sf)
    if percent is None or max_power_w is None:
        return None
    return round(max_power_w * percent / 100.0)


def ac_limit_watts_to_raw(
    watts: Any,
    rate_sf: int | None,
    max_power_w: float | None,
) -> int | None:
    """Convert an AC limit watt target into the SunSpec raw register."""
    if (
        rate_sf is None
        or max_power_w is None
        or max_power_w <= 0
        or isinstance(watts, bool)
        or not isinstance(watts, (int, float))
    ):
        return None

    clamped_watts = min(max(float(watts), 0.0), max_power_w)
    percent = (clamped_watts / max_power_w) * 100.0
    raw_unclamped = percent / (10**rate_sf)
    raw_max = round(100.0 / (10**rate_sf))
    raw_value = int(round(raw_unclamped, abs(rate_sf)))
    return max(0, min(raw_max, raw_value))


def signed_percent_to_register(rate: float) -> int:
    """Encode a signed percent value into the inverter register format."""
    raw_value = round(rate * 100)
    return 65536 + raw_value if raw_value < 0 else raw_value
