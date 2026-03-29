"""Pure Modbus value conversion and formatting helpers."""
from __future__ import annotations

import logging
from typing import Any

_LOGGER = logging.getLogger(__name__)


def strip_control_chars(value: str | None) -> str | None:
    """Strip ASCII control characters from decoded Modbus strings."""
    if value is None:
        return None
    control_chars = "".join(chr(i) for i in range(0, 32))
    return value.translate(str.maketrans("", "", control_chars)).strip()


def is_numeric_value(value: Any) -> bool:
    """Return whether the value should be treated as numeric runtime data."""
    return isinstance(value, (int, float, complex)) and not isinstance(value, bool)


def calculate_scaled_value(
    value: Any,
    scale_factor: Any,
    digits: int = 2,
    lower_bound: float | None = None,
    upper_bound: float | None = None,
):
    """Apply a scale factor and optional plausibility bounds."""
    if is_numeric_value(value) and is_numeric_value(scale_factor):
        scaled_value = round(value * 10**scale_factor, digits)
        if lower_bound is not None and scaled_value < lower_bound:
            _LOGGER.debug(
                "Calculated value %s below lower bound %s value=%s sf=%s digits=%s",
                scaled_value,
                lower_bound,
                value,
                scale_factor,
                digits,
                stack_info=True,
            )
            return None
        if upper_bound is not None and scaled_value > upper_bound:
            _LOGGER.debug(
                "Calculated value %s above upper bound %s value=%s sf=%s digits=%s",
                scaled_value,
                upper_bound,
                value,
                scale_factor,
                digits,
                stack_info=True,
            )
            return None
        return scaled_value

    _LOGGER.debug(
        "Cannot calculate non numeric value=%s sf=%s digits=%s",
        value,
        scale_factor,
        digits,
        stack_info=True,
    )
    return None


def bitmask_to_strings(bitmask: int, bitmask_list: dict | list | tuple, bits: int = 16) -> list[str]:
    """Return the labels for all enabled bits in a bitmask."""
    labels: list[str] = []
    available_labels = len(bitmask_list)
    for bit in range(bits):
        if bitmask & (1 << bit):
            label = bitmask_list[bit] if bit < available_labels else f"bit {bit} undefined"
            labels.append(label)
    return labels


def strings_to_string(strings: list[str], default: str = "NA", max_length: int = 255) -> str:
    """Join bitmask labels into the public string representation."""
    if strings:
        return ",".join(strings)[:max_length]
    return default


def bitmask_to_string(
    bitmask: int,
    bitmask_list: dict | list | tuple,
    default: str = "NA",
    max_length: int = 255,
    bits: int = 16,
) -> str:
    """Format a bitmask into the existing comma-separated label string."""
    return strings_to_string(
        bitmask_to_strings(bitmask=bitmask, bitmask_list=bitmask_list, bits=bits),
        default=default,
        max_length=max_length,
    )
