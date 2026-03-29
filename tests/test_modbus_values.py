from __future__ import annotations

from custom_components.fronius_modbus.modbus_values import (
    bitmask_to_string,
    calculate_scaled_value,
    is_numeric_value,
    strip_control_chars,
)


def test_strip_control_chars_removes_ascii_controls() -> None:
    assert strip_control_chars("Alpha\x00\x1f Beta\n") == "Alpha Beta"
    assert strip_control_chars(None) is None


def test_is_numeric_value_excludes_bool() -> None:
    assert is_numeric_value(12)
    assert is_numeric_value(3.5)
    assert not is_numeric_value(True)
    assert not is_numeric_value("12")


def test_calculate_scaled_value_applies_bounds() -> None:
    assert calculate_scaled_value(123, -1, digits=1) == 12.3
    assert calculate_scaled_value(123, -1, upper_bound=10) is None
    assert calculate_scaled_value("bad", -1) is None


def test_bitmask_to_string_formats_known_and_unknown_bits() -> None:
    labels = ["A", "B"]
    assert bitmask_to_string(0b0011, labels) == "A,B"
    assert bitmask_to_string(0b0100, labels) == "bit 2 undefined"
    assert bitmask_to_string(0, labels, default="None") == "None"
