from __future__ import annotations

from custom_components.fronius_modbus.control_values import signed_percent_to_register


def test_signed_percent_to_register_rounds_symmetrically() -> None:
    assert signed_percent_to_register(12.345) == 1234
    assert signed_percent_to_register(-12.345) == 65536 - 1234


def test_signed_percent_to_register_handles_negative_half_steps_like_positive_values() -> None:
    positive = signed_percent_to_register(12.355)
    negative = signed_percent_to_register(-12.355)

    assert positive == 1236
    assert negative == 65536 - positive
