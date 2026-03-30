from __future__ import annotations

from custom_components.fronius_modbus.value_normalization import enabled_state, is_enabled


def test_is_enabled_handles_common_string_inputs() -> None:
    assert is_enabled(" enabled ")
    assert is_enabled("TRUE")
    assert is_enabled("1")
    assert not is_enabled("off")


def test_enabled_state_uses_shared_normalization() -> None:
    assert enabled_state("yes") == "enabled"
    assert enabled_state(0) == "disabled"
