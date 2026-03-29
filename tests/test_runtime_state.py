from __future__ import annotations

from custom_components.fronius_modbus.runtime_state import FroniusRuntimeState


def test_runtime_state_sections_drive_the_flat_compatibility_view() -> None:
    state = FroniusRuntimeState()

    state.inverter.set("i_model", "Verto 17.5 Plus")
    state.storage.set("soc_minimum", 7)
    state.web_api.set("api_solar_api_enabled", True)
    state.meters.set_value(200, "phase_count", 3)

    assert state.data["i_model"] == "Verto 17.5 Plus"
    assert state.data["soc_minimum"] == 7
    assert state.data["api_solar_api_enabled"] is True
    assert state.data["meter_200_phase_count"] == 3
    assert state.snapshot()["meter_200_phase_count"] == 3


def test_runtime_state_flat_view_updates_named_sections() -> None:
    state = FroniusRuntimeState()

    state.data["i_sw_version"] = "1.39.5-1"
    state.data["api_solar_api_enabled"] = False
    state.data["meter_200_power"] = -1782.1

    assert state.inverter.get("i_sw_version") == "1.39.5-1"
    assert state.web_api.get("api_solar_api_enabled") is False
    assert state.meters.get_value(200, "power") == -1782.1


def test_runtime_state_clear_removes_meter_and_section_values() -> None:
    state = FroniusRuntimeState()

    state.inverter.set("i_sw_version", "1.39.5-1")
    state.storage.set("soc_minimum", 5)
    state.data["meter_200_power"] = -1782.1
    state.data["meter_201_phase_count"] = 3

    state.clear()

    assert state.snapshot() == {}
    assert state.data == {}
    assert state.inverter.get("i_sw_version") is None
    assert state.storage.get("soc_minimum") is None
    assert state.meters.get_value(200, "power") is None
    assert state.meters.get_value(201, "phase_count") is None
