from __future__ import annotations

from custom_components.fronius_modbus.button import iter_button_keys
from custom_components.fronius_modbus.number import iter_number_keys
from custom_components.fronius_modbus.registry_maintenance import _expected_entity_unique_ids
from custom_components.fronius_modbus.select import iter_select_keys
from custom_components.fronius_modbus.sensor import iter_sensor_keys
from custom_components.fronius_modbus.switch import iter_switch_keys


class _RegistryHub:
    def __init__(self) -> None:
        self.entity_prefix = "fm"
        self.web_api_configured = True
        self.storage_configured = True
        self.meter_configured = True
        self.mppt_configured = True
        self.supports_three_phase_inverter = False
        self.storage_extended_control_mode = 4
        self.meter_unit_ids = [200]
        self.visible_mppt_module_ids = [1, 3]
        self.mppt_module_count = 2
        self.device_info_inverter = {"identifiers": {("fronius_modbus", "inverter")}}
        self.device_info_storage = {"identifiers": {("fronius_modbus", "storage")}}
        self.data = {
            "MaxChaRte": 4000,
            "MaxDisChaRte": 5000,
            "max_power": 6000,
        }

    def meter_value(self, unit_id: int, key: str):
        assert unit_id == 200
        assert key == "phase_count"
        return 1

    def get_device_info_meter(self, unit_id: int) -> dict[str, object]:
        return {"identifiers": {("fronius_modbus", f"meter_{unit_id}")}}


def test_expected_entity_unique_ids_follow_platform_key_iterators() -> None:
    hub = _RegistryHub()

    platform_keys = {
        *iter_button_keys(hub),
        *iter_number_keys(hub),
        *iter_select_keys(hub),
        *iter_sensor_keys(hub),
        *iter_switch_keys(hub),
    }

    assert _expected_entity_unique_ids(hub) == {f"fm_{key}" for key in platform_keys}
    assert "fm_meter_200_power" in _expected_entity_unique_ids(hub)
    assert "fm_meter_200_AphB" not in _expected_entity_unique_ids(hub)
    assert "fm_mppt_module_0_dc_current" in _expected_entity_unique_ids(hub)
    assert "fm_mppt_module_2_dc_current" not in _expected_entity_unique_ids(hub)
