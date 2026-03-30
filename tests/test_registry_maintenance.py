from __future__ import annotations

from types import SimpleNamespace

from custom_components.fronius_modbus.button import iter_button_keys
from custom_components.fronius_modbus.number import iter_number_keys
from custom_components.fronius_modbus.registry_maintenance import (
    _expected_entity_unique_ids,
    async_migrate_v019_mppt_statistics,
    async_remove_unexpected_entities,
)
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


class _FakeEntityRegistry:
    def __init__(self, *entries) -> None:
        self._entries = {entry.entity_id: entry for entry in entries}
        self._entity_ids_by_unique_id = {
            entry.unique_id: entry.entity_id
            for entry in entries
            if getattr(entry, "unique_id", None)
        }
        self.update_calls: list[dict[str, str]] = []
        self.available_entity_id_calls: list[dict[str, object]] = []

    def async_get_entity_id(self, _platform: str, _domain: str, unique_id: str) -> str | None:
        return self._entity_ids_by_unique_id.get(unique_id)

    def async_get(self, entity_id: str):
        return self._entries.get(entity_id)

    def async_get_available_entity_id(
        self,
        _platform: str,
        suggested_object_id: str,
        *,
        current_entity_id: str,
        reserved_entity_ids: set[str],
    ) -> str:
        self.available_entity_id_calls.append(
            {
                "suggested_object_id": suggested_object_id,
                "current_entity_id": current_entity_id,
                "reserved_entity_ids": set(reserved_entity_ids),
            }
        )
        return "sensor.fronius_mppt_module_1_dc_power"

    def async_update_entity(
        self,
        entity_id: str,
        *,
        new_entity_id: str,
        new_unique_id: str,
    ) -> None:
        entry = self._entries.pop(entity_id)
        self._entity_ids_by_unique_id.pop(entry.unique_id, None)
        entry.entity_id = new_entity_id
        entry.unique_id = new_unique_id
        self._entries[new_entity_id] = entry
        self._entity_ids_by_unique_id[new_unique_id] = new_entity_id
        self.update_calls.append(
            {
                "entity_id": entity_id,
                "new_entity_id": new_entity_id,
                "new_unique_id": new_unique_id,
            }
        )


class _FakeDeviceRegistry:
    def __init__(self, devices: dict[str, object]) -> None:
        self._devices = devices

    def async_get(self, device_id: str):
        return self._devices.get(device_id)


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


async def test_async_remove_unexpected_entities_preserves_topology_sensitive_ids(
    monkeypatch,
) -> None:
    hub = _RegistryHub()
    registry_entries = [
        SimpleNamespace(entity_id="sensor.stale_stable", unique_id="fm_stale_sensor"),
        SimpleNamespace(
            entity_id="sensor.stale_mppt",
            unique_id="fm_mppt_module_9_dc_power",
        ),
        SimpleNamespace(
            entity_id="sensor.stale_storage_transfer",
            unique_id="fm_storage_charge_power",
        ),
    ]
    removed_entity_ids: list[str] = []
    registry = SimpleNamespace(
        async_remove=lambda entity_id: removed_entity_ids.append(entity_id)
    )

    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.er.async_get",
        lambda _hass: registry,
    )
    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.er.async_entries_for_config_entry",
        lambda _registry, _entry_id: registry_entries,
    )

    await async_remove_unexpected_entities(
        hass=object(),
        entry=SimpleNamespace(entry_id="entry-1"),
        runtime_data=hub,
        preserve_topology_sensitive_entities=True,
    )

    assert removed_entity_ids == ["sensor.stale_stable"]


async def test_async_migrate_v019_mppt_statistics_renames_old_mppt_entities(
    monkeypatch,
) -> None:
    hub = _RegistryHub()
    old_entry = SimpleNamespace(
        entity_id="sensor.legacy_mppt_1_power",
        unique_id="fm_mppt1_power",
        device_id="device-1",
    )
    registry = _FakeEntityRegistry(old_entry)
    device_registry = _FakeDeviceRegistry(
        {
            "device-1": SimpleNamespace(
                name="Fronius GEN24",
                name_by_user=None,
            )
        }
    )

    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.er.async_get",
        lambda _hass: registry,
    )
    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.er.async_entries_for_config_entry",
        lambda _registry, _entry_id: [old_entry],
    )
    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.dr.async_get",
        lambda _hass: device_registry,
    )
    async def _async_resolve_entity_name(*_args, **_kwargs) -> str:
        return "MPPT Module 1 DC Power"

    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.async_resolve_entity_name",
        _async_resolve_entity_name,
    )

    await async_migrate_v019_mppt_statistics(
        hass=object(),
        entry=SimpleNamespace(entry_id="entry-1"),
        runtime_data=hub,
    )

    assert registry.available_entity_id_calls == [
        {
            "suggested_object_id": "Fronius GEN24 MPPT Module 1 DC Power",
            "current_entity_id": "sensor.legacy_mppt_1_power",
            "reserved_entity_ids": set(),
        }
    ]
    assert registry.update_calls == [
        {
            "entity_id": "sensor.legacy_mppt_1_power",
            "new_entity_id": "sensor.fronius_mppt_module_1_dc_power",
            "new_unique_id": "fm_mppt_module_0_dc_power",
        }
    ]
    assert old_entry.entity_id == "sensor.fronius_mppt_module_1_dc_power"
    assert old_entry.unique_id == "fm_mppt_module_0_dc_power"


async def test_async_migrate_v019_mppt_statistics_skips_when_target_unique_id_exists(
    monkeypatch,
) -> None:
    hub = _RegistryHub()
    old_entry = SimpleNamespace(
        entity_id="sensor.legacy_mppt_1_power",
        unique_id="fm_mppt1_power",
        device_id="device-1",
    )
    new_entry = SimpleNamespace(
        entity_id="sensor.current_mppt_1_power",
        unique_id="fm_mppt_module_0_dc_power",
        device_id="device-1",
    )
    registry = _FakeEntityRegistry(old_entry, new_entry)

    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.er.async_get",
        lambda _hass: registry,
    )
    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.er.async_entries_for_config_entry",
        lambda _registry, _entry_id: [old_entry, new_entry],
    )
    monkeypatch.setattr(
        "custom_components.fronius_modbus.registry_maintenance.dr.async_get",
        lambda _hass: _FakeDeviceRegistry({}),
    )

    await async_migrate_v019_mppt_statistics(
        hass=object(),
        entry=SimpleNamespace(entry_id="entry-1"),
        runtime_data=hub,
    )

    assert registry.available_entity_id_calls == []
    assert registry.update_calls == []
    assert old_entry.entity_id == "sensor.legacy_mppt_1_power"
    assert old_entry.unique_id == "fm_mppt1_power"
