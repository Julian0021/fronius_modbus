from __future__ import annotations

from types import SimpleNamespace

from homeassistant.components.sensor import SensorDeviceClass
import pytest

from custom_components.fronius_modbus import (
    button as button_platform,
    number as number_platform,
    select as select_platform,
    sensor as sensor_platform,
    switch as switch_platform,
)
import custom_components.fronius_modbus.platform_setup as platform_setup_module
from test_support.platform_setup import _AsyncCallService, _entity_by_key


class _FakeHub:
    def __init__(
        self,
        hass,
        *,
        data: dict[str, object] | None = None,
        web_api_configured: bool = False,
        storage_configured: bool = False,
        meter_configured: bool = False,
        mppt_configured: bool = False,
        supports_three_phase_inverter: bool = False,
        storage_extended_control_mode: int = 0,
        meter_phase_counts: dict[int, int] | None = None,
        meter_unit_ids: list[int] | None = None,
        visible_mppt_module_ids: list[int] | None = None,
        mppt_module_count: int = 0,
        max_charge_rate_w: int = 4000,
        max_discharge_rate_w: int = 5000,
    ) -> None:
        self.entity_prefix = "fm"
        self.web_api_configured = web_api_configured
        self.storage_configured = storage_configured
        self.meter_configured = meter_configured
        self.mppt_configured = mppt_configured
        self.supports_three_phase_inverter = supports_three_phase_inverter
        self.storage_extended_control_mode = storage_extended_control_mode
        self.meter_unit_ids = meter_unit_ids or [200]
        self.visible_mppt_module_ids = visible_mppt_module_ids or []
        self.mppt_module_count = mppt_module_count
        self.max_charge_rate_w = max_charge_rate_w
        self.max_discharge_rate_w = max_discharge_rate_w
        self.device_info_inverter = {"identifiers": {("fronius_modbus", "inverter")}}
        self.device_info_storage = {"identifiers": {("fronius_modbus", "storage")}}
        self._meter_phase_counts = meter_phase_counts or {}
        self.command_calls: list[tuple[str, tuple, dict]] = []
        self.web_api_calls: list[tuple[str, tuple, dict]] = []
        self.command_service = _AsyncCallService(self.command_calls)
        self.web_api_service = _AsyncCallService(self.web_api_calls)
        self.hass = hass
        self.data = data or {}
        self.coordinator = SimpleNamespace(
            hass=self.hass,
            hub=self,
            data=self.data,
            last_update_success=True,
        )

    def meter_value(self, unit_id: int, key: str):
        if key != "phase_count":
            raise AssertionError(f"Unexpected meter lookup: {key}")
        return self._meter_phase_counts.get(unit_id)

    def get_device_info_meter(self, unit_id: int) -> dict[str, object]:
        return {"identifiers": {("fronius_modbus", f"meter_{unit_id}")}}


async def _collect_entities(hass, platform_async_setup, hub: _FakeHub):
    entities = []
    entry = SimpleNamespace(runtime_data=hub)
    await platform_async_setup(hass, entry, entities.extend)
    return entities


@pytest.mark.asyncio
async def test_async_platform_context_validates_descriptor_catalog(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    invalid_description = SimpleNamespace(
        translation_key="invalid_action",
        key="invalid_action",
        action_service="command_service",
        action="definitely_not_real",
    )

    async def _fake_ensure_translation_cache(_hass) -> None:
        return None

    monkeypatch.setattr(
        platform_setup_module,
        "ENTITY_DESCRIPTOR_COLLECTIONS",
        ((invalid_description,),),
    )
    monkeypatch.setattr(platform_setup_module, "_VALID_DESCRIPTOR_CATALOG", False)
    monkeypatch.setattr(
        platform_setup_module,
        "async_ensure_translation_cache",
        _fake_ensure_translation_cache,
    )

    with pytest.raises(ValueError, match="Unsupported action"):
        await platform_setup_module.async_platform_context(
            hass,
            SimpleNamespace(runtime_data=object()),
        )


@pytest.mark.asyncio
async def test_async_platform_context_validates_sensor_descriptor_catalog(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    invalid_sensor = SimpleNamespace(
        translation_key="invalid_sensor_device_class",
        key="invalid_sensor_device_class",
        device_class=SensorDeviceClass.POWER,
        state_class=None,
        options=["disabled", "enabled"],
    )

    async def _fake_ensure_translation_cache(_hass) -> None:
        return None

    monkeypatch.setattr(
        platform_setup_module,
        "ENTITY_DESCRIPTOR_COLLECTIONS",
        ((invalid_sensor,),),
    )
    monkeypatch.setattr(platform_setup_module, "_VALID_DESCRIPTOR_CATALOG", False)
    monkeypatch.setattr(
        platform_setup_module,
        "async_ensure_translation_cache",
        _fake_ensure_translation_cache,
    )

    with pytest.raises(ValueError, match="must not declare a non-enum device_class"):
        await platform_setup_module.async_platform_context(
            hass,
            SimpleNamespace(runtime_data=object()),
        )


@pytest.mark.asyncio
async def test_sensor_platform_setup_covers_meter_mppt_and_storage_gating(hass) -> None:
    hub = _FakeHub(
        hass,
        data={"api_solar_api_enabled": True},
        web_api_configured=True,
        storage_configured=True,
        meter_configured=True,
        mppt_configured=True,
        meter_phase_counts={200: 1},
        visible_mppt_module_ids=[1, 3],
        mppt_module_count=2,
    )

    entities = await _collect_entities(hass, sensor_platform.async_setup_entry, hub)
    keys = {entity._key for entity in entities}

    assert "A" in keys
    assert "AphB" not in keys
    assert "inverter_temperature" in keys
    assert "soc" in keys
    assert "meter_200_power" in keys
    assert "meter_200_AphB" not in keys
    assert "mppt_module_0_dc_current" in keys
    assert "mppt_module_2_dc_current" not in keys

    mppt_sensor = _entity_by_key(entities, "mppt_module_0_dc_current")
    assert mppt_sensor._attr_translation_key == "mppt_module_dc_current"
    assert mppt_sensor._attr_translation_placeholders == {"module": "1"}

    meter_entity = _entity_by_key(entities, "meter_200_power")
    assert meter_entity._attr_unique_id == "fm_meter_200_power"
    assert meter_entity._attr_device_info == hub.get_device_info_meter(200)

    grid_status = _entity_by_key(entities, "grid_status")
    assert grid_status._attr_device_class == SensorDeviceClass.ENUM
    assert grid_status._attr_options


@pytest.mark.asyncio
async def test_sensor_platform_setup_registers_multiple_meters_with_phase_specific_entities(
    hass,
) -> None:
    hub = _FakeHub(
        hass,
        meter_configured=True,
        meter_unit_ids=[200, 240],
        meter_phase_counts={200: 1, 240: 3},
    )

    entities = await _collect_entities(hass, sensor_platform.async_setup_entry, hub)
    keys = {entity._key for entity in entities}

    assert "meter_200_power" in keys
    assert "meter_240_power" in keys
    assert "meter_200_AphB" not in keys
    assert "meter_240_AphB" in keys

    meter_200_entity = _entity_by_key(entities, "meter_200_power")
    assert meter_200_entity._attr_unique_id == "fm_meter_200_power"
    assert meter_200_entity._attr_device_info == hub.get_device_info_meter(200)

    meter_240_entity = _entity_by_key(entities, "meter_240_power")
    assert meter_240_entity._attr_unique_id == "fm_meter_240_power"
    assert meter_240_entity._attr_device_info == hub.get_device_info_meter(240)


@pytest.mark.asyncio
async def test_number_platform_setup_builds_entities_with_availability_and_dispatch(
    hass,
) -> None:
    hub = _FakeHub(
        hass,
        data={
            "grid_charge_power": 50,
            "discharge_limit": 60,
            "api_battery_mode_effective_raw": 1,
            "api_battery_power": 1250,
            "soc_maximum": 90,
            "ac_limit_rate": 800,
            "power_factor": None,
            "MaxChaRte": 4000,
            "MaxDisChaRte": 5000,
            "max_power": 6000,
        },
        web_api_configured=True,
        storage_configured=True,
        storage_extended_control_mode=4,
        max_charge_rate_w=4000,
        max_discharge_rate_w=5000,
    )

    entities = await _collect_entities(hass, number_platform.async_setup_entry, hub)
    keys = {entity._key for entity in entities}

    assert "grid_charge_power" in keys
    assert "api_battery_power" in keys
    assert "ac_limit_rate" in keys

    grid_charge_power = _entity_by_key(entities, "grid_charge_power")
    discharge_limit = _entity_by_key(entities, "discharge_limit")
    api_battery_power = _entity_by_key(entities, "api_battery_power")
    power_factor = _entity_by_key(entities, "power_factor")
    soc_maximum = _entity_by_key(entities, "soc_maximum")

    assert grid_charge_power.available is True
    assert grid_charge_power.native_value == 2000.0
    assert grid_charge_power._attr_native_max_value == 4000
    assert discharge_limit.available is False
    assert api_battery_power.available is True
    assert power_factor.available is False

    await grid_charge_power.async_set_native_value(1300)
    await soc_maximum.async_set_native_value(12.6)

    assert ("set_grid_charge_power", (1300,), {}) in hub.command_calls
    assert ("set_api_soc_values", (13,), {}) in hub.command_calls


@pytest.mark.asyncio
async def test_select_switch_and_button_platforms_dispatch_real_descriptor_actions(
    hass,
) -> None:
    hub = _FakeHub(
        hass,
        data={
            "ext_control_mode": "auto",
            "api_battery_mode": "manual",
            "ac_limit_enable": "disabled",
            "power_factor_enable": "enabled",
            "Conn": "enabled",
            "api_solar_api_enabled": False,
            "api_charge_from_grid": False,
        },
        web_api_configured=True,
        storage_configured=True,
        storage_extended_control_mode=4,
    )

    select_entities = await _collect_entities(hass, select_platform.async_setup_entry, hub)
    switch_entities = await _collect_entities(hass, switch_platform.async_setup_entry, hub)
    button_entities = await _collect_entities(hass, button_platform.async_setup_entry, hub)

    select_keys = {entity._key for entity in select_entities}
    switch_keys = {entity._key for entity in switch_entities}
    button_keys = {entity._key for entity in button_entities}

    assert {"ext_control_mode", "api_battery_mode", "ac_limit_enable", "Conn"} <= select_keys
    assert {"api_charge_from_ac", "api_charge_from_grid"} <= switch_keys
    assert {"reset_modbus_control"} <= button_keys

    ac_limit_enable = _entity_by_key(select_entities, "ac_limit_enable")
    solar_api_switch = _entity_by_key(switch_entities, "api_solar_api_enabled")
    grid_charge_switch = _entity_by_key(switch_entities, "api_charge_from_grid")
    reset_button = _entity_by_key(button_entities, "reset_modbus_control")

    assert ac_limit_enable.available is True
    assert solar_api_switch.available is True
    assert solar_api_switch.is_on is False

    await ac_limit_enable.async_select_option("enabled")
    await grid_charge_switch.async_turn_on()
    await reset_button.async_press()

    assert ("set_ac_limit_enable", (1,), {}) in hub.command_calls
    assert (
        "set_api_charge_sources",
        (),
        {"charge_from_grid": True, "charge_from_ac": True},
    ) in hub.command_calls
    assert ("reset_modbus_control", (), {}) in hub.web_api_calls
