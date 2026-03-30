from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from homeassistant.core import HomeAssistant

import custom_components.fronius_modbus.platform_setup as platform_setup_module
from custom_components.fronius_modbus import (
    button as button_platform,
)
from custom_components.fronius_modbus import (
    number as number_platform,
)
from custom_components.fronius_modbus import (
    select as select_platform,
)
from custom_components.fronius_modbus import (
    sensor as sensor_platform,
)
from custom_components.fronius_modbus import (
    switch as switch_platform,
)
from custom_components.fronius_modbus.platform_setup import (
    descriptor_is_available,
    descriptor_number_value,
    descriptor_number_write_value,
    dispatch_service_action,
    entity_description_kwargs,
    extend_entities,
)
from custom_components.fronius_modbus.storage_modes import (
    STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
)


class _AsyncCallService:
    def __init__(self, calls: list[tuple[str, tuple, dict]]) -> None:
        self._calls = calls

    def __getattr__(self, name: str):
        async def _record(*args, **kwargs):
            self._calls.append((name, args, kwargs))

        return _record


class _FakeHub:
    def __init__(
        self,
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
        self.hass = HomeAssistant()
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


def _collect_entities(platform_async_setup, hub: _FakeHub):
    entities = []
    entry = SimpleNamespace(runtime_data=hub)
    asyncio.run(platform_async_setup(hub.hass, entry, entities.extend))
    return entities


def _entity_by_key(entities, key: str):
    return next(entity for entity in entities if entity._key == key)


def test_entity_description_kwargs_collects_common_descriptor_fields() -> None:
    description = SimpleNamespace(
        translation_key="example",
        key="example_key",
        icon="mdi:test",
        entity_category="diagnostic",
        options={0: "Disabled", 1: "Enabled"},
    )

    kwargs = entity_description_kwargs(
        coordinator="coordinator",
        device_info="device",
        description=description,
        minimum=5,
    )

    assert kwargs == {
        "coordinator": "coordinator",
        "device_info": "device",
        "name": "example",
        "translation_key": "example",
        "description": description,
        "key": "example_key",
        "icon": "mdi:test",
        "entity_category": "diagnostic",
        "options": {0: "Disabled", 1: "Enabled"},
        "minimum": 5,
    }


def test_extend_entities_only_adds_when_enabled() -> None:
    entities: list[str] = []

    extend_entities(entities, ["a", "b"], lambda item: item.upper(), include=False)
    extend_entities(entities, ["a", "b"], lambda item: item.upper())

    assert entities == ["A", "B"]


def test_descriptor_helpers_cover_entity_policies() -> None:
    hub = SimpleNamespace(
        web_api_configured=True,
        storage_configured=True,
        storage_extended_control_mode=4,
        max_charge_rate_w=5000,
        max_discharge_rate_w=6000,
    )
    coordinator = SimpleNamespace(
        data={
            "api_battery_mode_effective_raw": 1,
            "power_factor_enable": "Enabled",
            "power_factor": 0.97,
        }
    )

    storage_number = SimpleNamespace(
        key="grid_charge_power",
        availability="storage_mode",
        mode_capability=STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
        display_scale="charge_rate",
    )
    api_number = SimpleNamespace(
        key="api_battery_power",
        availability="web_api_manual_mode",
    )
    toggle = SimpleNamespace(
        key="api_solar_api_enabled",
        availability="web_api",
        turn_on_service="web_api_service",
        turn_on_action="set_solar_api_enabled",
        turn_on_kwargs={"enabled": True},
    )

    assert descriptor_is_available(hub, coordinator, storage_number)
    assert descriptor_is_available(hub, coordinator, api_number)
    assert descriptor_number_value(hub, storage_number, 50) == 2500.0
    assert descriptor_number_write_value(SimpleNamespace(value_transform="round_int"), 12.6) == 13

    calls: list[tuple[str, tuple, dict]] = []

    class _WebApiService:
        async def set_solar_api_enabled(self, enabled: bool) -> None:
            calls.append(("set_solar_api_enabled", (), {"enabled": enabled}))

    class CallHub:
        web_api_service = _WebApiService()

    result = asyncio.run(
        dispatch_service_action(
            CallHub(),
            toggle.turn_on_service,
            toggle.turn_on_action,
            **toggle.turn_on_kwargs,
        )
    )
    assert result is None
    assert calls == [("set_solar_api_enabled", (), {"enabled": True})]


def test_descriptor_helpers_reject_unknown_availability_policies() -> None:
    invalid_description = SimpleNamespace(
        translation_key="invalid",
        key="invalid_key",
        availability="definitely_not_real",
    )

    with pytest.raises(ValueError, match="Unsupported availability policy"):
        entity_description_kwargs(
            coordinator="coordinator",
            device_info="device",
            description=invalid_description,
        )

    with pytest.raises(ValueError, match="Unsupported availability policy"):
        descriptor_is_available(
            hub=SimpleNamespace(
                web_api_configured=True,
                storage_configured=True,
                storage_extended_control_mode=0,
            ),
            coordinator=SimpleNamespace(data={}),
            description=invalid_description,
        )


def test_descriptor_helpers_reject_invalid_descriptor_contract_fields() -> None:
    invalid_action_service = SimpleNamespace(
        translation_key="invalid",
        key="invalid_key",
        availability="always",
        action_service="not_a_service",
        action="set_mode",
    )
    invalid_action = SimpleNamespace(
        translation_key="invalid",
        key="invalid_key",
        availability="always",
        action_service="command_service",
        action="does_not_exist",
    )

    with pytest.raises(ValueError, match="Unsupported action_service"):
        entity_description_kwargs(
            coordinator="coordinator",
            device_info="device",
            description=invalid_action_service,
        )

    with pytest.raises(ValueError, match="Unsupported action"):
        entity_description_kwargs(
            coordinator="coordinator",
            device_info="device",
            description=invalid_action,
        )

    with pytest.raises(ValueError, match="Unsupported service_name"):
        asyncio.run(
            dispatch_service_action(
                hub=SimpleNamespace(),
                service_name="not_a_service",
                action="set_mode",
            )
        )

    with pytest.raises(ValueError, match="Unsupported display_scale"):
        descriptor_number_value(
            SimpleNamespace(max_charge_rate_w=1, max_discharge_rate_w=1),
            SimpleNamespace(display_scale="not_real"),
            5,
        )

    with pytest.raises(ValueError, match="Unsupported value_transform"):
        descriptor_number_write_value(
            SimpleNamespace(value_transform="not_real"),
            5,
        )


@pytest.mark.parametrize(
    ("description", "message"),
    [
        (
            SimpleNamespace(
                translation_key="invalid_service",
                key="invalid_service",
                action_service="definitely_not_real",
                action="set_mode",
            ),
            "Unsupported action_service",
        ),
        (
            SimpleNamespace(
                translation_key="invalid_action",
                key="invalid_action",
                action_service="command_service",
                action="definitely_not_real",
            ),
            "Unsupported action",
        ),
        (
            SimpleNamespace(
                translation_key="invalid_scale",
                key="invalid_scale",
                display_scale="definitely_not_real",
            ),
            "Unsupported display_scale",
        ),
        (
            SimpleNamespace(
                translation_key="invalid_transform",
                key="invalid_transform",
                value_transform="definitely_not_real",
            ),
            "Unsupported value_transform",
        ),
    ],
)
def test_entity_description_kwargs_reject_invalid_descriptor_behavior_strings(
    description: SimpleNamespace,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        entity_description_kwargs(
            coordinator="coordinator",
            device_info="device",
            description=description,
        )


def test_async_platform_context_validates_descriptor_catalog(
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
        asyncio.run(
            platform_setup_module.async_platform_context(
                HomeAssistant(),
                SimpleNamespace(runtime_data=object()),
            )
        )


def test_sensor_platform_setup_covers_meter_mppt_and_storage_gating() -> None:
    hub = _FakeHub(
        data={"api_solar_api_enabled": True},
        web_api_configured=True,
        storage_configured=True,
        meter_configured=True,
        mppt_configured=True,
        meter_phase_counts={200: 1},
        visible_mppt_module_ids=[1, 3],
        mppt_module_count=2,
    )

    entities = _collect_entities(sensor_platform.async_setup_entry, hub)
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


def test_sensor_platform_setup_registers_multiple_meters_with_phase_specific_entities() -> None:
    hub = _FakeHub(
        meter_configured=True,
        meter_unit_ids=[200, 240],
        meter_phase_counts={200: 1, 240: 3},
    )

    entities = _collect_entities(sensor_platform.async_setup_entry, hub)
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


def test_number_platform_setup_builds_entities_with_availability_and_dispatch() -> None:
    hub = _FakeHub(
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

    entities = _collect_entities(number_platform.async_setup_entry, hub)
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

    asyncio.run(grid_charge_power.async_set_native_value(1300))
    asyncio.run(soc_maximum.async_set_native_value(12.6))

    assert ("set_grid_charge_power", (1300,), {}) in hub.command_calls
    assert ("set_api_soc_values", (13,), {}) in hub.command_calls


def test_select_switch_and_button_platforms_dispatch_real_descriptor_actions() -> None:
    hub = _FakeHub(
        data={
            "ext_control_mode": "Auto",
            "api_battery_mode": "Manual",
            "ac_limit_enable": "Disabled",
            "power_factor_enable": "Enabled",
            "Conn": "Enabled",
            "api_solar_api_enabled": False,
            "api_charge_from_grid": False,
        },
        web_api_configured=True,
        storage_configured=True,
        storage_extended_control_mode=4,
    )

    select_entities = _collect_entities(select_platform.async_setup_entry, hub)
    switch_entities = _collect_entities(switch_platform.async_setup_entry, hub)
    button_entities = _collect_entities(button_platform.async_setup_entry, hub)

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

    asyncio.run(ac_limit_enable.async_select_option("Enabled"))
    asyncio.run(grid_charge_switch.async_turn_on())
    asyncio.run(reset_button.async_press())

    assert ("set_ac_limit_enable", (1,), {}) in hub.command_calls
    assert (
        "set_api_charge_sources",
        (),
        {"charge_from_grid": True, "charge_from_ac": True},
    ) in hub.command_calls
    assert ("reset_modbus_control", (), {}) in hub.web_api_calls
