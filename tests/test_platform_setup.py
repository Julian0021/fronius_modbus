from __future__ import annotations

import asyncio
from types import SimpleNamespace

from homeassistant.components.sensor import SensorDeviceClass
import pytest

from custom_components.fronius_modbus.platform_setup import (
    descriptor_is_available,
    descriptor_native_number_value,
    descriptor_number_write_value,
    dispatch_service_action,
    entity_description_kwargs,
    extend_entities,
)
from custom_components.fronius_modbus.storage_modes import (
    STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
)


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
            "power_factor_enable": "enabled",
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
    assert descriptor_native_number_value(hub, storage_number, 50) == 2500.0
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


def test_dispatch_service_action_rejects_invalid_service_name() -> None:
    with pytest.raises(ValueError, match="Unsupported service_name"):
        asyncio.run(
            dispatch_service_action(
                hub=SimpleNamespace(),
                service_name="not_a_service",
                action="set_mode",
            )
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


@pytest.mark.parametrize(
    ("description", "message"),
    [
        (
            SimpleNamespace(
                translation_key="invalid_sensor_options",
                key="invalid_sensor_options",
                device_class=None,
                state_class=None,
                options={0: "disabled", 1: "enabled"},
            ),
            "must declare options as a list or tuple",
        ),
        (
            SimpleNamespace(
                translation_key="invalid_sensor_device_class",
                key="invalid_sensor_device_class",
                device_class=SensorDeviceClass.POWER,
                state_class=None,
                options=["disabled", "enabled"],
            ),
            "must not declare a non-enum device_class",
        ),
    ],
)
def test_entity_description_kwargs_reject_invalid_sensor_metadata(
    description: SimpleNamespace,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        entity_description_kwargs(
            coordinator="coordinator",
            device_info="device",
            description=description,
        )
