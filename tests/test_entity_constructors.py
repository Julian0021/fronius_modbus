from __future__ import annotations

from types import SimpleNamespace

from homeassistant.core import HomeAssistant

from custom_components.fronius_modbus.button import FroniusModbusButton
from custom_components.fronius_modbus.number import FroniusModbusNumber
from custom_components.fronius_modbus.platform_setup import entity_description_kwargs
from custom_components.fronius_modbus.select import FroniusModbusSelect
from custom_components.fronius_modbus.switch import FroniusModbusSwitch


def _build_coordinator():
    hass = HomeAssistant()
    hub = SimpleNamespace(entity_prefix="fm", hass=hass)
    coordinator = SimpleNamespace(
        hass=hass,
        hub=hub,
        data={},
        last_update_success=True,
    )
    return coordinator, hub


def test_descriptor_entities_share_base_constructor_kwargs() -> None:
    coordinator, hub = _build_coordinator()
    device_info = {"identifiers": {("fronius_modbus", "device")}}

    number = FroniusModbusNumber(
        hub=hub,
        **entity_description_kwargs(
            coordinator=coordinator,
            device_info=device_info,
            description=SimpleNamespace(
                translation_key="charge_limit",
                key="charge_limit",
            ),
            minimum=10,
            maximum=90,
            unit="%",
            mode="slider",
            native_step=5,
        ),
    )
    select = FroniusModbusSelect(
        hub=hub,
        **entity_description_kwargs(
            coordinator=coordinator,
            device_info=device_info,
            description=SimpleNamespace(
                translation_key="battery_mode",
                key="battery_mode",
                options={0: "Auto", 1: "Manual"},
            ),
        ),
    )
    switch = FroniusModbusSwitch(
        hub=hub,
        **entity_description_kwargs(
            coordinator=coordinator,
            device_info=device_info,
            description=SimpleNamespace(
                translation_key="solar_api",
                key="solar_api",
                icon="mdi:api",
                entity_category="diagnostic",
            ),
        ),
    )
    button = FroniusModbusButton(
        hub=hub,
        **entity_description_kwargs(
            coordinator=coordinator,
            device_info=device_info,
            description=SimpleNamespace(
                translation_key="reset_modbus_control",
                key="reset_modbus_control",
                icon="mdi:restart",
                entity_category="configuration",
            ),
        ),
    )

    assert number._hub is hub
    assert number._attr_unique_id == "fm_charge_limit"
    assert number._attr_native_min_value == 10
    assert number._attr_native_max_value == 90
    assert number._attr_native_unit_of_measurement == "%"
    assert number._attr_native_step == 5
    assert number._attr_mode == "slider"

    assert select._hub is hub
    assert select._options_dict == {0: "Auto", 1: "Manual"}
    assert select._attr_options == ["Auto", "Manual"]
    assert select._attr_translation_key == "battery_mode"

    assert switch._hub is hub
    assert switch.icon == "mdi:api"
    assert switch._attr_entity_category == "diagnostic"
    assert switch._attr_translation_key == "solar_api"

    assert button._hub is hub
    assert button.icon == "mdi:restart"
    assert button._attr_entity_category == "configuration"
    assert button._attr_translation_key == "reset_modbus_control"
