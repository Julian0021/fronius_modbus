from __future__ import annotations

import logging
from types import SimpleNamespace
from unittest.mock import Mock

from homeassistant.helpers.update_coordinator import DataUpdateCoordinator
import pytest
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.fronius_modbus.const import DOMAIN
from custom_components.fronius_modbus.platform_setup import entity_description_kwargs
from custom_components.fronius_modbus.sensor import FroniusModbusSensor


def _build_coordinator(hass) -> DataUpdateCoordinator:
    coordinator = DataUpdateCoordinator(
        hass,
        logger=logging.getLogger(__name__),
        name="fronius",
        config_entry=MockConfigEntry(domain=DOMAIN),
    )
    coordinator.hub = SimpleNamespace(entity_prefix="fm", hass=hass)
    return coordinator


def _build_sensor(coordinator: DataUpdateCoordinator) -> FroniusModbusSensor:
    return FroniusModbusSensor(
        **entity_description_kwargs(
            coordinator=coordinator,
            device_info={"identifiers": {("fronius_modbus", "device")}},
            description=SimpleNamespace(
                translation_key="pv_power",
                key="pv_power",
                state_class=None,
                unit="W",
            ),
        )
    )


@pytest.mark.asyncio
async def test_base_entity_registers_coordinator_listener_with_removable_callback(hass) -> None:
    coordinator = _build_coordinator(hass)
    entity = _build_sensor(coordinator)
    entity.hass = hass
    entity.async_write_ha_state = Mock()

    await entity.async_added_to_hass()

    assert entity.async_write_ha_state.call_count == 1

    coordinator.async_set_updated_data({"pv_power": 123})

    assert entity.async_write_ha_state.call_count == 2

    remove_callbacks = list(entity._on_remove)
    assert remove_callbacks
    for remove_callback in remove_callbacks:
        remove_callback()

    coordinator.async_set_updated_data({"pv_power": 456})

    assert entity.async_write_ha_state.call_count == 2
