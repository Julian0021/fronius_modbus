from __future__ import annotations

from types import SimpleNamespace

from homeassistant.components.sensor import SensorEntity
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator
import pytest

from custom_components.fronius_modbus.platform_setup import entity_description_kwargs
from custom_components.fronius_modbus.sensor import FroniusModbusSensor


def _build_coordinator() -> DataUpdateCoordinator:
    hass = HomeAssistant()
    coordinator = DataUpdateCoordinator(hass, name="fronius")
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
async def test_base_entity_registers_and_unregisters_coordinator_listener() -> None:
    coordinator = _build_coordinator()
    entity = _build_sensor(coordinator)

    assert coordinator.listener_count == 0

    await entity.async_added_to_hass()

    assert coordinator.listener_count == 1
    assert entity._async_write_count == 1

    coordinator.async_set_updated_data({"pv_power": 123})

    assert entity._async_write_count == 2
    assert entity._last_written_state == 123

    await entity.async_will_remove_from_hass()

    assert coordinator.listener_count == 0

    coordinator.async_set_updated_data({"pv_power": 456})

    assert entity._async_write_count == 2
    assert entity._last_written_state == 123


def test_sensor_entity_stub_rejects_container_states() -> None:
    class _InvalidSensor(SensorEntity):
        @property
        def native_value(self):
            return {"unexpected": "mapping"}

    entity = _InvalidSensor()

    with pytest.raises(TypeError, match="Unsupported state type"):
        entity.async_write_ha_state()
