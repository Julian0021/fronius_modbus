"""Platform for sensor integration."""
from __future__ import annotations

import logging

from homeassistant.components.sensor import SensorDeviceClass, SensorEntity
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from . import HubConfigEntry
from .base import FroniusModbusBaseEntity
from .const import (
    INVERTER_SENSOR_TYPES,
    INVERTER_STORAGE_SENSOR_TYPES,
    INVERTER_SYMO_SENSOR_TYPES,
    INVERTER_WEB_SENSOR_TYPES,
    METER_SENSOR_TYPES,
    MPPT_MODULE_SENSOR_TYPES,
    SENSOR_STATE_OPTIONS,
    SINGLE_PHASE_UNSUPPORTED_METER_SENSOR_KEYS,
    STORAGE_SENSOR_TYPES,
)
from .platform_setup import async_platform_context, entity_description_kwargs, extend_entities

_LOGGER = logging.getLogger(__name__)


def _meter_prefix(unit_id: int) -> str:
    return f"meter_{int(unit_id)}_"


def _device_class_for_spec(spec) -> SensorDeviceClass | None:
    options = SENSOR_STATE_OPTIONS.get(spec.translation_key)
    return SensorDeviceClass.ENUM if options is not None else spec.device_class


def _build_sensor(
    *,
    coordinator,
    device_info,
    description,
    key: str | None = None,
    translation_placeholders: dict[str, str] | None = None,
) -> FroniusModbusSensor:
    return FroniusModbusSensor(
        **entity_description_kwargs(
            coordinator=coordinator,
            device_info=device_info,
            description=description,
            key=description.key if key is None else key,
            translation_placeholders=translation_placeholders,
            device_class=_device_class_for_spec(description),
            state_class=description.state_class,
            unit=description.unit,
            options=SENSOR_STATE_OPTIONS.get(description.translation_key),
        )
    )


async def async_setup_entry(
    hass: HomeAssistant,
    config_entry: HubConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    """Add sensors for passed config_entry in HA."""
    hub, coordinator = await async_platform_context(hass, config_entry)

    entities = []
    extend_entities(
        entities,
        INVERTER_SENSOR_TYPES.values(),
        lambda description: _build_sensor(
            coordinator=coordinator,
            device_info=hub.device_info_inverter,
            description=description,
        ),
    )
    extend_entities(
        entities,
        INVERTER_SYMO_SENSOR_TYPES.values(),
        lambda description: _build_sensor(
            coordinator=coordinator,
            device_info=hub.device_info_inverter,
            description=description,
        ),
        include=hub.supports_three_phase_inverter,
    )
    extend_entities(
        entities,
        INVERTER_WEB_SENSOR_TYPES.values(),
        lambda description: _build_sensor(
            coordinator=coordinator,
            device_info=hub.device_info_inverter,
            description=description,
        ),
        include=hub.web_api_configured,
    )

    if hub.mppt_configured:
        for module_id in hub.visible_mppt_module_ids:
            if module_id < 1 or module_id > hub.mppt_module_count:
                continue
            module_idx = module_id - 1
            for description in MPPT_MODULE_SENSOR_TYPES:
                entities.append(
                    _build_sensor(
                        coordinator=coordinator,
                        device_info=hub.device_info_inverter,
                        description=description,
                        key=f"mppt_module_{module_idx}_{description.key_suffix}",
                        translation_placeholders={"module": str(module_id)},
                    )
                )

    if hub.meter_configured:
        for meter_unit_id in hub.meter_unit_ids:
            prefix = _meter_prefix(meter_unit_id)
            phase_count = hub.meter_value(meter_unit_id, "phase_count")
            for description in METER_SENSOR_TYPES.values():
                if (
                    phase_count == 1
                    and description.key in SINGLE_PHASE_UNSUPPORTED_METER_SENSOR_KEYS
                ):
                    continue
                entities.append(
                    _build_sensor(
                        coordinator=coordinator,
                        device_info=hub.get_device_info_meter(meter_unit_id),
                        description=description,
                        key=f"{prefix}{description.key}",
                    )
                )

    if hub.storage_configured:
        extend_entities(
            entities,
            INVERTER_STORAGE_SENSOR_TYPES.values(),
            lambda description: _build_sensor(
                coordinator=coordinator,
                device_info=hub.device_info_inverter,
                description=description,
            ),
        )
        extend_entities(
            entities,
            STORAGE_SENSOR_TYPES.values(),
            lambda description: _build_sensor(
                coordinator=coordinator,
                device_info=hub.device_info_storage,
                description=description,
            ),
        )

    async_add_entities(entities)


class FroniusModbusSensor(FroniusModbusBaseEntity, SensorEntity):
    """Representation of a Fronius Modbus sensor."""

    _translation_platform = "sensor"

    @property
    def native_value(self):
        """Return the native sensor value."""
        if not isinstance(self.coordinator.data, dict):
            return None
        value = self.coordinator.data.get(self._key)
        if isinstance(value, str) and len(value) > 255:
            _LOGGER.error("State length >255 for %s", self._key)
            return value[:255]
        return value

    @property
    def extra_state_attributes(self):
        return None
