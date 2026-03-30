from __future__ import annotations

from homeassistant.components.number import NumberEntity

from .base import FroniusModbusBaseEntity
from .const import (
    INVERTER_NUMBER_TYPES,
    STORAGE_API_NUMBER_TYPES,
    STORAGE_MODBUS_NUMBER_TYPES,
)
from .hub import Hub
from .platform_setup import (
    async_platform_context,
    descriptor_is_available,
    descriptor_native_number_value,
    descriptor_number_write_value,
    dispatch_service_action,
    entity_description_kwargs,
)


def _resolve_max_value(hub: Hub, description) -> float:
    if description.max_key is None:
        return description.maximum
    dynamic_max = hub.data.get(description.max_key)
    if isinstance(dynamic_max, (int, float)) and dynamic_max > 0:
        return dynamic_max
    return description.maximum


def _iter_number_specs(hub: Hub):
    if hub.storage_configured:
        for description in STORAGE_MODBUS_NUMBER_TYPES:
            yield (
                hub.device_info_storage,
                description,
                {
                    "minimum": description.minimum,
                    "maximum": _resolve_max_value(hub, description),
                    "unit": description.unit,
                    "mode": description.mode,
                    "native_step": description.step,
                },
            )

    if hub.storage_configured and hub.web_api_configured:
        for description in STORAGE_API_NUMBER_TYPES:
            yield (
                hub.device_info_storage,
                description,
                {
                    "minimum": description.minimum,
                    "maximum": description.maximum,
                    "unit": description.unit,
                    "mode": description.mode,
                    "native_step": description.step,
                },
            )

    for description in INVERTER_NUMBER_TYPES:
        yield (
            hub.device_info_inverter,
            description,
            {
                "minimum": description.minimum,
                "maximum": _resolve_max_value(hub, description),
                "unit": description.unit,
                "mode": description.mode,
                "native_step": description.step,
            },
        )


def iter_number_keys(hub: Hub):
    for _device_info, description, _extra_kwargs in _iter_number_specs(hub):
        yield description.key


async def async_setup_entry(hass, config_entry, async_add_entities) -> None:
    hub, coordinator = await async_platform_context(hass, config_entry)

    entities = [
        FroniusModbusNumber(
            hub=hub,
            **entity_description_kwargs(
                coordinator=coordinator,
                device_info=device_info,
                description=description,
                **extra_kwargs,
            ),
        )
        for device_info, description, extra_kwargs in _iter_number_specs(hub)
    ]

    async_add_entities(entities)


class FroniusModbusNumber(FroniusModbusBaseEntity, NumberEntity):
    """Representation of a Battery Storage Modbus number."""

    _translation_platform = "number"

    @property
    def native_value(self):
        """Return the native number value."""
        if not isinstance(self.coordinator.data, dict) or self._key not in self.coordinator.data:
            return None
        return descriptor_native_number_value(
            self._hub,
            self._description,
            self.coordinator.data[self._key],
        )

    async def async_set_native_value(self, value: float) -> None:
        """Change the selected value."""
        await dispatch_service_action(
            self._hub,
            self._description.action_service,
            self._description.action,
            descriptor_number_write_value(self._description, value),
        )

    @property
    def available(self) -> bool:
        """Return depending on mode."""
        return super().available and descriptor_is_available(
            self._hub,
            self.coordinator,
            self._description,
        )
