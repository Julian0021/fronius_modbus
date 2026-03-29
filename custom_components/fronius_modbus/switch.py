from __future__ import annotations

from homeassistant.components.switch import SwitchEntity

from .base import FroniusModbusBaseEntity
from .const import INVERTER_API_SWITCH_TYPES, STORAGE_API_SWITCH_TYPES
from .platform_setup import (
    async_platform_context,
    descriptor_is_available,
    dispatch_service_action,
    entity_description_kwargs,
    extend_entities,
)


async def async_setup_entry(hass, config_entry, async_add_entities) -> None:
    hub, coordinator = await async_platform_context(hass, config_entry)

    entities = []

    extend_entities(
        entities,
        STORAGE_API_SWITCH_TYPES,
        lambda description: FroniusModbusSwitch(
            hub=hub,
            **entity_description_kwargs(
                coordinator=coordinator,
                device_info=hub.device_info_storage,
                description=description,
            ),
        ),
        include=hub.storage_configured and hub.web_api_configured,
    )
    extend_entities(
        entities,
        INVERTER_API_SWITCH_TYPES,
        lambda description: FroniusModbusSwitch(
            hub=hub,
            **entity_description_kwargs(
                coordinator=coordinator,
                device_info=hub.device_info_inverter,
                description=description,
            ),
        ),
        include=hub.web_api_configured,
    )

    async_add_entities(entities)


class FroniusModbusSwitch(FroniusModbusBaseEntity, SwitchEntity):
    """Representation of a Fronius Web API switch."""

    _translation_platform = "switch"

    def __init__(
        self,
        coordinator,
        device_info,
        name,
        key,
        icon,
        hub,
        description=None,
        entity_category=None,
        translation_key=None,
    ):
        super().__init__(
            coordinator=coordinator,
            device_info=device_info,
            name=name,
            key=key,
            description=description,
            icon=icon,
            entity_category=entity_category,
            translation_key=translation_key,
        )
        self._hub = hub

    @property
    def is_on(self):
        value = None
        if isinstance(self.coordinator.data, dict):
            value = self.coordinator.data.get(self._key)
        if value is None:
            return None
        return bool(value)

    async def async_turn_on(self, **kwargs) -> None:
        del kwargs
        await dispatch_service_action(
            self._hub,
            self._description.turn_on_service,
            self._description.turn_on_action,
            **self._description.turn_on_kwargs,
        )

    async def async_turn_off(self, **kwargs) -> None:
        del kwargs
        await dispatch_service_action(
            self._hub,
            self._description.turn_off_service,
            self._description.turn_off_action,
            **self._description.turn_off_kwargs,
        )

    @property
    def available(self) -> bool:
        return super().available and descriptor_is_available(
            self._hub,
            self.coordinator,
            self._description,
        )
