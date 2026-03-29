from __future__ import annotations

from homeassistant.components.button import ButtonEntity

from .base import FroniusModbusBaseEntity
from .const import INVERTER_API_BUTTON_TYPES
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
        INVERTER_API_BUTTON_TYPES,
        lambda description: FroniusModbusButton(
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


class FroniusModbusButton(FroniusModbusBaseEntity, ButtonEntity):
    """Representation of a Fronius Web API button."""

    _translation_platform = "button"

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

    async def async_press(self) -> None:
        await dispatch_service_action(
            self._hub,
            self._description.action_service,
            self._description.action,
        )

    @property
    def available(self) -> bool:
        return super().available and descriptor_is_available(
            self._hub,
            self.coordinator,
            self._description,
        )
