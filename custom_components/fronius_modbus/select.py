from __future__ import annotations

from homeassistant.components.select import SelectEntity

from .base import FroniusModbusBaseEntity
from .const import INVERTER_SELECT_TYPES, STORAGE_API_SELECT_TYPES, STORAGE_MODBUS_SELECT_TYPES
from .platform_setup import (
    async_platform_context,
    descriptor_is_available,
    dispatch_service_action,
    entity_description_kwargs,
)


def _option_key(options: dict[int, str], selection: str) -> int | None:
    for key, value in options.items():
        if value == selection:
            return key
    return None


def _iter_select_descriptions(hub):
    if hub.storage_configured:
        for description in STORAGE_MODBUS_SELECT_TYPES:
            yield hub.device_info_storage, description

    if hub.storage_configured and hub.web_api_configured:
        for description in STORAGE_API_SELECT_TYPES:
            yield hub.device_info_storage, description

    for description in INVERTER_SELECT_TYPES:
        yield hub.device_info_inverter, description


def iter_select_keys(hub):
    for _device_info, description in _iter_select_descriptions(hub):
        yield description.key


async def async_setup_entry(hass, config_entry, async_add_entities) -> None:
    hub, coordinator = await async_platform_context(hass, config_entry)

    entities = [
        FroniusModbusSelect(
            hub=hub,
            **entity_description_kwargs(
                coordinator=coordinator,
                device_info=device_info,
                description=description,
            ),
        )
        for device_info, description in _iter_select_descriptions(hub)
    ]

    async_add_entities(entities)


class FroniusModbusSelect(FroniusModbusBaseEntity, SelectEntity):
    """Representation of a Battery Storage select."""

    _translation_platform = "select"

    def __init__(
        self,
        coordinator,
        device_info,
        name,
        key,
        options,
        hub,
        description=None,
        translation_key=None,
    ):
        super().__init__(
            coordinator=coordinator,
            device_info=device_info,
            name=name,
            key=key,
            description=description,
            translation_key=translation_key,
            options=options,
        )
        self._hub = hub

    @property
    def current_option(self) -> str | None:
        if isinstance(self.coordinator.data, dict):
            return self.coordinator.data.get(self._key)
        return None

    async def async_select_option(self, option: str) -> None:
        """Change the selected option."""
        new_mode = _option_key(self._options_dict, option)
        if new_mode is None:
            raise ValueError(f"Unsupported option for {self._key}: {option}")

        await dispatch_service_action(
            self._hub,
            self._description.action_service,
            self._description.action,
            new_mode,
        )

    @property
    def available(self) -> bool:
        return super().available and descriptor_is_available(
            self._hub,
            self.coordinator,
            self._description,
        )
