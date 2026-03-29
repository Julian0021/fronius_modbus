from __future__ import annotations

import logging
from collections.abc import Mapping
from typing import Any

from homeassistant.core import callback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .entity_names import resolve_cached_entity_name

_LOGGER = logging.getLogger(__name__)


class FroniusModbusBaseEntity(CoordinatorEntity):
    """Base entity for Fronius Modbus devices."""

    _key: str | None = None
    _options_dict: dict[Any, str] | None = None
    _translation_platform: str | None = None

    def _resolve_entity_name(
        self,
        coordinator,
        name,
        translation_key,
        translation_placeholders,
    ):
        """Resolve a localized fallback entity name from bundled translation files."""
        return resolve_cached_entity_name(
            coordinator.hass,
            entity_platform=self._translation_platform,
            translation_key=translation_key,
            placeholders=translation_placeholders,
            fallback=name,
            logger=_LOGGER,
        )

    def __init__(
        self,
        *,
        coordinator,
        device_info,
        name: str | None,
        key: str,
        hub=None,
        description: Any = None,
        device_class: Any = None,
        state_class: Any = None,
        unit: str | None = None,
        icon: str | None = None,
        entity_category: Any = None,
        translation_key: str | None = None,
        translation_placeholders: Mapping[str, str] | None = None,
        options: Mapping[Any, str] | list[str] | tuple[str, ...] | None = None,
        minimum: float | int | None = None,
        maximum: float | int | None = None,
        native_step: float | int | None = None,
        mode: Any = None,
    ) -> None:
        """Initialize the entity."""
        super().__init__(coordinator)
        self._hub = hub
        self._key = key
        self._description = description
        self._unit_of_measurement = unit
        self._icon = icon

        if device_class is not None:
            self._attr_device_class = device_class
        if state_class is not None:
            self._attr_state_class = state_class
        if entity_category is not None:
            self._attr_entity_category = entity_category
        if options is not None:
            if isinstance(options, Mapping):
                self._options_dict = dict(options)
                self._attr_options = list(options.values())
            else:
                self._attr_options = list(options)
        if unit is not None:
            self._attr_native_unit_of_measurement = unit
        if minimum is not None:
            self._attr_native_min_value = minimum
        if maximum is not None:
            self._attr_native_max_value = maximum
        if native_step is not None:
            self._attr_native_step = native_step
        if mode is not None:
            self._attr_mode = mode

        self._attr_has_entity_name = True
        if translation_key is not None:
            self._attr_translation_key = translation_key
            if translation_placeholders is not None:
                self._attr_translation_placeholders = translation_placeholders
        self._attr_name = self._resolve_entity_name(
            coordinator=coordinator,
            name=name,
            translation_key=translation_key,
            translation_placeholders=translation_placeholders,
        )
        self._attr_unique_id = f"{coordinator.hub.entity_prefix}_{self._key}"
        self._attr_device_info = device_info

    @property
    def available(self) -> bool:
        """Return if entity is available."""
        return self.coordinator.last_update_success

    async def async_added_to_hass(self) -> None:
        """When entity is added to hass."""
        await super().async_added_to_hass()
        self._handle_coordinator_update()

    @callback
    def _handle_coordinator_update(self) -> None:
        """Handle updated data from the coordinator."""
        self.async_write_ha_state()

    @property
    def should_poll(self) -> bool:
        """Data is delivered by the coordinator."""
        return False

    @property
    def unit_of_measurement(self):
        """Return the unit of measurement."""
        return self._unit_of_measurement

    @property
    def icon(self):
        """Return the sensor icon."""
        return self._icon
