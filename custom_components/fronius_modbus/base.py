from __future__ import annotations

from collections.abc import Mapping
import logging
import re
from typing import Any
import unicodedata

from homeassistant.core import HomeAssistant, callback
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .entity_names import (
    resolve_cached_entity_name,
    resolve_cached_entity_name_for_language,
)

_LOGGER = logging.getLogger(__name__)
_NON_OBJECT_ID_RE = re.compile(r"[^a-z0-9_]+")
_MULTI_UNDERSCORE_RE = re.compile(r"_+")


def _slugify_object_id(value: str) -> str:
    normalized = (
        unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode("ascii")
    )
    normalized = normalized.lower().replace("-", "_").replace(" ", "_")
    normalized = _NON_OBJECT_ID_RE.sub("_", normalized)
    normalized = _MULTI_UNDERSCORE_RE.sub("_", normalized).strip("_")
    return normalized or "entity"


def _normalize_generated_device_name(device_name: str, device_info) -> str:
    if not isinstance(device_info, Mapping):
        return device_name
    identifiers = device_info.get("identifiers")
    if (
        isinstance(identifiers, set)
        and any(
            isinstance(identifier, tuple)
            and len(identifier) == 2
            and isinstance(identifier[1], str)
            and identifier[1].endswith("_inverter")
            for identifier in identifiers
        )
    ):
        normalized = re.sub(r"\s+\d+(?:\.\d+)?$", "", device_name.strip())
        if normalized:
            return normalized
    return device_name


def _device_registry_name(hass: HomeAssistant, device_info) -> str | None:
    if not isinstance(device_info, Mapping):
        return None
    identifiers = device_info.get("identifiers")
    connections = device_info.get("connections")
    if not isinstance(identifiers, set) and not isinstance(connections, set):
        return None
    registry = dr.async_get(hass)
    if registry is None:
        return None
    device_entry = registry.async_get_device(
        identifiers=identifiers if isinstance(identifiers, set) else None,
        connections=connections if isinstance(connections, set) else None,
    )
    if device_entry is None:
        return None
    for candidate in (device_entry.name_by_user, device_entry.name):
        if isinstance(candidate, str) and candidate:
            return candidate
    return None


def _preferred_device_name(
    device_info,
    hass: HomeAssistant | None = None,
) -> str | None:
    if hass is not None and (registry_name := _device_registry_name(hass, device_info)):
        return registry_name
    if isinstance(device_info, Mapping):
        raw_name = device_info.get("name")
        if isinstance(raw_name, str) and raw_name:
            return _normalize_generated_device_name(raw_name, device_info)
    return None


def _full_object_id(
    device_info,
    entity_name: str,
    hass: HomeAssistant | None = None,
) -> str:
    device_name = _preferred_device_name(device_info, hass)
    return _slugify_object_id(
        f"{device_name} {entity_name}" if device_name else entity_name
    )


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
        english_name = resolve_cached_entity_name_for_language(
            coordinator.hass,
            language="en",
            entity_platform=self._translation_platform,
            translation_key=translation_key,
            placeholders=translation_placeholders,
            fallback=name or key,
            logger=_LOGGER,
        )
        object_id = _full_object_id(device_info, english_name, coordinator.hass)
        self._attr_suggested_object_id = object_id
        if self._translation_platform is not None:
            self.entity_id = f"{self._translation_platform}.{object_id}"
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
