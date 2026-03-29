from __future__ import annotations

import logging
import re
from typing import TYPE_CHECKING

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er

from .const import (
    DOMAIN,
    INVERTER_API_BUTTON_TYPES,
    INVERTER_API_SWITCH_TYPES,
    INVERTER_NUMBER_TYPES,
    INVERTER_SELECT_TYPES,
    INVERTER_SENSOR_TYPES,
    INVERTER_STORAGE_SENSOR_TYPES,
    INVERTER_SYMO_SENSOR_TYPES,
    INVERTER_WEB_SENSOR_TYPES,
    METER_SENSOR_TYPES,
    MPPT_MODULE_SENSOR_TYPES,
    SINGLE_PHASE_UNSUPPORTED_METER_SENSOR_KEYS,
    STORAGE_API_NUMBER_TYPES,
    STORAGE_API_SELECT_TYPES,
    STORAGE_API_SWITCH_TYPES,
    STORAGE_MODBUS_NUMBER_TYPES,
    STORAGE_MODBUS_SELECT_TYPES,
    STORAGE_SENSOR_TYPES,
)
from .entity_names import async_resolve_entity_name

if TYPE_CHECKING:
    from .hub import Hub

_LOGGER = logging.getLogger(__name__)

_LEGACY_METER_DEVICE_RE = re.compile(r".*_meter\d+")
_V019_MPPT_UNIQUE_ID_MAPPINGS = (
    ("fm_mppt1_power", "mppt_module_0_dc_power", "mppt_module_dc_power", {"module": "1"}),
    ("fm_mppt2_power", "mppt_module_1_dc_power", "mppt_module_dc_power", {"module": "2"}),
    ("fm_mppt1_lfte", "mppt_module_0_lifetime_energy", "mppt_module_lifetime_energy", {"module": "1"}),
    ("fm_mppt2_lfte", "mppt_module_1_lifetime_energy", "mppt_module_lifetime_energy", {"module": "2"}),
    ("fm_mppt3_power", "storage_charge_power", "storage_charge_power", None),
    ("fm_mppt4_power", "storage_discharge_power", "storage_discharge_power", None),
    ("fm_mppt3_lfte", "storage_charge_lfte", "storage_charge_lfte", None),
    ("fm_mppt4_lfte", "storage_discharge_lfte", "storage_discharge_lfte", None),
)

_INVERTER_ENTITY_DEFINITIONS = (
    INVERTER_SELECT_TYPES,
    INVERTER_NUMBER_TYPES,
    INVERTER_SENSOR_TYPES,
)
_WEB_INVERTER_ENTITY_DEFINITIONS = (
    INVERTER_WEB_SENSOR_TYPES,
    INVERTER_API_SWITCH_TYPES,
    INVERTER_API_BUTTON_TYPES,
)
_STORAGE_ENTITY_DEFINITIONS = (
    STORAGE_MODBUS_SELECT_TYPES,
    STORAGE_MODBUS_NUMBER_TYPES,
    INVERTER_STORAGE_SENSOR_TYPES,
    STORAGE_SENSOR_TYPES,
)
_WEB_STORAGE_ENTITY_DEFINITIONS = (
    STORAGE_API_SELECT_TYPES,
    STORAGE_API_NUMBER_TYPES,
    STORAGE_API_SWITCH_TYPES,
)


def _entity_entries_for_config_entry(registry, entry: ConfigEntry):
    """Return all registry entities attached to a config entry."""
    return list(er.async_entries_for_config_entry(registry, entry.entry_id))


def _legacy_meter_device_needs_removal(device) -> bool:
    """Return whether a device only exists because of the pre-web-api layout."""
    identifiers = getattr(device, "identifiers", set())
    return any(
        identifier_domain == DOMAIN and _LEGACY_METER_DEVICE_RE.fullmatch(identifier)
        for identifier_domain, identifier in identifiers
    )


def _entity_unique_id(runtime_data: Hub, key: str) -> str:
    """Return the entity unique id derived from the current hub prefix."""
    return f"{runtime_data.entity_prefix}_{key}"


def _definition_keys(definitions) -> list[str]:
    """Extract entity keys from a definition collection."""
    items = definitions.values() if isinstance(definitions, dict) else definitions
    return [item.key for item in items]


def _add_expected_keys(expected: set[str], runtime_data: Hub, keys) -> None:
    """Add all expected unique ids for the provided keys to the target set."""
    for key in keys:
        expected.add(_entity_unique_id(runtime_data, key))


def _visible_mppt_module_ids(runtime_data: Hub) -> list[int]:
    """Return the MPPT module ids that should be exposed for the current runtime data."""
    return runtime_data.visible_mppt_module_ids


def _expected_mppt_unique_ids(runtime_data: Hub) -> set[str]:
    """Return the MPPT entity ids that should exist for the current inverter layout."""
    expected: set[str] = set()
    if not runtime_data.mppt_configured:
        return expected

    module_count = int(runtime_data.mppt_module_count)
    for module_id in _visible_mppt_module_ids(runtime_data):
        if module_id < 1 or module_id > module_count:
            continue
        module_idx = module_id - 1
        for description in MPPT_MODULE_SENSOR_TYPES:
            key = f"mppt_module_{module_idx}_{description.key_suffix}"
            expected.add(_entity_unique_id(runtime_data, key))
    return expected


def _expected_meter_unique_ids(runtime_data: Hub) -> set[str]:
    """Return the meter entity ids that should exist for the current meter set."""
    expected: set[str] = set()
    if not runtime_data.meter_configured:
        return expected

    for meter_unit_id in runtime_data.meter_unit_ids:
        prefix = f"meter_{int(meter_unit_id)}_"
        phase_count = runtime_data.meter_value(meter_unit_id, "phase_count")
        for description in METER_SENSOR_TYPES.values():
            if phase_count == 1 and description.key in SINGLE_PHASE_UNSUPPORTED_METER_SENSOR_KEYS:
                continue
            expected.add(_entity_unique_id(runtime_data, f"{prefix}{description.key}"))
    return expected


def _expected_entity_unique_ids(runtime_data: Hub) -> set[str]:
    """Return the full set of entity unique ids the current runtime model supports."""
    expected: set[str] = set()

    for definitions in _INVERTER_ENTITY_DEFINITIONS:
        _add_expected_keys(expected, runtime_data, _definition_keys(definitions))

    if runtime_data.supports_three_phase_inverter:
        _add_expected_keys(expected, runtime_data, _definition_keys(INVERTER_SYMO_SENSOR_TYPES))

    if runtime_data.web_api_configured:
        for definitions in _WEB_INVERTER_ENTITY_DEFINITIONS:
            _add_expected_keys(expected, runtime_data, _definition_keys(definitions))

    if runtime_data.storage_configured:
        for definitions in _STORAGE_ENTITY_DEFINITIONS:
            _add_expected_keys(expected, runtime_data, _definition_keys(definitions))

        if runtime_data.web_api_configured:
            for definitions in _WEB_STORAGE_ENTITY_DEFINITIONS:
                _add_expected_keys(expected, runtime_data, _definition_keys(definitions))

    expected.update(_expected_mppt_unique_ids(runtime_data))
    expected.update(_expected_meter_unique_ids(runtime_data))

    return expected


async def async_migrate_v019_mppt_statistics(
    hass: HomeAssistant,
    entry: ConfigEntry,
    runtime_data: Hub,
) -> None:
    """Rename old v0.1.9 MPPT entities so recorder keeps history/statistics."""
    registry = er.async_get(hass)
    device_registry = dr.async_get(hass)
    entity_entries = _entity_entries_for_config_entry(registry, entry)
    old_unique_ids = {candidate.unique_id or "" for candidate in entity_entries}
    expected_unique_ids = _expected_entity_unique_ids(runtime_data)
    reserved_entity_ids = {candidate.entity_id for candidate in entity_entries}

    for old_unique_id, new_key, translation_key, placeholders in _V019_MPPT_UNIQUE_ID_MAPPINGS:
        if old_unique_id not in old_unique_ids:
            continue

        new_unique_id = _entity_unique_id(runtime_data, new_key)
        if new_unique_id not in expected_unique_ids:
            continue

        old_entity_id = registry.async_get_entity_id("sensor", DOMAIN, old_unique_id)
        if old_entity_id is None:
            continue

        if registry.async_get_entity_id("sensor", DOMAIN, new_unique_id) is not None:
            _LOGGER.debug(
                "Skipping v0.1.9 MPPT migration for %s because target unique id %s already exists",
                old_entity_id,
                new_unique_id,
            )
            continue

        entity_entry = registry.async_get(old_entity_id)
        if entity_entry is None:
            continue

        sensor_name = await async_resolve_entity_name(
            hass,
            entity_platform="sensor",
            translation_key=translation_key,
            placeholders=placeholders,
            fallback=translation_key,
            logger=_LOGGER,
        )
        suggested_object_id = sensor_name
        if entity_entry.device_id and (device_entry := device_registry.async_get(entity_entry.device_id)):
            device_name = device_entry.name_by_user or device_entry.name
            if device_name:
                suggested_object_id = f"{device_name} {sensor_name}"

        reserved_entity_ids.discard(old_entity_id)
        new_entity_id = registry.async_get_available_entity_id(
            "sensor",
            suggested_object_id,
            current_entity_id=old_entity_id,
            reserved_entity_ids=reserved_entity_ids,
        )
        reserved_entity_ids.add(new_entity_id)

        registry.async_update_entity(
            old_entity_id,
            new_entity_id=new_entity_id,
            new_unique_id=new_unique_id,
        )
        _LOGGER.info(
            "Migrated v0.1.9 MPPT entity %s -> %s to preserve statistics/history",
            old_entity_id,
            new_entity_id,
        )


async def async_remove_legacy_devices(
    hass: HomeAssistant,
    entry: ConfigEntry,
) -> None:
    """Remove device registry entries that only exist for the old device layout."""
    device_registry = dr.async_get(hass)

    removed_devices = 0
    for device in dr.async_entries_for_config_entry(device_registry, entry.entry_id):
        if _legacy_meter_device_needs_removal(device):
            device_registry.async_remove_device(device.id)
            removed_devices += 1

    if removed_devices:
        _LOGGER.info("Removed %s legacy meter devices from pre-web-api config", removed_devices)


async def async_remove_unexpected_entities(
    hass: HomeAssistant,
    entry: ConfigEntry,
    runtime_data: Hub,
) -> None:
    """Drop registry entities that are not part of the current runtime model."""
    registry = er.async_get(hass)
    expected_unique_ids = _expected_entity_unique_ids(runtime_data)
    removed = 0
    for entity_entry in _entity_entries_for_config_entry(registry, entry):
        unique_id = entity_entry.unique_id or ""
        if not unique_id or unique_id in expected_unique_ids:
            continue
        registry.async_remove(entity_entry.entity_id)
        removed += 1

    if removed:
        _LOGGER.info(
            "Removed %s stale entities that are no longer registered by the integration",
            removed,
        )
