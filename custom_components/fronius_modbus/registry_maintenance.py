from __future__ import annotations

import logging
import re
from typing import TYPE_CHECKING

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers import device_registry as dr
from homeassistant.helpers import entity_registry as er

from .button import iter_button_keys
from .const import (
    DOMAIN,
)
from .entity_names import async_resolve_entity_name
from .number import iter_number_keys
from .select import iter_select_keys
from .sensor import iter_sensor_keys
from .switch import iter_switch_keys

if TYPE_CHECKING:
    from .hub import Hub

_LOGGER = logging.getLogger(__name__)

_LEGACY_METER_DEVICE_RE = re.compile(r".*_meter\d+")
_TOPOLOGY_SENSITIVE_ENTITY_KEY_PREFIXES = (
    "meter_",
    "mppt_module_",
    "storage_charge_",
    "storage_discharge_",
)
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


def _expected_entity_unique_ids(runtime_data: Hub) -> set[str]:
    """Return the entity ids derived from the same platform-level key iterators."""
    platform_keys = (
        *iter_button_keys(runtime_data),
        *iter_number_keys(runtime_data),
        *iter_select_keys(runtime_data),
        *iter_sensor_keys(runtime_data),
        *iter_switch_keys(runtime_data),
    )
    return {_entity_unique_id(runtime_data, key) for key in platform_keys}


def _is_topology_sensitive_unique_id(runtime_data: Hub, unique_id: str) -> bool:
    """Return whether a unique id belongs to a soft-discovered topology entity."""
    entity_prefix = f"{runtime_data.entity_prefix}_"
    if not unique_id.startswith(entity_prefix):
        return False
    key = unique_id.removeprefix(entity_prefix)
    return key.startswith(_TOPOLOGY_SENSITIVE_ENTITY_KEY_PREFIXES)


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
    *,
    preserve_topology_sensitive_entities: bool = False,
) -> None:
    """Drop registry entities that are not part of the current runtime model."""
    registry = er.async_get(hass)
    expected_unique_ids = _expected_entity_unique_ids(runtime_data)
    removed = 0
    for entity_entry in _entity_entries_for_config_entry(registry, entry):
        unique_id = entity_entry.unique_id or ""
        if not unique_id or unique_id in expected_unique_ids:
            continue
        if (
            preserve_topology_sensitive_entities
            and _is_topology_sensitive_unique_id(runtime_data, unique_id)
        ):
            continue
        registry.async_remove(entity_entry.entity_id)
        removed += 1

    if removed:
        _LOGGER.info(
            "Removed %s stale entities that are no longer registered by the integration",
            removed,
        )
