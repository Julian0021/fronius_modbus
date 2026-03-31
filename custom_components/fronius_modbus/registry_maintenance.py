from __future__ import annotations

from collections import Counter
import logging
import re
from typing import TYPE_CHECKING

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers import device_registry as dr, entity_registry as er

from .button import iter_button_keys
from .const import DOMAIN, ENTITY_PREFIX
from .entity_names import async_resolve_entity_name
from .number import iter_number_keys
from .select import iter_select_keys
from .sensor import iter_sensor_keys
from .switch import iter_switch_keys

if TYPE_CHECKING:
    from .hub import Hub

_LOGGER = logging.getLogger(__name__)

_LEGACY_METER_DEVICE_RE = re.compile(r".*_meter\d+")
_LEGACY_NAME_METER_DEVICE_RE = re.compile(r".*_meter_(\d+)")
_LEGACY_INDEXED_METER_DEVICE_RE = re.compile(r".*_meter(\d+)")
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


def _device_entries_for_config_entry(registry, entry: ConfigEntry):
    """Return all device registry entries attached to a config entry."""
    return list(dr.async_entries_for_config_entry(registry, entry.entry_id))


def _legacy_meter_device_needs_removal(device) -> bool:
    """Return whether a device only exists because of the pre-web-api layout."""
    identifiers = getattr(device, "identifiers", set())
    return any(
        identifier_domain == DOMAIN and _LEGACY_METER_DEVICE_RE.fullmatch(identifier)
        for identifier_domain, identifier in identifiers
    )


def _platform_keys(runtime_data: Hub) -> tuple[str, ...]:
    return (
        *iter_button_keys(runtime_data),
        *iter_number_keys(runtime_data),
        *iter_select_keys(runtime_data),
        *iter_sensor_keys(runtime_data),
        *iter_switch_keys(runtime_data),
    )


def _entity_unique_id(runtime_data: Hub, key: str) -> str:
    """Return the entity unique id derived from the current hub prefix."""
    return f"{runtime_data.entity_prefix}_{key}"


def _expected_entity_unique_ids(runtime_data: Hub) -> set[str]:
    """Return the entity ids derived from the same platform-level key iterators."""
    return {_entity_unique_id(runtime_data, key) for key in _platform_keys(runtime_data)}


def _is_legacy_named_unique_id(unique_id: str, *, key: str, new_unique_id: str) -> bool:
    return (
        unique_id != new_unique_id
        and unique_id.startswith(f"{ENTITY_PREFIX}_")
        and unique_id.endswith(f"_{key}")
    )


def _entry_device_ref_counts(entity_entries) -> Counter[str]:
    return Counter(
        entity_entry.device_id
        for entity_entry in entity_entries
        if getattr(entity_entry, "device_id", None)
    )


def _target_device_identifier(runtime_data: Hub, identifier: str) -> str | None:
    if identifier == runtime_data.inverter_device_identifier:
        return identifier
    if identifier == runtime_data.storage_device_identifier:
        return identifier

    if identifier.endswith("_inverter"):
        return runtime_data.inverter_device_identifier
    if identifier.endswith("_battery_storage"):
        return runtime_data.storage_device_identifier

    if match := _LEGACY_NAME_METER_DEVICE_RE.fullmatch(identifier):
        unit_id = int(match.group(1))
        if unit_id in runtime_data.meter_unit_ids:
            return runtime_data.meter_device_identifier(unit_id)
        return None

    if match := _LEGACY_INDEXED_METER_DEVICE_RE.fullmatch(identifier):
        meter_index = int(match.group(1)) - 1
        if 0 <= meter_index < len(runtime_data.meter_unit_ids):
            return runtime_data.meter_device_identifier(
                runtime_data.meter_unit_ids[meter_index]
            )
    return None


async def async_migrate_legacy_entity_unique_ids(
    hass: HomeAssistant,
    entry: ConfigEntry,
    runtime_data: Hub,
) -> None:
    """Migrate legacy name-based unique ids to the stable entry-based namespace."""
    registry = er.async_get(hass)
    entity_entries = _entity_entries_for_config_entry(registry, entry)

    for key in _platform_keys(runtime_data):
        new_unique_id = _entity_unique_id(runtime_data, key)
        if any((candidate.unique_id or "") == new_unique_id for candidate in entity_entries):
            continue

        for candidate in entity_entries:
            candidate_unique_id = candidate.unique_id or ""
            if not _is_legacy_named_unique_id(
                candidate_unique_id,
                key=key,
                new_unique_id=new_unique_id,
            ):
                continue
            registry.async_update_entity(
                candidate.entity_id,
                new_unique_id=new_unique_id,
            )
            _LOGGER.info(
                "Migrated legacy unique id %s -> %s",
                candidate_unique_id,
                new_unique_id,
            )
            break


async def async_migrate_legacy_devices(
    hass: HomeAssistant,
    entry: ConfigEntry,
    runtime_data: Hub,
) -> None:
    """Migrate legacy name-based device identifiers to the stable entry namespace."""
    device_registry = dr.async_get(hass)
    entity_registry = er.async_get(hass)
    entity_entries = _entity_entries_for_config_entry(entity_registry, entry)
    device_ref_counts = _entry_device_ref_counts(entity_entries)
    device_entries = _device_entries_for_config_entry(device_registry, entry)

    candidates_by_target: dict[str, list[object]] = {}
    for device in device_entries:
        domain_identifiers = {
            identifier
            for identifier_domain, identifier in getattr(device, "identifiers", set())
            if identifier_domain == DOMAIN
        }
        target_identifiers = {
            target_identifier
            for identifier in domain_identifiers
            if (target_identifier := _target_device_identifier(runtime_data, identifier))
            is not None
        }
        if len(target_identifiers) != 1:
            continue
        target_identifier = next(iter(target_identifiers))
        if domain_identifiers == {target_identifier}:
            continue
        candidates_by_target.setdefault(target_identifier, []).append(device)

    for target_identifier, candidates in candidates_by_target.items():
        keeper = max(
            candidates,
            key=lambda device: (device_ref_counts.get(device.id, 0), device.id),
        )
        device_registry.async_update_device(
            keeper.id,
            new_identifiers={(DOMAIN, target_identifier)},
        )
        _LOGGER.info(
            "Migrated legacy device identifiers for %s -> %s",
            keeper.id,
            target_identifier,
        )

        for device in candidates:
            if device is keeper or device_ref_counts.get(device.id, 0):
                continue
            device_registry.async_remove_device(device.id)
            _LOGGER.info(
                "Removed orphaned duplicate legacy device %s after identifier migration",
                device.id,
            )


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
