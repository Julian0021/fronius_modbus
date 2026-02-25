"""The Fronius Modbus integration."""

from __future__ import annotations

import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.const import Platform
from homeassistant.helpers import entity_registry as er

from homeassistant.const import CONF_NAME, CONF_HOST, CONF_PORT, CONF_SCAN_INTERVAL
from .const import (
    DOMAIN,
    CONF_INVERTER_UNIT_ID,
    CONF_METER_UNIT_ID,
)

from . import hub

_LOGGER = logging.getLogger(__name__)

# List of platforms to support. There should be a matching .py file for each,
# eg <cover.py> and <sensor.py>
PLATFORMS = [Platform.NUMBER, Platform.SELECT, Platform.SENSOR]

type HubConfigEntry = ConfigEntry[hub.Hub]

LEGACY_MPPT_ENTITY_KEYS = (
    "mppt1_current",
    "mppt1_voltage",
    "mppt1_power",
    "mppt1_lfte",
    "mppt2_current",
    "mppt2_voltage",
    "mppt2_power",
    "mppt2_lfte",
    "mppt3_current",
    "mppt3_voltage",
    "mppt3_pv_power",
    "mppt3_pv_lfte",
    "mppt3_power",
    "mppt4_power",
    "mppt3_lfte",
    "mppt4_lfte",
)


def _is_legacy_mppt_unique_id(unique_id: str) -> bool:
    return any(unique_id.endswith(f"_{key}") for key in LEGACY_MPPT_ENTITY_KEYS)


async def _async_remove_legacy_mppt_entities(hass: HomeAssistant, entry: ConfigEntry) -> None:
    registry = er.async_get(hass)
    removed = 0
    for entity_entry in er.async_entries_for_config_entry(registry, entry.entry_id):
        unique_id = entity_entry.unique_id or ""
        if _is_legacy_mppt_unique_id(unique_id):
            registry.async_remove(entity_entry.entity_id)
            removed += 1
    if removed:
        _LOGGER.info("Removed %s legacy MPPT entities", removed)


async def async_setup_entry(hass: HomeAssistant, entry: HubConfigEntry) -> bool:
    """Set up Fronius Modbus from a config entry."""

    name = entry.data[CONF_NAME]
    host = entry.data[CONF_HOST]
    name = entry.data[CONF_NAME]
    port = entry.data[CONF_PORT]
    inverter_unit_id = entry.data.get(CONF_INVERTER_UNIT_ID, 1)
    scan_interval = entry.data[CONF_SCAN_INTERVAL]

    meter_unit_id = entry.data[CONF_METER_UNIT_ID]
    if meter_unit_id and meter_unit_id > 0:
        meter_unit_ids = [meter_unit_id]
    else:
        meter_unit_ids = []

    _LOGGER.debug("Setup %s.%s", DOMAIN, name)

    await _async_remove_legacy_mppt_entities(hass, entry)

    # Store an instance of the "connecting" class that does the work of speaking
    # with your actual devices.
    entry.runtime_data = hub.Hub(hass = hass, name = name, host = host, port = port, inverter_unit_id=inverter_unit_id, meter_unit_ids=meter_unit_ids, scan_interval = scan_interval)
    
    await entry.runtime_data.init_data(config_entry=entry)

    # This creates each HA object for each platform your device requires.
    # It's done by calling the `async_setup_entry` function in each platform module.
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True

async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    # This is called when an entry/configured device is to be removed. The class
    # needs to unload itself, and remove callbacks. See the classes for further
    # details
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)

    return unload_ok

async def reload_service_handler(service: ServiceCall) -> None:
    """Remove all user-defined groups and load new ones from config."""
    conf = None
    with contextlib.suppress(HomeAssistantError):
        conf = await async_integration_yaml_config(hass, DOMAIN)
    if conf is None:
        return
    await async_reload_integration_platforms(hass, DOMAIN, PLATFORMS)
    _async_setup_shared_data(hass)
    await _async_process_config(hass, conf)    
