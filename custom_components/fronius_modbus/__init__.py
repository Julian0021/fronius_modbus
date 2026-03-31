"""The Fronius Modbus integration."""

from __future__ import annotations

import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import (
    CONF_HOST,
    CONF_NAME,
    CONF_PORT,
    CONF_SCAN_INTERVAL,
    Platform,
)
from homeassistant.core import HomeAssistant

from . import entry_reconfigure, hub, migrations, registry_maintenance
from .config_data import merged_entry_config
from .const import (
    API_USERNAME,
    CONF_INVERTER_UNIT_ID,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_INVERTER_UNIT_ID,
    DEFAULT_METER_UNIT_IDS,
    DEFAULT_NAME,
    DEFAULT_PORT,
    DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_SCAN_INTERVAL,
    DOMAIN,
)

_LOGGER = logging.getLogger(__name__)

PLATFORMS = [Platform.SELECT, Platform.SWITCH, Platform.NUMBER, Platform.SENSOR, Platform.BUTTON]

HubConfigEntry = ConfigEntry[hub.Hub]

async def async_migrate_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Migrate old config entries."""
    return await migrations.async_migrate_entry(hass, entry)


async def _async_reload_entry(hass: HomeAssistant, entry: ConfigEntry) -> None:
    await hass.config_entries.async_reload(entry.entry_id)


async def async_setup_entry(hass: HomeAssistant, entry: HubConfigEntry) -> bool:
    """Set up Fronius Modbus from a config entry."""
    config = merged_entry_config(entry)
    name = config.get(CONF_NAME, DEFAULT_NAME)
    host = config.get(CONF_HOST)
    port = config.get(CONF_PORT, DEFAULT_PORT)
    inverter_unit_id = config.get(CONF_INVERTER_UNIT_ID, DEFAULT_INVERTER_UNIT_ID)
    scan_interval = config.get(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL)
    restrict_modbus_to_this_ip = config.get(
        CONF_RESTRICT_MODBUS_TO_THIS_IP,
        DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
    )

    _LOGGER.debug("Setup %s.%s", DOMAIN, name)

    api_token = await entry_reconfigure.async_prepare_entry_token(hass, entry, host)
    await entry_reconfigure.async_sync_reconfigure_issue(
        hass,
        entry,
        has_token=api_token is not None,
    )

    runtime_data = hub.Hub(
        hass=hass,
        name=name,
        host=host,
        port=port,
        inverter_unit_id=inverter_unit_id,
        meter_unit_ids=list(DEFAULT_METER_UNIT_IDS),
        scan_interval=scan_interval,
        api_username=API_USERNAME if api_token else None,
        api_token=api_token,
        auto_enable_modbus=False,
        restrict_modbus_to_this_ip=restrict_modbus_to_this_ip,
    )

    try:
        await runtime_data.bootstrap_service.init_data(config_entry=entry)
        await registry_maintenance.async_migrate_legacy_entity_unique_ids(
            hass,
            entry,
            runtime_data,
        )
        await registry_maintenance.async_migrate_v019_mppt_statistics(
            hass,
            entry,
            runtime_data,
        )
        await registry_maintenance.async_remove_unexpected_entities(
            hass,
            entry,
            runtime_data,
            preserve_topology_sensitive_entities=not runtime_data.entity_registry_cleanup_safe,
        )
        await registry_maintenance.async_migrate_legacy_devices(
            hass,
            entry,
            runtime_data,
        )
        await registry_maintenance.async_remove_legacy_devices(hass, entry)
        await entry_reconfigure.async_sync_reconfigure_issue(
            hass,
            entry,
            has_token=runtime_data.web_api_configured,
        )

        entry.runtime_data = runtime_data
        await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
        entry.async_on_unload(entry.add_update_listener(_async_reload_entry))
    except Exception:
        if getattr(entry, "runtime_data", None) is runtime_data:
            delattr(entry, "runtime_data")
        runtime_data.close()
        raise
    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    if unload_ok and getattr(entry, "runtime_data", None) is not None:
        entry.runtime_data.close()

    return unload_ok
