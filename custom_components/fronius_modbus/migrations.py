from __future__ import annotations

import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant

from .const import (
    CONF_METER_UNIT_ID,
    CONF_METER_UNIT_IDS,
    CONF_RECONFIGURE_REQUIRED,
)

_LOGGER = logging.getLogger(__name__)

_TARGET_VERSION = 1
_TARGET_MINOR_VERSION = 8


async def async_migrate_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Migrate old config entries to the current stored config shape."""
    _LOGGER.debug(
        "Migrating config entry %s version=%s minor=%s",
        entry.entry_id,
        entry.version,
        entry.minor_version,
    )

    if entry.version > _TARGET_VERSION:
        _LOGGER.error("Unsupported config entry version: %s", entry.version)
        return False

    if entry.version == _TARGET_VERSION and entry.minor_version < _TARGET_MINOR_VERSION:
        new_data = dict(entry.data)
        new_options = dict(entry.options)

        new_data.pop(CONF_METER_UNIT_ID, None)
        new_data.pop(CONF_METER_UNIT_IDS, None)
        new_options.pop(CONF_METER_UNIT_ID, None)
        new_options.pop(CONF_METER_UNIT_IDS, None)
        new_data[CONF_RECONFIGURE_REQUIRED] = True
        new_options[CONF_RECONFIGURE_REQUIRED] = True

        hass.config_entries.async_update_entry(
            entry,
            data=new_data,
            options=new_options,
            version=_TARGET_VERSION,
            minor_version=_TARGET_MINOR_VERSION,
        )

    return True
