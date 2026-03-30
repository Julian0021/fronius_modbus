from __future__ import annotations

import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant

_LOGGER = logging.getLogger(__name__)

async def async_migrate_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Keep legacy entries loadable without rewriting stored config."""
    _LOGGER.debug("Skipping config entry migration for %s", entry.entry_id)
    del hass
    return True
