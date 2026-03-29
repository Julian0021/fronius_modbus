from __future__ import annotations

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_NAME
from homeassistant.core import HomeAssistant
from homeassistant.helpers import issue_registry as ir

from .config_data import entry_value
from .const import (
    API_USERNAME,
    CONF_RECONFIGURE_REQUIRED,
    DOMAIN,
    MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX,
)
from .token_store import async_get_token_store


def _migration_issue_id(entry: ConfigEntry) -> str:
    """Build the stable repair-issue id for one config entry."""
    return f"{MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX}{entry.entry_id}"


async def _async_set_reconfigure_required(
    hass: HomeAssistant,
    entry: ConfigEntry,
    required: bool,
) -> None:
    """Synchronize the reconfigure flag across config data and options."""
    new_data = dict(entry.data)
    new_options = dict(entry.options)
    changed = False

    if new_data.get(CONF_RECONFIGURE_REQUIRED) != required:
        new_data[CONF_RECONFIGURE_REQUIRED] = required
        changed = True
    if new_options.get(CONF_RECONFIGURE_REQUIRED) != required:
        new_options[CONF_RECONFIGURE_REQUIRED] = required
        changed = True

    if changed:
        hass.config_entries.async_update_entry(entry, data=new_data, options=new_options)


async def async_prepare_entry_token(
    hass: HomeAssistant,
    entry: ConfigEntry,
    host: str,
) -> dict[str, str] | None:
    """Load any saved API token and mirror the reconfigure flag to its presence."""
    token = await async_get_token_store(hass).async_load_token(host, API_USERNAME)
    await _async_set_reconfigure_required(hass, entry, not bool(token))
    return token


async def async_sync_reconfigure_issue(
    hass: HomeAssistant,
    entry: ConfigEntry,
    *,
    has_token: bool,
) -> None:
    """Create or clear the migration repair issue based on token and reconfigure state."""
    issue_id = _migration_issue_id(entry)
    needs_reconfigure = bool(entry_value(entry, CONF_RECONFIGURE_REQUIRED, False)) or not has_token
    if needs_reconfigure:
        ir.async_create_issue(
            hass,
            DOMAIN,
            issue_id,
            is_fixable=True,
            is_persistent=True,
            severity=ir.IssueSeverity.WARNING,
            translation_key="legacy_modbus_only_entry_reconfigure",
            translation_placeholders={
                "entry_title": entry.title or entry_value(entry, CONF_NAME, "Fronius"),
            },
            data={"entry_id": entry.entry_id},
        )
        return

    ir.async_delete_issue(hass, DOMAIN, issue_id)
