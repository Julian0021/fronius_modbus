"""Warning and issue-registry helpers for Hub lifecycle."""
from __future__ import annotations

from homeassistant.helpers import issue_registry as ir

from .const import DOMAIN

_SOLAR_API_WARNING_TRANSLATION_KEY = "solar_api_low_firmware"
_SOLAR_API_MINIMUM_VERSION = (1, 40, 7, 1)
_SOLAR_API_MINIMUM_VERSION_TEXT = "1.40.7-1"


class HubWarningService:
    """Own issue-registry policies derived from current hub state."""

    def __init__(self, hub) -> None:
        self._hub = hub

    def solar_api_warning_needed(self) -> bool:
        if not self._hub.web_api_configured:
            return False

        firmware_version = self._hub._parse_firmware_version(
            self._hub.inverter_state.get("i_sw_version")
        )
        if firmware_version is None:
            return False

        solar_api_enabled = self._hub.web_state.get("api_solar_api_enabled")
        if solar_api_enabled is not True:
            return False

        return firmware_version < _SOLAR_API_MINIMUM_VERSION

    async def async_sync_solar_api_warning(self) -> None:
        issue_id = self._hub._solar_api_warning_issue_id()
        if issue_id is None:
            return

        if not self.solar_api_warning_needed():
            ir.async_delete_issue(self._hub._hass, DOMAIN, issue_id)
            return

        current_version = self._hub.inverter_state.get("i_sw_version")
        ir.async_create_issue(
            self._hub._hass,
            DOMAIN,
            issue_id,
            is_fixable=True,
            is_persistent=True,
            severity=ir.IssueSeverity.WARNING,
            translation_key=_SOLAR_API_WARNING_TRANSLATION_KEY,
            translation_placeholders={
                "entry_title": (
                    self._hub._config_entry.title
                    if self._hub._config_entry is not None
                    else self._hub._name
                ),
                "current_version": str(current_version),
                "minimum_version": _SOLAR_API_MINIMUM_VERSION_TEXT,
            },
            data={
                "entry_id": self._hub._config_entry.entry_id,
                "current_version": str(current_version),
                "minimum_version": _SOLAR_API_MINIMUM_VERSION_TEXT,
            },
        )
