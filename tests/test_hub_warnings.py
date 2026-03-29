from __future__ import annotations

from homeassistant.core import HomeAssistant
from homeassistant.helpers import issue_registry as ir

from custom_components.fronius_modbus.const import DOMAIN
from custom_components.fronius_modbus.hub_warnings import HubWarningService


class _WarningHub:
    def __init__(self) -> None:
        self.web_api_configured = True
        self.inverter_state = {"i_sw_version": "1.40.0-0"}
        self.web_state = {"api_solar_api_enabled": True}
        self.hass = HomeAssistant()
        self.solar_api_warning_issue_id = "solar-api-warning"
        self.warning_entry_id = "entry-1"
        self.warning_entry_title = "Fronius"

    @staticmethod
    def parse_firmware_version(_version_text: object):
        return (1, 40, 0, 0)


async def test_warning_service_uses_public_hub_boundary_for_issue_sync() -> None:
    hub = _WarningHub()
    service = HubWarningService(hub)

    await service.async_sync_solar_api_warning()

    issue = ir.async_get(hub.hass).async_get_issue(DOMAIN, hub.solar_api_warning_issue_id)
    assert issue is not None
    assert issue.translation_placeholders == {
        "entry_title": "Fronius",
        "current_version": "1.40.0-0",
        "minimum_version": "1.40.7-1",
    }
    assert issue.data == {
        "entry_id": "entry-1",
        "current_version": "1.40.0-0",
        "minimum_version": "1.40.7-1",
    }
