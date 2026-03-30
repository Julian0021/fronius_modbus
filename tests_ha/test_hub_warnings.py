from __future__ import annotations

from homeassistant.helpers import issue_registry as ir
import pytest

from custom_components.fronius_modbus.const import DOMAIN
from custom_components.fronius_modbus.hub_warnings import HubWarningService


class _WarningHub:
    def __init__(self, hass) -> None:
        self.web_api_configured = True
        self.inverter_state = {"i_sw_version": "1.40.0-0"}
        self.web_state = {"api_solar_api_enabled": True}
        self.hass = hass
        self.solar_api_warning_issue_id = "solar-api-warning"
        self.warning_entry_id = "entry-1"
        self.warning_entry_title = "Fronius"

    @staticmethod
    def parse_firmware_version(_version_text: object):
        return (1, 40, 0, 0)


@pytest.mark.asyncio
async def test_warning_service_uses_public_hub_boundary_for_issue_sync(hass) -> None:
    hub = _WarningHub(hass)
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
