from __future__ import annotations

from datetime import timedelta

from homeassistant.core import HomeAssistant

from custom_components.fronius_modbus.hub import FroniusCoordinator


class _RuntimeService:
    def __init__(self) -> None:
        self.calls = 0

    async def async_refresh_data(self) -> dict[str, bool]:
        self.calls += 1
        return {"ok": True}


class _HubWithRuntimeService:
    _id = "hub"
    _scan_interval = timedelta(seconds=10)

    def __init__(self) -> None:
        self.runtime_service = _RuntimeService()


async def test_coordinator_refreshes_via_runtime_service() -> None:
    hub = _HubWithRuntimeService()

    coordinator = FroniusCoordinator(HomeAssistant(), hub)
    result = await coordinator._async_update_data()

    assert result == {"ok": True}
    assert hub.runtime_service.calls == 1
