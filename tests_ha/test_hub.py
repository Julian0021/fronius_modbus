from __future__ import annotations

from datetime import timedelta

import pytest
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.fronius_modbus.const import DOMAIN
import custom_components.fronius_modbus.hub as hub_module
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


@pytest.mark.asyncio
async def test_coordinator_refreshes_via_runtime_service(hass) -> None:
    hub = _HubWithRuntimeService()

    coordinator = FroniusCoordinator(hass, hub)
    result = await coordinator._async_update_data()

    assert result == {"ok": True}
    assert hub.runtime_service.calls == 1


def test_coordinator_passes_config_entry_when_supported(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    hub = _HubWithRuntimeService()
    config_entry = MockConfigEntry(domain=DOMAIN)

    monkeypatch.setattr(hub_module, "_coordinator_init_supports_config_entry", lambda: True)

    coordinator = FroniusCoordinator(hass, hub, config_entry=config_entry)

    assert coordinator.config_entry is config_entry


def test_coordinator_skips_config_entry_when_not_supported(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    hub = _HubWithRuntimeService()
    config_entry = MockConfigEntry(domain=DOMAIN)

    monkeypatch.setattr(hub_module, "_coordinator_init_supports_config_entry", lambda: False)

    coordinator = FroniusCoordinator(hass, hub, config_entry=config_entry)

    assert coordinator.config_entry is None


def test_coordinator_does_not_mask_constructor_type_errors(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    hub = _HubWithRuntimeService()
    config_entry = MockConfigEntry(domain=DOMAIN)

    def _raise_type_error(self, hass, logger, **kwargs) -> None:
        raise TypeError(f"bad kwargs: {sorted(kwargs)}")

    monkeypatch.setattr(hub_module, "_coordinator_init_supports_config_entry", lambda: True)
    monkeypatch.setattr(hub_module.DataUpdateCoordinator, "__init__", _raise_type_error)

    with pytest.raises(TypeError, match="bad kwargs"):
        FroniusCoordinator(hass, hub, config_entry=config_entry)
