from __future__ import annotations

from datetime import timedelta

import pytest
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant

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


async def test_coordinator_refreshes_via_runtime_service() -> None:
    hub = _HubWithRuntimeService()

    coordinator = FroniusCoordinator(HomeAssistant(), hub)
    result = await coordinator._async_update_data()

    assert result == {"ok": True}
    assert hub.runtime_service.calls == 1


def test_coordinator_passes_config_entry_when_supported(monkeypatch) -> None:
    hub = _HubWithRuntimeService()
    config_entry = ConfigEntry(entry_id="entry-1")

    monkeypatch.setattr(hub_module, "_coordinator_init_supports_config_entry", lambda: True)

    coordinator = FroniusCoordinator(HomeAssistant(), hub, config_entry=config_entry)

    assert coordinator.config_entry is config_entry


def test_coordinator_skips_config_entry_when_not_supported(monkeypatch) -> None:
    hub = _HubWithRuntimeService()
    config_entry = ConfigEntry(entry_id="entry-1")

    monkeypatch.setattr(hub_module, "_coordinator_init_supports_config_entry", lambda: False)

    coordinator = FroniusCoordinator(HomeAssistant(), hub, config_entry=config_entry)

    assert coordinator.config_entry is None


def test_coordinator_does_not_mask_constructor_type_errors(monkeypatch) -> None:
    hub = _HubWithRuntimeService()
    config_entry = ConfigEntry(entry_id="entry-1")

    def _raise_type_error(self, hass, logger, **kwargs) -> None:
        raise TypeError(f"bad kwargs: {sorted(kwargs)}")

    monkeypatch.setattr(hub_module, "_coordinator_init_supports_config_entry", lambda: True)
    monkeypatch.setattr(hub_module.DataUpdateCoordinator, "__init__", _raise_type_error)

    with pytest.raises(TypeError, match="bad kwargs"):
        FroniusCoordinator(HomeAssistant(), hub, config_entry=config_entry)
