from __future__ import annotations

import importlib
from types import SimpleNamespace

import pytest
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST
from homeassistant.core import HomeAssistant

integration = importlib.import_module("custom_components.fronius_modbus.__init__")


class _FakeRuntimeData:
    def __init__(
        self,
        *,
        web_api_configured: bool = True,
        bootstrap_error: Exception | None = None,
    ) -> None:
        self.web_api_configured = web_api_configured
        self._bootstrap_error = bootstrap_error
        self.bootstrap_calls: list[ConfigEntry] = []
        self.close_calls = 0
        self.bootstrap_service = SimpleNamespace(init_data=self._async_init_data)

    async def _async_init_data(self, *, config_entry: ConfigEntry) -> None:
        self.bootstrap_calls.append(config_entry)
        if self._bootstrap_error is not None:
            raise self._bootstrap_error

    def close(self) -> None:
        self.close_calls += 1


class _FakeConfigEntries:
    def __init__(self, *, forward_error: Exception | None = None) -> None:
        self.forward_error = forward_error
        self.forward_calls: list[tuple[ConfigEntry, list[str]]] = []

    async def async_forward_entry_setups(
        self,
        entry: ConfigEntry,
        platforms: list[str],
    ) -> None:
        self.forward_calls.append((entry, list(platforms)))
        if self.forward_error is not None:
            raise self.forward_error


def _make_entry() -> ConfigEntry:
    return ConfigEntry(
        data={CONF_HOST: "inverter.local"},
        options={},
        entry_id="entry-1",
        title="Fronius",
    )


def _make_hass(*, forward_error: Exception | None = None) -> HomeAssistant:
    hass = HomeAssistant()
    hass.config_entries = _FakeConfigEntries(forward_error=forward_error)
    return hass


def _patch_setup_dependencies(
    monkeypatch: pytest.MonkeyPatch,
    runtime_data: _FakeRuntimeData,
    issue_calls: list[bool],
    registry_calls: list[tuple[str, object]],
) -> None:
    async def _async_prepare_entry_token(_hass, _entry, _host):
        return "api-token"

    async def _async_sync_reconfigure_issue(_hass, _entry, *, has_token: bool) -> None:
        issue_calls.append(has_token)

    async def _async_migrate_v019_mppt_statistics(_hass, _entry, provided_runtime_data) -> None:
        registry_calls.append(("migrate_mppt", provided_runtime_data))

    async def _async_remove_unexpected_entities(_hass, _entry, provided_runtime_data) -> None:
        registry_calls.append(("remove_unexpected", provided_runtime_data))

    async def _async_remove_legacy_devices(_hass, _entry) -> None:
        registry_calls.append(("remove_legacy", None))

    monkeypatch.setattr(integration.hub, "Hub", lambda **_kwargs: runtime_data)
    monkeypatch.setattr(
        integration.entry_reconfigure,
        "async_prepare_entry_token",
        _async_prepare_entry_token,
    )
    monkeypatch.setattr(
        integration.entry_reconfigure,
        "async_sync_reconfigure_issue",
        _async_sync_reconfigure_issue,
    )
    monkeypatch.setattr(
        integration.registry_maintenance,
        "async_migrate_v019_mppt_statistics",
        _async_migrate_v019_mppt_statistics,
    )
    monkeypatch.setattr(
        integration.registry_maintenance,
        "async_remove_unexpected_entities",
        _async_remove_unexpected_entities,
    )
    monkeypatch.setattr(
        integration.registry_maintenance,
        "async_remove_legacy_devices",
        _async_remove_legacy_devices,
    )


async def test_async_setup_entry_attaches_runtime_data_only_on_success(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_data = _FakeRuntimeData(web_api_configured=True)
    issue_calls: list[bool] = []
    registry_calls: list[tuple[str, object]] = []
    _patch_setup_dependencies(monkeypatch, runtime_data, issue_calls, registry_calls)

    hass = _make_hass()
    entry = _make_entry()

    result = await integration.async_setup_entry(hass, entry)

    assert result is True
    assert entry.runtime_data is runtime_data
    assert runtime_data.bootstrap_calls == [entry]
    assert runtime_data.close_calls == 0
    assert hass.config_entries.forward_calls == [(entry, integration.PLATFORMS)]
    assert issue_calls == [True, True]
    assert registry_calls == [
        ("migrate_mppt", runtime_data),
        ("remove_unexpected", runtime_data),
        ("remove_legacy", None),
    ]
    assert hasattr(entry, "_update_listener")


async def test_async_setup_entry_closes_runtime_data_when_bootstrap_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_data = _FakeRuntimeData(bootstrap_error=RuntimeError("bootstrap failed"))
    issue_calls: list[bool] = []
    registry_calls: list[tuple[str, object]] = []
    _patch_setup_dependencies(monkeypatch, runtime_data, issue_calls, registry_calls)

    hass = _make_hass()
    entry = _make_entry()

    with pytest.raises(RuntimeError, match="bootstrap failed"):
        await integration.async_setup_entry(hass, entry)

    assert not hasattr(entry, "runtime_data")
    assert runtime_data.bootstrap_calls == [entry]
    assert runtime_data.close_calls == 1
    assert hass.config_entries.forward_calls == []
    assert issue_calls == [True]
    assert registry_calls == []
    assert not hasattr(entry, "_update_listener")


async def test_async_setup_entry_closes_and_detaches_runtime_data_when_forwarding_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_data = _FakeRuntimeData(web_api_configured=True)
    issue_calls: list[bool] = []
    registry_calls: list[tuple[str, object]] = []
    _patch_setup_dependencies(monkeypatch, runtime_data, issue_calls, registry_calls)

    hass = _make_hass(forward_error=RuntimeError("forward failed"))
    entry = _make_entry()

    with pytest.raises(RuntimeError, match="forward failed"):
        await integration.async_setup_entry(hass, entry)

    assert not hasattr(entry, "runtime_data")
    assert runtime_data.bootstrap_calls == [entry]
    assert runtime_data.close_calls == 1
    assert hass.config_entries.forward_calls == [(entry, integration.PLATFORMS)]
    assert issue_calls == [True, True]
    assert registry_calls == [
        ("migrate_mppt", runtime_data),
        ("remove_unexpected", runtime_data),
        ("remove_legacy", None),
    ]
    assert not hasattr(entry, "_update_listener")
