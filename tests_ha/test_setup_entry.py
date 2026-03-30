from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock

from homeassistant.config_entries import ConfigEntryState
from homeassistant.const import CONF_HOST, CONF_NAME
import pytest
from pytest_homeassistant_custom_component.common import MockConfigEntry

import custom_components.fronius_modbus as integration
from custom_components.fronius_modbus.const import DEFAULT_NAME, DOMAIN
import custom_components.fronius_modbus.hub_bootstrap as hub_bootstrap_module


class _FakeRuntimeData:
    def __init__(
        self,
        *,
        web_api_configured: bool = True,
        entity_registry_cleanup_safe: bool = True,
        bootstrap_error: Exception | None = None,
    ) -> None:
        self.web_api_configured = web_api_configured
        self.entity_registry_cleanup_safe = entity_registry_cleanup_safe
        self._bootstrap_error = bootstrap_error
        self.bootstrap_calls: list[MockConfigEntry] = []
        self.close_calls = 0
        self.bootstrap_service = SimpleNamespace(init_data=self._async_init_data)

    async def _async_init_data(self, *, config_entry) -> None:
        self.bootstrap_calls.append(config_entry)
        if self._bootstrap_error is not None:
            raise self._bootstrap_error

    def close(self) -> None:
        self.close_calls += 1


def _make_entry() -> MockConfigEntry:
    return MockConfigEntry(
        domain=DOMAIN,
        title="Fronius",
        data={
            CONF_HOST: "inverter.local",
            CONF_NAME: DEFAULT_NAME,
        },
    )


def _patch_setup_dependencies(
    hass,
    monkeypatch: pytest.MonkeyPatch,
    runtime_data: _FakeRuntimeData,
    issue_calls: list[bool],
    registry_calls: list[tuple[str, object]],
    *,
    forward_error: Exception | None = None,
) -> None:
    async def _async_prepare_entry_token(_hass, _entry, _host):
        return "api-token"

    async def _async_sync_reconfigure_issue(_hass, _entry, *, has_token: bool) -> None:
        issue_calls.append(has_token)

    async def _async_migrate_v019_mppt_statistics(_hass, _entry, provided_runtime_data) -> None:
        registry_calls.append(("migrate_mppt", provided_runtime_data))

    async def _async_remove_unexpected_entities(
        _hass,
        _entry,
        provided_runtime_data,
        *,
        preserve_topology_sensitive_entities: bool = False,
    ) -> None:
        registry_calls.append(
            (
                "remove_unexpected",
                provided_runtime_data,
                preserve_topology_sensitive_entities,
            )
        )

    async def _async_remove_legacy_devices(_hass, _entry) -> None:
        registry_calls.append(("remove_legacy", None))

    async def _async_forward_entry_setups(entry, platforms):
        if forward_error is not None:
            raise forward_error
        return True

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
    monkeypatch.setattr(
        hass.config_entries,
        "async_forward_entry_setups",
        _async_forward_entry_setups,
    )


@pytest.mark.asyncio
async def test_async_setup_and_unload_entry_with_real_homeassistant(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    entry = MockConfigEntry(
        domain=DOMAIN,
        title="Fronius",
        data={
            CONF_HOST: "inverter.local",
            CONF_NAME: DEFAULT_NAME,
        },
    )
    entry.add_to_hass(hass)

    prepare_entry_token = AsyncMock(return_value=None)
    sync_reconfigure_issue = AsyncMock()
    migrate_mppt = AsyncMock()
    remove_unexpected = AsyncMock()
    remove_legacy = AsyncMock()
    bootstrap_init = AsyncMock(return_value=None)
    forward_entry_setups = AsyncMock(return_value=True)
    unload_platforms = AsyncMock(return_value=True)
    close = Mock()

    monkeypatch.setattr(
        integration.entry_reconfigure,
        "async_prepare_entry_token",
        prepare_entry_token,
    )
    monkeypatch.setattr(
        integration.entry_reconfigure,
        "async_sync_reconfigure_issue",
        sync_reconfigure_issue,
    )
    monkeypatch.setattr(
        integration.registry_maintenance,
        "async_migrate_v019_mppt_statistics",
        migrate_mppt,
    )
    monkeypatch.setattr(
        integration.registry_maintenance,
        "async_remove_unexpected_entities",
        remove_unexpected,
    )
    monkeypatch.setattr(
        integration.registry_maintenance,
        "async_remove_legacy_devices",
        remove_legacy,
    )
    monkeypatch.setattr(
        hub_bootstrap_module.HubBootstrapService,
        "init_data",
        bootstrap_init,
    )
    monkeypatch.setattr(
        hass.config_entries,
        "async_forward_entry_setups",
        forward_entry_setups,
    )
    monkeypatch.setattr(
        hass.config_entries,
        "async_unload_platforms",
        unload_platforms,
    )
    monkeypatch.setattr(integration.hub.Hub, "close", close)

    assert await hass.config_entries.async_setup(entry.entry_id)
    await hass.async_block_till_done()

    assert entry.state is ConfigEntryState.LOADED
    assert isinstance(entry.runtime_data, integration.hub.Hub)
    prepare_entry_token.assert_awaited_once()
    assert sync_reconfigure_issue.await_count == 2
    bootstrap_init.assert_awaited_once()
    forward_entry_setups.assert_awaited_once_with(entry, integration.PLATFORMS)
    migrate_mppt.assert_awaited_once()
    remove_unexpected.assert_awaited_once()
    remove_legacy.assert_awaited_once()

    assert await hass.config_entries.async_unload(entry.entry_id)
    await hass.async_block_till_done()

    assert entry.state is ConfigEntryState.NOT_LOADED
    unload_platforms.assert_awaited_once_with(entry, integration.PLATFORMS)
    close.assert_called_once()


@pytest.mark.asyncio
async def test_async_setup_entry_closes_runtime_data_when_bootstrap_fails_with_patched_runtime_on_real_ha(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_data = _FakeRuntimeData(bootstrap_error=RuntimeError("bootstrap failed"))
    issue_calls: list[bool] = []
    registry_calls: list[tuple[str, object]] = []
    _patch_setup_dependencies(hass, monkeypatch, runtime_data, issue_calls, registry_calls)

    entry = _make_entry()
    entry.add_to_hass(hass)

    with pytest.raises(RuntimeError, match="bootstrap failed"):
        await integration.async_setup_entry(hass, entry)

    assert not hasattr(entry, "runtime_data")
    assert runtime_data.bootstrap_calls == [entry]
    assert runtime_data.close_calls == 1
    assert issue_calls == [True]
    assert registry_calls == []
    assert not hasattr(entry, "_update_listener")


@pytest.mark.asyncio
async def test_async_setup_entry_closes_and_detaches_runtime_data_when_forwarding_fails_with_patched_runtime_on_real_ha(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_data = _FakeRuntimeData(web_api_configured=True)
    issue_calls: list[bool] = []
    registry_calls: list[tuple[str, object]] = []
    _patch_setup_dependencies(
        hass,
        monkeypatch,
        runtime_data,
        issue_calls,
        registry_calls,
        forward_error=RuntimeError("forward failed"),
    )

    entry = _make_entry()
    entry.add_to_hass(hass)

    with pytest.raises(RuntimeError, match="forward failed"):
        await integration.async_setup_entry(hass, entry)

    assert not hasattr(entry, "runtime_data")
    assert runtime_data.bootstrap_calls == [entry]
    assert runtime_data.close_calls == 1
    assert issue_calls == [True, True]
    assert registry_calls == [
        ("migrate_mppt", runtime_data),
        ("remove_unexpected", runtime_data, False),
        ("remove_legacy", None),
    ]
    assert not hasattr(entry, "_update_listener")


@pytest.mark.asyncio
async def test_async_setup_entry_preserves_topology_entities_when_patched_runtime_model_is_uncertain_on_real_ha(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime_data = _FakeRuntimeData(
        web_api_configured=True,
        entity_registry_cleanup_safe=False,
    )
    issue_calls: list[bool] = []
    registry_calls: list[tuple[str, object]] = []
    _patch_setup_dependencies(hass, monkeypatch, runtime_data, issue_calls, registry_calls)

    entry = _make_entry()
    entry.add_to_hass(hass)

    result = await integration.async_setup_entry(hass, entry)

    assert result is True
    assert entry.runtime_data is runtime_data
    assert runtime_data.close_calls == 0
    assert issue_calls == [True, True]
    assert registry_calls == [
        ("migrate_mppt", runtime_data),
        ("remove_unexpected", runtime_data, True),
        ("remove_legacy", None),
    ]
