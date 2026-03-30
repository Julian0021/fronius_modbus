from __future__ import annotations

import types

from homeassistant.config_entries import ConfigEntry

from custom_components.fronius_modbus.const import CONF_RECONFIGURE_REQUIRED
import custom_components.fronius_modbus.entry_reconfigure as entry_reconfigure


async def test_async_prepare_entry_token_sets_reconfigure_required_without_token(
    monkeypatch,
) -> None:
    updates: list[dict[str, object]] = []
    hass = types.SimpleNamespace(
        config_entries=types.SimpleNamespace(
            async_update_entry=lambda entry, **kwargs: updates.append(kwargs)
        )
    )
    entry = ConfigEntry(entry_id="entry-1", data={}, options={})

    class _TokenStore:
        async def async_load_token(self, host, username):
            assert host == "inverter.local"
            assert username == "customer"
            return None

    monkeypatch.setattr(
        entry_reconfigure,
        "async_get_token_store",
        lambda _hass: _TokenStore(),
    )

    token = await entry_reconfigure.async_prepare_entry_token(
        hass,
        entry,
        "inverter.local",
    )

    assert token is None
    assert updates == [
        {
            "data": {CONF_RECONFIGURE_REQUIRED: True},
            "options": {CONF_RECONFIGURE_REQUIRED: True},
        }
    ]


async def test_async_sync_reconfigure_issue_deletes_issue_when_not_needed(
    monkeypatch,
) -> None:
    deleted: list[tuple[object, str, str]] = []
    hass = object()
    entry = ConfigEntry(
        entry_id="entry-1",
        title="Inverter",
        data={CONF_RECONFIGURE_REQUIRED: False},
        options={},
    )

    monkeypatch.setattr(
        entry_reconfigure.ir,
        "async_delete_issue",
        lambda *args: deleted.append(args),
    )

    await entry_reconfigure.async_sync_reconfigure_issue(
        hass,
        entry,
        has_token=True,
    )

    assert deleted == [
        (
            hass,
            "fronius_modbus",
            "legacy_modbus_only_reconfigure_entry-1",
        )
    ]
