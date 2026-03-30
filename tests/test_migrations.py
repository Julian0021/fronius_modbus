from __future__ import annotations

import types

from homeassistant.config_entries import ConfigEntry

from custom_components.fronius_modbus.migrations import async_migrate_entry


async def test_async_migrate_entry_allows_older_entries_without_rewriting() -> None:
    updates: list[dict[str, object]] = []
    hass = types.SimpleNamespace(
        config_entries=types.SimpleNamespace(
            async_update_entry=lambda entry, **kwargs: updates.append(kwargs)
        )
    )
    entry = ConfigEntry(
        entry_id="entry-1",
        version=1,
        minor_version=7,
        data={"legacy": True},
        options={"legacy_option": True},
    )

    migrated = await async_migrate_entry(hass, entry)

    assert migrated is True
    assert updates == []


async def test_async_migrate_entry_allows_newer_entry_versions_without_rewriting() -> None:
    updates: list[dict[str, object]] = []
    hass = types.SimpleNamespace(
        config_entries=types.SimpleNamespace(
            async_update_entry=lambda entry, **kwargs: updates.append(kwargs)
        )
    )
    entry = ConfigEntry(
        entry_id="entry-2",
        version=2,
        minor_version=1,
        data={},
        options={},
    )

    migrated = await async_migrate_entry(hass, entry)

    assert migrated is True
    assert updates == []
