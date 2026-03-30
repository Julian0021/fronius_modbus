from __future__ import annotations

import types

from custom_components.fronius_modbus.migrations import async_migrate_entry
from test_support.fakes import make_entry


async def test_async_migrate_entry_allows_older_entries_without_rewriting() -> None:
    updates: list[dict[str, object]] = []
    hass = types.SimpleNamespace(
        config_entries=types.SimpleNamespace(
            async_update_entry=lambda entry, **kwargs: updates.append(kwargs)
        )
    )
    entry = make_entry(
        entry_id="entry-1",
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
    entry = make_entry(
        entry_id="entry-2",
        data={},
        options={},
    )

    migrated = await async_migrate_entry(hass, entry)

    assert migrated is True
    assert updates == []
