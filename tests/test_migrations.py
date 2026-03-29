from __future__ import annotations

import types

from homeassistant.config_entries import ConfigEntry

from custom_components.fronius_modbus.const import (
    CONF_METER_UNIT_ID,
    CONF_METER_UNIT_IDS,
    CONF_RECONFIGURE_REQUIRED,
)
from custom_components.fronius_modbus.migrations import async_migrate_entry


async def test_async_migrate_entry_only_updates_legacy_config_shape() -> None:
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
        data={CONF_METER_UNIT_ID: 200, CONF_METER_UNIT_IDS: [200]},
        options={CONF_METER_UNIT_ID: 201, CONF_METER_UNIT_IDS: [201]},
    )

    migrated = await async_migrate_entry(hass, entry)

    assert migrated is True
    assert updates == [
        {
            "data": {CONF_RECONFIGURE_REQUIRED: True},
            "options": {CONF_RECONFIGURE_REQUIRED: True},
            "version": 1,
            "minor_version": 8,
        }
    ]
