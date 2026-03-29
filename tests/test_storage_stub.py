from __future__ import annotations

from homeassistant.core import HomeAssistant
from homeassistant.helpers.storage import Store


async def test_storage_stub_reset_storage_clears_process_global_state() -> None:
    hass = HomeAssistant()
    store = Store(hass, 1, "example")

    await store.async_save({"value": 1})

    assert await store.async_load() == {"value": 1}

    Store.reset_storage()

    assert await store.async_load() is None
