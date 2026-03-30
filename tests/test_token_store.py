from __future__ import annotations

from types import SimpleNamespace

import pytest

from custom_components.fronius_modbus.token_store import FroniusTokenStore


@pytest.mark.asyncio
async def test_async_save_token_keeps_cache_unchanged_when_persistence_fails(monkeypatch) -> None:
    hass = SimpleNamespace(data={})
    token_store = FroniusTokenStore(hass)

    await token_store.async_save_token("inverter.local", "realm", "persisted")

    async def _failing_save(_data) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(token_store._store, "async_save", _failing_save)

    with pytest.raises(RuntimeError, match="boom"):
        await token_store.async_save_token("new-host.local", "realm", "new-token")

    assert await token_store.async_load_token("inverter.local") == {
        "realm": "realm",
        "token": "persisted",
    }
    assert await token_store.async_load_token("new-host.local") is None


@pytest.mark.asyncio
async def test_async_delete_token_keeps_cache_unchanged_when_persistence_fails(monkeypatch) -> None:
    hass = SimpleNamespace(data={})
    token_store = FroniusTokenStore(hass)

    await token_store.async_save_token("inverter.local", "realm", "persisted")

    async def _failing_save(_data) -> None:
        raise RuntimeError("boom")

    monkeypatch.setattr(token_store._store, "async_save", _failing_save)

    with pytest.raises(RuntimeError, match="boom"):
        await token_store.async_delete_token("inverter.local")

    assert await token_store.async_load_token("inverter.local") == {
        "realm": "realm",
        "token": "persisted",
    }
