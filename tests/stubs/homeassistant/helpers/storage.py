from __future__ import annotations

from typing import Any


class Store:
    _storage: dict[tuple[int, int, str], Any] = {}

    def __init__(self, hass, version: int, key: str) -> None:
        self._storage_key = (id(hass), version, key)

    @classmethod
    def reset_storage(cls) -> None:
        cls._storage.clear()

    def __class_getitem__(cls, _item):
        return cls

    async def async_load(self):
        return self._storage.get(self._storage_key)

    async def async_save(self, data) -> None:
        self._storage[self._storage_key] = data
