from __future__ import annotations


class _AsyncCallService:
    def __init__(self, calls: list[tuple[str, tuple, dict]]) -> None:
        self._calls = calls

    def __getattr__(self, name: str):
        async def _record(*args, **kwargs):
            self._calls.append((name, args, kwargs))

        return _record


def _entity_by_key(entities, key: str):
    return next(entity for entity in entities if entity._key == key)
