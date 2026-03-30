from __future__ import annotations

import asyncio
from types import SimpleNamespace


class FakeHass:
    def __init__(self, *, language: str | None = None) -> None:
        self.config = SimpleNamespace(language=language)

    async def async_add_executor_job(self, func, *args):
        return func(*args)

    def async_create_task(self, coro):
        return asyncio.create_task(coro)


def make_entry(**kwargs):
    entry = {
        "entry_id": "entry-1",
        "title": "Fronius",
        "data": {},
        "options": {},
    }
    entry.update(kwargs)
    return SimpleNamespace(**entry)
