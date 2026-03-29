from __future__ import annotations

import asyncio
from types import SimpleNamespace


def callback(func):
    return func


class HomeAssistant:
    def __init__(self) -> None:
        self.data: dict[str, object] = {}
        self.config = SimpleNamespace(language="en")
        try:
            self.loop = asyncio.get_running_loop()
        except RuntimeError:
            self.loop = None

    async def async_add_executor_job(self, func, *args):
        return func(*args)

    def async_create_task(self, coro):
        return asyncio.create_task(coro)
