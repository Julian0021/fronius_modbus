from __future__ import annotations

import pytest

from custom_components.fronius_modbus.hub_runtime import HubRuntimeService


class _ReadService:
    async def read_inverter_data(self) -> None:
        raise RuntimeError("unexpected failure")


class _Client:
    def __init__(self) -> None:
        self.read_service = _ReadService()

    def start_load_poll_cycle(self) -> None:
        return None


class _DerivedState:
    def set(self, _key: str, _value) -> None:
        return None


class _RuntimeHub:
    def __init__(self) -> None:
        self.derived_state = _DerivedState()
        self._client = _Client()


async def test_async_refresh_data_propagates_unexpected_core_failures() -> None:
    service = HubRuntimeService(_RuntimeHub())

    with pytest.raises(RuntimeError, match="unexpected failure"):
        await service.async_refresh_data()
