from __future__ import annotations

from types import SimpleNamespace

import pytest

from custom_components.fronius_modbus.froniusmodbus_runtime import (
    FroniusModbusRuntimeService,
)
from custom_components.fronius_modbus.integration_errors import FroniusReadError


class _RuntimeFacade:
    def __init__(
        self,
        *,
        meter_unit_ids: list[int],
        primary_meter_unit_id: int,
        failing_meter_unit_ids: set[int] | None = None,
    ) -> None:
        self._host = "fixture-host"
        self._port = 502
        self._inverter_unit_id = 1
        self._meter_unit_ids = list(meter_unit_ids)
        self._primary_meter_unit_id = int(primary_meter_unit_id)
        self.mppt_configured = False
        self.meter_configured = bool(meter_unit_ids)
        self.data: dict[str, object] = {}
        self._failing_meter_unit_ids = set(failing_meter_unit_ids or set())
        self.set_meter_unit_ids_calls: list[tuple[list[int], int | None]] = []
        self.read_service = SimpleNamespace(
            read_device_info_data=self._read_device_info_data,
            read_mppt_data=self._read_mppt_data,
            read_inverter_nameplate_data=self._read_inverter_nameplate_data,
        )

    @property
    def meter_unit_ids(self) -> tuple[int, ...]:
        return tuple(self._meter_unit_ids)

    @property
    def primary_meter_unit_id(self) -> int:
        return self._primary_meter_unit_id

    async def connect(self) -> bool:
        return True

    def _meter_prefix(self, unit_id: int) -> str:
        return f"meter_{int(unit_id)}_"

    async def _read_device_info_data(self, *, prefix: str, unit_id: int) -> bool:
        if prefix == "i_":
            self.data["i_model"] = "Verto"
            return True
        if unit_id in self._failing_meter_unit_ids:
            raise FroniusReadError("meter probe failed")

        self.data[f"{prefix}manufacturer"] = "Fronius"
        self.data[f"{prefix}model"] = "Smart Meter TS"
        return True

    async def _read_mppt_data(self) -> bool:
        return False

    async def _read_inverter_nameplate_data(self) -> bool:
        return True

    def set_meter_unit_ids(
        self,
        unit_ids: list[int] | tuple[int, ...] | None,
        primary_unit_id: int | None = None,
    ) -> None:
        normalized = [int(unit_id) for unit_id in unit_ids or []]
        self.set_meter_unit_ids_calls.append((normalized, primary_unit_id))
        self._meter_unit_ids = normalized
        if primary_unit_id is not None and int(primary_unit_id) in normalized:
            self._primary_meter_unit_id = int(primary_unit_id)
        elif normalized:
            self._primary_meter_unit_id = normalized[0]


@pytest.mark.asyncio
async def test_runtime_service_keeps_primary_meter_aligned_with_discovered_units() -> None:
    facade = _RuntimeFacade(
        meter_unit_ids=[200, 201],
        primary_meter_unit_id=200,
        failing_meter_unit_ids={200},
    )
    service = FroniusModbusRuntimeService(facade)

    assert await service.init_data() is True
    assert facade.set_meter_unit_ids_calls[-1] == ([201], 200)
    assert facade.meter_unit_ids == (201,)
    assert facade.primary_meter_unit_id == 201


@pytest.mark.asyncio
async def test_runtime_service_keeps_previous_primary_meter_when_discovery_fails() -> None:
    facade = _RuntimeFacade(
        meter_unit_ids=[200, 201],
        primary_meter_unit_id=201,
        failing_meter_unit_ids={200, 201},
    )
    service = FroniusModbusRuntimeService(facade)

    assert await service.init_data() is True
    assert facade.set_meter_unit_ids_calls[-1] == ([200, 201], 201)
    assert facade.meter_unit_ids == (200, 201)
    assert facade.primary_meter_unit_id == 201
