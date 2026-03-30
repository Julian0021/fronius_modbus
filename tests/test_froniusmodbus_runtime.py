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
        meter_probe_results: dict[int, bool | Exception],
    ) -> None:
        self._host = "fixture-host"
        self._port = 502
        self._inverter_unit_id = 1
        self._meter_address_offset = 200
        self._meter_unit_ids = list(meter_unit_ids)
        self._primary_meter_unit_id = primary_meter_unit_id
        self.meter_configured = bool(meter_unit_ids)
        self.mppt_configured = False
        self.data: dict[str, object] = {}
        self.set_meter_unit_ids_calls: list[tuple[list[int], int | None]] = []
        self._meter_probe_results = dict(meter_probe_results)
        self.read_service = SimpleNamespace(
            read_device_info_data=self._read_device_info_data,
            read_mppt_data=self._read_mppt_data,
            read_inverter_nameplate_data=self._read_inverter_nameplate_data,
        )

    @property
    def primary_meter_unit_id(self) -> int:
        return self._primary_meter_unit_id

    @property
    def meter_unit_ids(self) -> tuple[int, ...]:
        return tuple(self._meter_unit_ids)

    async def connect(self) -> bool:
        return True

    def _meter_prefix(self, unit_id: int) -> str:
        return f"meter_{int(unit_id)}_"

    def set_meter_unit_ids(
        self,
        unit_ids: list[int] | tuple[int, ...] | None,
        primary_unit_id: int | None = None,
    ) -> None:
        self.set_meter_unit_ids_calls.append((list(unit_ids or []), primary_unit_id))
        normalized = list(unit_ids or [])
        self._meter_unit_ids = normalized
        if primary_unit_id is not None and primary_unit_id in normalized:
            self._primary_meter_unit_id = int(primary_unit_id)
        elif normalized:
            self._primary_meter_unit_id = normalized[0]
        else:
            self._primary_meter_unit_id = self._meter_address_offset

    async def _read_device_info_data(self, *, prefix: str, unit_id: int) -> bool:
        if prefix == "i_":
            return True

        probe_result = self._meter_probe_results[int(unit_id)]
        if isinstance(probe_result, Exception):
            raise probe_result

        if probe_result:
            self.data[f"{prefix}manufacturer"] = "Fronius"
            self.data[f"{prefix}model"] = "Smart Meter TS 65A-3"
        return bool(probe_result)

    async def _read_mppt_data(self) -> bool:
        return False

    async def _read_inverter_nameplate_data(self) -> bool:
        return True


@pytest.mark.asyncio
async def test_runtime_init_realigns_primary_meter_when_discovery_changes_meter_set() -> None:
    facade = _RuntimeFacade(
        meter_unit_ids=[200, 240],
        primary_meter_unit_id=240,
        meter_probe_results={
            200: True,
            240: FroniusReadError("temporary meter probe failure"),
        },
    )
    service = FroniusModbusRuntimeService(facade)

    assert await service.init_data() is True
    assert facade.meter_unit_ids == (200,)
    assert facade.primary_meter_unit_id == 200
    assert facade.set_meter_unit_ids_calls[-1] == ([200], None)


@pytest.mark.asyncio
async def test_runtime_init_preserves_primary_meter_when_discovery_falls_back() -> None:
    facade = _RuntimeFacade(
        meter_unit_ids=[200, 240],
        primary_meter_unit_id=240,
        meter_probe_results={
            200: FroniusReadError("temporary meter probe failure"),
            240: FroniusReadError("temporary meter probe failure"),
        },
    )
    service = FroniusModbusRuntimeService(facade)

    assert await service.init_data() is True
    assert facade.meter_unit_ids == (200, 240)
    assert facade.primary_meter_unit_id == 240
    assert facade.set_meter_unit_ids_calls[-1] == ([200, 240], 240)
