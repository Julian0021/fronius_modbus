from __future__ import annotations

from types import SimpleNamespace

from pymodbus.exceptions import ConnectionException
import pytest

from custom_components.fronius_modbus.extmodbusclient import (
    ExtModbusClient,
    ModbusReadError,
    ModbusWriteError,
)


class _FakeClient:
    connected = True

    def __init__(self, *, read_exception=None, write_exception=None) -> None:
        self._read_exception = read_exception
        self._write_exception = write_exception

    async def read_holding_registers(self, **_kwargs):
        if self._read_exception is not None:
            raise self._read_exception
        return SimpleNamespace(isError=lambda: False, registers=[1, 2, 3])

    async def write_registers(self, **_kwargs):
        if self._write_exception is not None:
            raise self._write_exception
        return SimpleNamespace(isError=lambda: False)


def _build_client(*, read_exception=None, write_exception=None) -> ExtModbusClient:
    client = object.__new__(ExtModbusClient)
    client._host = "inverter.local"
    client._port = 502
    client._client = _FakeClient(
        read_exception=read_exception,
        write_exception=write_exception,
    )
    client._check_and_reconnect = _noop_reconnect
    return client


async def _noop_reconnect() -> bool:
    return True


async def test_read_holding_registers_wraps_transport_failures() -> None:
    client = _build_client(read_exception=ConnectionException("offline"))

    with pytest.raises(ModbusReadError, match="Failed reading registers"):
        await client.read_holding_registers(unit_id=1, address=40100, count=2)


async def test_read_holding_registers_preserves_unexpected_failures() -> None:
    client = _build_client(read_exception=ValueError("decode bug"))

    with pytest.raises(ValueError, match="decode bug"):
        await client.read_holding_registers(unit_id=1, address=40100, count=2)


async def test_write_registers_wraps_transport_failures() -> None:
    client = _build_client(write_exception=ConnectionException("offline"))

    with pytest.raises(ModbusWriteError, match="Failed writing registers"):
        await client.write_registers(unit_id=1, address=40300, payload=[1])


async def test_write_registers_preserves_unexpected_failures() -> None:
    client = _build_client(write_exception=RuntimeError("serializer bug"))

    with pytest.raises(RuntimeError, match="serializer bug"):
        await client.write_registers(unit_id=1, address=40300, payload=[1])
