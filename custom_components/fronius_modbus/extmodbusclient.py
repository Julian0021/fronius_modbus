"""Extended Modbus client helpers."""

from __future__ import annotations

import asyncio
import logging

from pymodbus import ExceptionResponse
from pymodbus.client import AsyncModbusTcpClient
from pymodbus.exceptions import ConnectionException, ModbusIOException

from .integration_errors import FroniusConnectionError

_LOGGER = logging.getLogger(__name__)


class ModbusTransportError(FroniusConnectionError):
    """Base transport error raised by the modbus helper."""


class ModbusReadError(ModbusTransportError):
    """Raised when a register read fails."""


class ModbusWriteError(ModbusTransportError):
    """Raised when a register write fails."""


class ExtModbusClient:
    def __init__(self, host: str, port: int, unit_id: int, timeout: int, framer: str = None) -> None:
        """Initialize the Modbus client."""
        self._host = host
        self._port = port
        self._unit_id = unit_id
        if framer is not None:
            self._client = AsyncModbusTcpClient(
                host=host,
                port=port,
                framer=framer,
                timeout=timeout,
            )
        else:
            self._client = AsyncModbusTcpClient(host=host, port=port, timeout=timeout)

    def close(self):
        """Disconnect client."""
        self._client.close()

    async def connect(self, retries=3):
        """Connect client."""
        for attempt in range(1, retries + 1):
            if attempt > 1:
                _LOGGER.debug(
                    "Connect retry attempt %s/%s to %s:%s",
                    attempt,
                    retries,
                    self._host,
                    self._port,
                )
                await asyncio.sleep(0.2)
            connected = await self._client.connect()
            if connected:
                _LOGGER.debug(
                    "Successfully connected to %s:%s",
                    self._client.comm_params.host,
                    self._client.comm_params.port,
                )
                return True
        raise ModbusTransportError(
            f"Failed to connect to {self._host}:{self._port} retries: {retries}"
        )

    async def _check_and_reconnect(self):
        """Reconnect lazily so read/write helpers can treat disconnects as transient."""
        if not self._client.connected:
            _LOGGER.warning("Modbus client is not connected, reconnecting...")
            return await self.connect()
        return self._client.connected

    @property
    def connected(self) -> bool:
        return self._client.connected

    async def read_holding_registers(self, unit_id, address, count):
        """Read holding registers once."""
        await self._check_and_reconnect()
        try:
            return await self._client.read_holding_registers(
                address=address,
                count=count,
                device_id=unit_id,
            )
        except (ModbusIOException, ConnectionException) as err:
            raise ModbusReadError(
                f"Failed reading registers address={address} count={count} unit_id={unit_id}"
            ) from err

    async def get_registers(self, unit_id, address, count, retries=1):
        """Read registers with limited retries for transport and response failures."""
        last_error: ModbusReadError | None = None
        for attempt in range(retries + 1):
            try:
                data = await self.read_holding_registers(
                    unit_id=unit_id,
                    address=address,
                    count=count,
                )
            except ModbusReadError as err:
                last_error = err
                if attempt >= retries:
                    raise
                _LOGGER.debug(
                    "Retrying register read %s/%s for address=%s count=%s unit_id=%s after transport failure",
                    attempt + 1,
                    retries,
                    address,
                    count,
                    unit_id,
                )
                await asyncio.sleep(0.2)
                continue

            if not data.isError():
                return data.registers

            last_error = ModbusReadError(
                "Register read failed for "
                f"address={address} count={count} unit_id={unit_id}: {data}"
            )
            if attempt >= retries:
                raise last_error

            level = logging.DEBUG if isinstance(data, ExceptionResponse) else logging.WARNING
            _LOGGER.log(
                level,
                "Retrying register read %s/%s for address=%s count=%s unit_id=%s after response error: %s",
                attempt + 1,
                retries,
                address,
                count,
                unit_id,
                data,
            )
            await asyncio.sleep(0.2)

        if last_error is not None:
            raise last_error
        raise ModbusReadError(
            f"Register read failed address={address} count={count} unit_id={unit_id}"
        )

    async def write_registers(self, unit_id, address, payload):
        """Write registers."""
        await self._check_and_reconnect()

        try:
            result = await self._client.write_registers(
                address=address,
                values=payload,
                device_id=unit_id,
            )
        except (ModbusIOException, ConnectionException) as err:
            raise ModbusWriteError(
                f"Failed writing registers address={address} unit_id={unit_id}"
            ) from err

        if result.isError():
            raise ModbusWriteError(
                f"Write response error address={address} unit_id={unit_id}: {result}"
            )

        return result
