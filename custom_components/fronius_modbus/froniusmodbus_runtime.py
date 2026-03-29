"""Runtime and discovery helpers for the Fronius Modbus client."""
from __future__ import annotations

import logging

from .integration_errors import FroniusDiscoveryError, FroniusError

_LOGGER = logging.getLogger(__name__)


class FroniusModbusRuntimeService:
    """Own discovery, capability probing, and SunSpec topology state."""

    def __init__(self, facade) -> None:
        self._facade = facade

    async def init_data(self):
        """Probe inverter capabilities and cache the devices that are present."""
        previous_mppt_configured = self._facade.mppt_configured
        previous_meter_unit_ids = list(self._facade._meter_unit_ids)

        try:
            await self._facade.connect()
            await self._facade.read_service.read_device_info_data(
                prefix="i_",
                unit_id=self._facade._inverter_unit_id,
            )
        except FroniusError as err:
            _LOGGER.error(
                "Error reading inverter info %s:%s unit id %s",
                self._facade._host,
                self._facade._port,
                self._facade._inverter_unit_id,
                exc_info=True,
            )
            raise FroniusDiscoveryError(
                f"Error reading inverter info unit id: {self._facade._inverter_unit_id}"
            ) from err

        try:
            if await self._facade.read_service.read_mppt_data():
                self._facade.mppt_configured = True
        except FroniusError as err:
            self._facade.mppt_configured = previous_mppt_configured
            _LOGGER.warning("Error while checking mppt data: %s", err)

        discovered_meter_unit_ids: list[int] = []
        meter_probe_failed = False
        for unit_id in self._facade._meter_unit_ids:
            prefix = self._facade._meter_prefix(unit_id)
            try:
                await self._facade.read_service.read_device_info_data(
                    prefix=prefix,
                    unit_id=unit_id,
                )
            except FroniusError:
                meter_probe_failed = True
                _LOGGER.debug(
                    "Meter info probe failed for configured unit %s on %s:%s",
                    unit_id,
                    self._facade._host,
                    self._facade._port,
                    exc_info=True,
                )
                continue

            manufacturer = str(self._facade.data.get(prefix + "manufacturer") or "").strip()
            model = str(self._facade.data.get(prefix + "model") or "").strip()
            if manufacturer == "Fronius" and "meter" in model.lower():
                discovered_meter_unit_ids.append(unit_id)

        if discovered_meter_unit_ids or not meter_probe_failed:
            self._facade._meter_unit_ids = discovered_meter_unit_ids
        else:
            self._facade._meter_unit_ids = previous_meter_unit_ids
            _LOGGER.debug(
                "Keeping existing smart meter unit ids for %s:%s after discovery probe failures: %s",
                self._facade._host,
                self._facade._port,
                previous_meter_unit_ids,
            )
        self._facade.meter_configured = bool(self._facade._meter_unit_ids)

        if self._facade.meter_configured:
            _LOGGER.info(
                "Configured Fronius smart meter unit ids on %s:%s: %s",
                self._facade._host,
                self._facade._port,
                self._facade._meter_unit_ids,
            )

        try:
            await self._facade.read_service.read_inverter_nameplate_data()
        except FroniusError:
            _LOGGER.error("Error reading nameplate data", exc_info=True)
        else:
            if self._facade.mppt_configured:
                try:
                    await self._facade.read_service.read_mppt_data()
                except FroniusError as err:
                    _LOGGER.warning("Error refreshing mppt data after nameplate probe: %s", err)

        _LOGGER.debug("Init done. data: %s", self._facade.data)
        return True
