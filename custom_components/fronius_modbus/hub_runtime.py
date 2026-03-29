from __future__ import annotations

import logging

from homeassistant.helpers.update_coordinator import UpdateFailed

from .integration_errors import FroniusError

_LOGGER = logging.getLogger(__name__)

LOAD_MAX_SAMPLE_SKEW_SECONDS = 0.5
LOAD_GLITCH_MIN_EXPORT_W = 1000.0
LOAD_GLITCH_MAX_ABS_INVERTER_W = 50.0
LOAD_GLITCH_MIN_PV_W = 1000.0
LOAD_STORAGE_CHARGE_MIN_W = 1000.0


class HubRuntimeService:
    def __init__(self, hub) -> None:
        self._hub = hub

    async def async_refresh_data(self) -> dict:
        self._hub.derived_state.set("load", None)
        self._hub._client.start_load_poll_cycle()
        try:
            await self._hub._client.read_service.read_inverter_data()
        except FroniusError as err:
            core_err: FroniusError = err

        else:
            self._handle_core_modbus_success()
            await self._async_refresh_optional_data()
            return self._hub.snapshot_data()

        if self._handle_core_modbus_failure(core_err):
            return self._hub.snapshot_data()
        raise UpdateFailed(f"Fronius data update failed: {core_err}")

    async def _async_refresh_optional_data(self) -> None:
        self._hub.derived_state.set("load", None)
        await self._async_optional_poll(
            "inverter status",
            self._hub._client.read_service.read_inverter_status_data,
        )
        await self._async_optional_poll(
            "inverter settings",
            self._hub._client.read_service.read_inverter_model_settings_data,
        )
        await self._async_optional_poll(
            "inverter controls",
            self._hub._client.read_service.read_inverter_controls_data,
        )

        if self._hub.meter_configured:
            if self._hub.primary_meter_unit_id not in self._hub.meter_unit_ids:
                self._hub.derived_state.set("load", None)
                self._hub.derived_state.set("grid_status", None)

            for meter_address in self._hub.meter_unit_ids:
                await self._async_optional_poll(
                    f"meter {meter_address}",
                    self._hub._client.read_service.read_meter_data,
                    unit_id=meter_address,
                    is_primary=meter_address == self._hub.primary_meter_unit_id,
                )

        if self._hub.mppt_configured:
            await self._async_optional_poll(
                "mppt",
                self._hub._client.read_service.read_mppt_data,
            )

        await self._async_optional_poll(
            "ac limit",
            self._hub._client.read_service.read_ac_limit_data,
        )

        if self._hub.storage_configured:
            await self._async_optional_poll(
                "storage",
                self._hub._client.read_service.read_inverter_storage_data,
            )

        self._apply_modbus_load_data()

        if self._hub.web_api_configured:
            try:
                await self._hub.web_api_service.refresh_web_data()
            except FroniusError as err:
                _LOGGER.warning("Fronius web API refresh failed: %s", err)

    async def _async_optional_poll(self, label: str, func, *args, **kwargs) -> bool:
        try:
            await func(*args, **kwargs)
        except FroniusError as err:
            _LOGGER.warning("Optional Fronius %s refresh failed: %s", label, err)
            return False
        return True

    def _handle_core_modbus_success(self) -> None:
        if self._hub._battery_write_transition_active():
            _LOGGER.debug(
                "Modbus recovered after battery API write on %s",
                self._hub._host,
            )
        self._hub._clear_battery_write_transition()

    def _handle_core_modbus_failure(self, err: FroniusError) -> bool:
        if not self._hub._battery_write_transition_active():
            return False

        if not self._hub._battery_write_transition_warned:
            _LOGGER.warning(
                "Suppressing temporary Modbus outage after battery API write on %s: %s",
                self._hub._host,
                err,
            )
            self._hub._battery_write_transition_warned = True
        else:
            _LOGGER.debug(
                "Modbus still recovering after battery API write on %s: %s",
                self._hub._host,
                err,
            )
        return True

    def _apply_modbus_load_data(self) -> None:
        self._hub.derived_state.set("load", None)
        if not self._hub._client.meter_configured:
            self._hub._reset_bad_load_tracking()
            return

        primary_unit_id = self._hub._client.primary_meter_unit_id
        meter_location = self._hub._as_int(
            self._hub.meter_value(primary_unit_id, "location")
        )
        meter_power = self._hub.meter_value(primary_unit_id, "power")
        inverter_power = self._hub.inverter_state.get("acpower")
        inverter_sample_ts, meter_sample_ts = self._hub._client.get_load_sample_timestamps(
            primary_unit_id
        )
        if (
            meter_sample_ts is None
            or meter_power is None
            or not self._hub._client.is_numeric(meter_power)
        ):
            self._hub._reset_bad_load_tracking()
            return
        meter_power_f = float(meter_power)

        if meter_location == 1 or (
            meter_location is not None and 256 <= meter_location <= 511
        ):
            self._hub._set_load(-meter_power_f, cache=meter_power_f <= 0)
            return

        if meter_location != 0:
            self._hub._reset_bad_load_tracking()
            return

        if (
            inverter_sample_ts is None
            or inverter_power is None
            or not self._hub._client.is_numeric(inverter_power)
            or abs(inverter_sample_ts - meter_sample_ts) > LOAD_MAX_SAMPLE_SKEW_SECONDS
        ):
            self._hub._reset_bad_load_tracking()
            return
        inverter_power_f = float(inverter_power)

        candidate_load = meter_power_f + inverter_power_f
        if self._hub._is_inverter_power_glitch(
            candidate_load=candidate_load,
            meter_power=meter_power_f,
            inverter_power=inverter_power_f,
        ):
            self._hub._apply_glitch_load_fallback()
            return

        storage_charge_power = self._hub.storage_state.get("storage_charge_power")
        if (
            candidate_load < 0
            and self._hub.storage_configured
            and self._hub._client.is_numeric(storage_charge_power)
            and float(storage_charge_power) >= LOAD_STORAGE_CHARGE_MIN_W
        ):
            self._hub._reset_bad_load_tracking()
            return

        self._hub._set_load(
            max(candidate_load, 0.0),
            cache=True,
            inverter_power=inverter_power_f,
        )
