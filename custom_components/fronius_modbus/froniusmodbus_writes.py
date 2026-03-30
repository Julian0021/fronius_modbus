"""Write and control helpers for the Fronius Modbus client."""
from __future__ import annotations

import asyncio
import logging
import time

from .control_values import (
    ac_limit_raw_to_percent,
    ac_limit_raw_to_watts,
    ac_limit_watts_to_raw,
    coerce_scale_factor,
    power_factor_raw_to_value,
    power_factor_value_to_raw,
    signed_percent_to_register,
)
from .extmodbusclient import ModbusReadError
from .froniusmodbusclient_const import (
    AC_LIMIT_ENABLE_ADDRESS,
    AC_LIMIT_RATE_ADDRESS,
    AC_LIMIT_STATUS,
    CHARGE_RATE_OFFSET,
    CONN_ADDRESS,
    CONTROL_STATE_DISABLED,
    CONTROL_STATE_ENABLED,
    CONTROL_STATUS,
    DEFAULT_MINIMUM_RESERVE_PERCENT,
    DISCHARGE_RATE_OFFSET,
    MINIMUM_RESERVE_OFFSET,
    MINIMUM_RESERVE_PERCENT_MIN,
    OUT_PF_SET_ADDRESS,
    OUT_PF_SET_ENABLE_ADDRESS,
    STORAGE_CONTROL_MODE_OFFSET,
    UINT16_MODULUS,
)
from .storage_modes import (
    STORAGE_MODE_CAPABILITY_CHARGE_LIMIT,
    STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT,
    STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
    STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER,
    SUPPORTED_STORAGE_CONTROL_MODES,
    StorageExtendedControlMode,
    get_storage_mode_policy,
    storage_mode_supports,
)

_LOGGER = logging.getLogger(__name__)
APPLY_TOGGLE_DELAY_SECONDS = 1.0
APPLY_TOGGLE_MASK_SECONDS = APPLY_TOGGLE_DELAY_SECONDS + 0.5


class FroniusModbusWriteService:
    """Own write-side controls, conversions, and apply toggles."""

    def __init__(self, facade) -> None:
        self._facade = facade

    def _get_ac_limit_rate_sf(self) -> int | None:
        value = self._facade.data.get("ac_limit_rate_sf")
        rate_sf = coerce_scale_factor(value)
        if rate_sf is None and self._facade.is_numeric(value):
            _LOGGER.error("Invalid AC limit scale factor: %s", value)
        return rate_sf

    def _get_inverter_max_power_w(self) -> float | None:
        value = self._facade.data.get("max_power")
        if not self._facade.is_numeric(value):
            return None
        max_power_w = float(value)
        if max_power_w <= 0:
            return None
        return max_power_w

    def _ac_limit_raw_to_percent(self, raw_value: int) -> float | None:
        percent = ac_limit_raw_to_percent(raw_value, self._get_ac_limit_rate_sf())
        if percent is None and self._facade.is_numeric(raw_value):
            _LOGGER.error(
                "AC limit percent out of range: raw=%s sf=%s",
                raw_value,
                self._get_ac_limit_rate_sf(),
            )
        return percent

    def _ac_limit_raw_to_watts(self, raw_value: int) -> int | None:
        return ac_limit_raw_to_watts(
            raw_value,
            self._get_ac_limit_rate_sf(),
            self._get_inverter_max_power_w(),
        )

    def _get_power_factor_sf(self) -> int | None:
        value = self._facade.data.get("power_factor_sf")
        power_factor_sf = coerce_scale_factor(value)
        if power_factor_sf is None and self._facade.is_numeric(value):
            _LOGGER.error("Invalid power factor scale factor: %s", value)
        return power_factor_sf

    def _power_factor_raw_to_value(self, raw_value: int) -> float | None:
        value = power_factor_raw_to_value(raw_value, self._get_power_factor_sf())
        if value is None and self._facade.is_numeric(raw_value):
            _LOGGER.error(
                "Power factor out of range: raw=%s sf=%s",
                raw_value,
                self._get_power_factor_sf(),
            )
        return value

    def _power_factor_value_to_raw(self, value: float) -> int | None:
        return power_factor_value_to_raw(value, self._get_power_factor_sf())

    def _ac_limit_watts_to_raw(self, watts: float) -> int | None:
        return ac_limit_watts_to_raw(
            watts,
            self._get_ac_limit_rate_sf(),
            self._get_inverter_max_power_w(),
        )

    async def _read_enable_raw(self, address: int) -> int | None:
        try:
            regs = await self._facade.get_registers(
                unit_id=self._facade._inverter_unit_id,
                address=address,
                count=1,
            )
        except ModbusReadError:
            return None

        enable_raw = self._facade._client.convert_from_registers(
            regs[0:1],
            data_type=self._facade._client.DATATYPE.UINT16,
        )
        if not self._facade.is_numeric(enable_raw):
            return None
        return int(enable_raw)

    async def _read_ac_limit_enable_raw(self) -> int | None:
        return await self._read_enable_raw(AC_LIMIT_ENABLE_ADDRESS)

    async def _read_power_factor_enable_raw(self) -> int | None:
        return await self._read_enable_raw(OUT_PF_SET_ENABLE_ADDRESS)

    def _set_ac_limit_enable_state(self, enable_raw: int) -> None:
        self._facade.data["ac_limit_enable"] = AC_LIMIT_STATUS.get(enable_raw, "unknown")

    def _set_ac_limit_control_state(self, enable_raw: int) -> None:
        self._facade.data["WMaxLim_Ena"] = self._facade._map_value(
            CONTROL_STATUS,
            enable_raw,
            "throttle control",
        )
        self._set_ac_limit_enable_state(enable_raw)

    def _set_power_factor_enable_state(self, enable_raw: int) -> None:
        status = self._facade._map_value(
            CONTROL_STATUS,
            enable_raw,
            "fixed power factor",
        )
        self._facade.data["OutPFSet_Ena"] = status
        self._facade.data["power_factor_enable"] = status

    def _set_ac_limit_rate_values(self, raw_value: int | None) -> None:
        self._facade.data["ac_limit_rate_raw"] = raw_value
        self._facade.data["ac_limit_rate_pct"] = self._ac_limit_raw_to_percent(raw_value)
        self._facade.data["ac_limit_rate"] = self._ac_limit_raw_to_watts(raw_value)

    def _rate_watts_to_percent(self, value_w: float, max_rate_w: float) -> float:
        if value_w > max_rate_w:
            return 100
        if value_w < max_rate_w * -1:
            return -100
        return value_w / max_rate_w * 100

    async def _write_signed_percent_register(self, address: int, rate: float) -> None:
        raw_rate = signed_percent_to_register(rate)
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=address,
            payload=[raw_rate],
        )

    async def _set_named_mode(
        self,
        *,
        mode: int,
        charge_limit: float,
        discharge_limit: float,
        extended_mode: int,
        log_message: str,
    ) -> None:
        await self.change_settings(
            mode=mode,
            charge_limit=charge_limit,
            discharge_limit=discharge_limit,
            extended_mode=extended_mode,
        )
        _LOGGER.info(log_message)

    async def _pulse_enable_for_apply(
        self,
        *,
        read_enable_raw,
        enable_address: int,
        mask_attr: str,
        set_enabled_state,
    ) -> tuple[int | None, bool]:
        """Temporarily disable an enabled control so the following write is applied."""
        enable_raw = await read_enable_raw()
        was_enabled = enable_raw == CONTROL_STATE_ENABLED

        if not was_enabled:
            setattr(self._facade, mask_attr, 0.0)
            if enable_raw is not None:
                set_enabled_state(int(enable_raw))
            return enable_raw, False

        setattr(
            self._facade,
            mask_attr,
            time.monotonic() + APPLY_TOGGLE_MASK_SECONDS,
        )
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=enable_address,
            payload=[CONTROL_STATE_DISABLED],
        )
        return enable_raw, True

    async def _refresh_storage_state(self) -> None:
        await self._facade.read_service.read_inverter_storage_data()

    async def _refresh_inverter_controls_state(self) -> None:
        await self._facade.read_service.read_inverter_controls_data()

    async def _refresh_ac_limit_state(self) -> None:
        await self._facade.read_service.read_inverter_controls_data()
        await self._facade.read_service.read_ac_limit_data()

    async def _write_storage_control_mode(self, mode: int) -> None:
        if mode not in SUPPORTED_STORAGE_CONTROL_MODES:
            _LOGGER.error(
                "Attempted to set unsupported storage control mode value=%s",
                mode,
            )
            return
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=self._facade._storage_register_address(STORAGE_CONTROL_MODE_OFFSET),
            payload=[mode],
        )

    async def _write_minimum_reserve(self, minimum_reserve: float) -> None:
        minimum_reserve = int(minimum_reserve) * 100
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=self._facade._storage_register_address(MINIMUM_RESERVE_OFFSET),
            payload=[minimum_reserve],
        )

    async def _write_discharge_rate_percent(self, discharge_rate: float) -> None:
        await self._write_signed_percent_register(
            self._facade._storage_register_address(DISCHARGE_RATE_OFFSET),
            discharge_rate,
        )

    async def _write_charge_rate_percent(self, charge_rate: float) -> None:
        await self._write_signed_percent_register(
            self._facade._storage_register_address(CHARGE_RATE_OFFSET),
            charge_rate,
        )

    async def set_storage_control_mode(self, mode: int):
        await self._write_storage_control_mode(mode)
        await self._refresh_storage_state()

    async def set_power_factor(self, power_factor: float):
        raw_value = self._power_factor_value_to_raw(power_factor)
        if raw_value is None:
            raise ValueError("Power factor must be between -1 and 1")
        power_factor_enable_raw, was_enabled = await self._pulse_enable_for_apply(
            read_enable_raw=self._read_power_factor_enable_raw,
            enable_address=OUT_PF_SET_ENABLE_ADDRESS,
            mask_attr="_power_factor_enable_mask_until",
            set_enabled_state=self._set_power_factor_enable_state,
        )
        if raw_value < 0:
            raw_value = UINT16_MODULUS + raw_value
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=OUT_PF_SET_ADDRESS,
            payload=[raw_value],
        )
        if was_enabled:
            await asyncio.sleep(APPLY_TOGGLE_DELAY_SECONDS)
            await self._facade.write_registers(
                unit_id=self._facade._inverter_unit_id,
                address=OUT_PF_SET_ENABLE_ADDRESS,
                payload=[CONTROL_STATE_ENABLED],
            )
        await self._refresh_inverter_controls_state()
        _LOGGER.info(
            "Set power factor to %s (enable_before=%s, pulsed_enable=%s)",
            self._facade.data.get("power_factor"),
            power_factor_enable_raw,
            was_enabled,
        )

    async def set_power_factor_enable(self, enable: int):
        if enable not in (CONTROL_STATE_DISABLED, CONTROL_STATE_ENABLED):
            raise ValueError(f"Unsupported power factor control state: {enable}")
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=OUT_PF_SET_ENABLE_ADDRESS,
            payload=[enable],
        )
        self._facade._power_factor_enable_mask_until = 0.0
        await self._refresh_inverter_controls_state()

    async def set_minimum_reserve(self, minimum_reserve: float):
        if not float(minimum_reserve).is_integer():
            raise ValueError("SoC Minimum must be a whole number")
        if minimum_reserve < MINIMUM_RESERVE_PERCENT_MIN:
            _LOGGER.error(
                "Attempted to set SoC Minimum below %s%% value=%s",
                MINIMUM_RESERVE_PERCENT_MIN,
                minimum_reserve,
            )
            return
        await self._write_minimum_reserve(minimum_reserve)
        await self._refresh_storage_state()

    async def set_discharge_rate_w(self, discharge_rate_w):
        await self._write_discharge_rate_percent(
            self._rate_watts_to_percent(
                discharge_rate_w,
                self._facade.max_discharge_rate_w,
            )
        )
        await self._refresh_storage_state()

    async def set_discharge_rate(self, discharge_rate):
        await self._write_discharge_rate_percent(discharge_rate)
        await self._refresh_storage_state()

    async def set_charge_rate_w(self, charge_rate_w):
        await self._write_charge_rate_percent(
            self._rate_watts_to_percent(
                charge_rate_w,
                self._facade.max_charge_rate_w,
            )
        )
        await self._refresh_storage_state()

    async def set_grid_charge_power(self, value):
        """Value is in W from HA, store percent internally."""
        if not storage_mode_supports(
            self._facade.storage_extended_control_mode,
            STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
        ):
            return
        await self._write_discharge_rate_percent(
            self._rate_watts_to_percent(
                value * -1,
                self._facade.max_discharge_rate_w,
            )
        )
        await self._refresh_storage_state()

    async def set_grid_discharge_power(self, value):
        """Value is in W from HA, store percent internally."""
        if not storage_mode_supports(
            self._facade.storage_extended_control_mode,
            STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER,
        ):
            return
        await self._write_charge_rate_percent(
            self._rate_watts_to_percent(
                value * -1,
                self._facade.max_charge_rate_w,
            )
        )
        await self._refresh_storage_state()

    async def set_charge_limit(self, value):
        """Value is in W from HA, store percent internally."""
        if not storage_mode_supports(
            self._facade.storage_extended_control_mode,
            STORAGE_MODE_CAPABILITY_CHARGE_LIMIT,
        ):
            return
        await self._write_charge_rate_percent(
            self._rate_watts_to_percent(
                value,
                self._facade.max_charge_rate_w,
            )
        )
        await self._refresh_storage_state()

    async def set_discharge_limit(self, value):
        """Value is in W from HA, store percent internally."""
        if not storage_mode_supports(
            self._facade.storage_extended_control_mode,
            STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT,
        ):
            return
        await self._write_discharge_rate_percent(
            self._rate_watts_to_percent(
                value,
                self._facade.max_discharge_rate_w,
            )
        )
        await self._refresh_storage_state()

    async def set_charge_rate(self, charge_rate):
        await self._write_charge_rate_percent(charge_rate)
        await self._refresh_storage_state()

    async def change_settings(
        self,
        mode,
        charge_limit,
        discharge_limit,
        minimum_reserve=None,
        extended_mode: int | None = None,
    ):
        effective_mode = (
            self._facade.storage_extended_control_mode
            if extended_mode is None
            else extended_mode
        )
        await self._write_storage_control_mode(mode)
        await self._write_charge_rate_percent(charge_limit)
        await self._write_discharge_rate_percent(discharge_limit)
        if minimum_reserve is not None:
            await self._write_minimum_reserve(minimum_reserve)
        await self._refresh_storage_state()
        if effective_mode != self._facade.storage_extended_control_mode:
            _LOGGER.debug(
                "Storage mode refresh returned %s instead of requested %s",
                self._facade.storage_extended_control_mode,
                effective_mode,
            )

    async def restore_defaults(self):
        policy = get_storage_mode_policy(StorageExtendedControlMode.AUTO)
        if policy is None:
            raise ValueError("Auto storage mode policy is not configured")
        await self.change_settings(
            mode=policy.modbus_control_mode,
            charge_limit=policy.default_charge_limit,
            discharge_limit=policy.default_discharge_limit,
            minimum_reserve=DEFAULT_MINIMUM_RESERVE_PERCENT,
            extended_mode=int(StorageExtendedControlMode.AUTO),
        )
        _LOGGER.info("Restored defaults")

    async def set_extended_mode(self, mode: int | StorageExtendedControlMode) -> None:
        policy = get_storage_mode_policy(mode)
        if policy is None:
            raise ValueError(f"Unsupported storage mode: {mode}")
        await self._set_named_mode(
            mode=policy.modbus_control_mode,
            charge_limit=policy.default_charge_limit,
            discharge_limit=policy.default_discharge_limit,
            extended_mode=int(StorageExtendedControlMode(int(mode))),
            log_message=policy.log_message,
        )

    async def set_auto_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.AUTO)

    async def set_charge_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.PV_CHARGE_LIMIT)

    async def set_discharge_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.DISCHARGE_LIMIT)

    async def set_charge_discharge_mode(self):
        await self.set_extended_mode(
            StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT
        )

    async def set_grid_charge_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.CHARGE_FROM_GRID)

    async def set_grid_discharge_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.DISCHARGE_TO_GRID)

    async def set_block_discharge_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.BLOCK_DISCHARGING)

    async def set_block_charge_mode(self):
        await self.set_extended_mode(StorageExtendedControlMode.BLOCK_CHARGING)

    async def set_ac_limit_rate(self, rate):
        """Set AC limit rate in watts and write WMaxLimPct raw value."""
        raw_rate = self._ac_limit_watts_to_raw(rate)
        if raw_rate is None:
            _LOGGER.error("Cannot set AC limit rate, missing max power or scale factor")
            return

        ac_limit_enable_raw, was_enabled = await self._pulse_enable_for_apply(
            read_enable_raw=self._read_ac_limit_enable_raw,
            enable_address=AC_LIMIT_ENABLE_ADDRESS,
            mask_attr="_ac_limit_enable_mask_until",
            set_enabled_state=self._set_ac_limit_control_state,
        )

        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=AC_LIMIT_RATE_ADDRESS,
            payload=[round(raw_rate)],
        )

        if was_enabled:
            await asyncio.sleep(APPLY_TOGGLE_DELAY_SECONDS)
            await self._facade.write_registers(
                unit_id=self._facade._inverter_unit_id,
                address=AC_LIMIT_ENABLE_ADDRESS,
                payload=[CONTROL_STATE_ENABLED],
            )
        await self._refresh_ac_limit_state()
        _LOGGER.info(
            "Set AC limit rate to %s W (raw=%s, enable_before=%s, pulsed_enable=%s)",
            self._facade.data.get("ac_limit_rate"),
            raw_rate,
            ac_limit_enable_raw,
            was_enabled,
        )

    async def set_ac_limit_enable(self, enable):
        """Enable or disable AC limit (0=Disabled, 1=Enabled)."""
        enable_value = CONTROL_STATE_ENABLED if enable else CONTROL_STATE_DISABLED
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=AC_LIMIT_ENABLE_ADDRESS,
            payload=[enable_value],
        )
        self._facade._ac_limit_enable_mask_until = 0.0
        await self._refresh_ac_limit_state()
        _LOGGER.info("Set AC limit enable to %s", enable_value)

    async def set_conn_status(self, enable):
        """Enable or disable inverter connection."""
        conn_value = CONTROL_STATE_ENABLED if enable else CONTROL_STATE_DISABLED
        await self._facade.write_registers(
            unit_id=self._facade._inverter_unit_id,
            address=CONN_ADDRESS,
            payload=[conn_value],
        )
        await self._refresh_inverter_controls_state()
        _LOGGER.info(
            "Set inverter connection status to %s (%s)",
            conn_value,
            "Connected" if enable else "Disconnected/Standby",
        )
