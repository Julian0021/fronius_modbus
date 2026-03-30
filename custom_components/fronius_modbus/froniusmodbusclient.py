"""Fronius Modbus client state plus read, write, and runtime services."""
from __future__ import annotations

from typing import Any

from .extmodbusclient import ExtModbusClient
from .froniusmodbus_reads import FroniusModbusReadService
from .froniusmodbus_runtime import FroniusModbusRuntimeService
from .froniusmodbus_writes import FroniusModbusWriteService
from .froniusmodbusclient_const import STORAGE_ADDRESS
from .modbus_values import (
    bitmask_to_string,
    calculate_scaled_value,
    is_numeric_value,
    strip_control_chars,
)
from .runtime_state import FroniusRuntimeState, StateSection


class FroniusModbusClient(ExtModbusClient):
    """Shared Modbus client state with explicit read/write/runtime services."""

    def __init__(
        self,
        host: str,
        port: int,
        inverter_unit_id: int,
        meter_unit_ids,
        timeout: int,
    ) -> None:
        """Initialize the client facade and internal services."""
        super().__init__(
            host=host,
            port=port,
            unit_id=inverter_unit_id,
            timeout=timeout,
        )

        self.initialized = False

        self._inverter_unit_id = inverter_unit_id
        self._meter_address_offset = int(meter_unit_ids[0]) if meter_unit_ids else 200
        self._meter_unit_ids = [self._meter_address_offset]
        self._primary_meter_unit_id = self._meter_address_offset

        self.meter_configured = False
        self.mppt_configured = False
        self.storage_configured = False
        self.entity_registry_cleanup_safe = True
        self.storage_extended_control_mode = 0
        self.max_charge_rate_w = 11000
        self.max_discharge_rate_w = 11000
        self._storage_address = STORAGE_ADDRESS
        self.mppt_module_count = 2
        self.mppt_model_length = 88
        self._sunspec_models_by_id = {}
        self._sunspec_model_headers = []
        self._grid_frequency = 50
        self._grid_frequency_lower_bound = self._grid_frequency - 0.2
        self._grid_frequency_upper_bound = self._grid_frequency + 0.2
        self._inverter_frequency_lower_bound = self._grid_frequency - 5
        self._inverter_frequency_upper_bound = self._grid_frequency + 5

        self._ac_limit_enable_mask_until = 0.0
        self._power_factor_enable_mask_until = 0.0
        self._load_inverter_sample_ts: float | None = None
        self._load_meter_sample_ts: dict[int, float] = {}
        self.state = FroniusRuntimeState()
        self.data = self.state.data
        self.reset_storage_info()

        self._runtime_service = FroniusModbusRuntimeService(self)
        self._read_service = FroniusModbusReadService(self)
        self._write_service = FroniusModbusWriteService(self)

    def reset_storage_info(self) -> None:
        self.storage_state.update(
            {
                "s_manufacturer": None,
                "s_model": "Battery Storage",
                "s_serial": None,
            }
        )

    def set_storage_info(
        self,
        manufacturer: str | None = None,
        model: str | None = None,
        serial: str | None = None,
    ) -> None:
        self.reset_storage_info()
        if manufacturer:
            self.storage_state.set("s_manufacturer", manufacturer)
        if model:
            self.storage_state.set("s_model", model)
        if serial:
            self.storage_state.set("s_serial", serial)

    def _meter_prefix(self, unit_id: int) -> str:
        return f"meter_{int(unit_id)}_"

    def _decode_reg(self, regs, start: int, data_type, length: int = 1):
        return self._client.convert_from_registers(
            regs[start : start + length],
            data_type=data_type,
        )

    def _decode_registers(self, regs, specs) -> dict[str, Any]:
        return {
            name: self._decode_reg(regs, start, data_type, length)
            for name, start, length, data_type in specs
        }

    def _set_calculated(
        self,
        key: str,
        raw_value,
        scale_factor,
        precision: int | None = None,
        minimum=None,
        maximum=None,
    ):
        value = self.calculate_value(raw_value, scale_factor, precision, minimum, maximum)
        self.data[key] = value
        return value

    def _set_mapped(self, key: str, mapping: dict, raw_value, field_name: str):
        value = self._map_value(mapping, raw_value, field_name)
        self.data[key] = value
        return value

    def calculate_value(
        self,
        value,
        scale_factor,
        digits: int = 2,
        lower_bound=None,
        upper_bound=None,
    ):
        return calculate_scaled_value(
            value,
            scale_factor,
            digits=digits,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

    def is_numeric(self, value: Any) -> bool:
        return is_numeric_value(value)

    def bitmask_to_string(
        self,
        bitmask,
        bitmask_list,
        default: str = "NA",
        max_length: int = 255,
        bits: int = 16,
    ) -> str:
        return bitmask_to_string(
            bitmask,
            bitmask_list,
            default=default,
            max_length=max_length,
            bits=bits,
        )

    def get_string_from_registers(self, regs) -> str | None:
        return strip_control_chars(
            self._client.convert_from_registers(
                regs,
                data_type=self._client.DATATYPE.STRING,
            )
        )

    def start_load_poll_cycle(self) -> None:
        self._load_inverter_sample_ts = None
        self._load_meter_sample_ts = {}

    def get_load_sample_timestamps(self, unit_id: int) -> tuple[float | None, float | None]:
        return (
            self._load_inverter_sample_ts,
            self._load_meter_sample_ts.get(int(unit_id)),
        )

    @property
    def primary_meter_unit_id(self) -> int:
        return self._primary_meter_unit_id

    @property
    def meter_unit_ids(self) -> tuple[int, ...]:
        return tuple(self._meter_unit_ids)

    def set_meter_unit_ids(
        self,
        unit_ids: list[int] | tuple[int, ...] | None,
        primary_unit_id: int | None = None,
    ) -> None:
        normalized: list[int] = []
        seen: set[int] = set()
        for unit_id in unit_ids or []:
            if not self.is_numeric(unit_id):
                continue
            normalized_unit_id = int(unit_id)
            if normalized_unit_id <= 0 or normalized_unit_id in seen:
                continue
            seen.add(normalized_unit_id)
            normalized.append(normalized_unit_id)

        self._meter_unit_ids = normalized
        if (
            primary_unit_id is not None
            and self.is_numeric(primary_unit_id)
            and int(primary_unit_id) in seen
        ):
            self._primary_meter_unit_id = int(primary_unit_id)
        elif normalized:
            self._primary_meter_unit_id = normalized[0]
        else:
            self._primary_meter_unit_id = self._meter_address_offset

    def _map_value(self, values: dict, key: int, field_name: str):
        value = values.get(key)
        if value is None:
            return f"Unknown ({key})"
        return value

    def _storage_register_address(self, offset: int) -> int:
        return self._storage_address + offset

    @property
    def runtime_service(self) -> FroniusModbusRuntimeService:
        return self._runtime_service

    @property
    def read_service(self) -> FroniusModbusReadService:
        return self._read_service

    @property
    def write_service(self) -> FroniusModbusWriteService:
        return self._write_service

    @property
    def inverter_state(self) -> StateSection:
        return self.state.inverter

    @property
    def storage_state(self) -> StateSection:
        return self.state.storage

    @property
    def web_api_state(self) -> StateSection:
        return self.state.web_api

    @property
    def mppt_state(self) -> StateSection:
        return self.state.mppt

    @property
    def derived_state(self) -> StateSection:
        return self.state.derived

    def meter_state(self, unit_id: int) -> StateSection:
        return self.state.meters.section(unit_id)

    def meter_value(self, unit_id: int, suffix: str, default: Any = None) -> Any:
        return self.state.meters.get_value(unit_id, suffix, default)

    def set_meter_value(self, unit_id: int, suffix: str, value: Any) -> None:
        self.state.meters.set_value(unit_id, suffix, value)

    def data_snapshot(self) -> dict[str, Any]:
        return self.state.snapshot()
