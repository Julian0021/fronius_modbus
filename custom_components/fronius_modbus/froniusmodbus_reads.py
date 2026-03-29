"""Read and parse helpers for the Fronius Modbus client."""
from __future__ import annotations

import logging
import time
from functools import wraps
from typing import Any

from .control_values import (
    ac_limit_raw_to_percent,
    ac_limit_raw_to_watts,
    coerce_scale_factor,
    power_factor_raw_to_value,
)
from .froniusmodbusclient_const import (
    AC_LIMIT_ENABLE_ADDRESS,
    AC_LIMIT_RATE_ADDRESS,
    AC_LIMIT_STATUS,
    CHARGE_GRID_STATUS,
    CHARGE_STATUS,
    COMMON_ADDRESS,
    CONNECTION_STATUS_CONDENSED,
    CONTROL_STATE_ENABLED,
    CONTROL_STATUS,
    ECP_CONNECTION_STATUS,
    FRONIUS_INVERTER_STATUS,
    GRID_STATUS,
    INVERTER_ADDRESS,
    INVERTER_CONTROLS,
    INVERTER_CONTROLS_ADDRESS,
    INVERTER_CONTROLS_COUNT,
    INVERTER_EVENTS,
    INVERTER_MODEL_SETTINGS_ADDRESS,
    INVERTER_MODEL_SETTINGS_COUNT,
    INVERTER_STATUS,
    INVERTER_STATUS_ADDRESS,
    INVERTER_STATUS_COUNT,
    METER_ADDRESS,
    MPPT_MODEL_MAX_LENGTH,
    MPPT_MODEL_MIN_LENGTH,
    MPPT_MODEL_MIN_REGISTERS,
    MPPT_MODULE_BLOCK_WORDS,
    MPPT_MODULE_CURRENT_OFFSET,
    MPPT_MODULE_LABEL_OFFSET,
    MPPT_MODULE_LABEL_WORDS,
    MPPT_MODULE_LIFETIME_ENERGY_OFFSET,
    MPPT_MODULE_POWER_OFFSET,
    MPPT_MODULE_TIMESTAMP_OFFSET,
    MPPT_MODULE_VOLTAGE_OFFSET,
    MPPT_UNAVAILABLE_U16,
    MPPT_UNAVAILABLE_U32,
    NAMEPLATE_ADDRESS,
    STORAGE_CONTROL_MODE,
    STORAGE_DER_TYPE,
    STORAGE_EXT_CONTROL_MODE,
    STORAGE_MODEL_LENGTH,
    SUNSPEC_END_MODEL_ID,
    SUNSPEC_FIRST_MODEL_HEADER_ADDRESS,
    SUNSPEC_ID_ADDRESS,
    SUNSPEC_ID_WORD_0,
    SUNSPEC_ID_WORD_1,
    SUNSPEC_MAX_MODEL_LENGTH,
    SUNSPEC_MODEL_HEADER_WORDS,
    SUNSPEC_MODEL_MAX_ADDRESS,
    SUNSPEC_MODEL_MIN_ADDRESS,
    SUNSPEC_MPPT_MODEL_ID,
    SUNSPEC_SCAN_MAX_MODELS,
    SUNSPEC_STORAGE_MODEL_ID,
)
from .integration_errors import FroniusError, FroniusReadError
from .storage_modes import derive_storage_extended_mode

_LOGGER = logging.getLogger(__name__)


def _safe_read(label: str):
    def decorator(func):
        @wraps(func)
        async def wrapper(self, *args, **kwargs):
            try:
                result = await func(self, *args, **kwargs)
            except FroniusError:
                raise
            except Exception as err:
                raise FroniusReadError(
                    f"Failed reading Fronius {label} data from "
                    f"{self._facade._host}:{self._facade._port}"
                ) from err
            if result is False:
                raise FroniusReadError(
                    f"Failed reading Fronius {label} data from "
                    f"{self._facade._host}:{self._facade._port}"
                )
            return result

        return wrapper

    return decorator


class FroniusModbusReadService:
    """Own Modbus read/parse flows and read-side normalization."""

    def __init__(self, facade) -> None:
        self._facade = facade

    def _get_ac_limit_rate_sf(self) -> int | None:
        value = self._facade.data.get("ac_limit_rate_sf")
        rate_sf = coerce_scale_factor(value)
        if rate_sf is None and self._facade.is_numeric(value):
            _LOGGER.error("Invalid AC limit scale factor: %s", value)
        return rate_sf

    def _get_power_factor_sf(self) -> int | None:
        value = self._facade.data.get("power_factor_sf")
        power_factor_sf = coerce_scale_factor(value)
        if power_factor_sf is None and self._facade.is_numeric(value):
            _LOGGER.error("Invalid power factor scale factor: %s", value)
        return power_factor_sf

    def _get_inverter_max_power_w(self) -> float | None:
        value = self._facade.data.get("max_power")
        if not self._facade.is_numeric(value):
            return None
        max_power_w = float(value)
        return max_power_w if max_power_w > 0 else None

    def _set_ac_limit_enable_state(self, enable_raw: int) -> None:
        self._facade.data["ac_limit_enable"] = AC_LIMIT_STATUS.get(enable_raw, "Unknown")

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
        rate_sf = self._get_ac_limit_rate_sf()
        self._facade.data["ac_limit_rate_raw"] = raw_value
        self._facade.data["ac_limit_rate_pct"] = ac_limit_raw_to_percent(
            raw_value,
            rate_sf,
        )
        self._facade.data["ac_limit_rate"] = ac_limit_raw_to_watts(
            raw_value,
            rate_sf,
            self._get_inverter_max_power_w(),
        )

    async def _read_enable_raw(self, address: int) -> int | None:
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=address,
            count=1,
        )

        enable_raw = self._facade._client.convert_from_registers(
            regs[0:1],
            data_type=self._facade._client.DATATYPE.UINT16,
        )
        if not self._facade.is_numeric(enable_raw):
            return None
        return int(enable_raw)

    def _update_storage_base_address(
        self,
        mppt_model_length: int,
        mppt_data_address: int | None = None,
    ) -> None:
        if not self._facade.is_numeric(mppt_model_length):
            return

        model_length = int(mppt_model_length)
        if model_length <= 0 or model_length > SUNSPEC_MAX_MODEL_LENGTH:
            return

        if not self._facade.is_numeric(mppt_data_address):
            return

        candidate = int(mppt_data_address) + model_length + SUNSPEC_MODEL_HEADER_WORDS
        if candidate < SUNSPEC_MODEL_MIN_ADDRESS or candidate > SUNSPEC_MODEL_MAX_ADDRESS:
            return

        self._facade.mppt_model_length = model_length
        self._facade._storage_address = candidate
        self._facade.data["storage_model_address"] = candidate

    def _get_sunspec_model(self, model_id: int):
        models = self._facade._sunspec_models_by_id.get(model_id)
        if not models:
            return None
        return models[0]

    async def _scan_sunspec_models(self, force: bool = False) -> bool:
        """Discover and cache the SunSpec model headers exposed by the inverter."""
        if self._facade._sunspec_model_headers and not force:
            return True

        sid_regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=SUNSPEC_ID_ADDRESS,
            count=2,
        )
        if len(sid_regs) != 2:
            return False
        if sid_regs[0] != SUNSPEC_ID_WORD_0 or sid_regs[1] != SUNSPEC_ID_WORD_1:
            _LOGGER.error("Invalid SunSpec SID at %s: %s", SUNSPEC_ID_ADDRESS, sid_regs)
            return False

        models_by_id = {}
        model_headers = []
        header_address = SUNSPEC_FIRST_MODEL_HEADER_ADDRESS

        for _ in range(SUNSPEC_SCAN_MAX_MODELS):
            header_regs = await self._facade.get_registers(
                unit_id=self._facade._inverter_unit_id,
                address=header_address,
                count=2,
            )
            if len(header_regs) != 2:
                return False

            model_id = int(header_regs[0])
            model_length = int(header_regs[1])
            if model_id == SUNSPEC_END_MODEL_ID:
                break
            if (
                model_id <= 0
                or model_length <= 0
                or model_length > SUNSPEC_MAX_MODEL_LENGTH
            ):
                _LOGGER.error(
                    "Invalid SunSpec model header at %s: id=%s length=%s",
                    header_address,
                    model_id,
                    model_length,
                )
                return False

            model_entry = {
                "id": model_id,
                "length": model_length,
                "id_address": header_address,
                "l_address": header_address + 1,
                "data_address": header_address + SUNSPEC_MODEL_HEADER_WORDS,
            }
            model_headers.append(model_entry)
            models_by_id.setdefault(model_id, []).append(model_entry)

            header_address = model_entry["data_address"] + model_length

        if not model_headers:
            return False

        self._facade._sunspec_models_by_id = models_by_id
        self._facade._sunspec_model_headers = model_headers
        self._facade.data["sunspec_model_count"] = len(model_headers)
        return True

    @_safe_read("device info")
    async def read_device_info_data(self, prefix, unit_id):
        regs = await self._facade.get_registers(
            unit_id=unit_id,
            address=COMMON_ADDRESS,
            count=65,
        )

        for field, start, end in (
            ("manufacturer", 0, 16),
            ("model", 16, 32),
            ("options", 32, 40),
            ("sw_version", 40, 48),
            ("serial", 48, 64),
        ):
            self._facade.data[prefix + field] = self._facade.get_string_from_registers(
                regs[start:end]
            )
        self._facade.data[prefix + "unit_id"] = self._facade._decode_reg(
            regs,
            64,
            self._facade._client.DATATYPE.UINT16,
        )

        return True

    @_safe_read("inverter")
    async def read_inverter_data(self):
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=INVERTER_ADDRESS,
            count=50,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("A", 0, 1, dt.UINT16),
                ("AphA", 1, 1, dt.UINT16),
                ("AphB", 2, 1, dt.UINT16),
                ("AphC", 3, 1, dt.UINT16),
                ("A_SF", 4, 1, dt.INT16),
                ("PPVphAB", 5, 1, dt.UINT16),
                ("PPVphBC", 6, 1, dt.UINT16),
                ("PPVphCA", 7, 1, dt.UINT16),
                ("PhVphA", 8, 1, dt.UINT16),
                ("PhVphB", 9, 1, dt.UINT16),
                ("PhVphC", 10, 1, dt.UINT16),
                ("V_SF", 11, 1, dt.INT16),
                ("W", 12, 1, dt.INT16),
                ("W_SF", 13, 1, dt.INT16),
                ("Hz", 14, 1, dt.INT16),
                ("Hz_SF", 15, 1, dt.INT16),
                ("VAr", 18, 1, dt.INT16),
                ("VAr_SF", 19, 1, dt.INT16),
                ("WH", 22, 2, dt.UINT32),
                ("WH_SF", 24, 1, dt.INT16),
                ("St", 36, 1, dt.UINT16),
                ("StVnd", 37, 1, dt.UINT16),
                ("EvtVnd2", 44, 2, dt.UINT32),
            ),
        )

        for key in ("A", "AphA", "AphB", "AphC"):
            self._facade._set_calculated(key, raw[key], raw["A_SF"], 3, 0, 1000)
        for key in ("PPVphAB", "PPVphBC", "PPVphCA", "PhVphA", "PhVphB", "PhVphC"):
            self._facade._set_calculated(key, raw[key], raw["V_SF"])
        self._facade._set_calculated("acpower", raw["W"], raw["W_SF"], 2, -50000, 50000)
        self._facade._set_calculated("var", raw["VAr"], raw["VAr_SF"], 2, -50000, 50000)
        self._facade._set_calculated("line_frequency", raw["Hz"], raw["Hz_SF"], 2, 0, 100)
        self._facade._set_calculated("acenergy", raw["WH"], raw["WH_SF"])
        self._facade._set_mapped("status", INVERTER_STATUS, raw["St"], "inverter status")
        self._facade._set_mapped(
            "statusvendor",
            FRONIUS_INVERTER_STATUS,
            raw["StVnd"],
            "inverter status",
        )
        self._facade.data["statusvendor_id"] = raw["StVnd"]
        self._facade.data["events2"] = self._facade.bitmask_to_string(
            raw["EvtVnd2"],
            INVERTER_EVENTS,
            default="None",
            bits=32,
        )
        self._facade._load_inverter_sample_ts = time.monotonic()

        return True

    @_safe_read("inverter nameplate")
    async def read_inverter_nameplate_data(self):
        """Read inverter nameplate data and derive storage ratings when present."""
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=NAMEPLATE_ADDRESS,
            count=120,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("DERTyp", 0, 1, dt.UINT16),
                ("WHRtg", 17, 1, dt.UINT16),
                ("WHRtg_SF", 18, 1, dt.INT16),
                ("MaxChaRte", 21, 1, dt.UINT16),
                ("MaxChaRte_SF", 22, 1, dt.INT16),
                ("MaxDisChaRte", 23, 1, dt.UINT16),
                ("MaxDisChaRte_SF", 24, 1, dt.INT16),
            ),
        )

        has_storage_ratings = any(
            self._facade.is_numeric(value) and 0 < value < 65535
            for value in [raw["WHRtg"], raw["MaxChaRte"], raw["MaxDisChaRte"]]
        )
        if raw["DERTyp"] == STORAGE_DER_TYPE or has_storage_ratings:
            self._facade.storage_configured = True
        self._facade.data["DERTyp"] = raw["DERTyp"]
        self._facade._set_calculated("WHRtg", raw["WHRtg"], raw["WHRtg_SF"], 0)
        self._facade._set_calculated("MaxChaRte", raw["MaxChaRte"], raw["MaxChaRte_SF"], 0)
        self._facade._set_calculated(
            "MaxDisChaRte",
            raw["MaxDisChaRte"],
            raw["MaxDisChaRte_SF"],
            0,
        )

        if self._facade.is_numeric(self._facade.data["MaxChaRte"]):
            self._facade.max_charge_rate_w = self._facade.data["MaxChaRte"]
        if self._facade.is_numeric(self._facade.data["MaxDisChaRte"]):
            self._facade.max_discharge_rate_w = self._facade.data["MaxDisChaRte"]

        return True

    @_safe_read("inverter status")
    async def read_inverter_status_data(self):
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=INVERTER_STATUS_ADDRESS,
            count=INVERTER_STATUS_COUNT,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("PVConn", 0, 1, dt.UINT16),
                ("StorConn", 1, 1, dt.UINT16),
                ("ECPConn", 2, 1, dt.UINT16),
                ("StActCtl", 33, 2, dt.UINT32),
                ("Ris", 42, 1, dt.UINT16),
                ("Ris_SF", 43, 1, dt.UINT16),
            ),
        )

        self._facade._set_mapped(
            "pv_connection",
            CONNECTION_STATUS_CONDENSED,
            raw["PVConn"],
            "pv connection",
        )
        self._facade._set_mapped(
            "storage_connection",
            CONNECTION_STATUS_CONDENSED,
            raw["StorConn"],
            "storage connection",
        )
        self._facade._set_mapped(
            "ecp_connection",
            ECP_CONNECTION_STATUS,
            raw["ECPConn"],
            "electrical connection",
        )
        self._facade.data["inverter_controls"] = self._facade.bitmask_to_string(
            raw["StActCtl"],
            INVERTER_CONTROLS,
            "Normal",
        )
        self._facade._set_calculated(
            "isolation_resistance",
            raw["Ris"],
            raw["Ris_SF"] - 6,
        )

        return True

    @_safe_read("inverter settings")
    async def read_inverter_model_settings_data(self):
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=INVERTER_MODEL_SETTINGS_ADDRESS,
            count=INVERTER_MODEL_SETTINGS_COUNT,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("WMax", 0, 1, dt.UINT16),
                ("VRef", 1, 1, dt.UINT16),
                ("VRefOfs", 2, 1, dt.UINT16),
                ("WMax_SF", 20, 1, dt.INT16),
                ("VRef_SF", 21, 1, dt.INT16),
            ),
        )

        self._facade._set_calculated("max_power", raw["WMax"], raw["WMax_SF"], 2, 0, 50000)
        self._facade._set_calculated("vref", raw["VRef"], raw["VRef_SF"])
        self._facade._set_calculated("vrefofs", raw["VRefOfs"], raw["VRef_SF"])

        return True

    @_safe_read("inverter controls")
    async def read_inverter_controls_data(self):
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=INVERTER_CONTROLS_ADDRESS,
            count=INVERTER_CONTROLS_COUNT,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("Conn", 2, 1, dt.UINT16),
                ("WMaxLim_Ena", 7, 1, dt.UINT16),
                ("OutPFSet", 8, 1, dt.INT16),
                ("OutPFSet_Ena", 12, 1, dt.UINT16),
                ("VArPct_Ena", 20, 1, dt.INT16),
                ("WMaxLimPct_SF", 21, 1, dt.INT16),
                ("OutPFSet_SF", 22, 1, dt.INT16),
            ),
        )

        self._facade._set_mapped("Conn", CONTROL_STATUS, raw["Conn"], "connection control")
        if time.monotonic() < self._facade._ac_limit_enable_mask_until:
            self._set_ac_limit_control_state(1)
        else:
            self._set_ac_limit_control_state(raw["WMaxLim_Ena"])
        if time.monotonic() < self._facade._power_factor_enable_mask_until:
            self._set_power_factor_enable_state(1)
        else:
            self._set_power_factor_enable_state(raw["OutPFSet_Ena"])
            if raw["OutPFSet_Ena"] == CONTROL_STATE_ENABLED:
                self._facade._power_factor_enable_mask_until = 0.0
        self._facade._set_mapped(
            "VArPct_Ena",
            CONTROL_STATUS,
            raw["VArPct_Ena"],
            "VAr control",
        )
        self._facade.data["ac_limit_rate_sf"] = raw["WMaxLimPct_SF"]
        self._facade.data["power_factor_sf"] = raw["OutPFSet_SF"]
        self._facade.data["power_factor"] = power_factor_raw_to_value(
            raw["OutPFSet"],
            self._get_power_factor_sf(),
        )

        return True

    def protect_lfte(self, key, value):
        """Keep lifetime energy counters monotonic so HA statistics stay valid."""
        if key not in self._facade.data:
            _LOGGER.info("Initializing %s=%s", key, value)
            return value
        if self._facade.data[key] is None:
            _LOGGER.info("Found initial %s=None. Now using new value %s", key, value)
            return value
        if value is None:
            _LOGGER.warning(
                "Received implausible %s=%s. Using previous plausible value %s",
                key,
                value,
                self._facade.data[key],
            )
            return self._facade.data[key]
        if value < self._facade.data[key]:
            _LOGGER.warning(
                "Received implausible (too small) %s=%s < previous plausible value %s",
                key,
                value,
                self._facade.data[key],
            )
            return self._facade.data[key]
        if value > self._facade.data[key] + 100000:
            _LOGGER.warning(
                "Received implausible (too large) %s=%s >> previous plausible value %s",
                key,
                value,
                self._facade.data[key],
            )
            return self._facade.data[key]
        return value

    def _sanitize_mppt_u16(self, value: int | None) -> int | None:
        if not self._facade.is_numeric(value):
            return None
        sanitized = int(value)
        if sanitized == MPPT_UNAVAILABLE_U16:
            return None
        return sanitized

    def _sanitize_mppt_u32(self, value: int | None) -> int | None:
        if not self._facade.is_numeric(value):
            return None
        sanitized = int(value)
        if sanitized == MPPT_UNAVAILABLE_U32:
            return None
        return sanitized

    def _set_storage_transfer_data(
        self,
        direction: str,
        module_id: int | None,
        module_current: dict[int, Any],
        module_voltage: dict[int, Any],
        module_power: dict[int, Any],
        module_lfte: dict[int, Any],
    ) -> None:
        self._facade.data[f"storage_{direction}_module"] = module_id
        self._facade.data[f"storage_{direction}_current"] = (
            module_current.get(module_id) if module_id else None
        )
        self._facade.data[f"storage_{direction}_voltage"] = (
            module_voltage.get(module_id) if module_id else None
        )
        self._facade.data[f"storage_{direction}_power"] = (
            module_power.get(module_id) if module_id else None
        )
        self._facade.data[f"storage_{direction}_lfte"] = (
            self.protect_lfte(
                f"storage_{direction}_lfte",
                module_lfte.get(module_id),
            )
            if module_id
            else None
        )

    @_safe_read("mppt")
    async def read_mppt_data(self):
        """Read MPPT model 160 data and derive storage topology."""
        if not await self._scan_sunspec_models():
            return False

        mppt_model = self._get_sunspec_model(SUNSPEC_MPPT_MODEL_ID)
        if mppt_model is None:
            return False

        mppt_model_length = int(mppt_model["length"])
        if (
            mppt_model_length < MPPT_MODEL_MIN_LENGTH
            or mppt_model_length > MPPT_MODEL_MAX_LENGTH
        ):
            return False

        mppt_read_address = int(mppt_model["l_address"])
        self._update_storage_base_address(
            mppt_model_length,
            mppt_data_address=mppt_model["data_address"],
        )
        self._facade.data["mppt_model_length"] = mppt_model_length
        self._facade.data["mppt_register_address"] = mppt_read_address
        self._facade.data["mppt_model_id_address"] = int(mppt_model["id_address"])

        storage_model = self._get_sunspec_model(SUNSPEC_STORAGE_MODEL_ID)
        if storage_model is not None:
            storage_model_id = int(storage_model["id"])
            storage_model_length = int(storage_model["length"])
            self._facade.data["storage_model_id"] = storage_model_id
            self._facade.data["storage_model_length"] = storage_model_length
            self._facade.data["storage_model_address"] = int(storage_model["data_address"])
            if storage_model_length == STORAGE_MODEL_LENGTH:
                self._facade._storage_address = int(storage_model["data_address"])
                self._facade.storage_configured = True

        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=mppt_read_address,
            count=mppt_model_length,
        )

        model_limit = len(regs)
        if model_limit < MPPT_MODEL_MIN_REGISTERS:
            return False

        dt = self._facade._client.DATATYPE
        dca_sf = self._facade._client.convert_from_registers(
            regs[1:2],
            data_type=dt.INT16,
        )
        dcv_sf = self._facade._client.convert_from_registers(
            regs[2:3],
            data_type=dt.INT16,
        )
        dcw_sf = self._facade._client.convert_from_registers(
            regs[3:4],
            data_type=dt.INT16,
        )
        dcwh_sf = self._facade._client.convert_from_registers(
            regs[4:5],
            data_type=dt.INT16,
        )
        reported_module_count = self._facade._client.convert_from_registers(
            regs[6:7],
            data_type=dt.UINT16,
        )
        if (
            not self._facade.is_numeric(reported_module_count)
            or int(reported_module_count) <= 0
        ):
            return False

        max_modules_by_length = (
            model_limit - SUNSPEC_MODEL_HEADER_WORDS
        ) // MPPT_MODULE_BLOCK_WORDS
        module_count = min(int(reported_module_count), int(max_modules_by_length))
        if module_count <= 0:
            return False

        self._facade.mppt_module_count = module_count
        self._facade.data["mppt_module_count"] = module_count

        def read_u16(index: int):
            if index + 1 > model_limit:
                return None
            return self._facade._client.convert_from_registers(
                regs[index : index + 1],
                data_type=dt.UINT16,
            )

        def read_u32(index: int):
            if index + 2 > model_limit:
                return None
            return self._facade._client.convert_from_registers(
                regs[index : index + 2],
                data_type=dt.UINT32,
            )

        module_power = {}
        module_lfte = {}
        module_tms = {}
        module_labels = {}
        module_current = {}
        module_voltage = {}

        for module_id in range(1, module_count + 1):
            module_offset = MPPT_MODULE_BLOCK_WORDS * (module_id - 1)
            label_idx = module_offset + MPPT_MODULE_LABEL_OFFSET
            current_idx = module_offset + MPPT_MODULE_CURRENT_OFFSET
            voltage_idx = module_offset + MPPT_MODULE_VOLTAGE_OFFSET
            power_idx = module_offset + MPPT_MODULE_POWER_OFFSET
            lfte_idx = module_offset + MPPT_MODULE_LIFETIME_ENERGY_OFFSET
            tms_idx = module_offset + MPPT_MODULE_TIMESTAMP_OFFSET

            label = None
            if label_idx + MPPT_MODULE_LABEL_WORDS <= model_limit:
                try:
                    label = self._facade.get_string_from_registers(
                        regs[label_idx : label_idx + MPPT_MODULE_LABEL_WORDS]
                    )
                except (UnicodeDecodeError, ValueError):
                    label = None
            module_labels[module_id] = label

            raw_current = self._sanitize_mppt_u16(read_u16(current_idx))
            raw_voltage = self._sanitize_mppt_u16(read_u16(voltage_idx))
            raw_power = self._sanitize_mppt_u16(read_u16(power_idx))
            raw_lfte = self._sanitize_mppt_u32(read_u32(lfte_idx))
            raw_tms = self._sanitize_mppt_u32(read_u32(tms_idx))

            module_current[module_id] = (
                self._facade.calculate_value(raw_current, dca_sf, 2, 0, 100)
                if raw_current is not None
                else None
            )
            module_voltage[module_id] = (
                self._facade.calculate_value(raw_voltage, dcv_sf, 2, 0, 1500)
                if raw_voltage is not None
                else None
            )
            module_power[module_id] = (
                self._facade.calculate_value(raw_power, dcw_sf, 2, 0, 15000)
                if raw_power is not None
                else None
            )
            if raw_lfte is None or (
                raw_lfte == 0
                and raw_current is None
                and raw_voltage is None
                and raw_power is None
            ):
                module_lfte[module_id] = None
            else:
                module_lfte[module_id] = self._facade.calculate_value(raw_lfte, dcwh_sf)
            module_tms[module_id] = raw_tms

            self._facade.data[f"module{module_id}_label"] = label
            self._facade.data[f"module{module_id}_power"] = module_power[module_id]
            self._facade.data[f"module{module_id}_lfte"] = module_lfte[module_id]
            self._facade.data[f"module{module_id}_tms"] = module_tms[module_id]

            module_idx = module_id - 1
            self._facade.data[f"mppt_module_{module_idx}_label"] = label
            self._facade.data[f"mppt_module_{module_idx}_dc_current"] = module_current[
                module_id
            ]
            self._facade.data[f"mppt_module_{module_idx}_dc_voltage"] = module_voltage[
                module_id
            ]
            self._facade.data[f"mppt_module_{module_idx}_dc_power"] = module_power[
                module_id
            ]
            self._facade.data[
                f"mppt_module_{module_idx}_lifetime_energy"
            ] = self.protect_lfte(
                f"mppt_module_{module_idx}_lifetime_energy",
                module_lfte[module_id],
            )
            self._facade.data[f"mppt_module_{module_idx}_timestamp"] = module_tms[
                module_id
            ]

        storage_charge_module = None
        storage_discharge_module = None
        for module_id, label in module_labels.items():
            if not isinstance(label, str):
                continue
            normalized = label.replace(" ", "").upper()
            if "STDISCHA" in normalized:
                storage_discharge_module = module_id
            elif normalized.startswith("STCHA"):
                storage_charge_module = module_id

        if (
            self._facade.storage_configured
            and not (storage_charge_module and storage_discharge_module)
            and module_count >= 4
        ):
            storage_charge_module = module_count - 1
            storage_discharge_module = module_count

        if (
            self._facade.storage_configured
            and storage_charge_module
            and storage_discharge_module
        ):
            self._set_storage_transfer_data(
                "charge",
                storage_charge_module,
                module_current,
                module_voltage,
                module_power,
                module_lfte,
            )
            self._set_storage_transfer_data(
                "discharge",
                storage_discharge_module,
                module_current,
                module_voltage,
                module_power,
                module_lfte,
            )
        else:
            self._set_storage_transfer_data(
                "charge",
                None,
                module_current,
                module_voltage,
                module_power,
                module_lfte,
            )
            self._set_storage_transfer_data(
                "discharge",
                None,
                module_current,
                module_voltage,
                module_power,
                module_lfte,
            )

        pv_modules = []
        for module_id, label in module_labels.items():
            if isinstance(label, str) and "MPPT" in label.upper():
                pv_modules.append(module_id)
        if not pv_modules:
            if storage_charge_module and storage_discharge_module:
                pv_modules = [
                    module_id
                    for module_id in range(1, module_count + 1)
                    if module_id not in [storage_charge_module, storage_discharge_module]
                ]
            else:
                pv_modules = list(range(1, module_count + 1))

        self._facade.data["mppt_visible_module_ids"] = pv_modules
        pv_values = [
            module_power.get(module_id)
            for module_id in pv_modules
            if self._facade.is_numeric(module_power.get(module_id))
        ]
        self._facade.data["pv_power"] = round(sum(pv_values), 2) if pv_values else None
        _LOGGER.debug(
            "Parsed model 160 MPPT data: address=%s length=%s reported_count=%s "
            "visible_modules=%s storage_charge_module=%s "
            "storage_discharge_module=%s labels=%s",
            mppt_read_address,
            mppt_model_length,
            reported_module_count,
            pv_modules,
            storage_charge_module,
            storage_discharge_module,
            module_labels,
        )

        return True

    @_safe_read("storage")
    async def read_inverter_storage_data(self):
        """Read storage control data from the inverter storage model."""
        regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=self._facade._storage_address,
            count=STORAGE_MODEL_LENGTH,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("max_charge", 0, 1, dt.UINT16),
                ("WChaGra", 1, 1, dt.UINT16),
                ("WDisChaGra", 2, 1, dt.UINT16),
                ("storage_control_mode", 3, 1, dt.UINT16),
                ("minimum_reserve", 5, 1, dt.UINT16),
                ("charge_state", 6, 1, dt.UINT16),
                ("charge_status", 9, 1, dt.UINT16),
                ("discharge_power", 10, 1, dt.INT16),
                ("charge_power", 11, 1, dt.INT16),
                ("charge_grid_set", 15, 1, dt.UINT16),
            ),
        )

        if self._facade.is_numeric(raw["max_charge"]) and raw["max_charge"] > 0:
            self._facade.storage_configured = True

        soc_minimum_value = self._facade.calculate_value(
            raw["minimum_reserve"],
            -2,
            2,
            0,
            100,
        )
        if (
            self._facade.is_numeric(soc_minimum_value)
            and float(soc_minimum_value).is_integer()
        ):
            soc_minimum_value = int(soc_minimum_value)

        self._facade._set_mapped(
            "grid_charging",
            CHARGE_GRID_STATUS,
            raw["charge_grid_set"],
            "grid charging",
        )
        self._facade._set_mapped(
            "charge_status",
            CHARGE_STATUS,
            raw["charge_status"],
            "charge status",
        )
        self._facade.data["soc_minimum"] = soc_minimum_value
        self._facade._set_calculated(
            "discharging_power",
            raw["discharge_power"],
            -2,
            2,
            -100,
            100,
        )
        self._facade._set_calculated(
            "charging_power",
            raw["charge_power"],
            -2,
            2,
            -100,
            100,
        )
        self._facade._set_calculated("soc", raw["charge_state"], -2, 2, 0, 100)
        self._facade._set_calculated("max_charge", raw["max_charge"], 0, 0)
        self._facade._set_calculated("WChaGra", raw["WChaGra"], 0, 0)
        self._facade._set_calculated("WDisChaGra", raw["WDisChaGra"], 0, 0)

        mapped_control_mode = self._facade._map_value(
            STORAGE_CONTROL_MODE,
            raw["storage_control_mode"],
            "storage control mode",
        )
        if raw["discharge_power"] >= 0:
            self._facade.data["discharge_limit"] = raw["discharge_power"] / 100.0
            self._facade.data["grid_charge_power"] = 0
        else:
            self._facade.data["grid_charge_power"] = (raw["discharge_power"] * -1) / 100.0
            self._facade.data["discharge_limit"] = 0
        if raw["charge_power"] >= 0:
            self._facade.data["charge_limit"] = raw["charge_power"] / 100
            self._facade.data["grid_discharge_power"] = 0
        else:
            self._facade.data["grid_discharge_power"] = (raw["charge_power"] * -1) / 100.0
            self._facade.data["charge_limit"] = 0

        self._facade.data["control_mode"] = mapped_control_mode

        ext_control_mode = derive_storage_extended_mode(
            raw["storage_control_mode"],
            charge_power=raw["charge_power"],
            discharge_power=raw["discharge_power"],
        )

        if ext_control_mode is not None:
            self._facade.data["ext_control_mode"] = self._facade._map_value(
                STORAGE_EXT_CONTROL_MODE,
                int(ext_control_mode),
                "extended storage mode",
            )
            self._facade.storage_extended_control_mode = int(ext_control_mode)

        return True

    @_safe_read("meter")
    async def read_meter_data(self, unit_id, is_primary=False):
        """Read one meter model and update meter values and grid health."""
        meter_prefix = self._facade._meter_prefix(unit_id)
        regs = await self._facade.get_registers(
            unit_id=unit_id,
            address=METER_ADDRESS,
            count=103,
        )

        dt = self._facade._client.DATATYPE
        raw = self._facade._decode_registers(
            regs,
            (
                ("A", 0, 1, dt.INT16),
                ("AphA", 1, 1, dt.INT16),
                ("AphB", 2, 1, dt.INT16),
                ("AphC", 3, 1, dt.INT16),
                ("A_SF", 4, 1, dt.INT16),
                ("PhVphA", 6, 1, dt.INT16),
                ("PhVphB", 7, 1, dt.INT16),
                ("PhVphC", 8, 1, dt.INT16),
                ("PPV", 9, 1, dt.INT16),
                ("V_SF", 13, 1, dt.INT16),
                ("Hz", 14, 1, dt.INT16),
                ("Hz_SF", 15, 1, dt.INT16),
                ("W", 16, 1, dt.INT16),
                ("WphA", 17, 1, dt.INT16),
                ("WphB", 18, 1, dt.INT16),
                ("WphC", 19, 1, dt.INT16),
                ("W_SF", 20, 1, dt.INT16),
                ("TotWhExp", 36, 2, dt.UINT32),
                ("TotWhImp", 44, 2, dt.UINT32),
                ("TotWh_SF", 52, 1, dt.INT16),
            ),
        )

        acpower = self._facade.calculate_value(raw["W"], raw["W_SF"], 2, -50000, 50000)
        m_frequency = self._facade.calculate_value(raw["Hz"], raw["Hz_SF"], 2, 0, 100)

        for key in ("A", "AphA", "AphB", "AphC"):
            self._facade._set_calculated(
                meter_prefix + key,
                raw[key],
                raw["A_SF"],
                3,
                -1000,
                1000,
            )
        for key in ("PhVphA", "PhVphB", "PhVphC", "PPV"):
            self._facade._set_calculated(
                meter_prefix + key,
                raw[key],
                raw["V_SF"],
                1,
                0,
                1000,
            )
        for key in ("WphA", "WphB", "WphC"):
            self._facade._set_calculated(
                meter_prefix + key,
                raw[key],
                raw["W_SF"],
                2,
                -50000,
                50000,
            )
        self._facade.data[meter_prefix + "exported"] = self.protect_lfte(
            meter_prefix + "exported",
            self._facade.calculate_value(raw["TotWhExp"], raw["TotWh_SF"]),
        )
        self._facade.data[meter_prefix + "imported"] = self.protect_lfte(
            meter_prefix + "imported",
            self._facade.calculate_value(raw["TotWhImp"], raw["TotWh_SF"]),
        )
        self._facade.data[meter_prefix + "line_frequency"] = m_frequency
        self._facade.data[meter_prefix + "power"] = acpower
        self._facade._load_meter_sample_ts[int(unit_id)] = time.monotonic()

        if is_primary:
            status_str = None
            i_frequency = self._facade.data["line_frequency"]
            if (
                i_frequency is not None
                and self._facade.is_numeric(i_frequency)
                and m_frequency is not None
                and self._facade.is_numeric(m_frequency)
            ):
                m_online = False
                if (
                    m_frequency
                    and m_frequency > self._facade._grid_frequency_lower_bound
                    and m_frequency < self._facade._grid_frequency_upper_bound
                ):
                    m_online = True

                if (
                    m_online
                    and i_frequency > self._facade._grid_frequency_lower_bound
                    and i_frequency < self._facade._grid_frequency_upper_bound
                ):
                    status_str = GRID_STATUS.get(3)
                elif (
                    not m_online
                    and i_frequency > self._facade._inverter_frequency_lower_bound
                    and i_frequency < self._facade._inverter_frequency_upper_bound
                ):
                    status_str = GRID_STATUS.get(1)
                elif i_frequency < 1:
                    if m_online:
                        status_str = GRID_STATUS.get(2)
                    elif m_frequency < 1:
                        status_str = GRID_STATUS.get(0)
            if status_str is None:
                _LOGGER.error(
                    "Could not establish grid connection status m=%s i=%s",
                    m_frequency,
                    i_frequency,
                )
                self._facade.data["grid_status"] = None
            else:
                self._facade.data["grid_status"] = status_str

        return True

    @_safe_read("ac limit")
    async def read_ac_limit_data(self):
        """Read AC limit control registers."""
        rate_regs = await self._facade.get_registers(
            unit_id=self._facade._inverter_unit_id,
            address=AC_LIMIT_RATE_ADDRESS,
            count=1,
        )
        ac_limit_rate_raw = self._facade._decode_reg(
            rate_regs,
            0,
            self._facade._client.DATATYPE.UINT16,
        )
        self._set_ac_limit_rate_values(ac_limit_rate_raw)

        if time.monotonic() < self._facade._ac_limit_enable_mask_until:
            self._facade.data["ac_limit_enable"] = AC_LIMIT_STATUS.get(
                CONTROL_STATE_ENABLED,
                "Enabled",
            )
            return True

        ac_limit_enable_raw = await self._read_enable_raw(AC_LIMIT_ENABLE_ADDRESS)
        if ac_limit_enable_raw is not None:
            self._set_ac_limit_enable_state(ac_limit_enable_raw)
            if ac_limit_enable_raw == CONTROL_STATE_ENABLED:
                self._facade._ac_limit_enable_mask_until = 0.0
        elif "ac_limit_enable" not in self._facade.data:
            self._facade.data["ac_limit_enable"] = None

        return True
