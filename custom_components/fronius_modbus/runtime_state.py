"""Structured runtime state with flat dict compatibility for legacy callers."""
from __future__ import annotations

import re
from typing import Any

_METER_KEY_RE = re.compile(r"^meter_(\d+)_(.+)$")
_MODULE_KEY_RE = re.compile(r"^module\d+_")

_INVERTER_KEYS = {
    "A",
    "AphA",
    "AphB",
    "AphC",
    "PPVphAB",
    "PPVphBC",
    "PPVphCA",
    "PhVphA",
    "PhVphB",
    "PhVphC",
    "acpower",
    "var",
    "acenergy",
    "line_frequency",
    "status",
    "statusvendor",
    "statusvendor_id",
    "events2",
    "pv_connection",
    "storage_connection",
    "ecp_connection",
    "inverter_controls",
    "isolation_resistance",
    "max_power",
    "vref",
    "vrefofs",
    "Conn",
    "WMaxLim_Ena",
    "OutPFSet_Ena",
    "VArPct_Ena",
    "ac_limit_rate_sf",
    "power_factor_sf",
    "power_factor",
    "power_factor_enable",
    "ac_limit_rate_raw",
    "ac_limit_rate_pct",
    "ac_limit_rate",
    "ac_limit_enable",
}

_STORAGE_KEYS = {
    "DERTyp",
    "WHRtg",
    "MaxChaRte",
    "MaxDisChaRte",
    "soc_minimum",
    "soc",
    "charge_status",
    "grid_charging",
    "control_mode",
    "ext_control_mode",
    "charge_limit",
    "discharge_limit",
    "grid_charge_power",
    "grid_discharge_power",
    "charging_power",
    "discharging_power",
    "storage_temperature",
    "storage_model_address",
    "storage_model_id",
    "storage_model_length",
    "storage_charge_module",
    "storage_charge_current",
    "storage_charge_voltage",
    "storage_charge_power",
    "storage_charge_lfte",
    "storage_discharge_module",
    "storage_discharge_current",
    "storage_discharge_voltage",
    "storage_discharge_power",
    "storage_discharge_lfte",
}

_WEB_API_KEYS = {
    "inverter_temperature",
    "soc_maximum",
}

_MPPT_KEYS = {
    "pv_power",
    "sunspec_model_count",
    "mppt_visible_module_ids",
    "mppt_module_count",
    "mppt_model_length",
    "mppt_register_address",
    "mppt_model_id_address",
}

_DERIVED_KEYS = {
    "load",
    "grid_status",
}


class StateSection:
    """Named section of runtime state."""

    def __init__(self, owner: FroniusRuntimeState, name: str) -> None:
        self._owner = owner
        self._name = name
        self._values: dict[str, Any] = {}

    @property
    def name(self) -> str:
        return self._name

    def contains(self, key: str) -> bool:
        return key in self._values

    def get(self, key: str, default: Any = None) -> Any:
        return self._values.get(key, default)

    def set(self, key: str, value: Any) -> None:
        self._values[key] = value
        self._owner._publish_set(key, value)

    def update(self, values: dict[str, Any] | None = None, **kwargs: Any) -> None:
        payload = {}
        if values:
            payload.update(values)
        if kwargs:
            payload.update(kwargs)
        for key, value in payload.items():
            self.set(key, value)

    def pop(self, key: str, default: Any = None) -> Any:
        if key not in self._values:
            return default
        value = self._values.pop(key)
        self._owner._publish_delete(key)
        return value

    def clear(self) -> None:
        for key in list(self._values):
            self.pop(key)

    def snapshot(self) -> dict[str, Any]:
        return dict(self._values)


class MeterStateRegistry:
    """Per-meter runtime state sections keyed by Modbus unit id."""

    def __init__(self, owner: FroniusRuntimeState) -> None:
        self._owner = owner
        self._sections: dict[int, StateSection] = {}

    @staticmethod
    def _flat_key(unit_id: int, suffix: str) -> str:
        return f"meter_{int(unit_id)}_{suffix}"

    def section(self, unit_id: int) -> StateSection:
        normalized_unit_id = int(unit_id)
        section = self._sections.get(normalized_unit_id)
        if section is None:
            section = StateSection(self._owner, f"meter_{normalized_unit_id}")
            self._sections[normalized_unit_id] = section
        return section

    def contains_flat_key(self, key: str) -> bool:
        match = _METER_KEY_RE.match(key)
        if match is None:
            return False
        unit_id = int(match.group(1))
        section = self._sections.get(unit_id)
        return section is not None and section.contains(key)

    def get_flat_key(self, key: str, default: Any = None) -> Any:
        match = _METER_KEY_RE.match(key)
        if match is None:
            return default
        unit_id = int(match.group(1))
        return self.section(unit_id).get(key, default)

    def set_flat_key(self, key: str, value: Any) -> None:
        match = _METER_KEY_RE.match(key)
        if match is None:
            raise KeyError(key)
        unit_id = int(match.group(1))
        self.section(unit_id).set(key, value)

    def delete_flat_key(self, key: str) -> None:
        match = _METER_KEY_RE.match(key)
        if match is None:
            raise KeyError(key)
        unit_id = int(match.group(1))
        section = self.section(unit_id)
        section.pop(key)
        if not section.snapshot():
            self._sections.pop(unit_id, None)

    def get_value(self, unit_id: int, suffix: str, default: Any = None) -> Any:
        return self.section(unit_id).get(self._flat_key(unit_id, suffix), default)

    def set_value(self, unit_id: int, suffix: str, value: Any) -> None:
        self.section(unit_id).set(self._flat_key(unit_id, suffix), value)

    def snapshot(self) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        for section in self._sections.values():
            payload.update(section.snapshot())
        return payload

    def clear(self) -> None:
        for unit_id in list(self._sections):
            self._sections[unit_id].clear()
            self._sections.pop(unit_id, None)


class RuntimeStateDict(dict):
    """Mutable dict view kept in sync with the structured runtime state."""

    def __init__(self, state: FroniusRuntimeState) -> None:
        self._state = state
        self._syncing = False
        super().__init__(state.snapshot())

    def _apply_sync_set(self, key: str, value: Any) -> None:
        self._syncing = True
        try:
            super().__setitem__(key, value)
        finally:
            self._syncing = False

    def _apply_sync_delete(self, key: str) -> None:
        self._syncing = True
        try:
            if key in self:
                super().__delitem__(key)
        finally:
            self._syncing = False

    def refresh_from_state(self) -> None:
        self._syncing = True
        try:
            super().clear()
            super().update(self._state.snapshot())
        finally:
            self._syncing = False

    def __setitem__(self, key: str, value: Any) -> None:
        if self._syncing:
            super().__setitem__(key, value)
            return
        self._state.set_flat(key, value)

    def __delitem__(self, key: str) -> None:
        if self._syncing:
            super().__delitem__(key)
            return
        self._state.del_flat(key)

    def clear(self) -> None:
        if self._syncing:
            super().clear()
            return
        self._state.clear()

    def pop(self, key: str, default: Any = None) -> Any:
        if self._syncing:
            return super().pop(key, default)
        if key not in self:
            return default
        value = self[key]
        self._state.del_flat(key)
        return value

    def update(self, *args: Any, **kwargs: Any) -> None:
        if self._syncing:
            super().update(*args, **kwargs)
            return

        for mapping in args:
            for key, value in dict(mapping).items():
                self._state.set_flat(key, value)
        for key, value in kwargs.items():
            self._state.set_flat(key, value)


class FroniusRuntimeState:
    """Structured runtime state for inverter, storage, meters, and Web API data."""

    def __init__(self) -> None:
        self.inverter = StateSection(self, "inverter")
        self.storage = StateSection(self, "storage")
        self.web_api = StateSection(self, "web_api")
        self.mppt = StateSection(self, "mppt")
        self.derived = StateSection(self, "derived")
        self.meters = MeterStateRegistry(self)
        self._data_view = RuntimeStateDict(self)

    @property
    def data(self) -> RuntimeStateDict:
        return self._data_view

    def snapshot(self) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        payload.update(self.inverter.snapshot())
        payload.update(self.storage.snapshot())
        payload.update(self.web_api.snapshot())
        payload.update(self.mppt.snapshot())
        payload.update(self.derived.snapshot())
        payload.update(self.meters.snapshot())
        return payload

    def clear(self) -> None:
        self.inverter.clear()
        self.storage.clear()
        self.web_api.clear()
        self.mppt.clear()
        self.derived.clear()
        self.meters.clear()
        self._data_view.refresh_from_state()

    def get_flat(self, key: str, default: Any = None) -> Any:
        if self.meters.contains_flat_key(key):
            return self.meters.get_flat_key(key, default)

        section = self._find_existing_section(key)
        if section is None:
            section = self._target_section_for_new_key(key)
        return section.get(key, default)

    def set_flat(self, key: str, value: Any) -> None:
        if _METER_KEY_RE.match(key):
            self.meters.set_flat_key(key, value)
            return

        section = self._find_existing_section(key)
        if section is None:
            section = self._target_section_for_new_key(key)
        section.set(key, value)

    def del_flat(self, key: str) -> None:
        if _METER_KEY_RE.match(key):
            self.meters.delete_flat_key(key)
            return

        section = self._find_existing_section(key)
        if section is None:
            raise KeyError(key)
        section.pop(key)

    def _find_existing_section(self, key: str) -> StateSection | None:
        for section in (
            self.inverter,
            self.storage,
            self.web_api,
            self.mppt,
            self.derived,
        ):
            if section.contains(key):
                return section
        return None

    def _target_section_for_new_key(self, key: str) -> StateSection:
        if key.startswith("api_") or key in _WEB_API_KEYS:
            return self.web_api
        if key.startswith("i_") or key in _INVERTER_KEYS:
            return self.inverter
        if key.startswith("s_") or key.startswith("storage_") or key in _STORAGE_KEYS:
            return self.storage
        if key.startswith("mppt_") or _MODULE_KEY_RE.match(key) or key in _MPPT_KEYS:
            return self.mppt
        if key in _DERIVED_KEYS:
            return self.derived
        return self.derived

    def _publish_set(self, key: str, value: Any) -> None:
        self._data_view._apply_sync_set(key, value)

    def _publish_delete(self, key: str) -> None:
        self._data_view._apply_sync_delete(key)


__all__ = [
    "FroniusRuntimeState",
    "MeterStateRegistry",
    "RuntimeStateDict",
    "StateSection",
]
