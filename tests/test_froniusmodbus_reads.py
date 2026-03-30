from __future__ import annotations

from types import SimpleNamespace

import pytest

import custom_components.fronius_modbus.froniusmodbus_reads as reads_module
from custom_components.fronius_modbus.froniusmodbus_reads import FroniusModbusReadService
from custom_components.fronius_modbus.storage_modes import StorageExtendedControlMode


class _StorageReadFacade:
    def __init__(self, raw: dict[str, int]) -> None:
        self._host = "fixture-host"
        self._port = 502
        self._inverter_unit_id = 1
        self._storage_address = 40345
        self._client = SimpleNamespace(
            DATATYPE=SimpleNamespace(UINT16="uint16", INT16="int16")
        )
        self._raw = raw
        self.data: dict[str, object] = {}
        self.storage_configured = False
        self.storage_extended_control_mode = 0

    async def get_registers(self, *, unit_id, address, count, retries=1):
        del retries
        assert unit_id == self._inverter_unit_id
        assert address == self._storage_address
        assert count == reads_module.STORAGE_MODEL_LENGTH
        return [0] * count

    def _decode_registers(self, regs, specs):
        del regs, specs
        return dict(self._raw)

    def is_numeric(self, value) -> bool:
        return isinstance(value, (int, float))

    def calculate_value(
        self,
        value,
        scale_factor,
        digits: int = 2,
        lower_bound=None,
        upper_bound=None,
    ):
        del digits
        if not self.is_numeric(value):
            return None
        result = value / 100 if scale_factor == -2 else value
        if lower_bound is not None:
            result = max(lower_bound, result)
        if upper_bound is not None:
            result = min(upper_bound, result)
        return result

    def _set_mapped(self, key: str, mapping: dict, raw_value, field_name: str):
        del field_name
        self.data[key] = mapping.get(raw_value, f"Unknown ({raw_value})")
        return self.data[key]

    def _set_calculated(
        self,
        key: str,
        raw_value,
        scale_factor,
        precision: int | None = None,
        minimum=None,
        maximum=None,
    ):
        del precision
        value = self.calculate_value(
            raw_value,
            scale_factor,
            lower_bound=minimum,
            upper_bound=maximum,
        )
        self.data[key] = value
        return value

    def _map_value(self, values: dict, key: int, field_name: str):
        del field_name
        return values.get(key, f"Unknown ({key})")


@pytest.mark.asyncio
async def test_read_inverter_storage_data_passes_charge_grid_flag_to_mode_derivation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    captured_kwargs: dict[str, object] = {}

    def _fake_derive_mode(storage_control_mode: int, **kwargs):
        captured_kwargs["storage_control_mode"] = storage_control_mode
        captured_kwargs.update(kwargs)
        return StorageExtendedControlMode.CHARGE_FROM_GRID

    monkeypatch.setattr(reads_module, "derive_storage_extended_mode", _fake_derive_mode)

    service = FroniusModbusReadService(
        _StorageReadFacade(
            {
                "max_charge": 1,
                "WChaGra": 100,
                "WDisChaGra": 0,
                "storage_control_mode": 2,
                "minimum_reserve": 500,
                "charge_state": 7000,
                "charge_status": 4,
                "discharge_power": 0,
                "charge_power": 100,
                "charge_grid_set": 1,
            }
        )
    )

    assert await service.read_inverter_storage_data() is True
    assert captured_kwargs == {
        "storage_control_mode": 2,
        "charge_power": 100,
        "discharge_power": 0,
        "charge_grid_enabled": True,
    }
