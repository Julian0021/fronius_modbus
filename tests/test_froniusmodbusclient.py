from __future__ import annotations

import pytest

from custom_components.fronius_modbus.froniusmodbusclient import FroniusModbusClient


class SnapshotModbusClient(FroniusModbusClient):
    def __init__(self, register_snapshot: dict[str, object]) -> None:
        super().__init__(
            host="fixture-host",
            port=502,
            inverter_unit_id=1,
            meter_unit_ids=[200],
            timeout=1,
        )
        self._snapshot = {
            tuple(int(part) for part in key.split(":")): value
            for key, value in register_snapshot["register_reads"].items()
        }

    async def connect(self, retries=3):
        return True

    async def get_registers(self, unit_id, address, count, retries=0):
        key = (int(unit_id), int(address), int(count))
        if key not in self._snapshot:
            raise AssertionError(f"Missing live snapshot registers for {key}")
        return list(self._snapshot[key])

    def close(self) -> None:
        return None


@pytest.mark.asyncio
async def test_init_data_discovers_live_device_layout(live_modbus_registers) -> None:
    client = SnapshotModbusClient(live_modbus_registers)

    assert await client.runtime_service.init_data() is True

    assert client.mppt_configured is True
    assert client.storage_configured is True
    assert client.meter_configured is True
    assert client.primary_meter_unit_id == 200
    assert client._meter_unit_ids == [200]

    assert client.data["i_manufacturer"] == "Fronius"
    assert client.data["i_model"] == "Verto 17.5 Plus"
    assert client.data["i_serial"] == "INV-SERIAL-0001"
    assert client.data["i_sw_version"] == "1.39.5-1"
    assert client.data["meter_200_model"] == "Smart Meter TS 65A-3"
    assert client.data["WHRtg"] == 29696
    assert client.data["MaxChaRte"] == 25600
    assert client.data["MaxDisChaRte"] == 25600
    assert client.data["storage_model_address"] == 40365
    assert client.data["sunspec_model_count"] == 8
    assert client.data["mppt_module_count"] == 5
    assert client.data["mppt_visible_module_ids"] == [1, 2, 3]


@pytest.mark.asyncio
async def test_live_snapshot_decodes_expected_poll_values(live_modbus_registers) -> None:
    client = SnapshotModbusClient(live_modbus_registers)
    read_service = client.read_service

    assert await read_service.read_device_info_data(prefix="i_", unit_id=1) is True
    assert await read_service.read_inverter_nameplate_data() is True
    assert await read_service.read_inverter_data() is True
    assert await read_service.read_inverter_status_data() is True
    assert await read_service.read_inverter_model_settings_data() is True
    assert await read_service.read_inverter_controls_data() is True
    assert await read_service.read_mppt_data() is True
    assert await read_service.read_inverter_storage_data() is True
    assert await read_service.read_device_info_data(prefix="meter_200_", unit_id=200) is True
    assert await read_service.read_meter_data(unit_id=200, is_primary=True) is True

    assert client.data["acpower"] == pytest.approx(2268.6)
    assert client.data["line_frequency"] == pytest.approx(50.03)
    assert client.data["status"] == "normal"
    assert client.data["statusvendor"] == "normal"
    assert client.data["events2"] == "None"

    assert client.data["pv_connection"] == "operating"
    assert client.data["storage_connection"] == "operating"
    assert client.data["ecp_connection"] == "connected"
    assert client.data["inverter_controls"] == "Power reduction,Constant power factor"
    assert client.data["isolation_resistance"] == 15

    assert client.data["max_power"] == 17500
    assert client.data["vref"] == 231
    assert client.data["vrefofs"] == 0
    assert client.data["Conn"] == "enabled"
    assert client.data["WMaxLim_Ena"] == "enabled"
    assert client.data["ac_limit_enable"] == "enabled"
    assert client.data["power_factor_enable"] == "enabled"
    assert client.data["power_factor"] == pytest.approx(0.9)
    assert client.data["ac_limit_rate_sf"] == -2
    assert client.data["power_factor_sf"] == -3

    assert client.data["pv_power"] == 10791
    assert client.data["mppt_module_count"] == 5
    assert client.data["module1_power"] == 5498
    assert client.data["module2_power"] == 5293
    assert client.data["module3_power"] is None
    assert client.data["storage_charge_module"] == 4
    assert client.data["storage_charge_power"] == 8583
    assert client.data["storage_discharge_module"] == 5
    assert client.data["storage_discharge_power"] == 0

    assert client.data["soc_minimum"] == 5
    assert client.data["soc"] == pytest.approx(72.9)
    assert client.data["charge_status"] == "charging"
    assert client.data["grid_charging"] == "enabled"
    assert client.data["control_mode"] == "auto"
    assert client.data["ext_control_mode"] == "auto"
    assert client.data["charging_power"] == pytest.approx(100.0)
    assert client.data["discharging_power"] == pytest.approx(100.0)

    assert client.data["meter_200_manufacturer"] == "Fronius"
    assert client.data["meter_200_model"] == "Smart Meter TS 65A-3"
    assert client.data["meter_200_sw_version"] == "1.5"
    assert client.data["meter_200_serial"] == "METER-SERIAL-01"
    assert client.data["meter_200_power"] == pytest.approx(-1782.1)
    assert client.data["meter_200_line_frequency"] == pytest.approx(50.0)
    assert client.data["meter_200_WphA"] == pytest.approx(-609.5)
    assert client.data["meter_200_WphB"] == pytest.approx(-560.9)
    assert client.data["meter_200_WphC"] == pytest.approx(-611.6)
    assert client.data["grid_status"] == "on_grid_operating"
