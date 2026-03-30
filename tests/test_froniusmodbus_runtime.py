from __future__ import annotations

import pytest

from custom_components.fronius_modbus.froniusmodbusclient import FroniusModbusClient
from custom_components.fronius_modbus.integration_errors import FroniusReadError


def _make_client() -> FroniusModbusClient:
    client = FroniusModbusClient(
        host="fixture-host",
        port=502,
        inverter_unit_id=1,
        meter_unit_ids=[200, 201],
        timeout=1,
    )
    client.set_meter_unit_ids([200, 201], primary_unit_id=201)
    return client


@pytest.mark.asyncio
async def test_runtime_meter_discovery_realigns_primary_meter_to_discovered_set(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = _make_client()

    async def _fake_connect() -> bool:
        return True

    async def _fake_read_device_info_data(*, prefix: str, unit_id: int) -> bool:
        if prefix == "i_":
            return True
        if unit_id == 200:
            client.data[f"{prefix}manufacturer"] = "Vendor"
            client.data[f"{prefix}model"] = "Unknown"
        elif unit_id == 201:
            client.data[f"{prefix}manufacturer"] = "Fronius"
            client.data[f"{prefix}model"] = "Smart Meter TS"
        else:
            raise AssertionError(f"Unexpected unit id {unit_id}")
        return True

    async def _fake_read_mppt_data() -> bool:
        return False

    async def _fake_read_inverter_nameplate_data() -> bool:
        return True

    monkeypatch.setattr(client, "connect", _fake_connect)
    monkeypatch.setattr(
        client.read_service,
        "read_device_info_data",
        _fake_read_device_info_data,
    )
    monkeypatch.setattr(client.read_service, "read_mppt_data", _fake_read_mppt_data)
    monkeypatch.setattr(
        client.read_service,
        "read_inverter_nameplate_data",
        _fake_read_inverter_nameplate_data,
    )

    assert await client.runtime_service.init_data() is True
    assert client.meter_unit_ids == (201,)
    assert client.primary_meter_unit_id == 201
    assert client.meter_configured is True


@pytest.mark.asyncio
async def test_runtime_meter_discovery_preserves_primary_meter_on_probe_failures(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = _make_client()

    async def _fake_connect() -> bool:
        return True

    async def _fake_read_device_info_data(*, prefix: str, unit_id: int) -> bool:
        if prefix == "i_":
            return True
        raise FroniusReadError(f"meter {unit_id} unavailable")

    async def _fake_read_mppt_data() -> bool:
        return False

    async def _fake_read_inverter_nameplate_data() -> bool:
        return True

    monkeypatch.setattr(client, "connect", _fake_connect)
    monkeypatch.setattr(
        client.read_service,
        "read_device_info_data",
        _fake_read_device_info_data,
    )
    monkeypatch.setattr(client.read_service, "read_mppt_data", _fake_read_mppt_data)
    monkeypatch.setattr(
        client.read_service,
        "read_inverter_nameplate_data",
        _fake_read_inverter_nameplate_data,
    )

    assert await client.runtime_service.init_data() is True
    assert client.meter_unit_ids == (200, 201)
    assert client.primary_meter_unit_id == 201
    assert client.meter_configured is True
