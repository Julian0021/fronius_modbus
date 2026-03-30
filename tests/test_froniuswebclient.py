from __future__ import annotations

import pytest
from requests import RequestException

from custom_components.fronius_modbus.froniuswebclient import (
    FroniusWebAuthError,
    FroniusWebClient,
)
from custom_components.fronius_modbus.integration_errors import FroniusReadError

POWER_METER_READABLE_PAYLOAD = {
    "Body": {
        "Data": {
            "meter_node": {
                "attributes": {
                    "addr": "1",
                    "label": "<primary>",
                    "manufacturer": "Fronius",
                    "meter-location": "0",
                    "model": "Smart Meter TS 65A-3",
                    "phaseCnt": "3",
                }
            }
        }
    }
}

POWER_METER_TCP_READABLE_PAYLOAD = {
    "Body": {
        "Data": {
            "meter_node": {
                "attributes": {
                    "addr": "1",
                    "manufacturer": "Fronius",
                    "meter-location": "0",
                    "model": "Smart Meter 63A",
                    "phaseCnt": "3",
                    "connection": (
                        '{"id":"192.168.0.205:502","ip":"192.168.0.205",'
                        '"name":"modbus-tcp","port":502,"protocol":"ModbusTCP"}\n'
                    ),
                    "if": "modbus-tcp;192.168.0.205:502",
                }
            }
        }
    }
}

INVERTER_READABLE_PAYLOAD = {
    "Body": {
        "Data": {
            "inverter_node": {
                "channels": {
                    "DEVICE_TEMPERATURE_AMBIENTMEAN_01_F32": 34.798126220703125,
                }
            }
        }
    }
}

STORAGE_READABLE_PAYLOAD = {
    "Body": {
        "Data": {
            "storage_node": {
                "attributes": {
                    "DisplayName": "BYD3",
                    "manufacturer": "BYD",
                    "model": "HVB",
                    "nameplate": (
                        "{\"manufacturer\":\"BYD\",\"model\":\"HVB\","
                        "\"serial\":\"BATTERY-SERIAL-01\"}"
                    ),
                    "serial": "BATTERY-SERIAL-01",
                },
                "channels": {
                    "BAT_TEMPERATURE_CELL_F64": 14.450000000000001,
                },
            }
        }
    }
}


def test_get_power_meter_info_parses_live_payload(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")
    monkeypatch.setattr(client, "_get_public_json", lambda path: POWER_METER_READABLE_PAYLOAD)

    meter_info = client.get_power_meter_info()

    assert meter_info == {
        "unit_ids": [200],
        "primary_unit_id": 200,
        "phase_counts_by_unit_id": {200: 3},
        "locations_by_unit_id": {200: 0},
        "tcp_meter_connections_by_unit_id": {},
        "payload_shape": "Body.Data",
    }


def test_get_power_meter_info_parses_modbus_tcp_meter_endpoint(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")
    monkeypatch.setattr(client, "_get_public_json", lambda path: POWER_METER_TCP_READABLE_PAYLOAD)

    meter_info = client.get_power_meter_info()

    assert meter_info == {
        "unit_ids": [200],
        "primary_unit_id": 200,
        "phase_counts_by_unit_id": {200: 3},
        "locations_by_unit_id": {200: 0},
        "tcp_meter_connections_by_unit_id": {
            200: {"host": "192.168.0.205", "port": 502}
        },
        "payload_shape": "Body.Data",
    }


def test_get_power_meter_info_raises_read_error_for_unrecognized_payload(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")
    monkeypatch.setattr(client, "_get_public_json", lambda path: {"unexpected": "payload"})

    with pytest.raises(FroniusReadError):
        client.get_power_meter_info()


def test_get_inverter_info_parses_live_payload(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")
    monkeypatch.setattr(client, "_get_json", lambda path: INVERTER_READABLE_PAYLOAD)

    inverter_info = client.get_inverter_info()

    assert inverter_info["temperature"] == pytest.approx(34.798126220703125)


def test_get_storage_info_parses_live_payload(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")
    monkeypatch.setattr(client, "_get_json", lambda path: STORAGE_READABLE_PAYLOAD)

    storage_info = client.get_storage_info()

    assert storage_info == {
        "manufacturer": "BYD",
        "model": "HVB",
        "serial": "BATTERY-SERIAL-01",
        "cell_temperature": pytest.approx(14.45),
    }


def test_get_storage_info_propagates_request_failure(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")

    def _raise_request_error(_path: str):
        raise RequestException("boom")

    monkeypatch.setattr(client, "_get_json", _raise_request_error)

    with pytest.raises(RequestException):
        client.get_storage_info()


def test_get_storage_info_propagates_auth_failure(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")

    def _raise_auth_error(_path: str):
        raise FroniusWebAuthError("auth failed")

    monkeypatch.setattr(client, "_get_json", _raise_auth_error)

    with pytest.raises(FroniusWebAuthError):
        client.get_storage_info()


def test_get_storage_info_raises_read_error_for_unexpected_payload_shape(monkeypatch) -> None:
    client = FroniusWebClient("fixture-host")
    monkeypatch.setattr(client, "_get_json", lambda path: {"unexpected": "payload"})

    with pytest.raises(FroniusReadError):
        client.get_storage_info()
