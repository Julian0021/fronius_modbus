from __future__ import annotations

from types import SimpleNamespace

from homeassistant.core import HomeAssistant
import pytest

import custom_components.fronius_modbus.hub_bootstrap as hub_bootstrap_module
from custom_components.fronius_modbus.hub_bootstrap import (
    HubBootstrapService,
    check_pymodbus_version,
)
from custom_components.fronius_modbus.integration_errors import FroniusDependencyError


class _BootstrapClient:
    def __init__(self, calls: list[object]) -> None:
        self._calls = calls
        self.meter_unit_ids = [200]
        self.primary_meter_unit_id = 200
        self.runtime_service = SimpleNamespace(init_data=self._init_runtime_data)

    async def _init_runtime_data(self) -> None:
        self._calls.append(
            (
                "runtime_init",
                list(self.meter_unit_ids),
                self.primary_meter_unit_id,
            )
        )

    def is_numeric(self, value) -> bool:
        return isinstance(value, (int, float))

    def set_meter_unit_ids(self, unit_ids, *, primary_unit_id=None) -> None:
        self._calls.append(("set_meter_unit_ids", list(unit_ids or []), primary_unit_id))
        self.meter_unit_ids = list(unit_ids or [])
        if primary_unit_id is not None:
            self.primary_meter_unit_id = int(primary_unit_id)

    def reset_storage_info(self) -> None:
        self._calls.append("reset_storage_info")

    def set_storage_info(self, *, manufacturer, model, serial) -> None:
        self._calls.append(("set_storage_info", manufacturer, model, serial))


class _BootstrapHub:
    def __init__(
        self,
        calls: list[object],
        *,
        power_meter_info=None,
    ) -> None:
        self._calls = calls
        self._hass = HomeAssistant()
        self._config_entry = None
        self._auto_enable_modbus = False
        self._host = "inverter.local"
        self._power_meter_info = power_meter_info
        self._webclient = SimpleNamespace(
            get_power_meter_info="get_power_meter_info",
            get_storage_info="get_storage_info",
        )
        self._client = _BootstrapClient(calls)
        self.storage_state = SimpleNamespace(
            set=lambda key, value: calls.append(("storage_state.set", key, value))
        )
        self.web_api_service = SimpleNamespace(
            async_web_job=self._async_web_job,
            async_apply_modbus_config=self._async_apply_modbus_config,
            refresh_web_data=self._async_refresh_web_data,
        )
        self.coordinator = None

    @property
    def web_api_configured(self) -> bool:
        return True

    @property
    def storage_configured(self) -> bool:
        return True

    @property
    def meter_configured(self) -> bool:
        return bool(self._client.meter_unit_ids)

    @property
    def meter_unit_ids(self) -> list[int]:
        return list(self._client.meter_unit_ids)

    @property
    def primary_meter_unit_id(self) -> int:
        return self._client.primary_meter_unit_id

    def set_meter_value(self, unit_id: int, suffix: str, value) -> None:
        self._calls.append(("set_meter_value", unit_id, suffix, value))

    async def _async_apply_modbus_config(self) -> None:
        self._calls.append("apply_modbus_config")

    async def _async_refresh_web_data(self) -> None:
        self._calls.append("refresh_web_data")

    async def _async_web_job(self, func, *args, **kwargs):
        self._calls.append(("async_web_job", func, args, kwargs))
        if func == "get_power_meter_info":
            return self._power_meter_info or {
                "unit_ids": [200],
                "primary_unit_id": 200,
                "phase_counts_by_unit_id": {200: 3},
                "locations_by_unit_id": {200: 0},
            }
        if func == "get_storage_info":
            return {
                "manufacturer": "BYD",
                "model": "Battery Box",
                "serial": "serial-1",
                "cell_temperature": 23.5,
            }
        return None


async def test_bootstrap_service_runs_named_startup_steps(monkeypatch) -> None:
    calls: list[object] = []
    hub = _BootstrapHub(calls)
    service = HubBootstrapService(hub)

    def _fake_check_pymodbus_version() -> None:
        calls.append("check_pymodbus_version")

    monkeypatch.setattr(
        hub_bootstrap_module,
        "check_pymodbus_version",
        _fake_check_pymodbus_version,
    )

    await service.init_data(setup_coordinator=False)

    assert calls == [
        "check_pymodbus_version",
        ("async_web_job", "get_power_meter_info", (200,), {}),
        ("set_meter_unit_ids", [200], 200),
        ("runtime_init", [200], 200),
        ("set_meter_value", 200, "phase_count", 3),
        ("set_meter_value", 200, "location", 0),
        "reset_storage_info",
        ("async_web_job", "get_storage_info", (), {}),
        ("set_storage_info", "BYD", "Battery Box", "serial-1"),
        ("storage_state.set", "storage_temperature", 23.5),
        "refresh_web_data",
    ]


async def test_bootstrap_service_ignores_malformed_power_meter_topology(monkeypatch) -> None:
    calls: list[object] = []
    hub = _BootstrapHub(
        calls,
        power_meter_info={
            "unit_ids": "not-a-list",
            "primary_unit_id": 240,
            "phase_counts_by_unit_id": {200: 3},
            "locations_by_unit_id": {200: 0},
        },
    )
    service = HubBootstrapService(hub)

    monkeypatch.setattr(
        hub_bootstrap_module,
        "check_pymodbus_version",
        lambda: calls.append("check_pymodbus_version"),
    )

    await service.init_data(setup_coordinator=False)

    assert not any(
        isinstance(call, tuple) and call[0] == "set_meter_unit_ids"
        for call in calls
    )
    assert ("runtime_init", [200], 200) in calls
    assert hub.meter_unit_ids == [200]
    assert hub.primary_meter_unit_id == 200
    assert ("set_meter_value", 200, "phase_count", 3) in calls
    assert ("set_meter_value", 200, "location", 0) in calls


async def test_bootstrap_service_applies_validated_meter_topology_before_runtime_init(
    monkeypatch,
) -> None:
    calls: list[object] = []
    hub = _BootstrapHub(
        calls,
        power_meter_info={
            "unit_ids": [200, 240, 0, "bad", 200],
            "primary_unit_id": 999,
            "phase_counts_by_unit_id": {200: 3, 240: 1},
            "locations_by_unit_id": {200: 0, 240: 1},
        },
    )
    service = HubBootstrapService(hub)

    monkeypatch.setattr(
        hub_bootstrap_module,
        "check_pymodbus_version",
        lambda: calls.append("check_pymodbus_version"),
    )

    await service.init_data(setup_coordinator=False)

    assert calls[:4] == [
        "check_pymodbus_version",
        ("async_web_job", "get_power_meter_info", (200,), {}),
        ("set_meter_unit_ids", [200, 240], None),
        ("runtime_init", [200, 240], 200),
    ]
    assert hub.meter_unit_ids == [200, 240]
    assert hub.primary_meter_unit_id == 200
    assert ("set_meter_value", 240, "phase_count", 1) in calls
    assert ("set_meter_value", 240, "location", 1) in calls


def test_check_pymodbus_version_raises_dependency_error_on_resolution_failure(
    monkeypatch,
) -> None:
    def _raise_version_error(_name: str) -> str:
        raise RuntimeError("missing dist")

    monkeypatch.setattr(
        hub_bootstrap_module,
        "_manifest_pymodbus_minimum_version",
        lambda: "3.7.0",
    )
    monkeypatch.setattr(hub_bootstrap_module, "version", _raise_version_error)

    with pytest.raises(FroniusDependencyError, match="installed pymodbus version"):
        check_pymodbus_version()


def test_check_pymodbus_version_raises_dependency_error_on_old_version(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        hub_bootstrap_module,
        "_manifest_pymodbus_minimum_version",
        lambda: "3.7.0",
    )
    monkeypatch.setattr(hub_bootstrap_module, "version", lambda _name: "3.6.0")

    with pytest.raises(FroniusDependencyError, match=r"please update to 3\.7\.0 or higher"):
        check_pymodbus_version()
