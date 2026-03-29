from __future__ import annotations

import asyncio
from types import SimpleNamespace

from custom_components.fronius_modbus.froniusmodbus_writes import (
    FroniusModbusWriteService,
)
from custom_components.fronius_modbus.froniusmodbusclient_const import (
    CHARGE_RATE_OFFSET,
    CONTROL_STATE_ENABLED,
    DISCHARGE_RATE_OFFSET,
    MINIMUM_RESERVE_OFFSET,
    MINIMUM_RESERVE_PERCENT_MIN,
    OUT_PF_SET_ENABLE_ADDRESS,
    STORAGE_CONTROL_MODE_OFFSET,
)
from custom_components.fronius_modbus.hub_commands import HubCommandService
from custom_components.fronius_modbus.hub_web_api import HubWebApiService


class _WriteFacade:
    def __init__(self) -> None:
        self.storage_extended_control_mode = 4
        self.max_charge_rate_w = 1000
        self.max_discharge_rate_w = 1000
        self._inverter_unit_id = 1
        self.data = {"grid_charge_power": "stale", "max_power": 10000, "ac_limit_rate_sf": -2}
        self.write_calls: list[tuple[int, int]] = []
        self.storage_refreshes = 0
        self.controls_refreshes = 0
        self._power_factor_enable_mask_until = 0.0
        self.read_service = SimpleNamespace(
            read_inverter_storage_data=self.read_inverter_storage_data,
            read_inverter_controls_data=self.read_inverter_controls_data,
        )

    async def write_registers(self, *, unit_id, address, payload):
        self.write_calls.append((unit_id, address, payload[0]))

    async def read_inverter_storage_data(self):
        self.storage_refreshes += 1

    async def read_inverter_controls_data(self):
        self.controls_refreshes += 1

    def is_numeric(self, value) -> bool:
        return isinstance(value, (int, float))

    def _storage_register_address(self, offset: int) -> int:
        return 40300 + offset


async def test_modbus_write_service_uses_readback_instead_of_cache_mutation() -> None:
    facade = _WriteFacade()
    service = FroniusModbusWriteService(facade)

    await service.set_grid_charge_power(500)

    assert facade.data["grid_charge_power"] == "stale"
    assert facade.storage_refreshes == 1
    assert facade.write_calls == [(1, 40300 + DISCHARGE_RATE_OFFSET, 60536)]


async def test_storage_writes_use_named_protocol_offsets() -> None:
    facade = _WriteFacade()
    service = FroniusModbusWriteService(facade)

    await service.set_storage_control_mode(2)
    await service.set_minimum_reserve(MINIMUM_RESERVE_PERCENT_MIN)
    await service.set_discharge_rate(10)
    await service.set_charge_rate(20)

    assert facade.write_calls == [
        (1, 40300 + STORAGE_CONTROL_MODE_OFFSET, 2),
        (1, 40300 + MINIMUM_RESERVE_OFFSET, MINIMUM_RESERVE_PERCENT_MIN * 100),
        (1, 40300 + DISCHARGE_RATE_OFFSET, 1000),
        (1, 40300 + CHARGE_RATE_OFFSET, 2000),
    ]
    assert facade.storage_refreshes == 4


async def test_set_power_factor_enable_uses_named_control_state() -> None:
    facade = _WriteFacade()
    service = FroniusModbusWriteService(facade)

    await service.set_power_factor_enable(CONTROL_STATE_ENABLED)

    assert facade.write_calls == [(1, OUT_PF_SET_ENABLE_ADDRESS, CONTROL_STATE_ENABLED)]
    assert facade.controls_refreshes == 1


async def test_hub_command_service_does_not_optimistically_mutate_api_battery_power() -> None:
    calls: list[tuple] = []

    async def _fake_web_job(func, *args, **kwargs):
        calls.append(("web_job", func, args, kwargs))

    def _fake_transition(source: str) -> None:
        calls.append(("transition", source))

    hub = SimpleNamespace(
        _command_lock=asyncio.Lock(),
        _webclient=SimpleNamespace(set_battery_config="set_battery_config"),
        data={"api_battery_power": "stale", "api_battery_mode_effective_raw": 1},
        _as_int=lambda value: int(value) if isinstance(value, int) else None,
        _web_api_service=SimpleNamespace(
            async_web_job=_fake_web_job,
            start_battery_write_transition=_fake_transition,
        ),
    )
    service = HubCommandService(hub)

    await service.set_api_battery_power(321.4)

    assert hub.data["api_battery_power"] == "stale"
    assert calls == [
        (
            "web_job",
            "set_battery_config",
            (1, -321),
            {"raise_on_auth_failure": True},
        ),
        ("transition", "Target feed in"),
    ]


async def test_hub_command_service_uses_shared_mode_policy_for_dispatch() -> None:
    calls: list[tuple | str] = []

    async def _fake_set_extended_mode(mode):
        calls.append(("set_extended_mode", mode))

    hub = SimpleNamespace(
        _command_lock=asyncio.Lock(),
        _client=SimpleNamespace(
            write_service=SimpleNamespace(set_extended_mode=_fake_set_extended_mode)
        ),
        _webclient=object(),
        _publish_data_update=lambda: calls.append("publish"),
    )
    service = HubCommandService(hub)

    async def _fake_set_api_charge_sources(*, charge_from_grid: bool, charge_from_ac: bool):
        calls.append(
            (
                "set_api_charge_sources",
                charge_from_grid,
                charge_from_ac,
            )
        )

    service._set_api_charge_sources = _fake_set_api_charge_sources  # type: ignore[method-assign]

    await service.set_mode(4)

    assert calls == [
        ("set_extended_mode", 4),
        ("set_api_charge_sources", True, True),
        "publish",
    ]


async def test_web_api_toggle_refreshes_and_publishes_without_cache_shortcut() -> None:
    calls: list[tuple | str] = []

    hub = SimpleNamespace(
        _command_lock=asyncio.Lock(),
        _webclient=SimpleNamespace(set_solar_api_enabled="set_solar_api_enabled"),
        data={"api_solar_api_enabled": "stale"},
        _publish_data_update=lambda: calls.append("publish"),
    )
    service = HubWebApiService(hub)

    async def _fake_web_job(func, *args, **kwargs):
        calls.append(("web_job", func, args, kwargs))

    async def _fake_refresh():
        calls.append("refresh")

    service.async_web_job = _fake_web_job  # type: ignore[method-assign]
    service.refresh_web_data = _fake_refresh  # type: ignore[method-assign]

    await service.set_solar_api_enabled(True)

    assert hub.data["api_solar_api_enabled"] == "stale"
    assert calls == [
        (
            "web_job",
            "set_solar_api_enabled",
            (True,),
            {"raise_on_auth_failure": True},
        ),
        "refresh",
        "publish",
    ]
