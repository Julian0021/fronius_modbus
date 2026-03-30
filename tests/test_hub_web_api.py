from __future__ import annotations

from types import SimpleNamespace

from homeassistant.core import HomeAssistant

import custom_components.fronius_modbus.hub_web_api as hub_web_api_module
from custom_components.fronius_modbus.const import API_USERNAME
from custom_components.fronius_modbus.froniuswebclient import FroniusWebAuthError
from custom_components.fronius_modbus.hub_web_api import HubWebApiService
from custom_components.fronius_modbus.runtime_state import FroniusRuntimeState


def _enabled_bool(value) -> bool:
    return str(value).lower() in {"1", "enabled", "on", "true"}


def _enabled_state(value) -> str:
    return "Enabled" if _enabled_bool(value) else "Disabled"


def _build_hub() -> tuple[SimpleNamespace, list[tuple[str, str]], list[str]]:
    state = FroniusRuntimeState()
    deleted_tokens: list[tuple[str, str]] = []
    warning_calls: list[str] = []

    async def _sync_warning() -> None:
        warning_calls.append("sync")

    hub = SimpleNamespace(
        _hass=HomeAssistant(),
        _host="inverter.local",
        _name="Fronius",
        _webclient=SimpleNamespace(
            get_inverter_info=lambda: {"temperature": 41.2},
            get_modbus_config=lambda: {
                "slave": {
                    "mode": "tcp",
                    "sunspecMode": "float32",
                    "ctr": {
                        "on": "enabled",
                        "restriction": {"on": "enabled", "ip": "192.168.1.15"},
                    },
                }
            },
            get_solar_api_config=lambda: {"SolarAPIv1Enabled": "enabled"},
            get_storage_info=lambda: {"cell_temperature": 23.5},
            get_battery_config=lambda: {
                "HYB_EM_MODE": 1,
                "HYB_EM_POWER": 2500,
                "BAT_M0_SOC_MODE": "manual",
                "BAT_M0_SOC_MIN": 23,
                "BAT_M0_SOC_MAX": 95,
                "HYB_BACKUP_RESERVED": 7,
                "HYB_BM_CHARGEFROMAC": "enabled",
                "HYB_EVU_CHARGEFROMGRID": "disabled",
            },
        ),
        data=state.data,
        web_state=state.web_api,
        storage_state=state.storage,
        _as_int=lambda value: int(value) if value is not None else None,
        _derive_api_battery_mode=lambda raw_mode, _raw_soc_mode: raw_mode,
        _enabled_bool=_enabled_bool,
        _enabled_state=_enabled_state,
        storage_configured=True,
        warning_service=SimpleNamespace(async_sync_solar_api_warning=_sync_warning),
        _config_entry=None,
    )

    return hub, deleted_tokens, warning_calls


async def test_auth_failure_clears_tracked_web_cache_keys(monkeypatch) -> None:
    hub, deleted_tokens, warning_calls = _build_hub()
    service = HubWebApiService(hub)

    async def _delete_token(host: str, user: str) -> None:
        deleted_tokens.append((host, user))

    monkeypatch.setattr(
        hub_web_api_module,
        "async_get_token_store",
        lambda _hass: SimpleNamespace(async_delete_token=_delete_token),
    )

    await service.refresh_web_data()

    assert hub.data["api_modbus_restriction_ip"] == "192.168.1.15"
    assert hub.data["api_battery_mode_consistent"] is True
    assert hub.data["api_charge_from_ac"] is True
    assert hub.data["storage_temperature"] == 23.5
    assert hub.data["soc_minimum"] == 23
    assert service._clearable_web_state_keys >= {
        "api_battery_mode",
        "api_battery_mode_consistent",
        "api_modbus_restriction_ip",
        "api_solar_api_enabled",
        "inverter_temperature",
    }
    assert service._clearable_storage_state_keys == {"storage_temperature"}

    await service._async_handle_web_api_auth_failure(FroniusWebAuthError("401"))

    assert hub._webclient is None
    assert deleted_tokens == [("inverter.local", API_USERNAME)]
    assert warning_calls == ["sync", "sync"]
    assert hub.data["inverter_temperature"] is None
    assert hub.data["api_modbus_restriction_ip"] is None
    assert hub.data["api_solar_api_enabled"] is None
    assert hub.data["api_battery_mode"] is None
    assert hub.data["api_battery_mode_consistent"] is None
    assert hub.data["api_charge_from_ac"] is None
    assert hub.data["api_charge_from_grid"] is None
    assert hub.data["storage_temperature"] is None
    assert hub.data["soc_minimum"] == 23
