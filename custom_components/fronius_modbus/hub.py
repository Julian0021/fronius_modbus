"""Fronius Modbus hub state container and coordinator."""
from __future__ import annotations

import asyncio
from datetime import timedelta
from inspect import signature
import logging
import re
import time
from typing import Any

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator

from .const import DOMAIN, ENTITY_PREFIX, SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX
from .froniusmodbusclient import FroniusModbusClient
from .froniuswebclient import FroniusWebClient
from .hub_bootstrap import HubBootstrapService
from .hub_commands import HubCommandService
from .hub_runtime import HubRuntimeService
from .hub_warnings import HubWarningService
from .hub_web_api import HubWebApiService
from .runtime_state import FroniusRuntimeState, StateSection
from .value_normalization import enabled_state, is_enabled

_LOGGER = logging.getLogger(__name__)
_SOLAR_API_FIRMWARE_RE = re.compile(r"^(\d+)\.(\d+)\.(\d+)(?:-(\d+))?$")


def _coordinator_init_supports_config_entry() -> bool:
    try:
        return "config_entry" in signature(DataUpdateCoordinator.__init__).parameters
    except (TypeError, ValueError):
        return False


class FroniusCoordinator(DataUpdateCoordinator):
    """Coordinator for Fronius Modbus data updates."""

    def __init__(
        self,
        hass: HomeAssistant,
        hub: Hub,
        config_entry: ConfigEntry | None = None,
    ) -> None:
        """Initialize the coordinator."""
        init_kwargs = {
            "name": f"{DOMAIN}_{hub._id}_coordinator",
            "update_interval": hub._scan_interval,
        }
        if config_entry is not None and _coordinator_init_supports_config_entry():
            init_kwargs["config_entry"] = config_entry

        super().__init__(
            hass,
            _LOGGER,
            **init_kwargs,
        )
        self.hub = hub

    async def _async_update_data(self) -> dict:
        """Fetch all data from the Fronius device."""
        return await self.hub.runtime_service.async_refresh_data()


class Hub:
    """Shared runtime state and service access for the Fronius integration."""

    def __init__(
        self,
        hass: HomeAssistant,
        name: str,
        host: str,
        port: int,
        inverter_unit_id: int,
        meter_unit_ids,
        scan_interval: int,
        api_username: str | None = None,
        api_password: str | None = None,
        api_token: dict[str, str] | None = None,
        auto_enable_modbus: bool = True,
        restrict_modbus_to_this_ip: bool = False,
    ) -> None:
        """Initialize the hub state container and internal services."""
        self._hass = hass
        self._name = name
        self._host = host
        self._port = port
        self._inverter_unit_id = inverter_unit_id
        self._config_entry: ConfigEntry | None = None
        self._auto_enable_modbus = auto_enable_modbus
        self._restrict_modbus_to_this_ip = restrict_modbus_to_this_ip
        self._webclient: FroniusWebClient | None = None

        self._id = f"{name.lower()}_{host.lower().replace('.', '')}"

        self._client = FroniusModbusClient(
            host=host,
            port=port,
            inverter_unit_id=inverter_unit_id,
            meter_unit_ids=meter_unit_ids,
            timeout=max(3, scan_interval - 1),
        )
        if api_username and (api_password or api_token):
            self._webclient = FroniusWebClient(
                host=host,
                username=api_username,
                password=api_password or "",
                token=api_token,
            )

        self._scan_interval = timedelta(seconds=scan_interval)
        self.coordinator = None
        self._command_lock = asyncio.Lock()
        self._battery_write_transition_until = 0.0
        self._battery_write_transition_warned = False
        self._delayed_web_refresh_task: asyncio.Task | None = None
        self._last_good_load_w: float | None = None
        self._last_good_inverter_power_w: float | None = None
        self._consecutive_bad_load_polls = 0

        self._runtime_service = HubRuntimeService(self)
        self._bootstrap_service = HubBootstrapService(self)
        self._warning_service = HubWarningService(self)
        self._web_api_service = HubWebApiService(self)
        self._command_service = HubCommandService(self)

    def _meter_prefix(self, unit_id: int) -> str:
        return f"meter_{int(unit_id)}_"

    def _reset_bad_load_tracking(self) -> None:
        self._consecutive_bad_load_polls = 0

    def _set_load(
        self,
        load_w: float,
        *,
        cache: bool = False,
        inverter_power: float | None = None,
    ) -> None:
        load_value = round(float(load_w), 2)
        self.derived_state.set("load", load_value)
        if cache:
            self._last_good_load_w = load_value
            if inverter_power is not None:
                self._last_good_inverter_power_w = float(inverter_power)
        self._reset_bad_load_tracking()

    def _is_inverter_power_glitch(
        self,
        *,
        candidate_load: float,
        meter_power: float,
        inverter_power: float,
    ) -> bool:
        pv_power = self.mppt_state.get("pv_power")
        pv_active = (
            self._client.is_numeric(pv_power) and float(pv_power) >= 1000.0
        )
        previous_inverter_active = self._last_good_inverter_power_w is not None and (
            self._last_good_inverter_power_w >= 1000.0
        )
        return (
            candidate_load < 0
            and meter_power <= -1000.0
            and abs(inverter_power) <= 50.0
            and (pv_active or previous_inverter_active)
        )

    def _apply_glitch_load_fallback(self) -> None:
        self._consecutive_bad_load_polls += 1
        if self._consecutive_bad_load_polls == 1 and self._last_good_load_w is not None:
            self.derived_state.set("load", self._last_good_load_w)

    @property
    def solar_api_warning_issue_id(self) -> str | None:
        if self._config_entry is None:
            return None
        return f"{SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX}{self._config_entry.entry_id}"

    def parse_firmware_version(
        self,
        version_text: Any,
    ) -> tuple[int, int, int, int] | None:
        if not isinstance(version_text, str):
            return None

        match = _SOLAR_API_FIRMWARE_RE.fullmatch(version_text.strip())
        if match is None:
            return None

        major, minor, patch, build = match.groups(default="0")
        return (int(major), int(minor), int(patch), int(build))

    @property
    def hass(self) -> HomeAssistant:
        return self._hass

    @property
    def warning_entry_id(self) -> str | None:
        if self._config_entry is None:
            return None
        return self._config_entry.entry_id

    @property
    def warning_entry_title(self) -> str:
        if self._config_entry is not None:
            return self._config_entry.title
        return self._name

    def _battery_write_transition_active(self) -> bool:
        return time.monotonic() < self._battery_write_transition_until

    def _clear_battery_write_transition(self) -> None:
        self._battery_write_transition_until = 0.0
        self._battery_write_transition_warned = False

    def _as_int(self, value: Any) -> int | None:
        try:
            return int(value) if value is not None else None
        except (TypeError, ValueError):
            return None

    def _require_whole_number(self, value: Any, field_name: str) -> int:
        try:
            numeric_value = float(value)
        except (TypeError, ValueError) as err:
            raise ValueError(f"{field_name} must be a whole number") from err

        if not numeric_value.is_integer():
            raise ValueError(f"{field_name} must be a whole number")

        return int(numeric_value)

    def _enabled_state(self, value: Any) -> str:
        return enabled_state(value)

    def _enabled_bool(self, value: Any) -> bool:
        return is_enabled(value)

    @property
    def device_info_storage(self) -> dict:
        return {
            "identifiers": {(DOMAIN, self.storage_device_identifier)},
            "name": f"{self.storage_state.get('s_model')}",
            "manufacturer": self.storage_state.get("s_manufacturer"),
            "model": self.storage_state.get("s_model"),
            "serial_number": self.storage_state.get("s_serial"),
        }

    @property
    def device_info_inverter(self) -> dict:
        return {
            "identifiers": {(DOMAIN, self.inverter_device_identifier)},
            "name": f"Fronius {self.inverter_state.get('i_model')}",
            "manufacturer": self.inverter_state.get("i_manufacturer"),
            "model": self.inverter_state.get("i_model"),
            "serial_number": self.inverter_state.get("i_serial"),
            "sw_version": self.inverter_state.get("i_sw_version"),
        }

    def get_device_info_meter(self, unit_id: int) -> dict:
        try:
            meter_position = self.meter_unit_ids.index(unit_id) + 1
        except ValueError:
            meter_position = 1
        return {
            "identifiers": {(DOMAIN, self.meter_device_identifier(unit_id))},
            "name": (
                f"Fronius {self.meter_value(unit_id, 'model')} "
                f"Meter {meter_position}"
            ),
            "manufacturer": self.meter_value(unit_id, "manufacturer"),
            "model": self.meter_value(unit_id, "model"),
            "serial_number": self.meter_value(unit_id, "serial"),
            "sw_version": self.meter_value(unit_id, "sw_version"),
        }

    @property
    def hub_id(self) -> str:
        """ID for hub."""
        return self._id

    @property
    def entity_prefix(self) -> str:
        """Entity prefix for hub."""
        return f"{ENTITY_PREFIX}_{self.identity_namespace}"

    @property
    def identity_namespace(self) -> str:
        """Stable namespace used for registry identities and unique ids."""
        if self._config_entry is not None:
            return self._config_entry.entry_id
        return self._name.lower()

    def _device_identifier(self, suffix: str) -> str:
        if self._config_entry is not None:
            return f"entry_{self._config_entry.entry_id}_{suffix}"
        return f"{self._name}_{suffix}"

    @property
    def inverter_device_identifier(self) -> str:
        return self._device_identifier("inverter")

    @property
    def storage_device_identifier(self) -> str:
        return self._device_identifier("battery_storage")

    def meter_device_identifier(self, unit_id: int) -> str:
        return self._device_identifier(f"meter_{int(unit_id)}")

    def close(self):
        """Disconnect the Modbus client and cancel any delayed refresh task."""
        if self._delayed_web_refresh_task and not self._delayed_web_refresh_task.done():
            self._delayed_web_refresh_task.cancel()
        self._client.close()

    def _publish_data_update(self) -> None:
        """Push the current shared data snapshot to the coordinator when possible."""
        if self.coordinator is None:
            return
        set_updated_data = getattr(self.coordinator, "async_set_updated_data", None)
        if callable(set_updated_data):
            set_updated_data(self.snapshot_data())

    @property
    def data(self):
        return self._client.data

    @property
    def state(self) -> FroniusRuntimeState:
        return self._client.state

    @property
    def inverter_state(self) -> StateSection:
        return self._client.inverter_state

    @property
    def storage_state(self) -> StateSection:
        return self._client.storage_state

    @property
    def web_state(self) -> StateSection:
        return self._client.web_api_state

    @property
    def web_api_state(self) -> StateSection:
        return self.web_state

    @property
    def mppt_state(self) -> StateSection:
        return self._client.mppt_state

    @property
    def derived_state(self) -> StateSection:
        return self._client.derived_state

    def meter_state(self, unit_id: int) -> StateSection:
        return self._client.meter_state(unit_id)

    def meter_value(self, unit_id: int, suffix: str, default: Any = None) -> Any:
        return self._client.meter_value(unit_id, suffix, default)

    def set_meter_value(self, unit_id: int, suffix: str, value: Any) -> None:
        self._client.set_meter_value(unit_id, suffix, value)

    def snapshot_data(self) -> dict[str, Any]:
        return self._client.data_snapshot()

    @property
    def runtime_service(self) -> HubRuntimeService:
        return self._runtime_service

    @property
    def bootstrap_service(self) -> HubBootstrapService:
        return self._bootstrap_service

    @property
    def warning_service(self) -> HubWarningService:
        return self._warning_service

    @property
    def web_api_service(self) -> HubWebApiService:
        return self._web_api_service

    @property
    def command_service(self) -> HubCommandService:
        return self._command_service

    @property
    def web_api_configured(self) -> bool:
        return self._webclient is not None

    @property
    def meter_configured(self):
        return self._client.meter_configured

    @property
    def meter_unit_ids(self) -> list[int]:
        return list(self._client.meter_unit_ids)

    @property
    def primary_meter_unit_id(self) -> int:
        return self._client.primary_meter_unit_id

    @property
    def storage_configured(self):
        return self._client.storage_configured

    @property
    def mppt_configured(self) -> bool:
        return self._client.mppt_configured

    @property
    def entity_registry_cleanup_safe(self) -> bool:
        return bool(getattr(self._client, "entity_registry_cleanup_safe", True))

    @property
    def mppt_module_count(self) -> int:
        return int(self._client.mppt_module_count)

    @property
    def visible_mppt_module_ids(self) -> list[int]:
        visible_module_ids = self.mppt_state.get("mppt_visible_module_ids")
        if (
            isinstance(visible_module_ids, list)
            and all(isinstance(module_id, int) for module_id in visible_module_ids)
        ):
            return list(visible_module_ids)
        return list(range(1, self.mppt_module_count + 1))

    @property
    def supports_three_phase_inverter(self) -> bool:
        model = str(self.inverter_state.get("i_model") or "")
        return not model.startswith("Primo")

    @property
    def max_discharge_rate_w(self):
        return self._client.max_discharge_rate_w

    @property
    def max_charge_rate_w(self):
        return self._client.max_charge_rate_w

    @property
    def storage_extended_control_mode(self):
        return self._client.storage_extended_control_mode
