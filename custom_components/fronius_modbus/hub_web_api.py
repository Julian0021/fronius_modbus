"""Web API helpers for Hub lifecycle and cached state."""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

from homeassistant.helpers import issue_registry as ir
from requests import RequestException

from .const import API_USERNAME, DOMAIN, MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX
from .froniuswebclient import FroniusWebAuthError
from .hub_commands import toggle_busy
from .integration_errors import (
    FroniusAuthError,
    FroniusConnectionError,
    FroniusError,
    FroniusReadError,
)
from .token_store import async_get_token_store

_LOGGER = logging.getLogger(__name__)

WEB_API_DATA_KEYS = (
    "inverter_temperature",
    "api_modbus_mode",
    "api_modbus_control",
    "api_modbus_sunspec_mode",
    "api_modbus_restriction",
    "api_modbus_restriction_ip",
    "api_solar_api_enabled",
    "storage_temperature",
    "api_battery_mode_raw",
    "api_battery_mode_effective_raw",
    "api_battery_mode_consistent",
    "api_battery_mode",
    "api_battery_power",
    "api_soc_mode_raw",
    "api_soc_mode",
    "api_soc_min",
    "soc_maximum",
    "api_backup_reserved",
    "api_charge_from_ac",
    "api_charge_from_grid",
)

BATTERY_WRITE_MODBUS_RECOVERY_SECONDS = 30.0
BATTERY_WRITE_WEB_REFRESH_DELAY_SECONDS = 10.0


class HubWebApiService:
    """Own Web API calls, auth handling, and cached web-state refreshes."""

    def __init__(self, hub) -> None:
        self._hub = hub

    async def validate_web_api(self) -> bool:
        if not self._hub._webclient:
            raise FroniusAuthError("Fronius Web API is not configured")
        try:
            valid = await self._hub._hass.async_add_executor_job(self._hub._webclient.login)
        except FroniusWebAuthError as err:
            raise FroniusAuthError(
                "Fronius Web API authentication failed. Reconfigure the integration."
            ) from err
        except RequestException as err:
            raise FroniusConnectionError("Fronius Web API login failed") from err
        if not valid:
            raise FroniusAuthError(
                "Fronius Web API authentication failed. Reconfigure the integration."
            )
        return True

    async def async_apply_modbus_config(self) -> None:
        if not self._hub._webclient or not self._hub._auto_enable_modbus:
            return

        enabled = await self.async_web_job(
            self._hub._webclient.ensure_modbus_enabled,
            self._hub._port,
            self._hub.primary_meter_unit_id,
            self._hub._inverter_unit_id,
            self._hub._restrict_modbus_to_this_ip,
            raise_on_auth_failure=True,
        )
        if enabled:
            await asyncio.sleep(1.0)

    def _clear_web_api_data(self) -> None:
        for key in WEB_API_DATA_KEYS:
            self._hub.data[key] = None

    def _schedule_delayed_web_refresh(self) -> None:
        if (
            self._hub._delayed_web_refresh_task
            and not self._hub._delayed_web_refresh_task.done()
        ):
            self._hub._delayed_web_refresh_task.cancel()

        async def delayed_refresh() -> None:
            try:
                await asyncio.sleep(BATTERY_WRITE_WEB_REFRESH_DELAY_SECONDS)
                if self._hub._webclient:
                    await self.refresh_web_data()
                    self._hub._publish_data_update()
            except asyncio.CancelledError:
                raise
            except FroniusError as err:
                _LOGGER.warning("Delayed Fronius web API refresh failed: %s", err)

        create_task = getattr(self._hub._hass, "async_create_task", None)
        if callable(create_task):
            self._hub._delayed_web_refresh_task = create_task(delayed_refresh())
        else:
            self._hub._delayed_web_refresh_task = asyncio.create_task(delayed_refresh())

    def start_battery_write_transition(self, source: str) -> None:
        """Enter the post-write recovery window and defer web refresh."""
        self._hub._battery_write_transition_until = (
            time.monotonic() + BATTERY_WRITE_MODBUS_RECOVERY_SECONDS
        )
        self._hub._battery_write_transition_warned = False
        self._hub._client.close()
        self._schedule_delayed_web_refresh()
        _LOGGER.debug(
            "Started Modbus recovery window after %s write for %s",
            source,
            self._hub._host,
        )

    async def _async_handle_web_api_auth_failure(self, err: Exception) -> None:
        if not self._hub._webclient:
            return

        _LOGGER.warning(
            "Disabling Fronius web API for %s after auth failure: %s",
            self._hub._host,
            err,
        )
        self._hub._webclient = None
        self._clear_web_api_data()
        await async_get_token_store(self._hub._hass).async_delete_token(
            self._hub._host,
            API_USERNAME,
        )
        await self._hub.warning_service.async_sync_solar_api_warning()

        if self._hub._config_entry is not None:
            ir.async_create_issue(
                self._hub._hass,
                DOMAIN,
                f"{MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX}{self._hub._config_entry.entry_id}",
                is_fixable=True,
                is_persistent=True,
                severity=ir.IssueSeverity.WARNING,
                translation_key="legacy_modbus_only_entry_reconfigure",
                translation_placeholders={
                    "entry_title": self._hub._config_entry.title or self._hub._name
                },
                data={"entry_id": self._hub._config_entry.entry_id},
            )

    async def async_web_job(
        self,
        func,
        *args,
        raise_on_auth_failure: bool = False,
    ):
        """Run a web-client call and fold auth failures into hub state."""
        if not self._hub._webclient:
            if raise_on_auth_failure:
                raise FroniusAuthError("Fronius Web API is not configured")
            return None

        try:
            return await self._hub._hass.async_add_executor_job(func, *args)
        except FroniusWebAuthError as err:
            await self._async_handle_web_api_auth_failure(err)
            if raise_on_auth_failure:
                raise FroniusAuthError(
                    "Fronius Web API authentication failed. Reconfigure the integration."
                ) from err
            return None
        except RequestException as err:
            raise FroniusConnectionError("Fronius Web API request failed") from err

    async def _async_optional_web_read(self, label: str, func, *args):
        try:
            return await self.async_web_job(func, *args)
        except FroniusReadError as err:
            _LOGGER.warning(
                "Ignoring invalid Fronius %s web payload from %s: %s",
                label,
                self._hub._host,
                err,
            )
            return None

    def _apply_web_battery_config(self, battery_config: dict[str, Any]) -> None:
        raw_mode = self._hub._as_int(battery_config.get("HYB_EM_MODE"))
        raw_power = self._hub._as_int(battery_config.get("HYB_EM_POWER"))

        raw_soc_mode = battery_config.get("BAT_M0_SOC_MODE")
        raw_soc_mode = raw_soc_mode.lower() if isinstance(raw_soc_mode, str) else None

        effective_mode = self._hub._derive_api_battery_mode(raw_mode, raw_soc_mode)
        self._hub._set_effective_api_battery_mode(raw_mode, raw_soc_mode)
        self._hub.web_state.set(
            "api_battery_power",
            -raw_power if raw_power is not None else None,
        )
        api_soc_min = self._hub._as_int(battery_config.get("BAT_M0_SOC_MIN"))
        self._hub.web_state.set("api_soc_min", api_soc_min)
        self._hub.web_state.set(
            "soc_maximum",
            self._hub._as_int(battery_config.get("BAT_M0_SOC_MAX")),
        )
        self._hub.web_state.set(
            "api_backup_reserved",
            self._hub._as_int(battery_config.get("HYB_BACKUP_RESERVED")),
        )
        if effective_mode == 1 and api_soc_min is not None:
            self._hub.storage_state.set("soc_minimum", api_soc_min)
        self._hub.web_state.set(
            "api_charge_from_ac",
            self._hub._enabled_bool(battery_config.get("HYB_BM_CHARGEFROMAC")),
        )
        self._hub.web_state.set(
            "api_charge_from_grid",
            self._hub._enabled_bool(battery_config.get("HYB_EVU_CHARGEFROMGRID")),
        )

    def _apply_web_modbus_config(self, modbus_config: dict[str, Any]) -> None:
        slave = modbus_config.get("slave") or {}
        ctr = slave.get("ctr") or {}
        restriction = ctr.get("restriction") or {}
        mode = slave.get("mode")

        self._hub.web_state.set(
            "api_modbus_mode",
            str(mode).upper() if mode is not None else None,
        )
        self._hub.web_state.set(
            "api_modbus_control",
            self._hub._enabled_state(ctr.get("on")),
        )
        self._hub.web_state.set("api_modbus_sunspec_mode", slave.get("sunspecMode"))
        self._hub.web_state.set(
            "api_modbus_restriction",
            self._hub._enabled_state(restriction.get("on")),
        )
        self._hub.web_state.set("api_modbus_restriction_ip", restriction.get("ip"))

    async def refresh_web_data(self) -> None:
        """Refresh cached Web API state without changing device settings."""
        if not self._hub._webclient:
            return

        inverter_info = await self._async_optional_web_read(
            "inverter readable data",
            self._hub._webclient.get_inverter_info,
        )
        if isinstance(inverter_info, dict):
            self._hub.web_state.set(
                "inverter_temperature",
                inverter_info.get("temperature"),
            )
        else:
            self._hub.web_state.set("inverter_temperature", None)

        modbus_config = await self.async_web_job(self._hub._webclient.get_modbus_config)
        if isinstance(modbus_config, dict):
            self._apply_web_modbus_config(modbus_config)

        solar_api_config = await self.async_web_job(
            self._hub._webclient.get_solar_api_config
        )
        if isinstance(solar_api_config, dict):
            enabled = solar_api_config.get("SolarAPIv1Enabled")
            self._hub.web_state.set(
                "api_solar_api_enabled",
                self._hub._enabled_bool(enabled) if enabled is not None else None
            )
        else:
            self._hub.web_state.set("api_solar_api_enabled", None)

        if self._hub.storage_configured:
            storage_info = await self._async_optional_web_read(
                "storage readable data",
                self._hub._webclient.get_storage_info,
            )
            if isinstance(storage_info, dict):
                self._hub.storage_state.set(
                    "storage_temperature",
                    storage_info.get("cell_temperature"),
                )
            else:
                self._hub.storage_state.set("storage_temperature", None)

            battery_config = await self.async_web_job(
                self._hub._webclient.get_battery_config
            )
            if isinstance(battery_config, dict):
                self._apply_web_battery_config(battery_config)

        await self._hub.warning_service.async_sync_solar_api_warning()

    @toggle_busy
    async def set_solar_api_enabled(self, enabled: bool) -> None:
        if not self._hub._webclient:
            return

        await self.async_web_job(
            self._hub._webclient.set_solar_api_enabled,
            enabled,
            raise_on_auth_failure=True,
        )
        await self.refresh_web_data()
        self._hub._publish_data_update()

    @toggle_busy
    async def reset_modbus_control(self) -> None:
        if not self._hub._webclient:
            return

        await self.async_web_job(
            self._hub._webclient.reset_modbus_control,
            raise_on_auth_failure=True,
        )
        await self.refresh_web_data()
        self._hub._publish_data_update()
