from __future__ import annotations

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
import logging
from typing import Any

from homeassistant import config_entries
from homeassistant.const import CONF_HOST, CONF_NAME, CONF_SCAN_INTERVAL
from homeassistant.exceptions import HomeAssistantError
from homeassistant.helpers.selector import (
    TextSelector,
    TextSelectorConfig,
    TextSelectorType,
)
import voluptuous as vol

from .config_data import merged_entry_config
from .const import (
    CONF_API_PASSWORD,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_NAME,
    DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_SCAN_INTERVAL,
)
from .flow_helpers import (
    AddressesNotUnique,
    CannotConnect,
    CannotResolveLocalIp,
    InvalidApiCredentials,
    InvalidHost,
    InvalidPort,
    MissingApiPassword,
    ScanIntervalTooShort,
    UnsupportedHardware,
    async_load_token,
    async_mint_token,
    async_save_token,
    expand_settings_input,
    should_apply_modbus_config,
    validate_input,
    validate_static_input,
)

_LOGGER = logging.getLogger(__name__)

_FlowFinishCallback = Callable[
    [dict[str, Any], dict[str, Any], str | None, bool],
    Awaitable[Any],
]
_FlowRestartCallback = Callable[[], Awaitable[Any]]


@dataclass(slots=True)
class _PendingFlowState:
    """Transient config-flow state carried from settings to password validation."""

    settings: dict[str, Any]
    previous_host: str | None
    apply_modbus_config: bool


def entry_defaults(entry: config_entries.ConfigEntry) -> dict[str, Any]:
    defaults = merged_entry_config(entry)
    try:
        defaults[CONF_SCAN_INTERVAL] = int(
            defaults.get(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL)
        )
    except (TypeError, ValueError):
        defaults[CONF_SCAN_INTERVAL] = DEFAULT_SCAN_INTERVAL
    return defaults


def _build_settings_schema(defaults: dict[str, Any]) -> vol.Schema:
    return vol.Schema(
        {
            vol.Required(
                CONF_NAME,
                default=defaults.get(CONF_NAME, DEFAULT_NAME),
            ): str,
            vol.Required(CONF_HOST, default=defaults.get(CONF_HOST, "")): str,
            vol.Required(
                CONF_SCAN_INTERVAL,
                default=defaults.get(CONF_SCAN_INTERVAL, DEFAULT_SCAN_INTERVAL),
            ): vol.Coerce(int),
            vol.Required(
                CONF_RESTRICT_MODBUS_TO_THIS_IP,
                default=defaults.get(
                    CONF_RESTRICT_MODBUS_TO_THIS_IP,
                    DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
                ),
            ): bool,
        }
    )


def _build_password_schema() -> vol.Schema:
    return vol.Schema(
        {
            vol.Required(CONF_API_PASSWORD): TextSelector(
                TextSelectorConfig(
                    type=TextSelectorType.PASSWORD,
                    autocomplete="current-password",
                )
            )
        }
    )


def _set_form_error(errors: dict[str, str], err: Exception) -> None:
    if isinstance(err, CannotConnect):
        errors["base"] = "cannot_connect"
    elif isinstance(err, InvalidPort):
        errors["base"] = "invalid_port"
    elif isinstance(err, InvalidHost):
        errors["host"] = "invalid_host"
    elif isinstance(err, ScanIntervalTooShort):
        errors["base"] = "scan_interval_too_short"
    elif isinstance(err, MissingApiPassword):
        errors["base"] = "missing_api_password"
    elif isinstance(err, InvalidApiCredentials):
        errors["base"] = "invalid_api_credentials"
    elif isinstance(err, CannotResolveLocalIp):
        errors["base"] = "cannot_resolve_local_ip"
    elif isinstance(err, UnsupportedHardware):
        errors["base"] = "unsupported_hardware"
    elif isinstance(err, AddressesNotUnique):
        errors["base"] = "modbus_address_conflict"
    else:
        _LOGGER.exception("Unexpected exception")
        errors["base"] = "unknown"


def _prepare_settings_step_state(
    user_input: dict[str, Any],
    *,
    base_settings: dict[str, Any],
    previous_settings: dict[str, Any] | None,
    force_apply_modbus_config: bool,
) -> tuple[dict[str, Any], bool]:
    """Normalize settings input before any token-backed validation runs."""
    settings = expand_settings_input(user_input, base_settings)
    apply_modbus_config = force_apply_modbus_config or should_apply_modbus_config(
        settings,
        previous_settings,
    )
    validate_static_input(settings)
    return settings, apply_modbus_config


class TokenFlowMixin:
    """Shared token-backed settings/password steps for config, options, and repairs."""

    _pending_flow_state: _PendingFlowState | None = None

    async def _async_show_password_step(
        self,
        *,
        step_id: str,
        errors: dict[str, str] | None = None,
    ):
        return self.async_show_form(
            step_id=step_id,
            data_schema=_build_password_schema(),
            errors=errors or {},
        )

    async def _async_handle_settings_step(
        self,
        *,
        user_input: dict[str, Any] | None,
        step_id: str,
        password_step_id: str,
        form_defaults: dict[str, Any],
        base_settings: dict[str, Any],
        previous_host: str | None,
        previous_settings: dict[str, Any] | None,
        force_apply_modbus_config: bool = False,
        on_success: _FlowFinishCallback,
    ):
        """Validate the settings step and branch to password capture when a token is missing."""
        errors: dict[str, str] = {}

        if user_input is not None:
            try:
                settings, apply_modbus_config = _prepare_settings_step_state(
                    user_input,
                    base_settings=base_settings,
                    previous_settings=previous_settings,
                    force_apply_modbus_config=force_apply_modbus_config,
                )
            except HomeAssistantError as err:
                _set_form_error(errors, err)
            else:
                try:
                    token = await async_load_token(self.hass, settings[CONF_HOST])
                    if token is None:
                        self._pending_flow_state = _PendingFlowState(
                            settings,
                            previous_host,
                            apply_modbus_config,
                        )
                        return await self._async_show_password_step(
                            step_id=password_step_id
                        )

                    info = await validate_input(
                        self.hass,
                        settings,
                        api_token=token,
                        allow_unconfigured_modbus=apply_modbus_config,
                    )
                    self._pending_flow_state = None
                    return await on_success(
                        settings,
                        info,
                        previous_host,
                        apply_modbus_config,
                    )
                except InvalidApiCredentials:
                    self._pending_flow_state = _PendingFlowState(
                        settings,
                        previous_host,
                        apply_modbus_config,
                    )
                    return await self._async_show_password_step(step_id=password_step_id)
                except HomeAssistantError as err:
                    _set_form_error(errors, err)

        return self.async_show_form(
            step_id=step_id,
            data_schema=_build_settings_schema(form_defaults),
            errors=errors,
        )

    async def _async_handle_password_step(
        self,
        *,
        user_input: dict[str, Any] | None,
        step_id: str,
        restart_step: _FlowRestartCallback,
        on_success: _FlowFinishCallback,
    ):
        """Mint and persist a token before resuming the pending settings flow."""
        errors: dict[str, str] = {}
        state = self._pending_flow_state
        if state is None:
            return await restart_step()

        if user_input is not None:
            try:
                token = await async_mint_token(
                    self.hass,
                    state.settings[CONF_HOST],
                    user_input.get(CONF_API_PASSWORD, ""),
                )
                await async_save_token(self.hass, state.settings[CONF_HOST], token)
                info = await validate_input(
                    self.hass,
                    state.settings,
                    api_token=token,
                    allow_unconfigured_modbus=state.apply_modbus_config,
                )
                self._pending_flow_state = None
                return await on_success(
                    state.settings,
                    info,
                    state.previous_host,
                    state.apply_modbus_config,
                )
            except HomeAssistantError as err:
                _set_form_error(errors, err)

        return await self._async_show_password_step(step_id=step_id, errors=errors)
