from __future__ import annotations

import logging

from homeassistant import config_entries
from homeassistant.const import CONF_HOST, CONF_NAME
from homeassistant.core import callback

from .config_data import (
    form_setting_defaults,
    sanitize_config_payload,
)
from .const import DOMAIN
from .flow_helpers import (
    async_apply_requested_modbus_config,
    async_update_entry_from_input,
    runtime_defaults,
)
from .flow_steps import TokenFlowMixin, entry_defaults

_LOGGER = logging.getLogger(__name__)


class ConfigFlow(TokenFlowMixin, config_entries.ConfigFlow, domain=DOMAIN):
    """Handle a config flow."""

    VERSION = 1
    MINOR_VERSION = 8
    CONNECTION_CLASS = config_entries.CONN_CLASS_LOCAL_POLL

    def __init__(self) -> None:
        self._pending_flow_state = None

    @staticmethod
    @callback
    def async_get_options_flow(config_entry):
        return FroniusModbusOptionsFlow()

    async def _async_finish_user(self, settings, info, previous_host, apply_modbus_config):
        del info, previous_host
        await async_apply_requested_modbus_config(
            self.hass,
            settings,
            apply_modbus_config=apply_modbus_config,
        )
        return self.async_create_entry(
            title=settings[CONF_NAME],
            data=sanitize_config_payload(settings, reconfigure_required=False),
        )

    async def _async_finish_reconfigure(self, settings, info, previous_host, apply_modbus_config):
        del info
        await async_apply_requested_modbus_config(
            self.hass,
            settings,
            apply_modbus_config=apply_modbus_config,
        )
        entry = self._get_reconfigure_entry()
        await async_update_entry_from_input(
            self.hass,
            entry,
            settings,
            previous_host=previous_host,
        )
        return self.async_abort(reason="reconfigure_successful")

    async def async_step_user(self, user_input=None):
        default_settings = runtime_defaults()
        return await self._async_handle_settings_step(
            user_input=user_input,
            step_id="user",
            password_step_id="user_password",
            form_defaults=form_setting_defaults(default_settings),
            base_settings=default_settings,
            previous_host=None,
            previous_settings=None,
            force_apply_modbus_config=True,
            on_success=self._async_finish_user,
        )

    async def async_step_user_password(self, user_input=None):
        return await self._async_handle_password_step(
            user_input=user_input,
            step_id="user_password",
            restart_step=self.async_step_user,
            on_success=self._async_finish_user,
        )

    async def async_step_reconfigure(self, user_input=None):
        entry = self._get_reconfigure_entry()
        defaults = entry_defaults(entry)
        return await self._async_handle_settings_step(
            user_input=user_input,
            step_id="reconfigure",
            password_step_id="reconfigure_password",
            form_defaults=form_setting_defaults(defaults),
            base_settings=defaults,
            previous_host=defaults[CONF_HOST],
            previous_settings=defaults,
            force_apply_modbus_config=True,
            on_success=self._async_finish_reconfigure,
        )

    async def async_step_reconfigure_password(self, user_input=None):
        return await self._async_handle_password_step(
            user_input=user_input,
            step_id="reconfigure_password",
            restart_step=self.async_step_reconfigure,
            on_success=self._async_finish_reconfigure,
        )


class FroniusModbusOptionsFlow(TokenFlowMixin, config_entries.OptionsFlow):
    """Handle Fronius Modbus options."""

    async def _async_finish_options(self, settings, info, previous_host, apply_modbus_config):
        del info
        await async_apply_requested_modbus_config(
            self.hass,
            settings,
            apply_modbus_config=apply_modbus_config,
        )

        await async_update_entry_from_input(
            self.hass,
            self.config_entry,
            settings,
            previous_host=previous_host,
        )
        return self.async_create_entry(title="", data={})

    async def async_step_init(self, user_input=None):
        defaults = entry_defaults(self.config_entry)
        return await self._async_handle_settings_step(
            user_input=user_input,
            step_id="init",
            password_step_id="password",
            form_defaults=form_setting_defaults(defaults),
            base_settings=defaults,
            previous_host=defaults[CONF_HOST],
            previous_settings=defaults,
            on_success=self._async_finish_options,
        )

    async def async_step_password(self, user_input=None):
        return await self._async_handle_password_step(
            user_input=user_input,
            step_id="password",
            restart_step=self.async_step_init,
            on_success=self._async_finish_options,
        )
