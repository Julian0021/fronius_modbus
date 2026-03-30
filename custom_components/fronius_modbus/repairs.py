from __future__ import annotations

from typing import Any

from homeassistant.components.repairs import RepairsFlow
from homeassistant.helpers import issue_registry as ir
import voluptuous as vol

from .config_data import form_setting_defaults
from .const import (
    DOMAIN,
    MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX,
    SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX,
)
from .flow_helpers import (
    async_apply_requested_modbus_config,
    async_update_entry_from_input,
)
from .flow_steps import TokenFlowMixin, entry_defaults
from .integration_errors import (
    FroniusAuthError,
    FroniusConnectionError,
    FroniusError,
    FroniusOperationError,
)


def _solar_api_repair_error(err: FroniusError) -> str:
    if isinstance(err, FroniusConnectionError):
        return "cannot_connect"
    if isinstance(err, FroniusAuthError):
        return "invalid_api_credentials"
    if isinstance(err, FroniusOperationError):
        return "cannot_confirm_disable"
    return "unknown"


class FroniusReconfigureRepairFlow(TokenFlowMixin, RepairsFlow):
    """Repair flow that reuses the reconfigure fields and validation."""

    def __init__(self, entry_id: str) -> None:
        self._entry_id = entry_id

    def _issue_id(self) -> str:
        return f"{MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX}{self._entry_id}"

    def _resolve_issue(self) -> None:
        ir.async_delete_issue(self.hass, DOMAIN, self._issue_id())

    async def _async_finish_repair(
        self,
        settings,
        info,
        previous_host,
        apply_modbus_config,
    ):
        """Persist the reconfigure flow result and clear the pending repair issue."""
        del info
        entry = self.hass.config_entries.async_get_entry(self._entry_id)
        if entry is None:
            self._resolve_issue()
            return self.async_create_entry(title="", data={})

        await async_apply_requested_modbus_config(
            self.hass,
            settings,
            apply_modbus_config=apply_modbus_config,
        )

        await async_update_entry_from_input(
            self.hass,
            entry,
            settings,
            previous_host=previous_host,
        )
        self._resolve_issue()
        return self.async_create_entry(title="", data={})

    async def async_step_init(self, user_input: dict[str, Any] | None = None):
        entry = self.hass.config_entries.async_get_entry(self._entry_id)
        if entry is None:
            self._resolve_issue()
            return self.async_create_entry(title="", data={})

        defaults = entry_defaults(entry)
        return await self._async_handle_settings_step(
            user_input=user_input,
            step_id="init",
            password_step_id="password",
            form_defaults=form_setting_defaults(defaults),
            base_settings=defaults,
            previous_host=defaults["host"],
            previous_settings=defaults,
            force_apply_modbus_config=True,
            on_success=self._async_finish_repair,
        )

    async def async_step_password(self, user_input: dict[str, Any] | None = None):
        return await self._async_handle_password_step(
            user_input=user_input,
            step_id="password",
            restart_step=self.async_step_init,
            on_success=self._async_finish_repair,
        )


class FroniusDisableSolarApiRepairFlow(RepairsFlow):
    """Repair flow that disables Solar API on the inverter."""

    def __init__(self, entry_id: str) -> None:
        self._entry_id = entry_id

    def _issue_id(self) -> str:
        return f"{SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX}{self._entry_id}"

    def _resolve_issue(self) -> None:
        ir.async_delete_issue(self.hass, DOMAIN, self._issue_id())

    def _description_placeholders(self) -> dict[str, str] | None:
        """Return issue placeholders from the active Solar API repair issue."""
        issue = ir.async_get(self.hass).async_get_issue(DOMAIN, self._issue_id())
        return issue.translation_placeholders if issue else None

    async def _async_finish_repair(self):
        """Disable Solar API, confirm the new state, and resolve the repair issue."""
        entry = self.hass.config_entries.async_get_entry(self._entry_id)
        if entry is None:
            self._resolve_issue()
            return self.async_create_entry(title="", data={})

        hub = getattr(entry, "runtime_data", None)
        if hub is None or not getattr(hub, "web_api_configured", False):
            raise FroniusOperationError("Fronius Web API is not configured")

        await hub.web_api_service.set_solar_api_enabled(False)
        await hub.web_api_service.refresh_web_data()
        if not hub.web_api_configured or hub.web_state.get("api_solar_api_enabled") is not False:
            raise FroniusOperationError("Solar API disable could not be confirmed")
        if hub.coordinator is not None:
            hub.coordinator.async_set_updated_data(hub.snapshot_data())
        self._resolve_issue()
        return self.async_create_entry(title="", data={})

    async def async_step_init(self, user_input: dict[str, Any] | None = None):
        del user_input
        entry = self.hass.config_entries.async_get_entry(self._entry_id)
        if entry is None:
            self._resolve_issue()
            return self.async_create_entry(title="", data={})

        return self.async_show_menu(
            step_id="init",
            menu_options=["fix", "ignore"],
            description_placeholders=self._description_placeholders(),
        )

    async def async_step_fix(self, user_input: dict[str, Any] | None = None):
        del user_input
        return await self.async_step_confirm()

    async def async_step_ignore(self, user_input: dict[str, Any] | None = None):
        del user_input
        entry = self.hass.config_entries.async_get_entry(self._entry_id)
        if entry is None:
            self._resolve_issue()
            return self.async_create_entry(title="", data={})

        ir.async_ignore_issue(self.hass, DOMAIN, self._issue_id(), True)
        return self.async_abort(reason="issue_ignored")

    async def async_step_confirm(self, user_input: dict[str, Any] | None = None):
        description_placeholders = self._description_placeholders()
        if user_input is None:
            return self.async_show_form(
                step_id="confirm",
                data_schema=vol.Schema({}),
                description_placeholders=description_placeholders,
            )
        try:
            return await self._async_finish_repair()
        except FroniusError as err:
            return self.async_show_form(
                step_id="confirm",
                data_schema=vol.Schema({}),
                description_placeholders=description_placeholders,
                errors={"base": _solar_api_repair_error(err)},
            )


async def async_create_fix_flow(
    hass,
    issue_id: str,
    data: dict[str, Any] | None,
) -> RepairsFlow:
    """Create the appropriate repair flow for the issue type and stored entry data."""
    if issue_id.startswith(SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX):
        entry_id = str((data or {}).get("entry_id") or issue_id.removeprefix(SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX))
        return FroniusDisableSolarApiRepairFlow(entry_id)

    if not issue_id.startswith(MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX):
        raise ValueError(f"Unknown issue: {issue_id}")

    entry_id = str((data or {}).get("entry_id") or issue_id.removeprefix(MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX))
    return FroniusReconfigureRepairFlow(entry_id)
