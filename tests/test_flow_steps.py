from __future__ import annotations

from typing import Any

from homeassistant.const import (
    CONF_HOST,
    CONF_NAME,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
    CONF_SCAN_INTERVAL,
)

from custom_components.fronius_modbus.config_data import default_config_payload
from custom_components.fronius_modbus.flow_helpers import InvalidApiCredentials
import custom_components.fronius_modbus.flow_steps as flow_steps_module


class _Flow(flow_steps_module.TokenFlowMixin):
    def __init__(self) -> None:
        self.hass = object()
        self._pending_flow_state = None

    def async_show_form(self, **kwargs: Any) -> dict[str, Any]:
        return {"type": "form", **kwargs}

    async def _async_show_password_step(
        self,
        *,
        step_id: str,
        errors: dict[str, str] | None = None,
    ) -> dict[str, Any]:
        return {"type": "form", "step_id": step_id, "errors": errors or {}}


async def _unexpected_success(*_args: Any) -> None:
    raise AssertionError("settings step should not finish successfully")


def _settings(*, host: str) -> dict[str, Any]:
    settings = default_config_payload()
    settings[CONF_NAME] = "Existing Name"
    settings[CONF_HOST] = host
    settings[CONF_SCAN_INTERVAL] = 10
    settings[CONF_RESTRICT_MODBUS_TO_THIS_IP] = False
    return settings


async def test_handle_settings_step_shows_password_step_for_invalid_stored_token(
    monkeypatch,
) -> None:
    flow = _Flow()
    previous_settings = _settings(host="old-host")

    async def _fake_load_token(_hass: object, host: str) -> dict[str, str]:
        assert host == "new-host"
        return {"token": "bad-token", "realm": "Solar API"}

    async def _raise_invalid(*_args: Any, **_kwargs: Any) -> None:
        raise InvalidApiCredentials

    monkeypatch.setattr(flow_steps_module, "async_load_token", _fake_load_token)
    monkeypatch.setattr(flow_steps_module, "validate_input", _raise_invalid)

    result = await flow._async_handle_settings_step(
        user_input={
            CONF_NAME: "New Name",
            CONF_HOST: "new-host",
            CONF_SCAN_INTERVAL: 10,
            CONF_RESTRICT_MODBUS_TO_THIS_IP: False,
        },
        step_id="init",
        password_step_id="password",
        form_defaults={
            CONF_NAME: previous_settings[CONF_NAME],
            CONF_HOST: previous_settings[CONF_HOST],
            CONF_SCAN_INTERVAL: previous_settings[CONF_SCAN_INTERVAL],
            CONF_RESTRICT_MODBUS_TO_THIS_IP: previous_settings[
                CONF_RESTRICT_MODBUS_TO_THIS_IP
            ],
        },
        base_settings=previous_settings,
        previous_host=previous_settings[CONF_HOST],
        previous_settings=previous_settings,
        on_success=_unexpected_success,
    )

    assert result["type"] == "form"
    assert result["step_id"] == "password"
    assert flow._pending_flow_state is not None
    assert flow._pending_flow_state.settings[CONF_NAME] == "New Name"
    assert flow._pending_flow_state.settings[CONF_HOST] == "new-host"
    assert flow._pending_flow_state.previous_host == "old-host"
    assert flow._pending_flow_state.apply_modbus_config is True


async def test_handle_settings_step_does_not_crash_if_prevalidation_raises_invalid_credentials(
    monkeypatch,
) -> None:
    flow = _Flow()
    settings = _settings(host="new-host")

    monkeypatch.setattr(
        flow_steps_module,
        "expand_settings_input",
        lambda _user_input, _base_settings: settings,
    )
    monkeypatch.setattr(
        flow_steps_module,
        "should_apply_modbus_config",
        lambda _settings, _previous_settings: True,
    )

    def _raise_invalid(_settings: dict[str, Any]) -> None:
        raise InvalidApiCredentials

    monkeypatch.setattr(flow_steps_module, "validate_static_input", _raise_invalid)

    result = await flow._async_handle_settings_step(
        user_input={},
        step_id="init",
        password_step_id="password",
        form_defaults={
            CONF_NAME: "Existing Name",
            CONF_HOST: "old-host",
            CONF_SCAN_INTERVAL: 10,
            CONF_RESTRICT_MODBUS_TO_THIS_IP: False,
        },
        base_settings=_settings(host="old-host"),
        previous_host="old-host",
        previous_settings=_settings(host="old-host"),
        on_success=_unexpected_success,
    )

    assert result["type"] == "form"
    assert result["step_id"] == "init"
    assert result["errors"] == {"base": "invalid_api_credentials"}
    assert flow._pending_flow_state is None


def test_build_settings_schema_includes_name_field() -> None:
    schema = flow_steps_module._build_settings_schema(_settings(host="named-host"))

    schema_keys = {marker.schema for marker in schema.schema}

    assert CONF_NAME in schema_keys
