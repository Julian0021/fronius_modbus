from __future__ import annotations

from unittest.mock import Mock

import pytest
from pytest_homeassistant_custom_component.common import MockConfigEntry

from custom_components.fronius_modbus.config_data import form_setting_defaults
from custom_components.fronius_modbus.const import DOMAIN
from custom_components.fronius_modbus.flow_steps import entry_defaults
from custom_components.fronius_modbus.integration_errors import (
    FroniusAuthError,
    FroniusConnectionError,
    FroniusOperationError,
)
import custom_components.fronius_modbus.repairs as repairs_module
from custom_components.fronius_modbus.repairs import (
    FroniusDisableSolarApiRepairFlow,
    FroniusReconfigureRepairFlow,
)


@pytest.mark.asyncio
async def test_reconfigure_repair_uses_form_defaults(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    flow = FroniusReconfigureRepairFlow("entry-1")
    entry = MockConfigEntry(
        domain=DOMAIN,
        entry_id="entry-1",
        data={"host": "data-host", "scan_interval": 10},
        options={"host": "options-host", "scan_interval": 15},
    )
    entry.add_to_hass(hass)
    flow.hass = hass

    captured: dict[str, object] = {}

    async def _fake_handle_settings_step(**kwargs):
        captured.update(kwargs)
        return {"type": "form"}

    monkeypatch.setattr(flow, "_async_handle_settings_step", _fake_handle_settings_step)

    result = await flow.async_step_init()

    defaults = entry_defaults(entry)
    assert result == {"type": "form"}
    assert captured["form_defaults"] == form_setting_defaults(defaults)
    assert captured["base_settings"] == defaults


@pytest.mark.asyncio
async def test_reconfigure_repair_applies_modbus_config_before_updating_entry(
    hass,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    flow = FroniusReconfigureRepairFlow("entry-1")
    entry = MockConfigEntry(domain=DOMAIN, entry_id="entry-1")
    entry.add_to_hass(hass)
    flow.hass = hass

    calls: dict[str, object] = {}

    async def _fake_apply(_hass, settings, *, apply_modbus_config):
        calls["applied_settings"] = settings
        calls["apply_modbus_config"] = apply_modbus_config

    async def _fake_update(_hass, current_entry, settings, *, previous_host=None):
        calls["updated_entry"] = current_entry
        calls["updated_settings"] = settings
        calls["previous_host"] = previous_host

    monkeypatch.setattr(
        repairs_module,
        "async_apply_requested_modbus_config",
        _fake_apply,
    )
    monkeypatch.setattr(repairs_module, "async_update_entry_from_input", _fake_update)

    resolve_issue = Mock()
    monkeypatch.setattr(flow, "_resolve_issue", resolve_issue)
    monkeypatch.setattr(
        flow,
        "async_create_entry",
        lambda *, title, data: {"type": "create_entry", "title": title, "data": data},
    )

    settings = {"host": "inverter.local"}
    result = await flow._async_finish_repair(
        settings,
        info={},
        previous_host="old-host",
        apply_modbus_config=True,
    )

    assert result == {"type": "create_entry", "title": "", "data": {}}
    assert calls["applied_settings"] == settings
    assert calls["apply_modbus_config"] is True
    assert calls["updated_entry"] is entry
    assert calls["updated_settings"] == settings
    assert calls["previous_host"] == "old-host"
    resolve_issue.assert_called_once_with()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("error", "expected_error"),
    [
        (FroniusConnectionError("offline"), "cannot_connect"),
        (FroniusAuthError("bad token"), "invalid_api_credentials"),
        (
            FroniusOperationError("confirmation failed"),
            "cannot_confirm_disable",
        ),
    ],
)
async def test_disable_solar_api_repair_maps_specific_errors(
    hass,
    monkeypatch: pytest.MonkeyPatch,
    error,
    expected_error: str,
) -> None:
    flow = FroniusDisableSolarApiRepairFlow("entry-1")
    flow.hass = hass

    async def _raise_finish():
        raise error

    monkeypatch.setattr(flow, "_async_finish_repair", _raise_finish)

    result = await flow.async_step_confirm(user_input={})

    assert result["type"] == "form"
    assert result["step_id"] == "confirm"
    assert result["description_placeholders"] is None
    assert result["errors"] == {"base": expected_error}
