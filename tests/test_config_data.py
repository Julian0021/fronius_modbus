from __future__ import annotations

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST

from custom_components.fronius_modbus.config_data import (
    entry_value,
    form_setting_defaults,
    merged_entry_config,
    sanitize_config_payload,
)
from custom_components.fronius_modbus.const import (
    API_USERNAME,
    CONF_API_PASSWORD,
    CONF_INVERTER_UNIT_ID,
    CONF_METER_UNIT_ID,
    CONF_METER_UNIT_IDS,
    CONF_RECONFIGURE_REQUIRED,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_INVERTER_UNIT_ID,
    DEFAULT_PORT,
    DEFAULT_SCAN_INTERVAL,
    STORAGE_SENSOR_TYPES,
)


def test_sanitize_config_payload_keeps_password_and_removes_legacy_meter_keys() -> None:
    payload = sanitize_config_payload(
        {
            CONF_HOST: "inverter.local",
            CONF_API_PASSWORD: "secret",
            CONF_METER_UNIT_ID: 240,
            CONF_METER_UNIT_IDS: [240, 241],
            CONF_RECONFIGURE_REQUIRED: True,
        }
    )

    assert payload[CONF_HOST] == "inverter.local"
    assert payload[CONF_API_PASSWORD] == "secret"
    assert payload["api_username"] == API_USERNAME
    assert CONF_METER_UNIT_ID not in payload
    assert CONF_METER_UNIT_IDS not in payload
    assert payload[CONF_INVERTER_UNIT_ID] == DEFAULT_INVERTER_UNIT_ID
    assert payload["port"] == DEFAULT_PORT
    assert payload["scan_interval"] == DEFAULT_SCAN_INTERVAL


def test_sanitize_config_payload_drops_unknown_keys_and_preserves_supported_ones() -> None:
    payload = sanitize_config_payload(
        {
            CONF_HOST: "inverter.local",
            CONF_API_PASSWORD: "secret",
            "scan_intervall": 42,
            "typoed_flag": True,
        }
    )

    assert payload[CONF_HOST] == "inverter.local"
    assert payload[CONF_API_PASSWORD] == "secret"
    assert payload["scan_interval"] == DEFAULT_SCAN_INTERVAL
    assert "scan_intervall" not in payload
    assert "typoed_flag" not in payload


def test_entry_value_prefers_options_and_form_defaults_only_exposes_user_owned_fields() -> None:
    entry = ConfigEntry(
        data={
            CONF_HOST: "data-host",
            "scan_interval": 10,
            CONF_RESTRICT_MODBUS_TO_THIS_IP: False,
        },
        options={
            CONF_HOST: "options-host",
            "scan_interval": 15,
            CONF_RESTRICT_MODBUS_TO_THIS_IP: True,
        },
    )

    assert entry_value(entry, CONF_HOST) == "options-host"
    assert form_setting_defaults(merged_entry_config(entry)) == {
        CONF_HOST: "options-host",
        "scan_interval": 15,
        CONF_RESTRICT_MODBUS_TO_THIS_IP: True,
    }


def test_storage_charge_status_descriptor_keeps_diagnostic_category() -> None:
    assert STORAGE_SENSOR_TYPES["charge_status"].entity_category == "diagnostic"
