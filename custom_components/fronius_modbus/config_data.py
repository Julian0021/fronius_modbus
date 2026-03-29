from __future__ import annotations

from typing import Any

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST, CONF_NAME, CONF_PORT, CONF_SCAN_INTERVAL

from .const import (
    API_USERNAME,
    CONF_API_PASSWORD,
    CONF_API_USERNAME,
    CONF_AUTO_ENABLE_MODBUS,
    CONF_INVERTER_UNIT_ID,
    CONF_METER_UNIT_ID,
    CONF_METER_UNIT_IDS,
    CONF_RECONFIGURE_REQUIRED,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_AUTO_ENABLE_MODBUS,
    DEFAULT_INVERTER_UNIT_ID,
    DEFAULT_NAME,
    DEFAULT_PORT,
    DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_SCAN_INTERVAL,
)

LEGACY_CONFIG_KEYS = (
    CONF_METER_UNIT_ID,
    CONF_METER_UNIT_IDS,
)

FORM_SETTING_KEYS = (
    CONF_HOST,
    CONF_SCAN_INTERVAL,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
)

_DEFAULT_CONFIG_PAYLOAD = {
    CONF_NAME: DEFAULT_NAME,
    CONF_HOST: "",
    CONF_PORT: DEFAULT_PORT,
    CONF_INVERTER_UNIT_ID: DEFAULT_INVERTER_UNIT_ID,
    CONF_SCAN_INTERVAL: DEFAULT_SCAN_INTERVAL,
    CONF_API_USERNAME: API_USERNAME,
    CONF_AUTO_ENABLE_MODBUS: DEFAULT_AUTO_ENABLE_MODBUS,
    CONF_RESTRICT_MODBUS_TO_THIS_IP: DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
    CONF_RECONFIGURE_REQUIRED: False,
}

SUPPORTED_CONFIG_KEYS = frozenset((*_DEFAULT_CONFIG_PAYLOAD, CONF_API_PASSWORD))


def default_config_payload() -> dict[str, Any]:
    """Return the canonical config shape used by flows and migrations."""
    return dict(_DEFAULT_CONFIG_PAYLOAD)


def sanitize_config_payload(
    payload: dict[str, Any] | None,
    *,
    reconfigure_required: bool | None = None,
) -> dict[str, Any]:
    """Normalize stored config and strip legacy fields the runtime no longer owns."""
    sanitized = default_config_payload()
    if payload:
        for key, value in payload.items():
            if key in LEGACY_CONFIG_KEYS or key not in SUPPORTED_CONFIG_KEYS:
                continue
            sanitized[key] = value

    sanitized[CONF_API_USERNAME] = API_USERNAME
    if reconfigure_required is not None:
        sanitized[CONF_RECONFIGURE_REQUIRED] = bool(reconfigure_required)
    else:
        sanitized[CONF_RECONFIGURE_REQUIRED] = bool(
            sanitized.get(CONF_RECONFIGURE_REQUIRED, False)
        )
    return sanitized


def merged_entry_config(entry: ConfigEntry) -> dict[str, Any]:
    """Merge config-entry data and options into one normalized runtime payload."""
    return sanitize_config_payload({**entry.data, **entry.options})


def entry_value(entry: ConfigEntry, key: str, default=None):
    """Read a normalized config value with options taking precedence over data."""
    return merged_entry_config(entry).get(key, default)


def form_setting_defaults(config: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return only the settings owned by the config and options forms."""
    runtime_config = sanitize_config_payload(config)
    return {key: runtime_config[key] for key in FORM_SETTING_KEYS}
