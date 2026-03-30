from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from homeassistant.config_entries import ConfigEntry
from homeassistant.const import CONF_HOST, CONF_NAME, CONF_PORT, CONF_SCAN_INTERVAL
import voluptuous as vol

from .const import (
    API_USERNAME,
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


def _normalize_name(value: Any) -> str:
    return str(value).strip() or DEFAULT_NAME


def _normalize_host(value: Any) -> str:
    return str(value).strip()


def _normalize_int(value: Any) -> int:
    return int(value)


def _normalize_bool(value: Any) -> bool:
    return bool(value)


@dataclass(frozen=True, slots=True)
class EditableSettingSpec:
    key: str
    default: Any
    validator: Any
    normalize: Any


EDITABLE_SETTING_SPECS: tuple[EditableSettingSpec, ...] = (
    EditableSettingSpec(CONF_NAME, DEFAULT_NAME, str, _normalize_name),
    EditableSettingSpec(CONF_HOST, "", str, _normalize_host),
    EditableSettingSpec(
        CONF_SCAN_INTERVAL,
        DEFAULT_SCAN_INTERVAL,
        vol.Coerce(int),
        _normalize_int,
    ),
    EditableSettingSpec(
        CONF_RESTRICT_MODBUS_TO_THIS_IP,
        DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
        bool,
        _normalize_bool,
    ),
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

SUPPORTED_CONFIG_KEYS = frozenset(_DEFAULT_CONFIG_PAYLOAD)


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
    return {
        spec.key: runtime_config.get(spec.key, spec.default)
        for spec in EDITABLE_SETTING_SPECS
    }


def expand_editable_settings(
    user_input: dict[str, Any],
    base_payload: dict[str, Any] | None,
) -> dict[str, Any]:
    """Overlay editable settings onto a normalized payload."""
    payload = sanitize_config_payload(base_payload)
    for spec in EDITABLE_SETTING_SPECS:
        payload[spec.key] = spec.normalize(
            user_input.get(spec.key, payload.get(spec.key, spec.default))
        )
    return sanitize_config_payload(payload)


def editable_settings_schema(defaults: dict[str, Any]) -> vol.Schema:
    """Build the shared settings form schema from the editable-setting contract."""
    return vol.Schema(
        {
            vol.Required(
                spec.key,
                default=defaults.get(spec.key, spec.default),
            ): spec.validator
            for spec in EDITABLE_SETTING_SPECS
        }
    )
