from __future__ import annotations

import logging
from typing import Any

from homeassistant import config_entries, exceptions
from homeassistant.const import CONF_HOST, CONF_NAME, CONF_PORT, CONF_SCAN_INTERVAL
from homeassistant.core import HomeAssistant
from requests import RequestException

from .config_data import default_config_payload, sanitize_config_payload
from .const import (
    API_USERNAME,
    CONF_AUTO_ENABLE_MODBUS,
    CONF_INVERTER_UNIT_ID,
    CONF_RESTRICT_MODBUS_TO_THIS_IP,
    DEFAULT_AUTO_ENABLE_MODBUS,
    DEFAULT_INVERTER_UNIT_ID,
    DEFAULT_METER_UNIT_IDS,
    DEFAULT_NAME,
    DEFAULT_PORT,
    DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
    SUPPORTED_MANUFACTURERS,
    SUPPORTED_MODELS,
)
from .froniuswebclient import ClientIpResolutionError, FroniusWebAuthError, mint_token
from .hub import Hub
from .integration_errors import FroniusAuthError, FroniusError
from .token_store import async_get_token_store

_LOGGER = logging.getLogger(__name__)


class CannotConnect(exceptions.HomeAssistantError):
    """Raised when the hub cannot be reached."""


class InvalidHost(exceptions.HomeAssistantError):
    """Raised when the host value is invalid."""


class InvalidPort(exceptions.HomeAssistantError):
    """Raised when the port value is invalid."""


class UnsupportedHardware(exceptions.HomeAssistantError):
    """Raised when the detected inverter hardware is unsupported."""


class AddressesNotUnique(exceptions.HomeAssistantError):
    """Raised when the configured Modbus addresses collide."""


class ScanIntervalTooShort(exceptions.HomeAssistantError):
    """Raised when the scan interval is too short."""


class MissingApiPassword(exceptions.HomeAssistantError):
    """Raised when the Web API password is required."""


class InvalidApiCredentials(exceptions.HomeAssistantError):
    """Raised when Web API credentials are invalid."""


class CannotResolveLocalIp(exceptions.HomeAssistantError):
    """Raised when the local IP for Modbus restriction cannot be resolved."""


def runtime_defaults(defaults: dict[str, Any] | None = None) -> dict[str, Any]:
    payload = default_config_payload()
    if defaults:
        payload.update(sanitize_config_payload(defaults))
    return payload


def expand_settings_input(
    user_input: dict[str, Any],
    base_settings: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Overlay user input onto runtime defaults and normalize the stored payload."""
    payload = runtime_defaults(base_settings)
    payload[CONF_NAME] = str(user_input.get(CONF_NAME, payload[CONF_NAME])).strip() or DEFAULT_NAME
    payload[CONF_HOST] = str(user_input.get(CONF_HOST, payload[CONF_HOST])).strip()
    payload[CONF_SCAN_INTERVAL] = int(
        user_input.get(CONF_SCAN_INTERVAL, payload[CONF_SCAN_INTERVAL])
    )
    payload[CONF_RESTRICT_MODBUS_TO_THIS_IP] = bool(
        user_input.get(
            CONF_RESTRICT_MODBUS_TO_THIS_IP,
            payload[CONF_RESTRICT_MODBUS_TO_THIS_IP],
        )
    )
    return sanitize_config_payload(payload)


def validate_static_input(data: dict[str, Any]) -> None:
    if len(data[CONF_HOST]) < 3:
        raise InvalidHost
    if data[CONF_PORT] > 65535:
        raise InvalidPort
    if data[CONF_SCAN_INTERVAL] < 5:
        raise ScanIntervalTooShort

    all_addresses = [DEFAULT_METER_UNIT_IDS[0], data[CONF_INVERTER_UNIT_ID]]
    if len(all_addresses) > len(set(all_addresses)):
        _LOGGER.error("Modbus addresses are not unique %s", all_addresses)
        raise AddressesNotUnique


def should_apply_modbus_config(
    settings: dict[str, Any],
    previous_settings: dict[str, Any] | None,
) -> bool:
    """Return whether the new settings require reapplying inverter Modbus config."""
    if previous_settings is None:
        return True

    return (
        settings[CONF_HOST] != previous_settings.get(CONF_HOST, "")
        or settings[CONF_PORT] != previous_settings.get(CONF_PORT, DEFAULT_PORT)
        or settings[CONF_INVERTER_UNIT_ID]
        != previous_settings.get(CONF_INVERTER_UNIT_ID, DEFAULT_INVERTER_UNIT_ID)
        or settings[CONF_RESTRICT_MODBUS_TO_THIS_IP]
        != previous_settings.get(
            CONF_RESTRICT_MODBUS_TO_THIS_IP,
            DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
        )
    )


async def async_load_token(hass: HomeAssistant, host: str) -> dict[str, str] | None:
    return await async_get_token_store(hass).async_load_token(host, API_USERNAME)


async def async_save_token(hass: HomeAssistant, host: str, token: dict[str, str]) -> None:
    await async_get_token_store(hass).async_save_token(
        host,
        realm=token["realm"],
        token=token["token"],
        user=API_USERNAME,
    )


async def async_delete_token(hass: HomeAssistant, host: str | None) -> None:
    if host:
        await async_get_token_store(hass).async_delete_token(host, API_USERNAME)


async def async_mint_token(
    hass: HomeAssistant,
    host: str,
    password: str,
) -> dict[str, str]:
    password = str(password).strip()
    if password == "":
        raise MissingApiPassword

    try:
        token = await hass.async_add_executor_job(
            mint_token,
            host,
            API_USERNAME,
            password,
        )
    except FroniusWebAuthError as err:
        raise InvalidApiCredentials from err
    except RequestException as err:
        raise CannotConnect from err

    if not token:
        raise InvalidApiCredentials
    return token


def build_hub(
    hass: HomeAssistant,
    data: dict[str, Any],
    *,
    api_password: str = "",
    api_token: dict[str, str] | None = None,
) -> Hub:
    return Hub(
        hass,
        data[CONF_NAME],
        data[CONF_HOST],
        data[CONF_PORT],
        data[CONF_INVERTER_UNIT_ID],
        list(DEFAULT_METER_UNIT_IDS),
        data[CONF_SCAN_INTERVAL],
        api_username=API_USERNAME,
        api_password=api_password or None,
        api_token=api_token,
        auto_enable_modbus=data.get(CONF_AUTO_ENABLE_MODBUS, DEFAULT_AUTO_ENABLE_MODBUS),
        restrict_modbus_to_this_ip=data.get(
            CONF_RESTRICT_MODBUS_TO_THIS_IP,
            DEFAULT_RESTRICT_MODBUS_TO_THIS_IP,
        ),
    )


def validate_detected_hardware(runtime_hub: Hub) -> None:
    manufacturer = runtime_hub.inverter_state.get("i_manufacturer")
    if manufacturer is None:
        _LOGGER.error("No manufacturer is returned")
        raise UnsupportedHardware
    if manufacturer not in SUPPORTED_MANUFACTURERS:
        _LOGGER.error("Unsupported manufacturer: %r", manufacturer)
        raise UnsupportedHardware

    model = runtime_hub.inverter_state.get("i_model")
    if model is None:
        _LOGGER.error("No model type is returned")
        raise UnsupportedHardware

    if not any(model.startswith(supported_model) for supported_model in SUPPORTED_MODELS):
        _LOGGER.warning("Untested model %s", model)


async def _async_validate_runtime_hub(
    runtime_hub: Hub,
    *,
    host: str,
    apply_modbus_config: bool,
    allow_unconfigured_modbus: bool,
) -> None:
    """Validate the Web API and Modbus topology for a prepared hub."""
    try:
        await runtime_hub.web_api_service.validate_web_api()
        try:
            await runtime_hub.bootstrap_service.init_data(
                setup_coordinator=False,
                apply_modbus_config=apply_modbus_config,
            )
        except FroniusError:
            if not allow_unconfigured_modbus or apply_modbus_config:
                raise
            _LOGGER.info(
                "Skipping full Modbus probe during validation for %s until the explicit apply step",
                host,
            )
        else:
            validate_detected_hardware(runtime_hub)
    except ClientIpResolutionError as err:
        raise CannotResolveLocalIp from err
    except FroniusAuthError as err:
        raise InvalidApiCredentials from err
    except FroniusError as err:
        _LOGGER.error(
            "Cannot %s %s",
            "apply Modbus configuration" if apply_modbus_config else "start hub",
            err,
        )
        raise CannotConnect from err
    finally:
        runtime_hub.close()


async def validate_input(
    hass: HomeAssistant,
    data: dict[str, Any],
    *,
    api_password: str = "",
    api_token: dict[str, str] | None = None,
    allow_unconfigured_modbus: bool = False,
) -> dict[str, Any]:
    """Validate connectivity and hardware, optionally skipping the explicit Modbus apply probe."""
    validate_static_input(data)

    if not api_password and api_token is None:
        raise MissingApiPassword

    runtime_hub = build_hub(
        hass,
        data,
        api_password=api_password,
        api_token=api_token,
    )
    await _async_validate_runtime_hub(
        runtime_hub,
        host=data[CONF_HOST],
        apply_modbus_config=False,
        allow_unconfigured_modbus=allow_unconfigured_modbus,
    )

    return {"title": data[CONF_NAME]}


async def async_apply_modbus_config_and_validate(
    hass: HomeAssistant,
    settings: dict[str, Any],
    *,
    api_token: dict[str, str],
) -> None:
    """Apply Modbus settings and revalidate against the live inverter."""
    runtime_hub = build_hub(hass, settings, api_token=api_token)
    await _async_validate_runtime_hub(
        runtime_hub,
        host=settings[CONF_HOST],
        apply_modbus_config=True,
        allow_unconfigured_modbus=False,
    )


async def async_apply_requested_modbus_config(
    hass: HomeAssistant,
    settings: dict[str, Any],
    *,
    apply_modbus_config: bool,
) -> None:
    if not apply_modbus_config:
        return

    token = await async_load_token(hass, settings[CONF_HOST])
    if token is None:
        raise InvalidApiCredentials
    await async_apply_modbus_config_and_validate(
        hass,
        settings,
        api_token=token,
    )


async def async_update_entry_from_input(
    hass: HomeAssistant,
    entry: config_entries.ConfigEntry,
    validated_input: dict[str, Any],
    *,
    previous_host: str | None = None,
) -> None:
    """Persist normalized config input, clear stale options, and reload the entry."""
    updated_payload = sanitize_config_payload(
        validated_input,
        reconfigure_required=False,
    )
    hass.config_entries.async_update_entry(
        entry,
        data=updated_payload,
        options={},
        title=validated_input[CONF_NAME],
    )
    if previous_host and previous_host != validated_input[CONF_HOST]:
        await async_delete_token(hass, previous_host)
    await hass.config_entries.async_reload(entry.entry_id)
