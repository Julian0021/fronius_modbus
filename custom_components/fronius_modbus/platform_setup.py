"""Shared helpers for entity platform setup."""
from __future__ import annotations

from typing import Any

from .storage_modes import storage_mode_supports
from .translation import async_ensure_translation_cache


async def async_platform_context(hass, config_entry):
    """Return the shared hub/coordinator pair for a platform setup."""
    await async_ensure_translation_cache(hass)
    hub = config_entry.runtime_data
    return hub, hub.coordinator


def extend_entities(entities: list, descriptions, factory, *, include: bool = True) -> None:
    """Append entities produced from a descriptor sequence when enabled."""
    if not include:
        return
    entities.extend(factory(description) for description in descriptions)


def entity_description_kwargs(
    *,
    coordinator,
    device_info,
    description,
    key: str | None = None,
    translation_placeholders: dict[str, str] | None = None,
    **extra,
):
    """Build the common constructor kwargs shared by descriptor-backed entities."""
    kwargs = {
        "coordinator": coordinator,
        "device_info": device_info,
        "name": description.translation_key,
        "translation_key": description.translation_key,
        "description": description,
    }

    resolved_key = key if key is not None else getattr(description, "key", None)
    if resolved_key is not None:
        kwargs["key"] = resolved_key

    if translation_placeholders is not None:
        kwargs["translation_placeholders"] = translation_placeholders

    for attr in ("icon", "entity_category", "options"):
        value = getattr(description, attr, None)
        if value is not None:
            kwargs[attr] = value

    kwargs.update(extra)
    return kwargs


async def dispatch_service_action(
    hub,
    service_name: str | None,
    action: str | None,
    *args,
    **kwargs,
):
    """Invoke a named service action declared by an entity descriptor."""
    if action is None:
        return None
    target = hub if service_name is None else getattr(hub, service_name)
    handler = getattr(target, action)
    return await handler(*args, **kwargs)


async def dispatch_hub_action(
    hub,
    action: str | None,
    *args,
    service_name: str | None = None,
    **kwargs,
):
    """Backward-compatible wrapper around service-based descriptor dispatch."""
    return await dispatch_service_action(
        hub,
        service_name,
        action,
        *args,
        **kwargs,
    )


def descriptor_is_available(hub, coordinator, description) -> bool:
    """Evaluate the descriptor-specific availability policy."""
    availability = getattr(description, "availability", "always")
    data = coordinator.data if isinstance(coordinator.data, dict) else {}

    if availability == "always":
        return True
    if availability == "web_api":
        return bool(hub.web_api_configured)
    if availability == "storage_web_api":
        return bool(hub.storage_configured and hub.web_api_configured)
    if availability == "data_not_none":
        return data.get(description.key) is not None
    if availability == "web_api_manual_mode":
        return bool(
            hub.web_api_configured
            and data.get("api_battery_mode_effective_raw") == 1
        )
    if availability == "storage_mode":
        mode_capability = getattr(description, "mode_capability", None)
        if mode_capability is not None:
            return storage_mode_supports(
                hub.storage_extended_control_mode,
                mode_capability,
            )
        available_modes = getattr(description, "available_modes", None) or ()
        return hub.storage_extended_control_mode in available_modes
    return True


def descriptor_native_number_value(hub, description, value: Any):
    """Return the user-facing number value for a descriptor-backed entity."""
    if value is None:
        return None
    display_scale = getattr(description, "display_scale", None)
    if display_scale == "discharge_rate":
        return round(float(value) / 100.0 * hub.max_discharge_rate_w, 0)
    if display_scale == "charge_rate":
        return round(float(value) / 100.0 * hub.max_charge_rate_w, 0)
    return value


# Backward-compatible alias kept for tests and any older call sites.
descriptor_number_value = descriptor_native_number_value


def descriptor_number_write_value(description, value: Any):
    """Normalize a numeric entity value before sending it to the hub."""
    value_transform = getattr(description, "value_transform", None)
    if value_transform == "round_int":
        return int(round(value))
    return value
