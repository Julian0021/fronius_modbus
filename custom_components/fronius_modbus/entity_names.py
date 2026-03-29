from __future__ import annotations

import logging
from typing import Any

from homeassistant.core import HomeAssistant

from .translation import (
    async_get_translation_data,
    cached_translation_data,
    translation_language_candidates,
)


def _translated_name_from_data(
    translation_data: dict[str, Any],
    *,
    entity_platform: str,
    translation_key: str,
) -> str | None:
    translated_name = (
        translation_data.get("entity", {})
        .get(entity_platform, {})
        .get(translation_key, {})
        .get("name")
    )
    return translated_name if isinstance(translated_name, str) else None


def _format_translated_name(
    translated_name: str,
    *,
    entity_platform: str,
    translation_key: str,
    placeholders: dict[str, str] | None,
    logger: logging.Logger | None,
) -> str:
    if not placeholders:
        return translated_name

    try:
        return translated_name.format(**placeholders)
    except KeyError:
        if logger is not None:
            logger.warning(
                "Missing translation placeholders for %s.%s",
                entity_platform,
                translation_key,
            )
        return translated_name


def resolve_cached_entity_name(
    hass: HomeAssistant,
    *,
    entity_platform: str | None,
    translation_key: str | None,
    placeholders: dict[str, str] | None = None,
    fallback: str,
    logger: logging.Logger | None = None,
) -> str:
    """Resolve an entity name from the translation cache, or return the fallback."""
    if translation_key is None or entity_platform is None:
        return fallback

    for candidate in translation_language_candidates(hass):
        translated_name = _translated_name_from_data(
            cached_translation_data(candidate),
            entity_platform=entity_platform,
            translation_key=translation_key,
        )
        if translated_name is None:
            continue
        return _format_translated_name(
            translated_name,
            entity_platform=entity_platform,
            translation_key=translation_key,
            placeholders=placeholders,
            logger=logger,
        )

    return fallback


async def async_resolve_entity_name(
    hass: HomeAssistant,
    *,
    entity_platform: str,
    translation_key: str,
    placeholders: dict[str, str] | None = None,
    fallback: str,
    logger: logging.Logger | None = None,
) -> str:
    """Resolve an entity name from bundled translations, or return the fallback."""
    for candidate in translation_language_candidates(hass):
        translated_name = _translated_name_from_data(
            await async_get_translation_data(hass, candidate),
            entity_platform=entity_platform,
            translation_key=translation_key,
        )
        if translated_name is None:
            continue
        return _format_translated_name(
            translated_name,
            entity_platform=entity_platform,
            translation_key=translation_key,
            placeholders=placeholders,
            logger=logger,
        )

    return fallback
