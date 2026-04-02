from __future__ import annotations

import logging
import re
from typing import Any
import unicodedata

from homeassistant.core import HomeAssistant

from .translation import (
    async_get_translation_data,
    cached_translation_data,
    translation_language_candidates,
)

_NON_OBJECT_ID_RE = re.compile(r"[^a-z0-9_]+")
_MULTI_UNDERSCORE_RE = re.compile(r"_+")


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


def _resolve_entity_name_from_translation_data(
    translation_data: dict[str, Any],
    *,
    entity_platform: str,
    translation_key: str,
    placeholders: dict[str, str] | None,
    logger: logging.Logger | None,
) -> str | None:
    translated_name = _translated_name_from_data(
        translation_data,
        entity_platform=entity_platform,
        translation_key=translation_key,
    )
    if translated_name is None:
        return None
    return _format_translated_name(
        translated_name,
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        logger=logger,
    )


def _resolve_entity_name_from_lookup(
    hass: HomeAssistant,
    *,
    language_candidates: list[str],
    entity_platform: str | None,
    translation_key: str | None,
    placeholders: dict[str, str] | None,
    fallback: str,
    logger: logging.Logger | None,
    load_translation_data,
) -> str:
    if translation_key is None or entity_platform is None:
        return fallback

    for candidate in language_candidates:
        resolved_name = _resolve_entity_name_from_translation_data(
            load_translation_data(candidate),
            entity_platform=entity_platform,
            translation_key=translation_key,
            placeholders=placeholders,
            logger=logger,
        )
        if resolved_name is not None:
            return resolved_name

    return fallback


def _slugify_object_id(value: str) -> str:
    normalized = (
        unicodedata.normalize("NFKD", value).encode("ascii", "ignore").decode("ascii")
    )
    normalized = normalized.lower().replace("-", "_").replace(" ", "_")
    normalized = _NON_OBJECT_ID_RE.sub("_", normalized)
    normalized = _MULTI_UNDERSCORE_RE.sub("_", normalized).strip("_")
    return normalized or "entity"


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
    return _resolve_entity_name_from_lookup(
        hass,
        language_candidates=translation_language_candidates(hass),
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        fallback=fallback,
        logger=logger,
        load_translation_data=cached_translation_data,
    )


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
    translation_data_by_language = {}
    for candidate in translation_language_candidates(hass):
        translation_data_by_language[candidate] = await async_get_translation_data(
            hass,
            candidate,
        )

    return _resolve_entity_name_from_lookup(
        hass,
        language_candidates=translation_language_candidates(hass),
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        fallback=fallback,
        logger=logger,
        load_translation_data=translation_data_by_language.__getitem__,
    )


def resolve_cached_entity_name_for_language(
    hass: HomeAssistant,
    *,
    language: str,
    entity_platform: str | None,
    translation_key: str | None,
    placeholders: dict[str, str] | None = None,
    fallback: str,
    logger: logging.Logger | None = None,
) -> str:
    """Resolve an entity name for a specific language from the translation cache."""
    return _resolve_entity_name_from_lookup(
        hass,
        language_candidates=[language],
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        fallback=fallback,
        logger=logger,
        load_translation_data=cached_translation_data,
    )


async def async_resolve_entity_name_for_language(
    hass: HomeAssistant,
    *,
    language: str,
    entity_platform: str,
    translation_key: str,
    placeholders: dict[str, str] | None = None,
    fallback: str,
    logger: logging.Logger | None = None,
) -> str:
    """Resolve an entity name for a specific language from bundled translations."""
    translation_data = await async_get_translation_data(hass, language)
    return _resolve_entity_name_from_lookup(
        hass,
        language_candidates=[language],
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        fallback=fallback,
        logger=logger,
        load_translation_data={language: translation_data}.__getitem__,
    )


def resolve_cached_entity_object_id(
    hass: HomeAssistant,
    *,
    entity_platform: str | None,
    translation_key: str | None,
    placeholders: dict[str, str] | None = None,
    fallback: str,
    logger: logging.Logger | None = None,
) -> str:
    """Resolve a stable English object id for an entity."""
    entity_name = resolve_cached_entity_name_for_language(
        hass,
        language="en",
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        fallback=fallback,
        logger=logger,
    )
    return _slugify_object_id(entity_name)


async def async_resolve_entity_object_id(
    hass: HomeAssistant,
    *,
    entity_platform: str,
    translation_key: str,
    placeholders: dict[str, str] | None = None,
    fallback: str,
    logger: logging.Logger | None = None,
) -> str:
    """Resolve a stable English object id for an entity from bundled translations."""
    entity_name = await async_resolve_entity_name_for_language(
        hass,
        language="en",
        entity_platform=entity_platform,
        translation_key=translation_key,
        placeholders=placeholders,
        fallback=fallback,
        logger=logger,
    )
    return _slugify_object_id(entity_name)
