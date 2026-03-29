from __future__ import annotations

import json
import logging
from pathlib import Path

from homeassistant.core import HomeAssistant

_LOGGER = logging.getLogger(__name__)
_TRANSLATIONS_DIR = Path(__file__).resolve().parent / "translations"
_TRANSLATION_CACHE: dict[str, dict] = {}


def translation_language_candidates(hass: HomeAssistant) -> list[str]:
    candidates: list[str] = []
    language = getattr(getattr(hass, "config", None), "language", None)
    if isinstance(language, str) and language:
        candidates.append(language)
        if "-" in language:
            candidates.append(language.split("-", 1)[0])
    candidates.append("en")
    return candidates


def _read_translation_data(path: Path) -> dict:
    try:
        raw_data = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        _LOGGER.warning("Translation file is missing: %s", path.name)
        return {}

    try:
        translation_data = json.loads(raw_data)
    except json.JSONDecodeError as err:
        _LOGGER.warning("Translation file contains invalid JSON: %s (%s)", path.name, err)
        return {}

    if not isinstance(translation_data, dict):
        _LOGGER.warning(
            "Translation file must decode to a JSON object: %s",
            path.name,
        )
        return {}

    return translation_data


async def async_get_translation_data(hass: HomeAssistant, language: str) -> dict:
    if language not in _TRANSLATION_CACHE:
        path = _TRANSLATIONS_DIR / f"{language}.json"
        _TRANSLATION_CACHE[language] = await hass.async_add_executor_job(
            _read_translation_data,
            path,
        )
    return _TRANSLATION_CACHE[language]


async def async_ensure_translation_cache(hass: HomeAssistant) -> None:
    for language in translation_language_candidates(hass):
        await async_get_translation_data(hass, language)


def cached_translation_data(language: str) -> dict:
    return _TRANSLATION_CACHE.get(language, {})
