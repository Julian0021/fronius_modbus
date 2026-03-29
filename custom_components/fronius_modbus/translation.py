from __future__ import annotations

import json
from pathlib import Path

from homeassistant.core import HomeAssistant

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
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


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
