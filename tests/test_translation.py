from __future__ import annotations

import json
from pathlib import Path
import re

from homeassistant.core import HomeAssistant

import custom_components.fronius_modbus.const as const
import custom_components.fronius_modbus.translation as translation

_VALID_TRANSLATION_KEY_RE = re.compile(r"[a-z0-9](?:[a-z0-9-_]*[a-z0-9])?")


async def test_async_get_translation_data_warns_when_file_is_missing(
    monkeypatch,
    tmp_path,
    caplog,
) -> None:
    monkeypatch.setattr(translation, "_TRANSLATIONS_DIR", tmp_path)

    data = await translation.async_get_translation_data(HomeAssistant(), "missing")

    assert data == {}
    assert "Translation file is missing: missing.json" in caplog.text


async def test_async_get_translation_data_warns_when_json_is_invalid(
    monkeypatch,
    tmp_path,
    caplog,
) -> None:
    monkeypatch.setattr(translation, "_TRANSLATIONS_DIR", tmp_path)
    (tmp_path / "broken.json").write_text("{not json", encoding="utf-8")

    data = await translation.async_get_translation_data(HomeAssistant(), "broken")

    assert data == {}
    assert "Translation file contains invalid JSON: broken.json" in caplog.text


async def test_async_get_translation_data_warns_when_payload_is_not_an_object(
    monkeypatch,
    tmp_path,
    caplog,
) -> None:
    monkeypatch.setattr(translation, "_TRANSLATIONS_DIR", tmp_path)
    (tmp_path / "list.json").write_text(json.dumps(["not", "an", "object"]), encoding="utf-8")

    data = await translation.async_get_translation_data(HomeAssistant(), "list")

    assert data == {}
    assert "Translation file must decode to a JSON object: list.json" in caplog.text


def test_descriptor_translation_keys_match_hassfest_requirements() -> None:
    invalid_keys: set[str] = set()

    for value in vars(const).values():
        if isinstance(value, dict):
            candidates = value.values()
        elif isinstance(value, tuple):
            candidates = value
        else:
            continue

        for candidate in candidates:
            translation_key = getattr(candidate, "translation_key", None)
            if isinstance(translation_key, str) and not _VALID_TRANSLATION_KEY_RE.fullmatch(
                translation_key
            ):
                invalid_keys.add(translation_key)

    assert invalid_keys == set()


def test_translation_files_use_valid_entity_keys_and_no_legacy_config_title() -> None:
    translations_dir = Path("custom_components/fronius_modbus/translations")

    for path in (translations_dir / "en.json", translations_dir / "de.json"):
        data = json.loads(path.read_text(encoding="utf-8"))

        assert "title" not in data.get("config", {})

        invalid_entity_keys = {
            f"{platform}.{key}"
            for platform, entities in data.get("entity", {}).items()
            for key in entities
            if not _VALID_TRANSLATION_KEY_RE.fullmatch(key)
        }

        invalid_state_keys = {
            f"{platform}.{entity_key}.state.{state_key}"
            for platform, entities in data.get("entity", {}).items()
            for entity_key, entity_data in entities.items()
            if isinstance(entity_data, dict)
            for state_key in entity_data.get("state", {})
            if not _VALID_TRANSLATION_KEY_RE.fullmatch(state_key)
        }

        assert invalid_entity_keys == set()
        assert invalid_state_keys == set()
