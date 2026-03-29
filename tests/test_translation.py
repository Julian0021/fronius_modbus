from __future__ import annotations

import json

from homeassistant.core import HomeAssistant

import custom_components.fronius_modbus.translation as translation


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
