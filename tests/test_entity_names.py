from __future__ import annotations

import custom_components.fronius_modbus.entity_names as entity_names
from test_support.fakes import FakeHass


def test_resolve_cached_entity_name_formats_placeholders(monkeypatch) -> None:
    hass = FakeHass()

    monkeypatch.setattr(
        entity_names,
        "cached_translation_data",
        lambda _language: {
            "entity": {
                "sensor": {
                    "mppt_module_dc_power": {"name": "MPPT {module} Power"},
                }
            }
        },
    )

    resolved = entity_names.resolve_cached_entity_name(
        hass,
        entity_platform="sensor",
        translation_key="mppt_module_dc_power",
        placeholders={"module": "2"},
        fallback="Fallback",
    )

    assert resolved == "MPPT 2 Power"


async def test_async_resolve_entity_name_uses_fallback_when_missing(monkeypatch) -> None:
    hass = FakeHass()

    async def _fake_async_get_translation_data(_hass, _language):
        return {}

    monkeypatch.setattr(
        entity_names,
        "async_get_translation_data",
        _fake_async_get_translation_data,
    )

    resolved = await entity_names.async_resolve_entity_name(
        hass,
        entity_platform="sensor",
        translation_key="missing_key",
        fallback="Fallback",
    )

    assert resolved == "Fallback"


def test_resolve_cached_entity_object_id_uses_english_translation(monkeypatch) -> None:
    hass = FakeHass(language="de")

    monkeypatch.setattr(
        entity_names,
        "cached_translation_data",
        lambda language: {
            "entity": {
                "select": {
                    "ext_control_mode": {
                        "name": (
                            "Storage Control Mode"
                            if language == "en"
                            else "Speicher-Steuermodus"
                        )
                    }
                }
            }
        },
    )

    object_id = entity_names.resolve_cached_entity_object_id(
        hass,
        entity_platform="select",
        translation_key="ext_control_mode",
        fallback="ext_control_mode",
    )

    assert object_id == "storage_control_mode"
