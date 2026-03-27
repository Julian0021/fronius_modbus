import json
import logging
from pathlib import Path
from homeassistant.helpers.update_coordinator import CoordinatorEntity
from homeassistant.core import callback
from homeassistant.core import HomeAssistant
from .hub import Hub

_LOGGER = logging.getLogger(__name__)
_TRANSLATIONS_DIR = Path(__file__).resolve().parent / "translations"
_TRANSLATION_CACHE: dict[str, dict] = {}


def _translation_language_candidates(hass: HomeAssistant) -> list[str]:
    language_candidates: list[str] = []
    language = getattr(hass.config, "language", None)
    if isinstance(language, str) and language:
        language_candidates.append(language)
        if "-" in language:
            language_candidates.append(language.split("-", 1)[0])
    language_candidates.append("en")
    return language_candidates


def _read_translation_data(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return {}


async def async_ensure_translation_cache(hass: HomeAssistant) -> None:
    """Preload translation files used for entity-name fallback."""
    for language in _translation_language_candidates(hass):
        if language in _TRANSLATION_CACHE:
            continue
        path = _TRANSLATIONS_DIR / f"{language}.json"
        _TRANSLATION_CACHE[language] = await hass.async_add_executor_job(
            _read_translation_data,
            path,
        )


class FroniusModbusBaseEntity(CoordinatorEntity):
    """Base entity for Fronius Modbus devices."""
    _key = None
    _options_dict = None
    _translation_platform = None

    @classmethod
    def _load_translation_data(cls, language: str) -> dict:
        """Load a translation file for the requested language."""
        return _TRANSLATION_CACHE.get(language, {})

    def _resolve_entity_name(
        self,
        coordinator,
        name,
        translation_key,
        translation_placeholders,
    ):
        """Resolve a localized fallback entity name from bundled translation files."""
        if translation_key is None or self._translation_platform is None:
            return name

        for candidate in _translation_language_candidates(coordinator.hass):
            data = self._load_translation_data(candidate)
            translated_name = (
                data.get("entity", {})
                .get(self._translation_platform, {})
                .get(translation_key, {})
                .get("name")
            )
            if not isinstance(translated_name, str):
                continue
            if translation_placeholders:
                try:
                    return translated_name.format(**translation_placeholders)
                except KeyError:
                    _LOGGER.warning(
                        "Missing translation placeholders for %s.%s",
                        self._translation_platform,
                        translation_key,
                    )
                    return translated_name
            return translated_name
        return name

    def __init__(
        self,
        coordinator,
        device_info,
        name,
        key,
        device_class=None,
        state_class=None,
        unit=None,
        icon=None,
        entity_category=None,
        translation_key=None,
        translation_placeholders=None,
        options=None,
        min=None,
        max=None,
        native_step=None,
        mode=None,
    ):
        """Initialize the entity."""
        super().__init__(coordinator)
        self._key = key
        self._name = name
        self._unit_of_measurement = unit
        self._icon = icon
        self._device_info = device_info

        if device_class is not None:
            self._attr_device_class = device_class
        if state_class is not None:
            self._attr_state_class = state_class
        if entity_category is not None:
            self._attr_entity_category = entity_category
        if options is not None:
            if isinstance(options, dict):
                self._options_dict = options
                self._attr_options = list(options.values())
            else:
                self._attr_options = list(options)
        if unit is not None:
            self._attr_native_unit_of_measurement = unit
        if min is not None:
            self._attr_native_min_value = min
        if max is not None:
            self._attr_native_max_value = max
        if native_step is not None:
            self._attr_native_step = native_step
        if mode is not None:
            self._attr_mode = mode

        self._attr_has_entity_name = True
        if translation_key is not None:
            self._attr_translation_key = translation_key
            if translation_placeholders is not None:
                self._attr_translation_placeholders = translation_placeholders
        self._attr_name = self._resolve_entity_name(
            coordinator=coordinator,
            name=name,
            translation_key=translation_key,
            translation_placeholders=translation_placeholders,
        )
        self._attr_unique_id = f"{coordinator.hub.entity_prefix}_{self._key}"
        self._attr_device_info = device_info

    @property
    def available(self) -> bool:
        """Return if entity is available."""
        return self.coordinator.last_update_success

    async def async_added_to_hass(self) -> None:
        """When entity is added to hass."""
        await super().async_added_to_hass()
        self._handle_coordinator_update()

    @callback
    def _handle_coordinator_update(self) -> None:
        """Handle updated data from the coordinator."""
        self.async_write_ha_state()

    @property
    def should_poll(self) -> bool:
        """Data is delivered by the coordinator."""
        return False

    @property
    def unit_of_measurement(self):
        """Return the unit of measurement."""
        return self._unit_of_measurement

    @property
    def icon(self):
        """Return the sensor icon."""
        return self._icon

  
