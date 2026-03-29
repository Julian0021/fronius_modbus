from __future__ import annotations


class Entity:
    def __init__(self) -> None:
        self.hass = None
        self.platform = None
        self._on_remove_callbacks = []
        self._async_write_count = 0
        self._last_written_state = None

    async def async_added_to_hass(self) -> None:
        return None

    async def async_will_remove_from_hass(self) -> None:
        callbacks = list(self._on_remove_callbacks)
        self._on_remove_callbacks.clear()
        for callback in callbacks:
            callback()

    def async_on_remove(self, remove_callback) -> None:
        self._on_remove_callbacks.append(remove_callback)

    def _state_to_write(self):
        for attribute in ("native_value", "current_option", "is_on"):
            if hasattr(type(self), attribute):
                return getattr(self, attribute)
        return None

    def _validate_state(self, state) -> None:
        if isinstance(state, (dict, list, set, tuple)):
            raise TypeError(f"Unsupported state type: {type(state).__name__}")

    def async_write_ha_state(self) -> None:
        state = self._state_to_write()
        self._validate_state(state)
        self._last_written_state = state
        self._async_write_count += 1


class EntityCategory:
    DIAGNOSTIC = "diagnostic"
