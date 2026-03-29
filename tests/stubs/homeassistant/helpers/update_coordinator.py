from __future__ import annotations

from homeassistant.helpers.entity import Entity


class DataUpdateCoordinator:
    def __init__(
        self,
        hass,
        logger=None,
        *,
        name=None,
        update_interval=None,
        config_entry=None,
    ) -> None:
        self.hass = hass
        self.logger = logger
        self.name = name
        self.update_interval = update_interval
        self.config_entry = config_entry
        self.data = None
        self.last_update_success = True
        self._listeners = []

    async def async_refresh(self) -> None:
        update = getattr(self, "_async_update_data", None)
        if update is None:
            return
        try:
            self.async_set_updated_data(await update())
        except Exception:
            self.last_update_success = False
            raise

    async def async_config_entry_first_refresh(self) -> None:
        await self.async_refresh()

    def async_add_listener(self, update_callback):
        self._listeners.append(update_callback)

        def _remove_listener() -> None:
            if update_callback in self._listeners:
                self._listeners.remove(update_callback)

        return _remove_listener

    def async_set_updated_data(self, data) -> None:
        self.data = data
        self.last_update_success = True
        for listener in tuple(self._listeners):
            listener()

    @property
    def listener_count(self) -> int:
        return len(self._listeners)


class CoordinatorEntity(Entity):
    def __init__(self, coordinator) -> None:
        super().__init__()
        self.coordinator = coordinator

    async def async_added_to_hass(self) -> None:
        await super().async_added_to_hass()
        self.hass = self.coordinator.hass
        self.async_on_remove(
            self.coordinator.async_add_listener(self._handle_coordinator_update)
        )


class UpdateFailed(Exception):
    """Raised when a coordinator refresh fails."""
