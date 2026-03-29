from __future__ import annotations


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

    async def async_refresh(self) -> None:
        update = getattr(self, "_async_update_data", None)
        if update is None:
            return
        self.data = await update()
        self.last_update_success = True

    async def async_config_entry_first_refresh(self) -> None:
        await self.async_refresh()

    def async_set_updated_data(self, data) -> None:
        self.data = data
        self.last_update_success = True


class CoordinatorEntity:
    def __init__(self, coordinator) -> None:
        self.coordinator = coordinator

    async def async_added_to_hass(self) -> None:
        return None

    def async_write_ha_state(self) -> None:
        return None


class UpdateFailed(Exception):
    """Raised when a coordinator refresh fails."""
