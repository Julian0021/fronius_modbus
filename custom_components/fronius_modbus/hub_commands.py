"""Command handlers for Hub writes."""
from __future__ import annotations

from functools import wraps

from .storage_modes import get_storage_mode_policy


def toggle_busy(func):
    """Serialize write commands through the hub command lock."""

    @wraps(func)
    async def wrapper(self, *args, **kwargs):
        hub = getattr(self, "_hub", self)
        async with hub._command_lock:
            return await func(self, *args, **kwargs)

    return wrapper


def derive_api_battery_mode(
    raw_mode: int | None,
    raw_soc_mode: str | None,
) -> int | None:
    """Normalize the two Web API battery-mode signals into one display mode."""
    if raw_mode == 1 and raw_soc_mode == "manual":
        return 1
    if raw_mode == 0 and raw_soc_mode == "auto":
        return 0
    return None


class HubCommandService:
    """Own the user-facing write commands for the Fronius hub."""

    def __init__(self, hub) -> None:
        self._hub = hub

    def _get_next_soc_limits(
        self,
        *,
        soc_min: int | None = None,
        soc_max: int | None = None,
    ) -> tuple[int, int]:
        next_soc_min = (
            self._hub._as_int(self._hub.data.get("soc_minimum"))
            if soc_min is None
            else int(soc_min)
        )
        next_soc_max = (
            self._hub._as_int(self._hub.data.get("soc_maximum"))
            if soc_max is None
            else int(soc_max)
        )

        next_soc_min = 5 if next_soc_min is None else next_soc_min
        next_soc_max = 99 if next_soc_max is None else next_soc_max

        if next_soc_min < 5 or next_soc_min > 100:
            raise ValueError("SoC Minimum must be between 5 and 100")
        if next_soc_max < 0 or next_soc_max > 100:
            raise ValueError("SoC Maximum must be between 0 and 100")
        if next_soc_min > next_soc_max:
            raise ValueError("SoC Minimum must not exceed SoC Maximum")

        return next_soc_min, next_soc_max

    def _get_api_soc_values(
        self,
        *,
        soc_min: int | None = None,
        soc_max: int | None = None,
    ) -> tuple[int, int, int]:
        """Validate the next SoC limits and reuse the current backup reserve."""
        next_soc_min, next_soc_max = self._get_next_soc_limits(
            soc_min=soc_min,
            soc_max=soc_max,
        )
        next_backup_reserved = self._hub._as_int(self._hub.data.get("api_backup_reserved"))
        next_backup_reserved = 5 if next_backup_reserved is None else next_backup_reserved
        if next_backup_reserved < 5 or next_backup_reserved > 100:
            raise ValueError("Battery backup reserve must be between 5 and 100")

        return next_soc_min, next_soc_max, next_backup_reserved

    def _api_battery_mode_is_manual(self) -> bool:
        return self._hub._as_int(self._hub.data.get("api_battery_mode_effective_raw")) == 1

    def _require_api_battery_mode_manual(self, control_name: str) -> None:
        if not self._api_battery_mode_is_manual():
            raise ValueError(
                f"{control_name} can only be changed when Battery API mode is Manual"
            )

    async def _set_api_soc_manual(
        self,
        soc_min: int | None = None,
        soc_max: int | None = None,
        control_name: str = "SoC Maximum",
    ) -> tuple[int, int, int] | None:
        """Write manual battery SoC limits and start the recovery window."""
        if not self._hub._webclient:
            return None
        self._require_api_battery_mode_manual(control_name)

        next_soc_min, next_soc_max, next_backup_reserved = self._get_api_soc_values(
            soc_min=soc_min,
            soc_max=soc_max,
        )
        await self._hub._web_api_service.async_web_job(
            self._hub._webclient.set_battery_soc_config,
            next_soc_min,
            next_soc_max,
            next_backup_reserved,
            raise_on_auth_failure=True,
        )
        self._hub._web_api_service.start_battery_write_transition(control_name)
        return next_soc_min, next_soc_max, next_backup_reserved

    async def _set_api_charge_sources(
        self,
        *,
        charge_from_grid: bool | None = None,
        charge_from_ac: bool | None = None,
    ) -> None:
        if not self._hub._webclient:
            return

        if charge_from_ac is False:
            next_charge_from_grid = False
            next_charge_from_ac = False
        else:
            next_charge_from_grid = (
                self._hub._enabled_bool(self._hub.data.get("api_charge_from_grid"))
                if charge_from_grid is None
                else bool(charge_from_grid)
            )
            next_charge_from_ac = (
                self._hub._enabled_bool(self._hub.data.get("api_charge_from_ac"))
                if charge_from_ac is None
                else bool(charge_from_ac)
            )
            if next_charge_from_grid and charge_from_ac is None:
                next_charge_from_ac = True

        await self._hub._web_api_service.async_web_job(
            self._hub._webclient.set_battery_charge_sources,
            next_charge_from_grid,
            next_charge_from_ac,
            raise_on_auth_failure=True,
        )
        self._hub._web_api_service.start_battery_write_transition(
            "battery charge source"
        )

    @toggle_busy
    async def set_mode(self, mode):
        policy = get_storage_mode_policy(mode)
        if policy is None:
            raise ValueError(f"Unsupported storage mode: {mode}")

        await self._hub._client.write_service.set_extended_mode(mode)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_soc_minimum(self, value):
        soc_minimum = self._hub._require_whole_number(value, "SoC Minimum")
        if soc_minimum < 5 or soc_minimum > 100:
            raise ValueError("SoC Minimum must be between 5 and 100")
        if self._hub._webclient and self._api_battery_mode_is_manual():
            self._get_next_soc_limits(soc_min=soc_minimum)
        await self._hub._client.write_service.set_minimum_reserve(soc_minimum)
        self._hub._publish_data_update()
        if self._hub._webclient and self._api_battery_mode_is_manual():
            await self._set_api_soc_manual(
                soc_min=soc_minimum,
                control_name="SoC Minimum",
            )

    @toggle_busy
    async def set_charge_limit(self, value):
        await self._hub._client.write_service.set_charge_limit(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_discharge_limit(self, value):
        await self._hub._client.write_service.set_discharge_limit(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_grid_charge_power(self, value):
        await self._hub._client.write_service.set_grid_charge_power(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_grid_discharge_power(self, value):
        await self._hub._client.write_service.set_grid_discharge_power(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_api_battery_mode(self, mode: int):
        if not self._hub._webclient:
            return

        current_effective_mode = self._hub._as_int(
            self._hub.data.get("api_battery_mode_effective_raw")
        )
        display_power = self._hub._as_int(self._hub.data.get("api_battery_power"))
        if mode == 1 and display_power is None:
            display_power = 0
        power = -display_power if mode == 1 and display_power is not None else None
        soc_min = None
        if mode == 1 and current_effective_mode != 1:
            soc_min = self._hub._as_int(self._hub.data.get("soc_minimum"))
        await self._hub._web_api_service.async_web_job(
            self._hub._webclient.set_battery_config,
            mode,
            power,
            soc_min,
            raise_on_auth_failure=True,
        )
        self._hub._web_api_service.start_battery_write_transition("Battery API mode")
        await self._hub._web_api_service.refresh_web_data()
        self._hub._publish_data_update()

    @toggle_busy
    async def set_api_battery_power(self, value: float):
        if not self._hub._webclient:
            return
        self._require_api_battery_mode_manual("Target feed in")

        power = -int(round(value))
        await self._hub._web_api_service.async_web_job(
            self._hub._webclient.set_battery_config,
            1,
            power,
            raise_on_auth_failure=True,
        )
        self._hub._web_api_service.start_battery_write_transition("Target feed in")

    @toggle_busy
    async def set_api_soc_values(
        self,
        soc_max: int | None = None,
    ):
        if not self._hub._webclient:
            return

        await self._set_api_soc_manual(
            soc_max=soc_max,
            control_name="SoC Maximum",
        )

    @toggle_busy
    async def set_api_charge_sources(
        self,
        *,
        charge_from_grid: bool | None = None,
        charge_from_ac: bool | None = None,
    ) -> None:
        await self._set_api_charge_sources(
            charge_from_grid=charge_from_grid,
            charge_from_ac=charge_from_ac,
        )

    @toggle_busy
    async def set_ac_limit_rate(self, value):
        await self._hub._client.write_service.set_ac_limit_rate(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_ac_limit_enable(self, value):
        await self._hub._client.write_service.set_ac_limit_enable(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_power_factor(self, value):
        await self._hub._client.write_service.set_power_factor(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_power_factor_enable(self, value):
        await self._hub._client.write_service.set_power_factor_enable(value)
        self._hub._publish_data_update()

    @toggle_busy
    async def set_conn_status(self, enable):
        await self._hub._client.write_service.set_conn_status(enable)
        self._hub._publish_data_update()
