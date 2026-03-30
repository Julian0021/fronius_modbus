"""Bootstrap helpers for Hub setup and first-refresh orchestration."""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from importlib.metadata import version
from pathlib import Path

from homeassistant.config_entries import ConfigEntry
from packaging import version as pkg_version
from packaging.requirements import InvalidRequirement, Requirement

from .integration_errors import FroniusDependencyError, FroniusReadError

_LOGGER = logging.getLogger(__name__)


def _manifest_pymodbus_minimum_version() -> str | None:
    try:
        manifest = json.loads(
            Path(__file__).with_name("manifest.json").read_text(encoding="utf-8")
        )
    except (OSError, ValueError, json.JSONDecodeError):
        return None

    requirements = manifest.get("requirements")
    if not isinstance(requirements, list):
        return None

    for raw_requirement in requirements:
        if not isinstance(raw_requirement, str):
            continue
        try:
            requirement = Requirement(raw_requirement)
        except InvalidRequirement:
            continue
        if requirement.name != "pymodbus":
            continue
        for specifier in requirement.specifier:
            if specifier.operator == ">=":
                return specifier.version
    return None


def check_pymodbus_version() -> None:
    required_version = _manifest_pymodbus_minimum_version()
    if required_version is None:
        _LOGGER.debug("Skipping pymodbus version check because manifest parsing failed")
        return

    try:
        current_version = version("pymodbus")
        current = pkg_version.parse(current_version)
        required = pkg_version.parse(required_version)
    except Exception as err:
        raise FroniusDependencyError("Error checking installed pymodbus version") from err

    if current < required:
        raise FroniusDependencyError(
            f"pymodbus {current_version} found, please update to {required_version} or higher"
        )

    _LOGGER.debug("pymodbus %s", current_version)


@dataclass(slots=True)
class MeterTopologyDiscovery:
    """Normalized meter metadata discovered during bootstrap."""

    phase_counts: dict[int, int] = field(default_factory=dict)
    locations: dict[int, int] = field(default_factory=dict)


class HubBootstrapService:
    """Own startup sequencing, device enrichment, and coordinator setup."""

    def __init__(self, hub) -> None:
        self._hub = hub

    async def init_data(
        self,
        config_entry: ConfigEntry | None = None,
        setup_coordinator: bool = True,
        apply_modbus_config: bool = False,
    ) -> None:
        self._hub._config_entry = config_entry
        await self._async_check_dependencies()
        await self._async_apply_modbus_config(apply_modbus_config)
        meter_topology = await self._async_discover_meter_topology()
        await self._hub._client.runtime_service.init_data()
        self._apply_meter_topology(meter_topology)
        self._warn_missing_primary_meter()
        await self._async_enrich_storage_identity()
        await self._async_refresh_initial_web_state()
        await self._async_setup_coordinator(
            config_entry=config_entry,
            setup_coordinator=setup_coordinator,
        )

    async def _async_check_dependencies(self) -> None:
        await self._hub._hass.async_add_executor_job(check_pymodbus_version)

    async def _async_apply_modbus_config(self, apply_modbus_config: bool) -> None:
        if (
            apply_modbus_config
            and self._hub.web_api_configured
            and self._hub._auto_enable_modbus
        ):
            await self._hub.web_api_service.async_apply_modbus_config()

    async def _async_discover_meter_topology(self) -> MeterTopologyDiscovery:
        discovery = MeterTopologyDiscovery()
        if not self._hub.web_api_configured:
            return discovery

        try:
            meter_info = await self._hub.web_api_service.async_web_job(
                self._hub._webclient.get_power_meter_info,
                self._hub._client.primary_meter_unit_id,
            )
        except FroniusReadError as err:
            _LOGGER.debug(
                "Keeping existing smart meter unit ids for %s because PowerMeter payload parsing failed: %s",
                self._hub._host,
                err,
            )
            return discovery
        if meter_info is None:
            _LOGGER.debug(
                "Keeping existing smart meter unit ids for %s because PowerMeter payload parsing failed",
                self._hub._host,
            )
            return discovery
        if not isinstance(meter_info, dict):
            return discovery

        self._hub._client.set_meter_unit_ids(
            meter_info.get("unit_ids"),
            primary_unit_id=meter_info.get("primary_unit_id"),
        )
        discovery.phase_counts = self._coerce_positive_int_map(
            meter_info.get("phase_counts_by_unit_id"),
        )
        discovery.locations = self._coerce_non_negative_int_map(
            meter_info.get("locations_by_unit_id"),
        )
        return discovery

    def _coerce_positive_int_map(self, raw_values) -> dict[int, int]:
        values: dict[int, int] = {}
        if not isinstance(raw_values, dict):
            return values
        for raw_key, raw_value in raw_values.items():
            if (
                not self._hub._client.is_numeric(raw_key)
                or not self._hub._client.is_numeric(raw_value)
            ):
                continue
            key = int(raw_key)
            value = int(raw_value)
            if key <= 0 or value <= 0:
                continue
            values[key] = value
        return values

    def _coerce_non_negative_int_map(self, raw_values) -> dict[int, int]:
        values: dict[int, int] = {}
        if not isinstance(raw_values, dict):
            return values
        for raw_key, raw_value in raw_values.items():
            if (
                not self._hub._client.is_numeric(raw_key)
                or not self._hub._client.is_numeric(raw_value)
            ):
                continue
            key = int(raw_key)
            value = int(raw_value)
            if key <= 0 or value < 0:
                continue
            values[key] = value
        return values

    def _apply_meter_topology(self, discovery: MeterTopologyDiscovery) -> None:
        for unit_id in self._hub.meter_unit_ids:
            phase_count = discovery.phase_counts.get(unit_id)
            if phase_count is not None:
                self._hub.set_meter_value(unit_id, "phase_count", phase_count)
            location = discovery.locations.get(unit_id)
            if location is not None:
                self._hub.set_meter_value(unit_id, "location", location)

    def _warn_missing_primary_meter(self) -> None:
        if (
            self._hub.meter_configured
            and self._hub.primary_meter_unit_id not in self._hub.meter_unit_ids
        ):
            _LOGGER.warning(
                "Configured meter unit ids %s do not include the primary meter unit id %s; Load and Grid status will stay unavailable",
                self._hub.meter_unit_ids,
                self._hub.primary_meter_unit_id,
            )

    async def _async_enrich_storage_identity(self) -> None:
        if not self._hub.storage_configured:
            return

        self._hub._client.reset_storage_info()
        if not self._hub.web_api_configured:
            return

        try:
            storage_info = await self._hub.web_api_service.async_web_job(
                self._hub._webclient.get_storage_info
            )
        except FroniusReadError as err:
            _LOGGER.debug(
                "Skipping storage identity enrichment for %s because the web payload could not be parsed: %s",
                self._hub._host,
                err,
            )
            return
        if not isinstance(storage_info, dict):
            return

        self._hub._client.set_storage_info(
            manufacturer=storage_info.get("manufacturer"),
            model=storage_info.get("model"),
            serial=storage_info.get("serial"),
        )
        self._hub.storage_state.set(
            "storage_temperature",
            storage_info.get("cell_temperature"),
        )

    async def _async_refresh_initial_web_state(self) -> None:
        if self._hub.web_api_configured:
            await self._hub.web_api_service.refresh_web_data()

    async def _async_setup_coordinator(
        self,
        *,
        config_entry: ConfigEntry | None,
        setup_coordinator: bool,
    ) -> None:
        if not setup_coordinator:
            return

        from .hub import FroniusCoordinator

        self._hub.coordinator = FroniusCoordinator(
            self._hub._hass,
            self._hub,
            config_entry=config_entry,
        )
        if config_entry is not None:
            await self._hub.coordinator.async_config_entry_first_refresh()
        else:
            await self._hub.coordinator.async_refresh()
