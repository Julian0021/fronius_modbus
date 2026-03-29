from __future__ import annotations

import asyncio
import json
import sys
import types
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "custom_components" / "fronius_modbus"


def _install_homeassistant_stubs() -> None:
    if str(REPO_ROOT) not in sys.path:
        sys.path.insert(0, str(REPO_ROOT))

    homeassistant = sys.modules.setdefault("homeassistant", types.ModuleType("homeassistant"))

    components = sys.modules.setdefault(
        "homeassistant.components",
        types.ModuleType("homeassistant.components"),
    )
    sensor = sys.modules.setdefault(
        "homeassistant.components.sensor",
        types.ModuleType("homeassistant.components.sensor"),
    )

    class SensorDeviceClass:
        CURRENT = "current"
        POWER = "power"
        ENERGY = "energy"
        FREQUENCY = "frequency"
        VOLTAGE = "voltage"
        TEMPERATURE = "temperature"
        BATTERY = "battery"

    class SensorStateClass:
        MEASUREMENT = "measurement"
        TOTAL_INCREASING = "total_increasing"

    sensor.SensorDeviceClass = SensorDeviceClass
    sensor.SensorStateClass = SensorStateClass
    homeassistant.components = components
    components.sensor = sensor

    helpers = sys.modules.setdefault(
        "homeassistant.helpers",
        types.ModuleType("homeassistant.helpers"),
    )
    entity = sys.modules.setdefault(
        "homeassistant.helpers.entity",
        types.ModuleType("homeassistant.helpers.entity"),
    )

    class EntityCategory:
        DIAGNOSTIC = "diagnostic"

    entity.EntityCategory = EntityCategory
    helpers.entity = entity

    issue_registry = sys.modules.setdefault(
        "homeassistant.helpers.issue_registry",
        types.ModuleType("homeassistant.helpers.issue_registry"),
    )

    class IssueSeverity:
        WARNING = "warning"

    def async_create_issue(*args, **kwargs) -> None:
        return None

    def async_delete_issue(*args, **kwargs) -> None:
        return None

    issue_registry.IssueSeverity = IssueSeverity
    issue_registry.async_create_issue = async_create_issue
    issue_registry.async_delete_issue = async_delete_issue
    helpers.issue_registry = issue_registry

    update_coordinator = sys.modules.setdefault(
        "homeassistant.helpers.update_coordinator",
        types.ModuleType("homeassistant.helpers.update_coordinator"),
    )

    class DataUpdateCoordinator:
        def __init__(self, *args, **kwargs) -> None:
            self.args = args
            self.kwargs = kwargs

        async def async_refresh(self) -> None:
            return None

        async def async_config_entry_first_refresh(self) -> None:
            return None

    class UpdateFailed(Exception):
        pass

    update_coordinator.DataUpdateCoordinator = DataUpdateCoordinator
    update_coordinator.UpdateFailed = UpdateFailed
    helpers.update_coordinator = update_coordinator

    storage = sys.modules.setdefault(
        "homeassistant.helpers.storage",
        types.ModuleType("homeassistant.helpers.storage"),
    )

    class Store:
        def __init__(self, *args, **kwargs) -> None:
            self._data = None

        def __class_getitem__(cls, _item):
            return cls

        async def async_load(self):
            return self._data

        async def async_save(self, data) -> None:
            self._data = data

    storage.Store = Store
    helpers.storage = storage

    config_entries = sys.modules.setdefault(
        "homeassistant.config_entries",
        types.ModuleType("homeassistant.config_entries"),
    )

    class ConfigEntry:
        def __init__(self, **kwargs) -> None:
            self.__dict__.update(kwargs)

        def __class_getitem__(cls, _item):
            return cls

    config_entries.ConfigEntry = ConfigEntry
    homeassistant.config_entries = config_entries

    core = sys.modules.setdefault(
        "homeassistant.core",
        types.ModuleType("homeassistant.core"),
    )

    class HomeAssistant:
        def __init__(self) -> None:
            self.data: dict[str, object] = {}
            try:
                self.loop = asyncio.get_running_loop()
            except RuntimeError:
                self.loop = None

        async def async_add_executor_job(self, func, *args):
            return func(*args)

    core.HomeAssistant = HomeAssistant
    homeassistant.core = core


def _install_custom_component_stubs() -> None:
    custom_components = sys.modules.setdefault(
        "custom_components",
        types.ModuleType("custom_components"),
    )
    custom_components.__path__ = [str(REPO_ROOT / "custom_components")]

    package = sys.modules.setdefault(
        "custom_components.fronius_modbus",
        types.ModuleType("custom_components.fronius_modbus"),
    )
    package.__path__ = [str(PACKAGE_ROOT)]


_install_homeassistant_stubs()
_install_custom_component_stubs()


@pytest.fixture(scope="session")
def live_modbus_registers() -> dict[str, object]:
    fixture_path = REPO_ROOT / "tests" / "fixtures" / "live_modbus_registers.json"
    return json.loads(fixture_path.read_text())
