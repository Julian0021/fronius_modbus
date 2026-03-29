from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
STUB_ROOT = REPO_ROOT / "tests" / "stubs"
CUSTOM_COMPONENTS_ROOT = REPO_ROOT / "custom_components"

for path in (str(STUB_ROOT), str(REPO_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)


def _configure_custom_component_namespace() -> None:
    import custom_components

    repo_path = str(CUSTOM_COMPONENTS_ROOT)
    package_paths = getattr(custom_components, "__path__", None)
    if package_paths is None:
        raise RuntimeError("custom_components package does not define __path__")
    if repo_path not in package_paths:
        package_paths.append(repo_path)


_configure_custom_component_namespace()


def _clear_translation_cache() -> None:
    module = sys.modules.get("custom_components.fronius_modbus.translation")
    if module is None:
        return
    cache = getattr(module, "_TRANSLATION_CACHE", None)
    if isinstance(cache, dict):
        cache.clear()


def _clear_storage_stub() -> None:
    from homeassistant.helpers.storage import Store

    Store.reset_storage()


@pytest.fixture(autouse=True)
def reset_runtime_caches() -> None:
    _clear_translation_cache()
    _clear_storage_stub()
    yield
    _clear_translation_cache()
    _clear_storage_stub()


@pytest.fixture(scope="session")
def live_modbus_registers() -> dict[str, object]:
    fixture_path = REPO_ROOT / "tests" / "fixtures" / "live_modbus_registers.json"
    return json.loads(fixture_path.read_text())
