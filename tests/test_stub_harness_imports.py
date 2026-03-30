from __future__ import annotations

import importlib
from pathlib import Path

integration = importlib.import_module("custom_components.fronius_modbus")


def test_stub_harness_imports_the_production_package_object() -> None:
    assert Path(integration.__file__).resolve() == (
        Path(__file__).resolve().parents[1]
        / "custom_components"
        / "fronius_modbus"
        / "__init__.py"
    )
