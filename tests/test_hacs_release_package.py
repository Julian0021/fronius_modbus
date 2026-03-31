from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

REPO_ROOT = Path(__file__).resolve().parents[1]
STUB_ROOT = REPO_ROOT / "tests" / "stubs"
BUILD_SCRIPT = REPO_ROOT / "scripts" / "build_hacs_release.sh"


def _build_release_package(output_dir: Path) -> Path:
    subprocess.run(
        ["bash", str(BUILD_SCRIPT), str(output_dir)],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    archive_name = json.loads((REPO_ROOT / "hacs.json").read_text(encoding="utf-8"))[
        "filename"
    ]
    archive_path = output_dir / archive_name
    assert archive_path.is_file()
    return archive_path


def test_hacs_release_package_imports_from_installed_layout(tmp_path: Path) -> None:
    archive_path = _build_release_package(tmp_path / "dist")
    install_root = tmp_path / "installed"
    integration_root = install_root / "custom_components" / "fronius_modbus"
    integration_root.mkdir(parents=True)

    with zipfile.ZipFile(archive_path) as archive:
        archive.extractall(integration_root)

    manifest = json.loads((integration_root / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["domain"] == "fronius_modbus"

    smoke_code = """
import importlib
import json
import sys
import types
from pathlib import Path

install_root = Path(sys.argv[1]).resolve()
expected_root = install_root / "custom_components" / "fronius_modbus"
custom_components_root = install_root / "custom_components"

custom_components = types.ModuleType("custom_components")
custom_components.__path__ = [str(custom_components_root)]
sys.modules["custom_components"] = custom_components

package = importlib.import_module("custom_components.fronius_modbus")
config_flow = importlib.import_module("custom_components.fronius_modbus.config_flow")

package_path = Path(package.__file__).resolve().parent
config_flow_path = Path(config_flow.__file__).resolve().parent

if package_path != expected_root:
    raise SystemExit(f"package imported from unexpected path: {package_path}")
if config_flow_path != expected_root:
    raise SystemExit(f"config_flow imported from unexpected path: {config_flow_path}")

print(json.dumps({"package": str(package_path), "config_flow": str(config_flow_path)}))
"""

    env = os.environ.copy()
    env["PYTHONPATH"] = str(STUB_ROOT)

    result = subprocess.run(
        [sys.executable, "-c", smoke_code, str(install_root)],
        cwd=tmp_path,
        env=env,
        check=True,
        capture_output=True,
        text=True,
    )

    imported_paths = json.loads(result.stdout)
    assert imported_paths == {
        "package": str(integration_root),
        "config_flow": str(integration_root),
    }
