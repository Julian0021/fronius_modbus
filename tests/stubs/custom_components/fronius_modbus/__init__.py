from __future__ import annotations

from pathlib import Path

from homeassistant.config_entries import ConfigEntry

HubConfigEntry = ConfigEntry
__path__ = [
    str(Path(__file__).resolve().parents[4] / "custom_components" / "fronius_modbus")
]

__all__ = ["HubConfigEntry"]
