from __future__ import annotations

from homeassistant.helpers.entity import Entity


class SensorDeviceClass:
    CURRENT = "current"
    POWER = "power"
    ENERGY = "energy"
    FREQUENCY = "frequency"
    VOLTAGE = "voltage"
    TEMPERATURE = "temperature"
    BATTERY = "battery"
    ENUM = "enum"


class SensorStateClass:
    MEASUREMENT = "measurement"
    TOTAL_INCREASING = "total_increasing"


class SensorEntity(Entity):
    """Sensor entity stub with lightweight state validation."""
