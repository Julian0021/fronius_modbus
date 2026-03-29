from __future__ import annotations


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


class SensorEntity:
    """Minimal sensor entity stub."""
