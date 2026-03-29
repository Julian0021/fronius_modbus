from __future__ import annotations

CONF_HOST = "host"
CONF_NAME = "name"
CONF_PORT = "port"
CONF_SCAN_INTERVAL = "scan_interval"
CONF_RESTRICT_MODBUS_TO_THIS_IP = "restrict_modbus_to_this_ip"


class Platform:
    SELECT = "select"
    SWITCH = "switch"
    NUMBER = "number"
    SENSOR = "sensor"
    BUTTON = "button"
