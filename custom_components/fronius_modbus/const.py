from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal, TypeAlias

from homeassistant.components.sensor import SensorDeviceClass, SensorStateClass
from homeassistant.helpers.entity import EntityCategory

from .froniusmodbusclient_const import (
    AC_LIMIT_STATUS,
    CHARGE_GRID_STATUS,
    CHARGE_STATUS,
    CONNECTION_STATUS_CONDENSED,
    CONTROL_STATUS,
    ECP_CONNECTION_STATUS,
    FRONIUS_INVERTER_STATUS,
    GRID_STATUS,
    INVERTER_STATUS,
    STORAGE_CONTROL_MODE,
)
from .storage_modes import (
    STORAGE_MODE_CAPABILITY_CHARGE_LIMIT,
    STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT,
    STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
    STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER,
    STORAGE_MODE_OPTIONS,
)

DOMAIN = "fronius_modbus"
CONNECTION_MODBUS = "modbus"
DEFAULT_NAME = "Fronius"
ENTITY_PREFIX = "fm"
DEFAULT_SCAN_INTERVAL = 10
DEFAULT_PORT = 502
DEFAULT_INVERTER_UNIT_ID = 1
DEFAULT_METER_UNIT_ID = 200
DEFAULT_METER_UNIT_IDS = [DEFAULT_METER_UNIT_ID]
DEFAULT_AUTO_ENABLE_MODBUS = True
DEFAULT_RESTRICT_MODBUS_TO_THIS_IP = False
API_USERNAME = "customer"
CONF_RECONFIGURE_REQUIRED = "_reconfigure_required"
MIGRATION_RECONFIGURE_ISSUE_ID_PREFIX = "legacy_modbus_only_reconfigure_"
SOLAR_API_LOW_FIRMWARE_ISSUE_ID_PREFIX = "solar_api_low_firmware_"
CONF_INVERTER_UNIT_ID = "inverter_modbus_unit_id"
CONF_METER_UNIT_ID = "meter_modbus_unit_id"
CONF_METER_UNIT_IDS = "meter_modbus_unit_ids"
CONF_API_USERNAME = "api_username"
CONF_API_PASSWORD = "api_password"
CONF_AUTO_ENABLE_MODBUS = "auto_enable_modbus"
CONF_RESTRICT_MODBUS_TO_THIS_IP = "restrict_modbus_to_this_ip"
ATTR_MANUFACTURER = "Fronius"
SUPPORTED_MANUFACTURERS = ["Fronius"]
SUPPORTED_MODELS = ["Primo GEN24", "Symo GEN24", "Verto"]

AvailabilityPolicy: TypeAlias = Literal[
    "always",
    "web_api",
    "storage_web_api",
    "data_not_none",
    "web_api_manual_mode",
    "storage_mode",
]

DescriptorServiceName: TypeAlias = Literal[
    "command_service",
    "web_api_service",
]

DisplayScaleName: TypeAlias = Literal[
    "charge_rate",
    "discharge_rate",
]

ValueTransformName: TypeAlias = Literal["round_int"]

AVAILABILITY_POLICIES: tuple[AvailabilityPolicy, ...] = (
    "always",
    "web_api",
    "storage_web_api",
    "data_not_none",
    "web_api_manual_mode",
    "storage_mode",
)

DESCRIPTOR_SERVICE_NAMES: tuple[DescriptorServiceName, ...] = (
    "command_service",
    "web_api_service",
)

DISPLAY_SCALE_NAMES: tuple[DisplayScaleName, ...] = (
    "charge_rate",
    "discharge_rate",
)

VALUE_TRANSFORM_NAMES: tuple[ValueTransformName, ...] = ("round_int",)


@dataclass(frozen=True, slots=True)
class SelectEntitySpec:
    translation_key: str
    key: str
    options: dict[int, str]
    action: str | None = None
    availability: AvailabilityPolicy = "always"
    action_service: DescriptorServiceName | None = None


@dataclass(frozen=True, slots=True)
class NumberEntitySpec:
    translation_key: str
    key: str
    minimum: float
    maximum: float
    step: float
    mode: str
    unit: str | None
    max_key: str | None = None
    action: str | None = None
    value_transform: ValueTransformName | None = None
    display_scale: DisplayScaleName | None = None
    availability: AvailabilityPolicy = "always"
    mode_capability: str | None = None
    available_modes: tuple[int, ...] | None = None
    action_service: DescriptorServiceName | None = None


@dataclass(frozen=True, slots=True)
class ToggleEntitySpec:
    translation_key: str
    key: str
    icon: str
    entity_category: EntityCategory | None = None
    turn_on_action: str | None = None
    turn_on_kwargs: dict[str, Any] = field(default_factory=dict)
    turn_off_action: str | None = None
    turn_off_kwargs: dict[str, Any] = field(default_factory=dict)
    availability: AvailabilityPolicy = "always"
    turn_on_service: DescriptorServiceName | None = None
    turn_off_service: DescriptorServiceName | None = None


@dataclass(frozen=True, slots=True)
class ButtonEntitySpec:
    translation_key: str
    key: str
    icon: str
    entity_category: EntityCategory | None = None
    action: str | None = None
    availability: AvailabilityPolicy = "always"
    action_service: DescriptorServiceName | None = None


@dataclass(frozen=True, slots=True)
class SensorEntitySpec:
    translation_key: str
    key: str
    device_class: SensorDeviceClass | None = None
    state_class: SensorStateClass | None = None
    unit: str | None = None
    icon: str | None = None
    entity_category: EntityCategory | None = None


@dataclass(frozen=True, slots=True)
class MpptModuleSensorSpec:
    translation_key: str
    key_suffix: str
    device_class: SensorDeviceClass | None = None
    state_class: SensorStateClass | None = None
    unit: str | None = None
    icon: str | None = None
    entity_category: EntityCategory | None = None


API_BATTERY_MODE = {
    0: "auto",
    1: "manual",
}

API_SOC_MODE = {
    "auto": "Automatic",
    "manual": "Manual",
}

STORAGE_EXT_CONTROL_MODE = STORAGE_MODE_OPTIONS

STORAGE_MODBUS_SELECT_TYPES = (
    SelectEntitySpec(
        "ext_control_mode",
        "ext_control_mode",
        STORAGE_EXT_CONTROL_MODE,
        action="set_mode",
        action_service="command_service",
    ),
)

STORAGE_API_SELECT_TYPES = (
    SelectEntitySpec(
        "api_battery_mode",
        "api_battery_mode",
        API_BATTERY_MODE,
        action="set_api_battery_mode",
        availability="storage_web_api",
        action_service="command_service",
    ),
)

STORAGE_API_SWITCH_TYPES = (
    ToggleEntitySpec(
        "charge_from_ac",
        "api_charge_from_ac",
        "mdi:power-plug-battery",
        turn_on_action="set_api_charge_sources",
        turn_on_kwargs={"charge_from_ac": True},
        turn_off_action="set_api_charge_sources",
        turn_off_kwargs={"charge_from_grid": False, "charge_from_ac": False},
        availability="storage_web_api",
        turn_on_service="command_service",
        turn_off_service="command_service",
    ),
    ToggleEntitySpec(
        "charge_from_grid",
        "api_charge_from_grid",
        "mdi:transmission-tower-export",
        turn_on_action="set_api_charge_sources",
        turn_on_kwargs={"charge_from_grid": True, "charge_from_ac": True},
        turn_off_action="set_api_charge_sources",
        turn_off_kwargs={"charge_from_grid": False},
        availability="storage_web_api",
        turn_on_service="command_service",
        turn_off_service="command_service",
    ),
)

STORAGE_MODBUS_NUMBER_TYPES = (
    NumberEntitySpec(
        "grid_discharge_power",
        "grid_discharge_power",
        minimum=0,
        maximum=10100,
        step=10,
        mode="box",
        unit="W",
        max_key="MaxDisChaRte",
        action="set_grid_discharge_power",
        display_scale="discharge_rate",
        availability="storage_mode",
        mode_capability=STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER,
        action_service="command_service",
    ),
    NumberEntitySpec(
        "grid_charge_power",
        "grid_charge_power",
        minimum=0,
        maximum=10100,
        step=10,
        mode="box",
        unit="W",
        max_key="MaxChaRte",
        action="set_grid_charge_power",
        display_scale="charge_rate",
        availability="storage_mode",
        mode_capability=STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
        action_service="command_service",
    ),
    NumberEntitySpec(
        "discharge_limit",
        "discharge_limit",
        minimum=0,
        maximum=10100,
        step=10,
        mode="box",
        unit="W",
        max_key="MaxDisChaRte",
        action="set_discharge_limit",
        display_scale="discharge_rate",
        availability="storage_mode",
        mode_capability=STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT,
        action_service="command_service",
    ),
    NumberEntitySpec(
        "charge_limit",
        "charge_limit",
        minimum=0,
        maximum=10100,
        step=10,
        mode="box",
        unit="W",
        max_key="MaxChaRte",
        action="set_charge_limit",
        display_scale="charge_rate",
        availability="storage_mode",
        mode_capability=STORAGE_MODE_CAPABILITY_CHARGE_LIMIT,
        action_service="command_service",
    ),
    NumberEntitySpec(
        "soc_minimum",
        "soc_minimum",
        minimum=5,
        maximum=100,
        step=1,
        mode="box",
        unit="%",
        action="set_soc_minimum",
        action_service="command_service",
    ),
)

STORAGE_API_NUMBER_TYPES = (
    NumberEntitySpec(
        "api_battery_power",
        "api_battery_power",
        minimum=-20000,
        maximum=20000,
        step=10,
        mode="box",
        unit="W",
        action="set_api_battery_power",
        availability="web_api_manual_mode",
        action_service="command_service",
    ),
    NumberEntitySpec(
        "soc_maximum",
        "soc_maximum",
        minimum=0,
        maximum=100,
        step=1,
        mode="box",
        unit="%",
        action="set_api_soc_values",
        value_transform="round_int",
        availability="web_api_manual_mode",
        action_service="command_service",
    ),
)

INVERTER_NUMBER_TYPES = (
    NumberEntitySpec(
        "ac_limit_rate",
        "ac_limit_rate",
        minimum=0,
        maximum=50000,
        step=10,
        mode="box",
        unit="W",
        max_key="max_power",
        action="set_ac_limit_rate",
        action_service="command_service",
    ),
    NumberEntitySpec(
        "power_factor",
        "power_factor",
        minimum=-1,
        maximum=1,
        step=0.001,
        mode="box",
        unit=None,
        action="set_power_factor",
        availability="data_not_none",
        action_service="command_service",
    ),
)

INVERTER_SELECT_TYPES = (
    SelectEntitySpec(
        "ac_limit_enable",
        "ac_limit_enable",
        {0: "disabled", 1: "enabled"},
        action="set_ac_limit_enable",
        action_service="command_service",
    ),
    SelectEntitySpec(
        "power_factor_enable",
        "power_factor_enable",
        {0: "disabled", 1: "enabled"},
        action="set_power_factor_enable",
        availability="data_not_none",
        action_service="command_service",
    ),
    SelectEntitySpec(
        "conn",
        "Conn",
        {0: "disabled", 1: "enabled"},
        action="set_conn_status",
        action_service="command_service",
    ),
)

INVERTER_API_SWITCH_TYPES = (
    ToggleEntitySpec(
        "api_solar_api_enabled",
        "api_solar_api_enabled",
        "mdi:api",
        EntityCategory.DIAGNOSTIC,
        turn_on_action="set_solar_api_enabled",
        turn_on_kwargs={"enabled": True},
        turn_off_action="set_solar_api_enabled",
        turn_off_kwargs={"enabled": False},
        availability="web_api",
        turn_on_service="web_api_service",
        turn_off_service="web_api_service",
    ),
)

INVERTER_API_BUTTON_TYPES = (
    ButtonEntitySpec(
        "reset_modbus_control",
        "reset_modbus_control",
        "mdi:restart",
        EntityCategory.DIAGNOSTIC,
        action="reset_modbus_control",
        availability="web_api",
        action_service="web_api_service",
    ),
)

ENTITY_DESCRIPTOR_COLLECTIONS: tuple[tuple[Any, ...], ...] = (
    STORAGE_MODBUS_SELECT_TYPES,
    STORAGE_API_SELECT_TYPES,
    STORAGE_API_SWITCH_TYPES,
    STORAGE_MODBUS_NUMBER_TYPES,
    STORAGE_API_NUMBER_TYPES,
    INVERTER_NUMBER_TYPES,
    INVERTER_SELECT_TYPES,
    INVERTER_API_SWITCH_TYPES,
    INVERTER_API_BUTTON_TYPES,
)

INVERTER_SENSOR_TYPES = {
    "A": SensorEntitySpec("a", "A", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "AphA": SensorEntitySpec("apha", "AphA", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "acpower": SensorEntitySpec("acpower", "acpower", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "var": SensorEntitySpec("var", "var", None, SensorStateClass.MEASUREMENT, "var", "mdi:sine-wave"),
    "acenergy": SensorEntitySpec("acenergy", "acenergy", SensorDeviceClass.ENERGY, SensorStateClass.TOTAL_INCREASING, "Wh", "mdi:lightning-bolt"),
    "pv_power": SensorEntitySpec("pv_power", "pv_power", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:solar-power"),
    "load": SensorEntitySpec("load", "load", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "pv_connection": SensorEntitySpec("pv_connection", "pv_connection", entity_category=EntityCategory.DIAGNOSTIC),
    "ecp_connection": SensorEntitySpec("ecp_connection", "ecp_connection", entity_category=EntityCategory.DIAGNOSTIC),
    "status": SensorEntitySpec("status", "status", entity_category=EntityCategory.DIAGNOSTIC),
    "statusvendor": SensorEntitySpec("statusvendor", "statusvendor", entity_category=EntityCategory.DIAGNOSTIC),
    "line_frequency": SensorEntitySpec("line_frequency", "line_frequency", SensorDeviceClass.FREQUENCY, SensorStateClass.MEASUREMENT, "Hz"),
    "inverter_controls": SensorEntitySpec("inverter_controls", "inverter_controls", entity_category=EntityCategory.DIAGNOSTIC),
    "vref": SensorEntitySpec("vref", "vref", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt", EntityCategory.DIAGNOSTIC),
    "vrefofs": SensorEntitySpec("vrefofs", "vrefofs", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt", EntityCategory.DIAGNOSTIC),
    "max_power": SensorEntitySpec("max_power", "max_power", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "events2": SensorEntitySpec("events2", "events2", entity_category=EntityCategory.DIAGNOSTIC),
    "grid_status": SensorEntitySpec("grid_status", "grid_status", entity_category=EntityCategory.DIAGNOSTIC),
    "Conn": SensorEntitySpec("conn", "Conn", entity_category=EntityCategory.DIAGNOSTIC),
    "WMaxLim_Ena": SensorEntitySpec("wmaxlim_ena", "WMaxLim_Ena", entity_category=EntityCategory.DIAGNOSTIC),
    "OutPFSet_Ena": SensorEntitySpec("outpfset_ena", "OutPFSet_Ena", entity_category=EntityCategory.DIAGNOSTIC),
    "VArPct_Ena": SensorEntitySpec("varpct_ena", "VArPct_Ena", entity_category=EntityCategory.DIAGNOSTIC),
    "PhVphA": SensorEntitySpec("phvpha", "PhVphA", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "unit_id": SensorEntitySpec("unit_id", "i_unit_id", entity_category=EntityCategory.DIAGNOSTIC),
    "ac_limit_rate": SensorEntitySpec("ac_limit_rate", "ac_limit_rate", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:chart-line"),
    "ac_limit_enable": SensorEntitySpec("ac_limit_enable", "ac_limit_enable", icon="mdi:power-plug", entity_category=EntityCategory.DIAGNOSTIC),
    "isolation_resistance": SensorEntitySpec("isolation_resistance", "isolation_resistance", None, SensorStateClass.MEASUREMENT, "MΩ", "mdi:omega"),
}

INVERTER_WEB_SENSOR_TYPES = {
    "inverter_temperature": SensorEntitySpec("inverter_temperature", "inverter_temperature", SensorDeviceClass.TEMPERATURE, SensorStateClass.MEASUREMENT, "°C", "mdi:thermometer"),
    "api_modbus_mode": SensorEntitySpec("api_modbus_mode", "api_modbus_mode", entity_category=EntityCategory.DIAGNOSTIC),
    "api_modbus_control": SensorEntitySpec("api_modbus_control", "api_modbus_control", entity_category=EntityCategory.DIAGNOSTIC),
    "api_modbus_sunspec_mode": SensorEntitySpec("api_modbus_sunspec_mode", "api_modbus_sunspec_mode", entity_category=EntityCategory.DIAGNOSTIC),
    "api_modbus_restriction": SensorEntitySpec("api_modbus_restriction", "api_modbus_restriction", entity_category=EntityCategory.DIAGNOSTIC),
    "api_modbus_restriction_ip": SensorEntitySpec("api_modbus_restriction_ip", "api_modbus_restriction_ip", entity_category=EntityCategory.DIAGNOSTIC),
}

MPPT_MODULE_SENSOR_TYPES = (
    MpptModuleSensorSpec("mppt_module_dc_current", "dc_current", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-dc"),
    MpptModuleSensorSpec("mppt_module_dc_voltage", "dc_voltage", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    MpptModuleSensorSpec("mppt_module_dc_power", "dc_power", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:solar-power"),
    MpptModuleSensorSpec("mppt_module_lifetime_energy", "lifetime_energy", SensorDeviceClass.ENERGY, SensorStateClass.TOTAL_INCREASING, "Wh", "mdi:solar-panel"),
)

INVERTER_SYMO_SENSOR_TYPES = {
    "AphB": SensorEntitySpec("aphb", "AphB", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "AphC": SensorEntitySpec("aphc", "AphC", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "PhVphB": SensorEntitySpec("phvphb", "PhVphB", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PhVphC": SensorEntitySpec("phvphc", "PhVphC", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PPVphAB": SensorEntitySpec("ppvphab", "PPVphAB", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PPVphBC": SensorEntitySpec("ppvphbc", "PPVphBC", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PPVphCA": SensorEntitySpec("ppvphca", "PPVphCA", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
}

INVERTER_STORAGE_SENSOR_TYPES = {
    "storage_charge_current": SensorEntitySpec("storage_charge_current", "storage_charge_current", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-dc"),
    "storage_charge_voltage": SensorEntitySpec("storage_charge_voltage", "storage_charge_voltage", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "storage_charge_power": SensorEntitySpec("storage_charge_power", "storage_charge_power", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:home-battery"),
    "storage_charge_lfte": SensorEntitySpec("storage_charge_lfte", "storage_charge_lfte", SensorDeviceClass.ENERGY, SensorStateClass.TOTAL_INCREASING, "Wh", "mdi:home-battery"),
    "storage_discharge_current": SensorEntitySpec("storage_discharge_current", "storage_discharge_current", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-dc"),
    "storage_discharge_voltage": SensorEntitySpec("storage_discharge_voltage", "storage_discharge_voltage", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "storage_discharge_power": SensorEntitySpec("storage_discharge_power", "storage_discharge_power", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:home-battery"),
    "storage_discharge_lfte": SensorEntitySpec("storage_discharge_lfte", "storage_discharge_lfte", SensorDeviceClass.ENERGY, SensorStateClass.TOTAL_INCREASING, "Wh", "mdi:home-battery"),
    "storage_connection": SensorEntitySpec("storage_connection", "storage_connection", entity_category=EntityCategory.DIAGNOSTIC),
}

METER_SENSOR_TYPES = {
    "A": SensorEntitySpec("a", "A", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "AphA": SensorEntitySpec("apha", "AphA", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "AphB": SensorEntitySpec("aphb", "AphB", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "AphC": SensorEntitySpec("aphc", "AphC", SensorDeviceClass.CURRENT, SensorStateClass.MEASUREMENT, "A", "mdi:current-ac"),
    "power": SensorEntitySpec("power", "power", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "WphA": SensorEntitySpec("wpha", "WphA", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "WphB": SensorEntitySpec("wphb", "WphB", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "WphC": SensorEntitySpec("wphc", "WphC", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", "mdi:lightning-bolt"),
    "exported": SensorEntitySpec("exported", "exported", SensorDeviceClass.ENERGY, SensorStateClass.TOTAL_INCREASING, "Wh", "mdi:lightning-bolt"),
    "imported": SensorEntitySpec("imported", "imported", SensorDeviceClass.ENERGY, SensorStateClass.TOTAL_INCREASING, "Wh", "mdi:lightning-bolt"),
    "line_frequency": SensorEntitySpec("line_frequency", "line_frequency", SensorDeviceClass.FREQUENCY, SensorStateClass.MEASUREMENT, "Hz"),
    "PhVphA": SensorEntitySpec("phvpha", "PhVphA", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PhVphB": SensorEntitySpec("phvphb", "PhVphB", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PhVphC": SensorEntitySpec("phvphc", "PhVphC", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "PPV": SensorEntitySpec("ppv", "PPV", SensorDeviceClass.VOLTAGE, SensorStateClass.MEASUREMENT, "V", "mdi:lightning-bolt"),
    "unit_id": SensorEntitySpec("unit_id", "unit_id", entity_category=EntityCategory.DIAGNOSTIC),
}

SINGLE_PHASE_UNSUPPORTED_METER_SENSOR_KEYS = (
    "AphB",
    "AphC",
    "WphB",
    "WphC",
    "PhVphB",
    "PhVphC",
    "PPV",
)

STORAGE_SENSOR_TYPES = {
    "storage_temperature": SensorEntitySpec("storage_temperature", "storage_temperature", SensorDeviceClass.TEMPERATURE, SensorStateClass.MEASUREMENT, "°C", "mdi:thermometer"),
    "control_mode": SensorEntitySpec("control_mode", "control_mode", entity_category=EntityCategory.DIAGNOSTIC),
    "charge_status": SensorEntitySpec("charge_status", "charge_status", entity_category=EntityCategory.DIAGNOSTIC),
    "max_charge": SensorEntitySpec("max_charge", "max_charge", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", entity_category=EntityCategory.DIAGNOSTIC),
    "soc": SensorEntitySpec("soc", "soc", SensorDeviceClass.BATTERY, SensorStateClass.MEASUREMENT, "%"),
    "charging_power": SensorEntitySpec("charging_power", "charging_power", unit="%", icon="mdi:gauge", entity_category=EntityCategory.DIAGNOSTIC),
    "discharging_power": SensorEntitySpec("discharging_power", "discharging_power", unit="%", icon="mdi:gauge", entity_category=EntityCategory.DIAGNOSTIC),
    "soc_minimum": SensorEntitySpec("soc_minimum", "soc_minimum", unit="%", icon="mdi:gauge"),
    "grid_charging": SensorEntitySpec("grid_charging", "grid_charging", entity_category=EntityCategory.DIAGNOSTIC),
    "WHRtg": SensorEntitySpec("whrtg", "WHRtg", SensorDeviceClass.ENERGY, None, "Wh", entity_category=EntityCategory.DIAGNOSTIC),
    "MaxChaRte": SensorEntitySpec("maxcharte", "MaxChaRte", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", entity_category=EntityCategory.DIAGNOSTIC),
    "MaxDisChaRte": SensorEntitySpec("maxdischarte", "MaxDisChaRte", SensorDeviceClass.POWER, SensorStateClass.MEASUREMENT, "W", entity_category=EntityCategory.DIAGNOSTIC),
}


def _state_values(*mappings: dict[Any, str]) -> list[str]:
    return list(dict.fromkeys(value for mapping in mappings for value in mapping.values()))


INVERTER_CONTROL_STATE_VALUES = [
    "Normal",
    "Power reduction",
    "Constant reactive power",
    "Constant power factor",
    "Power reduction,Constant reactive power",
    "Power reduction,Constant power factor",
    "Constant reactive power,Constant power factor",
    "Power reduction,Constant reactive power,Constant power factor",
]

SENSOR_STATE_OPTIONS = {
    "pv_connection": _state_values(CONNECTION_STATUS_CONDENSED),
    "storage_connection": _state_values(CONNECTION_STATUS_CONDENSED),
    "ecp_connection": _state_values(ECP_CONNECTION_STATUS),
    "status": _state_values(INVERTER_STATUS),
    "statusvendor": _state_values(FRONIUS_INVERTER_STATUS),
    "grid_status": _state_values(GRID_STATUS),
    "conn": _state_values(CONTROL_STATUS),
    "wmaxlim_ena": _state_values(CONTROL_STATUS),
    "outpfset_ena": _state_values(CONTROL_STATUS),
    "varpct_ena": _state_values(CONTROL_STATUS),
    "ac_limit_enable": _state_values(AC_LIMIT_STATUS, {2: "unknown"}),
    "control_mode": _state_values(STORAGE_CONTROL_MODE),
    "charge_status": _state_values(CHARGE_STATUS),
    "grid_charging": _state_values(CHARGE_GRID_STATUS),
    "api_modbus_control": _state_values(CONTROL_STATUS),
    "api_modbus_restriction": _state_values(CONTROL_STATUS),
    "inverter_controls": INVERTER_CONTROL_STATE_VALUES,
}
