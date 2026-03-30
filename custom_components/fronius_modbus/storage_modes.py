"""Shared storage-mode policy and helpers."""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

STORAGE_MODE_CAPABILITY_CHARGE_LIMIT = "charge_limit"
STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT = "discharge_limit"
STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER = "grid_charge_power"
STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER = "grid_discharge_power"


class StorageExtendedControlMode(IntEnum):
    AUTO = 0
    PV_CHARGE_LIMIT = 1
    DISCHARGE_LIMIT = 2
    PV_CHARGE_AND_DISCHARGE_LIMIT = 3
    CHARGE_FROM_GRID = 4
    DISCHARGE_TO_GRID = 5
    BLOCK_DISCHARGING = 6
    BLOCK_CHARGING = 7


@dataclass(frozen=True, slots=True)
class StorageModePolicy:
    label: str
    modbus_control_mode: int
    default_charge_limit: int
    default_discharge_limit: int
    log_message: str
    capabilities: frozenset[str]


STORAGE_MODE_POLICIES: dict[StorageExtendedControlMode, StorageModePolicy] = {
    StorageExtendedControlMode.AUTO: StorageModePolicy(
        label="Auto",
        modbus_control_mode=0,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Auto mode",
        capabilities=frozenset(),
    ),
    StorageExtendedControlMode.PV_CHARGE_LIMIT: StorageModePolicy(
        label="PV Charge Limit",
        modbus_control_mode=1,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Set charge mode",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_CHARGE_LIMIT}),
    ),
    StorageExtendedControlMode.DISCHARGE_LIMIT: StorageModePolicy(
        label="Discharge Limit",
        modbus_control_mode=2,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Set discharge mode",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT}),
    ),
    StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT: StorageModePolicy(
        label="PV Charge and Discharge Limit",
        modbus_control_mode=3,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Set charge/discharge mode.",
        capabilities=frozenset(
            {
                STORAGE_MODE_CAPABILITY_CHARGE_LIMIT,
                STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT,
            }
        ),
    ),
    StorageExtendedControlMode.CHARGE_FROM_GRID: StorageModePolicy(
        label="Charge from Grid",
        modbus_control_mode=2,
        default_charge_limit=100,
        default_discharge_limit=0,
        log_message="Charge from grid enabled",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER}),
    ),
    StorageExtendedControlMode.DISCHARGE_TO_GRID: StorageModePolicy(
        label="Discharge to Grid",
        modbus_control_mode=1,
        default_charge_limit=0,
        default_discharge_limit=100,
        log_message="Discharge to grid enabled",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER}),
    ),
    StorageExtendedControlMode.BLOCK_DISCHARGING: StorageModePolicy(
        label="Block Discharging",
        modbus_control_mode=3,
        default_charge_limit=100,
        default_discharge_limit=0,
        log_message="blocked discharging",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_CHARGE_LIMIT}),
    ),
    StorageExtendedControlMode.BLOCK_CHARGING: StorageModePolicy(
        label="Block Charging",
        modbus_control_mode=3,
        default_charge_limit=0,
        default_discharge_limit=100,
        log_message="Block charging at 100",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT}),
    ),
}

STORAGE_MODE_OPTIONS = {
    int(mode): policy.label for mode, policy in STORAGE_MODE_POLICIES.items()
}

SUPPORTED_STORAGE_CONTROL_MODES = frozenset(
    policy.modbus_control_mode for policy in STORAGE_MODE_POLICIES.values()
)


def get_storage_mode_policy(
    mode: int | StorageExtendedControlMode | None,
) -> StorageModePolicy | None:
    """Return the shared policy for an extended storage mode value."""
    if mode is None:
        return None
    try:
        resolved_mode = StorageExtendedControlMode(int(mode))
    except (TypeError, ValueError):
        return None
    return STORAGE_MODE_POLICIES.get(resolved_mode)


def storage_modes_for_capability(capability: str) -> tuple[int, ...]:
    """Return the extended storage modes that expose a given control."""
    return tuple(
        int(mode)
        for mode, policy in STORAGE_MODE_POLICIES.items()
        if capability in policy.capabilities
    )


def storage_mode_supports(
    mode: int | StorageExtendedControlMode | None,
    capability: str,
) -> bool:
    """Return whether the extended storage mode exposes a capability."""
    policy = get_storage_mode_policy(mode)
    return policy is not None and capability in policy.capabilities


def derive_storage_extended_mode(
    storage_control_mode: int | None,
    *,
    charge_power: int | None,
    discharge_power: int | None,
    charge_grid_enabled: bool | None = None,
) -> StorageExtendedControlMode | None:
    """Map raw storage-control registers to the exposed extended mode."""
    if storage_control_mode == 0:
        return StorageExtendedControlMode.AUTO
    if storage_control_mode in (1, 3) and charge_power == 0:
        return StorageExtendedControlMode.BLOCK_CHARGING
    if storage_control_mode == 1:
        return StorageExtendedControlMode.PV_CHARGE_LIMIT
    if storage_control_mode in (2, 3) and discharge_power is not None and discharge_power < 0:
        return StorageExtendedControlMode.CHARGE_FROM_GRID
    if storage_control_mode in (2, 3) and charge_power is not None and charge_power < 0:
        return StorageExtendedControlMode.DISCHARGE_TO_GRID
    if storage_control_mode == 2 and discharge_power == 0 and charge_grid_enabled:
        return StorageExtendedControlMode.CHARGE_FROM_GRID
    if storage_control_mode in (2, 3) and discharge_power == 0:
        return StorageExtendedControlMode.BLOCK_DISCHARGING
    if storage_control_mode == 2:
        return StorageExtendedControlMode.DISCHARGE_LIMIT
    if storage_control_mode == 3:
        return StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT
    return None
