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


@dataclass(frozen=True, slots=True)
class StorageModeReadback:
    extended_mode: StorageExtendedControlMode | None
    control_mode_label: str | None
    charge_status_label: str | None
    charge_limit: float
    discharge_limit: float
    grid_charge_power: float
    grid_discharge_power: float


STORAGE_MODE_POLICIES: dict[StorageExtendedControlMode, StorageModePolicy] = {
    StorageExtendedControlMode.AUTO: StorageModePolicy(
        label="auto",
        modbus_control_mode=0,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Auto mode",
        capabilities=frozenset(),
    ),
    StorageExtendedControlMode.PV_CHARGE_LIMIT: StorageModePolicy(
        label="pv_charge_limit",
        modbus_control_mode=1,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Set charge mode",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_CHARGE_LIMIT}),
    ),
    StorageExtendedControlMode.DISCHARGE_LIMIT: StorageModePolicy(
        label="discharge_limit",
        modbus_control_mode=2,
        default_charge_limit=100,
        default_discharge_limit=100,
        log_message="Set discharge mode",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT}),
    ),
    StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT: StorageModePolicy(
        label="pv_charge_and_discharge_limit",
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
        label="charge_from_grid",
        modbus_control_mode=1,
        default_charge_limit=100,
        default_discharge_limit=0,
        log_message="Charge from grid enabled",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER}),
    ),
    StorageExtendedControlMode.DISCHARGE_TO_GRID: StorageModePolicy(
        label="discharge_to_grid",
        modbus_control_mode=2,
        default_charge_limit=0,
        default_discharge_limit=100,
        log_message="Discharge to grid enabled",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER}),
    ),
    StorageExtendedControlMode.BLOCK_DISCHARGING: StorageModePolicy(
        label="block_discharging",
        modbus_control_mode=3,
        default_charge_limit=100,
        default_discharge_limit=0,
        log_message="blocked discharging",
        capabilities=frozenset({STORAGE_MODE_CAPABILITY_CHARGE_LIMIT}),
    ),
    StorageExtendedControlMode.BLOCK_CHARGING: StorageModePolicy(
        label="block_charging",
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
    if storage_control_mode in (1, 2, 3) and discharge_power is not None and discharge_power < 0:
        return StorageExtendedControlMode.CHARGE_FROM_GRID
    if storage_control_mode in (1, 2, 3) and charge_power is not None and charge_power < 0:
        return StorageExtendedControlMode.DISCHARGE_TO_GRID
    if storage_control_mode == 1 and discharge_power == 0:
        return StorageExtendedControlMode.CHARGE_FROM_GRID
    if storage_control_mode == 2 and charge_power == 0:
        return StorageExtendedControlMode.DISCHARGE_TO_GRID
    if storage_control_mode == 1:
        return StorageExtendedControlMode.PV_CHARGE_LIMIT
    if storage_control_mode == 2 and discharge_power == 0 and charge_grid_enabled:
        return StorageExtendedControlMode.CHARGE_FROM_GRID
    if storage_control_mode == 3 and discharge_power == 0:
        return StorageExtendedControlMode.BLOCK_DISCHARGING
    if storage_control_mode == 3 and charge_power == 0:
        return StorageExtendedControlMode.BLOCK_CHARGING
    if storage_control_mode == 2:
        return StorageExtendedControlMode.DISCHARGE_LIMIT
    if storage_control_mode == 3:
        return StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT
    return None


def _split_signed_power(raw_value: int | None) -> tuple[float, float]:
    if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
        return 0.0, 0.0
    if raw_value >= 0:
        return raw_value / 100.0, 0.0
    return 0.0, (raw_value * -1) / 100.0


def _normalize_charge_status_label(
    mode: StorageExtendedControlMode | None,
    charge_status_label: str | None,
) -> str | None:
    if (
        mode == StorageExtendedControlMode.CHARGE_FROM_GRID
        and charge_status_label == "discharging"
    ):
        return "charging"
    if (
        mode == StorageExtendedControlMode.DISCHARGE_TO_GRID
        and charge_status_label == "charging"
    ):
        return "discharging"
    return charge_status_label


def _normalize_control_mode_label(
    mode: StorageExtendedControlMode | None,
    control_mode_label: str | None,
) -> str | None:
    if (
        mode == StorageExtendedControlMode.CHARGE_FROM_GRID
        and control_mode_label == "discharge"
    ):
        return "charge"
    if (
        mode == StorageExtendedControlMode.DISCHARGE_TO_GRID
        and control_mode_label == "charge"
    ):
        return "discharge"
    return control_mode_label


def derive_storage_mode_readback(
    storage_control_mode: int | None,
    *,
    charge_power: int | None,
    discharge_power: int | None,
    charge_grid_enabled: bool | None = None,
    control_mode_label: str | None = None,
    charge_status_label: str | None = None,
) -> StorageModeReadback:
    """Normalize the storage readback state from raw registers and mapped labels."""
    extended_mode = derive_storage_extended_mode(
        storage_control_mode,
        charge_power=charge_power,
        discharge_power=discharge_power,
        charge_grid_enabled=charge_grid_enabled,
    )
    discharge_limit, grid_charge_power = _split_signed_power(discharge_power)
    charge_limit, grid_discharge_power = _split_signed_power(charge_power)
    return StorageModeReadback(
        extended_mode=extended_mode,
        control_mode_label=_normalize_control_mode_label(
            extended_mode,
            control_mode_label,
        ),
        charge_status_label=_normalize_charge_status_label(
            extended_mode,
            charge_status_label,
        ),
        charge_limit=charge_limit,
        discharge_limit=discharge_limit,
        grid_charge_power=grid_charge_power,
        grid_discharge_power=grid_discharge_power,
    )
