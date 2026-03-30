from __future__ import annotations

import pytest

from custom_components.fronius_modbus.storage_modes import (
    STORAGE_MODE_CAPABILITY_CHARGE_LIMIT,
    STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT,
    STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER,
    STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER,
    STORAGE_MODE_OPTIONS,
    StorageExtendedControlMode,
    derive_storage_extended_mode,
    get_storage_mode_policy,
    storage_mode_supports,
    storage_modes_for_capability,
)


def test_storage_mode_capability_sets_match_existing_ui_contract() -> None:
    assert storage_modes_for_capability(STORAGE_MODE_CAPABILITY_CHARGE_LIMIT) == (1, 3, 6)
    assert storage_modes_for_capability(STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT) == (2, 3, 7)
    assert storage_modes_for_capability(STORAGE_MODE_CAPABILITY_GRID_CHARGE_POWER) == (4,)
    assert storage_modes_for_capability(STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER) == (5,)


def test_storage_mode_policy_exposes_select_options_and_web_sync_rules() -> None:
    assert STORAGE_MODE_OPTIONS[0] == "auto"
    assert STORAGE_MODE_OPTIONS[4] == "charge_from_grid"
    assert STORAGE_MODE_OPTIONS[5] == "discharge_to_grid"

    charge_from_grid = get_storage_mode_policy(StorageExtendedControlMode.CHARGE_FROM_GRID)
    assert charge_from_grid is not None
    assert charge_from_grid.modbus_control_mode == 1
    assert charge_from_grid.default_discharge_limit == 0

    discharge_to_grid = get_storage_mode_policy(
        StorageExtendedControlMode.DISCHARGE_TO_GRID
    )
    assert discharge_to_grid is not None
    assert discharge_to_grid.modbus_control_mode == 2
    assert discharge_to_grid.default_charge_limit == 0


def test_storage_mode_supports_checks_capabilities_from_shared_policy() -> None:
    assert storage_mode_supports(6, STORAGE_MODE_CAPABILITY_CHARGE_LIMIT)
    assert not storage_mode_supports(6, STORAGE_MODE_CAPABILITY_DISCHARGE_LIMIT)
    assert storage_mode_supports(5, STORAGE_MODE_CAPABILITY_GRID_DISCHARGE_POWER)


def test_derive_storage_extended_mode_matches_runtime_semantics() -> None:
    assert derive_storage_extended_mode(0, charge_power=100, discharge_power=100) == (
        StorageExtendedControlMode.AUTO
    )
    assert derive_storage_extended_mode(1, charge_power=100, discharge_power=100) == (
        StorageExtendedControlMode.PV_CHARGE_LIMIT
    )
    assert derive_storage_extended_mode(1, charge_power=100, discharge_power=0) == (
        StorageExtendedControlMode.CHARGE_FROM_GRID
    )
    assert derive_storage_extended_mode(2, charge_power=100, discharge_power=100) == (
        StorageExtendedControlMode.DISCHARGE_LIMIT
    )
    assert derive_storage_extended_mode(1, charge_power=100, discharge_power=-25) == (
        StorageExtendedControlMode.CHARGE_FROM_GRID
    )
    assert derive_storage_extended_mode(2, charge_power=0, discharge_power=100) == (
        StorageExtendedControlMode.DISCHARGE_TO_GRID
    )
    assert derive_storage_extended_mode(2, charge_power=-25, discharge_power=100) == (
        StorageExtendedControlMode.DISCHARGE_TO_GRID
    )
    assert derive_storage_extended_mode(
        2,
        charge_power=100,
        discharge_power=0,
        charge_grid_enabled=True,
    ) == (StorageExtendedControlMode.CHARGE_FROM_GRID)
    assert derive_storage_extended_mode(
        2,
        charge_power=100,
        discharge_power=0,
        charge_grid_enabled=False,
    ) == (StorageExtendedControlMode.DISCHARGE_LIMIT)
    assert derive_storage_extended_mode(3, charge_power=100, discharge_power=0) == (
        StorageExtendedControlMode.BLOCK_DISCHARGING
    )
    assert derive_storage_extended_mode(3, charge_power=0, discharge_power=100) == (
        StorageExtendedControlMode.BLOCK_CHARGING
    )
    assert derive_storage_extended_mode(3, charge_power=100, discharge_power=100) == (
        StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT
    )


@pytest.mark.parametrize(
    ("mode", "charge_power", "discharge_power"),
    (
        (StorageExtendedControlMode.AUTO, 100, 100),
        (StorageExtendedControlMode.PV_CHARGE_LIMIT, 100, 100),
        (StorageExtendedControlMode.DISCHARGE_LIMIT, 100, 100),
        (StorageExtendedControlMode.PV_CHARGE_AND_DISCHARGE_LIMIT, 100, 100),
        (StorageExtendedControlMode.CHARGE_FROM_GRID, 100, 0),
        (StorageExtendedControlMode.DISCHARGE_TO_GRID, 0, 100),
        (StorageExtendedControlMode.BLOCK_DISCHARGING, 100, 0),
        (StorageExtendedControlMode.BLOCK_CHARGING, 0, 100),
    ),
)
def test_storage_mode_policy_defaults_round_trip_through_derivation(
    mode: StorageExtendedControlMode,
    charge_power: int,
    discharge_power: int,
) -> None:
    policy = get_storage_mode_policy(mode)

    assert policy is not None
    assert (
        derive_storage_extended_mode(
            policy.modbus_control_mode,
            charge_power=charge_power,
            discharge_power=discharge_power,
            charge_grid_enabled=(mode == StorageExtendedControlMode.CHARGE_FROM_GRID),
        )
        == mode
    )


def test_derive_storage_extended_mode_preserves_legacy_grid_mode_signatures() -> None:
    assert derive_storage_extended_mode(2, charge_power=100, discharge_power=-25) == (
        StorageExtendedControlMode.CHARGE_FROM_GRID
    )
    assert derive_storage_extended_mode(1, charge_power=-25, discharge_power=100) == (
        StorageExtendedControlMode.DISCHARGE_TO_GRID
    )
