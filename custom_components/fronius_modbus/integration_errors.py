"""Shared runtime error model for the Fronius integration."""
from __future__ import annotations


class FroniusError(RuntimeError):
    """Base error for runtime failures inside the integration."""


class FroniusConnectionError(FroniusError):
    """Raised when the integration cannot reach the inverter or Web API."""


class FroniusAuthError(FroniusError):
    """Raised when Web API authentication is missing or invalid."""


class FroniusReadError(FroniusError):
    """Raised when a read/parse operation cannot produce a valid payload."""


class FroniusDiscoveryError(FroniusError):
    """Raised when required startup discovery cannot complete."""


class FroniusOperationError(FroniusError):
    """Raised when a user-triggered operation cannot be completed or verified."""
