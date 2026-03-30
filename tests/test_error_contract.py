from __future__ import annotations

from types import SimpleNamespace

import pytest
from requests import RequestException

import custom_components.fronius_modbus.flow_helpers as flow_helpers_module
from custom_components.fronius_modbus.flow_helpers import (
    CannotConnect,
    InvalidApiCredentials,
)
from custom_components.fronius_modbus.froniuswebclient import FroniusWebAuthError
from custom_components.fronius_modbus.hub_web_api import HubWebApiService
from custom_components.fronius_modbus.integration_errors import (
    FroniusAuthError,
    FroniusConnectionError,
)
from test_support.fakes import FakeHass


async def test_async_mint_token_maps_auth_failures_to_invalid_credentials(monkeypatch) -> None:
    hass = FakeHass()

    def _raise_auth(*_args, **_kwargs):
        raise FroniusWebAuthError("bad credentials")

    monkeypatch.setattr(flow_helpers_module, "mint_token", _raise_auth)

    with pytest.raises(InvalidApiCredentials):
        await flow_helpers_module.async_mint_token(hass, "inverter.local", "secret")


async def test_async_mint_token_maps_request_failures_to_cannot_connect(monkeypatch) -> None:
    hass = FakeHass()

    def _raise_request(*_args, **_kwargs):
        raise RequestException("network down")

    monkeypatch.setattr(flow_helpers_module, "mint_token", _raise_request)

    with pytest.raises(CannotConnect):
        await flow_helpers_module.async_mint_token(hass, "inverter.local", "secret")


async def test_validate_web_api_wraps_request_failures() -> None:
    hub = SimpleNamespace(
        _hass=FakeHass(),
        _webclient=SimpleNamespace(
            login=lambda: (_ for _ in ()).throw(RequestException("timeout"))
        ),
    )
    service = HubWebApiService(hub)

    with pytest.raises(FroniusConnectionError):
        await service.validate_web_api()


async def test_validate_web_api_wraps_auth_failures() -> None:
    hub = SimpleNamespace(
        _hass=FakeHass(),
        _webclient=SimpleNamespace(
            login=lambda: (_ for _ in ()).throw(FroniusWebAuthError("401"))
        ),
    )
    service = HubWebApiService(hub)

    with pytest.raises(FroniusAuthError):
        await service.validate_web_api()


async def test_async_web_job_wraps_request_failures() -> None:
    service = HubWebApiService(
        SimpleNamespace(
            _hass=FakeHass(),
            _webclient=object(),
        )
    )

    def _raise_request():
        raise RequestException("offline")

    with pytest.raises(FroniusConnectionError):
        await service.async_web_job(_raise_request)
