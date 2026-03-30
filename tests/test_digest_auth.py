from __future__ import annotations

import hashlib
from types import SimpleNamespace

import custom_components.fronius_modbus.froniuswebclient as froniuswebclient_module
from custom_components.fronius_modbus.froniuswebclient import (
    FroniusWebClient,
    XHeaderDigestAuth,
)


def _digest_value(secret: str, nonce: str, nc: str, cnonce: str, qop: str, ha2: str) -> str:
    return hashlib.md5(f"{secret}:{nonce}:{nc}:{cnonce}:{qop}:{ha2}".encode()).hexdigest()


def test_digest_auth_uses_md5_consistently(monkeypatch) -> None:
    auth = XHeaderDigestAuth("customer", password="secret")
    auth.mode = "md5"
    monkeypatch.setattr("os.urandom", lambda size: b"\x01" * size)

    header = auth._build_header(
        "GET",
        "/status",
        {
            "realm": "test-realm",
            "nonce": "abc123",
            "qop": "auth",
        },
    )

    expected_secret = hashlib.md5(b"customer:test-realm:secret").hexdigest()
    expected_ha2 = hashlib.md5(b"GET:/status").hexdigest()
    expected_response = _digest_value(
        expected_secret,
        "abc123",
        "00000001",
        "0101010101010101",
        "auth",
        expected_ha2,
    )

    assert f'response="{expected_response}"' in header


def test_web_client_honors_username_argument() -> None:
    client = FroniusWebClient("fixture-host", username="installer")

    assert client._username == "installer"
    assert client._auth.username == "installer"


def test_digest_auth_handle_401_retries_without_transport_send(monkeypatch) -> None:
    auth = XHeaderDigestAuth("customer", password="secret", timeout=7.5)
    auth.mode = "md5"
    monkeypatch.setattr("os.urandom", lambda size: b"\x01" * size)

    request_headers = {"Accept": "application/json"}

    class _PreparedRequest:
        def __init__(self, headers: dict[str, str]) -> None:
            self.method = "GET"
            self.url = "http://fixture-host/api/commands/Login?user=customer"
            self.headers = headers
            self.body = None

        def copy(self):
            return _PreparedRequest(dict(self.headers))

    class _InitialResponse:
        def __init__(self) -> None:
            self.status_code = 401
            self.request = _PreparedRequest(dict(request_headers))
            self.headers = {
                "X-WWW-Authenticate": 'Digest realm="test-realm", nonce="abc123", qop="auth"',
            }
            self.history = []
            self.content = b"denied"
            self.closed = False
            self.connection = SimpleNamespace(
                send=lambda *_args, **_kwargs: (_ for _ in ()).throw(
                    AssertionError("transport send should not be used")
                )
            )

        def close(self) -> None:
            self.closed = True

    retried = SimpleNamespace(
        status_code=200,
        history=[],
        request=SimpleNamespace(headers={}),
    )
    request_calls: list[tuple[tuple, dict]] = []

    def _fake_request(*args, **kwargs):
        request_calls.append((args, kwargs))
        return retried

    monkeypatch.setattr("requests.request", _fake_request)

    initial = _InitialResponse()
    result = auth.handle_401(initial, timeout=7.5, verify=False)

    assert result is retried
    assert initial.closed is True
    assert len(request_calls) == 1
    _, kwargs = request_calls[0]
    assert kwargs["timeout"] == 7.5
    assert kwargs["verify"] is False
    assert "Authorization" in kwargs["headers"]
    assert retried.history == [initial]
    assert auth.saved_token == {
        "realm": "test-realm",
        "token": hashlib.md5(b"customer:test-realm:secret").hexdigest(),
    }


def test_digest_auth_reprobes_after_provisional_mode_fails(monkeypatch) -> None:
    class _StatusResponse:
        def __init__(self, payload: dict[str, object]) -> None:
            self._payload = payload

        def raise_for_status(self) -> None:
            return None

        def json(self) -> dict[str, object]:
            return self._payload

    class _PreparedRequest:
        def __init__(self) -> None:
            self.method = "GET"
            self.url = "http://fixture-host/api/commands/Login?user=customer"
            self.headers: dict[str, str] = {"Accept": "application/json"}
            self.body = None
            self.hooks: dict[str, object] = {}

        def register_hook(self, name: str, callback: object) -> None:
            self.hooks[name] = callback

        def copy(self):
            copied = _PreparedRequest()
            copied.headers = dict(self.headers)
            return copied

    class _InitialResponse:
        def __init__(self, request: _PreparedRequest) -> None:
            self.status_code = 401
            self.request = request
            self.headers = {
                "X-WWW-Authenticate": 'Digest realm="test-realm", nonce="abc123", qop="auth"',
            }
            self.history: list[object] = []
            self.content = b"denied"
            self.closed = False

        def close(self) -> None:
            self.closed = True

    monkeypatch.setattr("os.urandom", lambda size: b"\x01" * size)
    froniuswebclient_module._HASH_MODE_CACHE.clear()

    status_calls: list[tuple[str, float]] = []
    status_responses = [
        _StatusResponse({"authenticationOptions": {"digest": {}}}),
        _StatusResponse({"authenticationOptions": {"digest": {"customerHashingVersion": 1}}}),
    ]

    def _fake_status_get(url: str, timeout: float):
        status_calls.append((url, timeout))
        return status_responses.pop(0)

    request_calls: list[dict[str, object]] = []
    retried_responses = [
        SimpleNamespace(
            status_code=401,
            history=[],
            request=SimpleNamespace(headers={"Authorization": "Digest bad"}),
        ),
        SimpleNamespace(
            status_code=200,
            history=[],
            request=SimpleNamespace(headers={"Authorization": "Digest good"}),
        ),
    ]

    def _fake_request(method: str, url: str, **kwargs):
        request_calls.append({"method": method, "url": url, **kwargs})
        return retried_responses.pop(0)

    monkeypatch.setattr(froniuswebclient_module.requests, "get", _fake_status_get)
    monkeypatch.setattr("requests.request", _fake_request)

    auth = XHeaderDigestAuth("customer", password="secret", timeout=7.5)

    first_request = _PreparedRequest()
    auth(first_request)
    assert auth.mode == "sha256"

    first_result = auth.handle_401(_InitialResponse(first_request), timeout=7.5)
    assert first_result.status_code == 401
    assert auth.mode is None

    second_request = _PreparedRequest()
    auth(second_request)
    assert auth.mode == "md5"

    second_result = auth.handle_401(_InitialResponse(second_request), timeout=7.5)
    assert second_result.status_code == 200
    assert auth.mode == "md5"
    assert status_calls == [
        ("http://fixture-host/api/status/common", 7.5),
        ("http://fixture-host/api/status/common", 7.5),
    ]
    assert request_calls[0]["headers"]["Authorization"].startswith("Digest ")
    assert request_calls[1]["headers"]["Authorization"].startswith("Digest ")
