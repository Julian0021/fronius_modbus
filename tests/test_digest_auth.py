from __future__ import annotations

import hashlib
from types import SimpleNamespace

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
