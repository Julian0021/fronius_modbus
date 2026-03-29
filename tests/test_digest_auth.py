from __future__ import annotations

import hashlib

from custom_components.fronius_modbus.froniuswebclient import FroniusWebClient, XHeaderDigestAuth


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
