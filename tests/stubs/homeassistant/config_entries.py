from __future__ import annotations

from typing import Any

CONN_CLASS_LOCAL_POLL = "local_poll"


class ConfigEntry:
    def __init__(self, **kwargs: Any) -> None:
        self.__dict__.update(kwargs)

    def __class_getitem__(cls, _item):
        return cls

    def add_update_listener(self, listener):
        self._update_listener = listener
        return lambda: None

    def async_on_unload(self, unload_callback) -> None:
        self._unload_callback = unload_callback


class _BaseFlow:
    def async_show_form(self, **kwargs):
        return {"type": "form", **kwargs}

    def async_create_entry(self, *, title, data):
        return {"type": "create_entry", "title": title, "data": data}

    def async_abort(self, *, reason):
        return {"type": "abort", "reason": reason}


class ConfigFlow(_BaseFlow):
    CONNECTION_CLASS = CONN_CLASS_LOCAL_POLL

    def __init_subclass__(cls, **kwargs):
        return None

    def _get_reconfigure_entry(self):
        return getattr(self, "_reconfigure_entry", None)


class OptionsFlow(_BaseFlow):
    pass
