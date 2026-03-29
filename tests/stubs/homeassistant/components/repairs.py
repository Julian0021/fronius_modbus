from __future__ import annotations


class RepairsFlow:
    def async_show_form(self, **kwargs):
        return {"type": "form", **kwargs}

    def async_create_entry(self, *, title, data):
        return {"type": "create_entry", "title": title, "data": data}

    def async_abort(self, *, reason):
        return {"type": "abort", "reason": reason}

    def async_show_menu(self, **kwargs):
        return {"type": "menu", **kwargs}
