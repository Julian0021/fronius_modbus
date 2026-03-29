from __future__ import annotations


class TextSelectorType:
    PASSWORD = "password"


class TextSelectorConfig:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs


class TextSelector:
    def __init__(self, config) -> None:
        self.config = config
