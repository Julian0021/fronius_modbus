from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

_REGISTRY_DATA_KEY = "_test_issue_registry"


class IssueSeverity:
    WARNING = "warning"


@dataclass(slots=True)
class Issue:
    domain: str
    issue_id: str
    is_fixable: bool = False
    is_persistent: bool = False
    severity: str | None = None
    translation_key: str | None = None
    translation_placeholders: dict[str, str] | None = None
    data: dict[str, Any] | None = None
    ignored: bool = False


@dataclass
class _IssueRegistry:
    issues: dict[tuple[str, str], Issue] = field(default_factory=dict)

    def async_get_issue(self, domain: str, issue_id: str) -> Issue | None:
        return self.issues.get((domain, issue_id))


def _issue_registry(hass) -> _IssueRegistry:
    data = getattr(hass, "data", None)
    if data is None:
        data = {}
        hass.data = data
    registry = data.get(_REGISTRY_DATA_KEY)
    if registry is None:
        registry = _IssueRegistry()
        data[_REGISTRY_DATA_KEY] = registry
    return registry


def async_get(hass) -> _IssueRegistry:
    return _issue_registry(hass)


def async_create_issue(hass, domain: str, issue_id: str, **kwargs) -> None:
    _issue_registry(hass).issues[(domain, issue_id)] = Issue(
        domain=domain,
        issue_id=issue_id,
        **kwargs,
    )


def async_delete_issue(hass, domain: str, issue_id: str) -> None:
    _issue_registry(hass).issues.pop((domain, issue_id), None)


def async_ignore_issue(hass, domain: str, issue_id: str, ignored: bool) -> None:
    issue = _issue_registry(hass).issues.get((domain, issue_id))
    if issue is not None:
        issue.ignored = ignored
