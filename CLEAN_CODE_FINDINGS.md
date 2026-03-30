# Clean Code Findings

Reviewed on 2026-03-30 against the current working tree.

Scope:
- Second-pass best-effort static clean-code review of the repository.
- Consolidated from six parallel `xhigh` review passes across runtime, protocol/client, config/flow, entity/platform, tests/harness, and repo-support code.
- Focused on maintainability, brittleness, regression risk, and CI/test blind spots.
- Follow-up fixes since the initial pass resolved the remaining medium findings around setup cleanup safety, primary meter alignment, and descriptor contract validation.
- The next follow-up batch resolved the first six low findings around runtime-state key routing, Web API cache clearing, Modbus exception surfacing, dependency-version error classification, coordinator compatibility probing, and CI compile coverage.
- A later live HA verification also fixed the narrow MPPT module translation-key mismatch; the broader German state-translation path was confirmed to be working as designed.
- Current verification from the working tree: `python -m ruff check custom_components tests` passed; `pytest -q` passed (`106 passed`).

No `Severe`, `High`, or `Medium` issues remain in this pass.

## Severe

No outstanding severe findings remain.

## High

No outstanding high findings remain.

## Medium

No outstanding medium findings remain.

## Low

1. `custom_components/fronius_modbus/platform_setup.py:40`
   `custom_components/fronius_modbus/base.py:97`
   `custom_components/fronius_modbus/entity_names.py:53`
   Fallback naming still uses `description.translation_key` as the nominal `name`, so any translation miss degrades directly to internal keys like `grid_charge_power` or `Conn`. Because the base entity resolves the name once during construction, that degraded label then stays fixed for the entity lifetime.

2. `custom_components/fronius_modbus/const.py:78`
   `custom_components/fronius_modbus/const.py:110`
   `custom_components/fronius_modbus/const.py:112`
   Several `frozen` descriptor dataclasses still hold mutable dictionaries such as `options`, `turn_on_kwargs`, and `turn_off_kwargs`. That makes the catalog only shallowly immutable: accidental mutation anywhere in process memory can still change global entity behavior at runtime.

## Nitpick

1. `custom_components/fronius_modbus/hub.py:165`
   `custom_components/fronius_modbus/hub_runtime.py:11`
   The inverter-glitch heuristic still hardcodes thresholds in `Hub._is_inverter_power_glitch()` even though the same rule is already represented by named constants in `hub_runtime.py`. One behavioral rule remains split across two modules, which invites threshold drift.

2. `custom_components/fronius_modbus/froniusmodbusclient.py:126`
   `custom_components/fronius_modbus/froniusmodbusclient.py:220`
   `_map_value()` still accepts `field_name`, but the argument is unused. The dead parameter adds noise to every mapped call site and still suggests validation/reporting that no longer exists.

3. `custom_components/fronius_modbus/translation.py:11`
   `custom_components/fronius_modbus/translation.py:48`
   Translation load failures are still cached as `{}` for the full process lifetime. That is fine in steady state, but during development or partial installs a transient missing/invalid translation file keeps producing fallback names until restart, even after the file is corrected.

4. `tests/test_platform_setup.py:103`
   `tests/test_platform_setup.py:201`
   `tests/test_platform_setup.py:309`
   `tests/test_platform_setup.py:353`
   This test file still relies on several `asyncio.run()` calls inside otherwise normal pytest tests. It works today, but it keeps event-loop ownership inconsistent with the rest of the suite and will get brittle if those tests later need async fixtures.
