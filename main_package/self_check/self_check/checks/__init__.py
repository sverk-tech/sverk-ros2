"""Реестр всех проверок self_check (00_contract.md §4).

REGISTRY: {check_id: BaseCheck-инстанс}. CATEGORY_ORDER задаёт порядок
прогона категорий (совпадает с таблицей §4 контракта); PASSIVE_ORDER /
INTERACTIVE_ORDER — порядок id внутри "пассивные, затем интерактивные"
(tz1 §2.2).
"""
from __future__ import annotations

from . import link, power, sensors, vision, motors, sbc, prearm

_MODULES = (link, power, sensors, vision, motors, sbc, prearm)

REGISTRY = {}
for _module in _MODULES:
    for _check in _module.CHECKS:
        if _check.id in REGISTRY:
            raise RuntimeError(f'duplicate check id: {_check.id}')
        REGISTRY[_check.id] = _check

ALL_IDS = list(REGISTRY.keys())
PASSIVE_IDS = [cid for cid in ALL_IDS if not REGISTRY[cid].interactive]
INTERACTIVE_IDS = [cid for cid in ALL_IDS if REGISTRY[cid].interactive]
