"""Базовый класс проверки self_check (00_contract.md §3-4, tz1 §2.1).

Тексты каталога двуязычные: `title`/`description` — язык по умолчанию (English),
`title_ru`/`description_ru` — русский вариант. Ноду запускают с параметром
`language` (по умолчанию 'en'); manifest_entry берёт нужный через
localized_title/localized_description.
"""
from __future__ import annotations


class BaseCheck:
    id: str = ''
    category: str = ''
    title: str = ''          # английский (язык каталога по умолчанию)
    title_ru: str = ''       # русский вариант заголовка
    description: str = ''     # английский
    description_ru: str = ''  # русский вариант описания
    interactive: bool = False
    dangerous: bool = False

    def available(self, profile: dict, lang: str = 'en'):
        """(True, "") если проверка применима к профилю платформы,
        иначе (False, "причина" на языке lang). По умолчанию всегда доступна."""
        return True, ''

    def localized_title(self, lang: str = 'en') -> str:
        return self.title_ru if (lang == 'ru' and self.title_ru) else self.title

    def localized_description(self, lang: str = 'en') -> str:
        return self.description_ru if (lang == 'ru' and self.description_ru) else self.description

    def run(self, ctx) -> None:
        """Выполнить проверку (блокирующий вызов из воркер-потока).

        Обязана в итоге выставить ctx.result.state в одно из
        report.STATE_PASS/WARN/FAIL/SKIP. ctx.aborted надо периодически
        проверять и завершаться пораньше, если он выставлен.
        """
        raise NotImplementedError
