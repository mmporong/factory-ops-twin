"""Small compatibility surface for the installed Isaac Sim 4.5 Python 3.10."""
from __future__ import annotations

import sys

if sys.version_info >= (3, 11):
    from enum import StrEnum
else:  # Python 3.10 in the vendor SDK
    from enum import Enum

    class StrEnum(str, Enum):
        """String-valued enums used by the operations ledger."""

        __str__ = str.__str__


def add_exception_note(error: BaseException, note: str) -> None:
    """Preserve cleanup context on the SDK's Python 3.10 as well as newer Python."""
    attach = getattr(error, "add_note", None)
    if callable(attach):
        attach(note)
    else:
        notes = list(getattr(error, "__notes__", []))
        notes.append(note)
        error.__notes__ = notes
