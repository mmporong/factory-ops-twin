"""Small compatibility surface for the installed Isaac Sim 4.5 Python 3.10."""
from __future__ import annotations

import sys
import math
import os
from pathlib import Path
import time

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


def replace_shared_file(source: Path, target: Path, *, timeout_s: float = .2) -> None:
    """Retry bounded sharing conflicts while retaining atomic replacement."""
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 1:
        raise ValueError("shared-file replacement requires a positive bound of at most one second")
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            os.replace(source, target)
            return
        except PermissionError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(.002)


def read_shared_text(path: Path, *, encoding: str = "utf-8", timeout_s: float = .2) -> str:
    """Read a complete file across bounded Windows/DrvFS replacement races."""
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 1:
        raise ValueError("shared-file reading requires a positive bound of at most one second")
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return path.read_text(encoding=encoding)
        except (PermissionError, FileNotFoundError):
            if time.monotonic() >= deadline:
                raise
            time.sleep(.002)
