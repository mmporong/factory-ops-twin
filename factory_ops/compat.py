"""Small compatibility surface for the installed Isaac Sim 4.5 Python 3.10."""
from __future__ import annotations

import sys
import errno
import json
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
        except OSError as error:
            if not isinstance(error, PermissionError) and error.errno != errno.EIO:
                raise
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
        except OSError as error:
            if not isinstance(error, (PermissionError, FileNotFoundError)) and error.errno != errno.EIO:
                raise
            if time.monotonic() >= deadline:
                raise
            time.sleep(.002)


def write_shared_text(path: Path, text: str, *, encoding: str = "utf-8", timeout_s: float = .2) -> None:
    """Retry a checkpoint's temporary-file write within the shared-I/O bound.

    Append journals are excluded: redoing an append could duplicate an event.
    ENOSPC and other permanent errors are raised without a retry.
    """
    if isinstance(timeout_s, bool) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 1:
        raise ValueError("shared-file writing requires a positive bound of at most one second")
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            path.write_text(text, encoding=encoding)
            return
        except OSError as error:
            if not isinstance(error, PermissionError) and error.errno != errno.EIO:
                raise
            if time.monotonic() >= deadline:
                raise
            time.sleep(.002)


def preserve_failure_checkpoint(record: dict, directory: Path, stream) -> dict:
    """Keep the first failure on native temporary storage if shared output fails."""
    run_id = record.get("run_id")
    if (not isinstance(run_id, str) or len(run_id) != 32
            or any(c not in "0123456789abcdef" for c in run_id)):
        raise ValueError("failure checkpoint requires the existing32hex run identity")
    payload = json.dumps(record, ensure_ascii=True, allow_nan=False)
    result = {"path": str(directory / (run_id+".json")), "saved": False}
    try:
        directory.mkdir(parents=True, exist_ok=True)
        with Path(result["path"]).open("x", encoding="utf-8") as file:
            file.write(payload+"\n")
        result["saved"] = True
    except FileExistsError:
        result["existing_first_failure_retained"] = True
    except OSError as error:
        result["checkpoint_error"] = str(error)
    try:
        print("FACTORY_OPS_PRIMARY_FAILURE "+payload, file=stream, flush=True)
    except OSError as error:
        result["stream_error"] = str(error)
    return result
