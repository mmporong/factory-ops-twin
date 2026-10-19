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
