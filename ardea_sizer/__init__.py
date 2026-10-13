from pathlib import Path

from ardea_sizer.models import Base
from ardea_sizer.pipeline import evaluate

__all__ = [
    "evaluate",
    "load"
]

def load(path: str | Path) -> Base:
    return Base.from_json(path)
