from ardea_sizer.models._base import Base
from pathlib import Path
from ardea_sizer.pipeline._evaluate import evaluate

__all__ = [
    "evaluate",
    "load"
]

def load(path: str | Path) -> Base:
    return Base.from_json(path)
