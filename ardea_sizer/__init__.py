from pathlib import Path

from ardea_sizer.models import Base
from ardea_sizer.pipeline import evaluate, run_pipeline

__all__ = [
    "evaluate",
    "load",
    "run_pipeline"
]

def load(path: str | Path) -> Base:
    return Base.from_json(path)
