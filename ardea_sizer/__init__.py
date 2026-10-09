from ardea_sizer._base import Base
from pathlib import Path

__all__ = [

]

def open(path: str | Path) -> Base:
    return Base.from_json(path)
