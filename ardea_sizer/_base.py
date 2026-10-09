import json
from pathlib import Path
from typing import Self

from pydantic import BaseModel, ConfigDict, Field, model_validator
from ._constants import G


class FrozenModel(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")


class Mission(FrozenModel):
    range: float
    cruise_speed: float
    stall_speed: float

    @model_validator(mode="after")
    def validate_speeds(self) -> Self:
        if self.cruise_speed <= self.stall_speed:
            raise ValueError("cruise_speed must be above stall_speed")
        return self


class Aero(FrozenModel):
    c_l_max_3d: float


class Mass(FrozenModel):
    fuel: float
    rest: float

    @property
    def total(self) -> float:
        """Take-off mass in kg."""
        return self.fuel + self.rest


class Engine(FrozenModel):
    p_max: float
    fuel_cons: float
    engine_eff: float = Field(gt=0, le=1)
    prop_eff: float = Field(gt=0, le=1)


class Geometry(FrozenModel):
    AR: float
    taper: float
    oswald_eff: float


class Base(FrozenModel):
    mission: Mission
    aero: Aero
    mass: Mass
    engine: Engine
    geometry: Geometry

    @classmethod
    def from_json(cls, path: str | Path) -> Self:
        return cls.model_validate_json(
            Path(path).read_text(encoding="utf-8")
        )

    @property
    def total_weight(self) -> float:
        """Take-off weight in N."""
        return self.mass.total * G

    def with_(self, **changes) -> Self:
        """Copy with changed values, supporting dotted paths."""
        data = self.model_dump()

        for key, value in changes.items():
            parts = key.split(".")
            target = data

            for part in parts[:-1]:
                target = target[part]

            target[parts[-1]] = value

        return type(self).model_validate(data)
