import json
from dataclasses import dataclass, replace
from pathlib import Path

G = 9.81  # m/s²


@dataclass(frozen=True)
class Mission:
    range: float          # m
    cruise_speed: float   # m/s
    stall_speed: float    # m/s

    def __post_init__(self):
        if self.cruise_speed <= self.stall_speed:
            raise ValueError("cruise_speed must be above stall_speed")


@dataclass(frozen=True)
class Aero:
    c_l_max_3d: float     # -


@dataclass(frozen=True)
class Mass:
    fuel: float   # kg
    rest: float   # kg

    @property
    def total(self) -> float:
        """Take-off mass in kg."""
        return self.fuel + self.rest


@dataclass(frozen=True)
class Engine:
    p_max: float       # kW
    fuel_cons: float   # kg/kWh (BSFC)
    engine_eff: float  # -
    prop_eff: float    # -

    def __post_init__(self):
        for name in ("engine_eff", "prop_eff"):
            if not 0 < getattr(self, name) <= 1:
                raise ValueError(f"{name} must be in (0, 1]")


@dataclass(frozen=True)
class Geometry:
    AR: float            # -
    taper: float         # -
    oswald_eff: float    # -
    frontal_area: float  # m²


@dataclass(frozen=True)
class Inputs:
    mission: Mission
    aero: Aero
    mass: Mass
    engine: Engine
    geometry: Geometry

    @classmethod
    def from_dict(cls, d: dict) -> "Inputs":
        return cls(
            mission=Mission(**d["mission"]),
            aero=Aero(**d["aero"]),
            mass=Mass(**d["mass"]),
            engine=Engine(**d["engine"]),
            geometry=Geometry(**d["geometry"]),
        )

    @classmethod
    def from_json(cls, path: str | Path) -> "Inputs":
        with open(path) as f:
            return cls.from_dict(json.load(f))

    @property
    def total_weight(self) -> float:
        """Take-off weight in N."""
        return self.mass.total * G

    def with_(self, **changes) -> "Inputs":
        """Copy with changed values, e.g. with_(**{"geometry.AR": 10, "mission.stall_speed": 18})."""
        new = self
        for key, value in changes.items():
            if "." in key:
                group, field = key.split(".")
                sub = replace(getattr(new, group), **{field: value})
                new = replace(new, **{group: sub})
            else:
                new = replace(new, **{key: value})
        return new
