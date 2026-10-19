from pathlib import Path
from typing import Iterator, Self

from pydantic import BaseModel, ConfigDict, Field, model_validator


class FrozenModel(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")


class Mission(FrozenModel):
    range: float
    reserve: float
    cruise_speed: float
    stall_speed: float

    @model_validator(mode="after")
    def validate_speeds(self) -> Self:
        if self.cruise_speed <= self.stall_speed:
            raise ValueError("cruise_speed must be above stall_speed")
        return self


class MassPoint(FrozenModel):
    x: float  # position from datum (nose), positive aft [m]
    mass: float = Field(ge=0)  # [kg]


class Masses(FrozenModel):
    propeller: MassPoint  # incl. variable-pitch hub
    prop_shaft: MassPoint
    gearbox: MassPoint
    engine: MassPoint
    heat_shield: MassPoint
    ecu: MassPoint
    fuel_pump: MassPoint
    fuel: MassPoint  # full tank
    payload: MassPoint
    flight_controller: MassPoint
    electronics: MassPoint  # excl. flight controller & battery
    battery: MassPoint
    fuselage: MassPoint  # shell
    tail: MassPoint  # H + V
    wing: MassPoint

    def dry(self) -> Iterator[tuple[str, MassPoint]]:
        """All components except fuel, as (name, MassPoint)."""
        for name in type(self).model_fields:
            if name != "fuel":
                yield name, getattr(self, name)

    @property
    def empty(self) -> float:
        """Everything except fuel [kg]."""
        return sum(p.mass for _, p in self.dry())

    @property
    def total(self) -> float:
        """Empty mass plus full tank [kg]."""
        return self.empty + self.fuel.mass

    @property
    def x_cg(self) -> float:
        """Centre of gravity [m]. The tank sits at the CG, so fuel doesn't move it."""
        return sum(p.mass * p.x for _, p in self.dry()) / self.empty

    @property
    def x_cg_full(self) -> float:
        """Centre of gravity with full tank, from the nose [m]."""
        return (self.empty * self.x_cg + self.fuel.mass * self.fuel.x) / self.total


class Engine(FrozenModel):
    p_max: float
    fuel_cons: float
    prop_eff: float = Field(gt=0, le=1)


class Tail(FrozenModel):
    arm_length: float  # CG (full tank) -> aerodynamic centre (25 % MAC) of each stabilizer [m]
    h_area: float  # horizontal tail area [m^2]
    h_ar: float  # horizontal tail aspect ratio [-]
    h_tr: float  # horizontal tail taper ratio [-]
    h_ia: float  # horizontal tail incidence angle [deg]
    v_area: float  # vertical tail area [m^2]
    v_ar: float  # vertical tail aspect ratio [-]
    v_tr: float  # vertical tail taper ratio [-]
    downforce: float


class Assumptions(FrozenModel):
    c_l_max_2d_takeoff: float  # airfoil, at take-off Re
    c_l_max_2d_cruise: float  # airfoil, at cruise Re
    alpha_0_l_cruise: float  # zero-lift AoA [deg]
    c_l_alpha_2d_cruise: float  # airfoil lift slope [1/deg]
    delta_c_l_flap: float
    flap_area_fraction: float
    oswald_eff: float
    c_d_0_wing: float  # wing parasite drag, referenced to its own area
    f_fixed: float  # drag area CD*S of everything except the wing (fuselage, tails) [m^2]
    wing_ar: float
    wing_tr: float
    wing_x: float  # root leading edge from the nose [m]
    wing_airfoil: str  # NACA 4-digit code


class Base(FrozenModel):
    mission: Mission
    masses: Masses
    engine: Engine
    tail: Tail
    assumptions: Assumptions

    @classmethod
    def from_json(cls, path: str | Path) -> Self:
        return cls.model_validate_json(
            Path(path).read_text(encoding="utf-8")
        )

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
