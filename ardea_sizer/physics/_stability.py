from dataclasses import dataclass, fields

from numpy.typing import ArrayLike


@dataclass
class MassPoint:
    x: float
    mass: float


@dataclass
class Masses:
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

    def _dry(self) -> list[MassPoint]:
        return [getattr(self, f.name) for f in fields(self) if f.name != "fuel"]

    @property
    def empty(self) -> float:
        """Everything except fuel."""
        return sum(p.mass for p in self._dry())

    def total(self, fuel_mass: ArrayLike | None = None) -> ArrayLike:
        """fuel_mass=None -> full tank."""
        fuel = self.fuel.mass if fuel_mass is None else fuel_mass
        return self.empty + fuel

    def x_cg(self, fuel_mass: ArrayLike | None = None) -> ArrayLike:
        """fuel_mass=None -> full tank, 0 -> empty tank."""
        fuel = self.fuel.mass if fuel_mass is None else fuel_mass
        moment = sum(p.mass * p.x for p in self._dry()) + fuel * self.fuel.x
        return moment / self.total(fuel_mass)


masses = Masses(
    propeller=MassPoint(x=0.040, mass=0.35),
    prop_shaft=MassPoint(x=0.083, mass=0.15),
    gearbox=MassPoint(x=0.158, mass=0.30),
    engine=MassPoint(x=0.268, mass=3.50),
    heat_shield=MassPoint(x=0.335, mass=0.0),  # mass not given yet
    ecu=MassPoint(x=0.368, mass=0.10),
    fuel_pump=MassPoint(x=0.365, mass=0.20),
    fuel=MassPoint(x=0.465, mass=5.00),
    payload=MassPoint(x=0.638, mass=1.00),
    flight_controller=MassPoint(x=0.803, mass=0.25),
    electronics=MassPoint(x=0.803, mass=0.60),
    battery=MassPoint(x=0.803, mass=0.50),
    fuselage=MassPoint(x=0.612, mass=3.00),
    tail=MassPoint(x=1.252, mass=0.40),
    wing=MassPoint(x=0.489, mass=3.00),
)
