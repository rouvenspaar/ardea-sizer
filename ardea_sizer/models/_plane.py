from dataclasses import dataclass


@dataclass(frozen=True)
class Plane:
    range: float
    power_req: float
    power_available: float
    wing_area: float
    span: float
    chord: float
