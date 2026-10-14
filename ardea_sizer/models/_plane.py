from dataclasses import dataclass


@dataclass(frozen=True)
class Plane:
    fuel_mass: float
    range: float
    power_req: float
    power_available: float
    wing_area: float
    c_d_0: float  # whole plane, referenced to wing_area
    taper: float
    aspect_ratio: float
