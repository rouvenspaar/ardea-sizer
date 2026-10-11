from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._base import Base
from ardea_sizer.physics import (
    get_range,
    get_required_power,
    get_total_mass,
    get_wing_area, get_c_d, get_c_l, get_drag_force, get_c_d_0
)
from ardea_sizer.physics import get_dynamic_pressure

@dataclass(frozen=True)
class Plane:
    range: float
    power_req: float
    power_available: float
    wing_area: float
    span: float
    chord: float


def evaluate(base: Base, *, aspect_ratio: ArrayLike, fuel_mass: ArrayLike) -> Plane:
    """Choose AR & fuel mass -> MTOW -> wing area -> drag -> range -> power."""
    m, e, a = base.mission, base.engine, base.assumptions

    q_cruise = get_dynamic_pressure(v=m.cruise_speed)
    q_stall = get_dynamic_pressure(v=m.stall_speed)

    mass = get_total_mass(fuel_mass=fuel_mass)

    wing_area = get_wing_area(
        mass=mass,
        q_stall=q_stall,
        c_l_max=a.c_l_max_3d + a.delta_c_l_flap
    )
    span = np.sqrt(aspect_ratio * wing_area)
    chord = wing_area / span

    c_l_cruise = get_c_l(mass=mass, wing_area=wing_area, q=q_cruise)
    c_d_0_cruise = get_c_d_0()
    c_d_cruise = get_c_d(
        c_d_0=c_d_0_cruise,
        c_l=c_l_cruise,
        oswald_eff=a.oswald_eff,
        aspect_ratio=aspect_ratio,
    )

    thrust_req = get_drag_force(q=q_cruise, c_d=c_d_cruise, wing_area=wing_area)
    power_req = get_required_power(
        thrust=thrust_req,
        v=m.cruise_speed,
        prop_eff=e.prop_eff
    )

    flight_range = get_range(
        q=q_cruise,
        wing_area=wing_area,
        c_d_0=c_d_0_cruise,
        oswald_eff=a.oswald_eff,
        aspect_ratio=aspect_ratio,
        mtow=mass,
        fuel_mass=fuel_mass,
        prop_eff=e.prop_eff,
        sfc=e.fuel_cons,
    )

    return Plane(
        range=flight_range,
        power_req=power_req,
        power_available=e.p_max,
        wing_area=wing_area,
        span=span,
        chord=chord,
    )
