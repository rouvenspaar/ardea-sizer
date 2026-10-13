from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._base import Base
from ardea_sizer.physics import (
    get_range,
    get_required_power,
    get_total_mass,
    get_wing_area, get_c_d, get_c_l, get_drag_force, TimeSegment, FlightSegment, cruise_steps, fly_mission,
    get_isa_density
)
from ardea_sizer.physics import get_dynamic_pressure
from ardea_sizer._constants import FIELD_ALTITUDE

@dataclass(frozen=True)
class Plane:
    range: float
    power_req: float
    power_available: float
    wing_area: float
    span: float
    chord: float


def evaluate(base: Base, *, aspect_ratio: ArrayLike, fuel_mass: ArrayLike, c_d_0_cruise: float) -> Plane:
    """Choose AR & fuel mass -> MTOW -> wing area -> drag -> range -> power."""
    m, e, a = base.mission, base.engine, base.assumptions

    rho_takeoff = get_isa_density(altitude=FIELD_ALTITUDE)
    rho_cruise = get_isa_density(altitude=FIELD_ALTITUDE + 120)

    q_cruise = get_dynamic_pressure(v=m.cruise_speed, rho=rho_cruise)
    q_stall = get_dynamic_pressure(v=m.stall_speed, rho=rho_takeoff)

    mass = get_total_mass(fuel_mass=fuel_mass)

    wing_area = get_wing_area(
        mass=mass,
        q_stall=q_stall,
        c_l_max=a.c_l_max_3d + a.delta_c_l_flap
    )
    span = np.sqrt(aspect_ratio * wing_area)
    chord = wing_area / span

    c_l_cruise = get_c_l(mass=mass, wing_area=wing_area, q=q_cruise)
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

    segments = [
        TimeSegment("Warm-up / taxi", duration=120, throttle=0.1, altitude=FIELD_ALTITUDE),
        TimeSegment("Take-off", duration=30, throttle=1.0, altitude=FIELD_ALTITUDE),
        FlightSegment("Climb", v=60, alt_start=440, alt_end=FIELD_ALTITUDE + 120, climb_rate=5),
        *cruise_steps(name="Cruise", v=m.cruise_speed, altitude=FIELD_ALTITUDE + 120, n=20),
        FlightSegment("Descent", v=60, alt_start=560, alt_end=FIELD_ALTITUDE, climb_rate=-3),
        TimeSegment("Approach / landing", duration=120, throttle=0.1, altitude=FIELD_ALTITUDE),
        FlightSegment("Reserve loiter", v=26, alt_start=560, alt_end=FIELD_ALTITUDE + 120, duration=0, enabled=False),
    ]
    flight_range = get_range(
        segments,
        fuel_mass=fuel_mass, mtow=mass, wing_area=wing_area, aspect_ratio=aspect_ratio,
        c_d_0=c_d_0_cruise, oswald_eff=a.oswald_eff, prop_eff=e.prop_eff,
        sfc=e.fuel_cons, p_max=e.p_max * 1000,
    )

    return Plane(
        range=flight_range,
        power_req=power_req,
        power_available=e.p_max,
        wing_area=wing_area,
        span=span,
        chord=chord,
    )
