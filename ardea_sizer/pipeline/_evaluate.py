from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer.models import Base, Plane, Surface
from ardea_sizer.physics import (
    get_range,
    get_required_power,
    get_total_mass,
    get_c_l_max_3d, get_wing_area, get_c_d, get_c_d_0, get_c_l_cruise, get_drag_force, TimeSegment, FlightSegment, cruise_steps, get_isa_density
)
from ardea_sizer.physics import get_dynamic_pressure
from ardea_sizer._constants import FIELD_ALTITUDE
from ardea_sizer._constants import G


# claude function
def evaluate(
    base: Base,
    *,
    fuel_guess: float = 6.5,
    rel_tol: float = 1e-3,
    max_iter: int = 50,
) -> Plane:
    """Return the plane whose fuel mass covers mission range + reserve.

    Solves range(fuel_mass) = target with a secant method; all design inputs come from `base`.
    """
    m = base.mission
    target = m.range + m.reserve
    tol = rel_tol * abs(target)

    def run(fuel: float) -> tuple[Plane, float]:
        plane = evaluate_from_fuel_mass(base, fuel_mass=fuel)
        return plane, float(plane.range) - target

    # Two starting points for the secant
    f0, f1 = fuel_guess, 1.2 * fuel_guess
    _, r0 = run(f0)
    plane, r1 = run(f1)

    for _ in range(max_iter):
        if abs(r1) <= tol:
            return plane

        slope = (r1 - r0) / (f1 - f0)  # d(range)/d(fuel_mass)
        if not slope > 0:
            raise ValueError(
                "Range does not increase with fuel mass: "
                "mission is not reachable (extra fuel costs more range than it adds)."
            )

        f0, r0 = f1, r1
        f1 = max(f1 - r1 / slope, 0.5 * f1)  # never go negative / jump wildly down
        plane, r1 = run(f1)

    raise RuntimeError(
        f"Fuel iteration did not converge in {max_iter} steps "
        f"(range error {abs(r1):.3g})."
    )

def evaluate_from_fuel_mass(base: Base, *, fuel_mass: ArrayLike) -> Plane:
    """Choose AR & fuel mass -> MTOW -> wing -> stability -> drag -> range -> power."""
    m, e, a = base.mission, base.engine, base.assumptions

    rho_takeoff = get_isa_density(altitude=FIELD_ALTITUDE)
    rho_cruise = get_isa_density(altitude=FIELD_ALTITUDE + 120)

    q_cruise = get_dynamic_pressure(v=m.cruise_speed, rho=rho_cruise)
    q_stall = get_dynamic_pressure(v=m.stall_speed, rho=rho_takeoff)

    masses = base.with_(**{"masses.fuel.mass": float(fuel_mass)}).masses
    mass = get_total_mass(empty_mass=masses.empty, fuel_mass=fuel_mass)
    force = mass * G + base.tail.downforce

    wing_area = get_wing_area(
        force=force,
        q_stall=q_stall,
        c_l_max=get_c_l_max_3d(c_l_max_2d=a.c_l_max_2d_takeoff) + a.delta_c_l_flap
    )
    span = np.sqrt(a.wing_ar * wing_area)
    chord = wing_area / span

    c_d_0 = get_c_d_0(c_d_0_wing=a.c_d_0_wing, f_fixed=a.f_fixed, wing_area=wing_area)

    c_l_cruise = get_c_l_cruise(mass=mass, wing_area=wing_area, q=q_cruise)
    c_d_cruise = get_c_d(
        c_d_0=c_d_0,
        c_l=c_l_cruise,
        oswald_eff=a.oswald_eff,
        aspect_ratio=a.wing_ar,
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
        fuel_mass=fuel_mass, mtow=mass, wing_area=wing_area, aspect_ratio=a.wing_ar,
        c_d_0=c_d_0, oswald_eff=a.oswald_eff, prop_eff=e.prop_eff,
        sfc=e.fuel_cons, p_max=e.p_max * 1000,
    )

    t = base.tail
    return Plane(
        masses=masses,
        wing=Surface(area=wing_area, aspect_ratio=a.wing_ar, taper=a.wing_tr, x_le=a.wing_x, incidence=a.wing_ia),
        wing_airfoil=a.wing_airfoil,
        h_tail=Surface(area=t.h_area, aspect_ratio=t.h_ar, taper=t.h_tr, x_le=t.x, incidence=t.h_ia),
        v_tail=Surface(area=t.v_area, aspect_ratio=t.v_ar, taper=t.v_tr, x_le=t.x),
        c_d_0=c_d_0,
        range=flight_range,
        power_req=power_req,
        power_available=e.p_max,
    )
