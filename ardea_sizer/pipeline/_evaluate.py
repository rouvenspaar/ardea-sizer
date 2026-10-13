from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer.models import Base, Plane
from ardea_sizer.physics import (
    get_range,
    get_required_power,
    get_total_mass,
    get_wing_area, get_c_d, get_c_l, get_drag_force, TimeSegment, FlightSegment, cruise_steps, get_isa_density
)
from ardea_sizer.physics import get_dynamic_pressure
from ardea_sizer._constants import FIELD_ALTITUDE


# claude function
def evaluate(
    base: Base,
    *,
    aspect_ratio: ArrayLike,
    fuel_guess: float = 6.5,
    rel_tol: float = 1e-3,
    max_iter: int = 50,
) -> Plane:
    """Return the plane whose fuel mass covers mission range + reserve.

    Solves range(fuel_mass) = target with a vectorised secant method, so every
    element of `aspect_ratio` gets its own fuel mass.
    """
    m = base.mission
    target = m.range + m.reserve
    tol = rel_tol * abs(target)
    shape = np.shape(aspect_ratio)

    def run(fuel: np.ndarray) -> tuple[Plane, np.ndarray]:
        plane = evaluate_from_fuel_mass(
            base,
            fuel_mass=fuel if fuel.ndim else float(fuel),
        )
        return plane, np.asarray(plane.range, dtype=float) - target

    # Two starting points for the secant
    f0 = np.full(shape, fuel_guess, dtype=float)
    _, r0 = run(f0)
    f1 = 1.2 * f0
    plane, r1 = run(f1)

    for _ in range(max_iter):
        done = np.abs(r1) <= tol
        if done.all():
            return plane

        with np.errstate(divide="ignore", invalid="ignore"):
            slope = (r1 - r0) / (f1 - f0)  # d(range)/d(fuel_mass)

        bad = ~done & ~(slope > 0)
        if bad.any():
            raise ValueError(
                "Range does not increase with fuel mass for aspect_ratio="
                f"{np.asarray(aspect_ratio)[bad] if shape else aspect_ratio}: "
                "mission is not reachable (extra fuel costs more range than it adds)."
            )

        step = np.where(done, 0.0, -r1 / np.where(done, 1.0, slope))
        f_new = np.maximum(f1 + step, 0.5 * f1)  # never go negative / jump wildly down

        f0, r0 = f1, r1
        f1 = f_new
        plane, r1 = run(f1)

    raise RuntimeError(
        f"Fuel iteration did not converge in {max_iter} steps "
        f"(worst range error {np.max(np.abs(r1)):.3g})."
    )

def evaluate_from_fuel_mass(base: Base, *, fuel_mass: ArrayLike) -> Plane:
    """Choose AR & fuel mass -> MTOW -> wing -> stability -> drag -> range -> power."""
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
    span = np.sqrt(a.wing_ar * wing_area)
    chord = wing_area / span

    c_l_cruise = get_c_l(mass=mass, wing_area=wing_area, q=q_cruise)
    c_d_cruise = get_c_d(
        c_d_0=a.c_d_0,
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
        c_d_0=a.c_d_0, oswald_eff=a.oswald_eff, prop_eff=e.prop_eff,
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
