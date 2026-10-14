from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._constants import G, RHO_SL
from ardea_sizer.physics._atmosphere import get_dynamic_pressure, get_isa_density
from ardea_sizer.physics._drag import get_c_d, get_drag_force
from ardea_sizer.physics._lift import get_c_l_cruise
from ardea_sizer.physics._power import get_required_power

J_PER_KWH = 3.6e6

# TODO: check

@dataclass(frozen=True)
class TimeSegment:
    name: str
    duration: float
    throttle: float
    altitude: float 
    enabled: bool = True


@dataclass(frozen=True)
class FlightSegment:
    name: str
    v: float
    alt_start: float
    alt_end: float
    climb_rate: float = 0.0
    distance: float | None = None
    duration: float | None = None
    enabled: bool = True


@dataclass(frozen=True)
class SegmentResult:
    name: str
    time: float
    distance: float
    power: ArrayLike
    throttle: ArrayLike
    fuel: ArrayLike
    mass_end: ArrayLike
    c_l: ArrayLike = np.nan
    c_d: ArrayLike = np.nan


@dataclass(frozen=True)
class MissionResult:
    segments: list[SegmentResult]
    fuel: ArrayLike
    max_throttle: ArrayLike

    @property
    def feasible(self) -> ArrayLike:
        return self.max_throttle <= 1.0


def cruise_steps(*, name: str, v: float, altitude: float, n: int) -> list[FlightSegment]:
    return [FlightSegment(f"{name} {i + 1}", v, altitude, altitude) for i in range(n)]


def _kinematics(seg: FlightSegment) -> tuple[float, float, float]:
    sin_g = seg.climb_rate / seg.v
    cos_g = np.sqrt(1 - sin_g**2)
    if not seg.enabled:
        t = 0.0
    elif seg.climb_rate:
        t = (seg.alt_end - seg.alt_start) / seg.climb_rate
    elif seg.distance is not None:
        t = seg.distance / (seg.v * cos_g)
    else:
        t = seg.duration or 0.0
    return t, seg.v * cos_g * t, sin_g


def _is_filler(s) -> bool:
    return (isinstance(s, FlightSegment) and s.enabled and not s.climb_rate
            and s.distance is None and s.duration is None)


def _fixed_distance(segments: list) -> float:
    return sum(_kinematics(s)[1] for s in segments if isinstance(s, FlightSegment) and not _is_filler(s))


def _fill_remaining_range(segments: list, mission_range: ArrayLike) -> list:
    fillers = [s for s in segments if _is_filler(s)]
    if not fillers:
        return segments
    leg = (mission_range - _fixed_distance(segments)) / len(fillers)
    if np.any(leg < 0):
        raise ValueError("climb/descent/loiter distance exceeds mission range")
    return [FlightSegment(s.name, s.v, s.alt_start, s.alt_end, distance=leg) if _is_filler(s) else s
            for s in segments]


def fly_mission(
    segments: list[TimeSegment | FlightSegment],
    *,
    mission_range: float,
    mtow: ArrayLike,
    wing_area: ArrayLike,
    aspect_ratio: ArrayLike,
    c_d_0: ArrayLike,
    oswald_eff: ArrayLike,
    prop_eff: ArrayLike,
    sfc: ArrayLike,
    p_max: ArrayLike,
) -> MissionResult:
    segments = _fill_remaining_range(segments, mission_range)
    mass = mtow
    results: list[SegmentResult] = []

    for seg in segments:
        if isinstance(seg, TimeSegment):
            t = seg.duration if seg.enabled else 0.0
            p_avail = p_max * get_isa_density(altitude=seg.altitude) / RHO_SL
            power = seg.throttle * p_avail
            fuel = power * t * sfc / J_PER_KWH
            mass = mass - fuel
            results.append(SegmentResult(seg.name, t, 0.0, power, seg.throttle, fuel, mass))
            continue

        t, distance, sin_g = _kinematics(seg)
        rho = get_isa_density(altitude=0.5 * (seg.alt_start + seg.alt_end))
        q = get_dynamic_pressure(v=seg.v, rho=rho)
        p_avail = p_max * rho / RHO_SL

        def power_at(m):
            c_l = get_c_l_cruise(mass=m, wing_area=wing_area, q=q)
            c_d = get_c_d(c_d_0=c_d_0, c_l=c_l, oswald_eff=oswald_eff, aspect_ratio=aspect_ratio)
            drag = get_drag_force(q=q, c_d=c_d, wing_area=wing_area)
            thrust = np.maximum(drag + m * G * sin_g, 0.0)  # steep descent -> idle, no negative fuel
            return get_required_power(thrust=thrust, v=seg.v, prop_eff=prop_eff), c_l, c_d

        power, _, _ = power_at(mass)
        mass_mid = mass - 0.5 * power * t * sfc / J_PER_KWH
        power, c_l, c_d = power_at(mass_mid)
        fuel = power * t * sfc / J_PER_KWH
        mass = mass - fuel
        results.append(SegmentResult(seg.name, t, distance, power, power / p_avail, fuel, mass, c_l, c_d))

    return MissionResult(
        segments=results,
        fuel=mtow - mass,
        max_throttle=np.max(np.stack(np.broadcast_arrays(*(r.throttle for r in results))), axis=0),
    )


def get_range(
    segments: list[TimeSegment | FlightSegment],
    *,
    fuel_mass: ArrayLike,
    mtow: ArrayLike,
    wing_area: ArrayLike,
    aspect_ratio: ArrayLike,
    c_d_0: ArrayLike,
    oswald_eff: ArrayLike,
    prop_eff: ArrayLike,
    sfc: ArrayLike,
    p_max: ArrayLike,
    tol: float = 1.0,
    max_iter: int = 20,
) -> ArrayLike:
    # mission range in m at which fly_mission burns exactly fuel_mass (secant on fuel(range))
    # nan where fuel_mass doesn't even cover the non-cruise segments
    aircraft = dict(mtow=mtow, wing_area=wing_area, aspect_ratio=aspect_ratio, c_d_0=c_d_0,
                    oswald_eff=oswald_eff, prop_eff=prop_eff, sfc=sfc, p_max=p_max)

    def excess_fuel(r):
        return fly_mission(segments, mission_range=r, **aircraft).fuel - fuel_mass

    r_min = _fixed_distance(segments)  # cruise length zero
    f_min = excess_fuel(r_min)
    r0, f0 = r_min, f_min
    r1 = r_min + 1e5
    f1 = excess_fuel(r1)
    for _ in range(max_iter):
        df = f1 - f0
        step = np.divide(f1 * (r1 - r0), df, out=np.zeros(np.shape(df)), where=df != 0)  # converged -> stay
        r2 = np.maximum(r1 - step, r_min)
        if np.all(np.abs(r2 - r1) < tol):
            break
        r0, f0 = r1, f1
        r1, f1 = r2, excess_fuel(r2)
    return np.where(f_min > 0, np.nan, r2)
