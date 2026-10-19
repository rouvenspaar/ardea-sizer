import re
from itertools import groupby

import numpy as np

from ardea_sizer.models import Base, Plane
from ardea_sizer.physics import (
    fly_mission,
    get_alpha_cruise,
    get_c_d,
    get_c_l_alpha_3d,
    get_c_l_cruise,
    get_c_l_max_3d,
    get_c_l_max_flapped,
    get_dynamic_pressure,
    get_isa_density,
)
from ardea_sizer._constants import CRUISE_ALTITUDE, FIELD_ALTITUDE, G, RHO_SL
from ._evaluate import mission_segments
from ._open_vsp import FUSELAGE_NAME, H_TAIL_NAME, V_TAIL_NAME, WING_NAME, get_vsp_model

WIDTH = 79
VSP_LABELS = {FUSELAGE_NAME: "Fuselage", WING_NAME: "Wing", H_TAIL_NAME: "H-tail", V_TAIL_NAME: "V-tail"}


def summary(plane: Plane, base: Base) -> str:
    """Detailed text report of a sized plane; `base` is the one it was sized with.

    Also sizes the OpenVSP model to the plane (saved to OUTPUT_FILE) and reports its geometry and drag.
    """
    m, e, a = base.mission, base.engine, base.assumptions
    masses = plane.masses
    vsp_model = get_vsp_model(plane)
    wing = plane.wing
    mtow = float(plane.mtow)
    weight = mtow * G
    s = float(wing.area)
    c_d_0 = float(plane.c_d_0)

    rho_field = get_isa_density(altitude=FIELD_ALTITUDE)
    rho_cruise = get_isa_density(altitude=CRUISE_ALTITUDE)
    q_cruise = get_dynamic_pressure(v=m.cruise_speed, rho=rho_cruise)

    lines: list[str] = []

    def section(title: str) -> None:
        lines.extend(["", title.upper(), "-" * WIDTH])

    def row(label: str, value: float | str, unit: str = "", fmt: str = ".3f") -> None:
        text = value if isinstance(value, str) else format(float(value), fmt)
        lines.append(f"  {label:<34}{text:>14}  {unit}")

    lines.extend(["=" * WIDTH, "ARDEA SIZING SUMMARY".center(WIDTH), "=" * WIDTH])

    # ------------------------------------------------------------------ mission
    section("Mission")
    row("Range (design)", m.range / 1000, "km", ".1f")
    row("Reserve", m.reserve / 1000, "km", ".1f")
    row("Range achieved (incl. reserve)", float(plane.range) / 1000, "km", ".1f")
    row("Cruise speed", m.cruise_speed, f"m/s  ({m.cruise_speed * 3.6:.0f} km/h)", ".2f")
    row("Stall speed (flaps, MTOW)", m.stall_speed, f"m/s  ({m.stall_speed * 3.6:.0f} km/h)", ".2f")
    row("Field / cruise altitude", f"{FIELD_ALTITUDE:.0f} / {CRUISE_ALTITUDE:.0f}", "m")

    # ------------------------------------------------------------------ masses
    section("Masses")
    lines.append(f"  {'Component':<22}{'mass [kg]':>11}{'% MTOW':>10}")
    for name in type(masses).model_fields:
        p = getattr(masses, name)
        lines.append(f"  {name:<22}{p.mass:>11.3f}{100 * p.mass / mtow:>10.1f}")
    lines.append("  " + "-" * (WIDTH - 2))
    row("Empty mass", masses.empty, f"kg  ({100 * masses.empty / mtow:.1f} % MTOW)")
    row("Fuel mass", plane.fuel_mass, f"kg  ({100 * float(plane.fuel_mass) / mtow:.1f} % MTOW)")
    row("Payload mass", masses.payload.mass, f"kg  ({100 * masses.payload.mass / mtow:.1f} % MTOW)")
    row("MTOW", mtow, f"kg  ({weight:.1f} N)")

    # ------------------------------------------------------------------ geometry
    section("Geometry (sizing | OpenVSP model)")
    surfaces = ((wing, vsp_model.wing), (plane.h_tail, vsp_model.h_tail), (plane.v_tail, vsp_model.v_tail))
    lines.append(f"  {'':<22}" + "".join(f"{name:>18}" for name in ("Wing", "H-tail", "V-tail")))
    lines.append(f"  {'':<22}" + f"{'sizing':>9}{'VSP':>9}" * 3)
    geometry = [
        ("Area [m^2]", lambda x: x.area, lambda x: x.area, ".4f"),
        ("Span tip-tip [m]", lambda x: x.span, lambda x: x.span, ".3f"),
        ("Root offset [m]", lambda x: 0.0, lambda x: x.root_offset, ".3f"),
        ("Root chord [m]", lambda x: x.root_chord, lambda x: x.root_chord, ".4f"),
        ("Tip chord [m]", lambda x: x.tip_chord, lambda x: x.tip_chord, ".4f"),
        ("Mean chord [m]", lambda x: x.area / x.span, lambda x: x.mean_chord, ".4f"),
        ("x root LE [m]", lambda x: x.x_le, lambda x: x.x, ".3f"),
        ("Incidence [deg]", lambda x: x.incidence, lambda x: x.incidence, ".2f"),
    ]
    for label, get_sized, get_vsp, fmt in geometry:
        lines.append(f"  {label:<22}" + "".join(
            f"{float(get_sized(sized)):>9{fmt}}{float(get_vsp(built)):>9{fmt}}" for sized, built in surfaces
        ))
    lines.append(f"  {'Aspect ratio [-]':<22}" + "".join(f"{float(sized.aspect_ratio):>18.2f}" for sized, _ in surfaces))
    lines.append(f"  {'Taper ratio [-]':<22}" + "".join(f"{float(sized.taper):>18.2f}" for sized, _ in surfaces))
    lines.append(f"  {'MAC [m]':<22}" + "".join(f"{float(sized.mac):>18.4f}" for sized, _ in surfaces))
    row("Wing airfoil", f"NACA {plane.wing_airfoil}")
    row("Tail AC (H & V, 25 % MAC)", plane.h_tail.x_ac, f"m   ({base.tail.arm_length:.3f} m behind the full-tank CG)")
    lines.append("  (spans: wing & H-tail tip to tip, V-tail height; the sizing has no root offset)")

    section("OpenVSP model")
    row("Overall length", vsp_model.length, "m")
    row("Overall wingspan (incl. tip caps)", vsp_model.width, "m")
    row("Overall height", vsp_model.height, "m")
    row("Fuselage length", vsp_model.fuselage_length, "m")
    row("Fuselage width / height", f"{vsp_model.fuselage_width:.3f} / {vsp_model.fuselage_height:.3f}", "m")
    fuselage = next(d for d in vsp_model.drag if d.name == FUSELAGE_NAME)
    row("Fuselage fineness (l / d)", fuselage.fineness, "-", ".2f")
    row("Total wetted area", sum(d.s_wet for d in vsp_model.drag if np.isfinite(d.s_wet)), "m^2", ".4f")
    row("Wetted / reference area", sum(d.s_wet for d in vsp_model.drag if np.isfinite(d.s_wet)) / vsp_model.s_ref, "-", ".2f")

    # ------------------------------------------------------------------ drag build-up
    section("Parasite drag build-up (OpenVSP)")
    lines.append(f"  at {vsp_model.altitude:.0f} m, {vsp_model.v_inf:.2f} m/s, Mach {vsp_model.mach:.3f}, "
                 f"S_ref {vsp_model.s_ref:.4f} m^2")
    lines.append(f"  {'Component':<20}{'S_wet':>8}{'Re':>9}{'Cf':>8}{'FF':>6}{'Q':>5}{'f [m^2]':>9}{'CD':>8}{'%':>6}")
    for d in vsp_model.drag:
        if np.isfinite(d.s_wet):
            label = f"{VSP_LABELS.get(d.name, d.name)[:19]:<20}"
            label += f"{d.s_wet:>8.4f}{d.re:>9.3g}{d.c_f:>8.5f}{d.form_factor:>6.3f}{d.q:>5.2f}"
        else:  # excrescence
            label = f"{d.name[:55]:<56}"
        lines.append(f"  {label}{d.f:>9.5f}{d.c_d:>8.5f}{100 * d.f / vsp_model.f_total:>6.1f}")
    lines.append("  " + "-" * (WIDTH - 2))
    lines.append(f"  {'Total (OpenVSP)':<56}{vsp_model.f_total:>9.5f}{vsp_model.c_d_0:>8.5f}{100:>6.1f}")
    lines.append("")
    lines.append("  OpenVSP treats the wing as fully turbulent; the sizing uses the XFOIL wing instead:")
    wing_label = f"Wing, XFOIL c_d {a.c_d_wing_section:.4f} x {a.c_d_wing_factor:.2f}"
    lines.append(f"  {wing_label:<56}{a.c_d_0_wing * s:>9.5f}{a.c_d_0_wing:>8.5f}")
    lines.append(f"  {'Everything else (OpenVSP, f_fixed)':<56}{a.f_fixed:>9.5f}{a.f_fixed / s:>8.5f}")
    lines.append(f"  {'Total used in sizing':<56}{c_d_0 * s:>9.5f}{c_d_0:>8.5f}")

    # ------------------------------------------------------------------ aerodynamics
    section("Aerodynamics (cruise, MTOW)")
    c_l = float(get_c_l_cruise(mass=mtow, wing_area=s, q=q_cruise))
    c_d = float(get_c_d(c_d_0=c_d_0, c_l=c_l, oswald_eff=a.oswald_eff, aspect_ratio=wing.aspect_ratio))
    c_d_i = c_d - c_d_0
    c_l_alpha = float(get_c_l_alpha_3d(c_l_alpha_2d=a.c_l_alpha_2d_cruise, aspect_ratio=wing.aspect_ratio))
    alpha = float(get_alpha_cruise(c_l=c_l, c_l_alpha_3d=c_l_alpha, alpha_0_l=a.alpha_0_l_cruise))
    c_l_max_clean = float(get_c_l_max_3d(c_l_max_2d=a.c_l_max_2d_cruise))
    c_l_max_flap = float(get_c_l_max_flapped(
        c_l_max_2d=a.c_l_max_2d_takeoff, delta_c_l_flap=a.delta_c_l_flap, flap_area_fraction=a.flap_area_fraction,
    ))
    v_stall_clean = np.sqrt(2 * weight / (rho_cruise * s * c_l_max_clean))
    # best L/D of the parabolic polar: CL = sqrt(CD0 * pi * e * AR)
    c_l_best = np.sqrt(c_d_0 * np.pi * a.oswald_eff * wing.aspect_ratio)
    v_best = m.cruise_speed * np.sqrt(c_l / c_l_best)

    row("Wing loading", weight / s, f"N/m^2  ({mtow / s:.1f} kg/m^2)", ".1f")
    row("Dynamic pressure", q_cruise, "Pa", ".1f")
    row("CL", c_l, "-", ".4f")
    row("CD0 (whole plane, ref. wing)", c_d_0, "-", ".5f")
    row("  CD0 wing (XFOIL x factor)", a.c_d_0_wing, "-", ".5f")
    row("  CD0 fixed (f / S)", a.f_fixed / s, f"-   (f = {a.f_fixed:.5f} m^2)", ".5f")
    row("CDi", c_d_i, f"-   ({100 * c_d_i / c_d:.0f} % of CD)", ".5f")
    row("CD", c_d, "-", ".5f")
    row("L/D", c_l / c_d, "-", ".2f")
    row("L/D max", c_l_best / (2 * c_d_0), f"-   (at {v_best:.1f} m/s)", ".2f")
    row("Drag", q_cruise * c_d * s, "N", ".2f")
    row("CL_alpha wing (3D)", c_l_alpha, "1/deg", ".4f")
    c_l_mid = float(get_c_l_cruise(mass=mtow - 0.5 * float(plane.fuel_mass), wing_area=s, q=q_cruise))
    alpha_mid = float(get_alpha_cruise(c_l=c_l_mid, c_l_alpha_3d=c_l_alpha, alpha_0_l=a.alpha_0_l_cruise))
    row("Angle of attack", alpha, "deg", ".2f")
    row("Wing incidence i_w", wing.incidence, "deg   (fuselage level at mid cruise)", ".2f")
    row("Fuselage angle (alpha - i_w)", alpha - wing.incidence, "deg", ".2f")
    row("Fuselage angle, mid cruise", alpha_mid - wing.incidence, "deg   (half the fuel burned)", ".2f")
    row("Oswald efficiency", a.oswald_eff, "-", ".2f")
    row("CLmax clean / flaps (3D)", f"{c_l_max_clean:.3f} / {c_l_max_flap:.3f}", "-")
    row("Stall speed clean (cruise alt.)", v_stall_clean, "m/s", ".2f")
    row("Cruise / clean stall speed", m.cruise_speed / v_stall_clean, "-", ".2f")

    # ------------------------------------------------------------------ propulsion
    section("Propulsion & performance")
    p_avail_cruise = e.p_max * rho_cruise / RHO_SL
    p_req = float(plane.power_req) / 1000
    row("Max power (sea level)", e.p_max, "kW", ".2f")
    row("Max power (cruise altitude)", p_avail_cruise, "kW", ".2f")
    row("Cruise power required (MTOW)", p_req, f"kW  ({100 * p_req / p_avail_cruise:.0f} % of available)", ".2f")
    row("Power loading", mtow / e.p_max, "kg/kW", ".2f")
    row("Specific fuel consumption", e.fuel_cons, "kg/kWh", ".3f")
    row("Propeller efficiency", e.prop_eff, "-", ".2f")

    result = fly_mission(
        mission_segments(m),
        mission_range=float(plane.range), mtow=mtow, wing_area=s, aspect_ratio=wing.aspect_ratio,
        c_d_0=c_d_0, oswald_eff=a.oswald_eff, prop_eff=e.prop_eff,
        sfc=e.fuel_cons, p_max=e.p_max * 1000,
    )
    total_time = sum(r.time for r in result.segments)
    row("Flight time", total_time / 3600, "h", ".2f")
    row("Fuel burned", result.fuel, "kg", ".3f")
    row("Fuel per 100 km", 100e3 * float(result.fuel) / float(plane.range), "kg", ".3f")
    row("Max throttle", 100 * float(result.max_throttle), "%", ".0f")
    if not result.feasible:
        lines.append("  !! NOT FEASIBLE: a segment needs more power than the engine delivers")

    # ------------------------------------------------------------------ mission profile
    section("Mission profile")
    lines.append(f"  {'Segment':<20}{'time [min]':>11}{'dist [km]':>11}{'P avg [kW]':>12}"
                 f"{'thr max':>9}{'fuel [kg]':>11}{'m end':>8}")
    # merge the cruise steps ("Cruise 1", "Cruise 2", ...) into one row
    for name, group in groupby(result.segments, key=lambda r: re.sub(r" \d+$", "", r.name)):
        group = list(group)
        time = sum(r.time for r in group)
        if time == 0:
            continue
        fuel = sum(float(r.fuel) for r in group)
        energy = sum(float(r.power) * r.time for r in group)
        lines.append(
            f"  {name:<20}{time / 60:>11.1f}{sum(r.distance for r in group) / 1000:>11.1f}"
            f"{energy / time / 1000:>12.2f}{max(100 * float(r.throttle) for r in group):>8.0f}%"
            f"{fuel:>11.3f}{float(group[-1].mass_end):>8.2f}"
        )

    lines.append("=" * WIDTH)
    return "\n".join(lines)

