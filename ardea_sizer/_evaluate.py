import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._base import Base
from ardea_sizer.physics import (
    get_range,
    get_required_power,
    get_total_mass,
    get_wing_area,
)


def evaluate(base: Base, *, aspect_ratio: ArrayLike, fuel_mass: ArrayLike):
    """Choose AR & fuel mass -> MTOW -> wing area -> drag -> range -> power."""
    m, e, a = base.mission, base.engine, base.assumptions
    bsfc = e.fuel_cons / 3.6e6  # kg/kWh -> kg/J
    p_max = e.p_max * 1000      # kW -> W


    mass = get_total_mass()

    wing_area = get_wing_area(
        mass=mass,
        v_stall=m.stall_speed,
        c_l_max=a.c_l_max_3d + a.delta_c_l_flap,
        v_cruise=m.cruise_speed,
        aspect_ratio=aspect_ratio,
        oswald_eff=a.oswald_eff,
        c_d0=a.c_d0,
    )
    span = np.sqrt(aspect_ratio * wing_area)
















    c_l = get_lift_coefficient(mass=mass, wing_area=wing_area, v=m.cruise_speed)
    c_d = get_drag_coefficient(
        c_l=c_l,
        c_d0=a.c_d0,
        aspect_ratio=aspect_ratio,
        oswald_eff=a.oswald_eff,
    )
    drag = get_total_drag(c_d=c_d, wing_area=wing_area, v=m.cruise_speed)
    lift_to_drag = c_l / c_d

    flight_range = get_range(
        lift_to_drag=lift_to_drag,
        mass=mass,
        fuel_mass=fuel_mass,
        bsfc=bsfc,
        prop_eff=e.prop_eff,
    )
    power_required = get_required_power(drag=drag, v=m.cruise_speed, prop_eff=e.prop_eff)
