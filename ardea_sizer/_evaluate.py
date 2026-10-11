import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._base import Base
from ardea_sizer.physics import (
    get_range,
    get_required_power,
    get_total_mass,
    get_wing_area, get_c_d, get_c_l,
)
from ardea_sizer.physics import get_dynamic_pressure


def evaluate(base: Base, *, aspect_ratio: ArrayLike, fuel_mass: ArrayLike):
    """Choose AR & fuel mass -> MTOW -> wing area -> drag -> range -> power."""
    m, e, a = base.mission, base.engine, base.assumptions
    bsfc = e.fuel_cons / 3.6e6  # kg/kWh -> kg/J
    p_max = e.p_max * 1000      # kW -> W

    q_cruise = get_dynamic_pressure(v=m.cruise_speed)
    q_stall = get_dynamic_pressure(v=m.stall_speed)

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

    c_l_cruise = get_c_l(mass=mass, wing_area=wing_area, q=q_cruise)
    c_d_cruise = get_c_d()

    flight_range = get_range(
        c_l=c_l_cruise,
        c_d=c_d_cruise,
        mtow=mass,
        fuel_mass=fuel_mass,
        prop_eff=e.prop_eff,
        sfc=e.fuel_cons,
    )

    power_req = get_required_power()

    print(flight_range)
