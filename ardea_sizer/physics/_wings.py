import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._constants import G, RHO


def get_wing_area(
    *,
    mass: ArrayLike,
    v_stall: ArrayLike,
    c_l_max: ArrayLike,
    v_cruise: ArrayLike,
    aspect_ratio: ArrayLike,
    oswald_eff: ArrayLike,
    c_d0: ArrayLike,
    rho: ArrayLike = RHO,
) -> ArrayLike:
    return np.maximum(
        get_wing_area_cruise(
            mass=mass,
            v_cruise=v_cruise,
            aspect_ratio=aspect_ratio,
            oswald_eff=oswald_eff,
            c_d0=c_d0,
            rho=rho,
        ),
        get_wing_area_stall(
            mass=mass,
            v_stall=v_stall,
            c_l_max=c_l_max,
            rho=rho,
        ),
    )

def get_wing_area_cruise(
    *,
    mass: ArrayLike,
    v_cruise: ArrayLike,
    aspect_ratio: ArrayLike,
    oswald_eff: ArrayLike,
    c_d0: ArrayLike,
    rho: ArrayLike = RHO,
) -> ArrayLike:
    c_l_opt = np.sqrt(np.pi * oswald_eff * aspect_ratio * c_d0)
    return (2 * mass * G) / (rho * v_cruise**2 * c_l_opt)

def get_wing_area_stall(
    *,
    mass: ArrayLike,
    v_stall: ArrayLike,
    c_l_max: ArrayLike,
    rho: ArrayLike = RHO,
) -> ArrayLike:
    return (2 * mass * G) / (rho * v_stall**2 * c_l_max)
