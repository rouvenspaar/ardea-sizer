import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._constants import G, RHO


def get_c_l(
    *,
    mass: ArrayLike,
    wing_area: ArrayLike,
    q: ArrayLike,
) -> ArrayLike:
    return (mass * G) / (q * wing_area)

def get_wing_area(
    *,
    mass: ArrayLike,
    v_stall: ArrayLike,
    c_l_max: ArrayLike,
    rho: ArrayLike = RHO,
) -> ArrayLike:
    return (2 * mass * G) / (rho * v_stall**2 * c_l_max)
