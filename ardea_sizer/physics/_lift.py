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
    q_stall: ArrayLike,
    c_l_max: ArrayLike,
) -> ArrayLike:
    return (mass * G) / (q_stall * c_l_max)
