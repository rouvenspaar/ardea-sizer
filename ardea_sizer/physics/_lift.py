import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._constants import G


def get_c_l_max_3d(
    *,
    c_l_max_2d: ArrayLike,
    factor: ArrayLike = 0.9,
) -> ArrayLike:
    # wing C_L,max from airfoil c_l,max; Raymer: ~0.9 for unswept, moderate-to-high AR
    return factor * c_l_max_2d

def get_wing_area(
    *,
    force: ArrayLike,
    q_stall: ArrayLike,
    c_l_max: ArrayLike,
) -> ArrayLike:
    return force / (q_stall * c_l_max)

def get_c_l_cruise(
    *,
    mass: ArrayLike,
    wing_area: ArrayLike,
    q: ArrayLike,
) -> ArrayLike:
    # C_L needed to carry the weight (also used for climb / descent steps)
    return (mass * G) / (q * wing_area)

def get_c_l_alpha_3d(
    *,
    c_l_alpha_2d: ArrayLike,
    aspect_ratio: ArrayLike,
) -> ArrayLike:
    kappa = c_l_alpha_2d * 180 / np.pi / (2 * np.pi)
    c_l_alpha_rad = 2 * np.pi * aspect_ratio / (2 + np.sqrt(aspect_ratio**2 / kappa**2 + 4))
    return c_l_alpha_rad * np.pi / 180

def get_alpha_cruise(
    *,
    c_l: ArrayLike,
    c_l_alpha_3d: ArrayLike,
    alpha_0_l: ArrayLike,
) -> ArrayLike:
    return alpha_0_l + c_l / c_l_alpha_3d
