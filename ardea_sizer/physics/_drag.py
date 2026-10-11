import numpy as np
from numpy.typing import ArrayLike


def get_c_d(
    *,
    c_d_0: ArrayLike,
    c_l: ArrayLike,
    oswald_eff: ArrayLike,
    aspect_ratio: ArrayLike,
) -> ArrayLike:
    return c_d_0 + c_l**2 / (np.pi * oswald_eff * aspect_ratio)

# TODO
def get_c_d_0() -> ArrayLike:
    return 0.025

def get_drag_force(
    *,
    q: ArrayLike,
    c_d: ArrayLike,
    wing_area: ArrayLike,
) -> ArrayLike:
    return q * c_d * wing_area
