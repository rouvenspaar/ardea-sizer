import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._constants import G


# claude formula
def get_range(
    *,
    q: ArrayLike,
    wing_area: ArrayLike,
    c_d_0: ArrayLike,
    oswald_eff: ArrayLike,
    aspect_ratio: ArrayLike,
    mtow: ArrayLike,
    fuel_mass: ArrayLike,
    prop_eff: ArrayLike,
    sfc: ArrayLike,
) -> ArrayLike:
    # constant-speed cruise, drag D = A + B * W², sfc in kg/kWh, returns m
    c = sfc / 3.6e6
    a = q * wing_area * c_d_0
    b = 1 / (np.pi * oswald_eff * aspect_ratio * q * wing_area)
    w_start = mtow * G
    w_end = (mtow - fuel_mass) * G
    k = np.sqrt(b / a)
    return prop_eff / (c * G * np.sqrt(a * b)) * (np.arctan(w_start * k) - np.arctan(w_end * k))
