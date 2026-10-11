import numpy as np
from numpy.typing import ArrayLike

from ardea_sizer._constants import G


def get_range(
    *,
    c_l: ArrayLike,
    c_d: ArrayLike,
    mtow: ArrayLike,
    fuel_mass: ArrayLike,
    prop_eff: ArrayLike,
    sfc: ArrayLike,
) -> ArrayLike:
    return (3.6e6 / G) * (prop_eff / sfc) * (c_l / c_d) * np.log(mtow / (mtow - fuel_mass))
