from numpy.typing import ArrayLike

from ardea_sizer._constants import RHO


def get_dynamic_pressure(
    *,
    v: ArrayLike,
    rho: ArrayLike = RHO,
) -> ArrayLike:
    return 0.5 * rho * v**2
