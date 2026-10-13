from numpy.typing import ArrayLike

from ardea_sizer._constants import RHO_SL


def get_dynamic_pressure(
    *,
    v: ArrayLike,
    rho: ArrayLike,
) -> ArrayLike:
    return 0.5 * rho * v**2

def get_isa_density(
    *,
    altitude: ArrayLike,
) -> ArrayLike:
    # ISA troposphere, altitude in m
    return RHO_SL * (1 - 2.25577e-5 * altitude) ** 4.2559
