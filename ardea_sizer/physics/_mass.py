from numpy.typing import ArrayLike

def get_total_mass(
        *,
        fuel_mass: ArrayLike,
) -> ArrayLike:
    return 14.0 + fuel_mass
