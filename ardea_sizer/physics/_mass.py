from numpy.typing import ArrayLike

def get_total_mass(
        *,
        empty_mass: ArrayLike,
        fuel_mass: ArrayLike,
) -> ArrayLike:
    return empty_mass + fuel_mass
