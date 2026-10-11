from numpy.typing import ArrayLike


def get_effective_power(
    *,
    thrust: ArrayLike,
    v: ArrayLike,
    prop_eff: ArrayLike,
) -> ArrayLike:
    return thrust * v / prop_eff

def get_required_power() -> ArrayLike:
    return 2000.0
