from numpy.typing import ArrayLike


def get_required_power(
    *,
    thrust: ArrayLike,
    v: ArrayLike,
    prop_eff: ArrayLike,
) -> ArrayLike:
    return thrust * v / prop_eff
