import numpy as np
from numpy.typing import ArrayLike


def get_span(
    *,
    area: ArrayLike,
    aspect_ratio: ArrayLike,
) -> ArrayLike:
    return np.sqrt(aspect_ratio * area)

def get_root_chord(
    *,
    area: ArrayLike,
    span: ArrayLike,
    taper: ArrayLike,
) -> ArrayLike:
    return 2 * area / (span * (1 + taper))

def get_mac(
    *,
    root_chord: ArrayLike,
    taper: ArrayLike,
) -> ArrayLike:
    return 2 / 3 * root_chord * (1 + taper + taper**2) / (1 + taper)
