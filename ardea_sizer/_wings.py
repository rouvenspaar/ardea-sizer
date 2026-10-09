import numpy as np
import pandas as pd

from ._columns import C
from ._constants import G, RHO


def get_wing_area(df: pd.DataFrame) -> pd.Series:
    return np.maximum(
        get_wing_area_cruise(df),
        get_wing_area_takeoff(df),
    )

def get_wing_area_cruise(df: pd.DataFrame) -> pd.Series:
    mtow = df[C.FUEL] + df[C.REST]
    v = df[C.CRUISE_SPEED]

    # TODO: S = 2*mtow*G / (RHO * v**2 * C_L_opt), C_L_opt = sqrt(pi * e * AR * C_D0)
    return pd.Series(0.0, index=df.index)

def get_wing_area_takeoff(df: pd.DataFrame) -> pd.Series:
    mtow = df[C.FUEL] + df[C.REST]
    v = df[C.STALL_SPEED]
    c_l_max = df[C.CL_MAX]

    return (2 * mtow * G) / (RHO * v**2 * c_l_max)
