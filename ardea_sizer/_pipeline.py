from numpy.typing import ArrayLike

from ardea_sizer import evaluate
from ardea_sizer._base import Base
from ardea_sizer._open_vsp import optimize


def run_pipeline(base: Base, *, aspect_ratio: ArrayLike):
    delta_cd0_converged = 0.001
    c_d_0_values: list[float] = [0.022]
    count = 0

    while count < 1 and abs(c_d_0_values[-1] - c_d_0_values[-2]) > delta_cd0_converged:
        plane = evaluate(base, aspect_ratio=aspect_ratio, fuel_mass=6.2, c_d_0_cruise=c_d_0_values[count])
        c_d_0_new = optimize(plane)
        c_d_0_values.append(c_d_0_new)
        count += 1
        print("Simulations run: " + str(count))

    print("c_d_0: " + str(c_d_0_values[-1]))
