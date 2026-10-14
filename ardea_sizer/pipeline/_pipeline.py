from ardea_sizer.models import Base, Plane
from ._evaluate import evaluate
from ._open_vsp import optimize


def run_pipeline(
    base: Base,
    *,
    abs_tol: float = 1e-4,
    max_iter: int = 10,
) -> tuple[Plane, Base]:
    """Size the plane and feed the OpenVSP parasite drag back until c_d_0 settles.

    Starts from c_d_0 in `base`: evaluate -> OpenVSP CD0 -> evaluate again with that CD0 ...
    Returns the converged plane and the base holding the c_d_0 it was sized with.
    """
    c_d_0 = base.assumptions.c_d_0

    for count in range(1, max_iter + 1):
        plane = evaluate(base)
        c_d_0_new = optimize(plane)
        print(f"Run {count}: c_d_0 {c_d_0:.5f} -> {c_d_0_new:.5f}   "
              f"S {float(plane.wing_area):.4f} m^2   fuel {float(plane.fuel_mass):.3f} kg")

        if abs(c_d_0_new - c_d_0) <= abs_tol:
            return plane, base

        c_d_0 = c_d_0_new
        base = base.with_(**{"assumptions.c_d_0": c_d_0})

    raise RuntimeError(f"c_d_0 did not converge in {max_iter} runs (last change {abs(c_d_0_new - c_d_0):.2g}).")
