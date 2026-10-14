from ardea_sizer.models import Base, Plane
from ardea_sizer.physics import get_c_d_0
from ._evaluate import evaluate, evaluate_from_fuel_mass
from ._open_vsp import optimize


def run_pipeline(
    base: Base,
    *,
    fuel_mass: float | None = None,
    abs_tol: float = 1e-5,
    max_iter: int = 10,
    verbose: bool = True,
) -> tuple[Plane, Base]:
    """Size the plane and feed the OpenVSP parasite drag back until it settles.

    Starts from c_d_0_wing and f_fixed in `base`: evaluate -> OpenVSP drag of that plane ->
    evaluate again with it ... until the plane's CD0 changes by less than `abs_tol`.
    By default the fuel is sized for the mission (`evaluate`); with `fuel_mass` it is fixed
    and the range follows (`evaluate_from_fuel_mass`).
    Returns the converged plane and the base holding the drag values it was sized with.
    """
    for count in range(1, max_iter + 1):
        if fuel_mass is None:
            plane = evaluate(base)
        else:
            plane = evaluate_from_fuel_mass(base, fuel_mass=fuel_mass)
        c_d_0_wing, f_fixed = optimize(plane)
        c_d_0_new = get_c_d_0(c_d_0_wing=c_d_0_wing, f_fixed=f_fixed, wing_area=plane.wing_area)
        if verbose:
            print(f"Run {count}: c_d_0_wing {base.assumptions.c_d_0_wing:.5f} -> {c_d_0_wing:.5f}   "
                  f"f_fixed {base.assumptions.f_fixed:.5f} -> {f_fixed:.5f} m^2   "
                  f"CD0 {float(plane.c_d_0):.5f} -> {float(c_d_0_new):.5f}   "
                  f"S {float(plane.wing_area):.4f} m^2   fuel {float(plane.fuel_mass):.3f} kg")

        if abs(c_d_0_new - plane.c_d_0) <= abs_tol:
            return plane, base

        base = base.with_(**{"assumptions.c_d_0_wing": c_d_0_wing, "assumptions.f_fixed": f_fixed})

    raise RuntimeError(f"CD0 did not converge in {max_iter} runs (last change {abs(c_d_0_new - plane.c_d_0):.2g}).")
