from dataclasses import dataclass, fields

import numpy as np
from numpy.typing import ArrayLike

DIAMETER_TAIL_TUBE = 0.02
HORIZONTAL_VOLUME_COEFFICIENT = 0.6
CORRECTION_TAIL_ASSUMPTION = 1.0



@dataclass
class MassPoint:
    x: float
    mass: float


@dataclass
class Masses:
    propeller: MassPoint  # incl. variable-pitch hub
    prop_shaft: MassPoint
    gearbox: MassPoint
    engine: MassPoint
    heat_shield: MassPoint
    ecu: MassPoint
    fuel_pump: MassPoint
    fuel: MassPoint  # full tank
    payload: MassPoint
    flight_controller: MassPoint
    electronics: MassPoint  # excl. flight controller & battery
    battery: MassPoint
    fuselage: MassPoint  # shell
    tail: MassPoint  # H + V
    wing: MassPoint

    def _dry(self) -> list[MassPoint]:
        return [getattr(self, f.name) for f in fields(self) if f.name != "fuel"]

    @property
    def empty(self) -> float:
        """Everything except fuel."""
        return sum(p.mass for p in self._dry())

    def total(self, fuel_mass: ArrayLike | None = None) -> ArrayLike:
        """fuel_mass=None -> full tank."""
        fuel = self.fuel.mass if fuel_mass is None else fuel_mass
        return self.empty + fuel

    def x_cg(self, fuel_mass: ArrayLike | None = None) -> ArrayLike:
        """fuel_mass=None -> full tank, 0 -> empty tank."""
        fuel = self.fuel.mass if fuel_mass is None else fuel_mass
        moment = sum(p.mass * p.x for p in self._dry()) + fuel * self.fuel.x
        return moment / self.total(fuel_mass)


masses = Masses(
    propeller=MassPoint(x=0.040, mass=0.35),
    prop_shaft=MassPoint(x=0.083, mass=0.15),
    gearbox=MassPoint(x=0.158, mass=0.30),
    engine=MassPoint(x=0.268, mass=3.50),
    heat_shield=MassPoint(x=0.335, mass=0.0),  # mass not given yet
    ecu=MassPoint(x=0.368, mass=0.10),
    fuel_pump=MassPoint(x=0.365, mass=0.20),
    fuel=MassPoint(x=0.465, mass=5.00),
    payload=MassPoint(x=0.638, mass=1.00),
    flight_controller=MassPoint(x=0.803, mass=0.25),
    electronics=MassPoint(x=0.803, mass=0.60),
    battery=MassPoint(x=0.803, mass=0.50),
    fuselage=MassPoint(x=0.612, mass=3.00),
    tail=MassPoint(x=1.252, mass=0.40),
    wing=MassPoint(x=0.489, mass=3.00),
)

def get_cg(masses: Masses) -> float:
    dry = [getattr(masses, f.name) for f in fields(masses) if f.name != "fuel"]
    x = np.array([p.x for p in dry], dtype=float)
    m = np.array([p.mass for p in dry], dtype=float)
    return float(m @ x / m.sum())  # sum(m_i * x_i) / sum(m_i)


"""Longitudinal static stability: tail sizing, neutral point, static margin, trim.

All angles in radians, all lengths in metres, positions measured from the datum (nose), positive aft.
"""

#import numpy as np
#from numpy.typing import ArrayLike


# --------------------------------------------------------------------------- #
# Tail sizing
# --------------------------------------------------------------------------- #


def get_optimal_tail_arm(
    *,
    mac: ArrayLike,
    wing_area: ArrayLike,
    tail_volume_coeff: ArrayLike,
    boom_diameter: ArrayLike,
    n_booms: int = 1,
    k_c: ArrayLike = 1.0,
) -> ArrayLike:
    """Tail arm (CG -> tail aerodynamic centre) that minimises tail + boom wetted area.

    Assumes the tail is carried on constant-diameter boom(s) (wetted area pi * D * l each).
    The spreadsheet uses Sadraey's version for a tapered tail cone (factor 4 instead of 2),
    which gives a sqrt(2) longer arm.

    :param mac: Mean aerodynamic chord of the wing [m].
    :param wing_area: Wing reference area S [m^2].
    :param tail_volume_coeff: Horizontal tail volume coefficient V_H [-].
    :param boom_diameter: Diameter of the tail boom [m].
    :param n_booms: Number of tail booms (2 for a twin-boom) [-].
    :param k_c: Empirical correction factor, 1 ... 1.4 [-].
    :return: Optimal tail arm [m].
    """
    return k_c * np.sqrt(
        2 * mac * wing_area * tail_volume_coeff / (np.pi * boom_diameter * n_booms)
    )

def get_x_ac_tail(
    *,
    x_cg: ArrayLike,
    tail_arm: ArrayLike,
) -> ArrayLike:
    """Position of the horizontal tail aerodynamic centre.

    :param x_cg: Position of the centre of gravity [m].
    :param tail_arm: Tail arm, CG -> tail aerodynamic centre [m].
    :return: Position of the tail aerodynamic centre [m].
    """
    return x_cg + tail_arm

def get_tail_area(
    *,
    tail_volume_coeff: ArrayLike,
    wing_area: ArrayLike,
    mac: ArrayLike,
    tail_arm: ArrayLike,
) -> ArrayLike:
    """Horizontal tail area from the tail volume coefficient.

    :param tail_volume_coeff: Horizontal tail volume coefficient V_H [-].
    :param wing_area: Wing reference area S [m^2].
    :param mac: Mean aerodynamic chord of the wing [m].
    :param tail_arm: Tail arm, CG -> tail aerodynamic centre [m].
    :return: Horizontal tail area S_h [m^2].
    """
    return tail_volume_coeff * wing_area * mac / tail_arm

# --------------------------------------------------------------------------- #
# Neutral point & static margin
# --------------------------------------------------------------------------- #


def get_downwash_gradient(
    *,
    cl_alpha_wing: ArrayLike,
    aspect_ratio: ArrayLike,
) -> ArrayLike:
    """Downwash gradient at the tail, elliptic-wing estimate (conservative: ignores tail position).

    :param cl_alpha_wing: 3D lift curve slope of the wing [1/rad].
    :param aspect_ratio: Wing aspect ratio [-].
    :return: Downwash gradient d(epsilon)/d(alpha) [-].
    """
    return 2 * cl_alpha_wing / (np.pi * aspect_ratio)

def get_fuselage_cm_alpha(
    *,
    k_fus: ArrayLike,
    fuselage_width: ArrayLike,
    fuselage_length: ArrayLike,
    wing_area: ArrayLike,
    mac: ArrayLike,
) -> ArrayLike:
    """Destabilising pitching moment slope of the fuselage (Raymer, Fig. 16.14).

    :param k_fus: Empirical fuselage pitching factor, 0.008 ... 0.020 [1/deg].
    :param fuselage_width: Maximum fuselage width [m].
    :param fuselage_length: Fuselage length [m].
    :param wing_area: Wing reference area S [m^2].
    :param mac: Mean aerodynamic chord of the wing [m].
    :return: Fuselage Cm_alpha [1/rad]; positive = destabilising.
    """
    per_deg = k_fus * fuselage_width**2 * fuselage_length / (wing_area * mac)
    return np.degrees(per_deg)  # 1/deg -> 1/rad

def get_tail_factor(
    *,
    eta_h: ArrayLike,
    tail_area: ArrayLike,
    wing_area: ArrayLike,
    cl_alpha_h: ArrayLike,
    cl_alpha_wing: ArrayLike,
    downwash_gradient: ArrayLike,
) -> ArrayLike:
    """Tail contribution factor K_t used in the neutral point equation.

    :param eta_h: Dynamic pressure ratio at the tail, q_tail / q [-].
    :param tail_area: Horizontal tail area S_h [m^2].
    :param wing_area: Wing reference area S [m^2].
    :param cl_alpha_h: 3D lift curve slope of the horizontal tail [1/rad].
    :param cl_alpha_wing: 3D lift curve slope of the wing [1/rad].
    :param downwash_gradient: Downwash gradient d(epsilon)/d(alpha) [-].
    :return: Tail factor K_t [-].
    """
    return (
        eta_h * (tail_area / wing_area) * (cl_alpha_h / cl_alpha_wing) * (1 - downwash_gradient)
    )

def get_neutral_point(
    *,
    x_ac_wing: ArrayLike,
    x_ac_tail: ArrayLike,
    mac: ArrayLike,
    cm_alpha_fuselage: ArrayLike,
    cl_alpha_wing: ArrayLike,
    tail_factor: ArrayLike,
) -> ArrayLike:
    """Position of the stick-fixed neutral point.

    The tail lever is measured from the neutral point itself, so x_np appears on both sides
    of the moment equation; this is the closed-form solution of that equation.

    :param x_ac_wing: Position of the wing aerodynamic centre [m].
    :param x_ac_tail: Position of the tail aerodynamic centre [m].
    :param mac: Mean aerodynamic chord of the wing [m].
    :param cm_alpha_fuselage: Fuselage Cm_alpha [1/rad]; positive = destabilising.
    :param cl_alpha_wing: 3D lift curve slope of the wing [1/rad].
    :param tail_factor: Tail factor K_t [-].
    :return: Position of the neutral point [m].
    """
    fuselage_shift = mac * cm_alpha_fuselage / cl_alpha_wing  # dCm_F/dCL in metres
    return (x_ac_wing - fuselage_shift + tail_factor * x_ac_tail) / (1 + tail_factor)

def get_static_margin(
    *,
    x_np: ArrayLike,
    x_cg: ArrayLike,
    mac: ArrayLike,
) -> ArrayLike:
    """Static margin.

    :param x_np: Position of the neutral point [m].
    :param x_cg: Position of the centre of gravity [m].
    :param mac: Mean aerodynamic chord of the wing [m].
    :return: Static margin as a fraction of the MAC [-]; positive = stable.
    """
    return (x_np - x_cg) / mac

# --------------------------------------------------------------------------- #
# Trim & tail incidence
# --------------------------------------------------------------------------- #


def get_clh_req(
    *,
    cm_ac: ArrayLike,
    cl_wing: ArrayLike,
    x_cg: ArrayLike,
    x_ac_wing: ArrayLike,
    mac: ArrayLike,
    eta_h: ArrayLike,
    tail_volume_coeff: ArrayLike,
) -> ArrayLike:
    """Tail lift coefficient needed for zero pitching moment about the CG.

    :param cm_ac: Pitching moment coefficient of the wing airfoil about its aerodynamic centre [-].
    :param cl_wing: Wing lift coefficient in the flight condition [-].
    :param x_cg: Position of the centre of gravity [m].
    :param x_ac_wing: Position of the wing aerodynamic centre [m].
    :param mac: Mean aerodynamic chord of the wing [m].
    :param eta_h: Dynamic pressure ratio at the tail, q_tail / q [-].
    :param tail_volume_coeff: Horizontal tail volume coefficient V_H [-].
    :return: Required tail lift coefficient [-]; negative means down-force.
    """
    return (cm_ac + cl_wing * (x_cg - x_ac_wing) / mac) / (eta_h * tail_volume_coeff)

def get_alpha_h(
    *,
    clh_req: ArrayLike,
    cl_alpha_h: ArrayLike,
) -> ArrayLike:
    """Tail angle of attack that produces the required tail lift (symmetric tail airfoil).

    :param clh_req: Required tail lift coefficient [-].
    :param cl_alpha_h: 3D lift curve slope of the horizontal tail [1/rad].
    :return: Tail angle of attack [rad].
    """
    return clh_req / cl_alpha_h

def get_eps(
    *,
    downwash_gradient: ArrayLike,
    cl_wing: ArrayLike,
    cl_alpha_wing: ArrayLike,
) -> ArrayLike:
    """Downwash angle at the horizontal tail.

    :param downwash_gradient: Downwash gradient d(epsilon)/d(alpha) [-].
    :param cl_wing: Wing lift coefficient in the flight condition [-].
    :param cl_alpha_wing: 3D lift curve slope of the wing [1/rad].
    :return: Downwash angle at the tail [rad].
    """
    return downwash_gradient * cl_wing / cl_alpha_wing

def get_alpha_w(
    *,
    cl_wing: ArrayLike,
    cl_alpha_wing: ArrayLike,
    alpha_0l: ArrayLike,
) -> ArrayLike:
    """Wing angle of attack for a given lift coefficient.

    :param cl_wing: Wing lift coefficient in the flight condition [-].
    :param cl_alpha_wing: 3D lift curve slope of the wing [1/rad].
    :param alpha_0l: Zero-lift angle of the wing airfoil [rad]; negative for cambered airfoils.
    :return: Wing angle of attack [rad].
    """
    return cl_wing / cl_alpha_wing + alpha_0l

def get_alpha_f(
    *,
    alpha_w: ArrayLike,
    i_w: ArrayLike,
) -> ArrayLike:
    """Fuselage angle of attack (angle between fuselage reference line and flow).

    :param alpha_w: Wing angle of attack [rad].
    :param i_w: Wing incidence relative to the fuselage reference line [rad].
    :return: Fuselage angle of attack [rad].
    """
    return alpha_w - i_w

def get_i_h(
    *,
    alpha_h: ArrayLike,
    alpha_f: ArrayLike,
    eps: ArrayLike,
) -> ArrayLike:
    """Tail incidence needed to trim, relative to the fuselage reference line.

    :param alpha_h: Required tail angle of attack [rad].
    :param alpha_f: Fuselage angle of attack [rad].
    :param eps: Downwash angle at the tail [rad].
    :return: Tail incidence [rad]; negative means leading edge down.
    """
    return alpha_h - alpha_f + eps
