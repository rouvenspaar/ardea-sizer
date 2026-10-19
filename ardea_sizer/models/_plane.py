from dataclasses import dataclass

from ardea_sizer.physics import get_mac, get_root_chord, get_span
from ._base import Masses


@dataclass(frozen=True)
class Surface:
    """Trapezoidal lifting surface. Wing & H-tail: both halves; V-tail: the single fin."""
    area: float  # [m^2]
    aspect_ratio: float  # span^2 / area (V-tail: height^2 / area) [-]
    taper: float  # tip / root chord [-]
    x_le: float  # root leading edge from the nose [m]
    incidence: float = 0.0  # [deg], positive leading edge up
    sweep_location: float = 0.5  # chord fraction of the unswept line (OpenVSP Sweep_Location)

    @property
    def span(self) -> float:
        """Span [m]; height for the V-tail."""
        return get_span(area=self.area, aspect_ratio=self.aspect_ratio)

    @property
    def root_chord(self) -> float:
        return get_root_chord(area=self.area, span=self.span, taper=self.taper)

    @property
    def tip_chord(self) -> float:
        return self.taper * self.root_chord

    @property
    def mac(self) -> float:
        return get_mac(root_chord=self.root_chord, taper=self.taper)

    @property
    def x_le_mac(self) -> float:
        """Leading edge of the mean aerodynamic chord from the nose [m]."""
        return self.x_le + self.sweep_location * (self.root_chord - self.mac)

    @property
    def x_ac(self) -> float:
        """Aerodynamic centre (25 % MAC) from the nose [m]."""
        return self.x_le_mac + 0.25 * self.mac


@dataclass(frozen=True)
class Plane:
    # mass
    masses: Masses  # all components with positions, fuel = sized fuel mass

    # geometry
    wing: Surface
    wing_airfoil: str  # NACA 4-digit code, root and tip
    h_tail: Surface
    v_tail: Surface

    # aerodynamics
    c_d_0: float  # whole plane, referenced to wing.area

    # performance
    range: float  # [m]
    power_req: float  # cruise [W]
    power_available: float  # [kW]

    @property
    def fuel_mass(self) -> float:
        """[kg]"""
        return self.masses.fuel.mass

    @property
    def mtow(self) -> float:
        """[kg]"""
        return self.masses.total

    @property
    def x_cg(self) -> float:
        """Centre of gravity with full tank, from the nose [m]."""
        m = self.masses
        return (m.empty * m.x_cg + m.fuel.mass * m.fuel.x) / m.total

    # shortcuts to the wing
    @property
    def wing_area(self) -> float:
        return self.wing.area

    @property
    def aspect_ratio(self) -> float:
        return self.wing.aspect_ratio

    @property
    def taper(self) -> float:
        return self.wing.taper
