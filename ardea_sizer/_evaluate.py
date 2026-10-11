from ardea_sizer import Base
from ardea_sizer.physics import get_wing_area, get_total_mass, get_total_drag, get_required_power, get_range


def evaluate(base: Base, ar: float, fuel_weight: float):
    mass = get_total_mass()
    area = get_wing_area()
    drag = get_total_drag()
    power = get_required_power()
    range = get_range()

    print(range)
