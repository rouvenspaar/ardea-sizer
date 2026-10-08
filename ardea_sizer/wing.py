from .inputs import Inputs


def get_wing_area(inp: Inputs) -> float:
    return max(
        get_wing_area_cruise(inp),
        get_wing_area_takeoff(inp)
    )

def get_wing_area_cruise(inp: Inputs) -> float:
    mtow = inp.mass.total
    g = 9.81  # TODO
    roh = 2  # TODO
    v = inp.cruise_speed

    #return (2 * mtow * g) / (roh * v**2) * math.sqrt(1 / (math.pi())
    return 0.1

def get_wing_area_takeoff(inp: Inputs) -> float:
    mtow = inp.mass.total
    c_l = 2 #TODO
    g = 9.81 #TODO
    roh = 2 #TODO
    v = inp.stall_speed

    return (2 * mtow * g) / (roh * v**2 * c_l)
