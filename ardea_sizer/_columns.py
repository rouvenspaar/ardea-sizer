from enum import StrEnum

class C(StrEnum):
    # mission
    RANGE        = "mission.range"
    CRUISE_SPEED = "mission.cruise_speed"
    STALL_SPEED  = "mission.stall_speed"
    # aero
    CL_MAX       = "aero.c_l_max_3d"
    # mass
    FUEL         = "mass.fuel"
    REST         = "mass.rest"
    # engine
    P_MAX        = "engine.p_max"
    FUEL_CONS    = "engine.fuel_cons"
    ENGINE_EFF   = "engine.engine_eff"
    PROP_EFF     = "engine.prop_eff"
    # geometry
    AR           = "geometry.AR"
    TAPER        = "geometry.taper"
    OSWALD       = "geometry.oswald_eff"
