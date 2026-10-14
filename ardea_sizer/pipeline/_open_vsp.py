from pathlib import Path

import openvsp as vsp

from ardea_sizer.models import Plane

VSP3_FILE = Path(__file__).resolve().parents[2] / "models" / "20261012_Convent_BM_V3.vsp3"
WING_NAME = "Main Wing"
GEOM_SET = 1                  # "Shown"
EXCLUDE = ["CG_Marker"]       # not part of the aircraft


def optimize(plane: Plane) -> float:
    """Return the parasite drag coefficient CD0 of the OpenVSP model with the sized wing.

    Loads the .vsp3 file (unchanged on disk), sets the main wing's area, aspect ratio and taper,
    and runs the parasite drag build-up with the flow conditions saved in the file.
    CD0 is referenced to the wing area, as c_d_0 in the sizer.
    """
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(str(VSP3_FILE))
    for name in EXCLUDE:
        for gid in vsp.FindGeomsWithName(name):
            vsp.SetSetFlag(gid, GEOM_SET, False)

    # One trapezoidal section; its values are per half-wing. Span and chords follow.
    wing_id = vsp.FindGeomsWithName(WING_NAME)[0]
    vsp.SetDriverGroup(wing_id, 1, vsp.AR_WSECT_DRIVER, vsp.AREA_WSECT_DRIVER, vsp.TAPER_WSECT_DRIVER)
    vsp.SetParmVal(wing_id, "Area", "XSec_1", float(plane.wing_area) / 2)
    vsp.SetParmVal(wing_id, "Aspect", "XSec_1", float(plane.aspect_ratio) / 2)
    vsp.SetParmVal(wing_id, "Taper", "XSec_1", float(plane.taper))
    vsp.Update()

    an = "ParasiteDrag"
    vsp.SetAnalysisInputDefaults(an)
    settings = vsp.FindContainer("ParasiteDragSettings", 0)
    saved = {vsp.GetParmName(pid): pid for pid in vsp.FindContainerParmIDs(settings)}
    for name in vsp.GetAnalysisInputNames(an):
        if name not in saved:
            continue
        kind = vsp.GetAnalysisInputType(an, name)
        if kind == vsp.INT_DATA:
            vsp.SetIntAnalysisInput(an, name, [int(vsp.GetParmVal(saved[name]))])
        elif kind == vsp.DOUBLE_DATA:
            vsp.SetDoubleAnalysisInput(an, name, [vsp.GetParmVal(saved[name])])
    vsp.SetIntAnalysisInput(an, "GeomSet", [GEOM_SET])
    vsp.SetIntAnalysisInput(an, "RefFlag", [0])
    vsp.SetDoubleAnalysisInput(an, "Sref", [vsp.GetParmVal(wing_id, "TotalArea", "WingGeom")])

    rid = vsp.ExecAnalysis(an)
    return float(vsp.GetDoubleResults(rid, "Total_CD_Total", 0)[0])
