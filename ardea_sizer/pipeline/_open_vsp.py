from pathlib import Path

import openvsp as vsp

from ardea_sizer.models import Plane

VSP3_FILE = Path(__file__).resolve().parents[2] / "models" / "20261012_Convent_BM_V3.vsp3"
OUTPUT_FILE = VSP3_FILE.with_name("output.vsp3")  # sized model, written by size_model
WING_NAME = "Main Wing"
H_TAIL_NAME = "Horizontal stabilizer"
V_TAIL_NAME = "vertical stabilizer"
FUSELAGE_NAME = "ARDEA_D3_Stack"
GEOM_SET = 1                  # "Shown"
EXCLUDE = ["CG_Marker"]       # not part of the aircraft

# Not in Plane yet: fixed at the values of the V3 model
WING_X = 0.445                # root leading edge [m]
WING_IA = 0.7                 # [deg]
WING_AIRFOIL = "2412"
H_TAIL = dict(h_area=0.06, h_ar=2.0, h_tr=0.7, h_x=1.5, h_ia=0.0)
V_TAIL = dict(v_area=0.03, v_ar=4.5, v_tr=0.7, v_x=1.5)

# Interference factors Q, Raymer Table 12.6
Q_VALUES = {
    FUSELAGE_NAME: 1.00,      # fuselage relative to other components
    WING_NAME: 1.05,          # high, mid or well-filleted low wing
    H_TAIL_NAME: 1.05,        # conventional tail group
    V_TAIL_NAME: 1.05,        # conventional tail group
}


def set_q_values(q_values: dict[str, float] = Q_VALUES) -> None:
    """Set the parasite drag interference factor Q of each named component of the loaded model."""
    for name, q in q_values.items():
        geom_id = vsp.FindGeomsWithName(name)[0]
        vsp.SetParmVal(geom_id, "Q", "ParasiteDragProps", q)
    vsp.Update()


def size_model(
    *,
    wing_area: float,
    wing_ar: float,
    wing_tr: float,
    wing_x: float,
    wing_ia: float,
    wing_airfoil: str,
    h_area: float,
    h_ar: float,
    h_tr: float,
    h_x: float,
    h_ia: float,
    v_area: float,
    v_ar: float,
    v_tr: float,
    v_x: float,
    wing_z: float | None = None,
    h_z: float | None = None,
    v_z: float | None = None,
    out_file: str | Path | None = OUTPUT_FILE,
) -> None:
    """Size main wing, horizontal and vertical stabilizer of the loaded OpenVSP model.

    Areas and aspect ratios are for the whole surface (both halves of the wing and the H-tail,
    the single fin of the V-tail). Positions are the root leading edge from the nose [m];
    a z left at None keeps the value in the file. Incidences are in degrees, positive leading edge up.
    wing_airfoil is a NACA 4-digit code (e.g. "2412"), used at root and tip.
    The sized model is saved to out_file (None: not saved); the file it was loaded from is not touched.
    """
    wing_id = _size_surface(WING_NAME, area=wing_area, ar=wing_ar, tr=wing_tr, x=wing_x, z=wing_z, ia=wing_ia)
    _set_naca_4(wing_id, wing_airfoil)
    _size_surface(H_TAIL_NAME, area=h_area, ar=h_ar, tr=h_tr, x=h_x, z=h_z, ia=h_ia)
    _size_surface(V_TAIL_NAME, area=v_area, ar=v_ar, tr=v_tr, x=v_x, z=v_z)
    vsp.Update()
    if out_file is not None:
        vsp.WriteVSPFile(str(out_file))


def _size_surface(
    name: str,
    *,
    area: float,
    ar: float,
    tr: float,
    x: float,
    z: float | None,
    ia: float | None = None,
) -> str:
    """Set area, aspect ratio, taper, position and incidence of a one-section wing geom; return its id."""
    geom_id = vsp.FindGeomsWithName(name)[0]

    # Section values of a mirrored surface are per half: half the area, half the aspect ratio
    halves = 2 if vsp.GetParmVal(geom_id, "Sym_Planar_Flag", "Sym") else 1
    vsp.SetDriverGroup(geom_id, 1, vsp.AR_WSECT_DRIVER, vsp.AREA_WSECT_DRIVER, vsp.TAPER_WSECT_DRIVER)
    vsp.SetParmVal(geom_id, "Area", "XSec_1", float(area) / halves)
    vsp.SetParmVal(geom_id, "Aspect", "XSec_1", float(ar) / halves)
    vsp.SetParmVal(geom_id, "Taper", "XSec_1", float(tr))

    # Attached to the fuselage, whose origin is the nose
    vsp.SetParmVal(geom_id, "X_Rel_Location", "XForm", float(x))
    if z is not None:
        vsp.SetParmVal(geom_id, "Z_Rel_Location", "XForm", float(z))
    if ia is not None:
        vsp.SetParmVal(geom_id, "Y_Rel_Rotation", "XForm", float(ia))
    return geom_id


def _set_naca_4(geom_id: str, code: str) -> None:
    """Give every cross-section of a wing geom the same NACA 4-digit airfoil."""
    if len(code) != 4 or not code.isdigit():
        raise ValueError(f"Not a NACA 4-digit code: {code!r}")
    camber, camber_loc, thickness = int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100

    surf_id = vsp.GetXSecSurf(geom_id, 0)
    for i in range(vsp.GetNumXSec(surf_id)):
        if vsp.GetXSecShape(vsp.GetXSec(surf_id, i)) != vsp.XS_FOUR_SERIES:
            vsp.ChangeXSecShape(surf_id, i, vsp.XS_FOUR_SERIES)
        xsec_id = vsp.GetXSec(surf_id, i)
        vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "Camber"), camber)
        if camber > 0:  # symmetric section: no camber location
            vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "CamberLoc"), camber_loc)
        vsp.SetParmVal(vsp.GetXSecParm(xsec_id, "ThickChord"), thickness)


def optimize(plane: Plane) -> tuple[float, float]:
    """Return (c_d_0_wing, f_fixed) of the OpenVSP model with the sized wing.

    Loads the .vsp3 file (unchanged on disk), sets the interference factors Q_VALUES, sizes it
    with size_model (wing area, aspect ratio and taper from the plane, everything else from the
    constants above; saved to OUTPUT_FILE), and runs the parasite drag build-up with the flow
    conditions saved in the file. No report files are kept.
    c_d_0_wing is the wing's CD on its own area; f_fixed the drag area CD*S of all other parts [m^2].
    """
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(str(VSP3_FILE))
    set_q_values()
    size_model(
        wing_area=plane.wing_area,
        wing_ar=plane.aspect_ratio,
        wing_tr=plane.taper,
        wing_x=WING_X,
        wing_ia=WING_IA,
        wing_airfoil=WING_AIRFOIL,
        **H_TAIL,
        **V_TAIL,
    )
    wing_id = vsp.FindGeomsWithName(WING_NAME)[0]

    # Analysis-only changes, after saving
    for name in EXCLUDE:
        for gid in vsp.FindGeomsWithName(name):
            vsp.SetSetFlag(gid, GEOM_SET, False)
    vehicle = vsp.FindContainer("Vehicle", 0)
    for pid in vsp.FindContainerParmIDs(vehicle):
        if vsp.GetParmName(pid) in ("CompGeom_CSV_Export", "CompGeom_TXT_Export"):
            vsp.SetParmVal(pid, 0)

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
    # The build-up table is always written next to the model file; only the results are needed
    vsp_file = Path(vsp.GetVSPFileName())
    vsp_file.with_name(f"{vsp_file.stem}_ParasiteBuildUp.csv").unlink(missing_ok=True)
    labels = list(vsp.GetStringResults(rid, "Comp_Label", 0))
    i_wing = labels.index(WING_NAME)
    c_d_0_wing = vsp.GetDoubleResults(rid, "Comp_CD", 0)[i_wing]  # Sref = wing area
    f_fixed = vsp.GetDoubleResults(rid, "Total_f_Total", 0)[0] - vsp.GetDoubleResults(rid, "Comp_f", 0)[i_wing]
    return float(c_d_0_wing), float(f_fixed)
