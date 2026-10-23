from dataclasses import dataclass
from pathlib import Path

import numpy as np
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
BOOM_XSEC = -2                # fuselage cross-section closing the tail boom (the last one is the end cap)
BOOM_OVERHANG = 0.02          # boom end behind the trailing edge of the tails [m], as in the original model
EXCRES_PERCENT_GEOM = "% of CD_Geom"  # Excres_Type of an excrescence given as a percentage of the geometry drag

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


def set_perc_lam(perc_lam: dict[str, float]) -> None:
    """Set the laminar flow percentage of each named component of the loaded model; others keep the file's."""
    for name, value in perc_lam.items():
        geom_id = vsp.FindGeomsWithName(name)[0]
        vsp.SetParmVal(geom_id, "PercLam", "ParasiteDragProps", value)
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
    wing_y: float | None = None,
    wing_z: float | None = None,
    h_z: float | None = None,
    v_z: float | None = None,
    out_file: str | Path | None = OUTPUT_FILE,
) -> None:
    wing_id = _size_surface(WING_NAME, area=wing_area, ar=wing_ar, tr=wing_tr, x=wing_x, z=wing_z, ia=wing_ia)
    _set_naca_4(wing_id, wing_airfoil)
    vsp.Update()  # root chord and position for the fuselage width below
    if wing_y is None:
        wing_y = _fuselage_half_width(wing_id, wing_airfoil)
    vsp.SetParmVal(wing_id, "Y_Rel_Location", "XForm", float(wing_y))
    tail_ids = [
        _size_surface(H_TAIL_NAME, area=h_area, ar=h_ar, tr=h_tr, x=h_x, z=h_z, ia=h_ia),
        _size_surface(V_TAIL_NAME, area=v_area, ar=v_ar, tr=v_tr, x=v_x, z=v_z),
    ]
    vsp.Update()
    _fit_boom(tail_ids)
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


def _fit_boom(tail_ids: list[str]) -> None:
    """Stretch or shorten the tail boom so it ends BOOM_OVERHANG behind the rearmost tail root trailing edge.

    Only the constant-diameter boom section changes; the fuselage ahead of it and the end cap keep their shape.
    """
    fuselage_id = vsp.FindGeomsWithName(FUSELAGE_NAME)[0]
    surf_id = vsp.GetXSecSurf(fuselage_id, 0)
    i_boom = vsp.GetNumXSec(surf_id) + BOOM_XSEC
    boom_start = vsp.GetParmVal(fuselage_id, "X_Location", "XForm") + sum(
        vsp.GetParmVal(vsp.GetXSecParm(vsp.GetXSec(surf_id, i), "XDelta")) for i in range(i_boom)
    )
    tail_end = max(
        vsp.GetParmVal(gid, "X_Location", "XForm") + vsp.GetParmVal(gid, "Root_Chord", "XSec_1")
        for gid in tail_ids
    )
    boom_length = tail_end + BOOM_OVERHANG - boom_start
    if boom_length <= 0:
        raise ValueError(f"Tails end at x = {tail_end:.3f} m, ahead of the tail boom (starts at {boom_start:.3f} m).")
    vsp.SetParmVal(vsp.GetXSecParm(vsp.GetXSec(surf_id, i_boom), "XDelta"), float(boom_length))
    vsp.Update()


def _fuselage_half_width(wing_id: str, airfoil: str, overlap: float = 0.001) -> float:
    """Spanwise root offset that keeps the whole root airfoil inside the fuselage [m].

    The fuselage half-width at every point of the root airfoil outline (upper and lower surface,
    with incidence), measured on the fuselage surface; the smallest one minus a small overlap.
    """
    x_le = vsp.GetParmVal(wing_id, "X_Location", "XForm")
    z_le = vsp.GetParmVal(wing_id, "Z_Location", "XForm")
    chord = vsp.GetParmVal(wing_id, "Root_Chord", "XSec_1")
    ia = np.radians(vsp.GetParmVal(wing_id, "Y_Rotation", "XForm"))

    xc, zc = _naca_4_outline(airfoil)
    x = x_le + chord * (xc * np.cos(ia) + zc * np.sin(ia))
    z = z_le + chord * (zc * np.cos(ia) - xc * np.sin(ia))  # positive incidence: trailing edge down

    fuselage_id = vsp.FindGeomsWithName(FUSELAGE_NAME)[0]
    half_widths = [vsp.AxisProjPnt01(fuselage_id, 0, vsp.Y_DIR, vsp.vec3d(xi, 0.0, zi))[0] for xi, zi in zip(x, z)]
    return float(min(half_widths)) - overlap


def _naca_4(code: str) -> tuple[float, float, float]:
    """(max camber, camber position, thickness) of a NACA 4-digit code, as chord fractions."""
    if len(code) != 4 or not code.isdigit():
        raise ValueError(f"Not a NACA 4-digit code: {code!r}")
    return int(code[0]) / 100, int(code[1]) / 10, int(code[2:]) / 100


def _naca_4_outline(code: str, n: int = 40) -> tuple[np.ndarray, np.ndarray]:
    """Upper and lower surface points (x, z) of a NACA 4-digit airfoil with unit chord, sharp trailing edge."""
    m, p, t = _naca_4(code)
    xc = (1 - np.cos(np.linspace(0, np.pi, n))) / 2  # clustered at the edges
    yt = 5 * t * (0.2969 * np.sqrt(xc) - 0.1260 * xc - 0.3516 * xc**2 + 0.2843 * xc**3 - 0.1036 * xc**4)
    if m > 0:
        yc = np.where(xc < p, m / p**2 * (2 * p * xc - xc**2), m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * xc - xc**2))
    else:
        yc = np.zeros_like(xc)
    return np.concatenate([xc, xc]), np.concatenate([yc + yt, yc - yt])


def _set_naca_4(geom_id: str, code: str) -> None:
    """Give every cross-section of a wing geom the same NACA 4-digit airfoil."""
    camber, camber_loc, thickness = _naca_4(code)

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
    """Return (c_d_0_wing, f_fixed) of the OpenVSP model sized to the plane.

    Loads the .vsp3 file (unchanged on disk), sets Q_VALUES and plane.wing_perc_lam, sizes wing and stabilizers
    to the plane with size_model (saved to OUTPUT_FILE), and runs the parasite drag build-up with the
    flow conditions saved in the file. No report files are kept.
    c_d_0_wing is the wing's CD on its own area; f_fixed the drag area CD*S of all other parts [m^2].
    Excrescences given as a % of the geometry drag are split between the two by their geometry drag,
    the others (drag area, CD, count) go to f_fixed; together they are OpenVSP's total.
    """
    _load_sized(plane, out_file=OUTPUT_FILE)
    rid = _parasite_drag()
    labels = list(vsp.GetStringResults(rid, "Comp_Label", 0))
    i_wing = labels.index(WING_NAME)
    c_d_wing = vsp.GetDoubleResults(rid, "Comp_CD", 0)[i_wing]  # Sref = wing area
    comp_f = list(vsp.GetDoubleResults(rid, "Comp_f", 0))
    f_other = sum(comp_f) - comp_f[i_wing]

    percent, f_excres = 0.0, 0.0
    for kind, value, f in zip(
        vsp.GetStringResults(rid, "Excres_Type", 0),
        vsp.GetDoubleResults(rid, "Excres_Input", 0),
        vsp.GetDoubleResults(rid, "Excres_f", 0),
    ):
        if kind == EXCRES_PERCENT_GEOM:
            percent += value
        elif "margin" in kind.lower():  # % of the total incl. excrescences: not split between wing and rest
            raise ValueError(f"Excrescence type {kind!r} is not supported; use '{EXCRES_PERCENT_GEOM}' instead.")
        else:  # drag area, CD or count: independent of the wing
            f_excres += f
    k = 1 + percent / 100
    return float(k * c_d_wing), float(k * f_other + f_excres)


@dataclass(frozen=True)
class VspSurface:
    """Planform of a wing geom as built in OpenVSP. Wing & H-tail: both halves; V-tail: the single fin."""
    area: float  # [m^2]
    span: float  # tip to tip, incl. the root offset of both halves [m]; height for the V-tail
    root_offset: float  # spanwise offset of each root from the centreline [m]
    root_chord: float  # [m]
    tip_chord: float  # [m]
    mean_chord: float  # area / panel span [m]
    x: float  # root leading edge from the nose [m]
    z: float  # root leading edge above the datum [m]
    incidence: float  # [deg]


@dataclass(frozen=True)
class DragItem:
    """One row of the parasite drag build-up; excrescences only have f and c_d."""
    name: str
    f: float  # drag area CD*S [m^2]
    c_d: float  # referenced to VspModel.s_ref
    s_wet: float = np.nan  # [m^2]
    l_ref: float = np.nan  # reference length [m]
    fineness: float = np.nan  # t/c for surfaces, l/d for bodies
    form_factor: float = np.nan
    re: float = np.nan
    c_f: float = np.nan
    q: float = np.nan  # interference factor


@dataclass(frozen=True)
class VspModel:
    """Geometry and parasite drag of the OpenVSP model sized to a plane."""
    wing: VspSurface
    h_tail: VspSurface
    v_tail: VspSurface
    length: float  # overall [m]
    width: float  # overall, tip to tip [m]
    height: float  # overall [m]
    fuselage_length: float  # [m]
    fuselage_width: float  # [m]
    fuselage_height: float  # [m]
    altitude: float  # flow conditions of the drag build-up [m]
    v_inf: float  # [m/s]
    mach: float
    s_ref: float  # [m^2]
    drag: list[DragItem]  # components, then excrescences
    f_total: float  # [m^2]
    c_d_0: float  # referenced to s_ref


def get_vsp_model(plane: Plane) -> VspModel:
    """Size the OpenVSP model to the plane (saved to OUTPUT_FILE) and read back its geometry and drag build-up."""
    _load_sized(plane, out_file=OUTPUT_FILE)
    wing, h_tail, v_tail = (_read_surface(name) for name in (WING_NAME, H_TAIL_NAME, V_TAIL_NAME))

    boxes = [_bbox(gid) for gid in vsp.FindGeoms() if vsp.GetGeomName(gid) not in EXCLUDE]
    lo = np.min([b[0] for b in boxes], axis=0)
    hi = np.max([b[1] for b in boxes], axis=0)
    f_lo, f_hi = _bbox(vsp.FindGeomsWithName(FUSELAGE_NAME)[0])

    rid = _parasite_drag()

    def results(name: str) -> list[float]:
        return list(vsp.GetDoubleResults(rid, name, 0))

    s_ref = results("FC_Sref")[0]
    drag = [
        DragItem(name=name, f=f, c_d=c_d, s_wet=s_wet, l_ref=l_ref, fineness=fineness,
                 form_factor=ff, re=re, c_f=c_f, q=q)
        for name, f, c_d, s_wet, l_ref, fineness, ff, re, c_f, q in zip(
            vsp.GetStringResults(rid, "Comp_Label", 0), results("Comp_f"), results("Comp_CD"),
            results("Comp_Swet"), results("Comp_Lref"), results("Comp_FineRat"), results("Comp_FFOut"),
            results("Comp_Re"), results("Comp_Cf"), results("Comp_Q"),
        )
    ]
    if vsp.GetIntResults(rid, "Num_Excres", 0)[0]:
        drag += [DragItem(name=name, f=f, c_d=f / s_ref)
                 for name, f in zip(vsp.GetStringResults(rid, "Excres_Label", 0), results("Excres_f"))]

    return VspModel(
        wing=wing,
        h_tail=h_tail,
        v_tail=v_tail,
        length=float(hi[0] - lo[0]),
        width=float(2 * max(abs(lo[1]), abs(hi[1]))),  # mirrored geoms report one half only
        height=float(hi[2] - lo[2]),
        fuselage_length=float(f_hi[0] - f_lo[0]),
        fuselage_width=float(f_hi[1] - f_lo[1]),
        fuselage_height=float(f_hi[2] - f_lo[2]),
        altitude=results("FC_Alt")[0],
        v_inf=results("FC_Vinf")[0],
        mach=results("FC_Mach")[0],
        s_ref=s_ref,
        drag=drag,
        f_total=results("Total_f_Total")[0],
        c_d_0=results("Total_CD_Total")[0],
    )


def _load_sized(plane: Plane, *, out_file: str | Path | None) -> None:
    """Load the .vsp3 file (unchanged on disk), set Q_VALUES and the wing's laminar flow, and size it to the plane."""
    w, h, v = plane.wing, plane.h_tail, plane.v_tail
    vsp.ClearVSPModel()
    vsp.ReadVSPFile(str(VSP3_FILE))
    set_q_values()
    set_perc_lam({WING_NAME: plane.wing_perc_lam})
    size_model(
        wing_area=w.area,
        wing_ar=w.aspect_ratio,
        wing_tr=w.taper,
        wing_x=w.x_le,
        wing_ia=w.incidence,
        wing_airfoil=plane.wing_airfoil,
        h_area=h.area,
        h_ar=h.aspect_ratio,
        h_tr=h.taper,
        h_x=h.x_le,
        h_ia=h.incidence,
        v_area=v.area,
        v_ar=v.aspect_ratio,
        v_tr=v.taper,
        v_x=v.x_le,
        out_file=out_file,
    )


def _parasite_drag() -> str:
    wing_id = vsp.FindGeomsWithName(WING_NAME)[0]

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
    return rid


def _read_surface(name: str) -> VspSurface:
    """Planform of the named wing geom of the loaded model."""
    geom_id = vsp.FindGeomsWithName(name)[0]
    n_sections = vsp.GetNumXSec(vsp.GetXSecSurf(geom_id, 0))
    mirrored = bool(vsp.GetParmVal(geom_id, "Sym_Planar_Flag", "Sym"))
    root_offset = vsp.GetParmVal(geom_id, "Y_Location", "XForm") if mirrored else 0.0
    return VspSurface(
        area=vsp.GetParmVal(geom_id, "TotalArea", "WingGeom"),
        span=vsp.GetParmVal(geom_id, "TotalProjectedSpan", "WingGeom") + 2 * root_offset,
        root_offset=root_offset,
        root_chord=vsp.GetParmVal(geom_id, "Root_Chord", "XSec_1"),
        tip_chord=vsp.GetParmVal(geom_id, "Tip_Chord", f"XSec_{n_sections - 1}"),
        mean_chord=vsp.GetParmVal(geom_id, "TotalChord", "WingGeom"),
        x=vsp.GetParmVal(geom_id, "X_Location", "XForm"),
        z=vsp.GetParmVal(geom_id, "Z_Location", "XForm"),
        incidence=vsp.GetParmVal(geom_id, "Y_Rotation", "XForm"),
    )


def _bbox(geom_id: str) -> tuple[np.ndarray, np.ndarray]:
    """(min, max) corner of the geom's bounding box in absolute coordinates [m]."""
    lo = vsp.GetGeomBBoxMin(geom_id, 0, True)
    hi = vsp.GetGeomBBoxMax(geom_id, 0, True)
    return np.array([lo.x(), lo.y(), lo.z()]), np.array([hi.x(), hi.y(), hi.z()])
