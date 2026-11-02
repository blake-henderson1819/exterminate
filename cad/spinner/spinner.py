"""Parametric balanced single-tooth horizontal bar spinner (CadQuery).
Spin axis = Z through origin. Arm (striking end) along +X, counterweight disc toward -X.
Spin direction: CCW seen from +Z, so the tooth's impact face is on the +Y (leading) side.
Plate is extruded symmetrically about z=0 -> mid-plane symmetry -> Z is a principal axis.
Run:  python spinner.py        -> 80 mm version (rect hole pattern): spinner_80mm.step/.stl, report.json
      python spinner.py 80v2   -> 80 mm v2 (plus hole pattern):    spinner_80mm_v2.step/.stl, report_v2.json
      python spinner.py 80v3   -> 80 mm v3 short wide counterweight:  spinner_80mm_v3.step/.stl, report_v3.json
      python spinner.py 120v4  -> 120 mm v4 snowman (disc + hub + hooked arm): spinner_120mm_v4.step/.stl, report_v4.json
      python spinner.py 120v5  -> 120 mm v5 = v4 with M2 plus-pattern holes (11.5 mm c-c): spinner_120mm_v5.step/.stl, report_v5.json
      python spinner.py 120v6  -> 120 mm v6 = v5 + Ø7.8 x 15 mm hollow peg on -Z face: spinner_120mm_v6.step/.stl, report_v6.json
      python spinner.py 120v7  -> 120 mm v7 = v6 with Ø2.4 M2 clearance holes + Ø4.4x2.2 top counterbores, 8 mm plate: spinner_120mm_v7.step/.stl, report_v7.json
      python spinner.py 120v8  -> 120 mm v8 = v7 with the Ø4.4x2.2 counterbores moved to the -Z (peg) face, cut through the peg root: spinner_120mm_v8.step/.stl, report_v8.json
      python spinner.py 160    -> PARKED 2x-length study (not current), 160 mm version (plus hole pattern):  spinner_160mm.step/.stl, report_160.json
"""
import json, math
import cadquery as cq
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

P = dict(
    R_MAX=80.0,        # tip radius (mm) - nothing beyond this
    T=10.0,            # plate thickness
    RHO=7.85e-3,       # g/mm^3 (AR500 / 4140)
    BORE=3.8,          # central spindle hole dia
    HUB_R=18.0,        # hub boss radius (min ~16.7 for 3 mm wall around the diagonal holes)
    HOLE_D=2.5,        # M3 thread-tapping / thread-forming pilot dia
    HOLE_DX=19.0,      # hole spacing along major (arm, X) axis
    HOLE_DY=16.0,      # hole spacing along minor (Y) axis
    ARM_W0=30.0,       # arm width at the axis
    ARM_W1=18.0,       # arm width at the tip (before tooth)
    TOOTH_L=16.0,      # length of tooth block measured in from the tip
    TOOTH_H=4.0,       # tooth protrusion on leading (+Y) side
    POCKET_X0=26.0, POCKET_X1=56.0,   # lightening pocket span (centre-line)
    POCKET_W0=12.0, POCKET_W1=6.0,    # pocket width at inner/outer ends
    DISC_R=28.0,       # counterweight disc radius
    DISC_CX=-27.0,     # disc centre x (solved)
    DISC_CY=0.0,       # disc centre y (solved; trims the tooth's Y moment)
    FILLET=3.0,        # 2D fillet on every profile corner (>=2 mm internal)
    HOLE_PATTERN="rect",  # "rect": (+-DX/2, +-DY/2) corners; "plus": (+-DX/2,0),(0,+-DY/2)
)

# 2x-length variant: tip at r=160, plus/cross motor bolt pattern (D2830: 19 mm along X, 16 mm along Y).
# Length-wise features ~2x; widths modestly increased for strength (not 2x - keeps it a bar).
P160 = dict(P,
    R_MAX=160.0,
    HOLE_PATTERN="plus",
    HUB_R=21.0,        # hub boss radius (was 18; keeps arm root corners off the hub circle, more meat round bolts)
    ARM_W0=36.0,       # arm width at the axis (was 30)
    ARM_W1=22.0,       # arm width at the tip (was 18)
    TOOTH_L=24.0,      # tooth block length (was 16)
    TOOTH_H=5.0,       # tooth step (was 4)
    POCKET_X0=52.0, POCKET_X1=112.0,  # 2x the 26-56 slot
    POCKET_W0=14.0, POCKET_W1=8.0,
    DISC_R=33.0,       # counterweight disc radius (tuned so back extent ~2x the 80 mm version: ~-108.7 vs -54.1)
    DISC_CX=-75.6, DISC_CY=-0.4,  # solved
    NECK_W1=56.0,      # neck width where it meets the disc centre (neck = trapezoid from ARM_W0 at axis)
)
# v2: same 80 mm design, motor bolts in the D2830 plus/cross pattern (+-9.5,0),(0,+-8); re-balanced.
P80V2 = dict(P, HOLE_PATTERN="plus")
# v3: short counterweight. Back reach fixed (BACK), counterweight is a wide rectangular lobe whose
# width (LOBE_W) and y-offset (LOBE_CY) are solved for CG = axis. Striking arm slimmed, slot enlarged.
P80V3 = dict(P80V2,
    ARM_W0=26.0, ARM_W1=14.0,          # was 30 -> 18
    POCKET_X0=24.0, POCKET_X1=62.0,    # was 26-56
    POCKET_W0=12.0, POCKET_W1=6.0,
    DISC_R=0.0,                        # no disc
    BACK=34.0,                         # max reach behind the axis (was 54.1)
    LOBE_X1=-3.0,                      # lobe front face (hub bridges lobe to arm)
    LOBE_W=80.0, LOBE_CY=0.0,          # solved
    SOLVE_KEYS=("LOBE_W", "LOBE_CY"),
)
# v4 "snowman" at 120 mm reach: R28-style round disc counterweight + R18 hub + tapering arm with hooked tooth.
# Arm widths kept close to v2 (not scaled 1.5x) to hold mass down; slot positions scaled 1.5x.
P120V4 = dict(P80V2,
    R_MAX=120.0,
    ARM_W0=26.0, ARM_W1=16.0,          # v2: 30 -> 18 (slimmer to hold mass ~440 g; slot side walls >= 5 mm)
    TOOTH_L=22.0, TOOTH_H=5.0,         # v2: 16 x 4 (hook kept prominent, ~same proportion)
    POCKET_X0=34.0, POCKET_X1=94.0,    # v2 26-56, stretched for the 120 mm arm
    POCKET_W0=13.0, POCKET_W1=7.0,
    DISC_R=33.0,                       # counterweight disc radius (v2: 28)
    DISC_CX=-33.8, DISC_CY=-0.4,       # solved
    COLOR="#7B2A9B",                   # anodized purple (matches motor); written into the STEP
)
# v5: v4 with M2 thread-tapping holes: 4x Ø1.6 pilots, equal plus pattern 11.5 mm c-c (+-5.75,0),(0,+-5.75).
P120V5 = dict(P120V4, HOLE_D=1.6, HOLE_DX=11.5, HOLE_DY=11.5)
# v6: v5 + hollow axial peg on the -Z (motor) face: OD 7.8, 15 mm long, 3.8 bore through (2.0 mm wall).
P120V6 = dict(P120V5, PEG_D=7.8, PEG_L=15.0, PEG_FILLET=0.5)
# v7: v6 with 4x Ø2.4 M2 clearance holes (ISO 273 medium) instead of Ø1.6 pilots, and plate 10 -> 8 mm.
# Peg stays Ø7.8 x 15 mm proud of the -Z face (0.5 fillet), 3.8 bore through all. Outline unchanged.
# Counterbores Ø4.4 x 2.2 deep on the +Z (top, screw-entry) face, opposite the peg: flush M2 SHCS (Ø3.8x2.0) / pan (Ø4.0x1.6).
P120V7 = dict(P120V6, HOLE_D=2.4, T=8.0, CBORE_D=4.4, CBORE_DEPTH=2.2)
# v8: v7 with the four Ø4.4 x 2.2 counterbores on the -Z (peg/motor) face instead of +Z; +Z face has plain Ø2.4 holes.
# The c'bores (r 3.55..7.95) overlap the Ø7.8 peg (r 3.9) by 0.35 mm, so the c'bore tool is extended axially through
# the full peg length (CBORE_THRU_PEG): 4 shallow axial grooves in the peg so the screw heads can be inserted and seat.
P120V8 = dict(P120V7, CBORE_FACE="-Z", CBORE_THRU_PEG=True)
# v8b (fallback, only if the v8 peg wall were < 0.5 mm): Ø4.0 c'bores, which only just touch the peg (r 3.75..7.75 vs 3.9).
P120V8B = dict(P120V8, CBORE_D=4.0)
# v9 (2026-11-01): v8 with plate 8 -> 12 mm and a Ø28.2 x 2.0 motor-bell recess on the -Z face (Motor CAN Ø27.7 + 0.5,
# measured from camera_mount/asm_cam5.step). Peg Ø7.8 x 15 now rises from the recess floor (same 15 mm proud of the motor
# contact face); c'bores Ø4.4 x 2.2 measured from the recess floor; bore/holes unchanged. Disc re-solved for balance.
P120V9 = dict(P120V8, T=12.0, RECESS_D=28.2, RECESS_DEPTH=2.0)
# v10 (2026-11-01): v9 with the peg removed (flat -Z face) and the Ø28.2 x 2.0 motor-bell recess moved to +Z.
# Motor bell now seats in the +Z recess, so the screws enter from -Z: Ø4.4 x 2.2 c'bores on the -Z face (opposite the bell).
P120V10 = dict(P120V9, PEG_L=0.0, RECESS_FACE="+Z", CBORE_FACE="-Z", CBORE_THRU_PEG=False)
VARIANTS = {"120v10_12mm": (P120V10, "spinner_120mm_v10_12mm", "report_v10_12mm.json"), "120v9_12mm": (P120V9, "spinner_120mm_v9_12mm", "report_v9_12mm.json"), "120v8": (P120V8, "spinner_120mm_v8", "report_v8.json"), "120v8b": (P120V8B, "spinner_120mm_v8b", "report_v8b.json"), "120v7": (P120V7, "spinner_120mm_v7", "report_v7.json"), "120v6": (P120V6, "spinner_120mm_v6", "report_v6.json"), "120v5": (P120V5, "spinner_120mm_v5", "report_v5.json"), "120v4": (P120V4, "spinner_120mm_v4", "report_v4.json"), "80v3": (P80V3, "spinner_80mm_v3", "report_v3.json"), "80": (P, "spinner_80mm", "report.json"), "80v2": (P80V2, "spinner_80mm_v2", "report_v2.json"), "160": (P160, "spinner_160mm", "report_160.json")}


def profile(p):
    R = p["R_MAX"]
    w = lambda x: p["ARM_W0"] + (p["ARM_W1"] - p["ARM_W0"]) * x / R   # taper
    xb = R + 10  # overshoot, then clip with r=R circle
    xt = R - p["TOOTH_L"]
    arm = [(0, -w(0) / 2), (xb, -w(xb) / 2), (xb, w(xb) / 2 + p["TOOTH_H"]),
           (xt, w(xt) / 2 + p["TOOTH_H"]), (xt, w(xt) / 2), (0, w(0) / 2)]
    a0, a1 = p["POCKET_X0"], p["POCKET_X1"]
    pk = [(a0, -p["POCKET_W0"] / 2), (a1, -p["POCKET_W1"] / 2), (a1, p["POCKET_W1"] / 2), (a0, p["POCKET_W0"] / 2)]
    if p.get("NECK_W1", 0) > 0:   # tapered neck (same polygon as the arm) out to a counterweight disc clear of the hub
        cx, cy = p["DISC_CX"], p["DISC_CY"]
        arm = [(cx, cy - p["NECK_W1"] / 2)] + arm + [(cx, cy + p["NECK_W1"] / 2)]
    sk = (cq.Sketch()
          .polygon(arm + [arm[0]])
          .push([(0, 0)]).circle(R, mode="i")          # clip tip to r = R_MAX
          .push([(0, 0)]).circle(p["HUB_R"], mode="a")
          .reset())
    if p["DISC_R"] > 0:
        sk = sk.push([(p["DISC_CX"], p["DISC_CY"])]).circle(p["DISC_R"], mode="a").reset()
    if p.get("LOBE_W", 0) > 0:   # v3: short, wide rectangular counterweight lobe spanning x = -BACK .. LOBE_X1
        x0, x1 = -p["BACK"], p["LOBE_X1"]
        sk = sk.push([((x0 + x1) / 2, p["LOBE_CY"])]).rect(x1 - x0, p["LOBE_W"], mode="a").reset()
    sk = (sk
          .polygon(pk + [pk[0]], mode="s")
          .clean()
          .reset().vertices().fillet(p["FILLET"]))
    return sk


def hole_pts(p):
    if p.get("HOLE_PATTERN", "rect") == "plus":
        return [(p["HOLE_DX"] / 2, 0.0), (-p["HOLE_DX"] / 2, 0.0), (0.0, p["HOLE_DY"] / 2), (0.0, -p["HOLE_DY"] / 2)]
    return [(sx * p["HOLE_DX"] / 2, sy * p["HOLE_DY"] / 2) for sx in (1, -1) for sy in (1, -1)]


def hole_walls(p):
    """Min distance (mm) from each tapping hole edge to the outer profile / bore / pocket edges."""
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    face = cq.Workplane("XY").placeSketch(profile(p)).val().Faces()[0] if False else None
    prof = cq.Workplane("XY").placeSketch(profile(p)).extrude(p["T"] / 2, both=True).val()
    out = {}
    for (x, y) in hole_pts(p):
        v = cq.Vertex.makeVertex(x, y, 0)
        dmin = 1e9
        for f in prof.Faces():
            if abs(f.normalAt().z) > 0.9: continue          # skip top/bottom faces
            d = BRepExtrema_DistShapeShape(v.wrapped, f.wrapped); d.Perform(); dmin = min(dmin, d.Value())
        others = [math.hypot(x - a, y - b) - p["HOLE_D"] for (a, b) in hole_pts(p) if (a, b) != (x, y)]
        out[f"({x:+.1f},{y:+.1f})"] = dict(to_outer_or_pocket_edge=dmin - p["HOLE_D"] / 2,
                                          to_bore=math.hypot(x, y) - p["HOLE_D"] / 2 - p["BORE"] / 2,
                                          to_nearest_other_hole=min(others))
        if p.get("PEG_L", 0) > 0:   # peg OD and its root fillet (fillet foot at r = PEG_D/2 + PEG_FILLET on the -Z face)
            out[f"({x:+.1f},{y:+.1f})"].update(
                to_peg_od=math.hypot(x, y) - p["HOLE_D"] / 2 - p["PEG_D"] / 2,
                to_peg_fillet_foot=math.hypot(x, y) - p["HOLE_D"] / 2 - p["PEG_D"] / 2 - p.get("PEG_FILLET", 0))
        if p.get("CBORE_D", 0) > 0:
            k = f"({x:+.1f},{y:+.1f})"; rc = p["CBORE_D"] / 2
            out[k].update(cbore_to_bore=math.hypot(x, y) - rc - p["BORE"] / 2,
                          cbore_to_nearest_other_cbore=min(math.hypot(x - a, y - b) for (a, b) in hole_pts(p) if (a, b) != (x, y)) - 2 * rc,
                          cbore_to_outer_or_pocket_edge=dmin - rc,
                          cbore_floor_thickness=p["T"] - p["CBORE_DEPTH"])
    return out


def build(p=P):
    s = cq.Workplane("XY").placeSketch(profile(p)).extrude(p["T"] / 2, both=True)
    PL = p.get("PEG_L", 0.0); RD = p.get("RECESS_DEPTH", 0.0)
    if RD > 0:   # v9: motor-bell recess on the -Z face
        if p.get("RECESS_FACE", "-Z") == "+Z":
            s = s.cut(cq.Workplane("XY").workplane(offset=p["T"] / 2 - RD).circle(p["RECESS_D"] / 2).extrude(RD + 1))
        else:
            s = s.cut(cq.Workplane("XY").workplane(offset=-p["T"] / 2 - 1).circle(p["RECESS_D"] / 2).extrude(RD + 1))
    zf = -p["T"] / 2 + (RD if p.get("RECESS_FACE", "-Z") == "-Z" else 0.0)   # motor-side contact face (recess floor)
    if PL > 0:   # axial peg/boss on the -Z (motor) face, fused to the plate
        peg = cq.Workplane("XY").workplane(offset=zf).circle(p["PEG_D"] / 2).extrude(-PL)
        s = s.union(peg)
        if p.get("PEG_FILLET", 0) > 0:
            s = s.edges(cq.selectors.BoxSelector((-p["PEG_D"], -p["PEG_D"], zf - 0.01),
                                                 (p["PEG_D"], p["PEG_D"], zf + 0.01))).fillet(p["PEG_FILLET"])
    s = s.cut(cq.Workplane("XY").workplane(offset=-p["T"] / 2 - PL - 1).circle(p["BORE"] / 2).extrude(p["T"] + PL + 2))
    s = s.cut(cq.Workplane("XY").pushPoints(hole_pts(p)).circle(p["HOLE_D"] / 2).extrude(p["T"], both=True))
    if p.get("CBORE_D", 0) > 0 and p.get("CBORE_FACE", "+Z") == "-Z":   # v8: c'bores from the -Z (peg) face
        z0 = zf + p["CBORE_DEPTH"]                                    # c'bore floor
        zb = zf - (PL + 1 if p.get("CBORE_THRU_PEG") else 1)          # tool runs out through the peg length
        s = s.cut(cq.Workplane("XY").workplane(offset=z0).pushPoints(hole_pts(p))
                  .circle(p["CBORE_D"] / 2).extrude(zb - z0))
    elif p.get("CBORE_D", 0) > 0:   # counterbores from the +Z (top) face, opposite the peg
        s = s.cut(cq.Workplane("XY").workplane(offset=p["T"] / 2 - p["CBORE_DEPTH"]).pushPoints(hole_pts(p))
                  .circle(p["CBORE_D"] / 2).extrude(p["CBORE_DEPTH"] + 1))
    return s


def massprops(shape, rho):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape.wrapped, g)
    c = g.CentreOfMass(); M = g.MatrixOfInertia()   # about CG, volume-weighted (mm^5)
    return g.Mass(), (c.X(), c.Y(), c.Z()), [[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]


def cg(p):
    v, c, _ = massprops(build(p).val(), 1)
    return c


def solve(p=P, tol=1e-5):
    """Newton on (DISC_CX, DISC_CY) -> CG (x, y) = 0 using the OCC kernel's centre of mass."""
    p = dict(p)
    for it in range(20):
        c = cg(p)
        k1, k2 = p.get("SOLVE_KEYS", ("DISC_CX", "DISC_CY"))
        print(f"iter {it}: {k1}={p[k1]:.5f} {k2}={p[k2]:.5f}  CG=({c[0]:+.6f},{c[1]:+.6f}) mm")
        if math.hypot(c[0], c[1]) < tol:
            break
        h = 0.01; J = []
        for k in p.get("SOLVE_KEYS", ("DISC_CX", "DISC_CY")):
            q = dict(p); q[k] += h; cc = cg(q); J.append(((cc[0] - c[0]) / h, (cc[1] - c[1]) / h))
        a, b, cc_, d = J[0][0], J[1][0], J[0][1], J[1][1]   # dCGx/dX, dCGx/dY, dCGy/dX, dCGy/dY
        det = a * d - b * cc_
        k1, k2 = p.get("SOLVE_KEYS", ("DISC_CX", "DISC_CY"))
        p[k1] -= (d * c[0] - b * c[1]) / det
        p[k2] -= (-cc_ * c[0] + a * c[1]) / det
    return p


def analyse(p):
    s = build(p); sh = s.val(); rho = p["RHO"]
    V, c, I = massprops(sh, rho)
    m = V * rho                                            # g
    # inertia about spin axis (through origin) via parallel axis from CG
    Izz_cg = I[2][2] * rho                                 # g*mm^2
    Izz = Izz_cg + m * (c[0] ** 2 + c[1] ** 2)
    BB = sh.BoundingBox()
    import numpy as np
    vs = np.array([[q.x, q.y, q.z] for q in sh.tessellate(0.001, 0.05)[0]])
    rmax = float(np.max(np.hypot(vs[:, 0], vs[:, 1])))
    # exact rmax: max over vertices + sample edges densely
    rr = 0
    for e in sh.Edges():
        for q in e.positions([i / 400 for i in range(401)]):
            rr = max(rr, math.hypot(q.x, q.y))
    big = cq.Workplane("XY").box(400, 400, 50)
    halves = {}
    for name, ctr in (("striking_side_x>0", 100), ("counterweight_side_x<0", -100)):
        h = s.intersect(cq.Workplane("XY").center(ctr, 0).rect(200, 400).extrude(25, both=True)).val()
        hv, hc, _ = massprops(h, rho)
        halves[name] = dict(mass_g=hv * rho, cg_x_mm=hc[0], cg_y_mm=hc[1], cg_radius_mm=math.hypot(hc[0], hc[1]),
                            moment_g_mm=hv * rho * math.hypot(hc[0], hc[1]))
    # motor
    KV = 1000; perf = {}
    for S in (2, 3, 4):
        Vn = 3.7 * S; rpm = 0.8 * KV * Vn; w = rpm * 2 * math.pi / 60
        I_si = Izz * 1e-9
        perf[f"{S}S"] = dict(pack_V_nominal=Vn, rpm_80pct=rpm, tip_speed_m_s=w * p["R_MAX"] / 1000,
                             energy_J=0.5 * I_si * w ** 2,
                             bite_1tooth_mm=2000 * 60 / rpm, bite_2teeth_mm=2000 * 60 / rpm / 2,
                             rpm_80pct_fullcharge=0.8 * KV * 4.2 * S)
    w10k = 10000 * 2 * math.pi / 60
    e_res = m * math.hypot(c[0], c[1])   # g*mm
    hw = hole_walls(p)
    min_wall = min(min(d.values()) for d in hw.values())
    out = dict(params=p, hole_walls_mm=hw, cg_z_mm=c[2], min_hole_wall_mm=min_wall, hole_walls_ok_3mm=min_wall >= 3.0,
               overall_length_tip_to_back_mm=BB.xmax - BB.xmin, overall_width_mm=BB.ymax - BB.ymin,
               strike_reach_mm=BB.xmax, back_reach_mm=-BB.xmin, strike_to_back_ratio=BB.xmax / -BB.xmin,
               hole_centres_mm=hole_pts(p), mass_g=m, volume_mm3=V, cg_mm=c, cg_offset_mm=math.hypot(c[0], c[1]),
               residual_unbalance_g_mm=e_res, Izz_axis_kg_m2=Izz * 1e-9,
               products_of_inertia_g_mm2=dict(Ixz=I[0][2] * rho, Iyz=I[1][2] * rho, Ixy=I[0][1] * rho),
               Ixx_Iyy_Izz_cg_kg_m2=[I[0][0] * rho * 1e-9, I[1][1] * rho * 1e-9, Izz_cg * 1e-9],
               r_max_mm=rr, bbox=[BB.xmin, BB.xmax, BB.ymin, BB.ymax, BB.zmin, BB.zmax],
               halves=halves, perf=perf,
               unbalance_force_10k_rpm_for_0p1_g_mm_N=0.1e-6 * w10k ** 2,
               unbalance_force_10k_rpm_actual_residual_N=e_res * 1e-6 * w10k ** 2)
    if p.get("PEG_L", 0) > 0 and p.get("CBORE_D", 0) > 0 and p.get("CBORE_FACE", "+Z") == "-Z":
        out["peg_notch"] = peg_notch(p, sh)
    return s, out


def peg_notch(p, sh):
    """Peg cross-section and walls where the -Z c'bores cut into the peg."""
    rp, rb, rc = p["PEG_D"] / 2, p["BORE"] / 2, p["CBORE_D"] / 2
    rh = min(math.hypot(x, y) for x, y in hole_pts(p))
    zmid = -p["T"] / 2 + p.get("RECESS_DEPTH", 0) - p["PEG_L"] / 2
    sec = cq.Workplane("XY").add(sh).section(zmid).val()
    a_sec = sum(f.Area() for f in sec.Faces())
    a_full = math.pi * (rp ** 2 - rb ** 2)
    notch = rp - (rh - rc)
    return dict(cbore_inner_radius_mm=rh - rc, peg_radius_mm=rp, notch_depth_radial_mm=notch,
                notch_chord_width_mm=2 * math.sqrt(max(0, rp ** 2 - ((rp ** 2 - rc ** 2 + rh ** 2) / (2 * rh)) ** 2)) if notch > 0 else 0.0,
                notch_runs_full_peg_length=bool(p.get("CBORE_THRU_PEG")),
                min_wall_cbore_to_spindle_hole_mm=rh - rc - rb, nominal_peg_wall_mm=rp - rb,
                peg_section_area_unnotched_mm2=a_full, peg_section_area_mm2=a_sec,
                peg_section_area_lost_pct=100 * (1 - a_sec / a_full), section_z_mm=zmid,
                weak_lt_0p5mm=(rh - rc - rb) < 0.5)


if __name__ == "__main__":
    import sys
    P0, stem, rep = VARIANTS[sys.argv[1] if len(sys.argv) > 1 else "80"]
    p = solve(P0)
    s, out = analyse(p)
    if p.get("COLOR"):   # coloured STEP via an Assembly (XCAF colour on the part)
        h = p["COLOR"].lstrip("#"); rgb = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
        assy = cq.Assembly(name="spinner")
        assy.add(s, name=stem, color=cq.Color(*rgb, 1.0))
        assy.save(stem + ".step", exportType="STEP")
    else:
        cq.exporters.export(s, stem + ".step")
    cq.exporters.export(s, stem + ".stl", tolerance=0.01, angularTolerance=0.05)
    json.dump(out, open(rep, "w"), indent=2)
    print(json.dumps(out, indent=2))
