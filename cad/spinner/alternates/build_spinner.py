"""Two-tooth 'twin-hammer' rim-weighted spinner for Exterminate. Offline CadQuery build.
Spin axis Z, CCW seen from +Z (same convention as spinner_120mm_v8). Motor interface copied from v8:
Ø3.8 bore, 4x Ø2.4 M2 clearance holes in plus pattern ±5.75 mm, Ø4.4x2.2 c'bores on the -Z (motor) face."""
import json, math, sys
import cadquery as cq, ezdxf
from concepts import *
import concepts
T=12.0; concepts.T=T   # 2026-11-01: plate thickness 8 -> 12 mm
POCKET_D=28.2; POCKET_DEP=2.0  # -Z motor-bell recess: Motor CAN Ø27.7 (asm_cam5.step) +0.5
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

BASE=dict(R_BODY=108,R_TIP=120,A_TOOTHBACK=5,A_TIP=28,RAKE_OFF=4.0,TRAIL_H=6,WEB_W=14,A_TRAIL=-75,A_MID=-35,R_IN=94)
def tune_350():
    lo,hi=-75,-20
    for _ in range(40):
        a=(lo+hi)/2; m=props(concept_B(dict(BASE,R_IN=96,A_TRAIL=a,A_MID=min(a+40,-5))))["mass_g"]
        if m>350: lo=a
        else: hi=a
    return dict(BASE,R_IN=96,A_TRAIL=a,A_MID=min(a+40,-5))
VARS={"twinhammer_500":BASE,"twinhammer_350":None}

def solid_from(g):
    g=g.simplify(0.005,preserve_topology=True)
    def wire(c): 
        pts=list(c.coords)[:-1]; return cq.Wire.makePolygon([cq.Vector(x,y,0) for x,y in pts],close=True)
    face=cq.Face.makeFromWires(wire(g.exterior),[wire(i) for i in g.interiors])
    s=cq.Workplane("XY").add(cq.Solid.extrudeLinear(face,cq.Vector(0,0,T))).translate((0,0,-T/2))
    pts=[(BOLT_R,0),(-BOLT_R,0),(0,BOLT_R),(0,-BOLT_R)]
    s=s.cut(cq.Workplane("XY").workplane(offset=-T/2-1).circle(BORE/2).extrude(T+2))
    s=s.cut(cq.Workplane("XY").workplane(offset=-T/2-1).pushPoints(pts).circle(HOLE_D/2).extrude(T+2))
    s=s.cut(cq.Workplane("XY").workplane(offset=-T/2-1).circle(POCKET_D/2).extrude(POCKET_DEP+1))
    s=s.cut(cq.Workplane("XY").workplane(offset=-T/2+POCKET_DEP-1).pushPoints(pts).circle(CB_D/2).extrude(CB_DEP+1))  # c'bores from pocket floor
    return s,g

def mp(sh):
    g=GProp_GProps(); BRepGProp.VolumeProperties_s(sh.wrapped,g); c=g.CentreOfMass(); M=g.MatrixOfInertia()
    V=g.Mass(); Izz=(M.Value(3,3)+V*(c.X()**2+c.Y()**2))*RHO*1e-9
    return V*RHO,(c.X(),c.Y(),c.Z()),Izz

def perf(I,R):
    out={}
    for S in (2,3,4):
        rpm=0.8*1000*3.7*S; w=rpm*2*math.pi/60
        out[f"{S}S"]=dict(rpm_80pct_KV1000=rpm,tip_speed_m_s=w*R/1000,energy_J=0.5*I*w*w,bite_2teeth_mm_at_2m_s=2000*60/rpm/2)
    return out

def dxf(g,fn):
    d=ezdxf.new("R2010",setup=True); d.units=ezdxf.units.MM; m=d.modelspace()
    d.layers.add("PROFILE",color=7); d.layers.add("HOLES_THRU",color=1); d.layers.add("CBORE_4.4x2.2_MOTOR_SIDE",color=3)
    for c in [g.exterior]+list(g.interiors): m.add_lwpolyline(list(c.coords),close=True,dxfattribs={"layer":"PROFILE"})
    m.add_circle((0,0),BORE/2,dxfattribs={"layer":"HOLES_THRU"})
    for p in [(BOLT_R,0),(-BOLT_R,0),(0,BOLT_R),(0,-BOLT_R)]:
        m.add_circle(p,HOLE_D/2,dxfattribs={"layer":"HOLES_THRU"}); m.add_circle(p,CB_D/2,dxfattribs={"layer":"CBORE_4.4x2.2_MOTOR_SIDE"})
    d.saveas(fn)

def rake_deg(P):
    Ri,Ro,at=P["R_IN"],P["R_TIP"],math.radians(P["A_TIP"]); ar=at-P["RAKE_OFF"]/Ri
    tip=(Ro*math.cos(at),Ro*math.sin(at)); root=(Ri*math.cos(ar),Ri*math.sin(ar))
    f=(tip[0]-root[0],tip[1]-root[1]); rad=(math.cos(at),math.sin(at))
    ang=math.degrees(math.acos((f[0]*rad[0]+f[1]*rad[1])/math.hypot(*f)))
    return ang  # tip leads root -> positive rake

if __name__=="__main__":
    VARS["twinhammer_350"]=json.load(open("report_8mm_backup.json"))["twinhammer_350"]["params"]  # keep 8mm geometry, do not retune
    rep={}
    for name,P in VARS.items():
        g=concept_B(P); s,g2=solid_from(g); sh=s.val()
        m,c,I=mp(sh)
        cq.exporters.export(s,f"{name}_12mm_v2.step"); cq.exporters.export(s,f"{name}_12mm_v2.stl",tolerance=0.02,angularTolerance=0.1)
        dxf(g2,f"{name}_12mm_v2_profile.dxf")
        bb=sh.BoundingBox()
        rep[name]=dict(params=P,thickness_mm=T,pocket_d_mm=POCKET_D,pocket_depth_mm=POCKET_DEP,floor_under_pocket_mm=T-POCKET_DEP,mass_g=m,Izz_kg_m2=I,r_gyration_mm=math.sqrt(I/(m*1e-3))*1e3,
            cg_mm=c,cg_radial_offset_mm=math.hypot(c[0],c[1]),bbox_mm=[bb.xmin,bb.xmax,bb.ymin,bb.ymax,bb.zmin,bb.zmax],
            tip_radius_mm=max(math.hypot(x,y) for x,y in g2.exterior.coords),rake_deg=rake_deg(P),
            tooth_height_above_body_mm=P["R_TIP"]-P["R_BODY"],hammer_depth_mm=P["R_BODY"]-P["R_IN"],
            min_feature_ge_5mm=min_width_ok(g2,5.0),min_feature_ge_6mm=min_width_ok(g2,6.0),
            internal_fillet_mm=RI_FIL,external_edge_round_mm=RE_FIL,perf=perf(I,P["R_TIP"]))
    cur=json.load(open("/workspace/spinner/report_v8.json"))
    rep["current_v8"]=dict(mass_g=cur["mass_g"],Izz_kg_m2=cur["Izz_axis_kg_m2"],perf=perf(cur["Izz_axis_kg_m2"],120))
    json.dump(rep,open("report_12mm_v2.json","w"),indent=1,default=float)
    for k,v in rep.items(): print(k,round(v["mass_g"],1),v["Izz_kg_m2"],{s:round(x["energy_J"]) for s,x in v["perf"].items()},v.get("cg_radial_offset_mm"),v.get("rake_deg"),v.get("min_feature_ge_5mm"),v.get("tip_radius_mm"))
