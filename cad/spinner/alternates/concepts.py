import math, numpy as np
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union
from shapely import affinity
RHO=7.85e-3; T=8.0
RI_FIL=4.0; RE_FIL=1.0
HUB_R=14.0; BORE=3.8; HOLE_D=2.4; BOLT_R=5.75; CB_D=4.4; CB_DEP=2.2

def polar(pts):  # [(deg, r)]
    return [(r*math.cos(math.radians(a)), r*math.sin(math.radians(a))) for a,r in pts]
def lin(a0,r0,a1,r1,n=None):
    n=n or max(2,int(abs(a1-a0)/0.25)+1)
    return [(a0+(a1-a0)*i/(n-1), r0+(r1-r0)*i/(n-1)) for i in range(n)]

def hammer(P):
    """one rim hammer around +X. CCW spin -> tooth at leading (+angle) end."""
    Ri,Rb,Ro=P["R_IN"],P["R_BODY"],P["R_TIP"]
    a_tr,a_m,a_tb,a_tip=P["A_TRAIL"],P["A_MID"],P["A_TOOTHBACK"],P["A_TIP"]
    face_root=a_tip - math.degrees(P["RAKE_OFF"]/Ri)   # root of face is BEHIND tip -> positive rake
    outer = lin(a_tr,Ri+P["TRAIL_H"],a_m,Rb)+lin(a_m,Rb,a_tb,Rb)[1:]+lin(a_tb,Rb,a_tip,Ro)[1:]
    inner = lin(face_root,Ri,a_tr,Ri)
    return Polygon(polar(outer+inner))

def finish(g):
    g=g.buffer(RI_FIL,resolution=64).buffer(-RI_FIL,resolution=64)   # round concave
    g=g.buffer(-RE_FIL,resolution=64).buffer(RE_FIL,resolution=64)   # round convex
    return g

def concept_B(P):  # two-hammer "crescent" bar
    h=hammer(P)
    web=box(0,-P["WEB_W"]/2,P["R_IN"]+5,P["WEB_W"]/2)
    half=unary_union([h,web])
    g=unary_union([half,affinity.rotate(half,180,origin=(0,0)),Point(0,0).buffer(HUB_R,resolution=64)])
    return finish(g)

def concept_A(P):  # full-ring egg-beater disc, 2 spokes, 2 teeth
    Ri,Rb=P["R_BODY"]-P["RING_W"],P["R_BODY"]
    ring=Point(0,0).buffer(Rb,resolution=720).difference(Point(0,0).buffer(Ri,resolution=720))
    tooth=hammer(dict(P,R_IN=Ri,A_TRAIL=P["A_TB"]-1,A_MID=P["A_TB"],A_TOOTHBACK=P["A_TB"],TRAIL_H=Rb-Ri))
    # notch ahead of the tooth for bite: cut ring rim down ahead of each tooth
    notch=Polygon(polar(lin(P["A_TIP"],Rb+1,P["A_TIP"]+P["NOTCH_DEG"],Rb+1)+lin(P["A_TIP"]+P["NOTCH_DEG"],Rb-P["NOTCH_D"],P["A_TIP"],Rb-P["NOTCH_D"])))
    spoke=box(-Ri-3,-P["WEB_W"]/2,Ri+3,P["WEB_W"]/2)
    g=unary_union([ring,tooth,affinity.rotate(tooth,180,origin=(0,0)),spoke,Point(0,0).buffer(HUB_R,resolution=64)])
    g=g.difference(notch).difference(affinity.rotate(notch,180,origin=(0,0)))
    return finish(g)

def concept_C(P):  # tapered two-tooth bar
    L=P["R_TIP"]; w0,w1=P["W0"],P["W1"]; tl=P["TOOTH_L"]; th=P["TOOTH_H"]
    half=Polygon([(0,-w0/2),(L+5,-w1/2),(L+5,w1/2+th),(L-tl,w1/2+th),(L-tl,w1/2),(0,w0/2)])
    half=half.intersection(Point(0,0).buffer(L,resolution=720))
    g=unary_union([half,affinity.rotate(half,180,origin=(0,0)),Point(0,0).buffer(HUB_R,resolution=64)])
    return finish(g)

def props(g):
    """area, centroid, polar 2nd moment about origin (mm^4) via Green on all rings; holes subtracted analytically."""
    def ring_J(c):
        x=np.array(c.xy[0]); y=np.array(c.xy[1]); x1=np.roll(x,-1); y1=np.roll(y,-1)
        cr=x*y1-x1*y
        A=cr.sum()/2; Ix=((y*y+y*y1+y1*y1)*cr).sum()/12; Iy=((x*x+x*x1+x1*x1)*cr).sum()/12
        Cx=((x+x1)*cr).sum()/6; Cy=((y+y1)*cr).sum()/6
        return A,Cx,Cy,Ix+Iy
    A=Cx=Cy=J=0
    for p in getattr(g,"geoms",[g]):
        for k,c in enumerate([p.exterior]+list(p.interiors)):
            a,cx,cy,j=ring_J(c); s=1 if k==0 else -1
            A+=s*abs(a); Cx+=s*math.copysign(1,a)*cx*(1 if k==0 else -1)*(1 if a>0 else 1); Cy+=0; J+=s*abs(j)
    c=g.centroid
    # holes: bore + 4 bolt holes (thru) + 4 cbores (depth CB_DEP)
    Vh=0; Jh=0
    for d,r,depth,n in ((BORE,0,T,1),(HOLE_D,BOLT_R,T-CB_DEP,4),(CB_D,BOLT_R,CB_DEP,4)):
        a=math.pi*d*d/4; Vh+=n*a*depth; Jh+=n*depth*(a*d*d/8+a*r*r)
    V=A*T-Vh; Jz=J*T-Jh
    m=V*RHO; I=Jz*RHO*1e-9  # g ; kg m^2
    return dict(area_mm2=A,mass_g=m,Izz_kg_m2=I,centroid_mm=(c.x,c.y),r_gyr_mm=math.sqrt(I/(m*1e-3))*1e3)

def min_width_ok(g,w=5.0):
    """True if eroding by w/2 keeps one connected piece (no neck thinner than w)."""
    e=g.buffer(-w/2)
    return e.geom_type=="Polygon"
