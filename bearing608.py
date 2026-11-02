import cadquery as cq
W=7.0
outer = cq.Workplane("XY").circle(11).circle(9.5).extrude(W).faces(">Z or <Z").edges().chamfer(0.3)
inner = cq.Workplane("XY").circle(6.0).circle(4.0).extrude(W).faces(">Z or <Z").edges().chamfer(0.3)
shield = cq.Workplane("XY").workplane(offset=0.4).circle(9.6).circle(5.9).extrude(W-0.8)
b = outer.union(inner).union(shield)
cq.exporters.export(b, "Bearing_608_22x8x7.step")
bb=b.val().BoundingBox(); print(b.val().Volume(), bb.xlen, bb.ylen, bb.zlen, len(b.solids().vals()))
