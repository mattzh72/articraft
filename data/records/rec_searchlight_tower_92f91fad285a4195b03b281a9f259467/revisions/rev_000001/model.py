from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="searchlight_tower")

    # -------------------------------------------------------------------------
    # Tower (Root)
    # -------------------------------------------------------------------------
    tower = model.part("tower")

    # Base pad (Z: 0 to 0.2)
    base_pad = cq.Workplane("XY").box(1.5, 1.5, 0.2).translate((0, 0, 0.1))
    
    # Lower flange (Z: 0.2 to 0.25)
    flange = cq.Workplane("XY").cylinder(0.05, 0.4).translate((0, 0, 0.225))
    
    # Mast (Z: 0.25 to 3.05)
    mast = cq.Workplane("XY").cylinder(2.8, 0.2).translate((0, 0, 1.65))
    
    # Platform (Z: 3.05 to 3.1)
    platform = cq.Workplane("XY").cylinder(0.05, 0.6).translate((0, 0, 3.075))
    
    tower_cq = base_pad.union(flange).union(mast).union(platform)

    # Gussets at the base
    for i in range(4):
        gusset_pts = [(0.2, 0.25), (0.4, 0.25), (0.2, 0.5)]
        gusset = cq.Workplane("XZ").polyline(gusset_pts).close().extrude(0.04).translate((0, -0.02, 0))
        gusset = gusset.rotate((0, 0, 0), (0, 0, 1), i * 90)
        tower_cq = tower_cq.union(gusset)
    
    # Control box
    control_box = cq.Workplane("XY").box(0.3, 0.2, 0.4).translate((0, -0.25, 0.8))
    tower_cq = tower_cq.union(control_box)

    # Ladder
    left_rail = cq.Workplane("XY").box(0.02, 0.02, 2.8).translate((-0.15, 0.25, 1.65))
    right_rail = cq.Workplane("XY").box(0.02, 0.02, 2.8).translate((0.15, 0.25, 1.65))
    tower_cq = tower_cq.union(left_rail).union(right_rail)
    for z in [0.4, 0.7, 1.0, 1.3, 1.6, 1.9, 2.2, 2.5, 2.8]:
        rung = cq.Workplane("XY").box(0.3, 0.02, 0.02).translate((0, 0.25, z))
        tower_cq = tower_cq.union(rung)
    for z in [0.6, 1.5, 2.4]:
        standoff = cq.Workplane("XY").box(0.3, 0.05, 0.02).translate((0, 0.225, z))
        tower_cq = tower_cq.union(standoff)
    
    # Railing
    for i in range(8):
        post = cq.Workplane("XY").cylinder(0.4, 0.02).translate((0.55, 0, 3.3)).rotate((0, 0, 0), (0, 0, 1), i * 45)
        tower_cq = tower_cq.union(post)
    
    ring = cq.Workplane("XY").workplane(offset=3.45).circle(0.57).circle(0.53).extrude(0.05)
    tower_cq = tower_cq.union(ring)

    tower.visual(
        mesh_from_cadquery(tower_cq, "tower_mesh"),
        color=(0.25, 0.25, 0.25, 1.0),
        name="tower_visual",
    )

    # -------------------------------------------------------------------------
    # Yoke (Pan)
    # -------------------------------------------------------------------------
    yoke = model.part("yoke")
    
    yoke_base = cq.Workplane("XY").cylinder(0.1, 0.25).translate((0, 0, 0.05))
    arm_base = cq.Workplane("XY").box(0.6, 0.1, 0.05).translate((0, 0, 0.125))
    left_arm = cq.Workplane("XY").box(0.05, 0.1, 0.4).translate((-0.275, 0, 0.35))
    right_arm = cq.Workplane("XY").box(0.05, 0.1, 0.4).translate((0.275, 0, 0.35))
    
    left_boss = cq.Workplane("YZ").cylinder(0.03, 0.04).translate((-0.235, 0, 0.45))
    right_boss = cq.Workplane("YZ").cylinder(0.03, 0.04).translate((0.235, 0, 0.45))
    
    yoke_cq = yoke_base.union(arm_base).union(left_arm).union(right_arm).union(left_boss).union(right_boss)

    yoke.visual(
        mesh_from_cadquery(yoke_cq, "yoke_mesh"),
        color=(0.35, 0.35, 0.35, 1.0),
        name="yoke_visual",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 3.1)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=-3.14, upper=3.14),
    )

    # -------------------------------------------------------------------------
    # Lamp Head (Tilt)
    # -------------------------------------------------------------------------
    lamp_head = model.part("lamp_head")
    
    lamp_body = cq.Workplane("XZ").circle(0.22).circle(0.2).extrude(0.4)
    
    back_cone_outer = cq.Workplane("XZ").workplane(offset=0).circle(0.22).workplane(offset=-0.2).circle(0.1).loft()
    back_cone_inner = cq.Workplane("XZ").workplane(offset=0).circle(0.2).workplane(offset=-0.18).circle(0.08).loft()
    back_cone = back_cone_outer.cut(back_cone_inner)
    
    back_plate = cq.Workplane("XZ").workplane(offset=-0.22).circle(0.1).extrude(0.02)
    
    # Cooling fins on the back cone
    fin_pts = [(0.22, 0), (0.1, -0.2), (0.15, -0.2), (0.27, 0)]
    fin_base = cq.Workplane("XY").polyline(fin_pts).close().extrude(0.01).translate((0, 0, -0.005))
    
    # We need to rotate this fin around the Y axis.
    # In cadquery, rotate takes an axis. The Y axis is (0,0,0) to (0,1,0).
    fins_cq = fin_base
    for i in range(1, 12):
        rotated_fin = fin_base.rotate((0, 0, 0), (0, 1, 0), i * 30)
        fins_cq = fins_cq.union(rotated_fin)
    
    bezel = cq.Workplane("XZ").workplane(offset=0.4).circle(0.25).circle(0.18).extrude(0.05)
    
    lamp_cq = lamp_body.union(back_cone).union(back_plate).union(fins_cq).union(bezel)

    lamp_head.visual(
        mesh_from_cadquery(lamp_cq, "lamp_body_mesh"),
        color=(0.5, 0.5, 0.5, 1.0),
        name="lamp_body_visual",
    )
    
    lens_cq = cq.Workplane("XZ").workplane(offset=0.39).circle(0.2).extrude(0.02)
    lamp_head.visual(
        mesh_from_cadquery(lens_cq, "lens_mesh"),
        color=(0.9, 0.9, 1.0, 0.8),
        name="lens_visual",
    )

    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=1.0, lower=-0.5, upper=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ctx.allow_overlap("tower", "yoke", reason="Yoke base sits flush on the tower platform.")
    ctx.allow_overlap("yoke", "lamp_head", reason="Lamp head bosses rotate inside the yoke arms.")

    ctx.expect_contact("tower", "yoke", name="Yoke sits on tower")
    ctx.expect_contact("yoke", "lamp_head", name="Lamp head connects to yoke")

    with ctx.pose(pan=1.5, tilt=0.5):
        ctx.expect_contact("tower", "yoke", name="Yoke stays on tower during pan")
        ctx.expect_contact("yoke", "lamp_head", name="Lamp head stays connected during tilt")

    return ctx.report()


object_model = build_object_model()