from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def make_base():
    return (
        cq.Workplane("XY")
        .box(0.20, 0.16, 0.18, centered=(True, True, False))
        .edges("|Z").fillet(0.04)
        .faces("+Z").shell(-0.005)
    )

def make_lid():
    return (
        cq.Workplane("XY")
        .box(0.20, 0.16, 0.08, centered=(True, True, False))
        .edges("|Z").fillet(0.04)
        .faces(">Z").workplane().center(0, 0)
        .circle(0.035).extrude(0.10)
        .faces(">Z or <Z").shell(-0.005)
    )

def make_pulp_bin():
    return (
        cq.Workplane("XY")
        .box(0.18, 0.12, 0.17, centered=(True, True, False))
        .edges("|Z").fillet(0.04)
        .faces("+Z").shell(-0.005)
    )

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base(), "base_mesh"),
        origin=Origin(xyz=(0, -0.02, 0)),
        name="base_body",
        color=(0.9, 0.9, 0.9)
    )
    base.visual(
        Cylinder(radius=0.01, height=0.04),
        origin=Origin(xyz=(0, -0.11, 0.05), rpy=(-0.5, 0, 0)),
        name="spout",
        color=(0.8, 0.8, 0.8)
    )

    pulp_bin = model.part("pulp_bin")
    pulp_bin.visual(
        mesh_from_cadquery(make_pulp_bin(), "bin_mesh"),
        origin=Origin(xyz=(0, 0, 0)),
        name="bin_body",
        color=(0.2, 0.2, 0.2)
    )
    model.articulation(
        "bin_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=pulp_bin,
        origin=Origin(xyz=(0, 0.12, 0)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=0.0, upper=0.15)
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(make_lid(), "lid_mesh"),
        origin=Origin(xyz=(0, -0.08, 0)),
        name="lid_body",
        color=(0.8, 0.9, 1.0, 0.4)
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0, 0.06, 0.18)),
        axis=(-1, 0, 0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.57)
    )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.07, height=0.06),
        origin=Origin(xyz=(0, 0, 0)),
        name="basket_mesh",
        color=(0.8, 0.8, 0.8)
    )
    basket.visual(
        Cylinder(radius=0.01, height=0.115),
        origin=Origin(xyz=(0, 0, -0.0875)),
        name="drive_shaft",
        color=(0.7, 0.7, 0.7)
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0, -0.02, 0.15)),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0)
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.033, height=0.12),
        origin=Origin(xyz=(0, 0, -0.06)),
        name="pusher_body",
        color=(0.1, 0.1, 0.1)
    )
    pusher.visual(
        Cylinder(radius=0.04, height=0.01),
        origin=Origin(xyz=(0, 0, 0.005)),
        name="pusher_cap",
        color=(0.2, 0.2, 0.2)
    )
    model.articulation(
        "pusher_slide",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0, -0.08, 0.22)),
        axis=(0, 0, -1),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.10)
    )

    dial = model.part("speed_dial")
    dial.visual(
        Cylinder(radius=0.02, height=0.015),
        origin=Origin(xyz=(0, -0.0075, 0), rpy=(1.5708, 0, 0)),
        name="dial_knob",
        color=(0.2, 0.2, 0.2)
    )
    model.articulation(
        "dial_turn",
        ArticulationType.REVOLUTE,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.05, -0.10, 0.09)),
        axis=(0, -1, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-1.57, upper=1.57)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    pulp_bin = object_model.get_part("pulp_bin")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    dial = object_model.get_part("speed_dial")
    
    lid_hinge = object_model.get_articulation("lid_hinge")
    pusher_slide = object_model.get_articulation("pusher_slide")
    bin_slide = object_model.get_articulation("bin_slide")
    
    ctx.allow_overlap(base, basket, reason="Basket sits inside the base cavity.")
    ctx.allow_overlap(lid, basket, reason="Lid covers the basket.")
    ctx.allow_overlap(lid, pusher, reason="Pusher slides inside the chute.")
    ctx.allow_overlap(base, pulp_bin, reason="Pulp bin touches the base.")
    ctx.allow_overlap(lid, pulp_bin, reason="Lid touches the pulp bin.")
    ctx.allow_overlap(base, dial, reason="Dial is mounted on the base.")
    
    ctx.expect_within(pusher, lid, axes="xy", inner_elem="pusher_body", outer_elem="lid_body", margin=0.01)
    
    with ctx.pose({bin_slide: 0.15}):
        ctx.expect_gap(pulp_bin, base, axis="y", min_gap=0.01)
        
    with ctx.pose({pusher_slide: 0.10}):
        ctx.expect_overlap(pusher, lid, axes="z", elem_a="pusher_body", elem_b="lid_body", min_overlap=0.05)

    return ctx.report()

object_model = build_object_model()
