from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    barrel = model.part("barrel")
    
    # Barrel dimensions
    barrel_r_out = 0.010
    barrel_r_in = 0.0085
    barrel_length = 0.090
    flange_r = 0.020
    flange_t = 0.003
    nozzle_r = 0.002
    nozzle_length = 0.015

    # Center hole
    flange = (
        cq.Workplane("XY")
        .circle(flange_r)
        .circle(barrel_r_in)
        .extrude(flange_t)
    )

    tube = (
        cq.Workplane("XY")
        .circle(barrel_r_out)
        .circle(barrel_r_in)
        .extrude(-barrel_length)
    )
    
    # Cone section
    cone_l = 0.005
    cone_outer = (
        cq.Workplane("XY").workplane(offset=-barrel_length)
        .circle(barrel_r_out)
        .workplane(offset=-cone_l)
        .circle(nozzle_r + 0.001)
        .loft()
    )
    
    cone_inner = (
        cq.Workplane("XY").workplane(offset=-barrel_length)
        .circle(barrel_r_in)
        .workplane(offset=-cone_l)
        .circle(nozzle_r)
        .loft()
    )
    cone = cone_outer.cut(cone_inner)

    nozzle = (
        cq.Workplane("XY").workplane(offset=-(barrel_length + cone_l))
        .circle(nozzle_r + 0.001)
        .circle(nozzle_r)
        .extrude(-nozzle_length)
    )

    barrel_cq = flange.union(tube).union(cone).union(nozzle)

    barrel.visual(
        mesh_from_cadquery(barrel_cq, "barrel_geom"),
        origin=Origin(),
        name="barrel_shell"
    )

    plunger = model.part("plunger")

    thumb_t = 0.003
    plunger_length = 0.080
    head_l = 0.006
    head_tip_l = 0.004
    
    thumb_r = 0.015
    head_r = 0.0083

    thumb = (
        cq.Workplane("XY")
        .circle(thumb_r)
        .extrude(-thumb_t)
    )

    # Use a cross shape for the rod to make it look like a real syringe plunger
    cross_w = 0.002
    cross_r = 0.0075
    rod = (
        cq.Workplane("XY").workplane(offset=-thumb_t)
        .rect(cross_r * 2, cross_w)
        .extrude(-plunger_length)
        .union(
            cq.Workplane("XY").workplane(offset=-thumb_t)
            .rect(cross_w, cross_r * 2)
            .extrude(-plunger_length)
        )
    )

    head = (
        cq.Workplane("XY").workplane(offset=-(thumb_t + plunger_length))
        .circle(head_r)
        .extrude(-head_l)
    )

    head_tip = (
        cq.Workplane("XY").workplane(offset=-(thumb_t + plunger_length + head_l))
        .circle(head_r)
        .workplane(offset=-head_tip_l)
        .circle(0.002)
        .loft(combine=True)
    )

    plunger_cq = thumb.union(rod).union(head).union(head_tip)

    plunger.visual(
        mesh_from_cadquery(plunger_cq, "plunger_geom"),
        origin=Origin(),
        name="plunger_shell"
    )

    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.080)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("plunger_slide")

    ctx.allow_overlap(
        barrel,
        plunger,
        reason="The plunger intentionally slides inside the barrel."
    )

    ctx.expect_within(
        plunger,
        barrel,
        axes="xy",
        margin=0.001,
        name="plunger stays centered in the barrel"
    )

    with ctx.pose({slide: 0.080}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="xy",
            margin=0.001,
            name="extended plunger stays centered in the barrel"
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="z",
            min_overlap=0.010,
            name="extended plunger retains insertion in the barrel"
        )

    return ctx.report()

object_model = build_object_model()
