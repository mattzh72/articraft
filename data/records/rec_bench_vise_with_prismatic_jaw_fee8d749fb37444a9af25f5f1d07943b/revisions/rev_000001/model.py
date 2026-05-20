from __future__ import annotations

import cadquery as cq

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

SWIVEL_GAP = 0.0
JAW_TRAVEL = 0.075
SWIVEL_LIMIT = 1.10


def _swivel_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(0.245, 0.155, 0.014).translate((0.0, 0.0, 0.007))

    top_ring = cq.Workplane("XY").circle(0.078).extrude(0.009).translate((0.0, 0.0, 0.013))
    center_recess = cq.Workplane("XY").circle(0.048).extrude(0.004).translate((0.0, 0.0, 0.018))

    slot_left = (
        cq.Workplane("XY").box(0.044, 0.010, 0.022).translate((0.072, 0.0, 0.007))
    )
    slot_left = slot_left.union(cq.Workplane("XY").cylinder(0.022, 0.005).translate((0.094, 0.0, 0.007)))
    slot_left = slot_left.union(cq.Workplane("XY").cylinder(0.022, 0.005).translate((0.050, 0.0, 0.007)))

    return base.union(top_ring).cut(center_recess).cut(slot_left).cut(slot_left.translate((-0.144, 0.0, 0.0)))


def _upper_body_shape() -> cq.Workplane:
    swivel_plate = cq.Workplane("XY").circle(0.073).extrude(0.010)
    pedestal = cq.Workplane("XY").box(0.150, 0.085, 0.022).translate((0.0, 0.0, 0.021))
    bed = cq.Workplane("XY").box(0.190, 0.090, 0.010).translate((0.010, 0.0, 0.037))

    left_rail = cq.Workplane("XY").box(0.170, 0.014, 0.012).translate((0.020, -0.038, 0.048))
    right_rail = cq.Workplane("XY").box(0.170, 0.014, 0.012).translate((0.020, 0.038, 0.048))

    rear_buttress = cq.Workplane("XY").box(0.056, 0.090, 0.028).translate((-0.068, 0.0, 0.056))
    fixed_jaw_block = cq.Workplane("XY").box(0.024, 0.072, 0.040).translate((-0.040, 0.0, 0.062))

    side_support = cq.Workplane("XY").box(0.065, 0.020, 0.020).translate((0.030, 0.046, 0.030))
    screw_housing = cq.Workplane("YZ").circle(0.010).extrude(0.127).translate((-0.005, 0.050, 0.030))
    bearing_boss = cq.Workplane("YZ").circle(0.0095).extrude(0.011).translate((0.122, 0.050, 0.030))

    clearance_slot = cq.Workplane("XY").box(0.140, 0.064, 0.050).translate((0.050, 0.0, 0.067))

    return (
        swivel_plate.union(pedestal)
        .union(bed)
        .union(left_rail)
        .union(right_rail)
        .union(rear_buttress)
        .union(fixed_jaw_block)
        .union(side_support)
        .union(screw_housing)
        .union(bearing_boss)
        .cut(clearance_slot)
    )


def _front_jaw_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(0.110, 0.058, 0.010).translate((0.035, 0.0, 0.005))
    carriage = cq.Workplane("XY").box(0.036, 0.058, 0.030).translate((0.000, 0.0, 0.025))
    jaw_cap = cq.Workplane("XY").box(0.014, 0.062, 0.040).translate((-0.012, 0.0, 0.030))
    nose = cq.Workplane("XY").box(0.018, 0.046, 0.020).translate((0.020, 0.0, 0.022))
    return shoe.union(carriage).union(jaw_cap).union(nose)


def _handwheel_shape() -> cq.Workplane:
    rim = cq.Workplane("XY").circle(0.040).circle(0.032).extrude(0.010)
    hub = cq.Workplane("XY").circle(0.018).circle(0.0105).extrude(0.010)
    spoke = cq.Workplane("XY").rect(0.005, 0.020).extrude(0.010).translate((0.0, 0.025, 0.0))

    wheel = rim.union(hub)
    for angle_deg in (0.0, 120.0, 240.0):
        wheel = wheel.union(spoke.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    spinner_stem = cq.Workplane("XY").circle(0.0032).extrude(0.010).translate((0.0, 0.034, 0.0))
    spinner_knob = cq.Workplane("XY").circle(0.0062).extrude(0.020).translate((0.0, 0.034, 0.006))
    return wheel.union(spinner_stem).union(spinner_knob).translate((0.0, 0.0, -0.005))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="angle_lock_drill_vise")

    cast_blue = model.material("cast_blue", rgba=(0.18, 0.34, 0.54, 1.0))
    base_gray = model.material("base_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    oxide = model.material("oxide", rgba=(0.11, 0.11, 0.12, 1.0))

    swivel_base = model.part("swivel_base")
    swivel_base.visual(
        mesh_from_cadquery(_swivel_base_shape(), "drill_vise_swivel_base"),
        material=base_gray,
        name="base_casting",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        mesh_from_cadquery(_upper_body_shape(), "drill_vise_upper_body"),
        material=cast_blue,
        name="body_casting",
    )
    upper_body.visual(
        Box((0.004, 0.072, 0.026)),
        origin=Origin(xyz=(-0.028, 0.0, 0.055)),
        material=steel,
        name="fixed_jaw_face",
    )
    upper_body.visual(
        Cylinder(radius=0.017, length=0.001),
        origin=Origin(xyz=(0.1225, 0.050, 0.030), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=steel,
        name="wheel_retainer",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        mesh_from_cadquery(_front_jaw_shape(), "drill_vise_front_jaw"),
        material=cast_blue,
        name="jaw_carriage",
    )
    front_jaw.visual(
        Box((0.004, 0.060, 0.026)),
        origin=Origin(xyz=(-0.020, 0.0, 0.023)),
        material=steel,
        name="front_jaw_face",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "drill_vise_handwheel"),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=oxide,
        name="wheel",
    )

    model.articulation(
        "base_to_upper_body",
        ArticulationType.REVOLUTE,
        parent=swivel_base,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, 0.022 + SWIVEL_GAP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-SWIVEL_LIMIT,
            upper=SWIVEL_LIMIT,
        ),
    )
    model.articulation(
        "body_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=upper_body,
        child=front_jaw,
        origin=Origin(xyz=(0.006, 0.0, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.08,
            lower=0.0,
            upper=JAW_TRAVEL,
        ),
    )
    model.articulation(
        "body_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=upper_body,
        child=handwheel,
        origin=Origin(xyz=(0.128, 0.050, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    swivel_base = object_model.get_part("swivel_base")
    upper_body = object_model.get_part("upper_body")
    front_jaw = object_model.get_part("front_jaw")
    handwheel = object_model.get_part("handwheel")

    swivel_joint = object_model.get_articulation("base_to_upper_body")
    jaw_joint = object_model.get_articulation("body_to_front_jaw")
    wheel_joint = object_model.get_articulation("body_to_handwheel")

    ctx.expect_gap(
        upper_body,
        swivel_base,
        axis="z",
        min_gap=0.0,
        max_gap=0.0005,
        name="swivel plate seats onto base ring",
    )

    with ctx.pose({swivel_joint: 0.0, jaw_joint: 0.0}):
        ctx.expect_gap(
            front_jaw,
            upper_body,
            axis="x",
            positive_elem="front_jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.010,
            max_gap=0.022,
            name="closed jaw gap is tight but not touching",
        )
        ctx.expect_overlap(
            front_jaw,
            upper_body,
            axes="yz",
            min_overlap=0.040,
            name="front jaw sits within the guide footprint",
        )

    jaw_limits = jaw_joint.motion_limits
    jaw_rest = ctx.part_world_position(front_jaw)
    with ctx.pose({jaw_joint: JAW_TRAVEL}):
        jaw_open = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            front_jaw,
            upper_body,
            axis="x",
            positive_elem="front_jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.080,
            max_gap=0.100,
            name="opened jaw creates drill-vise workholding space",
        )
        ctx.expect_overlap(
            front_jaw,
            upper_body,
            axes="x",
            min_overlap=0.045,
            name="front jaw remains engaged on the guide at full opening",
        )
    ctx.check(
        "front jaw translates outward",
        jaw_limits is not None
        and jaw_rest is not None
        and jaw_open is not None
        and jaw_open[0] > jaw_rest[0] + 0.06,
        details=f"rest={jaw_rest!r}, open={jaw_open!r}",
    )

    wheel_limits = wheel_joint.motion_limits
    ctx.allow_overlap(
        upper_body,
        front_jaw,
        elem_a="body_casting",
        elem_b="jaw_carriage",
        reason="The sliding jaw carriage is simplified as a retained guide fit inside the vise body envelope.",
    )
    ctx.check(
        "handwheel joint is continuous",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS
        and wheel_limits is not None
        and wheel_limits.lower is None
        and wheel_limits.upper is None,
        details=f"type={wheel_joint.articulation_type!r}, limits={wheel_limits!r}",
    )

    wheel_rest = ctx.part_world_position(handwheel)
    with ctx.pose({swivel_joint: 0.65}):
        wheel_swiveled = ctx.part_world_position(handwheel)
    ctx.check(
        "swivel base yaws the vise body",
        wheel_rest is not None
        and wheel_swiveled is not None
        and abs(wheel_swiveled[1] - wheel_rest[1]) > 0.05,
        details=f"rest={wheel_rest!r}, swiveled={wheel_swiveled!r}",
    )

    return ctx.report()


object_model = build_object_model()
