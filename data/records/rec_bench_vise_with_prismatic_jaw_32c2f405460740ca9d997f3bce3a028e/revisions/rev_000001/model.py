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


def _handwheel_shape():
    rim = cq.Workplane("XZ").circle(0.031).circle(0.025).extrude(0.004, both=True)
    hub = cq.Workplane("XZ").circle(0.010).extrude(0.006, both=True)
    spoke = cq.Workplane("XZ").rect(0.006, 0.044).extrude(0.003, both=True)

    wheel = rim.union(hub)
    for angle_deg in (0.0, 60.0, 120.0):
        wheel = wheel.union(spoke.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), angle_deg))

    crank_arm = (
        cq.Workplane("XZ")
        .center(0.018, -0.014)
        .transformed(rotate=(0.0, 0.0, 32.0))
        .rect(0.006, 0.020)
        .extrude(0.003, both=True)
    )
    handle = cq.Workplane("XZ").center(0.022, -0.024).circle(0.0042).extrude(0.007, both=True)
    return wheel.union(crank_arm).union(handle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_vise")

    cast_iron = model.material("cast_iron", rgba=(0.33, 0.35, 0.37, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    oxide_black = model.material("oxide_black", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.118, 0.220, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        Box((0.016, 0.150, 0.010)),
        origin=Origin(xyz=(-0.046, -0.010, 0.021)),
        material=cast_iron,
        name="guide_left",
    )
    base.visual(
        Box((0.016, 0.150, 0.010)),
        origin=Origin(xyz=(0.046, -0.010, 0.021)),
        material=cast_iron,
        name="guide_right",
    )
    base.visual(
        Box((0.104, 0.024, 0.036)),
        origin=Origin(xyz=(0.0, 0.088, 0.033)),
        material=cast_iron,
        name="rear_jaw",
    )
    base.visual(
        Box((0.084, 0.007, 0.022)),
        origin=Origin(xyz=(0.0, 0.0785, 0.027)),
        material=ground_steel,
        name="rear_face",
    )
    base.visual(
        Box((0.050, 0.024, 0.024)),
        origin=Origin(xyz=(0.084, 0.046, 0.028)),
        material=cast_iron,
        name="wheel_support",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(0.102, 0.044, 0.030), rpy=(1.57079632679, 0.0, 0.0)),
        material=ground_steel,
        name="bearing",
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((0.072, 0.076, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, 0.007)),
        material=cast_iron,
        name="carriage",
    )
    jaw.visual(
        Box((0.094, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, 0.024, 0.030)),
        material=cast_iron,
        name="jaw_block",
    )
    jaw.visual(
        Box((0.078, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.033, 0.027)),
        material=ground_steel,
        name="jaw_face",
    )
    jaw.visual(
        Box((0.040, 0.030, 0.022)),
        origin=Origin(xyz=(0.054, -0.022, 0.023)),
        material=cast_iron,
        name="drive_housing",
    )

    handwheel = model.part("handwheel")
    handwheel.visual(
        mesh_from_cadquery(_handwheel_shape(), "handwheel"),
        material=oxide_black,
        name="wheel",
    )
    handwheel.visual(
        Cylinder(radius=0.0075, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=ground_steel,
        name="hub",
    )

    model.articulation(
        "base_to_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=jaw,
        origin=Origin(xyz=(0.0, -0.026, 0.016)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.050, lower=0.0, upper=0.066),
    )
    model.articulation(
        "base_to_handwheel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=handwheel,
        origin=Origin(xyz=(0.102, 0.028, 0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    jaw = object_model.get_part("jaw")
    handwheel = object_model.get_part("handwheel")
    jaw_slide = object_model.get_articulation("base_to_jaw")
    wheel_spin = object_model.get_articulation("base_to_handwheel")

    ctx.expect_gap(
        jaw,
        base,
        axis="z",
        positive_elem="carriage",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="jaw carriage sits on the vise bed",
    )
    ctx.expect_gap(
        base,
        jaw,
        axis="y",
        positive_elem="rear_face",
        negative_elem="jaw_face",
        min_gap=0.055,
        max_gap=0.075,
        name="rest pose leaves an open clamping gap",
    )
    ctx.expect_contact(
        handwheel,
        base,
        elem_a="hub",
        elem_b="bearing",
        name="handwheel hub seats against the bearing boss",
    )

    jaw_rest = ctx.part_world_position(jaw)
    wheel_rest = ctx.part_world_position(handwheel)
    with ctx.pose({jaw_slide: 0.066, wheel_spin: 1.7}):
        ctx.expect_gap(
            base,
            jaw,
            axis="y",
            positive_elem="rear_face",
            negative_elem="jaw_face",
            max_penetration=0.001,
            max_gap=0.004,
            name="closed pose brings the jaws nearly together",
        )
        ctx.expect_overlap(
            jaw,
            base,
            axes="y",
            elem_a="carriage",
            elem_b="base_plate",
            min_overlap=0.060,
            name="moving jaw retains carriage engagement on the base",
        )
        jaw_closed = ctx.part_world_position(jaw)
        wheel_spun = ctx.part_world_position(handwheel)

    ctx.check(
        "jaw travels toward the rear jaw",
        jaw_rest is not None and jaw_closed is not None and jaw_closed[1] > jaw_rest[1] + 0.05,
        details=f"rest={jaw_rest}, closed={jaw_closed}",
    )
    ctx.check(
        "handwheel rotates in place",
        wheel_rest is not None
        and wheel_spun is not None
        and abs(wheel_spun[0] - wheel_rest[0]) < 1e-6
        and abs(wheel_spun[1] - wheel_rest[1]) < 1e-6
        and abs(wheel_spun[2] - wheel_rest[2]) < 1e-6,
        details=f"rest={wheel_rest}, spun={wheel_spun}",
    )

    return ctx.report()


object_model = build_object_model()
