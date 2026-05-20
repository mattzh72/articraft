from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    graphite = model.material("graphite_plastic", rgba=(0.06, 0.065, 0.07, 1.0))
    charcoal = model.material("charcoal_rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    blue = model.material("blue_lacquer", rgba=(0.05, 0.23, 0.78, 1.0))
    silver = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    smoked = model.material("smoked_clear_bin", rgba=(0.62, 0.76, 0.86, 0.42))
    red = model.material("red_release", rgba=(0.9, 0.04, 0.025, 1.0))

    body = model.part("body")
    # The body frame is the fold-joint axis.  The motor, dust cup and handle
    # cluster tightly around that hinge so the folded vacuum reads compact.
    body.visual(
        Cylinder(radius=0.034, length=0.070),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="fold_barrel",
    )
    body.visual(
        Box((0.080, 0.064, 0.075)),
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
        material=blue,
        name="nose_socket",
    )
    body.visual(
        Cylinder(radius=0.102, length=0.168),
        origin=Origin(xyz=(-0.073, 0.000, 0.170), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="motor_pod",
    )
    body.visual(
        Cylinder(radius=0.058, length=0.220),
        origin=Origin(xyz=(0.052, 0.000, 0.190)),
        material=smoked,
        name="dust_bin",
    )
    body.visual(
        Sphere(radius=0.040),
        origin=Origin(xyz=(0.052, 0.000, 0.305)),
        material=blue,
        name="cyclone_cap",
    )
    body.visual(
        Cylinder(radius=0.066, length=0.036),
        origin=Origin(xyz=(-0.165, 0.000, 0.205), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_filter",
    )
    body.visual(
        Box((0.150, 0.046, 0.042)),
        origin=Origin(xyz=(-0.210, 0.000, 0.260)),
        material=charcoal,
        name="handle_top",
    )
    body.visual(
        Box((0.052, 0.046, 0.205)),
        origin=Origin(xyz=(-0.296, 0.000, 0.155)),
        material=charcoal,
        name="handle_grip",
    )
    body.visual(
        Box((0.160, 0.046, 0.040)),
        origin=Origin(xyz=(-0.212, 0.000, 0.047)),
        material=charcoal,
        name="handle_foot",
    )
    body.visual(
        Box((0.048, 0.020, 0.020)),
        origin=Origin(xyz=(-0.286, 0.000, 0.124)),
        material=red,
        name="release_button",
    )
    for i, z in enumerate((0.155, 0.178, 0.201)):
        body.visual(
            Box((0.006, 0.174, 0.008)),
            origin=Origin(xyz=(-0.030 + 0.010 * i, 0.000, z)),
            material=silver,
            name=f"vent_slat_{i}",
        )

    wand = model.part("wand")
    # Alternating hinge knuckles wrap around the body barrel without overlap.
    wand.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, -0.0550, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="top_knuckle_0",
    )
    wand.visual(
        Box((0.045, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, -0.0550, -0.049)),
        material=blue,
        name="top_fork_0",
    )
    wand.visual(
        Cylinder(radius=0.026, length=0.040),
        origin=Origin(xyz=(0.0, 0.0550, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="top_knuckle_1",
    )
    wand.visual(
        Box((0.045, 0.018, 0.080)),
        origin=Origin(xyz=(0.0, 0.0550, -0.049)),
        material=blue,
        name="top_fork_1",
    )
    wand.visual(
        Box((0.055, 0.135, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.084)),
        material=blue,
        name="top_bridge",
    )
    wand.visual(
        Cylinder(radius=0.028, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.082)),
        material=blue,
        name="top_collar",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, -0.445)),
        material=silver,
        name="straight_tube",
    )
    wand.visual(
        Cylinder(radius=0.027, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, -0.800)),
        material=blue,
        name="lower_collar",
    )
    wand.visual(
        Cylinder(radius=0.023, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.820), rpy=(pi / 2.0, 0.0, 0.0)),
        material=blue,
        name="head_barrel",
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.022, length=0.035),
        origin=Origin(xyz=(0.0, -0.0475, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pitch_knuckle_0",
    )
    floor_head.visual(
        Box((0.045, 0.017, 0.076)),
        origin=Origin(xyz=(0.0, -0.0475, -0.043)),
        material=graphite,
        name="pitch_fork_0",
    )
    floor_head.visual(
        Cylinder(radius=0.022, length=0.035),
        origin=Origin(xyz=(0.0, 0.0475, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pitch_knuckle_1",
    )
    floor_head.visual(
        Box((0.045, 0.017, 0.076)),
        origin=Origin(xyz=(0.0, 0.0475, -0.043)),
        material=graphite,
        name="pitch_fork_1",
    )
    floor_head.visual(
        Box((0.092, 0.120, 0.054)),
        origin=Origin(xyz=(0.020, 0.000, -0.077)),
        material=graphite,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.330, 0.625, 0.066)),
        origin=Origin(xyz=(0.075, 0.000, -0.125)),
        material=blue,
        name="nozzle_shell",
    )
    floor_head.visual(
        Cylinder(radius=0.027, length=0.560),
        origin=Origin(xyz=(0.205, 0.000, -0.148), rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="brush_roller",
    )
    floor_head.visual(
        Box((0.024, 0.600, 0.036)),
        origin=Origin(xyz=(0.244, 0.000, -0.112)),
        material=charcoal,
        name="front_bumper",
    )
    for suffix, y in (("0", -0.315), ("1", 0.315)):
        floor_head.visual(
            Cylinder(radius=0.026, length=0.030),
            origin=Origin(xyz=(-0.085, y, -0.152), rpy=(pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"rear_wheel_{suffix}",
        )

    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.95),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.820)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("fold_joint")
    pitch = object_model.get_articulation("head_pitch")

    ctx.expect_overlap(
        body,
        wand,
        axes="xz",
        elem_a="fold_barrel",
        elem_b="top_knuckle_1",
        min_overlap=0.025,
        name="fold knuckles share a compact hinge axis",
    )
    ctx.expect_gap(
        wand,
        body,
        axis="y",
        positive_elem="top_knuckle_1",
        negative_elem="fold_barrel",
        min_gap=0.0,
        max_gap=0.001,
        name="fold hinge knuckles touch without collision",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="xz",
        elem_a="head_barrel",
        elem_b="pitch_knuckle_1",
        min_overlap=0.020,
        name="floor head knuckles share a compact hinge axis",
    )
    ctx.expect_gap(
        floor_head,
        wand,
        axis="y",
        positive_elem="pitch_knuckle_1",
        negative_elem="head_barrel",
        min_gap=0.0,
        max_gap=0.001,
        name="floor head hinge knuckles touch without collision",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="xy",
        elem_a="straight_tube",
        elem_b="nozzle_shell",
        min_overlap=0.015,
        name="straight wand lands over the floor nozzle footprint",
    )

    rest_wand_box = ctx.part_world_aabb(wand)
    with ctx.pose({fold: 1.20}):
        folded_wand_box = ctx.part_world_aabb(wand)
    ctx.check(
        "fold joint swings the wand about the horizontal hinge",
        rest_wand_box is not None
        and folded_wand_box is not None
        and folded_wand_box[0][0] < rest_wand_box[0][0] - 0.30,
        details=f"rest={rest_wand_box}, folded={folded_wand_box}",
    )

    rest_front_box = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    with ctx.pose({pitch: -0.35}):
        pitched_front_box = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    ctx.check(
        "floor head pitches on its second horizontal hinge",
        rest_front_box is not None
        and pitched_front_box is not None
        and pitched_front_box[1][2] > rest_front_box[1][2] + 0.050,
        details=f"rest={rest_front_box}, pitched={pitched_front_box}",
    )

    return ctx.report()


object_model = build_object_model()
