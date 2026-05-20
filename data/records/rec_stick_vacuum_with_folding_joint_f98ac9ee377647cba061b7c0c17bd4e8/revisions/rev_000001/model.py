from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stick_vacuum_with_folding_wand")

    charcoal = model.material("charcoal", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.23, 0.24, 0.26, 1.0))
    silver = model.material("silver", rgba=(0.76, 0.78, 0.80, 1.0))
    copper = model.material("copper", rgba=(0.74, 0.50, 0.25, 1.0))
    red = model.material("red", rgba=(0.70, 0.09, 0.11, 1.0))
    translucent_bin = model.material("translucent_bin", rgba=(0.55, 0.62, 0.70, 0.92))

    motor_body = model.part("motor_body")
    motor_body.inertial = Inertial.from_geometry(
        Box((0.34, 0.16, 0.38)),
        mass=2.4,
        origin=Origin(xyz=(-0.10, 0.0, 0.16)),
    )
    motor_body.visual(
        Cylinder(radius=0.056, length=0.110),
        origin=Origin(xyz=(0.015, 0.0, 0.105), rpy=(0.0, pi / 2.0, 0.0)),
        material=translucent_bin,
        name="dust_bin",
    )
    motor_body.visual(
        Cylinder(radius=0.050, length=0.175),
        origin=Origin(xyz=(-0.115, 0.0, 0.110), rpy=(0.0, pi / 2.0, 0.0)),
        material=red,
        name="motor_barrel",
    )
    motor_body.visual(
        Cylinder(radius=0.034, length=0.082),
        origin=Origin(xyz=(0.105, 0.0, 0.083), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="front_nozzle",
    )
    motor_body.visual(
        Box((0.050, 0.036, 0.245)),
        origin=Origin(xyz=(-0.155, 0.0, 0.235), rpy=(0.0, -0.18, 0.0)),
        material=dark_gray,
        name="handle",
    )
    motor_body.visual(
        Box((0.108, 0.060, 0.084)),
        origin=Origin(xyz=(-0.245, 0.0, 0.048)),
        material=charcoal,
        name="battery_pack",
    )
    motor_body.visual(
        Box((0.082, 0.064, 0.040)),
        origin=Origin(xyz=(-0.020, 0.0, 0.040)),
        material=dark_gray,
        name="fold_yoke",
    )
    motor_body.visual(
        Cylinder(radius=0.040, length=0.042),
        origin=Origin(xyz=(-0.218, 0.0, 0.115), rpy=(0.0, pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_filter_cap",
    )

    left_cheek = model.part("left_cheek")
    left_cheek.inertial = Inertial.from_geometry(
        Box((0.060, 0.012, 0.078)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    left_cheek.visual(
        Box((0.060, 0.008, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_gray,
        name="left_plate",
    )
    left_cheek.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="left_bushing",
    )

    right_cheek = model.part("right_cheek")
    right_cheek.inertial = Inertial.from_geometry(
        Box((0.060, 0.012, 0.078)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )
    right_cheek.visual(
        Box((0.060, 0.008, 0.078)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_gray,
        name="right_plate",
    )
    right_cheek.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="right_bushing",
    )

    wand = model.part("wand")
    wand.inertial = Inertial.from_geometry(
        Box((0.060, 0.060, 0.730)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -0.365)),
    )
    wand.visual(
        Cylinder(radius=0.015, length=0.064),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="upper_barrel",
    )
    wand.visual(
        Box((0.046, 0.046, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=charcoal,
        name="hinge_tongue",
    )
    wand.visual(
        Cylinder(radius=0.0175, length=0.680),
        origin=Origin(xyz=(0.0, 0.0, -0.360)),
        material=copper,
        name="wand_tube",
    )
    wand.visual(
        Box((0.042, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.685)),
        material=charcoal,
        name="lower_collar",
    )

    floor_head = model.part("floor_head")
    floor_head.inertial = Inertial.from_geometry(
        Box((0.320, 0.120, 0.055)),
        mass=0.85,
        origin=Origin(xyz=(0.090, 0.0, -0.028)),
    )
    floor_head.visual(
        Box((0.055, 0.052, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_gray,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.070, 0.115, 0.022)),
        origin=Origin(xyz=(-0.028, 0.0, -0.024)),
        material=charcoal,
        name="rear_bumper",
    )
    floor_head.visual(
        Box((0.300, 0.108, 0.028)),
        origin=Origin(xyz=(0.100, 0.0, -0.028)),
        material=charcoal,
        name="head_body",
    )
    floor_head.visual(
        Box((0.320, 0.086, 0.010)),
        origin=Origin(xyz=(0.100, 0.0, -0.046)),
        material=red,
        name="front_lip",
    )

    model.articulation(
        "body_to_left_cheek",
        ArticulationType.FIXED,
        parent=motor_body,
        child=left_cheek,
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
    )
    model.articulation(
        "body_to_right_cheek",
        ArticulationType.FIXED,
        parent=motor_body,
        child=right_cheek,
        origin=Origin(xyz=(0.0, -0.036, 0.0)),
    )
    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=2.0,
            lower=0.0,
            upper=radians(78.0),
        ),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.700)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=radians(-32.0),
            upper=radians(32.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    left_cheek = object_model.get_part("left_cheek")
    right_cheek = object_model.get_part("right_cheek")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_pitch = object_model.get_articulation("wand_to_floor_head")

    ctx.expect_origin_gap(
        left_cheek,
        wand,
        axis="y",
        min_gap=0.020,
        max_gap=0.050,
        name="left cheek sits outside the wand pivot",
    )
    ctx.expect_origin_gap(
        wand,
        right_cheek,
        axis="y",
        min_gap=0.020,
        max_gap=0.050,
        name="right cheek sits outside the wand pivot",
    )
    ctx.expect_origin_gap(
        motor_body,
        floor_head,
        axis="z",
        min_gap=0.65,
        name="motor body stays above the floor head",
    )
    ctx.expect_gap(
        wand,
        floor_head,
        axis="z",
        positive_elem="lower_collar",
        negative_elem="neck_block",
        max_gap=0.001,
        max_penetration=1e-6,
        name="wand lower collar seats onto the floor-head neck",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="y",
        min_overlap=0.045,
        name="floor head stays centered under the wand",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint swings the wand forward and upward",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] > rest_head_pos[0] + 0.35
        and folded_head_pos[2] > rest_head_pos[2] + 0.18,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    rest_front = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    with ctx.pose({head_pitch: head_pitch.motion_limits.upper}):
        pitched_front = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    ctx.check(
        "floor head pitch lifts the nose",
        rest_front is not None
        and pitched_front is not None
        and pitched_front[1][2] > rest_front[1][2] + 0.035,
        details=f"rest={rest_front}, pitched={pitched_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
