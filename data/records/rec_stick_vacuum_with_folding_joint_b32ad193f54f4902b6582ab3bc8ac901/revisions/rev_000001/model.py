from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
    model = ArticulatedObject(name="folding_stick_vacuum")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    wand_silver = model.material("wand_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    accent_red = model.material("accent_red", rgba=(0.82, 0.12, 0.14, 1.0))
    floor_black = model.material("floor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dustbin_clear = model.material("dustbin_clear", rgba=(0.62, 0.76, 0.86, 0.45))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Box((0.060, 0.070, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, 0.043)),
        material=body_dark,
        name="fold_housing",
    )
    motor_body.visual(
        Box((0.070, 0.075, 0.220)),
        origin=Origin(xyz=(0.000, 0.000, 0.160)),
        material=body_dark,
        name="spine",
    )
    motor_body.visual(
        Cylinder(radius=0.060, length=0.220),
        origin=Origin(xyz=(0.035, 0.000, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dustbin_clear,
        name="dustbin",
    )
    motor_body.visual(
        Cylinder(radius=0.050, length=0.100),
        origin=Origin(xyz=(0.140, 0.000, 0.300), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_red,
        name="motor_pod",
    )
    motor_body.visual(
        Cylinder(radius=0.022, length=0.230),
        origin=Origin(xyz=(-0.050, 0.000, 0.395), rpy=(0.0, -0.47, 0.0)),
        material=body_dark,
        name="handle_tube",
    )
    motor_body.visual(
        Cylinder(radius=0.028, length=0.180),
        origin=Origin(xyz=(-0.128, 0.000, 0.558), rpy=(0.0, -0.36, 0.0)),
        material=body_dark,
        name="hand_grip",
    )
    motor_body.visual(
        Box((0.094, 0.074, 0.185)),
        origin=Origin(xyz=(-0.085, 0.000, 0.250)),
        material=accent_red,
        name="battery_pack",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.340, 0.160, 0.700)),
        mass=3.6,
        origin=Origin(xyz=(0.000, 0.000, 0.320)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="upper_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.023, length=0.060),
        origin=Origin(xyz=(0.000, 0.000, -0.030)),
        material=body_dark,
        name="upper_collar",
    )
    wand.visual(
        Cylinder(radius=0.016, length=0.700),
        origin=Origin(xyz=(0.000, 0.000, -0.380)),
        material=wand_silver,
        name="wand_tube",
    )
    wand.visual(
        Box((0.050, 0.048, 0.050)),
        origin=Origin(xyz=(0.000, 0.000, -0.717)),
        material=body_dark,
        name="lower_neck",
    )
    wand.visual(
        Cylinder(radius=0.013, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, -0.742), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="head_pivot_barrel",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.080, 0.080, 0.780)),
        mass=0.85,
        origin=Origin(xyz=(0.000, 0.000, -0.390)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.046, 0.050, 0.040)),
        origin=Origin(xyz=(0.037, 0.000, -0.020)),
        material=body_dark,
        name="head_neck",
    )
    floor_head.visual(
        Box((0.270, 0.112, 0.030)),
        origin=Origin(xyz=(0.135, 0.000, -0.047)),
        material=floor_black,
        name="nozzle_body",
    )
    floor_head.visual(
        Box((0.292, 0.118, 0.010)),
        origin=Origin(xyz=(0.146, 0.000, -0.066)),
        material=accent_red,
        name="base_plate",
    )
    floor_head.visual(
        Cylinder(radius=0.016, length=0.102),
        origin=Origin(xyz=(0.274, 0.000, -0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="front_bumper",
    )
    floor_head.visual(
        Cylinder(radius=0.012, length=0.084),
        origin=Origin(xyz=(-0.005, 0.000, -0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="rear_roller",
    )
    floor_head.visual(
        Cylinder(radius=0.011, length=0.090),
        origin=Origin(xyz=(0.240, 0.000, -0.072), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="front_roller",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.310, 0.130, 0.090)),
        mass=1.15,
        origin=Origin(xyz=(0.145, 0.000, -0.045)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.000, 0.000, -0.742)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_pitch = object_model.get_articulation("wand_to_floor_head_pitch")

    ctx.expect_origin_gap(
        motor_body,
        floor_head,
        axis="z",
        min_gap=0.70,
        max_gap=0.78,
        name="floor head sits far below the motor body",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="xy",
        min_overlap=0.01,
        elem_a="lower_neck",
        elem_b="head_neck",
        name="wand neck stays aligned over floor head hinge",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    rest_front_aabb = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    upper_fold = fold_joint.motion_limits.upper if fold_joint.motion_limits else None
    upper_pitch = head_pitch.motion_limits.upper if head_pitch.motion_limits else None

    with ctx.pose({fold_joint: upper_fold if upper_fold is not None else 1.25}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint lifts the long chain upward for storage",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[2] > rest_head_pos[2] + 0.40
        and folded_head_pos[0] < rest_head_pos[0] - 0.50,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    with ctx.pose({head_pitch: upper_pitch if upper_pitch is not None else 0.55}):
        pitched_front_aabb = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    ctx.check(
        "floor head pitch lifts the nose",
        rest_front_aabb is not None
        and pitched_front_aabb is not None
        and pitched_front_aabb[0][2] > rest_front_aabb[0][2] + 0.06,
        details=f"rest={rest_front_aabb}, pitched={pitched_front_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
