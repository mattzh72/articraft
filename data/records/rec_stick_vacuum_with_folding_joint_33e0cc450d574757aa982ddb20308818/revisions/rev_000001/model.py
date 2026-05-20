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

    body_gray = model.material("body_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.72, 0.75, 0.79, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.79, 0.81, 0.83, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.10, 0.11, 0.12, 1.0))
    accent_red = model.material("accent_red", rgba=(0.76, 0.15, 0.12, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.58, 0.66, 0.74, 0.42))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Cylinder(radius=0.055, length=0.22),
        origin=Origin(xyz=(-0.02, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_gray,
        name="motor_pod",
    )
    motor_body.visual(
        Cylinder(radius=0.046, length=0.18),
        origin=Origin(xyz=(0.085, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoke_clear,
        name="dust_bin",
    )
    motor_body.visual(
        Box((0.12, 0.045, 0.18)),
        origin=Origin(xyz=(-0.11, 0.0, 0.045)),
        material=accent_red,
        name="handle_grip",
    )
    motor_body.visual(
        Box((0.10, 0.075, 0.11)),
        origin=Origin(xyz=(-0.17, 0.0, -0.06)),
        material=dark_plastic,
        name="battery_pack",
    )
    motor_body.visual(
        Cylinder(radius=0.045, length=0.05),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_silver,
        name="rear_filter_cap",
    )
    motor_body.visual(
        Box((0.05, 0.07, 0.055)),
        origin=Origin(xyz=(0.15, 0.0, -0.045)),
        material=body_gray,
        name="front_mount",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.42, 0.14, 0.26)),
        mass=2.8,
        origin=Origin(xyz=(-0.02, 0.0, 0.0)),
    )

    fold_base = model.part("fold_base")
    fold_base.visual(
        Box((0.02, 0.09, 0.06)),
        origin=Origin(xyz=(0.01, 0.0, 0.0)),
        material=dark_plastic,
        name="rear_flange",
    )
    fold_base.visual(
        Box((0.046, 0.016, 0.06)),
        origin=Origin(xyz=(0.043, 0.028, 0.0)),
        material=dark_plastic,
        name="left_cheek",
    )
    fold_base.visual(
        Box((0.046, 0.016, 0.06)),
        origin=Origin(xyz=(0.043, -0.028, 0.0)),
        material=dark_plastic,
        name="right_cheek",
    )
    fold_base.visual(
        Box((0.024, 0.09, 0.02)),
        origin=Origin(xyz=(0.03, 0.0, 0.022)),
        material=dark_plastic,
        name="top_bridge",
    )
    fold_base.inertial = Inertial.from_geometry(
        Box((0.07, 0.10, 0.07)),
        mass=0.18,
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
    )

    wand_axis = (0.665, 0.0, -0.605)
    wand_axis_length = math.sqrt(wand_axis[0] ** 2 + wand_axis[2] ** 2)
    wand_axis_dir = (
        wand_axis[0] / wand_axis_length,
        0.0,
        wand_axis[2] / wand_axis_length,
    )
    wand_axis_pitch = math.atan2(wand_axis[0], wand_axis[2])

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.016, length=0.045),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wand_metal,
        name="fold_lug",
    )
    wand.visual(
        Cylinder(radius=0.018, length=wand_axis_length),
        origin=Origin(
            xyz=(
                wand_axis_dir[0] * wand_axis_length * 0.5,
                0.0,
                wand_axis_dir[2] * wand_axis_length * 0.5,
            ),
            rpy=(0.0, wand_axis_pitch, 0.0),
        ),
        material=wand_metal,
        name="wand_tube",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.70, 0.05, 0.66)),
        mass=0.5,
        origin=Origin(xyz=(0.33, 0.0, -0.30)),
    )

    head_yoke = model.part("head_yoke")
    rear_socket_length = 0.028
    head_yoke.visual(
        Cylinder(radius=0.020, length=rear_socket_length),
        origin=Origin(
            xyz=(
                wand_axis_dir[0] * rear_socket_length * 0.5,
                0.0,
                wand_axis_dir[2] * rear_socket_length * 0.5,
            ),
            rpy=(0.0, wand_axis_pitch, 0.0),
        ),
        material=dark_plastic,
        name="rear_socket",
    )
    head_yoke.visual(
        Box((0.066, 0.010, 0.012)),
        origin=Origin(xyz=(0.009, 0.021, -0.010)),
        material=dark_plastic,
        name="left_arm",
    )
    head_yoke.visual(
        Box((0.066, 0.010, 0.012)),
        origin=Origin(xyz=(0.009, -0.021, -0.010)),
        material=dark_plastic,
        name="right_arm",
    )
    head_yoke.visual(
        Box((0.018, 0.010, 0.032)),
        origin=Origin(xyz=(0.041, 0.023, -0.022)),
        material=dark_plastic,
        name="left_fork",
    )
    head_yoke.visual(
        Box((0.018, 0.010, 0.032)),
        origin=Origin(xyz=(0.041, -0.023, -0.022)),
        material=dark_plastic,
        name="right_fork",
    )
    head_yoke.visual(
        Box((0.018, 0.058, 0.008)),
        origin=Origin(xyz=(0.041, 0.0, -0.002)),
        material=dark_plastic,
        name="top_bridge",
    )
    head_yoke.inertial = Inertial.from_geometry(
        Box((0.09, 0.10, 0.09)),
        mass=0.16,
        origin=Origin(xyz=(0.015, 0.0, -0.015)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="pitch_lug",
    )
    floor_head.visual(
        Box((0.038, 0.032, 0.032)),
        origin=Origin(xyz=(0.019, 0.0, -0.015)),
        material=dark_plastic,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.29, 0.11, 0.032)),
        origin=Origin(xyz=(0.155, 0.0, -0.048)),
        material=dark_plastic,
        name="nozzle_base",
    )
    floor_head.visual(
        Box((0.115, 0.074, 0.022)),
        origin=Origin(xyz=(0.10, 0.0, -0.024)),
        material=accent_red,
        name="top_cover",
    )
    floor_head.visual(
        Box((0.28, 0.095, 0.014)),
        origin=Origin(xyz=(0.155, 0.0, -0.016)),
        material=satin_silver,
        name="front_lip",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.31, 0.12, 0.08)),
        mass=0.65,
        origin=Origin(xyz=(0.15, 0.0, -0.04)),
    )

    model.articulation(
        "body_to_fold_base",
        ArticulationType.FIXED,
        parent=motor_body,
        child=fold_base,
        origin=Origin(xyz=(0.175, 0.0, -0.045)),
    )
    model.articulation(
        "fold_joint",
        ArticulationType.REVOLUTE,
        parent=fold_base,
        child=wand,
        origin=Origin(xyz=(0.048, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.2,
        ),
    )
    model.articulation(
        "wand_to_head_yoke",
        ArticulationType.FIXED,
        parent=wand,
        child=head_yoke,
        origin=Origin(xyz=(0.665, 0.0, -0.605)),
    )
    model.articulation(
        "head_pitch",
        ArticulationType.REVOLUTE,
        parent=head_yoke,
        child=floor_head,
        origin=Origin(xyz=(0.048, 0.0, -0.023)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-0.35,
            upper=0.7,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    motor_body = object_model.get_part("motor_body")
    fold_base = object_model.get_part("fold_base")
    wand = object_model.get_part("wand")
    head_yoke = object_model.get_part("head_yoke")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("fold_joint")
    head_pitch = object_model.get_articulation("head_pitch")

    ctx.expect_contact(
        fold_base,
        motor_body,
        elem_a="rear_flange",
        elem_b="front_mount",
        name="fold module bolts to motor body nose",
    )
    ctx.expect_contact(
        wand,
        head_yoke,
        elem_a="wand_tube",
        elem_b="rear_socket",
        name="wand tube seats into head yoke",
    )
    ctx.expect_overlap(
        fold_base,
        wand,
        axes="z",
        elem_a="left_cheek",
        elem_b="fold_lug",
        min_overlap=0.02,
        name="fold lug stays captured between hinge cheeks",
    )

    rest_front = ctx.part_element_world_aabb(floor_head, elem="front_lip")
    rest_wand = ctx.part_world_aabb(wand)

    with ctx.pose({fold_joint: 1.0}):
        folded_wand = ctx.part_world_aabb(wand)
        ctx.check(
            "fold joint raises the wand",
            rest_wand is not None
            and folded_wand is not None
            and folded_wand[1][2] > rest_wand[1][2] + 0.20,
            details=f"rest={rest_wand}, folded={folded_wand}",
        )

    with ctx.pose({head_pitch: 0.65}):
        pitched_front = ctx.part_element_world_aabb(floor_head, elem="front_lip")
        ctx.check(
            "floor head pitches nose upward",
            rest_front is not None
            and pitched_front is not None
            and pitched_front[1][2] > rest_front[1][2] + 0.05,
            details=f"rest={rest_front}, pitched={pitched_front}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
