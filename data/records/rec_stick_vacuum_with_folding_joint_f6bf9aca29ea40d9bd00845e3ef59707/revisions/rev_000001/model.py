from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    body_dark = model.material("body_dark", rgba=(0.17, 0.18, 0.19, 1.0))
    battery_dark = model.material("battery_dark", rgba=(0.22, 0.23, 0.24, 1.0))
    tube_silver = model.material("tube_silver", rgba=(0.70, 0.73, 0.76, 1.0))
    accent_red = model.material("accent_red", rgba=(0.76, 0.18, 0.14, 1.0))
    head_dark = model.material("head_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Box((0.24, 0.10, 0.14)),
        origin=Origin(xyz=(0.00, 0.00, 0.13)),
        material=body_dark,
        name="main_shell",
    )
    motor_body.visual(
        Box((0.15, 0.085, 0.10)),
        origin=Origin(xyz=(-0.13, 0.00, 0.055)),
        material=battery_dark,
        name="battery_pack",
    )
    motor_body.visual(
        Box((0.12, 0.055, 0.12)),
        origin=Origin(xyz=(-0.02, 0.00, 0.20)),
        material=body_dark,
        name="handle_stem",
    )
    motor_body.visual(
        Box((0.09, 0.040, 0.05)),
        origin=Origin(xyz=(-0.08, 0.00, 0.28)),
        material=body_dark,
        name="handle_grip",
    )
    motor_body.visual(
        Box((0.08, 0.08, 0.09)),
        origin=Origin(xyz=(0.15, 0.00, 0.08)),
        material=accent_red,
        name="front_nose",
    )
    motor_body.visual(
        Box((0.05, 0.09, 0.07)),
        origin=Origin(xyz=(0.215, 0.00, 0.04)),
        material=body_dark,
        name="fold_cradle",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.45, 0.10, 0.31)),
        mass=3.7,
        origin=Origin(xyz=(0.02, 0.00, 0.155)),
    )

    wand = model.part("wand")
    wand.visual(
        Box((0.055, 0.08, 0.06)),
        origin=Origin(xyz=(0.0275, 0.00, 0.00)),
        material=body_dark,
        name="rear_knuckle",
    )
    wand.visual(
        Box((0.68, 0.042, 0.036)),
        origin=Origin(xyz=(0.34, 0.00, 0.00)),
        material=tube_silver,
        name="wand_tube",
    )
    wand.visual(
        Box((0.06, 0.065, 0.055)),
        origin=Origin(xyz=(0.68, 0.00, -0.005)),
        material=body_dark,
        name="front_socket",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.74, 0.08, 0.06)),
        mass=1.1,
        origin=Origin(xyz=(0.37, 0.00, 0.00)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.05, 0.085, 0.09)),
        origin=Origin(xyz=(0.025, 0.00, -0.01)),
        material=head_dark,
        name="neck_housing",
    )
    floor_head.visual(
        Box((0.03, 0.12, 0.04)),
        origin=Origin(xyz=(0.015, 0.00, 0.018)),
        material=body_dark,
        name="pivot_block",
    )
    floor_head.visual(
        Box((0.10, 0.28, 0.032)),
        origin=Origin(xyz=(0.05, 0.00, -0.061)),
        material=head_dark,
        name="head_nozzle",
    )
    floor_head.visual(
        Box((0.015, 0.28, 0.02)),
        origin=Origin(xyz=(0.1025, 0.00, -0.051)),
        material=accent_red,
        name="front_lip",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.11, 0.28, 0.10)),
        mass=0.9,
        origin=Origin(xyz=(0.055, 0.00, -0.02)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.24, 0.00, 0.04)),
        # The wand extends along local +X from the fold plane.
        # -Y makes positive q fold the free end upward toward +Z.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(135.0),
        ),
    )

    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.71, 0.00, -0.005)),
        # The floor head body projects forward along local +X from the hinge.
        # -Y makes positive q tip the nozzle upward.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=math.radians(-35.0),
            upper=math.radians(30.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    head_joint = object_model.get_articulation("wand_to_floor_head")

    ctx.expect_gap(
        wand,
        motor_body,
        axis="x",
        positive_elem="rear_knuckle",
        negative_elem="fold_cradle",
        max_gap=0.001,
        max_penetration=1e-5,
        name="wand knuckle seats against fold cradle",
    )
    ctx.expect_overlap(
        wand,
        motor_body,
        axes="yz",
        elem_a="rear_knuckle",
        elem_b="fold_cradle",
        min_overlap=0.05,
        name="fold joint housings overlap in side profile",
    )
    ctx.expect_gap(
        floor_head,
        wand,
        axis="x",
        positive_elem="neck_housing",
        negative_elem="front_socket",
        max_gap=0.001,
        max_penetration=1e-5,
        name="floor head neck meets wand socket",
    )
    ctx.expect_overlap(
        floor_head,
        wand,
        axes="yz",
        elem_a="neck_housing",
        elem_b="front_socket",
        min_overlap=0.045,
        name="floor head hinge has broad boxy overlap",
    )

    tube_rest = ctx.part_element_world_aabb(wand, elem="wand_tube")
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        tube_folded = ctx.part_element_world_aabb(wand, elem="wand_tube")
    ctx.check(
        "fold joint lifts the wand upward",
        tube_rest is not None
        and tube_folded is not None
        and tube_folded[1][2] > tube_rest[1][2] + 0.30,
        details=f"rest={tube_rest}, folded={tube_folded}",
    )

    nozzle_rest = ctx.part_element_world_aabb(floor_head, elem="head_nozzle")
    with ctx.pose({head_joint: head_joint.motion_limits.upper}):
        nozzle_pitched = ctx.part_element_world_aabb(floor_head, elem="head_nozzle")
    ctx.check(
        "floor head can pitch upward at the neck",
        nozzle_rest is not None
        and nozzle_pitched is not None
        and nozzle_pitched[1][2] > nozzle_rest[1][2] + 0.03,
        details=f"rest={nozzle_rest}, pitched={nozzle_pitched}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
