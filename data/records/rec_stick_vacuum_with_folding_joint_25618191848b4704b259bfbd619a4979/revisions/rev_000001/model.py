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
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    silver = model.material("silver", rgba=(0.74, 0.76, 0.79, 1.0))
    copper = model.material("copper", rgba=(0.78, 0.50, 0.22, 1.0))
    cup_clear = model.material("cup_clear", rgba=(0.62, 0.72, 0.82, 0.38))
    soft_black = model.material("soft_black", rgba=(0.08, 0.08, 0.09, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Box((0.16, 0.085, 0.11)),
        origin=Origin(xyz=(0.15, 0.0, 0.045)),
        material=charcoal,
        name="main_shell",
    )
    motor_body.visual(
        Box((0.12, 0.072, 0.075)),
        origin=Origin(xyz=(0.165, 0.0, 0.115)),
        material=charcoal,
        name="battery_pack",
    )
    motor_body.visual(
        Cylinder(radius=0.05, length=0.12),
        origin=Origin(xyz=(0.22, 0.0, 0.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="rear_motor_pod",
    )
    motor_body.visual(
        Cylinder(radius=0.043, length=0.16),
        origin=Origin(xyz=(0.04, 0.0, 0.035), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cup_clear,
        name="dust_cup",
    )
    motor_body.visual(
        Cylinder(radius=0.025, length=0.08),
        origin=Origin(xyz=(-0.05, 0.0, 0.025), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=copper,
        name="cyclone_nose",
    )
    motor_body.visual(
        Box((0.034, 0.058, 0.028)),
        origin=Origin(xyz=(-0.066, 0.0, 0.006)),
        material=charcoal,
        name="fold_mount",
    )
    motor_body.visual(
        Box((0.028, 0.074, 0.014)),
        origin=Origin(xyz=(-0.09, 0.0, -0.058)),
        material=charcoal,
        name="fold_bridge",
    )
    motor_body.visual(
        Box((0.032, 0.012, 0.056)),
        origin=Origin(xyz=(-0.07, 0.031, -0.03)),
        material=soft_black,
        name="left_fold_cheek",
    )
    motor_body.visual(
        Box((0.032, 0.012, 0.056)),
        origin=Origin(xyz=(-0.07, -0.031, -0.03)),
        material=soft_black,
        name="right_fold_cheek",
    )

    handle_geom = tube_from_spline_points(
        [
            (0.095, 0.0, 0.085),
            (0.075, 0.0, 0.155),
            (0.135, 0.0, 0.215),
            (0.235, 0.0, 0.14),
        ],
        radius=0.016,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    motor_body.visual(
        mesh_from_geometry(handle_geom, "vacuum_handle"),
        material=charcoal,
        name="handle_loop",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.36, 0.10, 0.26)),
        mass=2.8,
        origin=Origin(xyz=(0.12, 0.0, 0.07)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.016, length=0.05),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="fold_barrel",
    )
    wand.visual(
        Box((0.10, 0.046, 0.032)),
        origin=Origin(xyz=(0.05, 0.0, -0.018)),
        material=soft_black,
        name="rear_socket",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.66),
        origin=Origin(xyz=(0.41, 0.0, -0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="wand_tube",
    )
    wand.visual(
        Box((0.44, 0.012, 0.008)),
        origin=Origin(xyz=(0.46, 0.0, -0.018)),
        material=copper,
        name="power_strip",
    )
    wand.visual(
        Box((0.088, 0.06, 0.05)),
        origin=Origin(xyz=(0.74, 0.0, -0.045)),
        material=soft_black,
        name="head_connector",
    )
    wand.visual(
        Box((0.022, 0.012, 0.054)),
        origin=Origin(xyz=(0.773, 0.035, -0.045)),
        material=soft_black,
        name="left_pitch_cheek",
    )
    wand.visual(
        Box((0.022, 0.012, 0.054)),
        origin=Origin(xyz=(0.773, -0.035, -0.045)),
        material=soft_black,
        name="right_pitch_cheek",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.80, 0.07, 0.09)),
        mass=0.9,
        origin=Origin(xyz=(0.42, 0.0, -0.02)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.016, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="pitch_barrel",
    )
    floor_head.visual(
        Box((0.035, 0.05, 0.06)),
        origin=Origin(xyz=(0.015, 0.0, -0.03)),
        material=soft_black,
        name="neck_block",
    )
    floor_head.visual(
        Box((0.29, 0.105, 0.022)),
        origin=Origin(xyz=(0.12, 0.0, -0.056)),
        material=charcoal,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.12, 0.09, 0.018)),
        origin=Origin(xyz=(0.09, 0.0, -0.04)),
        material=charcoal,
        name="upper_hump",
    )
    floor_head.visual(
        Cylinder(radius=0.018, length=0.09),
        origin=Origin(xyz=(0.24, 0.0, -0.05), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=copper,
        name="front_roller",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.30, 0.11, 0.07)),
        mass=0.8,
        origin=Origin(xyz=(0.12, 0.0, -0.045)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(-0.07, 0.0, -0.03)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )

    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.80, 0.0, -0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=math.radians(-18.0),
            upper=math.radians(55.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("body_to_wand_fold")
    pitch = object_model.get_articulation("wand_to_floor_head_pitch")

    ctx.expect_gap(
        wand,
        floor_head,
        axis="z",
        positive_elem="wand_tube",
        negative_elem="head_shell",
        min_gap=0.015,
        max_gap=0.08,
        name="wand rides clearly above the floor head",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="y",
        elem_a="head_connector",
        elem_b="pitch_barrel",
        min_overlap=0.04,
        name="pitch barrel stays centered between wand cheeks",
    )

    rest_wand_tube = ctx.part_element_world_aabb(wand, elem="wand_tube")
    with ctx.pose({fold: math.radians(95.0)}):
        folded_wand_tube = ctx.part_element_world_aabb(wand, elem="wand_tube")
        folded_head_shell = ctx.part_element_world_aabb(floor_head, elem="head_shell")

    ctx.check(
        "fold joint raises the wand upward",
        rest_wand_tube is not None
        and folded_wand_tube is not None
        and folded_wand_tube[1][2] > rest_wand_tube[1][2] + 0.45,
        details=f"rest={rest_wand_tube}, folded={folded_wand_tube}",
    )
    ctx.check(
        "folded wand carries the floor head upward",
        folded_head_shell is not None and folded_head_shell[0][2] > 0.18,
        details=f"folded_head_shell={folded_head_shell}",
    )

    rest_roller = ctx.part_element_world_aabb(floor_head, elem="front_roller")
    with ctx.pose({pitch: math.radians(40.0)}):
        pitched_roller = ctx.part_element_world_aabb(floor_head, elem="front_roller")
    ctx.check(
        "floor head pitch lifts the front roller",
        rest_roller is not None
        and pitched_roller is not None
        and pitched_roller[1][2] > rest_roller[1][2] + 0.06,
        details=f"rest={rest_roller}, pitched={pitched_roller}",
    )

    ctx.expect_gap(
        floor_head,
        motor_body,
        axis="x",
        positive_elem="head_shell",
        negative_elem="cyclone_nose",
        min_gap=0.68,
        name="floor head stays well out in front of the motor body",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
