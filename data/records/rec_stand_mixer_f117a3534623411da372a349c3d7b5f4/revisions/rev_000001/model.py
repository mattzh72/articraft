from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _yz_rounded_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    loop = rounded_rect_profile(width, height, radius)
    return [(x_pos, y, z + z_center) for y, z in loop]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bowl_lift_stand_mixer")

    body_finish = model.material("body_finish", rgba=(0.78, 0.11, 0.10, 1.0))
    trim = model.material("trim", rgba=(0.14, 0.14, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.84, 0.85, 0.87, 1.0))
    carriage_metal = model.material("carriage_metal", rgba=(0.45, 0.47, 0.50, 1.0))
    bowl_steel = model.material("bowl_steel", rgba=(0.90, 0.91, 0.93, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.31, 0.275, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=body_finish,
        name="base",
    )

    column_shell = section_loft(
        [
            _yz_rounded_section(
                -0.105,
                width=0.126,
                height=0.240,
                radius=0.028,
                z_center=0.170,
            ),
            _yz_rounded_section(
                -0.082,
                width=0.136,
                height=0.282,
                radius=0.034,
                z_center=0.212,
            ),
            _yz_rounded_section(
                -0.045,
                width=0.146,
                height=0.312,
                radius=0.038,
                z_center=0.245,
            ),
        ]
    )
    body.visual(
        mesh_from_geometry(column_shell, "mixer_column_shell"),
        material=body_finish,
        name="column_shell",
    )
    body.visual(
        mesh_from_geometry(
            section_loft(
                [
                    _yz_rounded_section(
                        -0.005,
                        width=0.136,
                        height=0.104,
                        radius=0.028,
                        z_center=0.394,
                    ),
                    _yz_rounded_section(
                        0.105,
                        width=0.166,
                        height=0.124,
                        radius=0.038,
                        z_center=0.404,
                    ),
                    _yz_rounded_section(
                        0.220,
                        width=0.126,
                        height=0.098,
                        radius=0.028,
                        z_center=0.396,
                    ),
                ]
            ),
            "mixer_head_shell",
        ),
        material=body_finish,
        name="head_shell",
    )
    body.visual(
        Box((0.074, 0.100, 0.064)),
        origin=Origin(xyz=(-0.020, 0.0, 0.338)),
        material=body_finish,
        name="neck",
    )
    body.visual(
        Box((0.022, 0.024, 0.140)),
        origin=Origin(xyz=(-0.009, -0.125, 0.119)),
        material=body_finish,
        name="guide_0",
    )
    body.visual(
        Box((0.022, 0.024, 0.140)),
        origin=Origin(xyz=(-0.009, 0.125, 0.119)),
        material=body_finish,
        name="guide_1",
    )
    body.visual(
        Box((0.030, 0.016, 0.090)),
        origin=Origin(xyz=(-0.030, 0.127, 0.089)),
        material=body_finish,
        name="lever_bracket",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(-0.020, 0.1275, 0.126), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="lever_mount",
    )
    body.visual(
        Cylinder(radius=0.030, length=0.036),
        origin=Origin(xyz=(0.102, 0.0, 0.353)),
        material=trim,
        name="drive_housing",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.172, 0.270, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_metal,
        name="platform",
    )
    carriage.visual(
        Box((0.024, 0.018, 0.130)),
        origin=Origin(xyz=(-0.078, -0.125, 0.061)),
        material=carriage_metal,
        name="arm_0",
    )
    carriage.visual(
        Box((0.024, 0.018, 0.130)),
        origin=Origin(xyz=(-0.078, 0.125, 0.061)),
        material=carriage_metal,
        name="arm_1",
    )

    bowl = model.part("bowl")
    bowl_shell = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.030, 0.000),
            (0.062, 0.014),
            (0.096, 0.056),
            (0.108, 0.112),
            (0.112, 0.148),
        ],
        inner_profile=[
            (0.000, 0.004),
            (0.056, 0.016),
            (0.090, 0.058),
            (0.102, 0.112),
            (0.106, 0.143),
        ],
        segments=56,
        end_cap="round",
        lip_samples=8,
    )
    bowl.visual(
        mesh_from_geometry(bowl_shell, "mixer_bowl"),
        material=bowl_steel,
        name="bowl_shell",
    )

    beater = model.part("beater")
    beater.visual(
        Cylinder(radius=0.008, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=steel,
        name="shaft",
    )
    beater.visual(
        Box((0.014, 0.008, 0.098)),
        origin=Origin(xyz=(0.0, 0.0, -0.089)),
        material=steel,
        name="paddle",
    )
    beater.visual(
        Box((0.064, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.124)),
        material=steel,
        name="lower_bar",
    )
    beater.visual(
        Box((0.058, 0.008, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.057)),
        material=steel,
        name="upper_bar",
    )
    beater.visual(
        Box((0.012, 0.008, 0.055)),
        origin=Origin(xyz=(-0.026, 0.0, -0.091)),
        material=steel,
        name="leg_0",
    )
    beater.visual(
        Box((0.012, 0.008, 0.055)),
        origin=Origin(xyz=(0.026, 0.0, -0.091)),
        material=steel,
        name="leg_1",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.0, 0.009, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="pivot",
    )
    lever.visual(
        Box((0.062, 0.012, 0.010)),
        origin=Origin(xyz=(0.031, 0.017, 0.0)),
        material=trim,
        name="arm",
    )
    lever.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(xyz=(0.067, 0.017, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="grip",
    )

    model.articulation(
        "body_to_carriage",
        ArticulationType.PRISMATIC,
        parent=body,
        child=carriage,
        origin=Origin(xyz=(0.092, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.080,
            lower=0.0,
            upper=0.070,
        ),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "body_to_beater",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=beater,
        origin=Origin(xyz=(0.102, 0.0, 0.335)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(-0.020, 0.1375, 0.126)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    beater = object_model.get_part("beater")
    lever = object_model.get_part("lever")

    lift = object_model.get_articulation("body_to_carriage")
    spin = object_model.get_articulation("body_to_beater")
    lever_joint = object_model.get_articulation("body_to_lever")

    lift_upper = 0.070
    lever_upper = 0.55

    ctx.expect_gap(
        bowl,
        carriage,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="platform",
        max_gap=0.003,
        max_penetration=0.001,
        name="bowl seats on the platform",
    )
    ctx.expect_overlap(
        bowl,
        carriage,
        axes="xy",
        elem_a="bowl_shell",
        elem_b="platform",
        min_overlap=0.060,
        name="platform supports the bowl footprint",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: lift_upper}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_within(
            beater,
            bowl,
            axes="xy",
            inner_elem="paddle",
            outer_elem="bowl_shell",
            margin=0.018,
            name="raised bowl centers under the beater",
        )
        ctx.expect_gap(
            body,
            bowl,
            axis="z",
            positive_elem="drive_housing",
            negative_elem="bowl_shell",
            min_gap=0.002,
            max_gap=0.050,
            name="raised bowl stays just below the head housing",
        )

    with ctx.pose({lift: lift_upper, spin: math.pi / 3.0}):
        ctx.expect_within(
            beater,
            bowl,
            axes="xy",
            inner_elem="paddle",
            outer_elem="bowl_shell",
            margin=0.020,
            name="rotated beater remains inside the bowl plan",
        )

    ctx.check(
        "carriage lifts upward",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rest_grip = ctx.part_element_world_aabb(lever, elem="grip")
    with ctx.pose({lever_joint: lever_upper}):
        raised_grip = ctx.part_element_world_aabb(lever, elem="grip")

    lever_tip_rises = False
    lever_tip_details = f"rest={rest_grip}, raised={raised_grip}"
    if rest_grip is not None and raised_grip is not None:
        rest_center_z = 0.5 * (rest_grip[0][2] + rest_grip[1][2])
        raised_center_z = 0.5 * (raised_grip[0][2] + raised_grip[1][2])
        lever_tip_rises = raised_center_z > rest_center_z + 0.020

    ctx.check("lever rotates upward at its upper pose", lever_tip_rises, details=lever_tip_details)

    return ctx.report()


object_model = build_object_model()
