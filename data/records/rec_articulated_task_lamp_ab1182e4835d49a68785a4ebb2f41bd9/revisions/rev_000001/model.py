from __future__ import annotations

import math

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
    mesh_from_geometry,
    tube_from_spline_points,
)


CLAMP_WIDTH = 0.062
SHOULDER_Z = 0.138
LOWER_ARM_LENGTH = 0.275
UPPER_ARM_LENGTH = 0.245
ROD_OFFSET_Y = 0.018
ROD_RADIUS = 0.0045
BARREL_RADIUS = 0.0105
BARREL_LENGTH = 0.026
YOKE_Y = 0.021

SHADE_RADIUS = 0.034
SHADE_INNER_RADIUS = 0.031
SHADE_LENGTH = 0.085
SHADE_BACK_WALL = 0.007


def _spring_mesh(
    *,
    start_x: float,
    end_x: float,
    center_y: float,
    center_z: float,
    coil_radius: float,
    wire_radius: float,
    turns: float,
) -> object:
    lead = 0.006
    total_samples = max(48, int(turns * 18.0))
    points: list[tuple[float, float, float]] = [
        (start_x - lead, center_y, center_z),
        (start_x, center_y, center_z),
    ]
    coil_length = end_x - start_x
    for index in range(total_samples + 1):
        u = index / total_samples
        theta = turns * 2.0 * math.pi * u
        points.append(
            (
                start_x + coil_length * u,
                center_y + coil_radius * math.cos(theta),
                center_z + coil_radius * math.sin(theta),
            )
        )
    points.extend(
        [
            (end_x, center_y, center_z),
            (end_x + lead, center_y, center_z),
        ]
    )
    return tube_from_spline_points(
        points,
        radius=wire_radius,
        samples_per_segment=4,
        radial_segments=12,
        cap_ends=True,
    )


def _shade_shell_shape() -> object:
    return (
        cq.Workplane("YZ")
        .circle(SHADE_RADIUS)
        .extrude(SHADE_LENGTH)
        .faces(">X")
        .workplane()
        .circle(SHADE_INNER_RADIUS)
        .cutBlind(-(SHADE_LENGTH - SHADE_BACK_WALL))
    )


def _add_arm_link(
    part,
    *,
    length: float,
    rod_name_prefix: str,
    spring_mesh_name: str,
) -> None:
    part.visual(
        Cylinder(radius=BARREL_RADIUS, length=BARREL_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        name="root_barrel",
    )
    part.visual(
        Box((0.022, 0.034, 0.028)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        name="root_block",
    )

    rod_start_x = 0.012
    rod_end_x = length - 0.040
    rod_length = rod_end_x - rod_start_x
    rod_center_x = 0.5 * (rod_start_x + rod_end_x)
    for index, y_pos in enumerate((-ROD_OFFSET_Y, ROD_OFFSET_Y)):
        part.visual(
            Cylinder(radius=ROD_RADIUS, length=rod_length),
            origin=Origin(
                xyz=(rod_center_x, y_pos, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            name=f"{rod_name_prefix}_{index}",
        )

    part.visual(
        mesh_from_geometry(
            _spring_mesh(
                start_x=0.018,
                end_x=length - 0.038,
                center_y=0.0,
                center_z=0.010,
                coil_radius=0.0037,
                wire_radius=0.0015,
                turns=7.0,
            ),
            spring_mesh_name,
        ),
        name="balance_spring",
    )
    part.visual(
        Box((0.024, 0.056, 0.024)),
        origin=Origin(xyz=(length - 0.033, 0.0, 0.0)),
        name="end_bridge",
    )
    for index, y_pos in enumerate((-YOKE_Y, YOKE_Y)):
        part.visual(
            Box((0.022, 0.014, 0.034)),
            origin=Origin(xyz=(length - 0.011, y_pos, 0.0)),
            name=f"end_ear_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_task_lamp")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.27, 0.29, 0.31, 1.0))
    spring_steel = model.material("spring_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    satin_silver = model.material("satin_silver", rgba=(0.82, 0.83, 0.84, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.92, 0.88, 1.0))

    clamp = model.part("clamp")
    clamp.visual(
        Box((0.014, CLAMP_WIDTH, 0.112)),
        origin=Origin(xyz=(-0.007, 0.0, 0.056)),
        material=powder_black,
        name="back_plate",
    )
    clamp.visual(
        Box((0.084, CLAMP_WIDTH, 0.014)),
        origin=Origin(xyz=(0.028, 0.0, 0.103)),
        material=powder_black,
        name="top_jaw",
    )
    clamp.visual(
        Box((0.056, CLAMP_WIDTH, 0.016)),
        origin=Origin(xyz=(0.014, 0.0, 0.008)),
        material=powder_black,
        name="bottom_jaw",
    )
    clamp.visual(
        Box((0.030, CLAMP_WIDTH, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, 0.024)),
        material=powder_black,
        name="lower_reinforcement",
    )
    clamp.visual(
        Box((0.016, 0.038, 0.050)),
        origin=Origin(xyz=(-0.018, 0.0, 0.117)),
        material=powder_black,
        name="shoulder_column",
    )
    for index, y_pos in enumerate((-YOKE_Y, YOKE_Y)):
        clamp.visual(
            Box((0.018, 0.014, 0.032)),
            origin=Origin(xyz=(-0.008, y_pos, SHOULDER_Z)),
            material=powder_black,
            name=f"shoulder_ear_{index}",
        )
    clamp.visual(
        Cylinder(radius=0.007, length=0.076),
        origin=Origin(xyz=(0.034, 0.0, 0.058), rpy=(0.0, 0.0, 0.0)),
        material=graphite,
        name="clamp_screw",
    )
    clamp.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.034, 0.0, 0.017)),
        material=satin_silver,
        name="pressure_pad",
    )
    clamp.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.034, 0.0, 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="screw_knob",
    )

    lower_arm = model.part("lower_arm")
    _add_arm_link(
        lower_arm,
        length=LOWER_ARM_LENGTH,
        rod_name_prefix="lower_rod",
        spring_mesh_name="lower_balance_spring",
    )
    for visual_name in ("root_barrel", "root_block", "end_bridge", "end_ear_0", "end_ear_1"):
        lower_arm.get_visual(visual_name).material = graphite
    for visual_name in ("lower_rod_0", "lower_rod_1"):
        lower_arm.get_visual(visual_name).material = satin_silver
    lower_arm.get_visual("balance_spring").material = spring_steel

    upper_arm = model.part("upper_arm")
    _add_arm_link(
        upper_arm,
        length=UPPER_ARM_LENGTH,
        rod_name_prefix="upper_rod",
        spring_mesh_name="upper_balance_spring",
    )
    for visual_name in ("root_barrel", "root_block", "end_bridge", "end_ear_0", "end_ear_1"):
        upper_arm.get_visual(visual_name).material = graphite
    for visual_name in ("upper_rod_0", "upper_rod_1"):
        upper_arm.get_visual(visual_name).material = satin_silver
    upper_arm.get_visual("balance_spring").material = spring_steel

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.009, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    shade.visual(
        Box((0.042, 0.032, 0.028)),
        origin=Origin(xyz=(0.019, 0.0, -0.014)),
        material=graphite,
        name="mount_strap",
    )
    shade.visual(
        mesh_from_cadquery(_shade_shell_shape(), "task_lamp_shade_shell"),
        origin=Origin(xyz=(0.034, 0.0, -0.022)),
        material=powder_black,
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.045),
        origin=Origin(xyz=(0.046, 0.0, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="socket",
    )
    shade.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.076, 0.0, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="bulb",
    )

    model.articulation(
        "clamp_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.4,
            lower=-0.95,
            upper=1.15,
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.6,
            lower=-1.35,
            upper=1.25,
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.00,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp = object_model.get_part("clamp")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")
    shoulder = object_model.get_articulation("clamp_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_hinge = object_model.get_articulation("upper_arm_to_shade")

    ctx.expect_origin_gap(
        shade,
        clamp,
        axis="x",
        min_gap=0.48,
        name="shade reaches forward from the desk clamp",
    )

    upper_rest = ctx.part_world_position(upper_arm)
    with ctx.pose({shoulder: 0.95}):
        upper_raised = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder hinge lifts the elbow upward",
        upper_rest is not None
        and upper_raised is not None
        and upper_raised[2] > upper_rest[2] + 0.18,
        details=f"rest={upper_rest}, raised={upper_raised}",
    )

    with ctx.pose({shoulder: 0.45, elbow: 0.0}):
        shade_rest = ctx.part_world_position(shade)
    with ctx.pose({shoulder: 0.45, elbow: 0.95}):
        shade_raised = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge lifts the lamp head higher",
        shade_rest is not None
        and shade_raised is not None
        and shade_raised[2] > shade_rest[2] + 0.08,
        details=f"rest={shade_rest}, raised={shade_raised}",
    )

    with ctx.pose({shoulder: 0.40, elbow: 0.20, shade_hinge: -0.75}):
        low_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shoulder: 0.40, elbow: 0.20, shade_hinge: 0.55}):
        high_shell_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    low_shell_max_z = None if low_shell_aabb is None else low_shell_aabb[1][2]
    high_shell_max_z = None if high_shell_aabb is None else high_shell_aabb[1][2]
    ctx.check(
        "shade hinge changes the head aim noticeably",
        low_shell_max_z is not None
        and high_shell_max_z is not None
        and high_shell_max_z > low_shell_max_z + 0.03,
        details=f"down={low_shell_aabb}, up={high_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
