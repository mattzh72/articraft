from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PRIMARY_ARM_LENGTH = 0.42
SECONDARY_ARM_LENGTH = 0.285
PRIMARY_START = 0.019
SECONDARY_START = 0.016
HEAD_START = 0.014


def _box(size_x: float, size_y: float, size_z: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate((x, y, z))


def _cyl_y(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _cyl_z(radius: float, length: float, *, x: float = 0.0, y: float = 0.0, z_bottom: float = 0.0):
    return cq.Workplane("XY").center(x, y).circle(radius).extrude(length).translate((0.0, 0.0, z_bottom))


def _build_clamp_base_shape():
    tower = _box(0.030, 0.044, 0.150, x=-0.020, z=-0.087)
    jaw = _box(0.092, 0.070, 0.012, x=-0.014, z=-0.124)
    rear_block = _box(0.032, 0.070, 0.070, x=-0.044, z=-0.157)
    throat_bridge = _box(0.056, 0.024, 0.018, x=-0.018, z=-0.148)
    screw_boss = _box(0.032, 0.044, 0.016, x=0.006, z=-0.165)

    shoulder_web = _box(0.018, 0.044, 0.034, x=-0.024, z=-0.004)
    shoulder_cheek_pos = _cyl_y(0.015, 0.008, x=0.0, y=0.017, z=0.0)
    shoulder_cheek_neg = _cyl_y(0.015, 0.008, x=0.0, y=-0.017, z=0.0)

    screw = _cyl_z(0.006, 0.075, x=0.010, z_bottom=-0.220)
    pressure_pad = _cyl_z(0.018, 0.005, x=0.010, z_bottom=-0.149)
    knob_bar = _cyl_y(0.005, 0.085, x=0.010, z=-0.223)

    clamp_shape = tower
    for solid in (
        jaw,
        rear_block,
        throat_bridge,
        screw_boss,
        shoulder_web,
        shoulder_cheek_pos,
        shoulder_cheek_neg,
        screw,
        pressure_pad,
        knob_bar,
    ):
        clamp_shape = clamp_shape.union(solid)
    return clamp_shape


def _build_primary_arm_shape():
    shoulder_lug = _cyl_y(0.012, 0.026, x=0.0, z=0.0)
    shoulder_neck = _box(0.018, 0.022, 0.016, x=0.015)
    heel = _box(0.062, 0.030, 0.020, x=PRIMARY_START + 0.031)
    beam = _box(0.316, 0.026, 0.014, x=0.237)
    upper_rib = _box(0.170, 0.016, 0.007, x=0.205, z=0.010)
    yoke_plate_pos = _box(0.046, 0.010, 0.020, x=PRIMARY_ARM_LENGTH - 0.027, y=0.016)
    yoke_plate_neg = _box(0.046, 0.010, 0.020, x=PRIMARY_ARM_LENGTH - 0.027, y=-0.016)
    elbow_cheek_pos = _cyl_y(0.013, 0.008, x=PRIMARY_ARM_LENGTH, y=0.016)
    elbow_cheek_neg = _cyl_y(0.013, 0.008, x=PRIMARY_ARM_LENGTH, y=-0.016)

    arm_shape = shoulder_lug.union(shoulder_neck).union(heel)
    for solid in (
        beam,
        upper_rib,
        yoke_plate_pos,
        yoke_plate_neg,
        elbow_cheek_pos,
        elbow_cheek_neg,
    ):
        arm_shape = arm_shape.union(solid)
    return arm_shape


def _build_secondary_arm_shape():
    elbow_lug = _cyl_y(0.0105, 0.022, x=0.0, z=0.0)
    elbow_neck = _box(0.014, 0.020, 0.014, x=0.012)
    heel = _box(0.054, 0.026, 0.018, x=SECONDARY_START + 0.027)
    beam = _box(0.176, 0.022, 0.013, x=0.156)
    upper_rib = _box(0.108, 0.014, 0.006, x=0.154, z=0.009)
    yoke_plate_pos = _box(0.040, 0.009, 0.018, x=SECONDARY_ARM_LENGTH - 0.023, y=0.0145)
    yoke_plate_neg = _box(0.040, 0.009, 0.018, x=SECONDARY_ARM_LENGTH - 0.023, y=-0.0145)
    head_cheek_pos = _cyl_y(0.0115, 0.0075, x=SECONDARY_ARM_LENGTH, y=0.0145)
    head_cheek_neg = _cyl_y(0.0115, 0.0075, x=SECONDARY_ARM_LENGTH, y=-0.0145)

    arm_shape = elbow_lug.union(elbow_neck).union(heel)
    for solid in (
        beam,
        upper_rib,
        yoke_plate_pos,
        yoke_plate_neg,
        head_cheek_pos,
        head_cheek_neg,
    ):
        arm_shape = arm_shape.union(solid)
    return arm_shape


def _build_head_shape():
    head_lug = _cyl_y(0.0100, 0.020, x=0.0, z=0.0)
    head_neck = _box(0.014, 0.018, 0.014, x=0.012, z=-0.003)
    head_shell = (
        cq.Workplane("XY")
        .box(0.168, 0.098, 0.036)
        .edges()
        .fillet(0.012)
        .translate((0.108, 0.0, -0.020))
    )
    neck = _box(0.050, 0.036, 0.020, x=0.040, z=-0.010)
    rear_mount = _box(0.032, 0.028, 0.018, x=HEAD_START + 0.016, z=-0.004)

    head_shape = head_lug.union(head_neck).union(head_shell).union(neck).union(rear_mount)
    return head_shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cantilever_task_lamp")

    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    head_white = model.material("head_white", rgba=(0.88, 0.88, 0.84, 1.0))
    diffuser = model.material("diffuser", rgba=(0.96, 0.96, 0.90, 0.92))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        mesh_from_cadquery(_build_clamp_base_shape(), "clamp_base"),
        material=graphite,
        name="clamp_shell",
    )

    primary_arm = model.part("primary_arm")
    primary_arm.visual(
        mesh_from_cadquery(_build_primary_arm_shape(), "primary_arm"),
        material=aluminum,
        name="primary_shell",
    )

    secondary_arm = model.part("secondary_arm")
    secondary_arm.visual(
        mesh_from_cadquery(_build_secondary_arm_shape(), "secondary_arm"),
        material=aluminum,
        name="secondary_shell",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_build_head_shape(), "head"),
        material=head_white,
        name="head_shell",
    )
    head.visual(
        Box((0.122, 0.066, 0.006)),
        origin=Origin(xyz=(0.112, 0.0, -0.035)),
        material=diffuser,
        name="diffuser",
    )

    model.articulation(
        "clamp_to_primary",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=primary_arm,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-35.0),
            upper=math.radians(75.0),
            effort=30.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_arm,
        child=secondary_arm,
        origin=Origin(xyz=(PRIMARY_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-95.0),
            upper=math.radians(95.0),
            effort=18.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "secondary_to_head",
        ArticulationType.REVOLUTE,
        parent=secondary_arm,
        child=head,
        origin=Origin(xyz=(SECONDARY_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=math.radians(-65.0),
            upper=math.radians(45.0),
            effort=10.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clamp_base = object_model.get_part("clamp_base")
    primary_arm = object_model.get_part("primary_arm")
    secondary_arm = object_model.get_part("secondary_arm")
    head = object_model.get_part("head")
    shoulder = object_model.get_articulation("clamp_to_primary")
    elbow = object_model.get_articulation("primary_to_secondary")
    tilt = object_model.get_articulation("secondary_to_head")

    ctx.allow_overlap(
        clamp_base,
        primary_arm,
        reason="The shoulder hinge is represented as a nested clevis-and-lug joint without modeling the through-pin bore.",
    )
    ctx.allow_overlap(
        primary_arm,
        secondary_arm,
        reason="The elbow hinge uses interleaved hinge-lug proxies to keep the cantilever arm visually connected at the pivot.",
    )
    ctx.allow_overlap(
        secondary_arm,
        head,
        reason="The head tilt bracket is simplified as a clevis-and-lug hinge proxy at the lamp head pivot.",
    )

    ctx.expect_origin_gap(
        head,
        clamp_base,
        axis="x",
        min_gap=0.64,
        name="lamp head projects forward from the clamp tower",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({shoulder: math.radians(55.0), elbow: math.radians(10.0)}):
        raised_head_pos = ctx.part_world_position(head)
    ctx.check(
        "primary arm raises the working reach",
        rest_head_pos is not None
        and raised_head_pos is not None
        and raised_head_pos[2] > rest_head_pos[2] + 0.26,
        details=f"rest={rest_head_pos}, raised={raised_head_pos}",
    )

    with ctx.pose({elbow: math.radians(65.0)}):
        folded_head_pos = ctx.part_world_position(head)
    ctx.check(
        "secondary arm folds upward at the elbow",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[2] > rest_head_pos[2] + 0.20,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    def visual_center_z(part_name: str, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    level_diffuser_z = visual_center_z("head", "diffuser")
    with ctx.pose({tilt: math.radians(-45.0)}):
        down_diffuser_z = visual_center_z("head", "diffuser")
    ctx.check(
        "lamp head tilts downward from the end bracket",
        level_diffuser_z is not None
        and down_diffuser_z is not None
        and down_diffuser_z < level_diffuser_z - 0.03,
        details=f"level_z={level_diffuser_z}, down_z={down_diffuser_z}",
    )

    return ctx.report()


object_model = build_object_model()
