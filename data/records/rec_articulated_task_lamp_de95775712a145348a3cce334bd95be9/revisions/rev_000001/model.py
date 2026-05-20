from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BASE_RADIUS = 0.165
BASE_THICKNESS = 0.032
STEM_RADIUS = 0.015
SHOULDER_Z = 0.905

LOWER_ARM_LENGTH = 0.420
LOWER_ARM_REST_ANGLE = math.radians(24.0)
UPPER_ARM_LENGTH = 0.360
UPPER_ARM_REST_ANGLE = math.radians(18.0)

LOWER_ARM_WIDTH = 0.018
LOWER_ARM_HEIGHT = 0.014
UPPER_ARM_WIDTH = 0.016
UPPER_ARM_HEIGHT = 0.012

PLATE_THICKNESS = 0.006
BASE_YOKE_GAP = 0.022
LOWER_YOKE_GAP = 0.020
UPPER_YOKE_GAP = 0.019

LOWER_BARREL_RADIUS = 0.010
LOWER_BARREL_LENGTH = 0.019
UPPER_BARREL_RADIUS = 0.009
UPPER_BARREL_LENGTH = 0.017
SHADE_BARREL_RADIUS = 0.0085
SHADE_BARREL_LENGTH = 0.016

SHADE_LENGTH = 0.200
SHADE_REAR_X = 0.026
SHADE_AXIS_Z = -0.026
SHADE_REAR_RADIUS = 0.037
SHADE_FRONT_RADIUS = 0.108
SHADE_WALL = 0.0025

SHOULDER_LIMITS = (-0.35, 1.05)
ELBOW_LIMITS = (-0.95, 0.85)
SHADE_LIMITS = (-0.80, 0.50)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def _cylinder_x(
    radius: float, length: float, center: tuple[float, float, float]
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((cx, cy, cz))
    )


def _cylinder_y(
    radius: float, length: float, center: tuple[float, float, float]
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((cx, cy, cz))
    )


def _cylinder_z(
    radius: float, length: float, center: tuple[float, float, float]
) -> cq.Workplane:
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((cx, cy, cz - (length * 0.5)))
    )


def _rotate_up(shape: cq.Workplane, angle_rad: float) -> cq.Workplane:
    return shape.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -math.degrees(angle_rad))


def _fuse_all(*pieces: cq.Workplane) -> cq.Workplane:
    fused = pieces[0].val()
    for piece in pieces[1:]:
        fused = fused.fuse(piece.val())
    return cq.Workplane(obj=fused)


def _tip_coords(length: float, angle_rad: float) -> tuple[float, float, float]:
    return (
        length * math.cos(angle_rad),
        0.0,
        length * math.sin(angle_rad),
    )


def _build_base_shape() -> cq.Workplane:
    base_disk = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    plinth = cq.Workplane("XY").circle(BASE_RADIUS * 0.72).extrude(0.010).translate(
        (0.0, 0.0, BASE_THICKNESS)
    )

    stem_height = SHOULDER_Z - BASE_THICKNESS - 0.045
    stem = _cylinder_z(
        STEM_RADIUS,
        stem_height,
        (0.0, 0.0, BASE_THICKNESS + (stem_height * 0.5)),
    )

    return _fuse_all(base_disk, plinth, stem)


def _build_shoulder_bracket_shape() -> cq.Workplane:
    shoulder_socket = _box(
        (0.026, 0.028, 0.050),
        (-0.034, 0.0, SHOULDER_Z - 0.034),
    )

    left_plate = _box(
        (0.018, PLATE_THICKNESS, 0.040),
        (-0.009, 0.0125, SHOULDER_Z),
    )
    right_plate = _box(
        (0.018, PLATE_THICKNESS, 0.040),
        (-0.009, -0.0125, SHOULDER_Z),
    )

    rear_gusset = _box((0.020, 0.022, 0.022), (-0.031, 0.0, SHOULDER_Z - 0.058))
    bridge = _box((0.018, 0.020, 0.028), (-0.020, 0.0, SHOULDER_Z - 0.038))
    left_brace = _box((0.018, 0.010, 0.020), (-0.011, 0.0105, SHOULDER_Z - 0.020))
    right_brace = _box((0.018, 0.010, 0.020), (-0.011, -0.0105, SHOULDER_Z - 0.020))

    return _fuse_all(
        shoulder_socket,
        left_plate,
        right_plate,
        rear_gusset,
        bridge,
        left_brace,
        right_brace,
    )


def _build_lower_arm_shape() -> cq.Workplane:
    barrel = _cylinder_y(LOWER_BARREL_RADIUS, LOWER_BARREL_LENGTH, (0.0, 0.0, 0.0))
    root_block = _box((0.030, LOWER_ARM_WIDTH, 0.018), (0.016, 0.0, 0.0))
    beam = _box(
        (LOWER_ARM_LENGTH - 0.038, LOWER_ARM_WIDTH, LOWER_ARM_HEIGHT),
        ((LOWER_ARM_LENGTH - 0.038) * 0.5, 0.0, 0.0),
    )
    elbow_block = _box((0.028, 0.022, 0.018), (LOWER_ARM_LENGTH - 0.024, 0.0, 0.0))
    left_plate = _box(
        (0.018, PLATE_THICKNESS, 0.032),
        (
            LOWER_ARM_LENGTH - 0.009,
            0.0115,
            0.0,
        ),
    )
    right_plate = _box(
        (0.018, PLATE_THICKNESS, 0.032),
        (
            LOWER_ARM_LENGTH - 0.009,
            -0.0115,
            0.0,
        ),
    )

    return _rotate_up(
        _fuse_all(barrel, root_block, beam, elbow_block, left_plate, right_plate),
        LOWER_ARM_REST_ANGLE,
    )


def _build_upper_arm_shape() -> cq.Workplane:
    barrel = _cylinder_y(UPPER_BARREL_RADIUS, UPPER_BARREL_LENGTH, (0.0, 0.0, 0.0))
    root_block = _box((0.028, UPPER_ARM_WIDTH, 0.016), (0.015, 0.0, 0.0))
    beam = _box(
        (UPPER_ARM_LENGTH - 0.038, UPPER_ARM_WIDTH, UPPER_ARM_HEIGHT),
        ((UPPER_ARM_LENGTH - 0.038) * 0.5, 0.0, 0.0),
    )
    tip_block = _box((0.040, 0.021, 0.016), (UPPER_ARM_LENGTH - 0.031, 0.0, 0.0))
    left_plate = _box(
        (0.016, 0.0055, 0.028),
        (
            UPPER_ARM_LENGTH - 0.008,
            0.01075,
            0.0,
        ),
    )
    right_plate = _box(
        (0.016, 0.0055, 0.028),
        (
            UPPER_ARM_LENGTH - 0.008,
            -0.01075,
            0.0,
        ),
    )

    return _rotate_up(
        _fuse_all(barrel, root_block, beam, tip_block, left_plate, right_plate),
        UPPER_ARM_REST_ANGLE,
    )


def _build_shade_shape() -> cq.Workplane:
    pivot_barrel = _cylinder_y(SHADE_BARREL_RADIUS, SHADE_BARREL_LENGTH, (0.0, 0.0, 0.0))
    support_spine = _box((0.028, 0.014, 0.010), (0.020, 0.0, -0.010))
    rear_collar = _cylinder_x(0.024, 0.028, (0.030, 0.0, SHADE_AXIS_Z))

    rear_hole_radius = 0.014
    rear_inner_x = SHADE_REAR_X + 0.006
    rear_inner_radius = SHADE_REAR_RADIUS - SHADE_WALL
    front_x = SHADE_REAR_X + SHADE_LENGTH
    front_inner_radius = SHADE_FRONT_RADIUS - SHADE_WALL

    shade_shell = (
        cq.Workplane("XZ")
        .moveTo(SHADE_REAR_X, SHADE_AXIS_Z + SHADE_REAR_RADIUS)
        .lineTo(front_x, SHADE_AXIS_Z + SHADE_FRONT_RADIUS)
        .lineTo(front_x, SHADE_AXIS_Z + front_inner_radius)
        .lineTo(rear_inner_x, SHADE_AXIS_Z + rear_inner_radius)
        .lineTo(SHADE_REAR_X, SHADE_AXIS_Z + rear_hole_radius)
        .close()
        .revolve(
            360.0,
            axisStart=(0.0, 0.0, SHADE_AXIS_Z),
            axisEnd=(1.0, 0.0, SHADE_AXIS_Z),
        )
    )

    return _fuse_all(pivot_barrel, support_spine, rear_collar, shade_shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_task_lamp")

    powder_black = model.material("powder_black", rgba=(0.13, 0.13, 0.15, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.78, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "lamp_base"),
        material=powder_black,
        name="base_shell",
    )

    shoulder_bracket = model.part("shoulder_bracket")
    shoulder_bracket.visual(
        mesh_from_cadquery(_build_shoulder_bracket_shape(), "shoulder_bracket"),
        material=powder_black,
        name="shoulder_bracket_shell",
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        mesh_from_cadquery(_build_lower_arm_shape(), "lower_arm"),
        material=powder_black,
        name="lower_arm_shell",
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_build_upper_arm_shape(), "upper_arm"),
        material=powder_black,
        name="upper_arm_shell",
    )

    shade = model.part("shade")
    shade.visual(
        mesh_from_cadquery(_build_shade_shape(), "lamp_shade"),
        material=satin_steel,
        name="shade_shell",
    )

    lower_tip = _tip_coords(LOWER_ARM_LENGTH, LOWER_ARM_REST_ANGLE)
    upper_tip = _tip_coords(UPPER_ARM_LENGTH, UPPER_ARM_REST_ANGLE)

    model.articulation(
        "base_to_shoulder_bracket",
        ArticulationType.FIXED,
        parent=base,
        child=shoulder_bracket,
        origin=Origin(),
    )
    model.articulation(
        "base_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=shoulder_bracket,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=SHOULDER_LIMITS[0],
            upper=SHOULDER_LIMITS[1],
        ),
    )
    model.articulation(
        "lower_arm_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=lower_tip),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=ELBOW_LIMITS[0],
            upper=ELBOW_LIMITS[1],
        ),
    )
    model.articulation(
        "upper_arm_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=upper_tip),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=SHADE_LIMITS[0],
            upper=SHADE_LIMITS[1],
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    shoulder_bracket = object_model.get_part("shoulder_bracket")
    shade = object_model.get_part("shade")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")

    shoulder = object_model.get_articulation("base_to_lower_arm")
    elbow = object_model.get_articulation("lower_arm_to_upper_arm")
    shade_tilt = object_model.get_articulation("upper_arm_to_shade")

    ctx.allow_overlap(
        base,
        shoulder_bracket,
        reason="The shoulder bracket is a fixed welded subassembly nested into the top of the stem rather than split by a visible seam.",
    )
    ctx.allow_overlap(
        shoulder_bracket,
        lower_arm,
        reason="The shoulder clevis is modeled as a tight solid bracket around the lower-arm pivot rather than with a separate hinge pin and bushings.",
    )
    ctx.allow_overlap(
        lower_arm,
        upper_arm,
        reason="The elbow hinge uses simplified solid cheek plates and a pivot barrel proxy in place of a separately modeled pin and clearanced bushings.",
    )
    ctx.allow_overlap(
        upper_arm,
        shade,
        reason="The shade yoke and pivot barrel are represented as a tight hinge proxy without a separately modeled through-pin.",
    )

    ctx.expect_origin_gap(
        shade,
        base,
        axis="z",
        min_gap=1.18,
        name="resting shade hinge sits at reading-corner floor-lamp height",
    )

    rest_pos = ctx.part_world_position(shade)
    with ctx.pose({shoulder: SHOULDER_LIMITS[1]}):
        shoulder_up_pos = ctx.part_world_position(shade)
    ctx.check(
        "shoulder hinge raises the lamp head",
        rest_pos is not None
        and shoulder_up_pos is not None
        and shoulder_up_pos[2] > rest_pos[2] + 0.18,
        details=f"rest={rest_pos}, shoulder_up={shoulder_up_pos}",
    )

    with ctx.pose({elbow: ELBOW_LIMITS[1]}):
        elbow_up_pos = ctx.part_world_position(shade)
    ctx.check(
        "elbow hinge raises the shade assembly",
        rest_pos is not None
        and elbow_up_pos is not None
        and elbow_up_pos[2] > rest_pos[2] + 0.09,
        details=f"rest={rest_pos}, elbow_up={elbow_up_pos}",
    )

    with ctx.pose({shade_tilt: SHADE_LIMITS[0]}):
        low_tilt_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_tilt: SHADE_LIMITS[1]}):
        high_tilt_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
    ctx.check(
        "shade tilt swings the nose upward",
        low_tilt_aabb is not None
        and high_tilt_aabb is not None
        and high_tilt_aabb[1][2] > low_tilt_aabb[1][2] + 0.08,
        details=f"tilt_low={low_tilt_aabb}, tilt_high={high_tilt_aabb}",
    )

    with ctx.pose(
        {
            shoulder: SHOULDER_LIMITS[0],
            elbow: ELBOW_LIMITS[0],
            shade_tilt: SHADE_LIMITS[0],
        }
    ):
        low_pose_aabb = ctx.part_element_world_aabb(shade, elem="shade_shell")
        ctx.check(
            "lowest aimed pose still keeps the shade above the floor",
            low_pose_aabb is not None and low_pose_aabb[0][2] > 0.32,
            details=f"shade_aabb={low_pose_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
