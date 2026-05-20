from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

import cadquery as cq

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
    mesh_from_cadquery,
)


PLATE_LENGTH = 0.210
PLATE_WIDTH = 0.130
PLATE_THICKNESS = 0.014

SUPPORT_X = -0.082
SUPPORT_WIDTH = 0.060
SUPPORT_THICKNESS = 0.024
SUPPORT_HEIGHT = 0.146

YAW_Z = 0.142
ROOF_LENGTH = 0.106
ROOF_THICKNESS = 0.022
ROOF_CENTER_X = -0.015
ROOF_CENTER_Z = 0.161
ROOF_TOP = ROOF_CENTER_Z + ROOF_THICKNESS / 2.0
ROOF_BOTTOM = ROOF_CENTER_Z - ROOF_THICKNESS / 2.0

BEARING_PAD_RADIUS = 0.028
BEARING_PAD_HEIGHT = 0.016
YAW_ROTOR_RADIUS = 0.027
YAW_ROTOR_HEIGHT = 0.012
YAW_DRUM_RADIUS = 0.031
YAW_DRUM_HEIGHT = 0.038

PITCH_X = 0.072
PITCH_Z = -0.054
CHEEK_CENTER_Y = 0.043
CHEEK_THICKNESS = 0.012
CHEEK_LENGTH_X = 0.032
CHEEK_HEIGHT_Z = 0.082
CHEEK_CENTER_Z = -0.056
YOKE_PITCH_HOLE_RADIUS = 0.0115

FRAME_WIDTH_Y = 0.050
FRAME_CENTER_X = 0.046
FRAME_OUTER_X = 0.086
FRAME_OUTER_Z = 0.074
FRAME_INNER_X = 0.058
FRAME_INNER_Z = 0.046
TRUNNION_SHAFT_RADIUS = 0.0085
TRUNNION_COLLAR_RADIUS = 0.0135
TRUNNION_SHAFT_LENGTH = 0.029
TRUNNION_SHAFT_CENTER_Y = 0.0355
TRUNNION_COLLAR_THICKNESS = 0.006
TRUNNION_COLLAR_CENTER_Y = 0.052

YAW_LIMIT = 2.4
PITCH_LOWER = -1.00
PITCH_UPPER = 1.05


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_z(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1], center[2] - length / 2.0))
    )


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]):
    return (
        cq.Workplane("XZ")
        .center(center[0], center[2])
        .circle(radius)
        .extrude(length)
        .translate((0.0, center[1] + length / 2.0, 0.0))
    )


def _make_ground_support_shape():
    side_profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.105, 0.0),
                (0.060, 0.0),
                (0.060, PLATE_THICKNESS),
                (-0.020, PLATE_THICKNESS),
                (-0.020, ROOF_BOTTOM),
                (0.038, ROOF_BOTTOM),
                (0.038, ROOF_TOP),
                (-0.094, ROOF_TOP),
                (-0.094, PLATE_THICKNESS),
                (-0.105, PLATE_THICKNESS),
            ]
        )
        .close()
        .extrude(SUPPORT_WIDTH)
        .translate((0.0, -SUPPORT_WIDTH / 2.0, 0.0))
    )
    plate = _box(
        (PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS),
        (0.0, 0.0, PLATE_THICKNESS / 2.0),
    )
    return side_profile.union(plate).combine()


def _make_yaw_cartridge_shape():
    drum = _cylinder_z(
        YAW_DRUM_RADIUS,
        YAW_DRUM_HEIGHT,
        (0.0, 0.0, -0.029),
    )
    neck = _box((0.038, 0.052, 0.024), (0.024, 0.0, -0.028))
    bridge = _box((0.044, 0.098, 0.024), (0.050, 0.0, -0.020))
    left_cheek = _box(
        (CHEEK_LENGTH_X, CHEEK_THICKNESS, CHEEK_HEIGHT_Z),
        (PITCH_X, CHEEK_CENTER_Y, CHEEK_CENTER_Z),
    )
    right_cheek = _box(
        (CHEEK_LENGTH_X, CHEEK_THICKNESS, CHEEK_HEIGHT_Z),
        (PITCH_X, -CHEEK_CENTER_Y, CHEEK_CENTER_Z),
    )

    yoke = drum.union(neck).union(bridge).union(left_cheek).union(right_cheek)
    left_hole = _cylinder_y(
        YOKE_PITCH_HOLE_RADIUS,
        CHEEK_THICKNESS + 0.008,
        (PITCH_X, CHEEK_CENTER_Y, PITCH_Z),
    )
    right_hole = _cylinder_y(
        YOKE_PITCH_HOLE_RADIUS,
        CHEEK_THICKNESS + 0.008,
        (PITCH_X, -CHEEK_CENTER_Y, PITCH_Z),
    )
    return yoke.cut(left_hole).cut(right_hole)


def _make_pitch_frame_shell():
    return (
        cq.Workplane("XZ")
        .center(FRAME_CENTER_X, 0.0)
        .rect(FRAME_OUTER_X, FRAME_OUTER_Z)
        .rect(FRAME_INNER_X, FRAME_INNER_Z)
        .extrude(FRAME_WIDTH_Y)
        .translate((0.0, FRAME_WIDTH_Y / 2.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_pan_tilt_head")

    model.material("base_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("cartridge_dark", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("frame_silver", rgba=(0.74, 0.76, 0.79, 1.0))

    ground_support = model.part("ground_support")
    ground_support.visual(
        mesh_from_cadquery(_make_ground_support_shape(), "ground_support_shell"),
        material="base_paint",
        name="support_shell",
    )
    ground_support.inertial = Inertial.from_geometry(
        Box((0.210, 0.130, 0.180)),
        mass=4.6,
        origin=Origin(xyz=(-0.030, 0.0, 0.090)),
    )

    ground_support.visual(
        Cylinder(radius=BEARING_PAD_RADIUS, length=BEARING_PAD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, YAW_Z + BEARING_PAD_HEIGHT / 2.0)),
        material="base_paint",
        name="bearing_pad",
    )
    yaw_cartridge = model.part("yaw_cartridge")
    yaw_cartridge.visual(
        mesh_from_cadquery(_make_yaw_cartridge_shape(), "yaw_cartridge_shell"),
        material="cartridge_dark",
        name="yoke_shell",
    )
    yaw_cartridge.visual(
        Cylinder(radius=YAW_ROTOR_RADIUS, length=YAW_ROTOR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, -YAW_ROTOR_HEIGHT / 2.0)),
        material="cartridge_dark",
        name="yaw_rotor",
    )
    yaw_cartridge.inertial = Inertial.from_geometry(
        Box((0.120, 0.105, 0.110)),
        mass=1.4,
        origin=Origin(xyz=(0.038, 0.0, -0.028)),
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_frame_shell(), "pitch_frame_shell"),
        material="frame_silver",
        name="frame_shell",
    )
    pitch_frame.visual(
        Cylinder(radius=TRUNNION_SHAFT_RADIUS, length=TRUNNION_SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, TRUNNION_SHAFT_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="frame_silver",
        name="left_shaft",
    )
    pitch_frame.visual(
        Cylinder(radius=TRUNNION_COLLAR_RADIUS, length=TRUNNION_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(0.0, TRUNNION_COLLAR_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="frame_silver",
        name="left_collar",
    )
    pitch_frame.visual(
        Cylinder(radius=TRUNNION_SHAFT_RADIUS, length=TRUNNION_SHAFT_LENGTH),
        origin=Origin(
            xyz=(0.0, -TRUNNION_SHAFT_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="frame_silver",
        name="right_shaft",
    )
    pitch_frame.visual(
        Cylinder(radius=TRUNNION_COLLAR_RADIUS, length=TRUNNION_COLLAR_THICKNESS),
        origin=Origin(
            xyz=(0.0, -TRUNNION_COLLAR_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="frame_silver",
        name="right_collar",
    )
    pitch_frame.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_yaw",
        ArticulationType.REVOLUTE,
        parent=ground_support,
        child=yaw_cartridge,
        origin=Origin(xyz=(0.0, 0.0, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_cartridge,
        child=pitch_frame,
        origin=Origin(xyz=(PITCH_X, 0.0, PITCH_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.4,
            lower=PITCH_LOWER,
            upper=PITCH_UPPER,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    ground_support = object_model.get_part("ground_support")
    yaw_cartridge = object_model.get_part("yaw_cartridge")
    pitch_frame = object_model.get_part("pitch_frame")
    yaw_joint = object_model.get_articulation("base_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check(
        "yaw joint is vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint is horizontal",
        pitch_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )
    ctx.expect_contact(
        ground_support,
        yaw_cartridge,
        name="yaw rotor bears against the support pad",
    )
    ctx.expect_contact(
        yaw_cartridge,
        pitch_frame,
        name="pitch trunnions bear against the yoke cheeks",
    )
    ctx.expect_within(
        pitch_frame,
        yaw_cartridge,
        axes="y",
        inner_elem="frame_shell",
        outer_elem="yoke_shell",
        margin=0.0,
        name="inner pitch frame stays between the side cheeks",
    )

    rest_pos = ctx.part_world_position(pitch_frame)
    with ctx.pose({yaw_joint: 1.2}):
        yawed_pos = ctx.part_world_position(pitch_frame)
    ctx.check(
        "positive yaw swings the pitch frame sideways",
        rest_pos is not None
        and yawed_pos is not None
        and yawed_pos[1] > rest_pos[1] + 0.05
        and isclose(yawed_pos[2], rest_pos[2], abs_tol=1e-6),
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )

    rest_aabb = ctx.part_world_aabb(pitch_frame)
    with ctx.pose({pitch_joint: PITCH_UPPER}):
        raised_aabb = ctx.part_world_aabb(pitch_frame)
    ctx.check(
        "positive pitch raises the front frame envelope",
        rest_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > rest_aabb[1][2] + 0.02,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
