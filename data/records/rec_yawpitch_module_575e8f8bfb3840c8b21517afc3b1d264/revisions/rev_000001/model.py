from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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
)


BACKPLATE_WIDTH = 0.180
BACKPLATE_HEIGHT = 0.260
BACKPLATE_THICKNESS = 0.012
BACKPLATE_CORNER_RADIUS = 0.012
MOUNT_HOLE_DIAMETER = 0.007
MOUNT_HOLE_Y = 0.055
MOUNT_HOLE_Z_OFFSET = 0.080

YAW_BASE_Z = 0.072
STANDOFF_LENGTH = 0.040
STANDOFF_WIDTH = 0.056
STANDOFF_HEIGHT = 0.034

YAW_PLATE_THICKNESS = 0.014
YAW_PLATE_WIDTH = 0.084
YAW_PLATE_HEIGHT = 0.146
YAW_PLATE_CENTER_Z = 0.080
YAW_BASE_BOSS_RADIUS = 0.022
YAW_BASE_BOSS_HEIGHT = 0.020
YAW_NECK_LENGTH = 0.020
YAW_NECK_WIDTH = 0.056
YAW_NECK_HEIGHT = 0.024

PITCH_AXIS_X = 0.060
PITCH_AXIS_Z = 0.106
YOKE_ARM_LENGTH = 0.060
YOKE_ARM_THICKNESS = 0.012
YOKE_ARM_HEIGHT = 0.050
YOKE_INNER_GAP = 0.020
YOKE_ARM_CENTER_Y = (YOKE_INNER_GAP + YOKE_ARM_THICKNESS) / 2.0
YOKE_EAR_RADIUS = 0.018

CENTER_PLATE_WIDTH_X = 0.045
CENTER_PLATE_THICKNESS_Y = 0.010
CENTER_PLATE_HEIGHT_Z = 0.062
CENTER_HUB_WIDTH_X = 0.016
CENTER_HUB_HEIGHT_Z = 0.020
CENTER_PLATE_OFFSET_X = 0.040
CENTER_PLATE_OFFSET_Z = 0.0
TRUNNION_RADIUS = 0.0065
TRUNNION_LENGTH = YOKE_INNER_GAP + 2.0 * YOKE_ARM_THICKNESS
YOKE_BORE_RADIUS = TRUNNION_RADIUS


def _make_backplate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .center(0.0, BACKPLATE_HEIGHT / 2.0)
        .rect(BACKPLATE_WIDTH, BACKPLATE_HEIGHT)
        .extrude(BACKPLATE_THICKNESS, both=True)
        .edges("|X")
        .fillet(BACKPLATE_CORNER_RADIUS)
    )
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-MOUNT_HOLE_Y, -MOUNT_HOLE_Z_OFFSET),
                (MOUNT_HOLE_Y, -MOUNT_HOLE_Z_OFFSET),
                (-MOUNT_HOLE_Y, MOUNT_HOLE_Z_OFFSET),
                (MOUNT_HOLE_Y, MOUNT_HOLE_Z_OFFSET),
            ]
        )
        .hole(MOUNT_HOLE_DIAMETER)
    )

    standoff = cq.Workplane("XY").box(
        STANDOFF_LENGTH,
        STANDOFF_WIDTH,
        STANDOFF_HEIGHT,
    ).translate((BACKPLATE_THICKNESS / 2.0 + STANDOFF_LENGTH / 2.0, 0.0, YAW_BASE_Z))

    gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (BACKPLATE_THICKNESS / 2.0, YAW_BASE_Z - 0.048),
                (BACKPLATE_THICKNESS / 2.0, YAW_BASE_Z - 0.010),
                (BACKPLATE_THICKNESS / 2.0 + STANDOFF_LENGTH * 0.62, YAW_BASE_Z - 0.010),
                (BACKPLATE_THICKNESS / 2.0 + STANDOFF_LENGTH * 0.20, YAW_BASE_Z - 0.048),
            ]
        )
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )

    yaw_clearance = (
        cq.Workplane("XY")
        .circle(YAW_BASE_BOSS_RADIUS + 0.0005)
        .extrude(STANDOFF_HEIGHT + 0.010)
        .translate(
            (
                BACKPLATE_THICKNESS / 2.0 + STANDOFF_LENGTH,
                0.0,
                YAW_BASE_Z - (STANDOFF_HEIGHT + 0.010) / 2.0,
            )
        )
    )

    return plate.union(standoff).union(gusset).cut(yaw_clearance)


def _make_yaw_yoke_shape() -> cq.Workplane:
    yaw_plate = (
        cq.Workplane("YZ")
        .center(0.0, YAW_PLATE_CENTER_Z)
        .rect(YAW_PLATE_WIDTH, YAW_PLATE_HEIGHT)
        .extrude(YAW_PLATE_THICKNESS)
        .edges("|X")
        .fillet(0.008)
    )

    base_boss = (
        cq.Workplane("XY")
        .center(YAW_BASE_BOSS_RADIUS, 0.0)
        .circle(YAW_BASE_BOSS_RADIUS)
        .extrude(YAW_BASE_BOSS_HEIGHT)
        .translate((0.0, 0.0, -YAW_BASE_BOSS_HEIGHT / 2.0))
    )

    neck = cq.Workplane("XY").box(
        YAW_NECK_LENGTH,
        YAW_NECK_WIDTH,
        YAW_NECK_HEIGHT,
    ).translate((YAW_NECK_LENGTH / 2.0, 0.0, YAW_NECK_HEIGHT / 2.0))

    arm_blank = (
        cq.Workplane("XZ")
        .center(YAW_PLATE_THICKNESS + YOKE_ARM_LENGTH / 2.0, PITCH_AXIS_Z)
        .rect(YOKE_ARM_LENGTH, YOKE_ARM_HEIGHT)
        .extrude(YOKE_ARM_THICKNESS)
        .union(
            cq.Workplane("XZ")
            .center(PITCH_AXIS_X, PITCH_AXIS_Z)
            .circle(YOKE_EAR_RADIUS)
            .extrude(YOKE_ARM_THICKNESS)
        )
        .edges("|Y")
        .fillet(0.005)
    )
    bore = (
        cq.Workplane("XZ")
        .center(PITCH_AXIS_X, PITCH_AXIS_Z)
        .circle(YOKE_BORE_RADIUS)
        .extrude(YOKE_ARM_THICKNESS + 0.002)
        .translate((0.0, -0.001, 0.0))
    )
    arm_positive = arm_blank.cut(bore).translate((0.0, YOKE_INNER_GAP / 2.0, 0.0))
    arm_negative = arm_blank.cut(bore).translate((0.0, -(YOKE_INNER_GAP / 2.0 + YOKE_ARM_THICKNESS), 0.0))

    return (
        yaw_plate.union(base_boss)
        .union(neck)
        .union(arm_positive)
        .union(arm_negative)
    )


def _make_center_plate_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("XZ")
        .center(CENTER_HUB_WIDTH_X / 2.0, 0.0)
        .rect(CENTER_HUB_WIDTH_X, CENTER_HUB_HEIGHT_Z)
        .extrude(CENTER_PLATE_THICKNESS_Y, both=True)
        .edges("|Y")
        .fillet(0.0025)
    )
    plate = (
        cq.Workplane("XZ")
        .center(CENTER_PLATE_OFFSET_X, CENTER_PLATE_OFFSET_Z)
        .rect(CENTER_PLATE_WIDTH_X, CENTER_PLATE_HEIGHT_Z)
        .extrude(CENTER_PLATE_THICKNESS_Y, both=True)
        .edges("|Y")
        .fillet(0.004)
    )
    trunnion = (
        cq.Workplane("XZ")
        .circle(TRUNNION_RADIUS)
        .extrude(TRUNNION_LENGTH, both=True)
    )
    return hub.union(plate).union(trunnion)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((lo + hi) / 2.0 for lo, hi in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_yaw_pitch_bracket")

    backplate_finish = model.material("backplate_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    yoke_finish = model.material("yoke_finish", rgba=(0.68, 0.71, 0.74, 1.0))
    center_finish = model.material("center_finish", rgba=(0.82, 0.84, 0.86, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_make_backplate_shape(), "backplate"),
        material=backplate_finish,
        name="backplate_shell",
    )

    yaw_yoke = model.part("yaw_yoke")
    yaw_yoke.visual(
        Box((YAW_PLATE_THICKNESS, YAW_PLATE_WIDTH, YAW_PLATE_HEIGHT)),
        origin=Origin(xyz=(YAW_PLATE_THICKNESS / 2.0, 0.0, YAW_PLATE_CENTER_Z)),
        material=yoke_finish,
        name="yaw_yoke_shell",
    )
    yaw_yoke.visual(
        Cylinder(radius=YAW_BASE_BOSS_RADIUS, length=YAW_BASE_BOSS_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=yoke_finish,
        name="yaw_base_boss",
    )
    yaw_yoke.visual(
        Box((YAW_NECK_LENGTH, YAW_NECK_WIDTH, YAW_NECK_HEIGHT)),
        origin=Origin(xyz=(YAW_NECK_LENGTH / 2.0, 0.0, YAW_NECK_HEIGHT / 2.0)),
        material=yoke_finish,
        name="yaw_neck",
    )
    arm_origin_x = YAW_PLATE_THICKNESS + (YOKE_ARM_LENGTH + 0.002) / 2.0 - 0.001
    yaw_yoke.visual(
        Box((YOKE_ARM_LENGTH + 0.002, YOKE_ARM_THICKNESS, YOKE_ARM_HEIGHT)),
        origin=Origin(xyz=(arm_origin_x, YOKE_ARM_CENTER_Y, PITCH_AXIS_Z)),
        material=yoke_finish,
        name="left_yoke_arm",
    )
    yaw_yoke.visual(
        Box((YOKE_ARM_LENGTH + 0.002, YOKE_ARM_THICKNESS, YOKE_ARM_HEIGHT)),
        origin=Origin(xyz=(arm_origin_x, -YOKE_ARM_CENTER_Y, PITCH_AXIS_Z)),
        material=yoke_finish,
        name="right_yoke_arm",
    )

    center_plate = model.part("center_plate")
    center_plate.visual(
        Box((0.042, CENTER_PLATE_THICKNESS_Y, CENTER_PLATE_HEIGHT_Z)),
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        material=center_finish,
        name="center_plate_shell",
    )
    center_plate.visual(
        Box((0.024, CENTER_PLATE_THICKNESS_Y, 0.022)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=center_finish,
        name="center_hub",
    )
    center_plate.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=0.005),
        origin=Origin(
            xyz=(0.0, CENTER_PLATE_THICKNESS_Y / 2.0 + 0.0025, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=center_finish,
        name="left_trunnion",
    )
    center_plate.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=0.005),
        origin=Origin(
            xyz=(0.0, -(CENTER_PLATE_THICKNESS_Y / 2.0 + 0.0025), 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=center_finish,
        name="right_trunnion",
    )

    model.articulation(
        "backplate_to_yaw",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=yaw_yoke,
        origin=Origin(xyz=(BACKPLATE_THICKNESS / 2.0 + STANDOFF_LENGTH, 0.0, YAW_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "yoke_to_center",
        ArticulationType.REVOLUTE,
        parent=yaw_yoke,
        child=center_plate,
        origin=Origin(xyz=(PITCH_AXIS_X, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.6,
            lower=-1.15,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    yaw_yoke = object_model.get_part("yaw_yoke")
    center_plate = object_model.get_part("center_plate")
    yaw_joint = object_model.get_articulation("backplate_to_yaw")
    pitch_joint = object_model.get_articulation("yoke_to_center")

    backplate_shell = backplate.get_visual("backplate_shell")
    yaw_yoke_shell = yaw_yoke.get_visual("yaw_yoke_shell")
    center_plate_shell = center_plate.get_visual("center_plate_shell")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(backplate, yaw_yoke, name="yaw_stage_is_mounted_to_backplate")
    ctx.expect_contact(center_plate, yaw_yoke, name="center_plate_is_supported_by_pitch_yoke")
    ctx.expect_gap(
        center_plate,
        backplate,
        axis="x",
        min_gap=0.020,
        name="center_plate_sits_forward_of_backplate",
    )

    rest_aabb = ctx.part_element_world_aabb(center_plate, elem=center_plate_shell)
    rest_center = _aabb_center(rest_aabb)
    rest_max_x = None if rest_aabb is None else rest_aabb[1][0]

    with ctx.pose({yaw_joint: 0.9}):
        yawed_aabb = ctx.part_element_world_aabb(center_plate, elem=center_plate_shell)
        yawed_center = _aabb_center(yawed_aabb)
    ctx.check(
        "positive_yaw_swings_center_plate_sideways",
        rest_center is not None
        and yawed_center is not None
        and yawed_center[1] > rest_center[1] + 0.04,
        details=f"rest_center={rest_center}, yawed_center={yawed_center}",
    )

    with ctx.pose({pitch_joint: 0.75}):
        pitched_aabb = ctx.part_element_world_aabb(center_plate, elem=center_plate_shell)
        pitched_max_z = None if pitched_aabb is None else pitched_aabb[1][2]
    ctx.check(
        "positive_pitch_lifts_plate",
        rest_aabb is not None
        and pitched_max_z is not None
        and pitched_max_z > rest_aabb[1][2] + 0.015,
        details=f"rest_max_z={None if rest_aabb is None else rest_aabb[1][2]}, pitched_max_z={pitched_max_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
