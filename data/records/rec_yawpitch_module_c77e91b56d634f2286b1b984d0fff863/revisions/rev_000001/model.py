from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


PEDESTAL_FLANGE_RADIUS = 0.19
PEDESTAL_FLANGE_THICKNESS = 0.04
PEDESTAL_COLUMN_HEIGHT = 0.22
PEDESTAL_COLUMN_RADIUS_BOTTOM = 0.09
PEDESTAL_COLUMN_RADIUS_TOP = 0.075
PEDESTAL_COLLAR_HEIGHT = 0.04
PEDESTAL_COLLAR_RADIUS = 0.085
PEDESTAL_HEIGHT = (
    PEDESTAL_FLANGE_THICKNESS + PEDESTAL_COLUMN_HEIGHT + PEDESTAL_COLLAR_HEIGHT
)

YAW_DRUM_RADIUS = 0.11
YAW_DRUM_HEIGHT = 0.08
TURNTABLE_RADIUS = 0.16
TURNTABLE_THICKNESS = 0.022
CHEEK_SIZE_X = 0.08
CHEEK_SIZE_Y = 0.04
CHEEK_SIZE_Z = 0.17
CHEEK_CENTER_Y = 0.13
CHEEK_CENTER_Z = YAW_DRUM_HEIGHT + CHEEK_SIZE_Z / 2.0
BACK_WEB_SIZE_X = 0.075
BACK_WEB_SIZE_Y = 0.19
BACK_WEB_SIZE_Z = 0.11
BACK_WEB_CENTER_X = -0.047
BACK_WEB_CENTER_Z = 0.135
PITCH_AXIS_HEIGHT = 0.18
BEARING_BORE_RADIUS = 0.022

TRUNNION_SHAFT_RADIUS = 0.016
TRUNNION_SHAFT_LENGTH = 0.302
CHEEK_OUTER_Y = CHEEK_CENTER_Y + CHEEK_SIZE_Y / 2.0
SIDE_PLATE_SIZE = (0.22, 0.022, 0.09)
SIDE_PLATE_CENTER_X = 0.11
SIDE_PLATE_CENTER_Y = CHEEK_OUTER_Y + 0.0115
FRONT_BRIDGE_SIZE = (0.055, 0.323, 0.06)
FRONT_BRIDGE_CENTER = (0.2475, 0.0, 0.0)
POD_SIZE = (0.12, 0.11, 0.09)
POD_CENTER = (0.3325, 0.0, 0.0)
NOSE_RADIUS = 0.03
NOSE_CENTER_X = 0.405


def make_pedestal():
    flange = cq.Workplane("XY").circle(PEDESTAL_FLANGE_RADIUS).extrude(
        PEDESTAL_FLANGE_THICKNESS
    )
    column = (
        cq.Workplane("XY")
        .circle(PEDESTAL_COLUMN_RADIUS_BOTTOM)
        .workplane(offset=PEDESTAL_COLUMN_HEIGHT)
        .circle(PEDESTAL_COLUMN_RADIUS_TOP)
        .loft(combine=True)
        .translate((0.0, 0.0, PEDESTAL_FLANGE_THICKNESS))
    )
    collar = (
        cq.Workplane("XY")
        .circle(PEDESTAL_COLLAR_RADIUS)
        .extrude(PEDESTAL_COLLAR_HEIGHT)
        .translate((0.0, 0.0, PEDESTAL_FLANGE_THICKNESS + PEDESTAL_COLUMN_HEIGHT))
    )
    return flange.union(column).union(collar)


def make_yaw_base():
    lower_drum = cq.Workplane("XY").circle(YAW_DRUM_RADIUS).extrude(YAW_DRUM_HEIGHT)
    turntable = (
        cq.Workplane("XY")
        .circle(TURNTABLE_RADIUS)
        .extrude(TURNTABLE_THICKNESS)
        .translate((0.0, 0.0, YAW_DRUM_HEIGHT))
    )
    left_cheek = cq.Workplane("XY").box(
        CHEEK_SIZE_X, CHEEK_SIZE_Y, CHEEK_SIZE_Z
    ).translate((0.0, CHEEK_CENTER_Y, CHEEK_CENTER_Z))
    right_cheek = cq.Workplane("XY").box(
        CHEEK_SIZE_X, CHEEK_SIZE_Y, CHEEK_SIZE_Z
    ).translate((0.0, -CHEEK_CENTER_Y, CHEEK_CENTER_Z))
    back_web = cq.Workplane("XY").box(
        BACK_WEB_SIZE_X, BACK_WEB_SIZE_Y, BACK_WEB_SIZE_Z
    ).translate((BACK_WEB_CENTER_X, 0.0, BACK_WEB_CENTER_Z))

    bore = (
        cq.Workplane("XZ")
        .center(0.0, PITCH_AXIS_HEIGHT)
        .circle(BEARING_BORE_RADIUS)
        .extrude(0.20, both=True)
    )

    return lower_drum.union(turntable).union(left_cheek).union(right_cheek).union(
        back_web
    ).cut(bore)


def make_pitch_yoke():
    shaft = (
        cq.Workplane("XZ")
        .circle(TRUNNION_SHAFT_RADIUS)
        .extrude(TRUNNION_SHAFT_LENGTH / 2.0, both=True)
    )
    left_plate = cq.Workplane("XY").box(*SIDE_PLATE_SIZE).translate(
        (SIDE_PLATE_CENTER_X, SIDE_PLATE_CENTER_Y, 0.0)
    )
    right_plate = cq.Workplane("XY").box(*SIDE_PLATE_SIZE).translate(
        (SIDE_PLATE_CENTER_X, -SIDE_PLATE_CENTER_Y, 0.0)
    )
    front_bridge = cq.Workplane("XY").box(*FRONT_BRIDGE_SIZE).translate(
        FRONT_BRIDGE_CENTER
    )
    pod = (
        cq.Workplane("XY")
        .box(*POD_SIZE)
        .edges("|X")
        .fillet(0.012)
        .translate(POD_CENTER)
    )
    nose = cq.Workplane("XY").sphere(NOSE_RADIUS).translate((NOSE_CENTER_X, 0.0, 0.0))

    return (
        shaft.union(left_plate)
        .union(right_plate)
        .union(front_bridge)
        .union(pod)
        .union(nose)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_mounted_yaw_pitch_module")

    pedestal_gray = model.material("pedestal_gray", color=(0.24, 0.25, 0.27, 1.0))
    base_gray = model.material("base_gray", color=(0.32, 0.34, 0.37, 1.0))
    head_gray = model.material("head_gray", color=(0.52, 0.54, 0.57, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        mesh_from_cadquery(make_pedestal(), "pedestal_shell"),
        material=pedestal_gray,
        name="pedestal_shell",
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        mesh_from_cadquery(make_yaw_base(), "yaw_base_shell"),
        material=base_gray,
        name="yaw_base_shell",
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        mesh_from_cadquery(make_pitch_yoke(), "pitch_yoke_shell"),
        material=head_gray,
        name="pitch_yoke_shell",
    )

    model.articulation(
        "pedestal_to_yaw",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_base,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=-2.5,
            upper=2.5,
        ),
    )

    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_HEIGHT)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.12,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("pedestal_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        pitch_yoke,
        reason=(
            "The pitch yoke is retained by the authored pitch articulation with a "
            "small trunnion-bearing running clearance, so support is mechanical "
            "rather than exact zero-gap contact."
        ),
    )

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

    yaw_limits = yaw_joint.motion_limits
    pitch_limits = pitch_joint.motion_limits
    ctx.check(
        "yaw axis is vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"got yaw axis {yaw_joint.axis}",
    )
    ctx.check(
        "pitch axis is horizontal",
        pitch_joint.axis == (0.0, -1.0, 0.0),
        details=f"got pitch axis {pitch_joint.axis}",
    )
    ctx.check(
        "yaw range is broad",
        yaw_limits is not None
        and yaw_limits.lower is not None
        and yaw_limits.upper is not None
        and (yaw_limits.upper - yaw_limits.lower) >= 4.5,
        details=f"yaw limits were {yaw_limits}",
    )
    ctx.check(
        "pitch range supports upward tilt",
        pitch_limits is not None
        and pitch_limits.lower is not None
        and pitch_limits.upper is not None
        and pitch_limits.lower <= 0.0
        and pitch_limits.upper >= 0.8,
        details=f"pitch limits were {pitch_limits}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0}):
        ctx.expect_gap(
            yaw_base,
            pedestal,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw base seats on pedestal",
        )
        ctx.expect_overlap(
            yaw_base,
            pedestal,
            axes="xy",
            min_overlap=0.18,
            name="yaw base stays centered over pedestal",
        )
        ctx.expect_overlap(
            pitch_yoke,
            yaw_base,
            axes="yz",
            min_overlap=0.08,
            name="pitch yoke is carried by the yaw head",
        )

    closed_aabb = ctx.part_element_world_aabb(pitch_yoke, elem="pitch_yoke_shell")
    closed_center_z = (
        None if closed_aabb is None else 0.5 * (closed_aabb[0][2] + closed_aabb[1][2])
    )
    with ctx.pose({pitch_joint: 0.8}):
        raised_aabb = ctx.part_element_world_aabb(pitch_yoke, elem="pitch_yoke_shell")
        raised_center_z = (
            None
            if raised_aabb is None
            else 0.5 * (raised_aabb[0][2] + raised_aabb[1][2])
        )
        ctx.fail_if_parts_overlap_in_current_pose(
            name="no overlaps with the pitch yoke elevated"
        )

    ctx.check(
        "positive pitch lifts the carried head",
        closed_center_z is not None
        and raised_center_z is not None
        and raised_center_z > closed_center_z + 0.04,
        details=(
            f"closed center z={closed_center_z}, elevated center z={raised_center_z}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
