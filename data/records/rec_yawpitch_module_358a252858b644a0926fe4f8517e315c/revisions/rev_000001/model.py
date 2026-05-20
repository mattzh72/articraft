from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


TOP_PLATE_X = 0.220
TOP_PLATE_Y = 0.160
TOP_PLATE_T = 0.012

SUPPORT_BLOCK_X = 0.090
SUPPORT_BLOCK_Y = 0.070
SUPPORT_BLOCK_T = 0.020

SLEEVE_OUTER_R = 0.038
SLEEVE_INNER_R = 0.025
SLEEVE_LEN = 0.024
YAW_ORIGIN_Z = -0.056

SPINDLE_R = 0.022
SPINDLE_LEN = 0.008
ROTOR_R = 0.036
ROTOR_T = 0.008
PITCH_ORIGIN_Z = -0.078

CLEVIS_PLATE_T = 0.006
CLEVIS_HALF_SPAN_Y = 0.017
CLEVIS_PLATE_X = 0.010
CLEVIS_PLATE_Z = 0.058

TRUNNION_R = 0.009
TRUNNION_LEN = 0.040
CRADLE_ARM_X = 0.060
CRADLE_ARM_T = 0.006
CRADLE_ARM_Z = 0.110


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _make_top_support() -> cq.Workplane:
    plate = cq.Workplane("XY").box(TOP_PLATE_X, TOP_PLATE_Y, TOP_PLATE_T).translate(
        (0.0, 0.0, -TOP_PLATE_T * 0.5)
    )
    mount_block = cq.Workplane("XY").box(
        SUPPORT_BLOCK_X, SUPPORT_BLOCK_Y, SUPPORT_BLOCK_T
    ).translate((0.0, 0.0, -(TOP_PLATE_T + SUPPORT_BLOCK_T * 0.5)))
    rib_offset_y = SUPPORT_BLOCK_Y * 0.5 - 0.004
    rib = cq.Workplane("XY").box(0.070, 0.008, 0.030).translate((0.0, 0.0, -0.027))
    bearing_pod = (
        cq.Workplane("XY")
        .circle(0.034)
        .extrude(-SLEEVE_LEN)
        .translate((0.0, 0.0, -(TOP_PLATE_T + SUPPORT_BLOCK_T)))
    )
    hole_points = [
        (-0.082, -0.052),
        (-0.082, 0.052),
        (0.082, -0.052),
        (0.082, 0.052),
    ]
    holes = cq.Workplane("XY").pushPoints(hole_points).circle(0.005).extrude(-0.025)
    support = (
        plate.union(mount_block)
        .union(rib.translate((0.0, rib_offset_y, 0.0)))
        .union(rib.translate((0.0, -rib_offset_y, 0.0)))
        .union(bearing_pod)
        .cut(holes)
    )
    return support


def _make_yaw_stage() -> cq.Workplane:
    spindle = cq.Workplane("XY").circle(ROTOR_R).extrude(-SPINDLE_LEN)
    rotor = (
        cq.Workplane("XY")
        .circle(ROTOR_R)
        .extrude(-ROTOR_T)
        .translate((0.0, 0.0, -SPINDLE_LEN))
    )
    pod = cq.Workplane("XY").box(0.042, 0.044, 0.038).translate((0.0, 0.0, -0.035))
    clevis_front = cq.Workplane("XY").box(
        CLEVIS_PLATE_X, CLEVIS_PLATE_T, CLEVIS_PLATE_Z
    ).translate((0.0, CLEVIS_HALF_SPAN_Y, -0.083))
    clevis_rear = cq.Workplane("XY").box(
        CLEVIS_PLATE_X, CLEVIS_PLATE_T, CLEVIS_PLATE_Z
    ).translate((0.0, -CLEVIS_HALF_SPAN_Y, -0.083))

    yaw_stage = (
        spindle.union(rotor)
        .union(pod)
        .union(clevis_front)
        .union(clevis_rear)
    )
    return yaw_stage


def _make_pitch_cradle() -> cq.Workplane:
    lug = cq.Workplane("XY").box(0.016, 0.004, 0.018)
    bridge = cq.Workplane("XY").box(0.020, 0.044, 0.010).translate((0.010, 0.0, -0.010))
    arm = cq.Workplane("XY").box(0.054, 0.006, 0.096).translate((0.030, 0.0, -0.055))
    crossbar = cq.Workplane("XY").box(0.048, 0.040, 0.010).translate((0.036, 0.0, -0.106))
    saddle = cq.Workplane("XY").box(0.070, 0.040, 0.006).translate((0.052, 0.0, -0.074))

    cradle = (
        lug.translate((0.0, 0.022, 0.0))
        .union(lug.translate((0.0, -0.022, 0.0)))
        .union(bridge)
        .union(arm.translate((0.0, 0.019, 0.0)))
        .union(arm.translate((0.0, -0.019, 0.0)))
        .union(crossbar)
        .union(saddle)
    )
    return cradle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_yaw_pitch_fixture")

    model.material("support_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("stage_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    model.material("cradle_light", rgba=(0.70, 0.72, 0.75, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(_make_top_support(), "top_support"),
        material="support_dark",
        name="support_shell",
    )
    top_support.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_X, TOP_PLATE_Y, 0.110)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage(), "yaw_stage"),
        material="stage_gray",
        name="yaw_shell",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.120, 0.072, 0.138)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, -0.069)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_make_pitch_cradle(), "pitch_cradle"),
        material="cradle_light",
        name="cradle_shell",
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.074, 0.044, 0.112)),
        mass=0.9,
        origin=Origin(xyz=(0.037, 0.0, -0.056)),
    )

    model.articulation(
        "support_to_yaw",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-2.4,
            upper=2.4,
        ),
    )

    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_ORIGIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=-1.1,
            upper=1.1,
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

    top_support = object_model.get_part("top_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    yaw_joint = object_model.get_articulation("support_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.allow_overlap(
        yaw_stage,
        pitch_cradle,
        reason=(
            "The pitch pivot is represented as a captured clevis-and-lug interface, "
            "so the trunnion region is intentionally simplified as a solid nested fit."
        ),
    )

    ctx.check(
        "yaw axis is vertical",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch axis is horizontal",
        pitch_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )

    support_pos = ctx.part_world_position(top_support)
    yaw_pos = ctx.part_world_position(yaw_stage)
    cradle_pos = ctx.part_world_position(pitch_cradle)
    ctx.check(
        "assembly hangs in descending order",
        support_pos is not None
        and yaw_pos is not None
        and cradle_pos is not None
        and support_pos[2] > yaw_pos[2] > cradle_pos[2],
        details=f"support={support_pos}, yaw={yaw_pos}, cradle={cradle_pos}",
    )

    ctx.expect_overlap(
        top_support,
        yaw_stage,
        axes="xy",
        min_overlap=0.040,
        name="yaw stage stays centered under the top support",
    )
    ctx.expect_overlap(
        yaw_stage,
        pitch_cradle,
        axes="y",
        min_overlap=0.040,
        name="pitch cradle remains captured between the yaw-stage cheeks",
    )

    rest_center = _aabb_center(ctx.part_world_aabb(pitch_cradle))
    with ctx.pose({pitch_joint: 0.65}):
        pitched_center = _aabb_center(ctx.part_world_aabb(pitch_cradle))
    ctx.check(
        "pitch joint swings the cradle in the fore-aft plane",
        rest_center is not None
        and pitched_center is not None
        and abs(pitched_center[0] - rest_center[0]) > 0.010
        and abs(pitched_center[2] - rest_center[2]) > 0.010,
        details=f"rest={rest_center}, pitched={pitched_center}",
    )

    with ctx.pose({yaw_joint: 0.80}):
        yawed_center = _aabb_center(ctx.part_world_aabb(pitch_cradle))
    ctx.check(
        "yaw joint carries the cradle around the vertical axis",
        rest_center is not None
        and yawed_center is not None
        and abs(yawed_center[1] - rest_center[1]) > 0.015,
        details=f"rest={rest_center}, yawed={yawed_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
