from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


BASE_LENGTH = 0.125
BASE_WIDTH = 0.090
BASE_THICKNESS = 0.012

SPINE_X = -0.036
SPINE_LENGTH = 0.030
SPINE_WIDTH = 0.048
SPINE_HEIGHT = 0.120

TOP_ARM_LENGTH = 0.078
TOP_ARM_WIDTH = 0.044
TOP_ARM_THICKNESS = 0.018
TOP_ARM_BOTTOM_Z = BASE_THICKNESS + SPINE_HEIGHT - 0.016

YAW_BEARING_RADIUS = 0.030
YAW_BEARING_HEIGHT = 0.012
YAW_ORIGIN_Z = TOP_ARM_BOTTOM_Z + TOP_ARM_THICKNESS + YAW_BEARING_HEIGHT

PITCH_ORIGIN_X = 0.052
PITCH_ORIGIN_Z = 0.056


def _make_rear_support() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )

    spine = (
        cq.Workplane("XY")
        .box(
            SPINE_LENGTH,
            SPINE_WIDTH,
            SPINE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((SPINE_X, 0.0, BASE_THICKNESS))
    )

    top_arm = (
        cq.Workplane("XY")
        .box(
            TOP_ARM_LENGTH,
            TOP_ARM_WIDTH,
            TOP_ARM_THICKNESS,
            centered=(True, True, False),
        )
        .translate((-0.010, 0.0, TOP_ARM_BOTTOM_Z))
    )

    yaw_bearing = (
        cq.Workplane("XY")
        .circle(YAW_BEARING_RADIUS)
        .extrude(YAW_BEARING_HEIGHT)
        .translate((0.0, 0.0, TOP_ARM_BOTTOM_Z + TOP_ARM_THICKNESS))
    )

    rear_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.050, BASE_THICKNESS),
                (-0.050, 0.090),
                (-0.016, 0.126),
                (0.006, 0.126),
                (0.006, 0.114),
                (-0.020, 0.100),
                (-0.040, BASE_THICKNESS),
            ]
        )
        .close()
        .extrude(0.016, both=True)
    )

    return base.union(spine).union(top_arm).union(yaw_bearing).union(rear_web)


def _make_yaw_stage() -> cq.Workplane:
    rotor = cq.Workplane("XY").circle(0.032).extrude(0.018)

    rear_column = (
        cq.Workplane("XY")
        .box(0.026, 0.028, 0.042, centered=(True, True, False))
        .translate((-0.014, 0.0, 0.018))
    )

    crossbeam = (
        cq.Workplane("XY")
        .box(0.018, 0.086, 0.014, centered=(True, True, False))
        .translate((-0.016, 0.0, 0.060))
    )

    side_lug_pos = (
        cq.Workplane("XY")
        .box(0.038, 0.012, 0.018, centered=(True, True, True))
        .translate((0.019, 0.038, PITCH_ORIGIN_Z))
    )
    side_lug_neg = (
        cq.Workplane("XY")
        .box(0.038, 0.012, 0.018, centered=(True, True, True))
        .translate((0.019, -0.038, PITCH_ORIGIN_Z))
    )

    ear_pos = (
        cq.Workplane("XZ")
        .circle(0.012)
        .extrude(0.006, both=True)
        .translate((PITCH_ORIGIN_X, 0.038, PITCH_ORIGIN_Z))
    )
    ear_neg = (
        cq.Workplane("XZ")
        .circle(0.012)
        .extrude(0.006, both=True)
        .translate((PITCH_ORIGIN_X, -0.038, PITCH_ORIGIN_Z))
    )

    return (
        rotor.union(rear_column)
        .union(crossbeam)
        .union(side_lug_pos)
        .union(side_lug_neg)
        .union(ear_pos)
        .union(ear_neg)
    )


def _make_pitch_frame() -> cq.Workplane:
    ring = (
        cq.Workplane("XZ")
        .rect(0.050, 0.080)
        .rect(0.034, 0.052)
        .extrude(0.014, both=True)
    )

    trunnion = cq.Workplane("XZ").circle(0.010).extrude(0.022, both=True)
    top_rib = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.010, centered=(True, True, True))
        .translate((0.0, 0.0, 0.018))
    )
    bottom_rib = (
        cq.Workplane("XY")
        .box(0.018, 0.014, 0.010, centered=(True, True, True))
        .translate((0.0, 0.0, -0.018))
    )
    housing = cq.Workplane("YZ").circle(0.013).extrude(0.008, both=True)
    bore = cq.Workplane("YZ").circle(0.0072).extrude(0.016, both=True)

    return ring.union(trunnion).union(top_rib).union(bottom_rib).union(housing).cut(bore)


def _make_roll_spindle_body() -> cq.Workplane:
    shaft = (
        cq.Workplane("YZ")
        .circle(0.006)
        .extrude(0.050)
        .translate((-0.010, 0.0, 0.0))
    )
    flange = (
        cq.Workplane("YZ")
        .circle(0.018)
        .extrude(0.012)
        .translate((0.028, 0.0, 0.0))
    )
    return shaft.union(flange)


def _make_pitch_collar() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(0.013)
        .circle(0.0072)
        .extrude(0.010, both=True)
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    min_pt, max_pt = aabb
    return tuple((min_pt[i] + max_pt[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_ypr_head")

    model.material("powder_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("dark_gray", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("anodized_silver", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("accent_orange", rgba=(0.84, 0.46, 0.14, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_make_rear_support(), "rear_support"),
        material="powder_black",
        name="support_shell",
    )
    rear_support.inertial = Inertial.from_geometry(
        Box((0.125, 0.090, 0.146)),
        mass=1.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.073)),
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        Cylinder(radius=0.032, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material="dark_gray",
        name="yaw_rotor",
    )
    yaw_stage.visual(
        Box((0.056, 0.012, 0.018)),
        origin=Origin(xyz=(0.014, 0.034, PITCH_ORIGIN_Z)),
        material="dark_gray",
        name="left_pitch_arm",
    )
    yaw_stage.visual(
        Box((0.056, 0.012, 0.018)),
        origin=Origin(xyz=(0.014, -0.034, PITCH_ORIGIN_Z)),
        material="dark_gray",
        name="right_pitch_arm",
    )
    yaw_stage.visual(
        Box((0.018, 0.086, 0.014)),
        origin=Origin(xyz=(-0.016, 0.0, 0.067)),
        material="dark_gray",
        name="rear_crossbeam",
    )
    yaw_stage.visual(
        Box((0.026, 0.028, 0.042)),
        origin=Origin(xyz=(-0.014, 0.0, 0.039)),
        material="dark_gray",
        name="rear_column",
    )
    yaw_stage.visual(
        mesh_from_cadquery(
            cq.Workplane("XZ")
            .circle(0.012)
            .extrude(0.007, both=True)
            .translate((PITCH_ORIGIN_X, 0.038, PITCH_ORIGIN_Z)),
            "yaw_left_ear",
        ),
        material="dark_gray",
        name="left_pitch_ear",
    )
    yaw_stage.visual(
        mesh_from_cadquery(
            cq.Workplane("XZ")
            .circle(0.012)
            .extrude(0.007, both=True)
            .translate((PITCH_ORIGIN_X, -0.038, PITCH_ORIGIN_Z)),
            "yaw_right_ear",
        ),
        material="dark_gray",
        name="right_pitch_ear",
    )
    yaw_stage.inertial = Inertial.from_geometry(
        Box((0.084, 0.092, 0.074)),
        mass=0.65,
        origin=Origin(xyz=(0.006, 0.0, 0.037)),
    )

    pitch_frame = model.part("pitch_frame")
    pitch_frame.visual(
        mesh_from_cadquery(_make_pitch_collar(), "pitch_collar"),
        material="anodized_silver",
        name="pitch_collar",
    )
    pitch_frame.visual(
        Box((0.010, 0.014, 0.076)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material="anodized_silver",
        name="left_frame_rail",
    )
    pitch_frame.visual(
        Box((0.010, 0.014, 0.076)),
        origin=Origin(xyz=(-0.018, 0.0, 0.0)),
        material="anodized_silver",
        name="right_frame_rail",
    )
    pitch_frame.visual(
        Box((0.044, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material="anodized_silver",
        name="top_bridge",
    )
    pitch_frame.visual(
        Box((0.044, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material="anodized_silver",
        name="bottom_bridge",
    )
    pitch_frame.visual(
        Box((0.012, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material="anodized_silver",
        name="upper_bearing_rib",
    )
    pitch_frame.visual(
        Box((0.012, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material="anodized_silver",
        name="lower_bearing_rib",
    )
    pitch_frame.visual(
        Box((0.010, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material="anodized_silver",
        name="left_trunnion_stub",
    )
    pitch_frame.visual(
        Box((0.010, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.017, 0.0)),
        material="anodized_silver",
        name="right_trunnion_stub",
    )
    pitch_frame.inertial = Inertial.from_geometry(
        Box((0.062, 0.050, 0.088)),
        mass=0.42,
        origin=Origin(),
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        mesh_from_cadquery(_make_roll_spindle_body(), "roll_spindle_body"),
        material="anodized_silver",
        name="spindle_body",
    )
    roll_spindle.visual(
        mesh_from_cadquery(
            cq.Workplane("YZ").circle(0.010).extrude(0.004).translate((0.010, 0.0, 0.0)),
            "roll_thrust_collar",
        ),
        material="anodized_silver",
        name="thrust_collar",
    )
    roll_spindle.visual(
        Box((0.010, 0.048, 0.028)),
        origin=Origin(xyz=(0.044, 0.0, 0.0)),
        material="dark_gray",
        name="mount_plate",
    )
    roll_spindle.visual(
        mesh_from_cadquery(
            cq.Workplane("YZ").circle(0.004).extrude(0.006, both=True).translate((0.034, 0.0, 0.018)),
            "roll_index_knob",
        ),
        material="accent_orange",
        name="index_knob",
    )
    roll_spindle.inertial = Inertial.from_geometry(
        Box((0.060, 0.048, 0.032)),
        mass=0.18,
        origin=Origin(xyz=(0.024, 0.0, 0.0)),
    )

    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=rear_support,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=18.0, velocity=1.4),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_frame,
        origin=Origin(xyz=(PITCH_ORIGIN_X, 0.0, PITCH_ORIGIN_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.9, upper=1.15, effort=12.0, velocity=1.6),
    )
    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=pitch_frame,
        child=roll_spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.6, upper=1.6, effort=6.0, velocity=2.2),
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

    rear_support = object_model.get_part("rear_support")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_frame = object_model.get_part("pitch_frame")
    roll_spindle = object_model.get_part("roll_spindle")

    yaw_joint = object_model.get_articulation("yaw_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")
    roll_joint = object_model.get_articulation("roll_joint")

    ctx.check(
        "joint axes match yaw pitch roll convention",
        yaw_joint.axis == (0.0, 0.0, 1.0)
        and pitch_joint.axis == (0.0, -1.0, 0.0)
        and roll_joint.axis == (1.0, 0.0, 0.0),
        details=f"yaw={yaw_joint.axis}, pitch={pitch_joint.axis}, roll={roll_joint.axis}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            rear_support,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="yaw stage seats on rear support bearing",
        )
        ctx.expect_within(
            pitch_frame,
            yaw_stage,
            axes="y",
            margin=0.0,
            name="pitch frame stays between yaw carrier ears",
        )
        ctx.expect_origin_distance(
            roll_spindle,
            pitch_frame,
            axes="yz",
            min_dist=0.0,
            max_dist=0.000001,
            name="roll spindle remains centered in the pitch frame",
        )
        rest_spindle_center = _aabb_center(ctx.part_world_aabb(roll_spindle))
        rest_knob_center = _aabb_center(
            ctx.part_element_world_aabb(roll_spindle, elem="index_knob")
        )

    with ctx.pose({yaw_joint: 0.65, pitch_joint: 0.0, roll_joint: 0.0}):
        yawed_spindle_center = _aabb_center(ctx.part_world_aabb(roll_spindle))

    ctx.check(
        "positive yaw swings the head toward +Y",
        rest_spindle_center is not None
        and yawed_spindle_center is not None
        and yawed_spindle_center[1] > rest_spindle_center[1] + 0.015,
        details=f"rest={rest_spindle_center}, yawed={yawed_spindle_center}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.55, roll_joint: 0.0}):
        pitched_spindle_center = _aabb_center(ctx.part_world_aabb(roll_spindle))

    ctx.check(
        "positive pitch raises the spindle assembly",
        rest_spindle_center is not None
        and pitched_spindle_center is not None
        and pitched_spindle_center[2] > rest_spindle_center[2] + 0.005,
        details=f"rest={rest_spindle_center}, pitched={pitched_spindle_center}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.90}):
        rolled_knob_center = _aabb_center(
            ctx.part_element_world_aabb(roll_spindle, elem="index_knob")
        )

    ctx.check(
        "positive roll moves the top index knob toward -Y",
        rest_knob_center is not None
        and rolled_knob_center is not None
        and rolled_knob_center[1] < rest_knob_center[1] - 0.010,
        details=f"rest={rest_knob_center}, rolled={rolled_knob_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
