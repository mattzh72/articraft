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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.220
BASE_WIDTH = 0.180
BASE_THICKNESS = 0.016

TOWER_LENGTH = 0.084
TOWER_WIDTH = 0.070
TOWER_HEIGHT = 0.118

SHOULDER_LENGTH = 0.114
SHOULDER_WIDTH = 0.094
SHOULDER_HEIGHT = 0.028

CAP_RADIUS = 0.046
CAP_HEIGHT = 0.014
CAP_OVERLAP = 0.0005
SUPPORT_TOP_Z = BASE_THICKNESS + TOWER_HEIGHT + SHOULDER_HEIGHT
YAW_JOINT_Z = SUPPORT_TOP_Z + CAP_HEIGHT - CAP_OVERLAP

TURNTABLE_RADIUS = 0.054
TURNTABLE_HEIGHT = 0.016
TURNTABLE_GAP = 0.0

PITCH_AXIS_Z = 0.070
EAR_CENTER_Y = 0.049
EAR_THICKNESS = 0.012
EAR_WIDTH_X = 0.028
EAR_BOTTOM_Z = 0.018
EAR_HEIGHT = 0.076
PITCH_BORE_RADIUS = 0.0108

CRADLE_LENGTH = 0.044
CRADLE_OUTER_RADIUS = 0.036
CRADLE_INNER_RADIUS = 0.030
CRADLE_PAD_X = 0.020
CRADLE_PAD_Y = 0.010
CRADLE_PAD_Z = 0.028
CRADLE_PAD_CENTER_Y = 0.0355
TRUNNION_RADIUS = 0.0092
TRUNNION_LENGTH = 0.016
TRUNNION_CENTER_Y = 0.047
TRUNNION_COLLAR_RADIUS = 0.0155
TRUNNION_COLLAR_LENGTH = 0.004
TRUNNION_COLLAR_CENTER_Y = 0.057

ROLL_BODY_RADIUS = 0.0272
ROLL_BODY_LENGTH = 0.040
ROLL_REAR_RADIUS = 0.023
ROLL_REAR_LENGTH = 0.009
ROLL_NOSE_RADIUS = 0.020
ROLL_NOSE_LENGTH = 0.024
ROLL_NOSE_CENTER_X = 0.032
ROLL_REAR_CENTER_X = -0.0245
ROLL_FLANGE_RADIUS = 0.0335
ROLL_FLANGE_LENGTH = 0.004
ROLL_FRONT_FLANGE_CENTER_X = (CRADLE_LENGTH / 2.0) + (ROLL_FLANGE_LENGTH / 2.0)
ROLL_REAR_FLANGE_CENTER_X = -ROLL_FRONT_FLANGE_CENTER_X
ROLL_KEY_SIZE = (0.010, 0.008, 0.010)
ROLL_KEY_CENTER = (0.035, 0.0, 0.018)


def _make_tower_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_THICKNESS,
        centered=(True, True, False),
    )
    tower = cq.Workplane("XY").box(
        TOWER_LENGTH,
        TOWER_WIDTH,
        TOWER_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BASE_THICKNESS))
    shoulder = cq.Workplane("XY").box(
        SHOULDER_LENGTH,
        SHOULDER_WIDTH,
        SHOULDER_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BASE_THICKNESS + TOWER_HEIGHT))
    front_rib = cq.Workplane("XY").box(
        0.108,
        0.018,
        0.062,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BASE_THICKNESS + 0.010))
    side_web_left = cq.Workplane("XY").box(
        0.064,
        0.016,
        0.084,
        centered=(True, True, False),
    ).translate((0.0, 0.030, BASE_THICKNESS + 0.012))
    side_web_right = side_web_left.translate((0.0, -0.060, 0.0))
    return base.union(tower).union(shoulder).union(front_rib).union(side_web_left).union(side_web_right)


def _make_yaw_stage_shape() -> cq.Workplane:
    lower_bridge = cq.Workplane("XY").box(
        0.058,
        0.086,
        0.014,
        centered=(True, True, False),
    ).translate((0.0, 0.0, 0.010))
    hub = cq.Workplane("XY").box(
        0.030,
        0.040,
        0.010,
        centered=(True, True, False),
    ).translate((0.0, 0.0, 0.020))
    ear_left = cq.Workplane("XY").box(
        EAR_WIDTH_X,
        EAR_THICKNESS,
        EAR_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, EAR_CENTER_Y, EAR_BOTTOM_Z))
    ear_right = ear_left.translate((0.0, -2.0 * EAR_CENTER_Y, 0.0))
    clevis = lower_bridge.union(hub).union(ear_left).union(ear_right)
    pitch_bore = (
        cq.Workplane("XZ")
        .moveTo(0.0, PITCH_AXIS_Z)
        .circle(PITCH_BORE_RADIUS)
        .extrude(0.140, both=True)
    )
    return clevis.cut(pitch_bore)


def _make_pitch_cradle_shape() -> cq.Workplane:
    ring = (
        cq.Workplane("YZ")
        .circle(CRADLE_OUTER_RADIUS)
        .circle(CRADLE_INNER_RADIUS)
        .extrude(CRADLE_LENGTH, both=True)
    )
    left_pad = cq.Workplane("XY").box(
        CRADLE_PAD_X,
        CRADLE_PAD_Y,
        CRADLE_PAD_Z,
        centered=(True, True, True),
    ).translate((0.0, CRADLE_PAD_CENTER_Y, 0.0))
    right_pad = left_pad.translate((0.0, -2.0 * CRADLE_PAD_CENTER_Y, 0.0))
    return ring.union(left_pad).union(right_pad)


def _make_roll_body_shape() -> cq.Workplane:
    body = cq.Workplane("YZ").circle(ROLL_BODY_RADIUS).extrude(ROLL_BODY_LENGTH, both=True)
    rear_cap = (
        cq.Workplane("YZ")
        .circle(ROLL_REAR_RADIUS)
        .extrude(ROLL_REAR_LENGTH)
        .translate((ROLL_REAR_CENTER_X - (ROLL_REAR_LENGTH / 2.0), 0.0, 0.0))
    )
    return body.union(rear_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_wrist_roll_cartridge")

    model.material("tower_dark", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("stage_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("cradle_alloy", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("cartridge_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("nose_steel", rgba=(0.83, 0.85, 0.88, 1.0))
    model.material("indicator_red", rgba=(0.72, 0.18, 0.12, 1.0))

    tower = model.part("tower")
    tower.visual(
        mesh_from_cadquery(_make_tower_shape(), "tower_shell"),
        material="tower_dark",
        name="tower_shell",
    )
    tower.visual(
        Cylinder(radius=CAP_RADIUS, length=CAP_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, SUPPORT_TOP_Z + (CAP_HEIGHT / 2.0) - CAP_OVERLAP),
        ),
        material="stage_graphite",
        name="tower_cap",
    )

    yaw_stage = model.part("yaw_stage")
    yaw_stage.visual(
        mesh_from_cadquery(_make_yaw_stage_shape(), "yaw_stage_shell"),
        material="stage_graphite",
        name="yaw_stage_shell",
    )
    yaw_stage.visual(
        Cylinder(radius=TURNTABLE_RADIUS, length=TURNTABLE_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, TURNTABLE_GAP + (TURNTABLE_HEIGHT / 2.0)),
        ),
        material="stage_graphite",
        name="yaw_turntable",
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        mesh_from_cadquery(_make_pitch_cradle_shape(), "pitch_cradle_shell"),
        material="cradle_alloy",
        name="pitch_cradle_shell",
    )
    pitch_cradle.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, TRUNNION_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="cradle_alloy",
        name="left_trunnion",
    )
    pitch_cradle.visual(
        Cylinder(radius=TRUNNION_RADIUS, length=TRUNNION_LENGTH),
        origin=Origin(
            xyz=(0.0, -TRUNNION_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="cradle_alloy",
        name="right_trunnion",
    )
    pitch_cradle.visual(
        Cylinder(radius=TRUNNION_COLLAR_RADIUS, length=TRUNNION_COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, TRUNNION_COLLAR_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="cradle_alloy",
        name="left_trunnion_collar",
    )
    pitch_cradle.visual(
        Cylinder(radius=TRUNNION_COLLAR_RADIUS, length=TRUNNION_COLLAR_LENGTH),
        origin=Origin(
            xyz=(0.0, -TRUNNION_COLLAR_CENTER_Y, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material="cradle_alloy",
        name="right_trunnion_collar",
    )

    roll_cartridge = model.part("roll_cartridge")
    roll_cartridge.visual(
        mesh_from_cadquery(_make_roll_body_shape(), "roll_body"),
        material="cartridge_dark",
        name="roll_body",
    )
    roll_cartridge.visual(
        Cylinder(radius=ROLL_NOSE_RADIUS, length=ROLL_NOSE_LENGTH),
        origin=Origin(
            xyz=(ROLL_NOSE_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="nose_steel",
        name="roll_nose",
    )
    roll_cartridge.visual(
        Cylinder(radius=ROLL_FLANGE_RADIUS, length=ROLL_FLANGE_LENGTH),
        origin=Origin(
            xyz=(ROLL_FRONT_FLANGE_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="nose_steel",
        name="front_bearing_flange",
    )
    roll_cartridge.visual(
        Cylinder(radius=ROLL_FLANGE_RADIUS, length=ROLL_FLANGE_LENGTH),
        origin=Origin(
            xyz=(ROLL_REAR_FLANGE_CENTER_X, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="nose_steel",
        name="rear_bearing_flange",
    )
    roll_cartridge.visual(
        Box(ROLL_KEY_SIZE),
        origin=Origin(xyz=ROLL_KEY_CENTER),
        material="indicator_red",
        name="roll_key",
    )

    model.articulation(
        "tower_yaw",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=yaw_stage,
        origin=Origin(xyz=(0.0, 0.0, YAW_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.5,
            lower=-2.6,
            upper=2.6,
        ),
    )
    model.articulation(
        "yaw_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_stage,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, PITCH_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.6,
            lower=-1.2,
            upper=1.15,
        ),
    )
    model.articulation(
        "pitch_roll",
        ArticulationType.REVOLUTE,
        parent=pitch_cradle,
        child=roll_cartridge,
        origin=Origin(),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=4.0,
            lower=-2.9,
            upper=2.9,
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

    tower = object_model.get_part("tower")
    yaw_stage = object_model.get_part("yaw_stage")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_cartridge = object_model.get_part("roll_cartridge")

    yaw_joint = object_model.get_articulation("tower_yaw")
    pitch_joint = object_model.get_articulation("yaw_pitch")
    roll_joint = object_model.get_articulation("pitch_roll")

    ctx.check(
        "wrist parts are present",
        all(
            part is not None
            for part in (tower, yaw_stage, pitch_cradle, roll_cartridge)
        ),
    )
    ctx.check(
        "wrist joints are present",
        all(joint is not None for joint in (yaw_joint, pitch_joint, roll_joint)),
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.0}):
        ctx.expect_gap(
            yaw_stage,
            tower,
            axis="z",
            positive_elem="yaw_turntable",
            negative_elem="tower_cap",
            min_gap=0.0,
            max_gap=0.002,
            name="yaw turntable sits just above the tower cap",
        )
        ctx.expect_overlap(
            yaw_stage,
            tower,
            axes="xy",
            elem_a="yaw_turntable",
            elem_b="tower_cap",
            min_overlap=0.080,
            name="yaw turntable stays centered over the support cap",
        )
        ctx.expect_within(
            roll_cartridge,
            pitch_cradle,
            axes="yz",
            inner_elem="roll_body",
            outer_elem="pitch_cradle_shell",
            margin=0.006,
            name="roll body stays nested inside the pitch cradle",
        )
        ctx.expect_overlap(
            roll_cartridge,
            pitch_cradle,
            axes="x",
            elem_a="roll_body",
            elem_b="pitch_cradle_shell",
            min_overlap=0.035,
            name="roll body keeps retained engagement inside the cradle",
        )
        ctx.expect_origin_gap(
            pitch_cradle,
            yaw_stage,
            axis="z",
            min_gap=0.060,
            max_gap=0.080,
            name="pitch cradle axis rides above the yaw stage",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    rest_nose = _aabb_center(ctx.part_element_world_aabb(roll_cartridge, elem="roll_nose"))
    with ctx.pose({yaw_joint: 0.9, pitch_joint: 0.0, roll_joint: 0.0}):
        yawed_nose = _aabb_center(ctx.part_element_world_aabb(roll_cartridge, elem="roll_nose"))
    ctx.check(
        "positive yaw swings the cartridge nose toward +Y",
        rest_nose is not None
        and yawed_nose is not None
        and yawed_nose[1] > rest_nose[1] + 0.020,
        details=f"rest={rest_nose}, yawed={yawed_nose}",
    )

    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.65, roll_joint: 0.0}):
        pitched_nose = _aabb_center(ctx.part_element_world_aabb(roll_cartridge, elem="roll_nose"))
    ctx.check(
        "positive pitch lifts the cartridge nose",
        rest_nose is not None
        and pitched_nose is not None
        and pitched_nose[2] > rest_nose[2] + 0.018,
        details=f"rest={rest_nose}, pitched={pitched_nose}",
    )

    rest_key = _aabb_center(ctx.part_element_world_aabb(roll_cartridge, elem="roll_key"))
    with ctx.pose({yaw_joint: 0.0, pitch_joint: 0.0, roll_joint: 0.75}):
        rolled_key = _aabb_center(ctx.part_element_world_aabb(roll_cartridge, elem="roll_key"))
    ctx.check(
        "positive roll swings the keyed feature toward +Y",
        rest_key is not None
        and rolled_key is not None
        and rolled_key[1] > rest_key[1] + 0.010,
        details=f"rest={rest_key}, rolled={rolled_key}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
