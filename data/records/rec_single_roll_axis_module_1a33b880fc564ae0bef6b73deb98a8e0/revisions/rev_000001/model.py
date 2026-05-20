from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
)


FOOT_WIDTH = 0.052
FOOT_DEPTH = 0.094
FOOT_THICKNESS = 0.014

POST_WIDTH = 0.028
POST_DEPTH = 0.052
POST_HEIGHT = 0.114
POST_CENTER_X = 0.052
POST_CENTER_Z = 0.069

BRIDGE_WIDTH = 0.140
BRIDGE_DEPTH = 0.052
BRIDGE_THICKNESS = 0.026
BRIDGE_CENTER_Z = 0.137

SPINDLE_AXIS_Z = 0.098
SPINDLE_BODY_RADIUS = 0.018
SPINDLE_BODY_LENGTH = 0.050

BEARING_WIDTH = 0.012
BEARING_DEPTH = 0.030
BEARING_HEIGHT = 0.070
BEARING_CENTER_Z = 0.105
LEFT_BEARING_CENTER_X = -(SPINDLE_BODY_RADIUS + (BEARING_WIDTH / 2.0))
RIGHT_BEARING_CENTER_X = SPINDLE_BODY_RADIUS + (BEARING_WIDTH / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_supported_roll_spindle")

    model.material("frame_finish", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("spindle_finish", rgba=(0.74, 0.76, 0.80, 1.0))

    bridge_frame = model.part("bridge_frame")
    bridge_frame.visual(
        Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(-POST_CENTER_X, 0.0, FOOT_THICKNESS / 2.0)),
        material="frame_finish",
        name="left_foot",
    )
    bridge_frame.visual(
        Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(POST_CENTER_X, 0.0, FOOT_THICKNESS / 2.0)),
        material="frame_finish",
        name="right_foot",
    )
    bridge_frame.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(-POST_CENTER_X, 0.0, POST_CENTER_Z)),
        material="frame_finish",
        name="left_post",
    )
    bridge_frame.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(POST_CENTER_X, 0.0, POST_CENTER_Z)),
        material="frame_finish",
        name="right_post",
    )
    bridge_frame.visual(
        Box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BRIDGE_CENTER_Z)),
        material="frame_finish",
        name="top_bridge",
    )
    bridge_frame.visual(
        Box((BEARING_WIDTH, BEARING_DEPTH, BEARING_HEIGHT)),
        origin=Origin(xyz=(LEFT_BEARING_CENTER_X, 0.0, BEARING_CENTER_Z)),
        material="frame_finish",
        name="left_bearing",
    )
    bridge_frame.visual(
        Box((BEARING_WIDTH, BEARING_DEPTH, BEARING_HEIGHT)),
        origin=Origin(xyz=(RIGHT_BEARING_CENTER_X, 0.0, BEARING_CENTER_Z)),
        material="frame_finish",
        name="right_bearing",
    )
    bridge_frame.inertial = Inertial.from_geometry(
        Box((BRIDGE_WIDTH, FOOT_DEPTH, BRIDGE_CENTER_Z + (BRIDGE_THICKNESS / 2.0))),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.0, (BRIDGE_CENTER_Z + (BRIDGE_THICKNESS / 2.0)) / 2.0)),
    )

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(
        Cylinder(radius=SPINDLE_BODY_RADIUS, length=SPINDLE_BODY_LENGTH),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="spindle_finish",
        name="spindle_shell",
    )
    roll_spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=SPINDLE_BODY_RADIUS, length=SPINDLE_BODY_LENGTH),
        mass=0.55,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_spindle",
        ArticulationType.REVOLUTE,
        parent=bridge_frame,
        child=roll_spindle,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_AXIS_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=8.0,
            lower=-pi,
            upper=pi,
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

    bridge_frame = object_model.get_part("bridge_frame")
    roll_spindle = object_model.get_part("roll_spindle")
    spindle_joint = object_model.get_articulation("frame_to_spindle")
    left_bearing = bridge_frame.get_visual("left_bearing")
    right_bearing = bridge_frame.get_visual("right_bearing")

    limits = spindle_joint.motion_limits

    ctx.check(
        "bridge frame exists",
        bridge_frame is not None,
        details="Expected grounded bridge frame root part.",
    )
    ctx.check(
        "roll spindle exists",
        roll_spindle is not None,
        details="Expected rotating spindle child part.",
    )
    ctx.check(
        "spindle articulation connects frame to spindle",
        spindle_joint.parent == bridge_frame.name and spindle_joint.child == roll_spindle.name,
        details=f"parent={spindle_joint.parent}, child={spindle_joint.child}",
    )
    ctx.check(
        "spindle rolls about supported longitudinal y axis",
        tuple(spindle_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={spindle_joint.axis}",
    )
    ctx.check(
        "spindle joint uses broad revolute roll limits",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -3.0
        and limits.upper >= 3.0,
        details=f"limits={limits}",
    )

    ctx.expect_origin_distance(
        roll_spindle,
        bridge_frame,
        axes="x",
        max_dist=0.001,
        name="spindle axis stays centered between the bridge legs",
    )
    ctx.expect_origin_gap(
        roll_spindle,
        bridge_frame,
        axis="z",
        min_gap=0.09,
        max_gap=0.105,
        name="spindle axis sits high in the bridge opening",
    )
    ctx.expect_overlap(
        roll_spindle,
        bridge_frame,
        axes="xz",
        min_overlap=0.030,
        name="spindle remains projected inside its support span",
    )
    ctx.expect_contact(
        roll_spindle,
        bridge_frame,
        elem_b=left_bearing,
        name="spindle touches the left bridge support",
    )
    ctx.expect_contact(
        roll_spindle,
        bridge_frame,
        elem_b=right_bearing,
        name="spindle touches the right bridge support",
    )

    rest_pos = ctx.part_world_position(roll_spindle)
    with ctx.pose({spindle_joint: 1.2}):
        rolled_pos = ctx.part_world_position(roll_spindle)
        ctx.expect_contact(
            roll_spindle,
            bridge_frame,
            elem_b=left_bearing,
            name="rolled spindle stays seated on left support",
        )
        ctx.expect_contact(
            roll_spindle,
            bridge_frame,
            elem_b=right_bearing,
            name="rolled spindle stays seated on right support",
        )

    ctx.check(
        "spindle rotates in place about its supported axis",
        rest_pos is not None
        and rolled_pos is not None
        and max(abs(a - b) for a, b in zip(rest_pos, rolled_pos)) <= 1e-6,
        details=f"rest={rest_pos}, rolled={rolled_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
