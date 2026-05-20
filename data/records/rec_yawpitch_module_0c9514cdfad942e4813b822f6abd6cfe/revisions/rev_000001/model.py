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


BASE_WIDTH = 0.150
BASE_DEPTH = 0.110
BASE_THICKNESS = 0.012

PEDESTAL_WIDTH = 0.072
PEDESTAL_DEPTH = 0.056
PEDESTAL_HEIGHT = 0.028
PEDESTAL_Y = 0.014

BRIDGE_WIDTH = 0.116
BRIDGE_DEPTH = 0.022
BRIDGE_POST_WIDTH = 0.016
BRIDGE_HEIGHT = 0.158
BRIDGE_TOP_THICKNESS = 0.016
BRIDGE_Y = -0.043

YAW_Z = BASE_THICKNESS + PEDESTAL_HEIGHT
YAW_Y = PEDESTAL_Y

PITCH_Y = 0.004
PITCH_Z = 0.078


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_yaw_pitch_module")

    model.material("frame_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("machined_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("sensor_face", rgba=(0.17, 0.19, 0.22, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS)),
        material="frame_steel",
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        name="base_plate",
    )
    support_frame.visual(
        Box((PEDESTAL_WIDTH, PEDESTAL_DEPTH, PEDESTAL_HEIGHT)),
        material="frame_steel",
        origin=Origin(
            xyz=(0.0, PEDESTAL_Y, BASE_THICKNESS + PEDESTAL_HEIGHT * 0.5)
        ),
        name="pedestal",
    )
    support_frame.visual(
        Box((BRIDGE_POST_WIDTH, BRIDGE_DEPTH, BRIDGE_HEIGHT)),
        material="frame_steel",
        origin=Origin(
            xyz=(
                -(BRIDGE_WIDTH - BRIDGE_POST_WIDTH) * 0.5,
                BRIDGE_Y,
                BASE_THICKNESS + BRIDGE_HEIGHT * 0.5,
            )
        ),
        name="left_post",
    )
    support_frame.visual(
        Box((BRIDGE_POST_WIDTH, BRIDGE_DEPTH, BRIDGE_HEIGHT)),
        material="frame_steel",
        origin=Origin(
            xyz=(
                (BRIDGE_WIDTH - BRIDGE_POST_WIDTH) * 0.5,
                BRIDGE_Y,
                BASE_THICKNESS + BRIDGE_HEIGHT * 0.5,
            )
        ),
        name="right_post",
    )
    support_frame.visual(
        Box((BRIDGE_WIDTH, BRIDGE_DEPTH, BRIDGE_TOP_THICKNESS)),
        material="frame_steel",
        origin=Origin(
            xyz=(
                0.0,
                BRIDGE_Y,
                BASE_THICKNESS + BRIDGE_HEIGHT - BRIDGE_TOP_THICKNESS * 0.5,
            )
        ),
        name="top_bridge",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, 0.170)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    yaw_base = model.part("yaw_base")
    yaw_base.visual(
        Cylinder(radius=0.030, length=0.016),
        material="machined_gray",
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        name="turntable",
    )
    yaw_base.visual(
        Box((0.062, 0.050, 0.028)),
        material="machined_gray",
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        name="yaw_housing",
    )
    yaw_base.visual(
        Box((0.009, 0.024, 0.058)),
        material="machined_gray",
        origin=Origin(xyz=(-0.028, PITCH_Y, 0.069)),
        name="left_cheek",
    )
    yaw_base.visual(
        Box((0.009, 0.024, 0.058)),
        material="machined_gray",
        origin=Origin(xyz=(0.028, PITCH_Y, 0.069)),
        name="right_cheek",
    )
    yaw_base.visual(
        Box((0.047, 0.010, 0.018)),
        material="machined_gray",
        origin=Origin(xyz=(0.0, -0.010, 0.075)),
        name="rear_tie",
    )
    yaw_base.inertial = Inertial.from_geometry(
        Box((0.066, 0.060, 0.100)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(
        Box((0.036, 0.014, 0.020)),
        material="machined_gray",
        origin=Origin(xyz=(0.0, 0.007, 0.0)),
        name="hub",
    )
    pitch_yoke.visual(
        Box((0.008, 0.030, 0.032)),
        material="machined_gray",
        origin=Origin(xyz=(-0.014, 0.015, 0.0)),
        name="left_arm",
    )
    pitch_yoke.visual(
        Box((0.008, 0.030, 0.032)),
        material="machined_gray",
        origin=Origin(xyz=(0.014, 0.015, 0.0)),
        name="right_arm",
    )
    pitch_yoke.visual(
        Box((0.036, 0.008, 0.036)),
        material="machined_gray",
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        name="front_bridge",
    )
    pitch_yoke.visual(
        Box((0.007, 0.010, 0.010)),
        material="machined_gray",
        origin=Origin(xyz=(-0.020, 0.007, 0.0)),
        name="left_pivot_pad",
    )
    pitch_yoke.visual(
        Box((0.007, 0.010, 0.010)),
        material="machined_gray",
        origin=Origin(xyz=(0.020, 0.007, 0.0)),
        name="right_pivot_pad",
    )
    pitch_yoke.visual(
        Box((0.030, 0.008, 0.030)),
        material="sensor_face",
        origin=Origin(xyz=(0.0, 0.039, 0.0)),
        name="output_face",
    )
    pitch_yoke.inertial = Inertial.from_geometry(
        Box((0.046, 0.046, 0.038)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
    )

    model.articulation(
        "frame_to_yaw",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=yaw_base,
        origin=Origin(xyz=(0.0, YAW_Y, YAW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.5,
            lower=-5.0 * pi / 6.0,
            upper=5.0 * pi / 6.0,
        ),
    )
    model.articulation(
        "yaw_to_pitch",
        ArticulationType.REVOLUTE,
        parent=yaw_base,
        child=pitch_yoke,
        origin=Origin(xyz=(0.0, PITCH_Y, PITCH_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.8,
            lower=-0.55,
            upper=0.90,
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

    support_frame = object_model.get_part("support_frame")
    yaw_base = object_model.get_part("yaw_base")
    pitch_yoke = object_model.get_part("pitch_yoke")
    yaw_joint = object_model.get_articulation("frame_to_yaw")
    pitch_joint = object_model.get_articulation("yaw_to_pitch")

    ctx.check("support frame exists", support_frame is not None)
    ctx.check("yaw base exists", yaw_base is not None)
    ctx.check("pitch yoke exists", pitch_yoke is not None)

    ctx.check(
        "yaw joint uses vertical axis",
        yaw_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={yaw_joint.axis}",
    )
    ctx.check(
        "pitch joint uses horizontal axis",
        pitch_joint.axis == (1.0, 0.0, 0.0),
        details=f"axis={pitch_joint.axis}",
    )

    ctx.expect_origin_distance(
        yaw_base,
        support_frame,
        axes="x",
        max_dist=0.001,
        name="yaw base stays centered over the frame",
    )
    ctx.expect_origin_gap(
        yaw_base,
        support_frame,
        axis="z",
        min_gap=0.040,
        max_gap=0.040,
        name="yaw base sits on the raised frame pedestal",
    )
    ctx.expect_origin_gap(
        pitch_yoke,
        yaw_base,
        axis="z",
        min_gap=0.077,
        max_gap=0.079,
        name="pitch axis sits above the yaw platform",
    )
    ctx.expect_origin_distance(
        pitch_yoke,
        yaw_base,
        axes="y",
        min_dist=0.003,
        max_dist=0.005,
        name="pitch axis is slightly forward of the yaw center",
    )

    ctx.expect_within(
        pitch_yoke,
        yaw_base,
        axes="x",
        margin=0.0,
        name="pitch yoke remains nested laterally inside the yaw fork envelope",
    )

    rest_face = _aabb_center(ctx.part_element_world_aabb(pitch_yoke, elem="output_face"))
    with ctx.pose({yaw_joint: 0.90}):
        yawed_face = _aabb_center(ctx.part_element_world_aabb(pitch_yoke, elem="output_face"))
    ctx.check(
        "positive yaw swings the output face sideways",
        rest_face is not None
        and yawed_face is not None
        and yawed_face[0] < rest_face[0] - 0.020,
        details=f"rest={rest_face}, yawed={yawed_face}",
    )

    with ctx.pose({pitch_joint: 0.60}):
        pitched_face = _aabb_center(ctx.part_element_world_aabb(pitch_yoke, elem="output_face"))
    ctx.check(
        "positive pitch raises the output face",
        rest_face is not None
        and pitched_face is not None
        and pitched_face[2] > rest_face[2] + 0.015,
        details=f"rest={rest_face}, pitched={pitched_face}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
