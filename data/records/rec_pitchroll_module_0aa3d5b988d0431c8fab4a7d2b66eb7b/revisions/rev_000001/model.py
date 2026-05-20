from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    Inertial,
    TestContext,
    TestReport,
)


TOP_SUPPORT_WIDTH = 0.34
TOP_SUPPORT_DEPTH = 0.13
TOP_SUPPORT_THICKNESS = 0.018
SUPPORT_AXIS_TO_TOP_UNDERSIDE = 0.09
SUPPORT_CHEEK_THICKNESS = 0.018
SUPPORT_CHEEK_DEPTH = 0.065
SUPPORT_CHEEK_HEIGHT = 0.108
SUPPORT_CHEEK_CENTER_Z = 0.036
SUPPORT_INNER_FACE_X = 0.143

ROLL_FRAME_WIDTH = 0.262
ROLL_FRAME_PAD_THICKNESS = 0.012
ROLL_FRAME_PAD_HEIGHT = 0.05
ROLL_FRAME_PAD_DEPTH = 0.05
ROLL_FRAME_PAD_CENTER_X = 0.137
ROLL_FRAME_TOP_BAR_DEPTH = 0.11
ROLL_FRAME_TOP_BAR_THICKNESS = 0.024
ROLL_FRAME_FRONT_REAR_THICKNESS = 0.012
ROLL_FRAME_FRONT_REAR_CENTER_Y = 0.049
ROLL_FRAME_FRONT_REAR_HEIGHT = 0.112
ROLL_FRAME_FRONT_REAR_CENTER_Z = -0.08
ROLL_FRAME_LOWER_BLOCK_WIDTH = 0.024
ROLL_FRAME_LOWER_BLOCK_DEPTH = 0.086
ROLL_FRAME_LOWER_BLOCK_THICKNESS = 0.024
ROLL_FRAME_LOWER_BLOCK_CENTER_X = 0.12
ROLL_FRAME_LOWER_BLOCK_CENTER_Z = -0.148

PITCH_AXIS_DROP = 0.072
PITCH_CRADLE_PLATE_WIDTH = 0.15
PITCH_CRADLE_PLATE_THICKNESS = 0.01
PITCH_CRADLE_PLATE_HEIGHT = 0.14
PITCH_CRADLE_PLATE_CENTER_Y = 0.035
PITCH_CRADLE_PLATE_CENTER_Z = -0.06
PITCH_PIVOT_PAD_WIDTH = 0.05
PITCH_PIVOT_PAD_THICKNESS = 0.006
PITCH_PIVOT_PAD_HEIGHT = 0.036
PITCH_PIVOT_PAD_CENTER_Y = 0.04
PITCH_BOTTOM_RAIL_WIDTH = 0.15
PITCH_BOTTOM_RAIL_DEPTH = 0.07
PITCH_BOTTOM_RAIL_THICKNESS = 0.012
PITCH_BOTTOM_RAIL_CENTER_Z = -0.132
TOOL_FACE_CARRIER_THICKNESS = 0.018
TOOL_FACE_CARRIER_DEPTH = 0.07
TOOL_FACE_CARRIER_HEIGHT = 0.10
TOOL_FACE_CARRIER_CENTER_X = 0.033
TOOL_FACE_CENTER_Z = -0.06
TOOL_FACE_THICKNESS = 0.012
TOOL_FACE_WIDTH = 0.07
TOOL_FACE_HEIGHT = 0.14
TOOL_FACE_CENTER_X = 0.048


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_pitch_roll_fixture")

    model.material("support_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("frame_graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("cradle_silver", rgba=(0.71, 0.74, 0.77, 1.0))
    model.material("tool_face_light", rgba=(0.84, 0.86, 0.88, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        Box((TOP_SUPPORT_WIDTH, TOP_SUPPORT_DEPTH, TOP_SUPPORT_THICKNESS)),
        material="support_black",
        name="support_body",
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_AXIS_TO_TOP_UNDERSIDE + TOP_SUPPORT_THICKNESS / 2.0)),
    )
    top_support.visual(
        Box((SUPPORT_CHEEK_THICKNESS, SUPPORT_CHEEK_DEPTH, SUPPORT_CHEEK_HEIGHT)),
        material="support_black",
        name="left_support_cheek",
        origin=Origin(xyz=(-SUPPORT_INNER_FACE_X - SUPPORT_CHEEK_THICKNESS / 2.0, 0.0, SUPPORT_CHEEK_CENTER_Z)),
    )
    top_support.visual(
        Box((SUPPORT_CHEEK_THICKNESS, SUPPORT_CHEEK_DEPTH, SUPPORT_CHEEK_HEIGHT)),
        material="support_black",
        name="right_support_cheek",
        origin=Origin(xyz=(SUPPORT_INNER_FACE_X + SUPPORT_CHEEK_THICKNESS / 2.0, 0.0, SUPPORT_CHEEK_CENTER_Z)),
    )
    top_support.inertial = Inertial.from_geometry(
        Box((TOP_SUPPORT_WIDTH, TOP_SUPPORT_DEPTH, 0.126)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    roll_frame = model.part("roll_frame")
    roll_frame.visual(
        Box((ROLL_FRAME_WIDTH, ROLL_FRAME_TOP_BAR_DEPTH, ROLL_FRAME_TOP_BAR_THICKNESS)),
        material="frame_graphite",
        name="roll_frame_body",
        origin=Origin(xyz=(0.0, 0.0, -ROLL_FRAME_TOP_BAR_THICKNESS / 2.0)),
    )
    roll_frame.visual(
        Box((ROLL_FRAME_WIDTH, ROLL_FRAME_FRONT_REAR_THICKNESS, ROLL_FRAME_FRONT_REAR_HEIGHT)),
        material="frame_graphite",
        name="front_roll_cheek",
        origin=Origin(xyz=(0.0, ROLL_FRAME_FRONT_REAR_CENTER_Y, ROLL_FRAME_FRONT_REAR_CENTER_Z)),
    )
    roll_frame.visual(
        Box((ROLL_FRAME_WIDTH, ROLL_FRAME_FRONT_REAR_THICKNESS, ROLL_FRAME_FRONT_REAR_HEIGHT)),
        material="frame_graphite",
        name="rear_roll_cheek",
        origin=Origin(xyz=(0.0, -ROLL_FRAME_FRONT_REAR_CENTER_Y, ROLL_FRAME_FRONT_REAR_CENTER_Z)),
    )
    roll_frame.visual(
        Box((ROLL_FRAME_LOWER_BLOCK_WIDTH, ROLL_FRAME_LOWER_BLOCK_DEPTH, ROLL_FRAME_LOWER_BLOCK_THICKNESS)),
        material="frame_graphite",
        name="left_lower_tie",
        origin=Origin(xyz=(-ROLL_FRAME_LOWER_BLOCK_CENTER_X, 0.0, ROLL_FRAME_LOWER_BLOCK_CENTER_Z)),
    )
    roll_frame.visual(
        Box((ROLL_FRAME_LOWER_BLOCK_WIDTH, ROLL_FRAME_LOWER_BLOCK_DEPTH, ROLL_FRAME_LOWER_BLOCK_THICKNESS)),
        material="frame_graphite",
        name="right_lower_tie",
        origin=Origin(xyz=(ROLL_FRAME_LOWER_BLOCK_CENTER_X, 0.0, ROLL_FRAME_LOWER_BLOCK_CENTER_Z)),
    )
    roll_frame.visual(
        Box((ROLL_FRAME_PAD_THICKNESS, ROLL_FRAME_PAD_DEPTH, ROLL_FRAME_PAD_HEIGHT)),
        material="frame_graphite",
        name="left_roll_pad",
        origin=Origin(xyz=(-ROLL_FRAME_PAD_CENTER_X, 0.0, 0.0)),
    )
    roll_frame.visual(
        Box((ROLL_FRAME_PAD_THICKNESS, ROLL_FRAME_PAD_DEPTH, ROLL_FRAME_PAD_HEIGHT)),
        material="frame_graphite",
        name="right_roll_pad",
        origin=Origin(xyz=(ROLL_FRAME_PAD_CENTER_X, 0.0, 0.0)),
    )
    roll_frame.inertial = Inertial.from_geometry(
        Box((0.286, 0.11, 0.17)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
    )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Box((PITCH_CRADLE_PLATE_WIDTH, PITCH_CRADLE_PLATE_THICKNESS, PITCH_CRADLE_PLATE_HEIGHT)),
        material="cradle_silver",
        name="front_pitch_plate",
        origin=Origin(xyz=(0.0, PITCH_CRADLE_PLATE_CENTER_Y, PITCH_CRADLE_PLATE_CENTER_Z)),
    )
    pitch_cradle.visual(
        Box((PITCH_CRADLE_PLATE_WIDTH, PITCH_CRADLE_PLATE_THICKNESS, PITCH_CRADLE_PLATE_HEIGHT)),
        material="cradle_silver",
        name="rear_pitch_plate",
        origin=Origin(xyz=(0.0, -PITCH_CRADLE_PLATE_CENTER_Y, PITCH_CRADLE_PLATE_CENTER_Z)),
    )
    pitch_cradle.visual(
        Box((PITCH_PIVOT_PAD_WIDTH, PITCH_PIVOT_PAD_THICKNESS, PITCH_PIVOT_PAD_HEIGHT)),
        material="cradle_silver",
        name="front_pitch_pad",
        origin=Origin(xyz=(0.0, PITCH_PIVOT_PAD_CENTER_Y, 0.0)),
    )
    pitch_cradle.visual(
        Box((PITCH_PIVOT_PAD_WIDTH, PITCH_PIVOT_PAD_THICKNESS, PITCH_PIVOT_PAD_HEIGHT)),
        material="cradle_silver",
        name="rear_pitch_pad",
        origin=Origin(xyz=(0.0, -PITCH_PIVOT_PAD_CENTER_Y, 0.0)),
    )
    pitch_cradle.visual(
        Box((PITCH_BOTTOM_RAIL_WIDTH, PITCH_BOTTOM_RAIL_DEPTH, PITCH_BOTTOM_RAIL_THICKNESS)),
        material="cradle_silver",
        name="bottom_pitch_rail",
        origin=Origin(xyz=(0.0, 0.0, PITCH_BOTTOM_RAIL_CENTER_Z)),
    )
    pitch_cradle.visual(
        Box((TOOL_FACE_CARRIER_THICKNESS, TOOL_FACE_CARRIER_DEPTH, TOOL_FACE_CARRIER_HEIGHT)),
        material="cradle_silver",
        name="tool_face_carrier",
        origin=Origin(xyz=(TOOL_FACE_CARRIER_CENTER_X, 0.0, TOOL_FACE_CENTER_Z)),
    )
    pitch_cradle.visual(
        Box((TOOL_FACE_THICKNESS, TOOL_FACE_WIDTH, TOOL_FACE_HEIGHT)),
        material="tool_face_light",
        name="tool_face_plate",
        origin=Origin(xyz=(TOOL_FACE_CENTER_X, 0.0, TOOL_FACE_CENTER_Z)),
    )
    pitch_cradle.inertial = Inertial.from_geometry(
        Box((0.16, 0.086, 0.152)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
    )

    model.articulation(
        "support_to_roll",
        ArticulationType.REVOLUTE,
        parent=top_support,
        child=roll_frame,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.15, upper=1.15, effort=16.0, velocity=1.5),
    )
    model.articulation(
        "roll_to_pitch",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_cradle,
        origin=Origin(xyz=(0.0, 0.0, -PITCH_AXIS_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.65, upper=0.95, effort=12.0, velocity=1.6),
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
    roll_frame = object_model.get_part("roll_frame")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll_joint = object_model.get_articulation("support_to_roll")
    pitch_joint = object_model.get_articulation("roll_to_pitch")

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lower[i] + upper[i]) / 2.0 for i in range(3))

    ctx.check(
        "fixture parts resolve",
        all(part is not None for part in (top_support, roll_frame, pitch_cradle)),
        details="One or more prompt-critical parts could not be resolved.",
    )
    ctx.check(
        "roll and pitch axes are perpendicular",
        abs(sum(a * b for a, b in zip(roll_joint.axis, pitch_joint.axis))) < 1e-8,
        details=f"roll_axis={roll_joint.axis}, pitch_axis={pitch_joint.axis}",
    )
    ctx.expect_origin_gap(
        roll_frame,
        pitch_cradle,
        axis="z",
        min_gap=0.06,
        max_gap=0.08,
        name="pitch axis sits meaningfully below the roll axis",
    )
    ctx.expect_within(
        pitch_cradle,
        roll_frame,
        axes="x",
        margin=0.05,
        name="pitch cradle stays laterally nested within the roll frame",
    )

    rest_tool_center = aabb_center(ctx.part_element_world_aabb(pitch_cradle, elem="tool_face_plate"))

    with ctx.pose({pitch_joint: 0.6}):
        pitched_tool_center = aabb_center(ctx.part_element_world_aabb(pitch_cradle, elem="tool_face_plate"))
    ctx.check(
        "positive pitch lifts the tool face",
        rest_tool_center is not None
        and pitched_tool_center is not None
        and pitched_tool_center[2] > rest_tool_center[2] + 0.02,
        details=f"rest={rest_tool_center}, pitched={pitched_tool_center}",
    )

    with ctx.pose({roll_joint: 0.45}):
        rolled_tool_center = aabb_center(ctx.part_element_world_aabb(pitch_cradle, elem="tool_face_plate"))
    ctx.check(
        "positive roll swings the tool face toward positive Y",
        rest_tool_center is not None
        and rolled_tool_center is not None
        and rolled_tool_center[1] > rest_tool_center[1] + 0.02,
        details=f"rest={rest_tool_center}, rolled={rolled_tool_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
