from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_HEIGHT = 1.55
POST_CENTER_X = 0.26
CHANNEL_WIDTH = 0.07
CHANNEL_DEPTH = 0.085
CHANNEL_WALL = 0.008

TOP_BRIDGE_WIDTH = (2.0 * POST_CENTER_X) + CHANNEL_WIDTH
TOP_BRIDGE_DEPTH = 0.085
TOP_BRIDGE_HEIGHT = 0.09
TOP_BRIDGE_CENTER_Z = FRAME_HEIGHT - (TOP_BRIDGE_HEIGHT / 2.0)

LOWER_TIE_WIDTH = 0.48
LOWER_TIE_DEPTH = 0.05
LOWER_TIE_HEIGHT = 0.06
LOWER_TIE_CENTER_Y = -0.01
LOWER_TIE_CENTER_Z = 0.12

GUIDE_SHOE_WIDTH = 0.018
GUIDE_SHOE_DEPTH = 0.03
GUIDE_SHOE_HEIGHT = 0.30
GUIDE_SHOE_CENTER_X = 0.278

GUIDE_ARM_WIDTH = 0.049
GUIDE_ARM_DEPTH = 0.03
GUIDE_ARM_HEIGHT = 0.30
GUIDE_ARM_CENTER_X = 0.2445

CROSS_BEAM_WIDTH = 0.40
CROSS_BEAM_DEPTH = 0.05
CROSS_BEAM_HEIGHT = 0.055
CROSS_BEAM_Y = -0.005
CROSS_BEAM_UPPER_Z = 0.095
CROSS_BEAM_LOWER_Z = -0.095

SIDE_RIB_WIDTH = 0.06
SIDE_RIB_DEPTH = 0.06
SIDE_RIB_HEIGHT = 0.30
SIDE_RIB_CENTER_X = 0.19
SIDE_RIB_CENTER_Y = 0.03

SUPPORT_PLATE_WIDTH = 0.35
SUPPORT_PLATE_DEPTH = 0.03
SUPPORT_PLATE_HEIGHT = 0.30
SUPPORT_PLATE_CENTER_Y = 0.045

TOOL_PLATE_WIDTH = 0.40
TOOL_PLATE_DEPTH = 0.014
TOOL_PLATE_HEIGHT = 0.34
TOOL_PLATE_CENTER_Y = 0.066

CARRIAGE_HOME_Z = 0.36
CARRIAGE_TRAVEL = 0.82

CHANNEL_WEB_CENTER_OFFSET_X = (CHANNEL_WIDTH / 2.0) - (CHANNEL_WALL / 2.0)
CHANNEL_FLANGE_WIDTH = CHANNEL_WIDTH - CHANNEL_WALL
CHANNEL_FLANGE_CENTER_OFFSET_X = CHANNEL_WALL / 2.0
CHANNEL_FLANGE_CENTER_Y = (CHANNEL_DEPTH / 2.0) - (CHANNEL_WALL / 2.0)

def _channel_web_center_x(channel_center_x: float, open_direction: float) -> float:
    return channel_center_x - (open_direction * CHANNEL_WEB_CENTER_OFFSET_X)


def _channel_flange_center_x(channel_center_x: float, open_direction: float) -> float:
    return channel_center_x + (open_direction * CHANNEL_FLANGE_CENTER_OFFSET_X)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="channel_post_vertical_lift_carriage")

    model.material("frame_blue", rgba=(0.20, 0.30, 0.46, 1.0))
    model.material("frame_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("tool_plate_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    frame = model.part("frame")
    left_channel_center_x = -POST_CENTER_X
    right_channel_center_x = POST_CENTER_X

    frame.visual(
        Box((CHANNEL_WALL, CHANNEL_DEPTH, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                _channel_web_center_x(left_channel_center_x, open_direction=1.0),
                0.0,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material="frame_blue",
        name="left_channel_web",
    )
    frame.visual(
        Box((CHANNEL_FLANGE_WIDTH, CHANNEL_WALL, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                _channel_flange_center_x(left_channel_center_x, open_direction=1.0),
                CHANNEL_FLANGE_CENTER_Y,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material="frame_blue",
        name="left_channel_front_flange",
    )
    frame.visual(
        Box((CHANNEL_FLANGE_WIDTH, CHANNEL_WALL, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                _channel_flange_center_x(left_channel_center_x, open_direction=1.0),
                -CHANNEL_FLANGE_CENTER_Y,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material="frame_blue",
        name="left_channel_rear_flange",
    )
    frame.visual(
        Box((CHANNEL_WALL, CHANNEL_DEPTH, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                _channel_web_center_x(right_channel_center_x, open_direction=-1.0),
                0.0,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material="frame_blue",
        name="right_channel_web",
    )
    frame.visual(
        Box((CHANNEL_FLANGE_WIDTH, CHANNEL_WALL, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                _channel_flange_center_x(right_channel_center_x, open_direction=-1.0),
                CHANNEL_FLANGE_CENTER_Y,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material="frame_blue",
        name="right_channel_front_flange",
    )
    frame.visual(
        Box((CHANNEL_FLANGE_WIDTH, CHANNEL_WALL, FRAME_HEIGHT)),
        origin=Origin(
            xyz=(
                _channel_flange_center_x(right_channel_center_x, open_direction=-1.0),
                -CHANNEL_FLANGE_CENTER_Y,
                FRAME_HEIGHT / 2.0,
            )
        ),
        material="frame_blue",
        name="right_channel_rear_flange",
    )
    frame.visual(
        Box((TOP_BRIDGE_WIDTH, TOP_BRIDGE_DEPTH, TOP_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, TOP_BRIDGE_CENTER_Z)),
        material="frame_blue",
        name="top_bridge",
    )
    frame.visual(
        Box((LOWER_TIE_WIDTH, LOWER_TIE_DEPTH, LOWER_TIE_HEIGHT)),
        origin=Origin(xyz=(0.0, LOWER_TIE_CENTER_Y, LOWER_TIE_CENTER_Z)),
        material="frame_steel",
        name="lower_tie",
    )
    frame.inertial = Inertial.from_geometry(
        Box((TOP_BRIDGE_WIDTH, CHANNEL_DEPTH, FRAME_HEIGHT)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((GUIDE_SHOE_WIDTH, GUIDE_SHOE_DEPTH, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_SHOE_CENTER_X, 0.0, 0.0)),
        material="carriage_dark",
        name="left_guide_shoe",
    )
    carriage.visual(
        Box((GUIDE_SHOE_WIDTH, GUIDE_SHOE_DEPTH, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(GUIDE_SHOE_CENTER_X, 0.0, 0.0)),
        material="carriage_dark",
        name="right_guide_shoe",
    )
    carriage.visual(
        Box((GUIDE_ARM_WIDTH, GUIDE_ARM_DEPTH, GUIDE_ARM_HEIGHT)),
        origin=Origin(xyz=(-GUIDE_ARM_CENTER_X, 0.0, 0.0)),
        material="carriage_dark",
        name="left_guide_arm",
    )
    carriage.visual(
        Box((GUIDE_ARM_WIDTH, GUIDE_ARM_DEPTH, GUIDE_ARM_HEIGHT)),
        origin=Origin(xyz=(GUIDE_ARM_CENTER_X, 0.0, 0.0)),
        material="carriage_dark",
        name="right_guide_arm",
    )
    carriage.visual(
        Box((CROSS_BEAM_WIDTH, CROSS_BEAM_DEPTH, CROSS_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, CROSS_BEAM_Y, CROSS_BEAM_UPPER_Z)),
        material="carriage_dark",
        name="upper_cross_beam",
    )
    carriage.visual(
        Box((CROSS_BEAM_WIDTH, CROSS_BEAM_DEPTH, CROSS_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, CROSS_BEAM_Y, CROSS_BEAM_LOWER_Z)),
        material="carriage_dark",
        name="lower_cross_beam",
    )
    carriage.visual(
        Box((SIDE_RIB_WIDTH, SIDE_RIB_DEPTH, SIDE_RIB_HEIGHT)),
        origin=Origin(xyz=(-SIDE_RIB_CENTER_X, SIDE_RIB_CENTER_Y, 0.0)),
        material="carriage_dark",
        name="left_side_rib",
    )
    carriage.visual(
        Box((SIDE_RIB_WIDTH, SIDE_RIB_DEPTH, SIDE_RIB_HEIGHT)),
        origin=Origin(xyz=(SIDE_RIB_CENTER_X, SIDE_RIB_CENTER_Y, 0.0)),
        material="carriage_dark",
        name="right_side_rib",
    )
    carriage.visual(
        Box((SUPPORT_PLATE_WIDTH, SUPPORT_PLATE_DEPTH, SUPPORT_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, SUPPORT_PLATE_CENTER_Y, 0.0)),
        material="carriage_dark",
        name="support_plate",
    )
    carriage.visual(
        Box((TOOL_PLATE_WIDTH, TOOL_PLATE_DEPTH, TOOL_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, TOOL_PLATE_CENTER_Y, 0.0)),
        material="tool_plate_gray",
        name="tool_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((TOOL_PLATE_WIDTH, 0.11, TOOL_PLATE_HEIGHT)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.02, 0.0)),
    )

    model.articulation(
        "frame_to_carriage_lift",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=1600.0,
            velocity=0.25,
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

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage_lift")

    left_guide_shoe = carriage.get_visual("left_guide_shoe")
    right_guide_shoe = carriage.get_visual("right_guide_shoe")
    tool_plate = carriage.get_visual("tool_plate")
    support_plate = carriage.get_visual("support_plate")
    top_bridge = frame.get_visual("top_bridge")
    left_channel_web = frame.get_visual("left_channel_web")
    right_channel_web = frame.get_visual("right_channel_web")

    with ctx.pose({lift: 0.0}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=left_guide_shoe,
            elem_b=left_channel_web,
            name="left guide shoe bears on the left mast channel",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a=right_guide_shoe,
            elem_b=right_channel_web,
            name="right guide shoe bears on the right mast channel",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="y",
            positive_elem=tool_plate,
            min_gap=0.01,
            max_gap=0.03,
            name="tool plate sits slightly proud of the mast face",
        )
        ctx.expect_contact(
            carriage,
            carriage,
            elem_a=support_plate,
            elem_b=tool_plate,
            name="tool plate is mounted directly onto the carriage support plate",
        )
        rest_pos = ctx.part_world_position(carriage)

    upper = 0.0
    if lift.motion_limits is not None and lift.motion_limits.upper is not None:
        upper = lift.motion_limits.upper

    with ctx.pose({lift: upper}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem=top_bridge,
            negative_elem=support_plate,
            min_gap=0.08,
            name="carriage clears the top bridge at full lift",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic joint raises the carriage upward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.75,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
