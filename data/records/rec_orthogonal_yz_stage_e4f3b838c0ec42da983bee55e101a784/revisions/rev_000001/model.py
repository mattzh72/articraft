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


PLATE_WIDTH = 0.44
PLATE_HEIGHT = 0.72
PLATE_THICKNESS = 0.016
PLATE_CENTER_Y = -0.060
PLATE_CENTER_Z = -0.12

RAIL_LENGTH = 0.42
RAIL_DEPTH = 0.024
RAIL_HEIGHT = 0.024

STANDOFF_WIDTH = 0.014
STANDOFF_DEPTH = 0.040
STANDOFF_HEIGHT = 0.050
LEFT_STANDOFF_X = -0.195
RIGHT_STANDOFF_X = 0.195
STANDOFF_CENTER_Y = -0.032
STANDOFF_CENTER_Z = 0.0

SADDLE_JAW_LENGTH = 0.15
SADDLE_JAW_DEPTH = 0.040
SADDLE_JAW_CENTER_Y = 0.008
SADDLE_JAW_HEIGHT = 0.012
UPPER_JAW_CENTER_Z = 0.018
LOWER_JAW_CENTER_Z = -0.018

SADDLE_BRIDGE_WIDTH = 0.10
SADDLE_BRIDGE_DEPTH = 0.028
SADDLE_BRIDGE_HEIGHT = 0.048
SADDLE_BRIDGE_CENTER_Y = 0.026
SADDLE_BRIDGE_CENTER_Z = 0.0

HANGER_WIDTH = 0.080
HANGER_DEPTH = 0.028
HANGER_HEIGHT = 0.060
HANGER_CENTER_Y = 0.024
HANGER_CENTER_Z = -0.054

GUIDE_WIDTH = 0.034
GUIDE_DEPTH = 0.016
GUIDE_HEIGHT = 0.190
GUIDE_CENTER_Y = 0.024
GUIDE_TOP_Z = -0.084
GUIDE_CENTER_Z = GUIDE_TOP_Z - (GUIDE_HEIGHT / 2.0)

CARRIAGE_CHEEK_WIDTH = 0.018
CARRIAGE_CHEEK_DEPTH = 0.028
CARRIAGE_CHEEK_HEIGHT = 0.100
CARRIAGE_CHEEK_OFFSET_X = 0.026
CARRIAGE_CHEEK_CENTER_Y = 0.0
CARRIAGE_CHEEK_CENTER_Z = -0.050

TOOL_PLATE_WIDTH = 0.086
TOOL_PLATE_DEPTH = 0.010
TOOL_PLATE_HEIGHT = 0.076
TOOL_PLATE_CENTER_Y = 0.014
TOOL_PLATE_CENTER_Z = -0.040

BOTTOM_PAD_WIDTH = 0.062
BOTTOM_PAD_DEPTH = 0.034
BOTTOM_PAD_HEIGHT = 0.012
BOTTOM_PAD_CENTER_Y = 0.0
BOTTOM_PAD_CENTER_Z = -0.106

HORIZONTAL_TRAVEL = 0.105
VERTICAL_TRAVEL = 0.145


def _add_box(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_transfer_stage")

    model.material("powder_gray", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("anodized_silver", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("dark_carriage", rgba=(0.22, 0.24, 0.27, 1.0))

    fixed_plate = model.part("fixed_plate")
    _add_box(
        fixed_plate,
        size=(PLATE_WIDTH, PLATE_THICKNESS, PLATE_HEIGHT),
        center=(0.0, PLATE_CENTER_Y, PLATE_CENTER_Z),
        material="powder_gray",
        name="plate_panel",
    )
    _add_box(
        fixed_plate,
        size=(RAIL_LENGTH, RAIL_DEPTH, RAIL_HEIGHT),
        center=(0.0, 0.0, 0.0),
        material="anodized_silver",
        name="rail",
    )
    _add_box(
        fixed_plate,
        size=(STANDOFF_WIDTH, STANDOFF_DEPTH, STANDOFF_HEIGHT),
        center=(LEFT_STANDOFF_X, STANDOFF_CENTER_Y, STANDOFF_CENTER_Z),
        material="powder_gray",
        name="left_standoff",
    )
    _add_box(
        fixed_plate,
        size=(STANDOFF_WIDTH, STANDOFF_DEPTH, STANDOFF_HEIGHT),
        center=(RIGHT_STANDOFF_X, STANDOFF_CENTER_Y, STANDOFF_CENTER_Z),
        material="powder_gray",
        name="right_standoff",
    )
    fixed_plate.inertial = Inertial.from_geometry(
        Box((PLATE_WIDTH, 0.08, PLATE_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(0.0, -0.030, -0.10)),
    )

    saddle = model.part("saddle")
    _add_box(
        saddle,
        size=(SADDLE_JAW_LENGTH, RAIL_DEPTH, SADDLE_JAW_HEIGHT),
        center=(0.0, 0.0, UPPER_JAW_CENTER_Z),
        material="anodized_silver",
        name="upper_jaw",
    )
    _add_box(
        saddle,
        size=(SADDLE_JAW_LENGTH, RAIL_DEPTH, SADDLE_JAW_HEIGHT),
        center=(0.0, 0.0, LOWER_JAW_CENTER_Z),
        material="anodized_silver",
        name="lower_jaw",
    )
    _add_box(
        saddle,
        size=(SADDLE_BRIDGE_WIDTH, SADDLE_BRIDGE_DEPTH, SADDLE_BRIDGE_HEIGHT),
        center=(0.0, SADDLE_BRIDGE_CENTER_Y, SADDLE_BRIDGE_CENTER_Z),
        material="anodized_silver",
        name="front_bridge",
    )
    _add_box(
        saddle,
        size=(HANGER_WIDTH, HANGER_DEPTH, HANGER_HEIGHT),
        center=(0.0, HANGER_CENTER_Y, HANGER_CENTER_Z),
        material="anodized_silver",
        name="hanger",
    )
    _add_box(
        saddle,
        size=(GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT),
        center=(0.0, GUIDE_CENTER_Y, GUIDE_CENTER_Z),
        material="dark_carriage",
        name="vertical_guide",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.18, 0.06, 0.30)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.020, -0.09)),
    )

    carriage = model.part("carriage")
    _add_box(
        carriage,
        size=(CARRIAGE_CHEEK_WIDTH, 0.028, CARRIAGE_CHEEK_HEIGHT),
        center=(-CARRIAGE_CHEEK_OFFSET_X, CARRIAGE_CHEEK_CENTER_Y, CARRIAGE_CHEEK_CENTER_Z),
        material="dark_carriage",
        name="left_cheek",
    )
    _add_box(
        carriage,
        size=(CARRIAGE_CHEEK_WIDTH, 0.028, CARRIAGE_CHEEK_HEIGHT),
        center=(CARRIAGE_CHEEK_OFFSET_X, CARRIAGE_CHEEK_CENTER_Y, CARRIAGE_CHEEK_CENTER_Z),
        material="dark_carriage",
        name="right_cheek",
    )
    _add_box(
        carriage,
        size=(TOOL_PLATE_WIDTH, TOOL_PLATE_DEPTH, TOOL_PLATE_HEIGHT),
        center=(0.0, TOOL_PLATE_CENTER_Y, TOOL_PLATE_CENTER_Z),
        material="powder_gray",
        name="tool_plate",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.10, 0.06, 0.11)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.028, -0.06)),
    )

    model.articulation(
        "plate_to_saddle",
        ArticulationType.PRISMATIC,
        parent=fixed_plate,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-HORIZONTAL_TRAVEL,
            upper=HORIZONTAL_TRAVEL,
            effort=900.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "saddle_to_carriage",
        ArticulationType.PRISMATIC,
        parent=saddle,
        child=carriage,
        origin=Origin(xyz=(0.0, GUIDE_CENTER_Y, GUIDE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=VERTICAL_TRAVEL,
            effort=600.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_plate = object_model.get_part("fixed_plate")
    saddle = object_model.get_part("saddle")
    carriage = object_model.get_part("carriage")
    x_stage = object_model.get_articulation("plate_to_saddle")
    z_stage = object_model.get_articulation("saddle_to_carriage")

    rail = fixed_plate.get_visual("rail")
    upper_jaw = saddle.get_visual("upper_jaw")
    lower_jaw = saddle.get_visual("lower_jaw")
    guide = saddle.get_visual("vertical_guide")
    left_cheek = carriage.get_visual("left_cheek")
    right_cheek = carriage.get_visual("right_cheek")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "correct_joint_axes",
        x_stage.axis == (1.0, 0.0, 0.0) and z_stage.axis == (0.0, 0.0, -1.0),
        details=f"x axis={x_stage.axis}, z axis={z_stage.axis}",
    )
    ctx.expect_contact(
        saddle,
        fixed_plate,
        elem_a=upper_jaw,
        elem_b=rail,
        name="upper_jaw_rides_on_rail",
    )
    ctx.expect_contact(
        saddle,
        fixed_plate,
        elem_a=lower_jaw,
        elem_b=rail,
        name="lower_jaw_rides_on_rail",
    )
    ctx.expect_contact(
        carriage,
        saddle,
        elem_a=left_cheek,
        elem_b=guide,
        name="left_cheek_guides_on_vertical_rail",
    )
    ctx.expect_contact(
        carriage,
        saddle,
        elem_a=right_cheek,
        elem_b=guide,
        name="right_cheek_guides_on_vertical_rail",
    )

    with ctx.pose({x_stage: x_stage.motion_limits.upper}):
        ctx.expect_origin_gap(
            saddle,
            fixed_plate,
            axis="x",
            min_gap=HORIZONTAL_TRAVEL - 0.002,
            max_gap=HORIZONTAL_TRAVEL + 0.002,
            name="saddle_reaches_right_travel",
        )
    with ctx.pose({x_stage: x_stage.motion_limits.lower}):
        ctx.expect_origin_gap(
            fixed_plate,
            saddle,
            axis="x",
            min_gap=HORIZONTAL_TRAVEL - 0.002,
            max_gap=HORIZONTAL_TRAVEL + 0.002,
            name="saddle_reaches_left_travel",
        )

    with ctx.pose({z_stage: 0.0}):
        top_z = ctx.part_world_position(carriage)[2]
    with ctx.pose({z_stage: z_stage.motion_limits.upper}):
        low_z = ctx.part_world_position(carriage)[2]
    ctx.check(
        "carriage_moves_downward",
        low_z < top_z and abs((top_z - low_z) - VERTICAL_TRAVEL) <= 0.002,
        details=f"top_z={top_z} low_z={low_z} delta={top_z - low_z}",
    )

    with ctx.pose({x_stage: x_stage.motion_limits.lower, z_stage: z_stage.motion_limits.upper}):
        ctx.expect_contact(
            carriage,
            saddle,
            elem_a=left_cheek,
            elem_b=guide,
            name="left_cheek_stays_guided_at_left_low_pose",
        )
        ctx.expect_contact(
            carriage,
            saddle,
            elem_a=right_cheek,
            elem_b=guide,
            name="right_cheek_stays_guided_at_left_low_pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_left_low_pose")
    with ctx.pose({x_stage: x_stage.motion_limits.upper, z_stage: z_stage.motion_limits.upper}):
        ctx.expect_contact(
            carriage,
            saddle,
            elem_a=left_cheek,
            elem_b=guide,
            name="left_cheek_stays_guided_at_right_low_pose",
        )
        ctx.expect_contact(
            carriage,
            saddle,
            elem_a=right_cheek,
            elem_b=guide,
            name="right_cheek_stays_guided_at_right_low_pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_right_low_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
