from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


UPRIGHT_CENTER_X = 0.68
FOOT_SIZE = (0.24, 0.40, 0.06)
UPRIGHT_SIZE = (0.12, 0.16, 1.34)
TOP_BEAM_SIZE = (1.48, 0.20, 0.16)
RAIL_SIZE = (1.04, 0.026, 0.014)
RAIL_OFFSET_Y = 0.055

RUNNER_SIZE = (0.22, 0.026, 0.018)
CARRIAGE_BODY_SIZE = (0.24, 0.13, 0.10)
CARRIAGE_END_CAP_SIZE = (0.03, 0.11, 0.08)
CARRIAGE_FACE_SIZE = (0.24, 0.03, 0.12)
MOUNT_PLATE_SIZE = (0.14, 0.08, 0.02)

SLIDE_TOP_BLOCK_SIZE = (0.12, 0.07, 0.024)
SLIDE_COLUMN_SIZE = (0.12, 0.08, 0.36)
SLIDE_GUIDE_SIZE = (0.02, 0.012, 0.30)
SLIDE_TOOL_PLATE_SIZE = (0.16, 0.10, 0.02)
TOOL_NOSE_RADIUS = 0.028
TOOL_NOSE_LENGTH = 0.08

FRAME_FOOT_Z = FOOT_SIZE[2] / 2.0
FRAME_UPRIGHT_Z = FOOT_SIZE[2] + (UPRIGHT_SIZE[2] / 2.0)
FRAME_BEAM_Z = FOOT_SIZE[2] + UPRIGHT_SIZE[2] + (TOP_BEAM_SIZE[2] / 2.0)
BEAM_BOTTOM_Z = FRAME_BEAM_Z - (TOP_BEAM_SIZE[2] / 2.0)
RAIL_CENTER_Z = BEAM_BOTTOM_Z - (RAIL_SIZE[2] / 2.0)
RAIL_BOTTOM_Z = RAIL_CENTER_Z - (RAIL_SIZE[2] / 2.0)

RUNNER_CENTER_Z = -(RUNNER_SIZE[2] / 2.0)
CARRIAGE_BODY_CENTER_Z = -RUNNER_SIZE[2] - (CARRIAGE_BODY_SIZE[2] / 2.0)
CARRIAGE_END_CAP_CENTER_Z = -0.058
CARRIAGE_FACE_CENTER_Z = -0.070
MOUNT_PLATE_CENTER_Z = (
    -RUNNER_SIZE[2] - CARRIAGE_BODY_SIZE[2] - (MOUNT_PLATE_SIZE[2] / 2.0)
)
MOUNT_PLATE_BOTTOM_Z = MOUNT_PLATE_CENTER_Z - (MOUNT_PLATE_SIZE[2] / 2.0)

SLIDE_TOP_BLOCK_CENTER_Z = -(SLIDE_TOP_BLOCK_SIZE[2] / 2.0)
SLIDE_COLUMN_CENTER_Z = -SLIDE_TOP_BLOCK_SIZE[2] - (SLIDE_COLUMN_SIZE[2] / 2.0)
SLIDE_GUIDE_CENTER_Z = -0.174
SLIDE_TOOL_PLATE_CENTER_Z = (
    -SLIDE_TOP_BLOCK_SIZE[2] - SLIDE_COLUMN_SIZE[2] - (SLIDE_TOOL_PLATE_SIZE[2] / 2.0)
)
TOOL_NOSE_CENTER_Z = (
    SLIDE_TOOL_PLATE_CENTER_Z
    - (SLIDE_TOOL_PLATE_SIZE[2] / 2.0)
    - (TOOL_NOSE_LENGTH / 2.0)
)

X_TRAVEL = 0.40
Z_TRAVEL = 0.34


def _add_box(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_vertical_slide")

    model.material("frame_gray", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("rail_steel", rgba=(0.34, 0.36, 0.40, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.19, 0.22, 1.0))
    model.material("slide_light", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("tool_steel", rgba=(0.46, 0.48, 0.52, 1.0))
    model.material("pad_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        name="left_foot",
        size=FOOT_SIZE,
        xyz=(-UPRIGHT_CENTER_X, 0.0, FRAME_FOOT_Z),
        material="pad_black",
    )
    _add_box(
        frame,
        name="right_foot",
        size=FOOT_SIZE,
        xyz=(UPRIGHT_CENTER_X, 0.0, FRAME_FOOT_Z),
        material="pad_black",
    )
    _add_box(
        frame,
        name="left_upright",
        size=UPRIGHT_SIZE,
        xyz=(-UPRIGHT_CENTER_X, 0.0, FRAME_UPRIGHT_Z),
        material="frame_gray",
    )
    _add_box(
        frame,
        name="right_upright",
        size=UPRIGHT_SIZE,
        xyz=(UPRIGHT_CENTER_X, 0.0, FRAME_UPRIGHT_Z),
        material="frame_gray",
    )
    _add_box(
        frame,
        name="top_beam",
        size=TOP_BEAM_SIZE,
        xyz=(0.0, 0.0, FRAME_BEAM_Z),
        material="frame_gray",
    )
    _add_box(
        frame,
        name="front_rail",
        size=RAIL_SIZE,
        xyz=(0.0, RAIL_OFFSET_Y, RAIL_CENTER_Z),
        material="rail_steel",
    )
    _add_box(
        frame,
        name="rear_rail",
        size=RAIL_SIZE,
        xyz=(0.0, -RAIL_OFFSET_Y, RAIL_CENTER_Z),
        material="rail_steel",
    )

    carriage = model.part("beam_carriage")
    _add_box(
        carriage,
        name="front_runner",
        size=RUNNER_SIZE,
        xyz=(0.0, RAIL_OFFSET_Y, RUNNER_CENTER_Z),
        material="rail_steel",
    )
    _add_box(
        carriage,
        name="rear_runner",
        size=RUNNER_SIZE,
        xyz=(0.0, -RAIL_OFFSET_Y, RUNNER_CENTER_Z),
        material="rail_steel",
    )
    _add_box(
        carriage,
        name="body",
        size=CARRIAGE_BODY_SIZE,
        xyz=(0.0, 0.0, CARRIAGE_BODY_CENTER_Z),
        material="carriage_dark",
    )
    _add_box(
        carriage,
        name="front_face",
        size=CARRIAGE_FACE_SIZE,
        xyz=(0.0, (CARRIAGE_BODY_SIZE[1] / 2.0) + (CARRIAGE_FACE_SIZE[1] / 2.0), CARRIAGE_FACE_CENTER_Z),
        material="carriage_dark",
    )
    _add_box(
        carriage,
        name="left_end_cap",
        size=CARRIAGE_END_CAP_SIZE,
        xyz=(
            -((CARRIAGE_BODY_SIZE[0] / 2.0) - (CARRIAGE_END_CAP_SIZE[0] / 2.0)),
            0.0,
            CARRIAGE_END_CAP_CENTER_Z,
        ),
        material="carriage_dark",
    )
    _add_box(
        carriage,
        name="right_end_cap",
        size=CARRIAGE_END_CAP_SIZE,
        xyz=(
            (CARRIAGE_BODY_SIZE[0] / 2.0) - (CARRIAGE_END_CAP_SIZE[0] / 2.0),
            0.0,
            CARRIAGE_END_CAP_CENTER_Z,
        ),
        material="carriage_dark",
    )
    _add_box(
        carriage,
        name="mount_plate",
        size=MOUNT_PLATE_SIZE,
        xyz=(0.0, 0.0, MOUNT_PLATE_CENTER_Z),
        material="slide_light",
    )

    slide = model.part("z_slide")
    _add_box(
        slide,
        name="top_block",
        size=SLIDE_TOP_BLOCK_SIZE,
        xyz=(0.0, 0.0, SLIDE_TOP_BLOCK_CENTER_Z),
        material="carriage_dark",
    )
    _add_box(
        slide,
        name="column",
        size=SLIDE_COLUMN_SIZE,
        xyz=(0.0, 0.0, SLIDE_COLUMN_CENTER_Z),
        material="slide_light",
    )
    _add_box(
        slide,
        name="left_guide",
        size=SLIDE_GUIDE_SIZE,
        xyz=(0.035, 0.034, SLIDE_GUIDE_CENTER_Z),
        material="rail_steel",
    )
    _add_box(
        slide,
        name="right_guide",
        size=SLIDE_GUIDE_SIZE,
        xyz=(-0.035, 0.034, SLIDE_GUIDE_CENTER_Z),
        material="rail_steel",
    )
    _add_box(
        slide,
        name="tool_plate",
        size=SLIDE_TOOL_PLATE_SIZE,
        xyz=(0.0, 0.0, SLIDE_TOOL_PLATE_CENTER_Z),
        material="carriage_dark",
    )
    _add_cylinder(
        slide,
        name="tool_nose",
        radius=TOOL_NOSE_RADIUS,
        length=TOOL_NOSE_LENGTH,
        xyz=(0.0, 0.0, TOOL_NOSE_CENTER_Z),
        material="tool_steel",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.50,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_z",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, MOUNT_PLATE_BOTTOM_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.35,
            lower=0.0,
            upper=Z_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("beam_carriage")
    slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("frame_to_carriage")
    z_axis = object_model.get_articulation("carriage_to_z")

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

    ctx.check("frame_present", frame is not None)
    ctx.check("beam_carriage_present", carriage is not None)
    ctx.check("z_slide_present", slide is not None)
    ctx.check("x_axis_is_gantry_horizontal", x_axis.axis == (1.0, 0.0, 0.0))
    ctx.check("z_axis_is_vertical_down", z_axis.axis == (0.0, 0.0, -1.0))
    ctx.check(
        "x_travel_is_centered",
        x_axis.motion_limits is not None
        and x_axis.motion_limits.lower == -X_TRAVEL
        and x_axis.motion_limits.upper == X_TRAVEL,
    )
    ctx.check(
        "z_travel_is_downward_only",
        z_axis.motion_limits is not None
        and z_axis.motion_limits.lower == 0.0
        and z_axis.motion_limits.upper == Z_TRAVEL,
    )

    ctx.expect_contact(
        frame,
        carriage,
        elem_a="front_rail",
        elem_b="front_runner",
        name="front_runner_contacts_front_rail",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a="rear_rail",
        elem_b="rear_runner",
        name="rear_runner_contacts_rear_rail",
    )
    ctx.expect_contact(
        carriage,
        slide,
        elem_a="mount_plate",
        elem_b="top_block",
        name="z_slide_hangs_from_mount_plate",
    )
    ctx.expect_origin_distance(
        carriage,
        slide,
        axes="xy",
        max_dist=1e-6,
        name="slide_is_centered_under_carriage_at_rest",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    rest_slide_pos = ctx.part_world_position(slide)

    with ctx.pose({x_axis: X_TRAVEL}):
        moved_right_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage_moves_to_positive_x",
            moved_right_pos is not None
            and rest_carriage_pos is not None
            and moved_right_pos[0] > rest_carriage_pos[0] + 0.39,
        )
        ctx.expect_contact(
            frame,
            carriage,
            elem_a="front_rail",
            elem_b="front_runner",
            name="front_runner_stays_supported_at_right_limit",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="x",
            min_gap=0.08,
            positive_elem="right_upright",
            negative_elem="body",
            name="carriage_clears_right_upright",
        )

    with ctx.pose({x_axis: -X_TRAVEL}):
        moved_left_pos = ctx.part_world_position(carriage)
        ctx.check(
            "carriage_moves_to_negative_x",
            moved_left_pos is not None
            and rest_carriage_pos is not None
            and moved_left_pos[0] < rest_carriage_pos[0] - 0.39,
        )
        ctx.expect_contact(
            frame,
            carriage,
            elem_a="rear_rail",
            elem_b="rear_runner",
            name="rear_runner_stays_supported_at_left_limit",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="x",
            min_gap=0.08,
            positive_elem="body",
            negative_elem="left_upright",
            name="carriage_clears_left_upright",
        )

    with ctx.pose({z_axis: Z_TRAVEL}):
        lowered_slide_pos = ctx.part_world_position(slide)
        ctx.check(
            "z_slide_moves_downward",
            lowered_slide_pos is not None
            and rest_slide_pos is not None
            and lowered_slide_pos[2] < rest_slide_pos[2] - 0.33,
        )
        ctx.expect_origin_distance(
            carriage,
            slide,
            axes="xy",
            max_dist=1e-6,
            name="slide_stays_centered_when_lowered",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
