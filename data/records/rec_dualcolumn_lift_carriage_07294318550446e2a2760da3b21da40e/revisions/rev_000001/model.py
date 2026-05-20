from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import ArticulatedObject, ArticulationType, Box, MotionLimits, Origin, TestContext, TestReport


COLUMN_SIZE = 0.09
COLUMN_SPACING = 0.68
COLUMN_X = COLUMN_SPACING / 2.0

BASE_BEAM_W = 0.92
BASE_BEAM_D = 0.18
BASE_BEAM_H = 0.10
FOOT_W = 0.16
FOOT_D = 0.42
FOOT_H = 0.04

BRIDGE_W = 0.92
BRIDGE_D = 0.18
BRIDGE_H = 0.10

COLUMN_CLEAR_H = 1.56
COLUMN_EMBED = 0.012

CARRIAGE_START_Z = 0.72
CARRIAGE_TRAVEL = 0.70

GUIDE_RAIL_T = 0.028
GUIDE_RAIL_Y = 0.12
GUIDE_RAIL_H = 0.46

UPPER_BEAM_X = 0.80
UPPER_BEAM_Y = 0.03
UPPER_BEAM_H = 0.14
UPPER_BEAM_Y_CENTER = 0.065
UPPER_BEAM_Z_CENTER = 0.02

LOWER_BEAM_X = 0.80
LOWER_BEAM_Y = 0.03
LOWER_BEAM_H = 0.12
LOWER_BEAM_Y_CENTER = 0.065
LOWER_BEAM_Z_CENTER = -0.13

CENTER_PLATE_X = 0.22
CENTER_PLATE_Y = 0.14
CENTER_PLATE_H = 0.28
CENTER_PLATE_Z_CENTER = -0.04

DECK_X = 0.48
DECK_Y = 0.18
DECK_H = 0.035
DECK_Z_CENTER = 0.102


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    *,
    material,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_frame_twin_column_lift")

    frame_blue = model.material("frame_blue", rgba=(0.22, 0.35, 0.56, 1.0))
    column_gray = model.material("column_gray", rgba=(0.22, 0.24, 0.28, 1.0))
    carriage_orange = model.material("carriage_orange", rgba=(0.83, 0.46, 0.16, 1.0))

    frame = model.part("frame")
    _add_box(frame, (BASE_BEAM_W, BASE_BEAM_D, BASE_BEAM_H), (0.0, 0.0, BASE_BEAM_H / 2.0), material=frame_blue, name="lower_base")
    _add_box(frame, (FOOT_W, FOOT_D, FOOT_H), (-COLUMN_X, 0.0, FOOT_H / 2.0), material=frame_blue, name="left_foot")
    _add_box(frame, (FOOT_W, FOOT_D, FOOT_H), (COLUMN_X, 0.0, FOOT_H / 2.0), material=frame_blue, name="right_foot")
    _add_box(
        frame,
        (COLUMN_SIZE, COLUMN_SIZE, COLUMN_CLEAR_H + 2.0 * COLUMN_EMBED),
        (-COLUMN_X, 0.0, BASE_BEAM_H + (COLUMN_CLEAR_H / 2.0)),
        material=column_gray,
        name="left_column",
    )
    _add_box(
        frame,
        (COLUMN_SIZE, COLUMN_SIZE, COLUMN_CLEAR_H + 2.0 * COLUMN_EMBED),
        (COLUMN_X, 0.0, BASE_BEAM_H + (COLUMN_CLEAR_H / 2.0)),
        material=column_gray,
        name="right_column",
    )
    _add_box(
        frame,
        (BRIDGE_W, BRIDGE_D, BRIDGE_H),
        (0.0, 0.0, BASE_BEAM_H + COLUMN_CLEAR_H + (BRIDGE_H / 2.0)),
        material=frame_blue,
        name="upper_bridge",
    )

    carriage = model.part("carriage")
    guide_offset = (COLUMN_SIZE / 2.0) + (GUIDE_RAIL_T / 2.0)
    _add_box(
        carriage,
        (GUIDE_RAIL_T, GUIDE_RAIL_Y, GUIDE_RAIL_H),
        (-COLUMN_X - guide_offset, 0.0, 0.0),
        material=carriage_orange,
        name="left_outer_guide",
    )
    _add_box(
        carriage,
        (GUIDE_RAIL_T, GUIDE_RAIL_Y, GUIDE_RAIL_H),
        (-COLUMN_X + guide_offset, 0.0, 0.0),
        material=carriage_orange,
        name="left_inner_guide",
    )
    _add_box(
        carriage,
        (GUIDE_RAIL_T, GUIDE_RAIL_Y, GUIDE_RAIL_H),
        (COLUMN_X - guide_offset, 0.0, 0.0),
        material=carriage_orange,
        name="right_inner_guide",
    )
    _add_box(
        carriage,
        (GUIDE_RAIL_T, GUIDE_RAIL_Y, GUIDE_RAIL_H),
        (COLUMN_X + guide_offset, 0.0, 0.0),
        material=carriage_orange,
        name="right_outer_guide",
    )
    _add_box(
        carriage,
        (UPPER_BEAM_X, UPPER_BEAM_Y, UPPER_BEAM_H),
        (0.0, UPPER_BEAM_Y_CENTER, UPPER_BEAM_Z_CENTER),
        material=carriage_orange,
        name="upper_front_beam",
    )
    _add_box(
        carriage,
        (UPPER_BEAM_X, UPPER_BEAM_Y, UPPER_BEAM_H),
        (0.0, -UPPER_BEAM_Y_CENTER, UPPER_BEAM_Z_CENTER),
        material=carriage_orange,
        name="upper_rear_beam",
    )
    _add_box(
        carriage,
        (LOWER_BEAM_X, LOWER_BEAM_Y, LOWER_BEAM_H),
        (0.0, LOWER_BEAM_Y_CENTER, LOWER_BEAM_Z_CENTER),
        material=carriage_orange,
        name="lower_front_beam",
    )
    _add_box(
        carriage,
        (LOWER_BEAM_X, LOWER_BEAM_Y, LOWER_BEAM_H),
        (0.0, -LOWER_BEAM_Y_CENTER, LOWER_BEAM_Z_CENTER),
        material=carriage_orange,
        name="lower_rear_beam",
    )
    _add_box(
        carriage,
        (CENTER_PLATE_X, CENTER_PLATE_Y, CENTER_PLATE_H),
        (0.0, 0.0, CENTER_PLATE_Z_CENTER),
        material=carriage_orange,
        name="center_plate",
    )
    _add_box(
        carriage,
        (DECK_X, DECK_Y, DECK_H),
        (0.0, 0.0, DECK_Z_CENTER),
        material=carriage_orange,
        name="carriage_deck",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_START_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8000.0,
            velocity=0.35,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift_joint = object_model.get_articulation("frame_to_carriage")
    frame.get_visual("lower_base")
    frame.get_visual("left_column")
    frame.get_visual("right_column")
    frame.get_visual("upper_bridge")
    carriage.get_visual("left_outer_guide")
    carriage.get_visual("left_inner_guide")
    carriage.get_visual("right_inner_guide")
    carriage.get_visual("right_outer_guide")
    carriage.get_visual("upper_front_beam")
    carriage.get_visual("upper_rear_beam")
    carriage.get_visual("lower_front_beam")
    carriage.get_visual("lower_rear_beam")
    carriage.get_visual("center_plate")
    carriage.get_visual("carriage_deck")

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

    ctx.check(
        "carriage_joint_is_vertical_prismatic",
        lift_joint.articulation_type == ArticulationType.PRISMATIC and tuple(lift_joint.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic carriage joint, got type={lift_joint.articulation_type} axis={lift_joint.axis}",
    )

    lower = 0.0
    upper = CARRIAGE_TRAVEL
    if lift_joint.motion_limits is not None:
        if lift_joint.motion_limits.lower is not None:
            lower = lift_joint.motion_limits.lower
        if lift_joint.motion_limits.upper is not None:
            upper = lift_joint.motion_limits.upper

    with ctx.pose({lift_joint: lower}):
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_outer_guide",
            elem_b="left_column",
            name="left_outer_guide_contacts_left_column",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="left_inner_guide",
            elem_b="left_column",
            name="left_inner_guide_contacts_left_column",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_inner_guide",
            elem_b="right_column",
            name="right_inner_guide_contacts_right_column",
        )
        ctx.expect_contact(
            carriage,
            frame,
            elem_a="right_outer_guide",
            elem_b="right_column",
            name="right_outer_guide_contacts_right_column",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem="center_plate",
            negative_elem="lower_base",
            min_gap=0.35,
            max_gap=0.70,
            name="lowered_carriage_sits_clear_of_base",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="yz",
            elem_a="left_outer_guide",
            elem_b="left_column",
            min_overlap=0.09,
            name="left_outer_guide_runs_along_left_column",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="yz",
            elem_a="left_inner_guide",
            elem_b="left_column",
            min_overlap=0.09,
            name="left_inner_guide_runs_along_left_column",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="yz",
            elem_a="right_inner_guide",
            elem_b="right_column",
            min_overlap=0.09,
            name="right_inner_guide_runs_along_right_column",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="yz",
            elem_a="right_outer_guide",
            elem_b="right_column",
            min_overlap=0.09,
            name="right_outer_guide_runs_along_right_column",
        )
        lower_pos = ctx.part_world_position(carriage)

    with ctx.pose({lift_joint: upper}):
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="upper_bridge",
            negative_elem="carriage_deck",
            min_gap=0.02,
            max_gap=0.13,
            name="raised_carriage_clears_upper_bridge",
        )
        upper_pos = ctx.part_world_position(carriage)

    if lower_pos is not None and upper_pos is not None:
        ctx.check(
            "carriage_translates_upward",
            upper_pos[2] > lower_pos[2] + 0.65,
            f"Expected more than 0.65 m of upward travel, got lower_z={lower_pos[2]:.3f} upper_z={upper_pos[2]:.3f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
