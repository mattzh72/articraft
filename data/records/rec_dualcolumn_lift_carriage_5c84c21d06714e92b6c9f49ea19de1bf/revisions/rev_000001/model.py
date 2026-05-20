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


FRAME_WIDTH = 1.36
FRAME_DEPTH = 0.82
BASE_RAIL_HEIGHT = 0.18
BASE_RAIL_DEPTH = 0.14
SIDE_RAIL_WIDTH = 0.14
INNER_STRINGER_WIDTH = 0.14
INNER_STRINGER_LENGTH = FRAME_DEPTH - 2.0 * BASE_RAIL_DEPTH
INNER_STRINGER_HEIGHT = 0.14
INNER_STRINGER_X = 0.23
CENTER_BASE_BEAM_WIDTH = 0.32
CENTER_BASE_BEAM_DEPTH = 0.18
CENTER_BASE_BEAM_HEIGHT = 0.14

COLUMN_WIDTH = 0.14
COLUMN_DEPTH = 0.20
COLUMN_CENTER_X = 0.47
COLUMN_PLINTH_WIDTH = 0.24
COLUMN_PLINTH_DEPTH = 0.28
COLUMN_PLINTH_HEIGHT = 0.12
COLUMN_HEIGHT = 1.41

TOP_BEAM_WIDTH = 1.26
TOP_BEAM_DEPTH = 0.26
TOP_BEAM_HEIGHT = 0.20

GUIDE_SHOE_OUTER_X = 0.27
GUIDE_SHOE_OUTER_Y = 0.30
GUIDE_SHOE_HEIGHT = 0.62
GUIDE_INNER_X = COLUMN_WIDTH + 0.012
GUIDE_INNER_Y = COLUMN_DEPTH + 0.016
GUIDE_SIDE_WALL_X = (GUIDE_SHOE_OUTER_X - GUIDE_INNER_X) / 2.0
GUIDE_FACE_WALL_Y = (GUIDE_SHOE_OUTER_Y - GUIDE_INNER_Y) / 2.0
GUIDE_PAD_X = (GUIDE_INNER_X - COLUMN_WIDTH) / 2.0
GUIDE_PAD_DEPTH = 0.16
GUIDE_PAD_HEIGHT = 0.46

FRONT_PLATE_WIDTH = 0.96
FRONT_PLATE_DEPTH = 0.07
FRONT_PLATE_HEIGHT = 0.52
FRONT_PLATE_Y = GUIDE_SHOE_OUTER_Y / 2.0 + FRONT_PLATE_DEPTH / 2.0

STIFFENER_WIDTH = 2.0 * (COLUMN_CENTER_X - GUIDE_SHOE_OUTER_X / 2.0)
STIFFENER_DEPTH = 0.10
STIFFENER_HEIGHT = 0.08
STIFFENER_Y = 0.10
STIFFENER_Z = 0.18

CARRIAGE_REST_Z = 0.97
CARRIAGE_TRAVEL = 0.32


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_lift_carriage_module")

    frame_paint = model.material("frame_paint", rgba=(0.82, 0.68, 0.17, 1.0))
    guide_column_finish = model.material(
        "guide_column_finish", rgba=(0.27, 0.30, 0.33, 1.0)
    )
    carriage_paint = model.material("carriage_paint", rgba=(0.24, 0.39, 0.48, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((FRAME_WIDTH, FRAME_DEPTH, 1.91)),
        mass=1850.0,
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
    )

    front_rail_y = FRAME_DEPTH / 2.0 - BASE_RAIL_DEPTH / 2.0
    side_rail_x = FRAME_WIDTH / 2.0 - SIDE_RAIL_WIDTH / 2.0
    column_plinth_z = BASE_RAIL_HEIGHT + COLUMN_PLINTH_HEIGHT / 2.0
    column_z = BASE_RAIL_HEIGHT + COLUMN_PLINTH_HEIGHT + COLUMN_HEIGHT / 2.0
    top_beam_z = (
        BASE_RAIL_HEIGHT
        + COLUMN_PLINTH_HEIGHT
        + COLUMN_HEIGHT
        + TOP_BEAM_HEIGHT / 2.0
    )

    frame.visual(
        Box((FRAME_WIDTH, BASE_RAIL_DEPTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, front_rail_y, BASE_RAIL_HEIGHT / 2.0)),
        material=frame_paint,
        name="front_base_rail",
    )
    frame.visual(
        Box((FRAME_WIDTH, BASE_RAIL_DEPTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, -front_rail_y, BASE_RAIL_HEIGHT / 2.0)),
        material=frame_paint,
        name="rear_base_rail",
    )
    frame.visual(
        Box((SIDE_RAIL_WIDTH, INNER_STRINGER_LENGTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(-side_rail_x, 0.0, BASE_RAIL_HEIGHT / 2.0)),
        material=frame_paint,
        name="left_base_rail",
    )
    frame.visual(
        Box((SIDE_RAIL_WIDTH, INNER_STRINGER_LENGTH, BASE_RAIL_HEIGHT)),
        origin=Origin(xyz=(side_rail_x, 0.0, BASE_RAIL_HEIGHT / 2.0)),
        material=frame_paint,
        name="right_base_rail",
    )
    frame.visual(
        Box((INNER_STRINGER_WIDTH, INNER_STRINGER_LENGTH, INNER_STRINGER_HEIGHT)),
        origin=Origin(xyz=(-INNER_STRINGER_X, 0.0, INNER_STRINGER_HEIGHT / 2.0)),
        material=guide_column_finish,
        name="left_inner_stringer",
    )
    frame.visual(
        Box((INNER_STRINGER_WIDTH, INNER_STRINGER_LENGTH, INNER_STRINGER_HEIGHT)),
        origin=Origin(xyz=(INNER_STRINGER_X, 0.0, INNER_STRINGER_HEIGHT / 2.0)),
        material=guide_column_finish,
        name="right_inner_stringer",
    )
    frame.visual(
        Box((CENTER_BASE_BEAM_WIDTH, CENTER_BASE_BEAM_DEPTH, CENTER_BASE_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CENTER_BASE_BEAM_HEIGHT / 2.0)),
        material=guide_column_finish,
        name="center_base_beam",
    )
    frame.visual(
        Box((COLUMN_PLINTH_WIDTH, COLUMN_PLINTH_DEPTH, COLUMN_PLINTH_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.0, column_plinth_z)),
        material=guide_column_finish,
        name="left_column_plinth",
    )
    frame.visual(
        Box((COLUMN_PLINTH_WIDTH, COLUMN_PLINTH_DEPTH, COLUMN_PLINTH_HEIGHT)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, column_plinth_z)),
        material=guide_column_finish,
        name="right_column_plinth",
    )
    frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(-COLUMN_CENTER_X, 0.0, column_z)),
        material=guide_column_finish,
        name="left_column",
    )
    frame.visual(
        Box((COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT)),
        origin=Origin(xyz=(COLUMN_CENTER_X, 0.0, column_z)),
        material=guide_column_finish,
        name="right_column",
    )
    frame.visual(
        Box((TOP_BEAM_WIDTH, TOP_BEAM_DEPTH, TOP_BEAM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, top_beam_z)),
        material=frame_paint,
        name="top_tie_beam",
    )

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((1.10, 0.30, GUIDE_SHOE_HEIGHT)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_guide_x = -COLUMN_CENTER_X
    right_guide_x = COLUMN_CENTER_X
    guide_front_y = GUIDE_INNER_Y / 2.0 + GUIDE_FACE_WALL_Y / 2.0
    guide_side_x = GUIDE_INNER_X / 2.0 + GUIDE_SIDE_WALL_X / 2.0
    guide_pad_x = COLUMN_WIDTH / 2.0 + GUIDE_PAD_X / 2.0

    carriage.visual(
        Box((GUIDE_SHOE_OUTER_X, GUIDE_FACE_WALL_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x, guide_front_y, 0.0)),
        material=carriage_paint,
        name="left_guide_front_wall",
    )
    carriage.visual(
        Box((GUIDE_SHOE_OUTER_X, GUIDE_FACE_WALL_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x, -guide_front_y, 0.0)),
        material=carriage_paint,
        name="left_guide_rear_wall",
    )
    carriage.visual(
        Box((GUIDE_SIDE_WALL_X, GUIDE_INNER_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x + guide_side_x, 0.0, 0.0)),
        material=carriage_paint,
        name="left_guide_inboard_wall",
    )
    carriage.visual(
        Box((GUIDE_SIDE_WALL_X, GUIDE_INNER_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(left_guide_x - guide_side_x, 0.0, 0.0)),
        material=carriage_paint,
        name="left_guide_outboard_wall",
    )
    carriage.visual(
        Box((GUIDE_SHOE_OUTER_X, GUIDE_FACE_WALL_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x, guide_front_y, 0.0)),
        material=carriage_paint,
        name="right_guide_front_wall",
    )
    carriage.visual(
        Box((GUIDE_SHOE_OUTER_X, GUIDE_FACE_WALL_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x, -guide_front_y, 0.0)),
        material=carriage_paint,
        name="right_guide_rear_wall",
    )
    carriage.visual(
        Box((GUIDE_SIDE_WALL_X, GUIDE_INNER_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x - guide_side_x, 0.0, 0.0)),
        material=carriage_paint,
        name="right_guide_inboard_wall",
    )
    carriage.visual(
        Box((GUIDE_SIDE_WALL_X, GUIDE_INNER_Y, GUIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(right_guide_x + guide_side_x, 0.0, 0.0)),
        material=carriage_paint,
        name="right_guide_outboard_wall",
    )
    carriage.visual(
        Box((GUIDE_PAD_X, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(left_guide_x + guide_pad_x, 0.0, 0.0)),
        material=guide_column_finish,
        name="left_guide_inboard_pad",
    )
    carriage.visual(
        Box((GUIDE_PAD_X, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(left_guide_x - guide_pad_x, 0.0, 0.0)),
        material=guide_column_finish,
        name="left_guide_outboard_pad",
    )
    carriage.visual(
        Box((GUIDE_PAD_X, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(right_guide_x - guide_pad_x, 0.0, 0.0)),
        material=guide_column_finish,
        name="right_guide_inboard_pad",
    )
    carriage.visual(
        Box((GUIDE_PAD_X, GUIDE_PAD_DEPTH, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(right_guide_x + guide_pad_x, 0.0, 0.0)),
        material=guide_column_finish,
        name="right_guide_outboard_pad",
    )
    carriage.visual(
        Box((FRONT_PLATE_WIDTH, FRONT_PLATE_DEPTH, FRONT_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, FRONT_PLATE_Y, 0.0)),
        material=carriage_paint,
        name="front_plate",
    )
    carriage.visual(
        Box((STIFFENER_WIDTH, STIFFENER_DEPTH, STIFFENER_HEIGHT)),
        origin=Origin(xyz=(0.0, STIFFENER_Y, STIFFENER_Z)),
        material=carriage_paint,
        name="upper_stiffener",
    )
    carriage.visual(
        Box((STIFFENER_WIDTH, STIFFENER_DEPTH, STIFFENER_HEIGHT)),
        origin=Origin(xyz=(0.0, STIFFENER_Y, -STIFFENER_Z)),
        material=carriage_paint,
        name="lower_stiffener",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20000.0,
            velocity=0.40,
            lower=-CARRIAGE_TRAVEL,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("frame_to_carriage")

    left_column = frame.get_visual("left_column")
    right_column = frame.get_visual("right_column")
    top_tie_beam = frame.get_visual("top_tie_beam")
    center_base_beam = frame.get_visual("center_base_beam")

    left_guide_front_wall = carriage.get_visual("left_guide_front_wall")
    left_guide_rear_wall = carriage.get_visual("left_guide_rear_wall")
    left_guide_inboard_wall = carriage.get_visual("left_guide_inboard_wall")
    left_guide_outboard_wall = carriage.get_visual("left_guide_outboard_wall")
    right_guide_front_wall = carriage.get_visual("right_guide_front_wall")
    right_guide_rear_wall = carriage.get_visual("right_guide_rear_wall")
    right_guide_inboard_wall = carriage.get_visual("right_guide_inboard_wall")
    right_guide_outboard_wall = carriage.get_visual("right_guide_outboard_wall")
    left_guide_inboard_pad = carriage.get_visual("left_guide_inboard_pad")
    left_guide_outboard_pad = carriage.get_visual("left_guide_outboard_pad")
    right_guide_inboard_pad = carriage.get_visual("right_guide_inboard_pad")
    right_guide_outboard_pad = carriage.get_visual("right_guide_outboard_pad")
    front_plate = carriage.get_visual("front_plate")

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

    limits = lift.motion_limits
    ctx.check(
        "parts_present",
        frame.name == "frame" and carriage.name == "carriage",
        details="Frame and carriage parts should both exist.",
    )
    ctx.check(
        "lift_joint_vertical",
        lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0),
        details=f"Expected one vertical prismatic lift joint, got type={lift.articulation_type} axis={lift.axis}.",
    )
    ctx.check(
        "lift_joint_has_travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and (limits.upper - limits.lower) >= 0.60,
        details=f"Expected substantial bidirectional carriage travel, got limits={limits}.",
    )
    ctx.expect_origin_distance(
        carriage,
        frame,
        axes="xy",
        max_dist=0.001,
        name="carriage_centered_between_uprights",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=left_column,
        elem_b=left_guide_inboard_pad,
        contact_tol=1e-6,
        name="left_column_contacts_inboard_wear_pad",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=right_column,
        elem_b=right_guide_inboard_pad,
        contact_tol=1e-6,
        name="right_column_contacts_inboard_wear_pad",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=left_column,
        elem_b=left_guide_outboard_pad,
        contact_tol=1e-6,
        name="left_column_contacts_outboard_wear_pad",
    )
    ctx.expect_contact(
        frame,
        carriage,
        elem_a=right_column,
        elem_b=right_guide_outboard_pad,
        contact_tol=1e-6,
        name="right_column_contacts_outboard_wear_pad",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem=left_guide_front_wall,
        negative_elem=left_column,
        min_gap=0.007,
        max_gap=0.009,
        name="left_front_guide_wall_clears_column",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem=left_column,
        negative_elem=left_guide_rear_wall,
        min_gap=0.007,
        max_gap=0.009,
        name="left_rear_guide_wall_clears_column",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem=left_guide_inboard_wall,
        negative_elem=left_column,
        min_gap=0.005,
        max_gap=0.007,
        name="left_inboard_guide_wall_clears_column",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem=left_column,
        negative_elem=left_guide_outboard_wall,
        min_gap=0.005,
        max_gap=0.007,
        name="left_outboard_guide_wall_clears_column",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem=right_guide_front_wall,
        negative_elem=right_column,
        min_gap=0.007,
        max_gap=0.009,
        name="right_front_guide_wall_clears_column",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem=right_column,
        negative_elem=right_guide_rear_wall,
        min_gap=0.007,
        max_gap=0.009,
        name="right_rear_guide_wall_clears_column",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem=right_column,
        negative_elem=right_guide_inboard_wall,
        min_gap=0.005,
        max_gap=0.007,
        name="right_inboard_guide_wall_clears_column",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem=right_guide_outboard_wall,
        negative_elem=right_column,
        min_gap=0.005,
        max_gap=0.007,
        name="right_outboard_guide_wall_clears_column",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({lift: limits.lower}):
            ctx.expect_gap(
                carriage,
                frame,
                axis="z",
                positive_elem=front_plate,
                negative_elem=center_base_beam,
                min_gap=0.12,
                max_gap=0.35,
                name="lower_pose_plate_clears_base_frame",
            )
            ctx.expect_contact(
                frame,
                carriage,
                elem_a=left_column,
                elem_b=left_guide_inboard_pad,
                contact_tol=1e-6,
                name="lower_pose_left_guidance_contact",
            )
            ctx.expect_contact(
                frame,
                carriage,
                elem_a=right_column,
                elem_b=right_guide_inboard_pad,
                contact_tol=1e-6,
                name="lower_pose_right_guidance_contact",
            )

        with ctx.pose({lift: limits.upper}):
            ctx.expect_gap(
                frame,
                carriage,
                axis="z",
                positive_elem=top_tie_beam,
                negative_elem=front_plate,
                min_gap=0.10,
                max_gap=0.30,
                name="upper_pose_plate_clears_tie_beam",
            )
            ctx.expect_contact(
                frame,
                carriage,
                elem_a=left_column,
                elem_b=left_guide_inboard_pad,
                contact_tol=1e-6,
                name="upper_pose_left_guidance_contact",
            )
            ctx.expect_contact(
                frame,
                carriage,
                elem_a=right_column,
                elem_b=right_guide_inboard_pad,
                contact_tol=1e-6,
                name="upper_pose_right_guidance_contact",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
