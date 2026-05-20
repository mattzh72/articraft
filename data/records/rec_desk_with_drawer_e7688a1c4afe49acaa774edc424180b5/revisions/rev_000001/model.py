from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq


DESK_WIDTH = 1.70
DESK_DEPTH = 0.82
DESK_HEIGHT = 0.76
TOP_THICKNESS = 0.04
UNDERSIDE_Z = DESK_HEIGHT - TOP_THICKNESS

PEDESTAL_WIDTH = 0.48
PEDESTAL_DEPTH = 0.72
PEDESTAL_HEIGHT = UNDERSIDE_Z
PEDESTAL_CENTER_X = -0.02
PEDESTAL_CENTER_Y = -(DESK_WIDTH / 2.0 - PEDESTAL_WIDTH / 2.0)
PEDESTAL_WALL = 0.018
PEDESTAL_BACK = 0.016
PEDESTAL_TOP_BOTTOM = 0.018
PEDESTAL_INNER_WIDTH = PEDESTAL_WIDTH - 2.0 * PEDESTAL_WALL
PEDESTAL_FRONT_X = PEDESTAL_CENTER_X + PEDESTAL_DEPTH / 2.0
PEDESTAL_TOP_Z = PEDESTAL_HEIGHT - PEDESTAL_TOP_BOTTOM / 2.0
PEDESTAL_BOTTOM_Z = PEDESTAL_TOP_BOTTOM / 2.0

RIGHT_POST_Y = DESK_WIDTH / 2.0 - 0.15
RIGHT_POST_X_FRONT = DESK_DEPTH / 2.0 - 0.09
RIGHT_POST_X_REAR = -(DESK_DEPTH / 2.0 - 0.11)
POST_SIZE = 0.05

SHELF_CENTER_Y = 0.12
SHELF_WIDTH = 0.74
SHELF_GUIDE_LENGTH = 0.56
SHELF_GUIDE_THICKNESS = 0.03
SHELF_GUIDE_HEIGHT = 0.04
SHELF_GUIDE_FRONT_X = DESK_DEPTH / 2.0 - 0.06
SHELF_GUIDE_CENTER_X = SHELF_GUIDE_FRONT_X - SHELF_GUIDE_LENGTH / 2.0

DRAWER_OPENINGS = (0.138, 0.178, 0.332)
DRAWER_SEPARATOR = 0.018
DRAWER_BODY_DEPTH = 0.62
DRAWER_BODY_WIDTH = PEDESTAL_INNER_WIDTH - 0.028
DRAWER_SIDE_WALL = 0.012
DRAWER_BOTTOM = 0.012
DRAWER_FACE = 0.018
DRAWER_FRONT_PROUD = 0.001
PART_CONNECTION_OVERLAP = 0.004
DRAWER_RUNNER_LENGTH = 0.56
DRAWER_RUNNER_THICKNESS = 0.014
DRAWER_RUNNER_HEIGHT = 0.022

SHELF_DEPTH = 0.48
SHELF_THICKNESS = 0.025
SHELF_LIP_THICKNESS = 0.015
SHELF_LIP_HEIGHT = 0.035
SHELF_RUNNER_THICKNESS = 0.014
SHELF_RUNNER_HEIGHT = 0.028
SHELF_CENTER_Z = 0.635


def add_box(part, size, xyz, material, name):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_workstation_desk")

    top_wood = model.material("top_wood", rgba=(0.78, 0.69, 0.50, 1.0))
    cabinet_gray = model.material("cabinet_gray", rgba=(0.36, 0.38, 0.42, 1.0))
    frame_steel = model.material("frame_steel", rgba=(0.18, 0.19, 0.20, 1.0))

    desk_base = model.part("desk_base")

    add_box(
        desk_base,
        (DESK_DEPTH, DESK_WIDTH, TOP_THICKNESS),
        (0.0, 0.0, DESK_HEIGHT - TOP_THICKNESS / 2.0),
        top_wood,
        "top",
    )

    pedestal_left_y = PEDESTAL_CENTER_Y - (PEDESTAL_WIDTH / 2.0 - PEDESTAL_WALL / 2.0)
    pedestal_right_y = PEDESTAL_CENTER_Y + (PEDESTAL_WIDTH / 2.0 - PEDESTAL_WALL / 2.0)
    add_box(
        desk_base,
        (PEDESTAL_DEPTH, PEDESTAL_WALL, PEDESTAL_HEIGHT),
        (PEDESTAL_CENTER_X, pedestal_left_y, PEDESTAL_HEIGHT / 2.0),
        cabinet_gray,
        "pedestal_side_outer",
    )
    add_box(
        desk_base,
        (PEDESTAL_DEPTH, PEDESTAL_WALL, PEDESTAL_HEIGHT),
        (PEDESTAL_CENTER_X, pedestal_right_y, PEDESTAL_HEIGHT / 2.0),
        cabinet_gray,
        "pedestal_side_inner",
    )
    add_box(
        desk_base,
        (PEDESTAL_BACK, PEDESTAL_INNER_WIDTH, PEDESTAL_HEIGHT - 2.0 * PEDESTAL_TOP_BOTTOM),
        (
            PEDESTAL_CENTER_X - PEDESTAL_DEPTH / 2.0 + PEDESTAL_BACK / 2.0,
            PEDESTAL_CENTER_Y,
            PEDESTAL_HEIGHT / 2.0,
        ),
        cabinet_gray,
        "pedestal_back",
    )
    add_box(
        desk_base,
        (PEDESTAL_DEPTH, PEDESTAL_WIDTH, PEDESTAL_TOP_BOTTOM),
        (PEDESTAL_CENTER_X, PEDESTAL_CENTER_Y, PEDESTAL_BOTTOM_Z),
        cabinet_gray,
        "pedestal_bottom",
    )
    add_box(
        desk_base,
        (PEDESTAL_DEPTH, PEDESTAL_WIDTH, PEDESTAL_TOP_BOTTOM),
        (PEDESTAL_CENTER_X, PEDESTAL_CENTER_Y, PEDESTAL_TOP_Z),
        cabinet_gray,
        "pedestal_top",
    )

    separator_0_bottom = PEDESTAL_TOP_BOTTOM + DRAWER_OPENINGS[0]
    separator_1_bottom = separator_0_bottom + DRAWER_SEPARATOR + DRAWER_OPENINGS[1]
    separator_depth_center_x = PEDESTAL_CENTER_X - PEDESTAL_BACK / 2.0 + PEDESTAL_DEPTH / 2.0
    add_box(
        desk_base,
        (
            PEDESTAL_DEPTH - PEDESTAL_BACK,
            PEDESTAL_INNER_WIDTH,
            DRAWER_SEPARATOR,
        ),
        (
            separator_depth_center_x,
            PEDESTAL_CENTER_Y,
            separator_0_bottom + DRAWER_SEPARATOR / 2.0,
        ),
        cabinet_gray,
        "separator_0",
    )
    add_box(
        desk_base,
        (
            PEDESTAL_DEPTH - PEDESTAL_BACK,
            PEDESTAL_INNER_WIDTH,
            DRAWER_SEPARATOR,
        ),
        (
            separator_depth_center_x,
            PEDESTAL_CENTER_Y,
            separator_1_bottom + DRAWER_SEPARATOR / 2.0,
        ),
        cabinet_gray,
        "separator_1",
    )

    add_box(
        desk_base,
        (POST_SIZE, POST_SIZE, PEDESTAL_HEIGHT),
        (RIGHT_POST_X_FRONT, RIGHT_POST_Y, PEDESTAL_HEIGHT / 2.0),
        frame_steel,
        "leg_post_front",
    )
    add_box(
        desk_base,
        (POST_SIZE, POST_SIZE, PEDESTAL_HEIGHT),
        (RIGHT_POST_X_REAR, RIGHT_POST_Y, PEDESTAL_HEIGHT / 2.0),
        frame_steel,
        "leg_post_rear",
    )
    add_box(
        desk_base,
        (
            RIGHT_POST_X_FRONT - RIGHT_POST_X_REAR,
            POST_SIZE,
            0.06,
        ),
        (
            (RIGHT_POST_X_FRONT + RIGHT_POST_X_REAR) / 2.0,
            RIGHT_POST_Y,
            0.09,
        ),
        frame_steel,
        "leg_stretcher_low",
    )
    add_box(
        desk_base,
        (0.05, RIGHT_POST_Y - PEDESTAL_CENTER_Y, 0.20),
        (
            -DESK_DEPTH / 2.0 + 0.035,
            (RIGHT_POST_Y + PEDESTAL_CENTER_Y) / 2.0,
            0.45,
        ),
        frame_steel,
        "rear_beam",
    )

    shelf_guide_y_offset = SHELF_WIDTH / 2.0 + SHELF_GUIDE_THICKNESS / 2.0 + 0.012
    for index, sign in enumerate((-1.0, 1.0)):
        add_box(
            desk_base,
            (SHELF_GUIDE_LENGTH, SHELF_GUIDE_THICKNESS, SHELF_GUIDE_HEIGHT),
            (
                SHELF_GUIDE_CENTER_X,
                SHELF_CENTER_Y + sign * shelf_guide_y_offset,
                UNDERSIDE_Z - SHELF_GUIDE_HEIGHT / 2.0,
            ),
            frame_steel,
            f"shelf_guide_{index}",
        )

    drawer_opening_centers = []
    drawer_floor = PEDESTAL_TOP_BOTTOM
    runner_center_x = PEDESTAL_FRONT_X - 0.05 - DRAWER_RUNNER_LENGTH / 2.0
    runner_y_offset = PEDESTAL_INNER_WIDTH / 2.0 - DRAWER_RUNNER_THICKNESS / 2.0
    for index, opening_height in enumerate(DRAWER_OPENINGS):
        opening_center_z = drawer_floor + opening_height / 2.0
        drawer_opening_centers.append(opening_center_z)
        for side_index, side_sign in enumerate((-1.0, 1.0)):
            add_box(
                desk_base,
                (DRAWER_RUNNER_LENGTH, DRAWER_RUNNER_THICKNESS, DRAWER_RUNNER_HEIGHT),
                (
                    runner_center_x,
                    PEDESTAL_CENTER_Y + side_sign * runner_y_offset,
                    opening_center_z,
                ),
                frame_steel,
                f"drawer_track_{index}_{side_index}",
            )
        drawer_floor += opening_height + DRAWER_SEPARATOR

    drawer_front = model.material("drawer_front", rgba=(0.70, 0.72, 0.75, 1.0))
    handle_dark = model.material("handle_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    shelf_dark = model.material("shelf_dark", rgba=(0.20, 0.21, 0.22, 1.0))

    drawer_body_depth_inner = DRAWER_BODY_DEPTH - DRAWER_FACE + PART_CONNECTION_OVERLAP
    drawer_body_front_x = DRAWER_FRONT_PROUD + PART_CONNECTION_OVERLAP
    drawer_body_center_x = drawer_body_front_x - drawer_body_depth_inner / 2.0
    drawer_back_center_x = drawer_body_front_x - drawer_body_depth_inner - DRAWER_SIDE_WALL / 2.0
    for index, (opening_height, opening_center_z) in enumerate(zip(DRAWER_OPENINGS, drawer_opening_centers)):
        drawer = model.part(f"drawer_{index}")
        face_height = opening_height - 0.006
        box_height = opening_height - 0.026
        handle_width = min(0.30, PEDESTAL_WIDTH - 0.20)

        add_box(
            drawer,
            (DRAWER_FACE, PEDESTAL_WIDTH - 0.008, face_height),
            (DRAWER_FACE / 2.0 + DRAWER_FRONT_PROUD, 0.0, 0.0),
            drawer_front,
            "front",
        )
        add_box(
            drawer,
            (0.012, handle_width, 0.018),
            (0.023, 0.0, face_height / 2.0 - 0.030),
            handle_dark,
            "pull",
        )
        add_box(
            drawer,
            (drawer_body_depth_inner, DRAWER_SIDE_WALL, box_height),
            (
                drawer_body_center_x,
                -(DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_WALL / 2.0),
                0.0,
            ),
            cabinet_gray,
            "side_0",
        )
        add_box(
            drawer,
            (drawer_body_depth_inner, DRAWER_SIDE_WALL, box_height),
            (
                drawer_body_center_x,
                DRAWER_BODY_WIDTH / 2.0 - DRAWER_SIDE_WALL / 2.0,
                0.0,
            ),
            cabinet_gray,
            "side_1",
        )
        add_box(
            drawer,
            (
                drawer_body_depth_inner,
                DRAWER_BODY_WIDTH - 2.0 * DRAWER_SIDE_WALL,
                DRAWER_BOTTOM,
            ),
            (
                drawer_body_center_x,
                0.0,
                -box_height / 2.0 + DRAWER_BOTTOM / 2.0,
            ),
            cabinet_gray,
            "bottom",
        )
        add_box(
            drawer,
            (
                DRAWER_SIDE_WALL,
                DRAWER_BODY_WIDTH - 2.0 * DRAWER_SIDE_WALL,
                box_height,
            ),
            (drawer_back_center_x, 0.0, 0.0),
            cabinet_gray,
            "back",
        )

        model.articulation(
            f"desk_to_drawer_{index}",
            ArticulationType.PRISMATIC,
            parent=desk_base,
            child=drawer,
            origin=Origin(xyz=(PEDESTAL_FRONT_X, PEDESTAL_CENTER_Y, opening_center_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.35,
                lower=0.0,
                upper=0.28,
            ),
        )

    pullout_shelf = model.part("pullout_shelf")
    shelf_body_center_x = (DRAWER_FRONT_PROUD - SHELF_DEPTH) / 2.0
    add_box(
        pullout_shelf,
        (SHELF_DEPTH, SHELF_WIDTH, SHELF_THICKNESS),
        (shelf_body_center_x, 0.0, 0.0),
        shelf_dark,
        "board",
    )
    add_box(
        pullout_shelf,
        (SHELF_LIP_THICKNESS, SHELF_WIDTH - 0.05, SHELF_LIP_HEIGHT),
        (
            DRAWER_FRONT_PROUD + SHELF_LIP_THICKNESS / 2.0 - PART_CONNECTION_OVERLAP,
            0.0,
            SHELF_THICKNESS / 2.0 + SHELF_LIP_HEIGHT / 2.0 - PART_CONNECTION_OVERLAP,
        ),
        shelf_dark,
        "front_lip",
    )
    for index, sign in enumerate((-1.0, 1.0)):
        add_box(
            pullout_shelf,
            (SHELF_DEPTH - 0.04, 0.012, 0.016),
            (
                (DRAWER_FRONT_PROUD - (SHELF_DEPTH - 0.04)) / 2.0,
                sign * (SHELF_WIDTH / 2.0 - 0.007),
                SHELF_THICKNESS / 2.0 + 0.0045,
            ),
            shelf_dark,
            f"cleat_{index}",
        )
        add_box(
            pullout_shelf,
            (SHELF_DEPTH - 0.02, SHELF_RUNNER_THICKNESS, SHELF_RUNNER_HEIGHT),
            (
                (DRAWER_FRONT_PROUD - (SHELF_DEPTH - 0.02)) / 2.0,
                sign * (SHELF_WIDTH / 2.0 + 0.005),
                SHELF_THICKNESS / 2.0 + SHELF_RUNNER_HEIGHT / 2.0 + 0.0045,
            ),
            shelf_dark,
            f"runner_{index}",
        )

    model.articulation(
        "desk_to_pullout_shelf",
        ArticulationType.PRISMATIC,
        parent=desk_base,
        child=pullout_shelf,
        origin=Origin(xyz=(SHELF_GUIDE_FRONT_X, SHELF_CENTER_Y, SHELF_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.26,
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    desk_base = object_model.get_part("desk_base")
    shelf = object_model.get_part("pullout_shelf")
    shelf_joint = object_model.get_articulation("desk_to_pullout_shelf")
    drawer_pairs = [
        (
            object_model.get_part(f"drawer_{index}"),
            object_model.get_articulation(f"desk_to_drawer_{index}"),
        )
        for index in range(len(DRAWER_OPENINGS))
    ]

    for index, (drawer, _) in enumerate(drawer_pairs):
        ctx.expect_overlap(
            drawer,
            desk_base,
            axes="x",
            elem_a="side_0",
            elem_b=f"drawer_track_{index}_0",
            min_overlap=0.50,
            name=f"drawer_{index} remains seated on runner when closed",
        )

    ctx.expect_overlap(
        shelf,
        desk_base,
        axes="x",
        elem_a="runner_0",
        elem_b="shelf_guide_0",
        min_overlap=0.42,
        name="pullout shelf stays engaged with guide when closed",
    )
    ctx.expect_gap(
        desk_base,
        shelf,
        axis="z",
        positive_elem="top",
        negative_elem="front_lip",
        min_gap=0.02,
        name="pullout shelf clears underside of desktop when closed",
    )

    for index, (drawer, joint) in enumerate(drawer_pairs):
        closed_pos = ctx.part_world_position(drawer)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper}):
            open_pos = ctx.part_world_position(drawer)
            ctx.expect_overlap(
                drawer,
                desk_base,
                axes="x",
                elem_a="side_0",
                elem_b=f"drawer_track_{index}_0",
                min_overlap=0.23,
                name=f"drawer_{index} retains insertion at full extension",
            )
            ctx.check(
                f"drawer_{index} extends forward",
                closed_pos is not None
                and open_pos is not None
                and open_pos[0] > closed_pos[0] + 0.20,
                details=f"closed={closed_pos}, open={open_pos}",
            )

    shelf_closed = ctx.part_world_position(shelf)
    shelf_upper = shelf_joint.motion_limits.upper if shelf_joint.motion_limits is not None else None
    with ctx.pose({shelf_joint: shelf_upper}):
        shelf_open = ctx.part_world_position(shelf)
        ctx.expect_overlap(
            shelf,
            desk_base,
            axes="x",
            elem_a="runner_0",
            elem_b="shelf_guide_0",
            min_overlap=0.18,
            name="pullout shelf retains guide engagement when extended",
        )
        ctx.expect_gap(
            desk_base,
            shelf,
            axis="z",
            positive_elem="top",
            negative_elem="front_lip",
            min_gap=0.02,
            name="pullout shelf clears underside of desktop when extended",
        )
        ctx.check(
            "pullout shelf extends forward",
            shelf_closed is not None
            and shelf_open is not None
            and shelf_open[0] > shelf_closed[0] + 0.20,
            details=f"closed={shelf_closed}, open={shelf_open}",
        )

    return ctx.report()


object_model = build_object_model()
