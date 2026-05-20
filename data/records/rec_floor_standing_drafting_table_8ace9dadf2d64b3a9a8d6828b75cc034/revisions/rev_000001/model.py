from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_standing_drafting_table")

    frame_steel = model.material("frame_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    carriage_steel = model.material("carriage_steel", rgba=(0.38, 0.40, 0.44, 1.0))
    board_wood = model.material("board_wood", rgba=(0.73, 0.61, 0.43, 1.0))
    board_trim = model.material("board_trim", rgba=(0.16, 0.17, 0.19, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.20, 0.22, 0.25, 1.0))
    hardware = model.material("hardware", rgba=(0.70, 0.72, 0.75, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.12, 0.56, 0.05)),
        origin=Origin(xyz=(-0.35, 0.0, 0.025)),
        material=frame_steel,
        name="left_foot",
    )
    frame.visual(
        Box((0.12, 0.56, 0.05)),
        origin=Origin(xyz=(0.35, 0.0, 0.025)),
        material=frame_steel,
        name="right_foot",
    )
    frame.visual(
        Box((0.06, 0.08, 0.92)),
        origin=Origin(xyz=(-0.35, 0.0, 0.51)),
        material=frame_steel,
        name="left_post",
    )
    frame.visual(
        Box((0.06, 0.08, 0.92)),
        origin=Origin(xyz=(0.35, 0.0, 0.51)),
        material=frame_steel,
        name="right_post",
    )
    frame.visual(
        Box((0.70, 0.06, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.135)),
        material=frame_steel,
        name="lower_stretcher",
    )
    frame.visual(
        Box((0.10, 0.12, 0.80)),
        origin=Origin(xyz=(0.0, 0.0, 0.57)),
        material=carriage_steel,
        name="center_guide_rail",
    )
    frame.visual(
        Box((0.70, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
        material=frame_steel,
        name="top_bridge",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.82, 0.56, 1.00)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.04, 0.15, 0.22)),
        origin=Origin(xyz=(-0.07, 0.0, 0.0)),
        material=carriage_steel,
        name="left_inner_block",
    )
    carriage.visual(
        Box((0.04, 0.15, 0.22)),
        origin=Origin(xyz=(0.07, 0.0, 0.0)),
        material=carriage_steel,
        name="right_inner_block",
    )
    carriage.visual(
        Box((0.18, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, 0.08, 0.0)),
        material=carriage_steel,
        name="front_bridge",
    )
    carriage.visual(
        Box((0.18, 0.02, 0.12)),
        origin=Origin(xyz=(0.0, -0.08, 0.0)),
        material=carriage_steel,
        name="rear_bridge",
    )
    carriage.visual(
        Box((0.24, 0.05, 0.05)),
        origin=Origin(xyz=(-0.18, -0.085, -0.01)),
        material=carriage_steel,
        name="left_outboard_arm",
    )
    carriage.visual(
        Box((0.24, 0.05, 0.05)),
        origin=Origin(xyz=(0.18, -0.085, -0.01)),
        material=carriage_steel,
        name="right_outboard_arm",
    )
    carriage.visual(
        Box((0.05, 0.08, 0.18)),
        origin=Origin(xyz=(-0.27, -0.04, 0.05)),
        material=carriage_steel,
        name="left_yoke",
    )
    carriage.visual(
        Box((0.05, 0.08, 0.18)),
        origin=Origin(xyz=(0.27, -0.04, 0.05)),
        material=carriage_steel,
        name="right_yoke",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.64, 0.24, 0.24)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    board = model.part("board")
    board.visual(
        Box((0.06, 0.08, 0.20)),
        origin=Origin(xyz=(-0.27, 0.04, 0.0)),
        material=board_trim,
        name="left_hinge_rail",
    )
    board.visual(
        Box((0.06, 0.08, 0.20)),
        origin=Origin(xyz=(0.27, 0.04, 0.0)),
        material=board_trim,
        name="right_hinge_rail",
    )
    board.visual(
        Box((0.60, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.105, 0.03)),
        material=board_trim,
        name="rear_hinge_beam",
    )
    board.visual(
        Box((0.92, 0.64, 0.028)),
        origin=Origin(xyz=(0.0, 0.40, 0.046)),
        material=board_wood,
        name="board_panel",
    )
    board.visual(
        Box((0.03, 0.64, 0.04)),
        origin=Origin(xyz=(-0.445, 0.40, 0.046)),
        material=board_trim,
        name="left_trim",
    )
    board.visual(
        Box((0.03, 0.64, 0.04)),
        origin=Origin(xyz=(0.445, 0.40, 0.046)),
        material=board_trim,
        name="right_trim",
    )
    board.visual(
        Box((0.03, 0.38, 0.05)),
        origin=Origin(xyz=(-0.29, 0.53, -0.035)),
        material=board_trim,
        name="left_runner",
    )
    board.visual(
        Box((0.03, 0.38, 0.05)),
        origin=Origin(xyz=(0.29, 0.53, -0.035)),
        material=board_trim,
        name="right_runner",
    )
    board.visual(
        Box((0.66, 0.05, 0.08)),
        origin=Origin(xyz=(0.0, 0.34, 0.0)),
        material=board_trim,
        name="rear_drawer_crossmember",
    )
    board.visual(
        Box((0.74, 0.04, 0.10)),
        origin=Origin(xyz=(0.0, 0.724, -0.002)),
        material=board_trim,
        name="front_apron",
    )
    board.inertial = Inertial.from_geometry(
        Box((0.96, 0.84, 0.18)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.42, 0.0)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.48, 0.22, 0.008)),
        origin=Origin(xyz=(0.0, 0.11, -0.04)),
        material=drawer_finish,
        name="drawer_base",
    )
    drawer.visual(
        Box((0.01, 0.22, 0.05)),
        origin=Origin(xyz=(-0.235, 0.11, -0.015)),
        material=drawer_finish,
        name="left_wall",
    )
    drawer.visual(
        Box((0.01, 0.22, 0.05)),
        origin=Origin(xyz=(0.235, 0.11, -0.015)),
        material=drawer_finish,
        name="right_wall",
    )
    drawer.visual(
        Box((0.48, 0.01, 0.05)),
        origin=Origin(xyz=(0.0, 0.005, -0.015)),
        material=drawer_finish,
        name="back_wall",
    )
    drawer.visual(
        Box((0.50, 0.024, 0.07)),
        origin=Origin(xyz=(0.0, 0.216, -0.008)),
        material=drawer_finish,
        name="drawer_face",
    )
    drawer.inertial = Inertial.from_geometry(
        Box((0.58, 0.20, 0.08)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.11, -0.01)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.20,
            lower=-0.10,
            upper=0.18,
        ),
    )

    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.0,
            lower=-0.08,
            upper=0.95,
        ),
    )

    model.articulation(
        "board_to_drawer",
        ArticulationType.PRISMATIC,
        parent=board,
        child=drawer,
        origin=Origin(xyz=(0.0, 0.476, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.18,
            lower=0.0,
            upper=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    drawer = object_model.get_part("drawer")

    lift = object_model.get_articulation("frame_to_carriage")
    tilt = object_model.get_articulation("carriage_to_board")
    slide = object_model.get_articulation("board_to_drawer")

    left_hinge_rail = board.get_visual("left_hinge_rail")
    right_hinge_rail = board.get_visual("right_hinge_rail")
    board_panel = board.get_visual("board_panel")
    front_apron = board.get_visual("front_apron")
    drawer_base = drawer.get_visual("drawer_base")
    drawer_face = drawer.get_visual("drawer_face")
    left_yoke = carriage.get_visual("left_yoke")
    right_yoke = carriage.get_visual("right_yoke")
    lower_stretcher = frame.get_visual("lower_stretcher")

    ctx.check("frame part exists", frame is not None)
    ctx.check("carriage part exists", carriage is not None)
    ctx.check("board part exists", board is not None)
    ctx.check("drawer part exists", drawer is not None)

    ctx.expect_contact(
        board,
        carriage,
        elem_a=left_hinge_rail,
        elem_b=left_yoke,
        name="left hinge rail is seated on the left carriage yoke",
    )
    ctx.expect_contact(
        board,
        carriage,
        elem_a=right_hinge_rail,
        elem_b=right_yoke,
        name="right hinge rail is seated on the right carriage yoke",
    )

    ctx.expect_within(
        drawer,
        board,
        axes="x",
        inner_elem=drawer_face,
        outer_elem=front_apron,
        margin=0.0,
        name="drawer face stays centered under the board",
    )
    ctx.expect_contact(
        drawer,
        board,
        elem_a=drawer_face,
        elem_b=front_apron,
        name="closed drawer face closes flush against the front apron",
    )
    ctx.expect_overlap(
        drawer,
        board,
        axes="y",
        elem_a=drawer_base,
        elem_b=board_panel,
        min_overlap=0.16,
        name="closed drawer remains stowed under the drafting board",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.18}):
        raised_carriage_pos = ctx.part_world_position(carriage)
    ctx.check(
        "center carriage raises along the guide rail",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.15,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
    with ctx.pose({tilt: 0.80}):
        tilted_panel_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
    ctx.check(
        "board tilts upward about the carriage hinge",
        rest_panel_aabb is not None
        and tilted_panel_aabb is not None
        and tilted_panel_aabb[1][2] > rest_panel_aabb[1][2] + 0.20,
        details=f"rest={rest_panel_aabb}, tilted={tilted_panel_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({slide: 0.16}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            board,
            axes="y",
            elem_a=drawer_base,
            elem_b=board_panel,
            min_overlap=0.05,
            name="extended drawer retains insertion under the board",
        )
    ctx.check(
        "tool drawer slides outward from the working edge",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] > rest_drawer_pos[1] + 0.12,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    with ctx.pose({lift: -0.10, tilt: -0.08}):
        ctx.expect_gap(
            board,
            frame,
            axis="z",
            positive_elem=front_apron,
            negative_elem=lower_stretcher,
            min_gap=0.01,
            name="board clears the lower stretcher at the lowest working pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
