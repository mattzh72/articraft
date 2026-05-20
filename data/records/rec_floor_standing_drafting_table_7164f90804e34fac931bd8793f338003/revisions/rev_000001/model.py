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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="teaching_studio_drafting_table")

    steel = model.material("steel", rgba=(0.22, 0.23, 0.25, 1.0))
    graphite = model.material("graphite", rgba=(0.33, 0.35, 0.38, 1.0))
    board_surface = model.material("board_surface", rgba=(0.89, 0.88, 0.83, 1.0))
    ledge_trim = model.material("ledge_trim", rgba=(0.45, 0.37, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.08, 0.74, 0.04)),
        origin=Origin(xyz=(-0.32, 0.0, 0.02)),
        material=steel,
        name="left_foot",
    )
    base_frame.visual(
        Box((0.08, 0.74, 0.04)),
        origin=Origin(xyz=(0.32, 0.0, 0.02)),
        material=steel,
        name="right_foot",
    )
    base_frame.visual(
        Box((0.60, 0.10, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=steel,
        name="base_tie",
    )
    base_frame.visual(
        Box((0.07, 0.08, 0.96)),
        origin=Origin(xyz=(-0.32, 0.09, 0.52)),
        material=steel,
        name="left_upright",
    )
    base_frame.visual(
        Box((0.07, 0.08, 0.96)),
        origin=Origin(xyz=(0.32, 0.09, 0.52)),
        material=steel,
        name="right_upright",
    )
    base_frame.visual(
        Box((0.71, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, 0.09, 1.03)),
        material=steel,
        name="top_beam",
    )
    base_frame.visual(
        Box((0.12, 0.12, 0.18)),
        origin=Origin(xyz=(0.0, -0.08, 0.09)),
        material=steel,
        name="rail_mount",
    )
    base_frame.visual(
        Box((0.08, 0.06, 1.08)),
        origin=Origin(xyz=(0.0, -0.10, 0.72)),
        material=graphite,
        name="rail_column",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.80, 0.80, 1.32)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
    )

    foot_bar = model.part("foot_bar")
    foot_bar.visual(
        Cylinder(radius=0.015, length=0.58),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="foot_axle",
    )
    foot_bar.visual(
        Box((0.54, 0.09, 0.025)),
        origin=Origin(xyz=(0.0, 0.045, -0.022)),
        material=rubber,
        name="foot_plank",
    )
    foot_bar.inertial = Inertial.from_geometry(
        Box((0.58, 0.12, 0.07)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.03, -0.015)),
    )

    model.articulation(
        "base_to_foot_bar",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=foot_bar,
        origin=Origin(xyz=(0.0, 0.07, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=-0.35,
            upper=0.45,
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.025, 0.08, 0.22)),
        origin=Origin(xyz=(-0.055, -0.10, 0.0)),
        material=steel,
        name="left_guide_pad",
    )
    carriage.visual(
        Box((0.025, 0.08, 0.22)),
        origin=Origin(xyz=(0.055, -0.10, 0.0)),
        material=steel,
        name="right_guide_pad",
    )
    carriage.visual(
        Box((0.18, 0.03, 0.22)),
        origin=Origin(xyz=(0.0, 0.055, 0.0)),
        material=steel,
        name="front_plate",
    )
    carriage.visual(
        Box((0.052, 0.15, 0.025)),
        origin=Origin(xyz=(-0.066, -0.025, 0.0975)),
        material=steel,
        name="upper_left_bridge",
    )
    carriage.visual(
        Box((0.052, 0.15, 0.025)),
        origin=Origin(xyz=(0.066, -0.025, 0.0975)),
        material=steel,
        name="upper_right_bridge",
    )
    carriage.visual(
        Box((0.052, 0.15, 0.025)),
        origin=Origin(xyz=(-0.066, -0.025, -0.0975)),
        material=steel,
        name="lower_left_bridge",
    )
    carriage.visual(
        Box((0.052, 0.15, 0.025)),
        origin=Origin(xyz=(0.066, -0.025, -0.0975)),
        material=steel,
        name="lower_right_bridge",
    )
    carriage.visual(
        Box((0.10, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.22, 0.0)),
        material=steel,
        name="board_boom",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.16),
        origin=Origin(xyz=(0.0, 0.40, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="pivot_axle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.22, 0.52, 0.24)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.12, 0.0)),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.10, 0.58)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.42,
        ),
    )

    board = model.part("board")
    board.visual(
        Box((0.82, 0.02, 1.08)),
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
        material=board_surface,
        name="board_panel",
    )
    board.visual(
        Box((0.70, 0.06, 0.025)),
        origin=Origin(xyz=(0.0, 0.065, -0.5275)),
        material=ledge_trim,
        name="paper_ledge",
    )
    board.inertial = Inertial.from_geometry(
        Box((0.82, 0.10, 1.10)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.04, 0.0)),
    )

    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.40, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=-0.20,
            upper=0.75,
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

    base_frame = object_model.get_part("base_frame")
    foot_bar = object_model.get_part("foot_bar")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")

    foot_joint = object_model.get_articulation("base_to_foot_bar")
    carriage_slide = object_model.get_articulation("base_to_carriage")
    board_tilt = object_model.get_articulation("carriage_to_board")

    ctx.check("base frame exists", base_frame is not None)
    ctx.check("foot bar exists", foot_bar is not None)
    ctx.check("carriage exists", carriage is not None)
    ctx.check("board exists", board is not None)
    ctx.check("foot articulation exists", foot_joint is not None)
    ctx.check("carriage slide exists", carriage_slide is not None)
    ctx.check("board tilt exists", board_tilt is not None)

    ctx.expect_gap(
        board,
        base_frame,
        axis="y",
        positive_elem="board_panel",
        negative_elem="rail_column",
        min_gap=0.30,
        name="board panel sits clearly in front of rail at rest",
    )
    ctx.expect_gap(
        base_frame,
        carriage,
        axis="x",
        positive_elem="rail_column",
        negative_elem="left_guide_pad",
        min_gap=0.001,
        max_gap=0.006,
        name="left carriage guide clears rail side",
    )
    ctx.expect_gap(
        carriage,
        base_frame,
        axis="x",
        positive_elem="right_guide_pad",
        negative_elem="rail_column",
        min_gap=0.001,
        max_gap=0.006,
        name="right carriage guide clears rail side",
    )
    ctx.expect_overlap(
        carriage,
        base_frame,
        axes="z",
        elem_a="left_guide_pad",
        elem_b="rail_column",
        min_overlap=0.18,
        name="carriage guide remains engaged on rail at rest",
    )
    ctx.expect_contact(
        carriage,
        base_frame,
        elem_a="upper_left_bridge",
        elem_b="rail_column",
        name="upper left carriage clamp touches rail",
    )
    ctx.expect_contact(
        carriage,
        base_frame,
        elem_a="upper_right_bridge",
        elem_b="rail_column",
        name="upper right carriage clamp touches rail",
    )

    rest_carriage_pos = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.42}):
        raised_carriage_pos = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base_frame,
            axes="z",
            elem_a="left_guide_pad",
            elem_b="rail_column",
            min_overlap=0.18,
            name="carriage guide remains engaged on rail when raised",
        )

    ctx.check(
        "carriage slides upward on rail",
        rest_carriage_pos is not None
        and raised_carriage_pos is not None
        and raised_carriage_pos[2] > rest_carriage_pos[2] + 0.35,
        details=f"rest={rest_carriage_pos}, raised={raised_carriage_pos}",
    )

    rest_board_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
    with ctx.pose({board_tilt: 0.72}):
        tilted_board_aabb = ctx.part_element_world_aabb(board, elem="board_panel")
        ctx.expect_gap(
            board,
            base_frame,
            axis="y",
            positive_elem="board_panel",
            negative_elem="rail_column",
            min_gap=0.01,
            name="tilted board still clears the rail",
        )

    board_tilts_back = False
    board_tilt_details = f"rest={rest_board_aabb}, tilted={tilted_board_aabb}"
    if rest_board_aabb is not None and tilted_board_aabb is not None:
        rest_min, rest_max = rest_board_aabb
        tilt_min, tilt_max = tilted_board_aabb
        board_tilts_back = tilt_min[1] < rest_min[1] - 0.25 and tilt_max[2] < rest_max[2] - 0.08
    ctx.check("board rotates backward at carriage head", board_tilts_back, details=board_tilt_details)

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_foot_aabb = ctx.part_element_world_aabb(foot_bar, elem="foot_plank")
    with ctx.pose({foot_joint: 0.35}):
        raised_foot_aabb = ctx.part_element_world_aabb(foot_bar, elem="foot_plank")

    rest_foot_center = _aabb_center(rest_foot_aabb)
    raised_foot_center = _aabb_center(raised_foot_aabb)
    foot_bar_moves = False
    if rest_foot_center is not None and raised_foot_center is not None:
        foot_bar_moves = raised_foot_center[2] > rest_foot_center[2] + 0.01
    ctx.check(
        "foot bar rotates upward around transverse pivot",
        foot_bar_moves,
        details=f"rest_center={rest_foot_center}, raised_center={raised_foot_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
