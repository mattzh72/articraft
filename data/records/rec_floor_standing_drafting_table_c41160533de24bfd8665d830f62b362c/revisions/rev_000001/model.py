from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="freestanding_drafting_station")

    powder_coat = model.material("powder_coat", rgba=(0.19, 0.20, 0.22, 1.0))
    graphite = model.material("graphite", rgba=(0.29, 0.30, 0.32, 1.0))
    laminate = model.material("laminate", rgba=(0.83, 0.85, 0.82, 1.0))
    drawer_gray = model.material("drawer_gray", rgba=(0.69, 0.71, 0.74, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.72, 0.18, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=powder_coat,
        name="pedestal_foot_beam",
    )
    base.visual(
        Box((0.46, 0.34, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_coat,
        name="pedestal_cross_beam",
    )
    base.visual(
        Box((0.22, 0.26, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=graphite,
        name="pedestal_plinth",
    )

    for index, (x, y) in enumerate(
        (
            (-0.31, 0.0),
            (0.31, 0.0),
            (0.0, -0.13),
            (0.0, 0.13),
        )
    ):
        base.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(x, y, 0.006)),
            material=rubber,
            name=f"leveler_{index}",
        )

    sleeve_bottom = 0.10
    sleeve_height = 0.48
    sleeve_top = sleeve_bottom + sleeve_height
    sleeve_outer_x = 0.13
    sleeve_outer_y = 0.11
    sleeve_wall = 0.018
    sleeve_inner_x = sleeve_outer_x - 2.0 * sleeve_wall
    sleeve_inner_y = sleeve_outer_y - 2.0 * sleeve_wall
    sleeve_center_z = sleeve_bottom + sleeve_height / 2.0

    base.visual(
        Box((sleeve_wall, sleeve_outer_y, sleeve_height)),
        origin=Origin(xyz=(-(sleeve_outer_x - sleeve_wall) / 2.0, 0.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_left_wall",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer_y, sleeve_height)),
        origin=Origin(xyz=((sleeve_outer_x - sleeve_wall) / 2.0, 0.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_right_wall",
    )
    base.visual(
        Box((sleeve_inner_x, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, -(sleeve_outer_y - sleeve_wall) / 2.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_back_wall",
    )
    base.visual(
        Box((sleeve_inner_x, sleeve_wall, sleeve_height)),
        origin=Origin(xyz=(0.0, (sleeve_outer_y - sleeve_wall) / 2.0, sleeve_center_z)),
        material=graphite,
        name="sleeve_front_wall",
    )
    base.visual(
        Box((0.018, 0.14, 0.030)),
        origin=Origin(xyz=(-0.071, 0.0, sleeve_top + 0.015)),
        material=powder_coat,
        name="sleeve_cap_left",
    )
    base.visual(
        Box((0.018, 0.14, 0.030)),
        origin=Origin(xyz=(0.071, 0.0, sleeve_top + 0.015)),
        material=powder_coat,
        name="sleeve_cap_right",
    )
    base.visual(
        Box((0.094, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, -0.061, sleeve_top + 0.015)),
        material=powder_coat,
        name="sleeve_cap_back",
    )
    base.visual(
        Box((0.094, 0.018, 0.030)),
        origin=Origin(xyz=(0.0, 0.061, sleeve_top + 0.015)),
        material=powder_coat,
        name="sleeve_cap_front",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.34, 0.61)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
    )

    upright = model.part("upright")
    upright.visual(
        Box((0.078, 0.060, 0.700)),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material=powder_coat,
        name="inner_post",
    )
    upright.visual(
        Box((0.120, 0.096, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=graphite,
        name="head_block",
    )
    upright.visual(
        Box((0.160, 0.020, 0.120)),
        origin=Origin(xyz=(0.0, -0.036, 0.330)),
        material=graphite,
        name="cradle_bridge",
    )
    upright.visual(
        Box((0.018, 0.090, 0.150)),
        origin=Origin(xyz=(-0.070, 0.0, 0.345)),
        material=graphite,
        name="cradle_left_cheek",
    )
    upright.visual(
        Box((0.018, 0.090, 0.150)),
        origin=Origin(xyz=(0.070, 0.0, 0.345)),
        material=graphite,
        name="cradle_right_cheek",
    )
    upright.visual(
        Cylinder(radius=0.017, length=0.034),
        origin=Origin(
            xyz=(-0.092, 0.0, 0.345),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_coat,
        name="left_tension_knob",
    )
    upright.visual(
        Cylinder(radius=0.017, length=0.034),
        origin=Origin(
            xyz=(0.092, 0.0, 0.345),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_coat,
        name="right_tension_knob",
    )
    upright.inertial = Inertial.from_geometry(
        Box((0.20, 0.12, 0.92)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    model.articulation(
        "base_to_upright",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upright,
        origin=Origin(xyz=(0.0, 0.0, sleeve_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.12,
            lower=0.0,
            upper=0.28,
        ),
    )

    board = model.part("board")
    board_shell = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.88, 0.62, 0.022, corner_segments=10),
            0.028,
            cap=True,
            center=True,
        ),
        "drafting_board_shell",
    )
    board.visual(
        board_shell,
        origin=Origin(xyz=(0.0, 0.365, 0.046)),
        material=laminate,
        name="board_panel",
    )
    board.visual(
        Box((0.84, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, 0.657, 0.007)),
        material=graphite,
        name="front_apron",
    )
    board.visual(
        Box((0.026, 0.588, 0.050)),
        origin=Origin(xyz=(-0.427, 0.365, 0.007)),
        material=graphite,
        name="left_apron",
    )
    board.visual(
        Box((0.026, 0.588, 0.050)),
        origin=Origin(xyz=(0.427, 0.365, 0.007)),
        material=graphite,
        name="right_apron",
    )
    board.visual(
        Box((0.560, 0.040, 0.074)),
        origin=Origin(xyz=(0.0, 0.080, -0.002)),
        material=graphite,
        name="back_spine",
    )
    board.visual(
        Box((0.118, 0.060, 0.100)),
        origin=Origin(xyz=(0.0, 0.055, -0.004)),
        material=graphite,
        name="trunnion_bracket",
    )
    board.visual(
        Cylinder(radius=0.026, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="trunnion_barrel",
    )
    board.visual(
        Box((0.200, 0.022, 0.022)),
        origin=Origin(xyz=(0.305, 0.273, 0.021)),
        material=powder_coat,
        name="runner_front",
    )
    board.visual(
        Box((0.200, 0.022, 0.022)),
        origin=Origin(xyz=(0.305, 0.407, 0.021)),
        material=powder_coat,
        name="runner_rear",
    )
    board.inertial = Inertial.from_geometry(
        Box((0.88, 0.62, 0.17)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.365, 0.030)),
    )

    model.articulation(
        "upright_to_board",
        ArticulationType.REVOLUTE,
        parent=upright,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.8,
            lower=0.0,
            upper=1.18,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.260, 0.180, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=drawer_gray,
        name="tray_bottom",
    )
    tray.visual(
        Box((0.260, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.087, -0.001)),
        material=drawer_gray,
        name="tray_inner_left_wall",
    )
    tray.visual(
        Box((0.260, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, 0.087, -0.001)),
        material=drawer_gray,
        name="tray_inner_right_wall",
    )
    tray.visual(
        Box((0.006, 0.180, 0.018)),
        origin=Origin(xyz=(-0.127, 0.0, -0.001)),
        material=drawer_gray,
        name="tray_back_wall",
    )
    tray.visual(
        Box((0.006, 0.180, 0.026)),
        origin=Origin(xyz=(0.127, 0.0, -0.005)),
        material=drawer_gray,
        name="tray_front_pull",
    )
    tray.visual(
        Box((0.170, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.082, 0.006)),
        material=powder_coat,
        name="tray_runner_front",
    )
    tray.visual(
        Box((0.170, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.082, 0.006)),
        material=powder_coat,
        name="tray_runner_rear",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.27, 0.19, 0.04)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    model.articulation(
        "board_to_tray",
        ArticulationType.PRISMATIC,
        parent=board,
        child=tray,
        origin=Origin(xyz=(0.280, 0.340, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.15,
            lower=0.0,
            upper=0.12,
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

    base = object_model.get_part("base")
    upright = object_model.get_part("upright")
    board = object_model.get_part("board")
    tray = object_model.get_part("tray")

    lift = object_model.get_articulation("base_to_upright")
    tilt = object_model.get_articulation("upright_to_board")
    slide = object_model.get_articulation("board_to_tray")

    ctx.check(
        "all drafting station parts authored",
        all(part is not None for part in (base, upright, board, tray)),
        details="Expected base, upright, board, and tray parts to exist.",
    )
    ctx.check(
        "all drafting station articulations authored",
        all(joint is not None for joint in (lift, tilt, slide)),
        details="Expected lift, tilt, and tray slide articulations to exist.",
    )

    board_rest = ctx.part_world_position(board)
    tray_rest = ctx.part_world_position(tray)
    front_rest = ctx.part_element_world_aabb(board, elem="front_apron")
    runner_front_rest = ctx.part_element_world_aabb(board, elem="runner_front")

    with ctx.pose({lift: lift.motion_limits.upper}):
        board_raised = ctx.part_world_position(board)
    ctx.check(
        "upright telescopes upward",
        board_rest is not None
        and board_raised is not None
        and board_raised[2] > board_rest[2] + 0.22,
        details=f"rest={board_rest}, raised={board_raised}",
    )

    with ctx.pose({tilt: tilt.motion_limits.upper}):
        front_tilted = ctx.part_element_world_aabb(board, elem="front_apron")
        barrel_tilted = ctx.part_element_world_aabb(board, elem="trunnion_barrel")
    ctx.check(
        "board front edge rises around the trunnion",
        front_rest is not None
        and front_tilted is not None
        and barrel_tilted is not None
        and ((front_tilted[0][2] + front_tilted[1][2]) / 2.0)
        > ((front_rest[0][2] + front_rest[1][2]) / 2.0) + 0.30
        and front_tilted[0][2] > barrel_tilted[1][2] + 0.18,
        details=(
            f"front_rest={front_rest}, "
            f"front_tilted={front_tilted}, barrel_tilted={barrel_tilted}"
        ),
    )

    with ctx.pose({slide: slide.motion_limits.upper}):
        tray_extended = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            board,
            axes="x",
            elem_a="tray_runner_front",
            elem_b="runner_front",
            min_overlap=0.070,
            name="front tray runner retains insertion at full extension",
        )
        ctx.expect_overlap(
            tray,
            board,
            axes="x",
            elem_a="tray_runner_rear",
            elem_b="runner_rear",
            min_overlap=0.070,
            name="rear tray runner retains insertion at full extension",
        )
    ctx.check(
        "side tray slides outward from the board edge",
        tray_rest is not None
        and tray_extended is not None
        and tray_extended[0] > tray_rest[0] + 0.08,
        details=f"rest={tray_rest}, extended={tray_extended}",
    )

    ctx.expect_gap(
        board,
        tray,
        axis="z",
        positive_elem="runner_front",
        negative_elem="tray_front_pull",
        min_gap=0.002,
        max_gap=0.050,
        name="tray stays tucked beneath the board runners",
    )

    ctx.expect_overlap(
        tray,
        board,
        axes="x",
        elem_a="tray_runner_front",
        elem_b="runner_front",
        min_overlap=0.150,
        name="collapsed tray sits on its short front runner",
    )
    ctx.expect_overlap(
        tray,
        board,
        axes="x",
        elem_a="tray_runner_rear",
        elem_b="runner_rear",
        min_overlap=0.150,
        name="collapsed tray sits on its short rear runner",
    )

    ctx.check(
        "board runners are positioned above the tray",
        runner_front_rest is not None
        and runner_front_rest[0][2] > -0.001,
        details=f"runner_front_rest={runner_front_rest}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
