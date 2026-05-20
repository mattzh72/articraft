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
    sample_catmull_rom_spline_2d,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    cover_fabric = model.material("cover_fabric", rgba=(0.86, 0.87, 0.90, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.89, 0.90, 0.92, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    pad_gray = model.material("pad_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    board_top = model.part("board_top")
    board_profile = sample_catmull_rom_spline_2d(
        [
            (-0.62, -0.10),
            (-0.62, 0.10),
            (-0.47, 0.16),
            (-0.15, 0.20),
            (0.17, 0.19),
            (0.40, 0.15),
            (0.56, 0.08),
            (0.64, 0.00),
            (0.56, -0.08),
            (0.40, -0.15),
            (0.17, -0.19),
            (-0.15, -0.20),
            (-0.47, -0.16),
        ],
        samples_per_segment=10,
        closed=True,
    )
    board_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(board_profile, 0.028),
        "ironing_board_top",
    )
    board_top.visual(
        board_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.818)),
        material=cover_fabric,
        name="board_shell",
    )
    board_top.visual(
        Box((0.74, 0.18, 0.010)),
        origin=Origin(xyz=(-0.02, 0.0, 0.816)),
        material=pad_gray,
        name="underside_plate",
    )
    board_top.visual(
        Box((0.24, 0.14, 0.012)),
        origin=Origin(xyz=(-0.50, 0.0, 0.816)),
        material=pad_gray,
        name="tail_mount_pad",
    )
    board_top.inertial = Inertial.from_geometry(
        Box((1.28, 0.40, 0.04)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.0, 0.832)),
    )

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        Box((0.32, 0.24, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=dark_steel,
        name="mount_plate",
    )
    top_bracket.visual(
        Box((0.20, 0.10, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=dark_steel,
        name="hinge_block",
    )
    top_bracket.visual(
        Box((0.10, 0.22, 0.058)),
        origin=Origin(xyz=(0.088, 0.0, -0.075)),
        material=painted_steel,
        name="front_rib",
    )
    top_bracket.visual(
        Box((0.10, 0.22, 0.058)),
        origin=Origin(xyz=(-0.088, 0.0, -0.075)),
        material=painted_steel,
        name="rear_rib",
    )
    top_bracket.visual(
        Box((0.11, 0.16, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        material=dark_steel,
        name="axle_saddle",
    )
    top_bracket.inertial = Inertial.from_geometry(
        Box((0.32, 0.28, 0.18)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
    )

    main_leg_frame = model.part("main_leg_frame")
    leg_loop_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (0.10, -0.22, -0.79),
                (0.07, -0.21, -0.54),
                (0.03, -0.17, -0.22),
                (0.00, -0.126, -0.055),
                (0.00, 0.126, -0.055),
                (0.03, 0.17, -0.22),
                (0.07, 0.21, -0.54),
                (0.10, 0.22, -0.79),
            ],
            radius=0.019,
            radial_segments=18,
            corner_mode="fillet",
            corner_radius=0.085,
            cap_ends=True,
        ),
        "ironing_board_leg_loop",
    )
    main_leg_frame.visual(
        leg_loop_mesh,
        material=painted_steel,
        name="leg_loop",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.017, length=0.044),
        origin=Origin(xyz=(0.0, -0.104, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_pivot",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.017, length=0.044),
        origin=Origin(xyz=(0.0, 0.104, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_pivot",
    )
    main_leg_frame.visual(
        Box((0.052, 0.030, 0.054)),
        origin=Origin(xyz=(0.012, -0.104, -0.030)),
        material=dark_steel,
        name="left_hanger",
    )
    main_leg_frame.visual(
        Box((0.052, 0.030, 0.054)),
        origin=Origin(xyz=(0.012, 0.104, -0.030)),
        material=dark_steel,
        name="right_hanger",
    )
    main_leg_frame.visual(
        Box((0.14, 0.48, 0.040)),
        origin=Origin(xyz=(0.070, 0.0, -0.560)),
        material=dark_steel,
        name="lower_spreader",
    )
    main_leg_frame.visual(
        Box((0.060, 0.052, 0.070)),
        origin=Origin(xyz=(0.070, -0.205, -0.558)),
        material=dark_steel,
        name="left_spreader_clamp",
    )
    main_leg_frame.visual(
        Box((0.060, 0.052, 0.070)),
        origin=Origin(xyz=(0.070, 0.205, -0.558)),
        material=dark_steel,
        name="right_spreader_clamp",
    )
    main_leg_frame.visual(
        Box((0.060, 0.120, 0.520)),
        origin=Origin(xyz=(0.040, -0.155, -0.300)),
        material=dark_steel,
        name="left_side_brace",
    )
    main_leg_frame.visual(
        Box((0.060, 0.120, 0.520)),
        origin=Origin(xyz=(0.040, 0.155, -0.300)),
        material=dark_steel,
        name="right_side_brace",
    )
    main_leg_frame.visual(
        Box((0.20, 0.070, 0.046)),
        origin=Origin(xyz=(0.10, -0.22, -0.812)),
        material=foot_gray,
        name="left_foot",
    )
    main_leg_frame.visual(
        Box((0.20, 0.070, 0.046)),
        origin=Origin(xyz=(0.10, 0.22, -0.812)),
        material=foot_gray,
        name="right_foot",
    )
    main_leg_frame.inertial = Inertial.from_geometry(
        Box((0.28, 0.54, 0.86)),
        mass=3.5,
        origin=Origin(xyz=(0.05, 0.0, -0.43)),
    )

    tail_rest = model.part("tail_rest")
    tail_rest.visual(
        Box((0.12, 0.17, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_steel,
        name="rest_head",
    )
    tail_rest.visual(
        Box((0.028, 0.028, 0.76)),
        origin=Origin(xyz=(0.0, -0.072, -0.40)),
        material=painted_steel,
        name="left_stanchion",
    )
    tail_rest.visual(
        Box((0.028, 0.028, 0.76)),
        origin=Origin(xyz=(0.0, 0.072, -0.40)),
        material=painted_steel,
        name="right_stanchion",
    )
    tail_rest.visual(
        Box((0.10, 0.22, 0.048)),
        origin=Origin(xyz=(0.0, 0.0, -0.788)),
        material=foot_gray,
        name="rest_foot",
    )
    tail_rest.inertial = Inertial.from_geometry(
        Box((0.14, 0.24, 0.84)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, -0.42)),
    )

    model.articulation(
        "board_to_bracket",
        ArticulationType.FIXED,
        parent=board_top,
        child=top_bracket,
        origin=Origin(xyz=(-0.03, 0.0, 0.815)),
    )
    model.articulation(
        "board_to_tail_rest",
        ArticulationType.FIXED,
        parent=board_top,
        child=tail_rest,
        origin=Origin(xyz=(-0.50, 0.0, 0.810)),
    )
    model.articulation(
        "bracket_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=top_bracket,
        child=main_leg_frame,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=-1.00,
            upper=0.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board_top = object_model.get_part("board_top")
    main_leg_frame = object_model.get_part("main_leg_frame")
    tail_rest = object_model.get_part("tail_rest")
    fold_joint = object_model.get_articulation("bracket_to_leg_frame")

    ctx.expect_gap(
        board_top,
        main_leg_frame,
        axis="z",
        min_gap=0.035,
        max_gap=0.11,
        name="board stays above the open leg frame",
    )
    ctx.expect_gap(
        board_top,
        tail_rest,
        axis="z",
        max_gap=0.06,
        max_penetration=0.001,
        name="board seats onto the fixed tail rest",
    )
    ctx.expect_origin_gap(
        main_leg_frame,
        tail_rest,
        axis="x",
        min_gap=0.35,
        name="main trestle sits well ahead of the tail rest",
    )

    open_left_foot = ctx.part_element_world_aabb(main_leg_frame, elem="left_foot")
    with ctx.pose({fold_joint: -0.95}):
        ctx.expect_gap(
            board_top,
            main_leg_frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.24,
            name="folded frame remains tucked below the board",
        )
        ctx.expect_overlap(
            main_leg_frame,
            board_top,
            axes="x",
            min_overlap=0.40,
            name="folded frame still lies under the board lengthwise",
        )
        folded_left_foot = ctx.part_element_world_aabb(main_leg_frame, elem="left_foot")

    open_left_center_x = None
    folded_left_center_x = None
    if open_left_foot is not None:
        open_left_center_x = 0.5 * (open_left_foot[0][0] + open_left_foot[1][0])
    if folded_left_foot is not None:
        folded_left_center_x = 0.5 * (folded_left_foot[0][0] + folded_left_foot[1][0])
    ctx.check(
        "leg frame folds back toward the tail",
        open_left_center_x is not None
        and folded_left_center_x is not None
        and folded_left_center_x < open_left_center_x - 0.45,
        details=f"open_x={open_left_center_x}, folded_x={folded_left_center_x}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
