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
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    enamel_white = model.material("enamel_white", rgba=(0.95, 0.95, 0.96, 1.0))
    cover_blue = model.material("cover_blue", rgba=(0.42, 0.56, 0.78, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.66, 0.69, 0.72, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    board_outline = sample_catmull_rom_spline_2d(
        [
            (-0.68, 0.0),
            (-0.62, 0.06),
            (-0.50, 0.11),
            (-0.28, 0.16),
            (0.05, 0.19),
            (0.44, 0.19),
            (0.68, 0.17),
            (0.72, 0.11),
            (0.72, -0.11),
            (0.68, -0.17),
            (0.44, -0.19),
            (0.05, -0.19),
            (-0.28, -0.16),
            (-0.50, -0.11),
            (-0.62, -0.06),
        ],
        samples_per_segment=10,
        closed=True,
    )
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(board_outline, 0.008),
        "ironing_board_deck",
    )
    pad_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(board_outline, 0.014),
        "ironing_board_pad",
    )

    board_base = model.part("board_base")
    board_base.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.852)),
        material=enamel_white,
        name="deck_panel",
    )
    board_base.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material=cover_blue,
        name="pad_surface",
    )
    board_base.visual(
        Box((0.30, 0.10, 0.024)),
        origin=Origin(xyz=(0.025, 0.0, 0.840)),
        material=dark_gray,
        name="base_bracket",
    )
    board_base.visual(
        Box((0.18, 0.09, 0.032)),
        origin=Origin(xyz=(0.210, 0.0, 0.836)),
        material=steel_gray,
        name="rear_stiffener",
    )
    board_base.visual(
        Box((0.028, 0.022, 0.062)),
        origin=Origin(xyz=(0.0, -0.055, 0.821)),
        material=dark_gray,
        name="hinge_cheek_left",
    )
    board_base.visual(
        Box((0.028, 0.022, 0.062)),
        origin=Origin(xyz=(0.0, 0.055, 0.821)),
        material=dark_gray,
        name="hinge_cheek_right",
    )
    board_base.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, -0.055, 0.790), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="hinge_sleeve_left",
    )
    board_base.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.0, 0.055, 0.790), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="hinge_sleeve_right",
    )
    fixed_rest = tube_from_spline_points(
        [
            (-0.500, -0.110, -0.046),
            (-0.478, -0.106, 0.220),
            (-0.444, -0.096, 0.520),
            (-0.408, -0.090, 0.822),
            (-0.394, 0.000, 0.822),
            (-0.408, 0.090, 0.822),
            (-0.444, 0.096, 0.520),
            (-0.478, 0.106, 0.220),
            (-0.500, 0.110, -0.046),
        ],
        radius=0.010,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    board_base.visual(
        mesh_from_geometry(fixed_rest, "ironing_board_fixed_end_rest"),
        material=steel_gray,
        name="fixed_end_rest",
    )
    board_base.visual(
        Box((0.116, 0.184, 0.020)),
        origin=Origin(xyz=(-0.398, 0.0, 0.842)),
        material=dark_gray,
        name="fixed_rest_mount",
    )
    board_base.visual(
        Box((0.044, 0.034, 0.018)),
        origin=Origin(xyz=(-0.500, -0.110, -0.055)),
        material=rubber_dark,
        name="fixed_rest_foot_left",
    )
    board_base.visual(
        Box((0.044, 0.034, 0.018)),
        origin=Origin(xyz=(-0.500, 0.110, -0.055)),
        material=rubber_dark,
        name="fixed_rest_foot_right",
    )
    board_base.inertial = Inertial.from_geometry(
        Box((1.44, 0.40, 0.12)),
        mass=6.5,
        origin=Origin(xyz=(0.02, 0.0, 0.822)),
    )

    leg_frame = model.part("leg_frame")
    leg_frame.visual(
        Cylinder(radius=0.014, length=0.070),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="pivot_sleeve",
    )
    leg_frame.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(
            xyz=(0.026, -0.030, -0.015),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_gray,
        name="pivot_link_left",
    )
    leg_frame.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(
            xyz=(0.026, 0.030, -0.015),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel_gray,
        name="pivot_link_right",
    )

    left_hanger = tube_from_spline_points(
        [
            (0.050, -0.032, -0.018),
            (0.056, -0.108, -0.052),
            (0.070, -0.195, -0.110),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    right_hanger = tube_from_spline_points(
        [
            (0.050, 0.032, -0.018),
            (0.056, 0.108, -0.052),
            (0.070, 0.195, -0.110),
        ],
        radius=0.010,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    u_frame = tube_from_spline_points(
        [
            (0.250, -0.280, -0.842),
            (0.176, -0.278, -0.820),
            (0.126, -0.238, -0.440),
            (0.070, -0.195, -0.110),
            (0.070, 0.195, -0.110),
            (0.126, 0.238, -0.440),
            (0.176, 0.278, -0.820),
            (0.250, 0.280, -0.842),
        ],
        radius=0.012,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    leg_frame.visual(
        mesh_from_geometry(left_hanger, "ironing_board_leg_hanger_left"),
        material=steel_gray,
        name="hanger_left",
    )
    leg_frame.visual(
        mesh_from_geometry(right_hanger, "ironing_board_leg_hanger_right"),
        material=steel_gray,
        name="hanger_right",
    )
    leg_frame.visual(
        mesh_from_geometry(u_frame, "ironing_board_u_frame"),
        material=steel_gray,
        name="u_frame",
    )
    leg_frame.visual(
        Cylinder(radius=0.010, length=0.480),
        origin=Origin(xyz=(0.120, 0.0, -0.450), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="lower_brace",
    )
    leg_frame.visual(
        Box((0.050, 0.034, 0.012)),
        origin=Origin(xyz=(0.262, -0.280, -0.848)),
        material=rubber_dark,
        name="foot_pad_left",
    )
    leg_frame.visual(
        Box((0.050, 0.034, 0.012)),
        origin=Origin(xyz=(0.262, 0.280, -0.848)),
        material=rubber_dark,
        name="foot_pad_right",
    )
    leg_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.62, 0.88)),
        mass=2.9,
        origin=Origin(xyz=(0.15, 0.0, -0.43)),
    )

    model.articulation(
        "board_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=board_base,
        child=leg_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=1.20,
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
    board_base = object_model.get_part("board_base")
    leg_frame = object_model.get_part("leg_frame")
    leg_joint = object_model.get_articulation("board_to_leg_frame")

    ctx.expect_gap(
        board_base,
        leg_frame,
        axis="z",
        positive_elem="deck_panel",
        negative_elem="foot_pad_left",
        min_gap=0.82,
        max_gap=0.94,
        name="left foot sits far below the ironing surface",
    )
    ctx.expect_overlap(
        board_base,
        leg_frame,
        axes="xy",
        elem_a="base_bracket",
        elem_b="u_frame",
        min_overlap=0.10,
        name="leg frame stays centered under the board bracket",
    )

    rest_left = ctx.part_element_world_aabb(leg_frame, elem="foot_pad_left")
    fixed_rest_left = ctx.part_element_world_aabb(board_base, elem="fixed_rest_foot_left")
    fixed_rest_right = ctx.part_element_world_aabb(board_base, elem="fixed_rest_foot_right")
    open_leg = ctx.part_world_aabb(leg_frame)
    ctx.check(
        "fixed end rest sits beyond the main trestle",
        fixed_rest_left is not None
        and fixed_rest_right is not None
        and open_leg is not None
        and fixed_rest_left[1][0] < open_leg[0][0] - 0.35
        and fixed_rest_right[1][0] < open_leg[0][0] - 0.35,
        details=f"fixed_left={fixed_rest_left}, fixed_right={fixed_rest_right}, leg={open_leg}",
    )
    ctx.check(
        "fixed rest and folding feet share the floor height",
        rest_left is not None
        and fixed_rest_left is not None
        and abs(rest_left[0][2] - fixed_rest_left[0][2]) <= 0.015,
        details=f"leg_foot={rest_left}, fixed_rest={fixed_rest_left}",
    )

    with ctx.pose({leg_joint: 1.20}):
        folded_left = ctx.part_element_world_aabb(leg_frame, elem="foot_pad_left")
    ctx.check(
        "leg frame folds upward under the board",
        rest_left is not None
        and folded_left is not None
        and folded_left[1][2] > rest_left[1][2] + 0.55,
        details=f"rest={rest_left}, folded={folded_left}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
