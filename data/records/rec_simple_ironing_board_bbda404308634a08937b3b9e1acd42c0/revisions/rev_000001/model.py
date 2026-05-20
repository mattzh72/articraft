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
    model = ArticulatedObject(name="ironing_board")

    cover_fabric = model.material("cover_fabric", rgba=(0.62, 0.72, 0.82, 1.0))
    steel = model.material("steel", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.33, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    board_body = model.part("board_body")

    board_outline = sample_catmull_rom_spline_2d(
        [
            (0.62, 0.0),
            (0.52, 0.105),
            (0.20, 0.168),
            (-0.50, 0.176),
            (-0.66, 0.145),
            (-0.69, 0.0),
            (-0.66, -0.145),
            (-0.50, -0.176),
            (0.20, -0.168),
            (0.52, -0.105),
        ],
        samples_per_segment=10,
        closed=True,
    )
    board_mesh = mesh_from_geometry(
        ExtrudeGeometry(board_outline, 0.022, center=True),
        "ironing_board_top",
    )
    board_body.visual(
        board_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.731)),
        material=cover_fabric,
        name="board_top",
    )
    board_body.visual(
        Box((1.04, 0.24, 0.010)),
        origin=Origin(xyz=(-0.02, 0.0, 0.715)),
        material=steel,
        name="underside_pan",
    )
    board_body.visual(
        Box((0.34, 0.12, 0.030)),
        origin=Origin(xyz=(0.02, 0.0, 0.700)),
        material=dark_steel,
        name="base_bracket",
    )
    board_body.visual(
        Box((0.18, 0.030, 0.050)),
        origin=Origin(xyz=(-0.06, 0.0, 0.688)),
        material=dark_steel,
        name="bracket_spine",
    )
    board_body.visual(
        Box((0.18, 0.122, 0.028)),
        origin=Origin(xyz=(-0.005, 0.121, 0.686)),
        material=dark_steel,
        name="left_brace_arm",
    )
    board_body.visual(
        Box((0.18, 0.122, 0.028)),
        origin=Origin(xyz=(-0.005, -0.121, 0.686)),
        material=dark_steel,
        name="right_brace_arm",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        board_body.visual(
            Box((0.050, 0.022, 0.082)),
            origin=Origin(xyz=(0.060, sign * 0.185, 0.676)),
            material=dark_steel,
            name=f"{side}_hinge_cheek",
        )
    board_body.inertial = Inertial.from_geometry(
        Box((1.38, 0.38, 0.10)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.725)),
    )

    tail_rest = model.part("tail_rest")
    tail_rest.visual(
        Box((0.18, 0.190, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.021)),
        material=dark_steel,
        name="tail_mount",
    )
    tail_rest.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.082, -0.020),
                    (-0.025, 0.098, -0.240),
                    (-0.070, 0.110, -0.715),
                    (-0.070, -0.110, -0.715),
                    (-0.025, -0.098, -0.240),
                    (0.0, -0.082, -0.020),
                ],
                radius=0.012,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
            "ironing_board_tail_rest",
        ),
        material=steel,
        name="tail_frame",
    )
    tail_rest.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(
            xyz=(-0.070, 0.110, -0.715),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="tail_left_foot",
    )
    tail_rest.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(
            xyz=(-0.070, -0.110, -0.715),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=rubber,
        name="tail_right_foot",
    )
    tail_rest.visual(
        Box((0.040, 0.240, 0.022)),
        origin=Origin(xyz=(-0.070, 0.0, -0.715)),
        material=rubber,
        name="tail_foot_bar",
    )
    tail_rest.visual(
        Box((0.034, 0.032, 0.060)),
        origin=Origin(xyz=(-0.010, 0.084, -0.050)),
        material=dark_steel,
        name="tail_left_gusset",
    )
    tail_rest.visual(
        Box((0.034, 0.032, 0.060)),
        origin=Origin(xyz=(-0.010, -0.084, -0.050)),
        material=dark_steel,
        name="tail_right_gusset",
    )
    tail_rest.inertial = Inertial.from_geometry(
        Box((0.22, 0.25, 0.74)),
        mass=1.2,
        origin=Origin(xyz=(-0.035, 0.0, -0.360)),
    )

    model.articulation(
        "board_to_tail_rest",
        ArticulationType.FIXED,
        parent=board_body,
        child=tail_rest,
        origin=Origin(xyz=(-0.46, 0.0, 0.720)),
    )

    main_leg = model.part("main_leg")
    main_leg.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.205, -0.010),
                    (0.055, 0.230, -0.185),
                    (0.150, 0.255, -0.455),
                    (0.235, 0.285, -0.700),
                    (0.235, -0.285, -0.700),
                    (0.150, -0.255, -0.455),
                    (0.055, -0.230, -0.185),
                    (0.0, -0.205, -0.010),
                ],
                radius=0.013,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "ironing_board_main_leg",
        ),
        material=steel,
        name="leg_frame",
    )
    main_leg.visual(
        Cylinder(radius=0.015, length=0.070),
        origin=Origin(
            xyz=(0.0, 0.205, -0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    main_leg.visual(
        Cylinder(radius=0.015, length=0.070),
        origin=Origin(
            xyz=(0.0, -0.205, -0.010),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="right_hinge_barrel",
    )
    main_leg.visual(
        Box((0.055, 0.575, 0.024)),
        origin=Origin(xyz=(0.235, 0.0, -0.700)),
        material=rubber,
        name="main_foot_bar",
    )
    main_leg.inertial = Inertial.from_geometry(
        Box((0.30, 0.60, 0.74)),
        mass=1.8,
        origin=Origin(xyz=(0.145, 0.0, -0.360)),
    )

    model.articulation(
        "board_to_main_leg",
        ArticulationType.REVOLUTE,
        parent=board_body,
        child=main_leg,
        origin=Origin(xyz=(0.10, 0.0, 0.700)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board_body = object_model.get_part("board_body")
    tail_rest = object_model.get_part("tail_rest")
    main_leg = object_model.get_part("main_leg")
    leg_hinge = object_model.get_articulation("board_to_main_leg")

    def _elem_aabb(part, elem: str):
        return ctx.part_element_world_aabb(part, elem=elem)

    def _center_x(aabb) -> float:
        return 0.5 * (aabb[0][0] + aabb[1][0])

    def _min_z(aabb) -> float:
        return aabb[0][2]

    def _max_z(aabb) -> float:
        return aabb[1][2]

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_contact(
        tail_rest,
        board_body,
        elem_a="tail_mount",
        elem_b="underside_pan",
        name="tail rest mount seats against the board underside",
    )

    with ctx.pose({leg_hinge: 0.0}):
        board_top_aabb = _elem_aabb(board_body, "board_top")
        main_foot_aabb = _elem_aabb(main_leg, "main_foot_bar")
        tail_foot_aabb = _elem_aabb(tail_rest, "tail_foot_bar")

        support_spread = abs(_center_x(main_foot_aabb) - _center_x(tail_foot_aabb))
        foot_level_delta = abs(_min_z(main_foot_aabb) - _min_z(tail_foot_aabb))
        working_height = _max_z(board_top_aabb) - min(
            _min_z(main_foot_aabb), _min_z(tail_foot_aabb)
        )

        ctx.check(
            "support footprint is stretched along the board length",
            support_spread >= 0.80,
            details=f"support_spread={support_spread:.4f} m",
        )
        ctx.check(
            "main and tail supports land at nearly the same floor height",
            foot_level_delta <= 0.02,
            details=f"foot_level_delta={foot_level_delta:.4f} m",
        )
        ctx.check(
            "board stands in a low working-height range",
            0.70 <= working_height <= 0.78,
            details=f"working_height={working_height:.4f} m",
        )

    folded_upper = leg_hinge.motion_limits.upper if leg_hinge.motion_limits else 0.0
    with ctx.pose({leg_hinge: folded_upper}):
        folded_main_foot_aabb = _elem_aabb(main_leg, "main_foot_bar")
        ctx.check(
            "main trestle folds upward under the board",
            _min_z(folded_main_foot_aabb) >= 0.60,
            details=f"folded_main_foot_min_z={_min_z(folded_main_foot_aabb):.4f} m",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
