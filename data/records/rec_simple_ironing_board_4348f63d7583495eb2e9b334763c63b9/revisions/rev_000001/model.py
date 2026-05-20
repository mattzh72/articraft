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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _board_outline() -> list[tuple[float, float]]:
    control_points = [
        (-0.67, 0.00),
        (-0.64, 0.12),
        (-0.56, 0.18),
        (-0.38, 0.20),
        (-0.02, 0.20),
        (0.30, 0.17),
        (0.52, 0.11),
        (0.64, 0.05),
        (0.69, 0.00),
        (0.64, -0.05),
        (0.52, -0.11),
        (0.30, -0.17),
        (-0.02, -0.20),
        (-0.38, -0.20),
        (-0.56, -0.18),
        (-0.64, -0.12),
    ]
    return sample_catmull_rom_spline_2d(control_points, samples_per_segment=10, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.39, 0.41, 0.44, 1.0))
    board_shell = model.material("board_shell", rgba=(0.88, 0.87, 0.82, 1.0))
    cover = model.material("cover", rgba=(0.53, 0.71, 0.88, 1.0))
    plastic = model.material("plastic", rgba=(0.16, 0.17, 0.18, 1.0))

    board_assembly = model.part("board_assembly")
    board_assembly.inertial = Inertial.from_geometry(
        Box((1.38, 0.42, 0.89)),
        mass=7.8,
        origin=Origin(xyz=(0.0, 0.0, 0.445)),
    )

    outline = _board_outline()
    deck_mesh = _save_mesh("ironing_board_deck", ExtrudeGeometry.from_z0(outline, 0.018))
    cover_mesh = _save_mesh("ironing_board_cover", ExtrudeGeometry.from_z0(outline, 0.010))

    board_assembly.visual(
        deck_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material=board_shell,
        name="board_panel",
    )
    board_assembly.visual(
        cover_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.878)),
        material=cover,
        name="board_cover",
    )

    board_assembly.visual(
        Box((0.30, 0.30, 0.020)),
        origin=Origin(xyz=(0.16, 0.0, 0.850)),
        material=dark_steel,
        name="base_plate",
    )
    board_assembly.visual(
        Box((0.034, 0.030, 0.084)),
        origin=Origin(xyz=(0.206, -0.135, 0.800)),
        material=steel,
        name="left_hinge_cheek",
    )
    board_assembly.visual(
        Box((0.034, 0.030, 0.084)),
        origin=Origin(xyz=(0.206, 0.135, 0.800)),
        material=steel,
        name="right_hinge_cheek",
    )
    board_assembly.visual(
        Cylinder(radius=0.0075, length=0.042),
        origin=Origin(xyz=(0.1862, -0.135, 0.803), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_hinge_pin",
    )
    board_assembly.visual(
        Cylinder(radius=0.0075, length=0.042),
        origin=Origin(xyz=(0.1862, 0.135, 0.803), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_hinge_pin",
    )
    rear_rest_geom = tube_from_spline_points(
        [
            (-0.56, -0.105, 0.820),
            (-0.55, -0.105, 0.700),
            (-0.53, -0.115, 0.470),
            (-0.50, -0.125, 0.120),
            (-0.50, -0.125, 0.018),
            (-0.50, 0.125, 0.018),
            (-0.50, 0.125, 0.120),
            (-0.53, 0.115, 0.470),
            (-0.55, 0.105, 0.700),
            (-0.56, 0.105, 0.820),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=16,
        cap_ends=True,
    )
    board_assembly.visual(
        _save_mesh("rear_rest_frame", rear_rest_geom),
        material=steel,
        name="rear_rest_frame",
    )
    board_assembly.visual(
        Box((0.042, 0.030, 0.042)),
        origin=Origin(xyz=(-0.56, -0.105, 0.839)),
        material=dark_steel,
        name="rear_rest_left_mount",
    )
    board_assembly.visual(
        Box((0.042, 0.030, 0.042)),
        origin=Origin(xyz=(-0.56, 0.105, 0.839)),
        material=dark_steel,
        name="rear_rest_right_mount",
    )
    board_assembly.visual(
        Cylinder(radius=0.014, length=0.280),
        origin=Origin(xyz=(-0.50, 0.0, 0.018), rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="rear_rest_foot_bar",
    )

    main_leg_frame = model.part("main_leg_frame")
    main_leg_frame.inertial = Inertial.from_geometry(
        Box((0.32, 0.48, 0.84)),
        mass=2.4,
        origin=Origin(xyz=(-0.06, 0.0, -0.40)),
    )

    main_leg_geom = tube_from_spline_points(
        [
            (-0.012, -0.160, 0.000),
            (-0.052, -0.170, -0.180),
            (-0.112, -0.200, -0.520),
            (-0.136, -0.220, -0.800),
            (-0.136, 0.220, -0.800),
            (-0.112, 0.200, -0.520),
            (-0.052, 0.170, -0.180),
            (-0.012, 0.160, 0.000),
        ],
        radius=0.011,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    main_leg_frame.visual(
        _save_mesh("main_leg_u_frame", main_leg_geom),
        material=steel,
        name="u_frame_tube",
    )
    main_leg_frame.visual(
        Box((0.018, 0.030, 0.034)),
        origin=Origin(xyz=(-0.0103, -0.145, 0.0)),
        material=dark_steel,
        name="left_hinge_lug",
    )
    main_leg_frame.visual(
        Box((0.018, 0.030, 0.034)),
        origin=Origin(xyz=(-0.0103, 0.145, 0.0)),
        material=dark_steel,
        name="right_hinge_lug",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.014, length=0.460),
        origin=Origin(xyz=(-0.136, 0.0, -0.802), rpy=(pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="foot_bar",
    )

    model.articulation(
        "leg_fold",
        ArticulationType.REVOLUTE,
        parent=board_assembly,
        child=main_leg_frame,
        origin=Origin(xyz=(0.18, 0.0, 0.803)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.4,
            lower=0.0,
            upper=1.42,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board_assembly = object_model.get_part("board_assembly")
    main_leg_frame = object_model.get_part("main_leg_frame")
    leg_fold = object_model.get_articulation("leg_fold")

    ctx.expect_origin_distance(
        board_assembly,
        main_leg_frame,
        axes="y",
        max_dist=0.001,
        name="main leg stays centered on the board center plane",
    )
    ctx.expect_gap(
        board_assembly,
        main_leg_frame,
        axis="z",
        positive_elem="board_panel",
        negative_elem="foot_bar",
        min_gap=0.83,
        max_gap=0.90,
        name="open main foot bar holds the board near standard ironing height",
    )
    ctx.expect_overlap(
        board_assembly,
        main_leg_frame,
        axes="y",
        min_overlap=0.30,
        elem_a="base_plate",
        elem_b="u_frame_tube",
        name="open leg frame spans broadly under the central bracket",
    )

    open_aabb = ctx.part_element_world_aabb(main_leg_frame, elem="foot_bar")
    with ctx.pose({leg_fold: 1.35}):
        ctx.expect_gap(
            board_assembly,
            main_leg_frame,
            axis="z",
            positive_elem="board_panel",
            negative_elem="foot_bar",
            min_gap=0.015,
            max_gap=0.10,
            name="folded foot bar tucks close under the board",
        )
        folded_aabb = ctx.part_element_world_aabb(main_leg_frame, elem="foot_bar")

    open_top_z = open_aabb[1][2] if open_aabb is not None else None
    folded_top_z = folded_aabb[1][2] if folded_aabb is not None else None
    ctx.check(
        "leg frame rotates upward when folding",
        open_top_z is not None and folded_top_z is not None and folded_top_z > open_top_z + 0.70,
        details=f"open_top_z={open_top_z}, folded_top_z={folded_top_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
