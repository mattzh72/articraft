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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _board_outline() -> list[tuple[float, float]]:
    control_points = [
        (-0.74, 0.0),
        (-0.70, 0.17),
        (-0.18, 0.19),
        (0.36, 0.11),
        (0.64, 0.05),
        (0.74, 0.0),
        (0.64, -0.05),
        (0.36, -0.11),
        (-0.18, -0.19),
        (-0.70, -0.17),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=10,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    cover = model.material("cover", rgba=(0.67, 0.76, 0.88, 1.0))
    frame_paint = model.material("frame_paint", rgba=(0.82, 0.83, 0.84, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.16, 0.16, 0.17, 1.0))

    board_assembly = model.part("board_assembly")
    board_assembly.inertial = Inertial.from_geometry(
        Box((1.50, 0.40, 0.86)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, -0.42)),
    )

    board_shell = _mesh(
        "board_shell",
        ExtrudeGeometry.from_z0(_board_outline(), 0.018),
    )
    board_assembly.visual(
        board_shell,
        material=cover,
        name="board_shell",
    )
    board_assembly.visual(
        Box((0.46, 0.11, 0.035)),
        origin=Origin(xyz=(-0.10, 0.0, -0.0175)),
        material=steel,
        name="base_bracket",
    )
    board_assembly.visual(
        Box((0.24, 0.06, 0.025)),
        origin=Origin(xyz=(0.52, 0.0, -0.0125)),
        material=frame_paint,
        name="nose_support_rail",
    )
    board_assembly.visual(
        Box((0.08, 0.29, 0.03)),
        origin=Origin(xyz=(-0.12, -0.135, -0.015)),
        material=frame_paint,
        name="left_bracket_strap",
    )
    board_assembly.visual(
        Box((0.08, 0.29, 0.03)),
        origin=Origin(xyz=(-0.12, 0.135, -0.015)),
        material=frame_paint,
        name="right_bracket_strap",
    )

    fixed_rest_geom = tube_from_spline_points(
        [
            (0.48, -0.07, 0.0),
            (0.56, -0.07, -0.28),
            (0.62, -0.07, -0.79),
            (0.62, 0.07, -0.79),
            (0.56, 0.07, -0.28),
            (0.48, 0.07, 0.0),
        ],
        radius=0.012,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    board_assembly.visual(
        _mesh("fixed_rest_tube", fixed_rest_geom),
        material=frame_paint,
        name="fixed_rest_tube",
    )
    board_assembly.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.62, -0.05, -0.80), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="fixed_rest_left_foot",
    )
    board_assembly.visual(
        Cylinder(radius=0.018, length=0.10),
        origin=Origin(xyz=(0.62, 0.05, -0.80), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="fixed_rest_right_foot",
    )

    main_leg_frame = model.part("main_leg_frame")
    main_leg_frame.inertial = Inertial.from_geometry(
        Box((0.24, 0.52, 0.80)),
        mass=2.6,
        origin=Origin(xyz=(-0.12, 0.0, -0.39)),
    )

    leg_frame_geom = tube_from_spline_points(
        [
            (0.0, -0.23, 0.0),
            (-0.03, -0.225, -0.11),
            (-0.10, -0.215, -0.37),
            (-0.18, -0.205, -0.74),
            (-0.18, 0.205, -0.74),
            (-0.10, 0.215, -0.37),
            (-0.03, 0.225, -0.11),
            (0.0, 0.23, 0.0),
        ],
        radius=0.015,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    main_leg_frame.visual(
        _mesh("main_leg_frame", leg_frame_geom),
        material=frame_paint,
        name="leg_frame_tube",
    )
    main_leg_frame.visual(
        Box((0.05, 0.04, 0.03)),
        origin=Origin(xyz=(0.0, -0.225, -0.015)),
        material=steel,
        name="left_hinge_lug",
    )
    main_leg_frame.visual(
        Box((0.05, 0.04, 0.03)),
        origin=Origin(xyz=(0.0, 0.225, -0.015)),
        material=steel,
        name="right_hinge_lug",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.020, length=0.07),
        origin=Origin(xyz=(-0.18, -0.18, -0.755), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="left_leg_foot",
    )
    main_leg_frame.visual(
        Cylinder(radius=0.020, length=0.07),
        origin=Origin(xyz=(-0.18, 0.18, -0.755), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="right_leg_foot",
    )

    model.articulation(
        "main_leg_hinge",
        ArticulationType.REVOLUTE,
        parent=board_assembly,
        child=main_leg_frame,
        origin=Origin(xyz=(-0.12, 0.0, -0.03)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board_assembly = object_model.get_part("board_assembly")
    main_leg_frame = object_model.get_part("main_leg_frame")
    main_leg_hinge = object_model.get_articulation("main_leg_hinge")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_gap(
        board_assembly,
        main_leg_frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="left_bracket_strap",
        negative_elem="left_hinge_lug",
        name="left hinge lug seats against bracket strap",
    )
    ctx.expect_gap(
        board_assembly,
        main_leg_frame,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem="right_bracket_strap",
        negative_elem="right_hinge_lug",
        name="right hinge lug seats against bracket strap",
    )

    rest_foot_aabb = ctx.part_element_world_aabb(main_leg_frame, elem="left_leg_foot")
    with ctx.pose({main_leg_hinge: 0.60}):
        folded_foot_aabb = ctx.part_element_world_aabb(main_leg_frame, elem="left_leg_foot")
    ctx.check(
        "main leg frame folds upward",
        rest_foot_aabb is not None
        and folded_foot_aabb is not None
        and folded_foot_aabb[0][2] > rest_foot_aabb[0][2] + 0.10,
        details=f"rest={rest_foot_aabb}, folded={folded_foot_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
