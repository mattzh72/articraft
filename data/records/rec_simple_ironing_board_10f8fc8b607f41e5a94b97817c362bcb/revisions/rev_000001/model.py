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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _scale_profile(
    profile: list[tuple[float, float]],
    *,
    sx: float = 1.0,
    sy: float = 1.0,
) -> list[tuple[float, float]]:
    return [(x * sx, y * sy) for x, y in profile]


def _board_profile() -> list[tuple[float, float]]:
    upper = sample_catmull_rom_spline_2d(
        [
            (-0.69, 0.19),
            (-0.48, 0.19),
            (-0.16, 0.18),
            (0.16, 0.16),
            (0.43, 0.11),
            (0.63, 0.07),
            (0.73, 0.03),
            (0.76, 0.0),
        ],
        samples_per_segment=9,
    )
    lower = [(x, -y) for x, y in reversed(upper[:-1])]
    return upper + lower


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    cover_fabric = model.material("cover_fabric", rgba=(0.72, 0.77, 0.82, 1.0))
    board_shell = model.material("board_shell", rgba=(0.93, 0.93, 0.91, 1.0))
    steel = model.material("steel", rgba=(0.67, 0.69, 0.73, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    board_profile = _board_profile()
    board_deck = _mesh(
        "ironing_board_deck",
        ExtrudeGeometry.from_z0(board_profile, 0.012),
    )
    board_cover = _mesh(
        "ironing_board_cover",
        ExtrudeGeometry.from_z0(_scale_profile(board_profile, sx=0.985, sy=0.965), 0.007),
    )

    fixed_rest_geom = tube_from_spline_points(
        [
            (0.42, -0.12, -0.75),
            (0.53, -0.12, -0.40),
            (0.60, -0.11, -0.05),
            (0.60, 0.11, -0.05),
            (0.53, 0.12, -0.40),
            (0.42, 0.12, -0.75),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )

    board_assembly = model.part("board_assembly")
    board_assembly.visual(board_deck, material=board_shell, name="board_deck")
    board_assembly.visual(
        board_cover,
        origin=Origin(xyz=(0.0, 0.0, 0.0115)),
        material=cover_fabric,
        name="board_cover",
    )

    board_assembly.visual(
        Box((0.70, 0.034, 0.028)),
        origin=Origin(xyz=(-0.03, -0.072, -0.013)),
        material=steel,
        name="left_long_rail",
    )
    board_assembly.visual(
        Box((0.70, 0.034, 0.028)),
        origin=Origin(xyz=(-0.03, 0.072, -0.013)),
        material=steel,
        name="right_long_rail",
    )
    board_assembly.visual(
        Box((0.18, 0.18, 0.018)),
        origin=Origin(xyz=(-0.08, 0.0, -0.008)),
        material=steel,
        name="center_bridge",
    )
    board_assembly.visual(
        Box((0.16, 0.10, 0.018)),
        origin=Origin(xyz=(-0.38, 0.0, -0.008)),
        material=steel,
        name="tail_mount_plate",
    )
    board_assembly.visual(
        Box((0.22, 0.07, 0.018)),
        origin=Origin(xyz=(0.45, 0.0, -0.008)),
        material=steel,
        name="nose_mount_plate",
    )

    board_assembly.visual(
        Box((0.08, 0.014, 0.072)),
        origin=Origin(xyz=(-0.08, -0.041, -0.035)),
        material=dark_steel,
        name="left_hinge_plate",
    )
    board_assembly.visual(
        Box((0.08, 0.014, 0.072)),
        origin=Origin(xyz=(-0.08, 0.041, -0.035)),
        material=dark_steel,
        name="right_hinge_plate",
    )
    board_assembly.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.08, -0.040, -0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    board_assembly.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.08, 0.040, -0.070), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="right_hinge_barrel",
    )

    board_assembly.visual(
        Box((0.045, 0.018, 0.050)),
        origin=Origin(xyz=(0.585, -0.060, -0.024)),
        material=dark_steel,
        name="left_rest_mount",
    )
    board_assembly.visual(
        Box((0.045, 0.018, 0.050)),
        origin=Origin(xyz=(0.585, 0.060, -0.024)),
        material=dark_steel,
        name="right_rest_mount",
    )
    board_assembly.visual(
        _mesh("ironing_board_fixed_rest", fixed_rest_geom),
        material=steel,
        name="fixed_rest",
    )
    board_assembly.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.42, -0.12, -0.75)),
        material=rubber,
        name="left_rest_foot",
    )
    board_assembly.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.42, 0.12, -0.75)),
        material=rubber,
        name="right_rest_foot",
    )
    board_assembly.inertial = Inertial.from_geometry(
        Box((1.52, 0.40, 0.78)),
        mass=8.0,
        origin=Origin(xyz=(0.03, 0.0, -0.36)),
    )

    main_frame_geom = tube_from_spline_points(
        [
            (-0.22, -0.24, -0.82),
            (-0.12, -0.24, -0.50),
            (-0.03, -0.24, -0.12),
            (-0.01, -0.22, -0.08),
            (-0.01, 0.22, -0.08),
            (-0.03, 0.24, -0.12),
            (-0.12, 0.24, -0.50),
            (-0.22, 0.24, -0.82),
        ],
        radius=0.013,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )

    leg_frame = model.part("leg_frame")
    leg_frame.visual(
        _mesh("ironing_board_leg_frame", main_frame_geom),
        material=steel,
        name="main_frame",
    )
    leg_frame.visual(
        Box((0.065, 0.052, 0.102)),
        origin=Origin(xyz=(-0.030, 0.0, -0.049)),
        material=dark_steel,
        name="hinge_lug",
    )
    leg_frame.visual(
        Cylinder(radius=0.0115, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="leg_hinge_barrel",
    )
    leg_frame.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(-0.22, -0.24, -0.82)),
        material=rubber,
        name="left_foot_cap",
    )
    leg_frame.visual(
        Sphere(radius=0.017),
        origin=Origin(xyz=(-0.22, 0.24, -0.82)),
        material=rubber,
        name="right_foot_cap",
    )
    leg_frame.inertial = Inertial.from_geometry(
        Box((0.26, 0.50, 0.84)),
        mass=2.1,
        origin=Origin(xyz=(-0.10, 0.0, -0.41)),
    )

    model.articulation(
        "board_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=board_assembly,
        child=leg_frame,
        origin=Origin(xyz=(-0.08, 0.0, -0.070)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.6,
            lower=0.0,
            upper=1.26,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("board_assembly")
    leg = object_model.get_part("leg_frame")
    fold_joint = object_model.get_articulation("board_to_leg_frame")

    ctx.expect_gap(
        board,
        leg,
        axis="z",
        positive_elem="board_deck",
        negative_elem="main_frame",
        min_gap=0.06,
        max_gap=0.16,
        name="open leg frame hangs clearly below the board deck",
    )
    ctx.expect_overlap(
        board,
        leg,
        axes="xy",
        elem_a="center_bridge",
        elem_b="main_frame",
        min_overlap=0.10,
        name="main trestle stays centered under the hinge bracket",
    )

    open_foot = ctx.part_element_world_aabb(leg, elem="left_foot_cap")
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper}):
        ctx.expect_gap(
            board,
            leg,
            axis="z",
            positive_elem="board_deck",
            negative_elem="main_frame",
            min_gap=0.0,
            max_gap=0.12,
            name="folded leg frame nests beneath the board without penetrating it",
        )
        folded_foot = ctx.part_element_world_aabb(leg, elem="left_foot_cap")

    open_z = open_foot[1][2] if open_foot is not None else None
    folded_z = folded_foot[1][2] if folded_foot is not None else None
    ctx.check(
        "folding rotation lifts the trestle toward the underside",
        open_z is not None and folded_z is not None and folded_z > open_z + 0.55,
        details=f"open_z={open_z}, folded_z={folded_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
