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
    wire_from_points,
)


def _board_outline() -> list[tuple[float, float]]:
    control_points = [
        (-0.58, 0.00),
        (-0.54, -0.16),
        (-0.36, -0.19),
        (-0.04, -0.19),
        (0.22, -0.17),
        (0.45, -0.11),
        (0.60, -0.06),
        (0.68, 0.00),
        (0.60, 0.06),
        (0.45, 0.11),
        (0.22, 0.17),
        (-0.04, 0.19),
        (-0.36, 0.19),
        (-0.54, 0.16),
    ]
    return sample_catmull_rom_spline_2d(control_points, samples_per_segment=10, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    steel = model.material("steel", rgba=(0.72, 0.74, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.33, 0.37, 1.0))
    board_white = model.material("board_white", rgba=(0.94, 0.95, 0.96, 1.0))
    pad_blue = model.material("pad_blue", rgba=(0.44, 0.62, 0.79, 1.0))

    board_profile = _board_outline()
    board_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry(board_profile, 0.020, center=True),
        "board_shell",
    )
    pad_profile = [(0.97 * x, 0.94 * y) for x, y in board_profile]
    board_pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(pad_profile, 0.008, center=True),
        "board_pad",
    )

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.34, 0.10, 0.012)),
        origin=Origin(xyz=(0.07, 0.0, 0.054)),
        material=dark_steel,
        name="mount_plate",
    )
    for x in (0.00, 0.14):
        for y in (-0.035, 0.035):
            base_bracket.visual(
                Box((0.024, 0.024, 0.028)),
                origin=Origin(xyz=(x, y, 0.062)),
                material=dark_steel,
                name=f"standoff_{'front' if x > 0.05 else 'rear'}_{'l' if y < 0 else 'r'}",
            )
    for y in (-0.11, 0.11):
        base_bracket.visual(
            Box((0.060, 0.030, 0.048)),
            origin=Origin(xyz=(-0.010, y, 0.000)),
            material=steel,
            name=f"hinge_lug_{'l' if y < 0 else 'r'}",
        )
    base_bracket.visual(
        Box((0.10, 0.26, 0.016)),
        origin=Origin(xyz=(0.03, 0.0, 0.024)),
        material=dark_steel,
        name="hinge_bridge",
    )
    base_bracket.visual(
        Box((0.10, 0.06, 0.018)),
        origin=Origin(xyz=(0.06, 0.0, 0.039)),
        material=dark_steel,
        name="hinge_web",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.34, 0.26, 0.10)),
        mass=1.6,
        origin=Origin(xyz=(0.05, 0.0, 0.04)),
    )

    board_top = model.part("board_top")
    board_top.visual(
        board_shell_mesh,
        material=board_white,
        name="board_shell",
    )
    board_top.visual(
        board_pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=pad_blue,
        name="board_pad",
    )
    board_top.visual(
        Box((0.12, 0.30, 0.008)),
        origin=Origin(xyz=(-0.50, 0.0, -0.010)),
        material=dark_steel,
        name="tail_reinforcement",
    )
    board_top.inertial = Inertial.from_geometry(
        Box((1.26, 0.40, 0.032)),
        mass=5.5,
        origin=Origin(xyz=(0.03, 0.0, 0.008)),
    )

    leg_top_bracket = model.part("leg_top_bracket")
    leg_top_bracket.visual(
        Cylinder(radius=0.010, length=0.190),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_tube",
    )
    leg_top_bracket.visual(
        Box((0.060, 0.300, 0.026)),
        origin=Origin(xyz=(0.040, 0.0, -0.060)),
        material=steel,
        name="crossbar",
    )
    leg_top_bracket.visual(
        Box((0.018, 0.140, 0.080)),
        origin=Origin(xyz=(0.008, 0.0, -0.035)),
        material=steel,
        name="center_web",
    )
    for y in (-0.155, 0.155):
        leg_top_bracket.visual(
            Box((0.032, 0.030, 0.050)),
            origin=Origin(xyz=(0.055, y, -0.080)),
            material=steel,
            name=f"rail_socket_{'l' if y < 0 else 'r'}",
        )
    leg_top_bracket.inertial = Inertial.from_geometry(
        Box((0.09, 0.32, 0.08)),
        mass=1.2,
        origin=Origin(xyz=(0.03, 0.0, -0.02)),
    )

    rail_tube_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -0.060),
                (-0.028, 0.0, -0.230),
                (-0.085, 0.0, -0.560),
                (-0.150, 0.0, -0.885),
                (-0.095, 0.0, -0.885),
            ],
            radius=0.013,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        ),
        "leg_rail_tube",
    )

    left_leg_rail = model.part("left_leg_rail")
    left_leg_rail.visual(
        Box((0.030, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="mount_shoe",
    )
    left_leg_rail.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=steel,
        name="upper_stem",
    )
    left_leg_rail.visual(
        rail_tube_mesh,
        material=steel,
        name="rail_tube",
    )
    left_leg_rail.inertial = Inertial.from_geometry(
        Box((0.18, 0.04, 0.90)),
        mass=1.0,
        origin=Origin(xyz=(-0.08, 0.0, -0.45)),
    )

    right_leg_rail = model.part("right_leg_rail")
    right_leg_rail.visual(
        Box((0.030, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=steel,
        name="mount_shoe",
    )
    right_leg_rail.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=steel,
        name="upper_stem",
    )
    right_leg_rail.visual(
        rail_tube_mesh,
        material=steel,
        name="rail_tube",
    )
    right_leg_rail.inertial = Inertial.from_geometry(
        Box((0.18, 0.04, 0.90)),
        mass=1.0,
        origin=Origin(xyz=(-0.08, 0.0, -0.45)),
    )

    end_rest = model.part("end_rest")
    end_rest.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.0, -0.055, 0.0),
                    (0.0, -0.055, -0.220),
                    (0.0, 0.055, -0.220),
                    (0.0, 0.055, 0.0),
                ],
                radius=0.0095,
                radial_segments=16,
                closed_path=True,
                corner_mode="fillet",
                corner_radius=0.028,
                corner_segments=8,
            ),
            "end_rest_loop",
        ),
        material=steel,
        name="rest_loop",
    )
    end_rest.visual(
        Box((0.024, 0.118, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_steel,
        name="mount_saddle",
    )
    end_rest.inertial = Inertial.from_geometry(
        Box((0.04, 0.13, 0.23)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
    )

    model.articulation(
        "bracket_to_board",
        ArticulationType.FIXED,
        parent=base_bracket,
        child=board_top,
        origin=Origin(xyz=(0.10, 0.0, 0.086)),
    )
    model.articulation(
        "bracket_to_leg_top",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=leg_top_bracket,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.8,
            lower=-0.15,
            upper=1.35,
        ),
    )
    model.articulation(
        "leg_top_to_left_rail",
        ArticulationType.FIXED,
        parent=leg_top_bracket,
        child=left_leg_rail,
        origin=Origin(xyz=(0.055, -0.155, -0.105)),
    )
    model.articulation(
        "leg_top_to_right_rail",
        ArticulationType.FIXED,
        parent=leg_top_bracket,
        child=right_leg_rail,
        origin=Origin(xyz=(0.055, 0.155, -0.105)),
    )
    model.articulation(
        "board_to_end_rest",
        ArticulationType.FIXED,
        parent=board_top,
        child=end_rest,
        origin=Origin(xyz=(0.50, 0.0, -0.024)),
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

    board_top = object_model.get_part("board_top")
    base_bracket = object_model.get_part("base_bracket")
    leg_top_bracket = object_model.get_part("leg_top_bracket")
    left_leg_rail = object_model.get_part("left_leg_rail")
    right_leg_rail = object_model.get_part("right_leg_rail")
    end_rest = object_model.get_part("end_rest")
    leg_hinge = object_model.get_articulation("bracket_to_leg_top")

    ctx.expect_overlap(
        board_top,
        base_bracket,
        axes="xy",
        min_overlap=0.10,
        name="board footprint covers the base bracket",
    )
    with ctx.pose({leg_hinge: 0.0}):
        ctx.expect_gap(
            board_top,
            leg_top_bracket,
            axis="z",
            min_gap=0.010,
            max_gap=0.090,
            name="leg top bracket sits directly below the board",
        )
        ctx.expect_contact(
            left_leg_rail,
            leg_top_bracket,
            elem_a="mount_shoe",
            elem_b="rail_socket_l",
            name="left rail mounts into the top bracket socket",
        )
        ctx.expect_contact(
            right_leg_rail,
            leg_top_bracket,
            elem_a="mount_shoe",
            elem_b="rail_socket_r",
            name="right rail mounts into the top bracket socket",
        )
        ctx.expect_contact(
            end_rest,
            board_top,
            elem_a="mount_saddle",
            elem_b="board_shell",
            name="small fixed rest mounts against the board underside",
        )

    with ctx.pose({leg_hinge: 0.0}):
        open_left_aabb = ctx.part_world_aabb(left_leg_rail)
    with ctx.pose({leg_hinge: 1.20}):
        folded_left_aabb = ctx.part_world_aabb(left_leg_rail)
        ctx.expect_gap(
            board_top,
            left_leg_rail,
            axis="z",
            min_gap=0.015,
            max_gap=0.200,
            name="folded left rail clears the board underside",
        )
        ctx.expect_gap(
            board_top,
            right_leg_rail,
            axis="z",
            min_gap=0.015,
            max_gap=0.200,
            name="folded right rail clears the board underside",
        )
    ctx.check(
        "main trestle folds upward toward the board",
        open_left_aabb is not None
        and folded_left_aabb is not None
        and folded_left_aabb[0][2] > open_left_aabb[0][2] + 0.30,
        details=f"open_aabb={open_left_aabb}, folded_aabb={folded_left_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
