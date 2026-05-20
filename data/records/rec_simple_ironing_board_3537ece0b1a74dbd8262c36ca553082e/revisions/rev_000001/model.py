from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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


BOARD_LENGTH = 1.30
BOARD_THICKNESS = 0.018
BOARD_HALF_WIDTH = 0.185


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _board_outline() -> list[tuple[float, float]]:
    control_points = [
        (-0.64, 0.145),
        (-0.52, 0.176),
        (-0.22, 0.180),
        (0.10, 0.168),
        (0.42, 0.126),
        (0.68, 0.085),
        (0.82, 0.040),
        (0.89, 0.000),
        (0.82, -0.040),
        (0.68, -0.085),
        (0.42, -0.126),
        (0.10, -0.168),
        (-0.22, -0.180),
        (-0.52, -0.176),
        (-0.64, -0.145),
    ]
    return sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=10,
        closed=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    cover_fabric = model.material("cover_fabric", rgba=(0.74, 0.80, 0.92, 1.0))
    steel_paint = model.material("steel_paint", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    foot_cap = model.material("foot_cap", rgba=(0.16, 0.16, 0.17, 1.0))

    board_top = model.part("board_top")
    board_mesh = _mesh(
        "ironing_board_top",
        ExtrudeGeometry(_board_outline(), BOARD_THICKNESS, center=True),
    )
    board_top.visual(board_mesh, material=cover_fabric, name="board_shell")
    board_top.visual(
        Box((0.16, 0.11, 0.008)),
        origin=Origin(xyz=(0.36, 0.0, -0.013)),
        material=dark_steel,
        name="tip_reinforcement",
    )
    board_top.inertial = Inertial.from_geometry(
        Box((BOARD_LENGTH, BOARD_HALF_WIDTH * 2.0, BOARD_THICKNESS)),
        mass=5.2,
    )

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.42, 0.070, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark_steel,
        name="bracket_rail",
    )
    base_bracket.visual(
        Box((0.048, 0.060, 0.030)),
        origin=Origin(xyz=(-0.14, 0.0, -0.015)),
        material=dark_steel,
        name="mount_pad_aft",
    )
    base_bracket.visual(
        Box((0.048, 0.060, 0.030)),
        origin=Origin(xyz=(0.14, 0.0, -0.015)),
        material=dark_steel,
        name="mount_pad_fore",
    )
    base_bracket.visual(
        Box((0.078, 0.050, 0.046)),
        origin=Origin(xyz=(0.12, 0.0, -0.057)),
        material=dark_steel,
        name="hinge_saddle",
    )
    base_bracket.visual(
        Cylinder(radius=0.014, length=0.190),
        origin=Origin(xyz=(0.12, 0.0, -0.067), rpy=(radians(90.0), 0.0, 0.0)),
        material=steel_paint,
        name="hinge_tube",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.42, 0.19, 0.09)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    model.articulation(
        "board_to_bracket",
        ArticulationType.FIXED,
        parent=board_top,
        child=base_bracket,
        origin=Origin(xyz=(-0.02, 0.0, -0.009)),
    )

    main_leg_frame = model.part("main_leg_frame")
    main_leg_frame.visual(
        _mesh(
            "main_leg_frame",
            wire_from_points(
                [
                    (0.0, -0.155, 0.0),
                    (-0.10, -0.155, -0.70),
                    (-0.10, 0.155, -0.70),
                    (0.0, 0.155, 0.0),
                ],
                radius=0.012,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.060,
                corner_segments=10,
            ),
        ),
        material=steel_paint,
        name="main_leg_tube",
    )
    main_leg_frame.visual(
        Box((0.028, 0.335, 0.018)),
        origin=Origin(xyz=(-0.10, 0.0, -0.70)),
        material=foot_cap,
        name="main_leg_foot_bar",
    )
    main_leg_frame.visual(
        Box((0.030, 0.220, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=dark_steel,
        name="main_leg_crown",
    )
    main_leg_frame.visual(
        Box((0.030, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, -0.135, -0.015)),
        material=dark_steel,
        name="main_leg_left_ear",
    )
    main_leg_frame.visual(
        Box((0.030, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.135, -0.015)),
        material=dark_steel,
        name="main_leg_right_ear",
    )
    main_leg_frame.inertial = Inertial.from_geometry(
        Box((0.16, 0.34, 0.73)),
        mass=2.1,
        origin=Origin(xyz=(-0.05, 0.0, -0.35)),
    )

    model.articulation(
        "bracket_to_main_leg",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=main_leg_frame,
        origin=Origin(xyz=(0.12, 0.0, -0.067)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=radians(77.0),
        ),
    )

    end_rest = model.part("end_rest")
    end_rest.visual(
        Box((0.060, 0.230, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=dark_steel,
        name="rest_mount_plate",
    )
    end_rest.visual(
        _mesh(
            "end_rest_frame",
            wire_from_points(
                [
                    (0.0, -0.10, 0.0),
                    (0.09, -0.10, -0.58),
                    (0.09, 0.10, -0.58),
                    (0.0, 0.10, 0.0),
                ],
                radius=0.010,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.045,
                corner_segments=8,
            ),
        ),
        material=steel_paint,
        name="rest_tube",
    )
    end_rest.visual(
        Box((0.026, 0.215, 0.016)),
        origin=Origin(xyz=(0.09, 0.0, -0.58)),
        material=foot_cap,
        name="rest_foot_bar",
    )
    end_rest.inertial = Inertial.from_geometry(
        Box((0.12, 0.24, 0.60)),
        mass=0.9,
        origin=Origin(xyz=(0.045, 0.0, -0.29)),
    )

    model.articulation(
        "board_to_end_rest",
        ArticulationType.FIXED,
        parent=board_top,
        child=end_rest,
        origin=Origin(xyz=(0.54, 0.055, -0.009)),
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
    main_leg_frame = object_model.get_part("main_leg_frame")
    end_rest = object_model.get_part("end_rest")
    main_leg_hinge = object_model.get_articulation("bracket_to_main_leg")

    ctx.expect_gap(
        board_top,
        base_bracket,
        axis="z",
        positive_elem="board_shell",
        negative_elem="mount_pad_aft",
        min_gap=0.0,
        max_gap=0.001,
        name="bracket mount pads sit tight to board underside",
    )
    ctx.expect_overlap(
        board_top,
        base_bracket,
        axes="xy",
        elem_a="board_shell",
        elem_b="bracket_rail",
        min_overlap=0.05,
        name="base bracket sits beneath the board footprint",
    )
    ctx.expect_gap(
        board_top,
        end_rest,
        axis="z",
        positive_elem="board_shell",
        negative_elem="rest_mount_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="fixed rest mounts directly to the board underside",
    )
    ctx.expect_gap(
        board_top,
        main_leg_frame,
        axis="z",
        positive_elem="board_shell",
        negative_elem="main_leg_foot_bar",
        min_gap=0.70,
        name="deployed main leg reaches well below the board",
    )

    with ctx.pose({main_leg_hinge: main_leg_hinge.motion_limits.upper}):
        ctx.expect_gap(
            board_top,
            main_leg_frame,
            axis="z",
            positive_elem="board_shell",
            negative_elem="main_leg_foot_bar",
            min_gap=0.03,
            max_gap=0.13,
            name="folded main leg tucks near the underside",
        )
        ctx.expect_overlap(
            board_top,
            main_leg_frame,
            axes="x",
            elem_a="board_shell",
            elem_b="main_leg_foot_bar",
            min_overlap=0.02,
            name="folded main leg stays under the board length",
        )
        ctx.expect_gap(
            end_rest,
            main_leg_frame,
            axis="x",
            min_gap=0.30,
            name="folded main leg clears the fixed rest",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
