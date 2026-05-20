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
    wire_from_points,
)


def _arc_points(
    cx: float,
    cy: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(start_angle + (end_angle - start_angle) * (index / segments)),
            cy + radius * math.sin(start_angle + (end_angle - start_angle) * (index / segments)),
        )
        for index in range(segments + 1)
    ]


def _ironing_board_outline() -> list[tuple[float, float]]:
    board_length = 1.48
    tail_half_width = 0.19
    nose_half_width = 0.055
    rear_center_x = -0.55
    nose_center_x = 0.69

    outline: list[tuple[float, float]] = [
        (-0.55, tail_half_width),
        (-0.18, tail_half_width),
        (0.12, 0.155),
        (0.38, 0.108),
        (nose_center_x, nose_half_width),
    ]
    outline.extend(
        _arc_points(
            nose_center_x,
            0.0,
            nose_half_width,
            math.pi / 2.0,
            -math.pi / 2.0,
            segments=10,
        )[1:]
    )
    outline.extend(
        [
            (0.38, -0.108),
            (0.12, -0.155),
            (-0.18, -tail_half_width),
            (-0.55, -tail_half_width),
        ]
    )
    outline.extend(
        _arc_points(
            rear_center_x,
            0.0,
            tail_half_width,
            -math.pi / 2.0,
            -3.0 * math.pi / 2.0,
            segments=14,
        )[1:-1]
    )
    return outline


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_ironing_board")

    cover = model.material("cover", rgba=(0.66, 0.70, 0.76, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    foot_rubber = model.material("foot_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    board_top = model.part("board_top")
    board_thickness = 0.024
    board_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(_ironing_board_outline(), board_thickness),
        "ironing_board_top",
    )
    board_top.visual(
        board_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.878)),
        material=cover,
        name="board_surface",
    )
    board_top.visual(
        Box((0.54, 0.020, 0.018)),
        origin=Origin(xyz=(-0.06, -0.115, 0.873)),
        material=dark_steel,
        name="left_underside_rail",
    )
    board_top.visual(
        Box((0.54, 0.020, 0.018)),
        origin=Origin(xyz=(-0.06, 0.115, 0.873)),
        material=dark_steel,
        name="right_underside_rail",
    )
    board_top.visual(
        Box((0.28, 0.035, 0.016)),
        origin=Origin(xyz=(0.33, 0.0, 0.872)),
        material=dark_steel,
        name="nose_spine",
    )
    board_top.visual(
        Box((0.13, 0.055, 0.018)),
        origin=Origin(xyz=(0.54, 0.0, 0.873)),
        material=dark_steel,
        name="end_rest_mount",
    )
    board_top.inertial = Inertial.from_geometry(
        Box((1.48, 0.39, 0.055)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, 0.889)),
    )

    base_bracket = model.part("base_bracket")
    base_bracket.visual(
        Box((0.42, 0.085, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="mount_plate",
    )
    base_bracket.visual(
        Cylinder(radius=0.008, length=0.110),
        origin=Origin(xyz=(0.07, 0.0, -0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    base_bracket.visual(
        Box((0.076, 0.030, 0.030)),
        origin=Origin(xyz=(0.032, 0.0, -0.033)),
        material=steel,
        name="hinge_saddle",
    )
    base_bracket.visual(
        Box((0.180, 0.020, 0.048)),
        origin=Origin(xyz=(-0.020, -0.030, -0.033)),
        material=steel,
        name="left_web",
    )
    base_bracket.visual(
        Box((0.180, 0.020, 0.048)),
        origin=Origin(xyz=(-0.020, 0.030, -0.033)),
        material=steel,
        name="right_web",
    )
    base_bracket.inertial = Inertial.from_geometry(
        Box((0.42, 0.10, 0.09)),
        mass=1.2,
        origin=Origin(xyz=(0.02, 0.0, -0.028)),
    )

    model.articulation(
        "board_to_bracket",
        ArticulationType.FIXED,
        parent=board_top,
        child=base_bracket,
        origin=Origin(xyz=(-0.10, 0.0, 0.878)),
    )

    leg_frame = model.part("leg_frame")
    leg_frame.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.004, -0.095, -0.004),
                    (0.098, -0.150, -0.338),
                    (0.205, -0.190, -0.820),
                    (0.205, 0.190, -0.820),
                    (0.098, 0.150, -0.338),
                    (0.004, 0.095, -0.004),
                ],
                radius=0.0085,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.070,
                corner_segments=10,
            ),
            "ironing_board_leg_frame",
        ),
        material=steel,
        name="main_u_frame",
    )
    leg_frame.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, -0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_hinge_collar",
    )
    leg_frame.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.0, 0.072, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_hinge_collar",
    )
    leg_frame.visual(
        Box((0.016, 0.015, 0.018)),
        origin=Origin(xyz=(0.0, -0.0965, 0.0)),
        material=steel,
        name="left_hinge_spacer",
    )
    leg_frame.visual(
        Box((0.016, 0.015, 0.018)),
        origin=Origin(xyz=(0.0, 0.0965, 0.0)),
        material=steel,
        name="right_hinge_spacer",
    )
    leg_frame.visual(
        Box((0.022, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.116, 0.0)),
        material=steel,
        name="left_hinge_pad",
    )
    leg_frame.visual(
        Box((0.022, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.116, 0.0)),
        material=steel,
        name="right_hinge_pad",
    )
    leg_frame.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.205, -0.190, -0.820), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=foot_rubber,
        name="left_main_foot",
    )
    leg_frame.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(0.205, 0.190, -0.820), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=foot_rubber,
        name="right_main_foot",
    )
    leg_frame.inertial = Inertial.from_geometry(
        Box((0.25, 0.42, 0.84)),
        mass=2.0,
        origin=Origin(xyz=(0.12, 0.0, -0.42)),
    )

    model.articulation(
        "bracket_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=base_bracket,
        child=leg_frame,
        origin=Origin(xyz=(0.07, 0.0, -0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.00,
        ),
    )

    end_rest = model.part("end_rest")
    end_rest.visual(
        Box((0.050, 0.155, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, -0.021)),
        material=steel,
        name="rest_saddle",
    )
    end_rest.visual(
        mesh_from_geometry(
            wire_from_points(
                [
                    (0.0, -0.070, -0.010),
                    (-0.015, -0.074, -0.395),
                    (-0.035, -0.080, -0.865),
                    (-0.035, 0.080, -0.865),
                    (-0.015, 0.074, -0.395),
                    (0.0, 0.070, -0.010),
                ],
                radius=0.0075,
                radial_segments=14,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.050,
                corner_segments=10,
            ),
            "ironing_board_end_rest",
        ),
        material=steel,
        name="fixed_rest_frame",
    )
    end_rest.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(-0.035, -0.080, -0.865), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=foot_rubber,
        name="left_rest_foot",
    )
    end_rest.visual(
        Cylinder(radius=0.011, length=0.040),
        origin=Origin(xyz=(-0.035, 0.080, -0.865), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=foot_rubber,
        name="right_rest_foot",
    )
    end_rest.inertial = Inertial.from_geometry(
        Box((0.08, 0.18, 0.88)),
        mass=0.8,
        origin=Origin(xyz=(-0.02, 0.0, -0.43)),
    )

    model.articulation(
        "board_to_end_rest",
        ArticulationType.FIXED,
        parent=board_top,
        child=end_rest,
        origin=Origin(xyz=(0.54, 0.0, 0.878)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board_top = object_model.get_part("board_top")
    base_bracket = object_model.get_part("base_bracket")
    leg_frame = object_model.get_part("leg_frame")
    end_rest = object_model.get_part("end_rest")
    hinge = object_model.get_articulation("bracket_to_leg_frame")

    ctx.expect_gap(
        board_top,
        base_bracket,
        axis="z",
        positive_elem="board_surface",
        negative_elem="mount_plate",
        max_gap=0.002,
        max_penetration=0.0,
        name="base bracket seats directly under the board top",
    )
    ctx.expect_gap(
        board_top,
        leg_frame,
        axis="z",
        min_gap=0.018,
        name="deployed leg frame stays below the board",
    )
    ctx.expect_origin_gap(
        end_rest,
        base_bracket,
        axis="x",
        min_gap=0.55,
        name="fixed rest is mounted toward the narrow end",
    )

    deployed_leg_aabb = ctx.part_world_aabb(leg_frame)
    deployed_rest_aabb = ctx.part_world_aabb(end_rest)
    hinge_upper = hinge.motion_limits.upper if hinge.motion_limits is not None else None

    with ctx.pose({hinge: hinge_upper if hinge_upper is not None else 0.0}):
        ctx.expect_gap(
            board_top,
            leg_frame,
            axis="z",
            max_gap=0.11,
            max_penetration=0.0,
            name="folded leg frame tucks close under the board",
        )
        folded_leg_aabb = ctx.part_world_aabb(leg_frame)

    ctx.check(
        "leg frame folds upward toward the underside",
        deployed_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[1][2] > deployed_leg_aabb[1][2] + 0.14,
        details=f"deployed={deployed_leg_aabb}, folded={folded_leg_aabb}",
    )
    ctx.check(
        "main trestle and fixed rest land at comparable floor height",
        deployed_leg_aabb is not None
        and deployed_rest_aabb is not None
        and abs(deployed_leg_aabb[0][2] - deployed_rest_aabb[0][2]) <= 0.035,
        details=f"leg_min_z={None if deployed_leg_aabb is None else deployed_leg_aabb[0][2]}, "
        f"rest_min_z={None if deployed_rest_aabb is None else deployed_rest_aabb[0][2]}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
