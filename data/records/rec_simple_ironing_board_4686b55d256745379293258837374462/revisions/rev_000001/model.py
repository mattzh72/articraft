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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _board_profile() -> list[tuple[float, float]]:
    return [
        (-0.61, -0.19),
        (-0.52, -0.20),
        (-0.26, -0.20),
        (0.08, -0.19),
        (0.32, -0.15),
        (0.48, -0.10),
        (0.57, -0.05),
        (0.62, 0.0),
        (0.57, 0.05),
        (0.48, 0.10),
        (0.32, 0.15),
        (0.08, 0.19),
        (-0.26, 0.20),
        (-0.52, 0.20),
        (-0.61, 0.19),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ironing_board")

    cover = model.material("cover", rgba=(0.86, 0.88, 0.92, 1.0))
    pad = model.material("pad", rgba=(0.96, 0.97, 0.99, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.71, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    board_mesh = _save_mesh(
        "ironing_board_top",
        ExtrudeGeometry.from_z0(_board_profile(), 0.018),
    )
    pad_mesh = _save_mesh(
        "ironing_board_pad",
        ExtrudeGeometry.from_z0(_board_profile(), 0.006),
    )

    top_assembly = model.part("top_assembly")
    top_assembly.visual(
        board_mesh,
        material=cover,
        name="board_shell",
    )
    top_assembly.visual(
        pad_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=pad,
        name="cover_pad",
    )
    top_assembly.visual(
        Box((0.42, 0.22, 0.016)),
        origin=Origin(xyz=(-0.06, 0.0, -0.007)),
        material=dark_steel,
        name="mounting_plate",
    )
    top_assembly.visual(
        Box((0.34, 0.055, 0.05)),
        origin=Origin(xyz=(-0.06, -0.1375, -0.036)),
        material=steel,
        name="left_bracket_rail",
    )
    top_assembly.visual(
        Box((0.34, 0.055, 0.05)),
        origin=Origin(xyz=(-0.06, 0.1375, -0.036)),
        material=steel,
        name="right_bracket_rail",
    )
    top_assembly.visual(
        Box((0.08, 0.24, 0.034)),
        origin=Origin(xyz=(0.07, 0.0, -0.028)),
        material=steel,
        name="front_bridge",
    )
    top_assembly.visual(
        Box((0.05, 0.03, 0.08)),
        origin=Origin(xyz=(-0.06, -0.115, -0.006)),
        material=dark_steel,
        name="left_hinge_cheek",
    )
    top_assembly.visual(
        Box((0.05, 0.03, 0.08)),
        origin=Origin(xyz=(-0.06, 0.115, -0.006)),
        material=dark_steel,
        name="right_hinge_cheek",
    )
    top_assembly.visual(
        Box((0.11, 0.025, 0.010)),
        origin=Origin(xyz=(-0.66, -0.105, 0.028)),
        material=steel,
        name="iron_rest_arm_left",
    )
    top_assembly.visual(
        Box((0.11, 0.025, 0.010)),
        origin=Origin(xyz=(-0.66, 0.105, 0.028)),
        material=steel,
        name="iron_rest_arm_right",
    )
    top_assembly.visual(
        Box((0.18, 0.27, 0.010)),
        origin=Origin(xyz=(-0.73, 0.0, 0.032)),
        material=steel,
        name="iron_rest_plate",
    )
    top_assembly.visual(
        Box((0.014, 0.27, 0.025)),
        origin=Origin(xyz=(-0.81, 0.0, 0.040)),
        material=steel,
        name="iron_rest_stop",
    )
    top_assembly.inertial = Inertial.from_geometry(
        Box((1.55, 0.45, 0.12)),
        mass=6.5,
        origin=Origin(xyz=(-0.10, 0.0, 0.015)),
    )

    leg_frame = model.part("leg_frame")
    leg_frame.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_tube",
    )
    leg_frame.visual(
        Cylinder(radius=0.012, length=0.722),
        origin=Origin(xyz=(0.0, -0.135, -0.36), rpy=(-0.097, 0.0, 0.0)),
        material=steel,
        name="left_leg",
    )
    leg_frame.visual(
        Cylinder(radius=0.012, length=0.722),
        origin=Origin(xyz=(0.0, 0.135, -0.36), rpy=(0.097, 0.0, 0.0)),
        material=steel,
        name="right_leg",
    )
    leg_frame.visual(
        Cylinder(radius=0.012, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, -0.72), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="bottom_crossbar",
    )
    leg_frame.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, -0.170, -0.735)),
        material=rubber,
        name="left_foot",
    )
    leg_frame.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.170, -0.735)),
        material=rubber,
        name="right_foot",
    )
    leg_frame.inertial = Inertial.from_geometry(
        Box((0.08, 0.40, 0.78)),
        mass=2.8,
        origin=Origin(xyz=(0.01, 0.0, -0.39)),
    )

    end_rest = model.part("end_rest")
    end_rest.visual(
        _save_mesh(
            "ironing_board_end_rest",
            wire_from_points(
                [
                    (0.000, -0.100, -0.009),
                    (-0.010, -0.100, -0.295),
                    (-0.010, 0.100, -0.295),
                    (0.000, 0.100, -0.009),
                ],
                radius=0.009,
                radial_segments=16,
                cap_ends=True,
                corner_mode="fillet",
                corner_radius=0.030,
                corner_segments=8,
            ),
        ),
        material=steel,
        name="rest_frame",
    )
    end_rest.visual(
        Box((0.060, 0.220, 0.026)),
        origin=Origin(xyz=(0.000, 0.0, -0.013)),
        material=dark_steel,
        name="rest_mount",
    )
    end_rest.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.010, -0.100, -0.295)),
        material=rubber,
        name="left_rest_foot",
    )
    end_rest.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.010, 0.100, -0.295)),
        material=rubber,
        name="right_rest_foot",
    )
    end_rest.inertial = Inertial.from_geometry(
        Box((0.05, 0.24, 0.33)),
        mass=0.9,
        origin=Origin(xyz=(-0.005, 0.0, -0.16)),
    )

    model.articulation(
        "board_to_leg_frame",
        ArticulationType.REVOLUTE,
        parent=top_assembly,
        child=leg_frame,
        origin=Origin(xyz=(-0.06, 0.0, -0.058)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=2.2,
            lower=0.0,
            upper=1.47,
        ),
    )
    model.articulation(
        "board_to_end_rest",
        ArticulationType.FIXED,
        parent=top_assembly,
        child=end_rest,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_assembly = object_model.get_part("top_assembly")
    leg_frame = object_model.get_part("leg_frame")
    end_rest = object_model.get_part("end_rest")
    leg_hinge = object_model.get_articulation("board_to_leg_frame")

    ctx.expect_gap(
        top_assembly,
        leg_frame,
        axis="z",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="left_hinge_cheek",
        negative_elem="hinge_tube",
        name="leg frame hangs just below the mounting bracket",
    )
    ctx.expect_within(
        leg_frame,
        top_assembly,
        axes="y",
        margin=0.030,
        inner_elem="bottom_crossbar",
        outer_elem="board_shell",
        name="leg frame stays within board width",
    )
    ctx.expect_origin_gap(
        end_rest,
        top_assembly,
        axis="x",
        min_gap=0.30,
        name="small fixed rest is mounted near the nose end",
    )

    rest_aabb = ctx.part_world_aabb(leg_frame)
    with ctx.pose({leg_hinge: 1.40}):
        ctx.expect_gap(
            top_assembly,
            leg_frame,
            axis="z",
            min_gap=0.0,
            max_gap=0.22,
            positive_elem="mounting_plate",
            negative_elem="bottom_crossbar",
            name="folded leg frame remains tucked below the board",
        )
        folded_aabb = ctx.part_world_aabb(leg_frame)

    ctx.check(
        "leg frame folds upward toward the nose",
        rest_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][2] > rest_aabb[0][2] + 0.45
        and folded_aabb[1][0] > rest_aabb[1][0] + 0.35,
        details=f"rest_aabb={rest_aabb}, folded_aabb={folded_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
