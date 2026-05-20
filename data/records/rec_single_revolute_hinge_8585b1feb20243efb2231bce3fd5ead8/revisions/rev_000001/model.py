from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _triangular_gusset(
    points_xz: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    *,
    y_center: float,
    thickness: float,
) -> MeshGeometry:
    """Return a thin triangular prism in the XZ plane, extruded along Y."""
    y0 = y_center - thickness / 2.0
    y1 = y_center + thickness / 2.0
    vertices = [
        (points_xz[0][0], y0, points_xz[0][1]),
        (points_xz[1][0], y0, points_xz[1][1]),
        (points_xz[2][0], y0, points_xz[2][1]),
        (points_xz[0][0], y1, points_xz[0][1]),
        (points_xz[1][0], y1, points_xz[1][1]),
        (points_xz[2][0], y1, points_xz[2][1]),
    ]
    faces = [
        (0, 1, 2),
        (3, 5, 4),
        (0, 3, 4),
        (0, 4, 1),
        (1, 4, 5),
        (1, 5, 2),
        (2, 5, 3),
        (2, 3, 0),
    ]
    return MeshGeometry(vertices=vertices, faces=faces)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gusseted_service_cover_hinge")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    dark_steel = model.material("dark_pin_steel", rgba=(0.12, 0.12, 0.13, 1.0))
    cover_paint = model.material("blue_gray_cover", rgba=(0.25, 0.35, 0.43, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.035, 0.035, 0.038, 1.0))

    hinge_axis_z = 0.065
    barrel_radius = 0.018
    pin_radius = 0.006

    bracket = model.part("bracket_leaf")
    bracket.visual(
        Box((0.190, 0.360, 0.008)),
        origin=Origin(xyz=(-0.030, 0.0, 0.004)),
        material=galvanized,
        name="ground_plate",
    )

    fixed_segments = (
        (-0.105, 0.066),
        (0.0, 0.060),
        (0.105, 0.066),
    )
    for index, (y_center, length) in enumerate(fixed_segments):
        bracket.visual(
            Box((0.020, length, 0.060)),
            origin=Origin(xyz=(-0.016, y_center, 0.038)),
            material=galvanized,
            name=f"barrel_support_{index}",
        )
        bracket.visual(
            Cylinder(radius=barrel_radius, length=length),
            origin=Origin(
                xyz=(0.0, y_center, hinge_axis_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=galvanized,
            name=f"fixed_knuckle_{index}",
        )
        bracket.visual(
            mesh_from_geometry(
                _triangular_gusset(
                    ((-0.098, 0.006), (-0.024, 0.006), (-0.024, 0.060)),
                    y_center=y_center,
                    thickness=0.018,
                ),
                f"bracket_gusset_{index}",
            ),
            material=galvanized,
            name=f"gusset_{index}",
        )

    bracket.visual(
        Cylinder(radius=pin_radius, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    for index, (x, y) in enumerate(
        ((-0.090, -0.135), (-0.090, 0.135), (0.030, -0.135), (0.030, 0.135))
    ):
        bracket.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(xyz=(x, y, 0.010)),
            material=bolt_black,
            name=f"base_bolt_{index}",
        )

    cover = model.part("cover_leaf")
    cover.visual(
        Box((0.380, 0.320, 0.010)),
        origin=Origin(xyz=(0.240, 0.0, -0.025)),
        material=cover_paint,
        name="cover_panel",
    )
    cover.visual(
        Box((0.360, 0.010, 0.020)),
        origin=Origin(xyz=(0.250, -0.165, -0.016)),
        material=cover_paint,
        name="side_flange_0",
    )
    cover.visual(
        Box((0.360, 0.010, 0.020)),
        origin=Origin(xyz=(0.250, 0.165, -0.016)),
        material=cover_paint,
        name="side_flange_1",
    )
    cover.visual(
        Box((0.010, 0.320, 0.020)),
        origin=Origin(xyz=(0.435, 0.0, -0.016)),
        material=cover_paint,
        name="front_lip",
    )

    moving_segments = ((-0.050, 0.036), (0.050, 0.036))
    for index, (y_center, length) in enumerate(moving_segments):
        cover.visual(
            Cylinder(radius=0.016, length=length),
            origin=Origin(xyz=(0.0, y_center, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=cover_paint,
            name=f"cover_knuckle_{index}",
        )
        cover.visual(
            Box((0.070, length, 0.010)),
            origin=Origin(xyz=(0.035, y_center, -0.016)),
            material=cover_paint,
            name=f"hinge_strap_{index}",
        )

    for index, (x, y) in enumerate(
        ((0.155, -0.105), (0.155, 0.105), (0.330, -0.105), (0.330, 0.105))
    ):
        cover.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(x, y, -0.019)),
            material=bolt_black,
            name=f"cover_bolt_{index}",
        )

    model.articulation(
        "bracket_to_cover",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cover,
        origin=Origin(xyz=(0.0, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket_leaf")
    cover = object_model.get_part("cover_leaf")
    hinge = object_model.get_articulation("bracket_to_cover")

    ctx.check(
        "single cover hinge joint",
        len(object_model.articulations) == 1 and hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"articulations={object_model.articulations!r}",
    )

    for knuckle_name in ("cover_knuckle_0", "cover_knuckle_1"):
        ctx.allow_overlap(
            bracket,
            cover,
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            reason="The fixed hinge pin intentionally passes through the rotating cover knuckle.",
        )
        ctx.expect_within(
            bracket,
            cover,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=knuckle_name,
            margin=0.002,
            name=f"pin centered in {knuckle_name}",
        )
        ctx.expect_overlap(
            bracket,
            cover,
            axes="y",
            elem_a="hinge_pin",
            elem_b=knuckle_name,
            min_overlap=0.030,
            name=f"pin spans {knuckle_name}",
        )

    bracket_aabb = ctx.part_world_aabb(bracket)
    cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "grounded bracket sits on floor plane",
        bracket_aabb is not None and abs(float(bracket_aabb[0][2])) <= 0.001,
        details=f"bracket_aabb={bracket_aabb!r}",
    )
    ctx.check(
        "cover leaf is larger than bracket leaf",
        bracket_aabb is not None
        and cover_aabb is not None
        and float(cover_aabb[1][0] - cover_aabb[0][0]) > float(bracket_aabb[1][0] - bracket_aabb[0][0]),
        details=f"bracket_aabb={bracket_aabb!r}, cover_aabb={cover_aabb!r}",
    )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 1.20}):
        opened_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "positive hinge rotation lifts cover",
        closed_aabb is not None
        and opened_aabb is not None
        and float(opened_aabb[1][2]) > float(closed_aabb[1][2]) + 0.20,
        details=f"closed={closed_aabb!r}, opened={opened_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
