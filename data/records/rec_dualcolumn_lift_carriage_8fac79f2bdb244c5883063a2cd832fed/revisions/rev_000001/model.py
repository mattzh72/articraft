from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _bearing_block_mesh(
    *,
    width: float = 0.17,
    depth: float = 0.19,
    height: float = 0.16,
    bore_radius: float = 0.052,
) -> object:
    """One linear bearing block: a machined block with a vertical guide bore."""

    block = (
        cq.Workplane("XY")
        .box(width, depth, height)
        .edges("|Z")
        .fillet(0.006)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(bore_radius)
        .cutThruAll()
    )
    # Four shallow counterbores on the front face read as cap screw pockets.
    for x in (-0.052, 0.052):
        for z in (-0.050, 0.050):
            cutter = (
                cq.Workplane("XZ")
                .workplane(offset=-depth / 2.0 - 0.001)
                .center(x, z)
                .circle(0.014)
                .extrude(0.012)
            )
            block = block.cut(cutter)
    return block


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_column_lift_carriage")

    dark_paint = Material("satin_black_powder_coat", rgba=(0.02, 0.023, 0.025, 1.0))
    charcoal = Material("dark_machined_carriage", rgba=(0.10, 0.11, 0.12, 1.0))
    steel = Material("brushed_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    bolt_black = Material("black_oxide_fasteners", rgba=(0.005, 0.005, 0.006, 1.0))
    safety_blue = Material("blue_load_face", rgba=(0.05, 0.22, 0.55, 1.0))
    rubber = Material("matte_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    bronze = Material("oil_bronze_bearing_shoes", rgba=(0.72, 0.48, 0.22, 1.0))

    # Static machine frame: two polished columns held by a base and a top tie.
    frame = model.part("frame")
    frame.visual(
        Box((1.02, 0.36, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_paint,
        name="base_plate",
    )
    frame.visual(
        Box((0.96, 0.30, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.470)),
        material=dark_paint,
        name="top_tie_plate",
    )
    frame.visual(
        Box((0.88, 0.050, 1.370)),
        origin=Origin(xyz=(0.0, 0.165, 0.755)),
        material=dark_paint,
        name="rear_spine",
    )

    column_x = 0.350
    column_radius = 0.040
    column_center_z = 0.755
    column_length = 1.370
    for i, x in enumerate((-column_x, column_x)):
        frame.visual(
            Cylinder(radius=column_radius, length=column_length),
            origin=Origin(xyz=(x, 0.0, column_center_z)),
            material=steel,
            name=f"guide_column_{i}",
        )
        frame.visual(
            Box((0.22, 0.18, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.084)),
            material=dark_paint,
            name=f"lower_column_clamp_{i}",
        )
        frame.visual(
            Box((0.22, 0.18, 0.028)),
            origin=Origin(xyz=(x, 0.0, 1.420)),
            material=dark_paint,
            name=f"upper_column_clamp_{i}",
        )

    for i, x in enumerate((-0.43, 0.43)):
        for j, y in enumerate((-0.13, 0.13)):
            frame.visual(
                Cylinder(radius=0.045, length=0.018),
                origin=Origin(xyz=(x, y, -0.009)),
                material=rubber,
                name=f"leveling_foot_{i}_{j}",
            )

    # Moving carriage.  Its local origin is the middle of the crosshead; the
    # prismatic joint places that origin on the column centerline at low travel.
    carriage = model.part("carriage")
    bearing_mesh = mesh_from_cadquery(
        _bearing_block_mesh(),
        "linear_bearing_block",
        tolerance=0.0008,
        angular_tolerance=0.08,
    )

    # A rigid front crosshead bridges both columns and both bearing levels.
    carriage.visual(
        Box((0.82, 0.080, 0.410)),
        origin=Origin(xyz=(0.0, -0.160, 0.0)),
        material=charcoal,
        name="crosshead_web",
    )
    carriage.visual(
        Box((0.70, 0.030, 0.260)),
        origin=Origin(xyz=(0.0, -0.218, 0.0)),
        material=safety_blue,
        name="load_mount_face",
    )
    carriage.visual(
        Box((0.88, 0.110, 0.052)),
        origin=Origin(xyz=(0.0, -0.130, 0.240)),
        material=charcoal,
        name="upper_cross_rail",
    )
    carriage.visual(
        Box((0.88, 0.110, 0.052)),
        origin=Origin(xyz=(0.0, -0.130, -0.240)),
        material=charcoal,
        name="lower_cross_rail",
    )

    # Four linear bearing blocks surround the columns with real clearance bores.
    bearing_positions = [
        (-column_x, 0.0, -0.170),
        (column_x, 0.0, -0.170),
        (-column_x, 0.0, 0.170),
        (column_x, 0.0, 0.170),
    ]
    for i, (x, y, z) in enumerate(bearing_positions):
        carriage.visual(
            bearing_mesh,
            origin=Origin(xyz=(x, y, z)),
            material=charcoal,
            name=f"bearing_block_{i}",
        )
        outward = -1.0 if x < 0.0 else 1.0
        carriage.visual(
            Box((0.012, 0.032, 0.070)),
            origin=Origin(xyz=(x + outward * 0.046, 0.0, z)),
            material=bronze,
            name=f"bearing_shoe_{i}",
        )
        # Short, thick machined lugs tie each block back into the front web.
        carriage.visual(
            Box((0.150, 0.042, 0.080)),
            origin=Origin(xyz=(x, -0.108, z)),
            material=charcoal,
            name=f"bearing_lug_{i}",
        )

        for sx in (-0.052, 0.052):
            for sz in (-0.048, 0.048):
                carriage.visual(
                    Cylinder(radius=0.010, length=0.010),
                    origin=Origin(
                        xyz=(x + sx, -0.101, z + sz),
                        rpy=(math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=bolt_black,
                    name=f"bearing_bolt_{i}_{0 if sx < 0 else 1}_{0 if sz < 0 else 1}",
                )

    # Central lifting lug and small reference pointer reinforce the machine-lift
    # carriage identity without adding extra degrees of freedom.
    carriage.visual(
        Box((0.150, 0.050, 0.150)),
        origin=Origin(xyz=(0.0, -0.245, 0.0)),
        material=charcoal,
        name="center_lifting_lug",
    )
    carriage.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(xyz=(0.0, -0.272, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="load_pin",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.35, lower=0.0, upper=0.650),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("carriage_slide")

    # Each bearing block remains centered around its guide column footprint.
    ctx.expect_within(
        frame,
        carriage,
        axes="xy",
        inner_elem="guide_column_0",
        outer_elem="bearing_block_0",
        margin=0.002,
        name="lower bearing 0 surrounds guide column",
    )
    ctx.expect_within(
        frame,
        carriage,
        axes="xy",
        inner_elem="guide_column_1",
        outer_elem="bearing_block_1",
        margin=0.002,
        name="lower bearing 1 surrounds guide column",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="bearing_block_0",
        elem_b="guide_column_0",
        min_overlap=0.12,
        name="low carriage keeps bearing engaged",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.650}):
        ctx.expect_within(
            frame,
            carriage,
            axes="xy",
            inner_elem="guide_column_0",
            outer_elem="bearing_block_2",
            margin=0.002,
            name="raised bearing remains coaxial",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="bearing_block_2",
            elem_b="guide_column_0",
            min_overlap=0.12,
            name="raised carriage keeps bearing engaged",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates upward along columns",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.60,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
