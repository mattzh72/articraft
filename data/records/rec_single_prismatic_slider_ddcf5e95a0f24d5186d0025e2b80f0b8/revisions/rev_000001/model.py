from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _base_extrusion() -> cq.Workplane:
    """Fixed compact bench rail with an integral raised guide and end stops."""

    base_len = 0.520
    base_w = 0.160
    plate_h = 0.025
    rail_len = 0.480
    rail_w = 0.060
    rail_h = 0.035

    plate = cq.Workplane("XY").box(base_len, base_w, plate_h).translate((0.0, 0.0, plate_h / 2.0))
    rail = cq.Workplane("XY").box(rail_len, rail_w, rail_h).translate(
        (0.0, 0.0, plate_h + rail_h / 2.0)
    )
    rail = rail.edges("|X and >Z").chamfer(0.004)

    stop_h = 0.035
    stop_len = 0.018
    stop_w = 0.072
    base = plate.union(rail)
    for x in (-0.235, 0.235):
        stop = cq.Workplane("XY").box(stop_len, stop_w, stop_h).translate(
            (x, 0.0, plate_h + stop_h / 2.0)
        )
        base = base.union(stop)

    # Four counterbored bench-mounting holes in the base flanges.
    for x in (-0.180, 0.180):
        for y in (-0.055, 0.055):
            through = cq.Workplane("XY").circle(0.0065).extrude(0.045).translate((x, y, -0.005))
            counterbore = cq.Workplane("XY").circle(0.011).extrude(0.007).translate(
                (x, y, plate_h - 0.007)
            )
            base = base.cut(through).cut(counterbore)

    return base


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bench_slider")

    steel = model.material("brushed_steel", color=(0.55, 0.58, 0.58, 1.0))
    dark_steel = model.material("dark_anodized_steel", color=(0.08, 0.09, 0.10, 1.0))
    brass = model.material("oiled_bronze_gib", color=(0.70, 0.50, 0.24, 1.0))
    black = model.material("blackened_fastener", color=(0.01, 0.01, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_extrusion(), "base_extrusion", tolerance=0.0008),
        material=steel,
        name="base_extrusion",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.160, 0.110, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_steel,
        name="carriage_saddle",
    )
    for side, y in enumerate((-0.041, 0.041)):
        carriage.visual(
            Box((0.145, 0.010, 0.033)),
            origin=Origin(xyz=(0.0, y, -0.0125)),
            material=dark_steel,
            name=f"side_guide_{side}",
        )

    # Bronze side guide/gib plates make the carriage read as a guided sliding block.
    for side, y in enumerate((-0.053, 0.053)):
        carriage.visual(
            Box((0.130, 0.004, 0.022)),
            origin=Origin(xyz=(0.0, y, -0.010)),
            material=brass,
            name=f"side_gib_{side}",
        )

    # Flush top mounting holes on the flat carriage face.
    for ix, x in enumerate((-0.045, 0.045)):
        for iy, y in enumerate((-0.026, 0.026)):
            carriage.visual(
                Cylinder(radius=0.006, length=0.0010),
                origin=Origin(xyz=(x, y, 0.0546)),
                material=black,
                name=f"mount_hole_{ix}_{iy}",
            )

    # Small side fastener heads retaining the gib plates.
    for side, y in enumerate((-0.0555, 0.0555)):
        for ix, x in enumerate((-0.045, 0.045)):
            carriage.visual(
                Cylinder(radius=0.0045, length=0.003),
                origin=Origin(xyz=(x, y, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=black,
                name=f"gib_screw_{side}_{ix}",
            )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        # The joint frame lies on the guide axis at the top center of the rail.
        origin=Origin(xyz=(-0.105, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.210),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("base_to_carriage")

    ctx.check(
        "single prismatic carriage joint",
        len(object_model.articulations) == 1 and slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"articulations={object_model.articulations}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.145,
            elem_a="carriage_saddle",
            elem_b="base_extrusion",
            name="carriage is engaged on rail at retracted stop",
        )
        rest_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.210}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.145,
            elem_a="carriage_saddle",
            elem_b="base_extrusion",
            name="carriage remains engaged on rail at extended stop",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along fixed guide axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.19,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
