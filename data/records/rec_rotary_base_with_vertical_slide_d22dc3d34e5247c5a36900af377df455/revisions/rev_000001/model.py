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


def _triangular_gusset_mesh(name: str):
    """A right-triangle web plate extruded through X, with its base on z=0."""
    thickness = 0.030
    depth = 0.220
    height = 0.220
    x0, x1 = -thickness / 2.0, thickness / 2.0
    y0, y1 = -depth / 2.0, depth / 2.0

    vertices = [
        (x0, y0, 0.0),
        (x0, y1, 0.0),
        (x0, y1, height),
        (x1, y0, 0.0),
        (x1, y1, 0.0),
        (x1, y1, height),
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
    return mesh_from_geometry(MeshGeometry(vertices=vertices, faces=faces), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_mast_module")

    cast_iron = model.material("cast_iron", rgba=(0.12, 0.13, 0.14, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.07, 0.075, 0.08, 1.0))
    machined = model.material("machined_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    anodized = model.material("black_anodized", rgba=(0.025, 0.027, 0.030, 1.0))
    carriage_red = model.material("safety_red", rgba=(0.74, 0.08, 0.045, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.009, 1.0))

    stator = model.part("stator")
    stator.visual(
        Cylinder(radius=0.430, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_cast,
        name="floor_plinth",
    )
    stator.visual(
        Cylinder(radius=0.390, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=machined,
        name="stator_race",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.360, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=cast_iron,
        name="rotating_base",
    )
    turntable.visual(
        Cylinder(radius=0.315, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.154)),
        material=dark_cast,
        name="top_flange",
    )
    turntable.visual(
        Cylinder(radius=0.145, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.188)),
        material=machined,
        name="center_hub",
    )

    bolt_radius = 0.016
    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        turntable.visual(
            Cylinder(radius=bolt_radius, length=0.020),
            origin=Origin(
                xyz=(0.285 * math.cos(angle), 0.285 * math.sin(angle), 0.180)
            ),
            material=machined,
            name=f"flange_bolt_{index}",
        )

    turntable.visual(
        Box((0.300, 0.260, 0.095)),
        origin=Origin(xyz=(0.0, 0.055, 0.180)),
        material=cast_iron,
        name="column_foot",
    )
    turntable.visual(
        Box((0.180, 0.150, 0.860)),
        origin=Origin(xyz=(0.0, 0.055, 0.570)),
        material=anodized,
        name="column_body",
    )
    turntable.visual(
        _triangular_gusset_mesh("gusset_mesh"),
        origin=Origin(xyz=(-0.105, 0.045, 0.140)),
        material=cast_iron,
        name="gusset_0",
    )
    turntable.visual(
        _triangular_gusset_mesh("gusset_mesh_mirror"),
        origin=Origin(xyz=(0.105, 0.045, 0.140)),
        material=cast_iron,
        name="gusset_1",
    )

    for index, x in enumerate((-0.072, 0.072)):
        turntable.visual(
            Box((0.024, 0.038, 0.650)),
            origin=Origin(xyz=(x, -0.037, 0.535)),
            material=machined,
            name=f"guide_track_{index}",
        )
        turntable.visual(
            Box((0.034, 0.010, 0.610)),
            origin=Origin(xyz=(x, -0.050, 0.535)),
            material=anodized,
            name=f"track_wiper_{index}",
        )

    turntable.visual(
        Box((0.082, 0.006, 0.620)),
        origin=Origin(xyz=(0.0, -0.023, 0.535)),
        material=dark_cast,
        name="recessed_way",
    )
    turntable.visual(
        Box((0.190, 0.034, 0.036)),
        origin=Origin(xyz=(0.0, -0.046, 0.240)),
        material=rubber,
        name="bottom_stop",
    )
    turntable.visual(
        Box((0.190, 0.034, 0.036)),
        origin=Origin(xyz=(0.0, -0.046, 0.745)),
        material=rubber,
        name="upper_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.220, 0.052, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_red,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.198, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.035, 0.058)),
        material=cast_iron,
        name="upper_rib",
    )
    carriage.visual(
        Box((0.198, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, -0.035, -0.058)),
        material=cast_iron,
        name="lower_rib",
    )
    for index, x in enumerate((-0.086, 0.086)):
        carriage.visual(
            Box((0.018, 0.018, 0.145)),
            origin=Origin(xyz=(x, -0.035, 0.0)),
            material=cast_iron,
            name=f"side_rib_{index}",
        )
    carriage.visual(
        Cylinder(radius=0.030, length=0.022),
        origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="tooling_boss",
    )
    for index, (x, z) in enumerate(
        ((-0.073, 0.058), (0.073, 0.058), (-0.073, -0.058), (0.073, -0.058))
    ):
        carriage.visual(
            Cylinder(radius=0.014, length=0.014),
            origin=Origin(xyz=(x, -0.039, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=machined,
            name=f"carriage_bolt_{index}",
        )

    model.articulation(
        "stator_to_turntable",
        ArticulationType.REVOLUTE,
        parent=stator,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "turntable_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.082, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.250),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stator = object_model.get_part("stator")
    turntable = object_model.get_part("turntable")
    carriage = object_model.get_part("carriage")
    rotary = object_model.get_articulation("stator_to_turntable")
    slide = object_model.get_articulation("turntable_to_carriage")

    ctx.expect_contact(
        stator,
        turntable,
        elem_a="stator_race",
        elem_b="rotating_base",
        name="rotating base sits on bearing race",
    )
    ctx.expect_gap(
        turntable,
        carriage,
        axis="y",
        positive_elem="guide_track_0",
        negative_elem="carriage_plate",
        min_gap=0.0,
        max_gap=0.001,
        name="carriage back face runs on guide track",
    )
    ctx.expect_gap(
        carriage,
        turntable,
        axis="z",
        positive_elem="carriage_plate",
        negative_elem="bottom_stop",
        min_gap=0.010,
        name="lower stop remains below carriage at rest",
    )
    ctx.expect_overlap(
        carriage,
        turntable,
        axes="z",
        elem_a="carriage_plate",
        elem_b="guide_track_0",
        min_overlap=0.150,
        name="carriage stays engaged on vertical track at rest",
    )

    with ctx.pose({slide: 0.250}):
        ctx.expect_gap(
            turntable,
            carriage,
            axis="z",
            positive_elem="upper_stop",
            negative_elem="carriage_plate",
            min_gap=0.020,
            name="upper stop remains clear at full travel",
        )
        ctx.expect_overlap(
            carriage,
            turntable,
            axes="z",
            elem_a="carriage_plate",
            elem_b="guide_track_0",
            min_overlap=0.150,
            name="carriage remains retained on the track at full travel",
        )

    with ctx.pose({rotary: 1.1, slide: 0.180}):
        ctx.expect_contact(
            stator,
            turntable,
            elem_a="stator_race",
            elem_b="rotating_base",
            name="bearing support remains seated while rotated",
        )
        ctx.expect_gap(
            turntable,
            carriage,
            axis="z",
            positive_elem="upper_stop",
            negative_elem="carriage_plate",
            min_gap=0.050,
            name="combined rotary and slide pose clears upper stop",
        )

    return ctx.report()


object_model = build_object_model()
