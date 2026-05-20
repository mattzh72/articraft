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


def _tapered_round_pole(height: float, bottom_radius: float, top_radius: float) -> cq.Workplane:
    """A vertical conical frustum with its bottom face on local z=0."""
    return (
        cq.Workplane("XY")
        .circle(bottom_radius)
        .workplane(offset=height)
        .circle(top_radius)
        .loft(combine=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="single_arm_roadway_floodlight")

    concrete = model.material("cast_concrete", rgba=(0.55, 0.55, 0.52, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.65, 0.65, 1.0))
    dark_metal = model.material("dark_powdercoat", rgba=(0.05, 0.055, 0.055, 1.0))
    glass = model.material("warm_prismatic_glass", rgba=(0.95, 0.86, 0.48, 0.65))
    rubber = model.material("black_gasket", rgba=(0.01, 0.01, 0.01, 1.0))

    support = model.part("support")
    support.visual(
        Box((0.90, 0.90, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="concrete_base",
    )
    support.visual(
        Box((0.38, 0.38, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.270)),
        material=galvanized,
        name="base_plate",
    )
    for ix, x in enumerate((-0.145, 0.145)):
        for iy, y in enumerate((-0.145, 0.145)):
            support.visual(
                Cylinder(radius=0.022, length=0.060),
                origin=Origin(xyz=(x, y, 0.315)),
                material=galvanized,
                name=f"anchor_bolt_{ix}_{iy}",
            )

    support.visual(
        mesh_from_cadquery(_tapered_round_pole(5.40, 0.090, 0.055), "tapered_pole"),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=galvanized,
        name="tapered_pole",
    )
    support.visual(
        Cylinder(radius=0.075, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 5.675)),
        material=galvanized,
        name="top_collar",
    )
    support.visual(
        Cylinder(radius=0.040, length=1.36),
        origin=Origin(xyz=(0.650, 0.0, 5.675), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="outreach_arm",
    )
    support.visual(
        Box((0.170, 0.170, 0.120)),
        origin=Origin(xyz=(1.345, 0.0, 5.675)),
        material=galvanized,
        name="arm_end_block",
    )
    support.visual(
        Box((0.165, 0.420, 0.045)),
        origin=Origin(xyz=(1.445, 0.0, 5.625)),
        material=galvanized,
        name="yoke_crossbar",
    )
    support.visual(
        Box((0.120, 0.020, 0.180)),
        origin=Origin(xyz=(1.445, -0.190, 5.550)),
        material=galvanized,
        name="yoke_plate_0",
    )
    support.visual(
        Box((0.120, 0.020, 0.180)),
        origin=Origin(xyz=(1.445, 0.190, 5.550)),
        material=galvanized,
        name="yoke_plate_1",
    )

    floodlight = model.part("floodlight")
    floodlight.visual(
        Box((0.540, 0.300, 0.180)),
        origin=Origin(xyz=(0.320, 0.0, -0.040)),
        material=dark_metal,
        name="housing_shell",
    )
    floodlight.visual(
        Box((0.026, 0.330, 0.210)),
        origin=Origin(xyz=(0.590, 0.0, -0.040)),
        material=rubber,
        name="front_bezel",
    )
    floodlight.visual(
        Box((0.018, 0.250, 0.135)),
        origin=Origin(xyz=(0.606, 0.0, -0.040)),
        material=glass,
        name="front_lens",
    )
    for y in (-0.075, 0.0, 0.075):
        floodlight.visual(
            Box((0.006, 0.018, 0.115)),
            origin=Origin(xyz=(0.618, y, -0.040)),
            material=galvanized,
            name=f"lens_prism_{y:+.3f}",
        )
    for i, x in enumerate((0.180, 0.260, 0.340, 0.420, 0.500)):
        floodlight.visual(
            Box((0.020, 0.275, 0.060)),
            origin=Origin(xyz=(x, 0.0, 0.080)),
            material=dark_metal,
            name=f"cooling_fin_{i}",
        )
    floodlight.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, -0.165, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_lug_0",
    )
    floodlight.visual(
        Cylinder(radius=0.055, length=0.030),
        origin=Origin(xyz=(0.0, 0.165, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_lug_1",
    )

    model.articulation(
        "bracket_to_floodlight",
        ArticulationType.REVOLUTE,
        parent=support,
        child=floodlight,
        origin=Origin(xyz=(1.445, 0.0, 5.550)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.8, lower=-0.35, upper=0.75),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    support = object_model.get_part("support")
    floodlight = object_model.get_part("floodlight")
    hinge = object_model.get_articulation("bracket_to_floodlight")

    ctx.expect_contact(
        floodlight,
        support,
        elem_a="pivot_lug_0",
        elem_b="yoke_plate_0",
        contact_tol=0.001,
        name="lower side pivot lug seats in fixed yoke",
    )
    ctx.expect_contact(
        floodlight,
        support,
        elem_a="pivot_lug_1",
        elem_b="yoke_plate_1",
        contact_tol=0.001,
        name="upper side pivot lug seats in fixed yoke",
    )
    ctx.expect_gap(
        floodlight,
        support,
        axis="z",
        positive_elem="housing_shell",
        negative_elem="concrete_base",
        min_gap=4.0,
        name="lamp head is elevated well above the concrete base",
    )

    rest_aabb = ctx.part_element_world_aabb(floodlight, elem="front_lens")
    with ctx.pose({hinge: 0.65}):
        aimed_aabb = ctx.part_element_world_aabb(floodlight, elem="front_lens")
    rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) / 2.0 if rest_aabb else None
    aimed_z = (aimed_aabb[0][2] + aimed_aabb[1][2]) / 2.0 if aimed_aabb else None
    ctx.check(
        "positive hinge angle aims floodlight downward",
        rest_z is not None and aimed_z is not None and aimed_z < rest_z - 0.20,
        details=f"rest_lens_z={rest_z}, aimed_lens_z={aimed_z}",
    )

    return ctx.report()


object_model = build_object_model()
