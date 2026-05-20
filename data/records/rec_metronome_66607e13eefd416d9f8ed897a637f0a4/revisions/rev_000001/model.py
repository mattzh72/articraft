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


def _tapered_body_mesh():
    """Truncated rectangular-pyramid metronome case, authored in meters."""
    body = (
        cq.Workplane("XY")
        .rect(0.140, 0.095)
        .workplane(offset=0.235)
        .rect(0.055, 0.045)
        .loft(ruled=True)
    )
    return mesh_from_cadquery(body, "tapered_wood_housing", tolerance=0.0008)


def _tempo_weight_mesh():
    """A hollow cylindrical slider weight with a real bore for the pendulum rod."""
    weight = (
        cq.Workplane("XY")
        .circle(0.022)
        .circle(0.0048)
        .extrude(0.032, both=True)
    )
    return mesh_from_cadquery(weight, "hollow_tempo_weight", tolerance=0.0005)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="traditional_mechanical_metronome")

    wood = model.material("dark_varnished_wood", rgba=(0.28, 0.12, 0.045, 1.0))
    black = model.material("black_felt_foot", rgba=(0.01, 0.009, 0.007, 1.0))
    brass = model.material("aged_brass", rgba=(0.82, 0.60, 0.24, 1.0))
    steel = model.material("brushed_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    ink = model.material("black_scale_ink", rgba=(0.02, 0.018, 0.012, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.180, 0.120, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=wood,
        name="rectangular_base",
    )
    housing.visual(
        _tapered_body_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=wood,
        name="tapered_body",
    )
    housing.visual(
        Box((0.070, 0.044, 0.010)),
        origin=Origin(xyz=(0.0, -0.004, 0.263)),
        material=wood,
        name="top_cap",
    )
    housing.visual(
        Box((0.024, 0.026, 0.165)),
        origin=Origin(xyz=(0.0, -0.035, 0.150)),
        material=brass,
        name="front_scale_plate",
    )
    for i, z in enumerate((0.088, 0.112, 0.136, 0.160, 0.184, 0.208)):
        housing.visual(
            Box((0.017 if i % 2 else 0.022, 0.004, 0.0025)),
            origin=Origin(xyz=(0.0, -0.049, z)),
            material=ink,
            name=f"scale_tick_{i}",
        )
    housing.visual(
        Cylinder(radius=0.010, length=0.062),
        origin=Origin(xyz=(0.0, -0.026, 0.250), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_shaft",
    )
    housing.visual(
        Box((0.190, 0.130, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=black,
        name="base_pad",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0030, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=brass,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0115, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.018, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.002, 0.012)),
        material=brass,
        name="rod_clamp",
    )

    tempo_weight = model.part("tempo_weight")
    tempo_weight.visual(
        _tempo_weight_mesh(),
        origin=Origin(),
        material=steel,
        name="sliding_collar",
    )
    tempo_weight.visual(
        Cylinder(radius=0.0027, length=0.026),
        origin=Origin(xyz=(0.0155, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=ink,
        name="set_screw",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0045, length=0.041),
        origin=Origin(xyz=(0.0, 0.0145, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_shaft",
    )
    winding_key.visual(
        Cylinder(radius=0.010, length=0.012),
        origin=Origin(xyz=(0.0, 0.037, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="center_boss",
    )
    winding_key.visual(
        Box((0.034, 0.008, 0.020)),
        origin=Origin(xyz=(-0.023, 0.041, 0.0)),
        material=brass,
        name="key_wing_0",
    )
    winding_key.visual(
        Box((0.034, 0.008, 0.020)),
        origin=Origin(xyz=(0.023, 0.041, 0.0)),
        material=brass,
        name="key_wing_1",
    )
    winding_key.visual(
        Cylinder(radius=0.005, length=0.008),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="end_button",
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.048, 0.250)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=tempo_weight,
        origin=Origin(xyz=(0.0, 0.0, 0.074)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.15, lower=0.0, upper=0.115),
    )
    model.articulation(
        "housing_to_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.0, 0.039, 0.162)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    tempo_weight = object_model.get_part("tempo_weight")
    winding_key = object_model.get_part("winding_key")
    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    weight_slide = object_model.get_articulation("pendulum_to_weight")
    key_joint = object_model.get_articulation("housing_to_key")

    ctx.allow_overlap(
        housing,
        pendulum,
        elem_a="pivot_shaft",
        elem_b="pivot_hub",
        reason="The visible pendulum hub is captured around the fixed pivot shaft at the top bearing.",
    )
    ctx.allow_overlap(
        housing,
        pendulum,
        elem_a="pivot_shaft",
        elem_b="rod_clamp",
        reason="The pendulum clamp is intentionally fastened around the top pivot shaft.",
    )
    ctx.allow_overlap(
        housing,
        winding_key,
        elem_a="tapered_body",
        elem_b="key_shaft",
        reason="The winding-key shaft is intentionally seated a few millimeters into the rear housing.",
    )
    ctx.allow_overlap(
        pendulum,
        tempo_weight,
        elem_a="rod",
        elem_b="set_screw",
        reason="The tiny set screw bears into the rod to represent the tempo-weight friction clamp.",
    )

    ctx.expect_contact(
        housing,
        pendulum,
        elem_a="pivot_shaft",
        elem_b="pivot_hub",
        contact_tol=0.012,
        name="pendulum hub is at pivot shaft",
    )
    ctx.expect_contact(
        housing,
        pendulum,
        elem_a="pivot_shaft",
        elem_b="rod_clamp",
        contact_tol=0.006,
        name="pendulum clamp bears on pivot shaft",
    )
    ctx.expect_gap(
        winding_key,
        housing,
        axis="y",
        positive_elem="key_shaft",
        negative_elem="tapered_body",
        max_penetration=0.016,
        name="winding shaft is only shallowly seated",
    )
    ctx.expect_within(
        tempo_weight,
        pendulum,
        axes="xy",
        inner_elem="sliding_collar",
        outer_elem="rod",
        margin=0.025,
        name="tempo weight remains centered on rod",
    )
    ctx.expect_overlap(
        tempo_weight,
        pendulum,
        axes="z",
        elem_a="sliding_collar",
        elem_b="rod",
        min_overlap=0.025,
        name="tempo weight collar surrounds rod length",
    )

    rest_pos = ctx.part_world_position(tempo_weight)
    with ctx.pose({weight_slide: 0.100}):
        raised_pos = ctx.part_world_position(tempo_weight)
        ctx.expect_overlap(
            tempo_weight,
            pendulum,
            axes="z",
            elem_a="sliding_collar",
            elem_b="rod",
            min_overlap=0.020,
            name="raised tempo weight still retained on rod",
        )

    ctx.check(
        "tempo weight slides upward along pendulum rod",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.080,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    rod_rest_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    with ctx.pose({pendulum_joint: 0.30}):
        rod_swung_aabb = ctx.part_element_world_aabb(pendulum, elem="rod")
    rest_rod_x = None if rod_rest_aabb is None else (rod_rest_aabb[0][0] + rod_rest_aabb[1][0]) / 2.0
    swung_rod_x = None if rod_swung_aabb is None else (rod_swung_aabb[0][0] + rod_swung_aabb[1][0]) / 2.0
    ctx.check(
        "pendulum joint is a front-view swing",
        rest_rod_x is not None and swung_rod_x is not None and abs(swung_rod_x - rest_rod_x) > 0.015,
        details=f"rest_x={rest_rod_x}, swung_x={swung_rod_x}",
    )

    with ctx.pose({key_joint: math.pi / 2.0}):
        ctx.expect_contact(
            winding_key,
            housing,
            elem_a="key_shaft",
            elem_b="tapered_body",
            contact_tol=0.010,
            name="rotating key remains on rear shaft",
        )

    return ctx.report()


object_model = build_object_model()
