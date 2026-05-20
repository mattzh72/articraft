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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _sliding_jaw_shape() -> cq.Workplane:
    """One-piece front jaw with guide-rod bores and raised grip serrations."""
    carriage = cq.Workplane("XY").box(0.088, 0.180, 0.075)
    upright = cq.Workplane("XY").box(0.034, 0.165, 0.120).translate((-0.035, 0.0, 0.055))
    jaw = carriage.union(upright)

    # Raised horizontal grip bars on the rear-facing jaw surface.
    for z in (0.020, 0.040, 0.060, 0.080, 0.100):
        tooth = cq.Workplane("XY").box(0.007, 0.142, 0.006).translate((-0.055, 0.0, z))
        jaw = jaw.union(tooth)

    # Two clearance bores let the static guide rods pass through the carriage.
    for y in (-0.055, 0.055):
        cutter = cq.Workplane("YZ").center(y, -0.025).circle(0.0145).extrude(0.160, both=True)
        jaw = jaw.cut(cutter)

    return jaw


def _threaded_boss_shape() -> cq.Workplane:
    """Annular lower screw boss with an actual center clearance hole."""
    return cq.Workplane("XY").circle(0.034).circle(0.015).extrude(0.026).translate((0.0, 0.0, -0.013))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clamp_on_welding_vise")

    red = model.material("red_enamel", rgba=(0.64, 0.07, 0.035, 1.0))
    dark = model.material("dark_cast_iron", rgba=(0.08, 0.09, 0.10, 1.0))
    steel = model.material("brushed_steel", rgba=(0.68, 0.66, 0.60, 1.0))
    black = model.material("blackened_thread", rgba=(0.025, 0.024, 0.022, 1.0))
    wood = model.material("workbench_wood", rgba=(0.55, 0.34, 0.17, 1.0))

    body = model.part("body")

    # Contextual bench edge, visibly gripped by the clamp-on base.
    body.visual(
        Box((0.340, 0.300, 0.050)),
        origin=Origin(xyz=(0.060, 0.0, 0.035)),
        material=wood,
        name="bench_edge",
    )

    # Parallel C-clamp frame: upper shoe on top of the bench, lower arm below it,
    # and a rear spine around the bench edge.
    body.visual(
        Box((0.340, 0.170, 0.025)),
        origin=Origin(xyz=(0.020, 0.0, 0.0725)),
        material=red,
        name="upper_clamp_shoe",
    )
    body.visual(
        Box((0.060, 0.160, 0.040)),
        origin=Origin(xyz=(-0.060, 0.0, 0.105)),
        material=red,
        name="shoe_to_bed_rib",
    )
    body.visual(
        Box((0.430, 0.175, 0.035)),
        origin=Origin(xyz=(0.015, 0.0, 0.1275)),
        material=red,
        name="vise_bed",
    )
    body.visual(
        Box((0.048, 0.180, 0.210)),
        origin=Origin(xyz=(-0.170, 0.0, 0.035)),
        material=red,
        name="rear_spine",
    )
    body.visual(
        Box((0.245, 0.130, 0.030)),
        origin=Origin(xyz=(-0.055, 0.0, -0.055)),
        material=red,
        name="lower_clamp_arm",
    )
    for i, y in enumerate((-0.045, 0.045)):
        body.visual(
            Box((0.070, 0.025, 0.026)),
            origin=Origin(xyz=(0.085, y, -0.055)),
            material=red,
            name=f"boss_yoke_{i}",
        )
    body.visual(
        mesh_from_cadquery(_threaded_boss_shape(), "threaded_boss"),
        origin=Origin(xyz=(0.110, 0.0, -0.055)),
        material=dark,
        name="threaded_boss",
    )

    # Fixed rear jaw and hard serrated insert.
    body.visual(
        Box((0.070, 0.178, 0.125)),
        origin=Origin(xyz=(-0.145, 0.0, 0.2075)),
        material=red,
        name="rear_jaw_block",
    )
    body.visual(
        Box((0.010, 0.152, 0.092)),
        origin=Origin(xyz=(-0.105, 0.0, 0.232)),
        material=steel,
        name="rear_jaw_face",
    )
    for i, z in enumerate((0.198, 0.216, 0.234, 0.252, 0.270)):
        body.visual(
            Box((0.007, 0.135, 0.006)),
            origin=Origin(xyz=(-0.098, 0.0, z)),
            material=black,
            name=f"rear_tooth_{i}",
        )

    # Static parallel guide rods for the sliding front jaw.
    for i, y in enumerate((-0.055, 0.055)):
        body.visual(
            Cylinder(radius=0.0085, length=0.345),
            origin=Origin(xyz=(0.055, y, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"guide_rod_{i}",
        )
        body.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(0.232, y, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"rod_cap_{i}",
        )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        mesh_from_cadquery(_sliding_jaw_shape(), "front_jaw_body", tolerance=0.0008),
        material=steel,
        name="jaw_body",
    )
    for i, y in enumerate((-0.055, 0.055)):
        front_jaw.visual(
            Box((0.072, 0.006, 0.007)),
            origin=Origin(xyz=(0.0, y, -0.013)),
            material=steel,
            name=f"bearing_pad_{i}",
        )

    clamp_screw = model.part("clamp_screw")
    clamp_screw.visual(
        Cylinder(radius=0.040, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=steel,
        name="pressure_pad",
    )
    clamp_screw.visual(
        Cylinder(radius=0.0105, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.041)),
        material=black,
        name="threaded_stem",
    )
    for i, z in enumerate((-0.074, -0.058, -0.042, -0.026, -0.010)):
        clamp_screw.visual(
            Cylinder(radius=0.0130, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=steel,
            name=f"thread_crest_{i}",
        )
    clamp_screw.visual(
        Cylinder(radius=0.0075, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, -0.086), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tommy_bar",
    )
    for i, y in enumerate((-0.058, 0.058)):
        clamp_screw.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.0, y, -0.086)),
            material=steel,
            name=f"bar_end_{i}",
        )

    model.articulation(
        "body_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_jaw,
        origin=Origin(xyz=(0.025, 0.0, 0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.110),
    )
    model.articulation(
        "body_to_clamp_screw",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clamp_screw,
        origin=Origin(xyz=(0.110, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_jaw = object_model.get_part("front_jaw")
    clamp_screw = object_model.get_part("clamp_screw")
    slide = object_model.get_articulation("body_to_front_jaw")
    screw = object_model.get_articulation("body_to_clamp_screw")

    ctx.check(
        "front jaw uses prismatic guide motion",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type is {slide.articulation_type}",
    )
    ctx.check(
        "clamp screw uses a revolute tightening joint",
        screw.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type is {screw.articulation_type}",
    )

    ctx.expect_gap(
        front_jaw,
        body,
        axis="x",
        positive_elem="jaw_body",
        negative_elem="rear_jaw_face",
        min_gap=0.040,
        max_gap=0.100,
        name="vise jaws start with a usable work gap",
    )
    for rod in ("guide_rod_0", "guide_rod_1"):
        ctx.expect_overlap(
            front_jaw,
            body,
            axes="x",
            elem_a="jaw_body",
            elem_b=rod,
            min_overlap=0.070,
            name=f"front jaw is retained on {rod} at rest",
        )

    ctx.expect_gap(
        body,
        clamp_screw,
        axis="z",
        positive_elem="bench_edge",
        negative_elem="pressure_pad",
        max_gap=0.0015,
        max_penetration=0.0,
        name="screw pressure pad seats against the bench underside",
    )
    ctx.expect_overlap(
        clamp_screw,
        body,
        axes="xy",
        elem_a="pressure_pad",
        elem_b="bench_edge",
        min_overlap=0.040,
        name="pressure pad footprint lies under the bench",
    )

    rest_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({slide: 0.110}):
        extended_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            front_jaw,
            body,
            axis="x",
            positive_elem="jaw_body",
            negative_elem="rear_jaw_face",
            min_gap=0.145,
            name="front jaw opens away from the fixed rear jaw",
        )
        for rod in ("guide_rod_0", "guide_rod_1"):
            ctx.expect_overlap(
                front_jaw,
                body,
                axes="x",
                elem_a="jaw_body",
                elem_b=rod,
                min_overlap=0.060,
                name=f"front jaw remains on {rod} when extended",
            )

    ctx.check(
        "positive slide travel moves the front jaw forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.095,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({screw: math.pi / 2.0}):
        ctx.expect_gap(
            body,
            clamp_screw,
            axis="z",
            positive_elem="bench_edge",
            negative_elem="pressure_pad",
            max_gap=0.0015,
            max_penetration=0.0,
            name="rotating screw keeps its clamp pad seated",
        )

    return ctx.report()


object_model = build_object_model()
