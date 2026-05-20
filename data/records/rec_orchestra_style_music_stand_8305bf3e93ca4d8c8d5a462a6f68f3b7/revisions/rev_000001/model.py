from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)
import cadquery as cq


def _carriage_geometry() -> cq.Workplane:
    """Sliding twin-collar carriage with a compact head for the tilt hinge."""

    post_spacing = 0.110
    outer_r = 0.036
    inner_r = 0.016
    collar_h = 0.130

    carriage = None
    for x in (-post_spacing / 2.0, post_spacing / 2.0):
        collar = (
            cq.Workplane("XY")
            .circle(outer_r)
            .circle(inner_r)
            .extrude(collar_h)
            .translate((x, 0.0, -collar_h / 2.0))
        )
        carriage = collar if carriage is None else carriage.union(collar)

    rear_bridge = cq.Workplane("XY").box(0.165, 0.026, 0.075).translate((0.0, 0.040, 0.006))
    center_neck = cq.Workplane("XY").box(0.060, 0.115, 0.055).translate((0.0, -0.020, 0.038))
    hinge_crossbar = cq.Workplane("XY").box(0.450, 0.030, 0.035).translate((0.0, -0.045, 0.045))
    cheek_a = cq.Workplane("XY").box(0.035, 0.055, 0.095).translate((-0.220, -0.085, 0.065))
    cheek_b = cq.Workplane("XY").box(0.035, 0.055, 0.095).translate((0.220, -0.085, 0.065))

    return carriage.union(rear_bridge).union(center_neck).union(hinge_crossbar).union(cheek_a).union(cheek_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conductors_music_stand")

    black = model.material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_metal = model.material("black_powder_coated_metal", rgba=(0.03, 0.032, 0.035, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    worn_edge = model.material("worn_black_edge_highlight", rgba=(0.10, 0.10, 0.09, 1.0))

    base = model.part("floor_base")
    base.visual(
        Cylinder(radius=0.310, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black,
        name="weighted_disk",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=dark_metal,
        name="center_boss",
    )
    base.visual(
        Box((0.160, 0.070, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_metal,
        name="post_socket",
    )
    for i, x in enumerate((-0.055, 0.055)):
        base.visual(
            Cylinder(radius=0.016, length=1.220),
            origin=Origin(xyz=(x, 0.0, 0.700)),
            material=dark_metal,
            name=f"post_{i}",
        )
    base.visual(
        Box((0.165, 0.055, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 1.325)),
        material=dark_metal,
        name="top_tie",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=rubber,
        name="floor_pad",
    )

    carriage = model.part("height_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_geometry(), "height_carriage_frame", tolerance=0.0007),
        material=dark_metal,
        name="carriage_frame",
    )
    carriage.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.055, 0.047, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="thread_boss",
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.800, 0.020, 0.480)),
        origin=Origin(xyz=(0.0, -0.130, 0.260), rpy=(-0.20, 0.0, 0.0)),
        material=black,
        name="broad_desk",
    )
    desk.visual(
        Box((0.830, 0.070, 0.035)),
        origin=Origin(xyz=(0.0, -0.185, 0.032)),
        material=black,
        name="music_lip",
    )
    desk.visual(
        Box((0.810, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, -0.080, 0.505), rpy=(-0.20, 0.0, 0.0)),
        material=worn_edge,
        name="top_rolled_edge",
    )
    for i, x in enumerate((-0.365, 0.365)):
        desk.visual(
            Box((0.026, 0.018, 0.455)),
            origin=Origin(xyz=(x, -0.130, 0.260), rpy=(-0.20, 0.0, 0.0)),
            material=worn_edge,
            name=f"side_rail_{i}",
        )
    desk.visual(
        Cylinder(radius=0.014, length=0.320),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="tilt_tube",
    )
    desk.visual(
        Box((0.360, 0.190, 0.018)),
        origin=Origin(xyz=(0.0, -0.095, 0.026)),
        material=dark_metal,
        name="hinge_backplate",
    )
    for i, x in enumerate((-0.120, 0.0, 0.120)):
        desk.visual(
            Box((0.035, 0.014, 0.056)),
            origin=Origin(xyz=(x, -0.004, 0.026)),
            material=dark_metal,
            name=f"hinge_leaf_{i}",
        )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.006, length=0.080),
        origin=Origin(xyz=(0.0, 0.040, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="clamp_screw",
    )
    clamp_mesh = mesh_from_geometry(
        KnobGeometry(
            0.064,
            0.028,
            body_style="lobed",
            base_diameter=0.042,
            top_diameter=0.058,
            crown_radius=0.002,
            grip=KnobGrip(style="ribbed", count=8, depth=0.0020, width=0.0040),
            bore=KnobBore(style="round", diameter=0.010),
        ),
        "lobed_clamp_knob",
    )
    clamp_knob.visual(
        clamp_mesh,
        origin=Origin(xyz=(0.0, 0.094, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="hand_knob",
    )

    model.articulation(
        "base_to_height_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.360),
    )
    model.articulation(
        "carriage_to_desk",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=desk,
        origin=Origin(xyz=(0.0, -0.085, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.40, upper=0.55),
    )
    model.articulation(
        "carriage_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=clamp_knob,
        origin=Origin(xyz=(0.055, 0.058, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("floor_base")
    carriage = object_model.get_part("height_carriage")
    desk = object_model.get_part("desk")
    clamp_knob = object_model.get_part("clamp_knob")

    height_slide = object_model.get_articulation("base_to_height_carriage")
    tilt_hinge = object_model.get_articulation("carriage_to_desk")
    knob_spin = object_model.get_articulation("carriage_to_clamp_knob")

    ctx.allow_overlap(
        base,
        carriage,
        elem_a="post_0",
        elem_b="carriage_frame",
        reason="The first mast post intentionally runs through a close-fitting sliding collar.",
    )
    ctx.allow_overlap(
        base,
        carriage,
        elem_a="post_1",
        elem_b="carriage_frame",
        reason="The second mast post intentionally runs through the matched sliding collar.",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="z",
        min_overlap=0.120,
        elem_a="carriage_frame",
        elem_b="post_0",
        name="left collar remains wrapped around the mast",
    )
    ctx.expect_overlap(
        carriage,
        base,
        axes="z",
        min_overlap=0.120,
        elem_a="carriage_frame",
        elem_b="post_1",
        name="right collar remains wrapped around the mast",
    )
    ctx.expect_contact(
        clamp_knob,
        carriage,
        elem_a="clamp_screw",
        elem_b="thread_boss",
        contact_tol=0.001,
        name="clamp screw bears on the collar boss",
    )
    ctx.allow_overlap(
        desk,
        carriage,
        elem_a="tilt_tube",
        elem_b="carriage_frame",
        reason="The desk tilt tube is intentionally captured through the carriage hinge head.",
    )
    ctx.expect_overlap(
        desk,
        carriage,
        axes="x",
        min_overlap=0.250,
        elem_a="tilt_tube",
        elem_b="carriage_frame",
        name="tilt tube is retained across the carriage hinge head",
    )

    post_0_aabb = ctx.part_element_world_aabb(base, elem="post_0")
    post_1_aabb = ctx.part_element_world_aabb(base, elem="post_1")
    ctx.check(
        "twin posts are matched supports",
        post_0_aabb is not None
        and post_1_aabb is not None
        and abs((post_0_aabb[1][0] - post_0_aabb[0][0]) - (post_1_aabb[1][0] - post_1_aabb[0][0])) < 0.002
        and abs(post_0_aabb[0][2] - post_1_aabb[0][2]) < 0.002
        and abs(post_0_aabb[1][2] - post_1_aabb[1][2]) < 0.002,
        details=f"post_0={post_0_aabb}, post_1={post_1_aabb}",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({height_slide: 0.360}):
        raised_carriage = ctx.part_world_position(carriage)
        ctx.expect_overlap(
            carriage,
            base,
            axes="z",
            min_overlap=0.120,
            elem_a="carriage_frame",
            elem_b="post_0",
            name="raised collar retains post insertion",
        )
    ctx.check(
        "height carriage slides upward on the twin-post mast",
        rest_carriage is not None
        and raised_carriage is not None
        and raised_carriage[2] > rest_carriage[2] + 0.300,
        details=f"rest={rest_carriage}, raised={raised_carriage}",
    )

    rest_desk_aabb = ctx.part_world_aabb(desk)
    with ctx.pose({tilt_hinge: 0.45}):
        tilted_desk_aabb = ctx.part_world_aabb(desk)
    ctx.check(
        "desk changes angle about the horizontal hinge",
        rest_desk_aabb is not None
        and tilted_desk_aabb is not None
        and tilted_desk_aabb[0][1] < rest_desk_aabb[0][1] - 0.060,
        details=f"rest={rest_desk_aabb}, tilted={tilted_desk_aabb}",
    )

    rest_knob = ctx.part_world_position(clamp_knob)
    with ctx.pose({knob_spin: math.pi}):
        spun_knob = ctx.part_world_position(clamp_knob)
    ctx.check(
        "clamp knob spins on its threaded axis",
        rest_knob is not None
        and spun_knob is not None
        and abs(rest_knob[0] - spun_knob[0]) < 1e-6
        and abs(rest_knob[1] - spun_knob[1]) < 1e-6
        and abs(rest_knob[2] - spun_knob[2]) < 1e-6,
        details=f"rest={rest_knob}, spun={spun_knob}",
    )

    return ctx.report()


object_model = build_object_model()
