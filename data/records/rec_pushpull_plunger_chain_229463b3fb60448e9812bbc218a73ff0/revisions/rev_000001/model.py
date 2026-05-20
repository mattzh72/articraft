from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_x(radius: float, length: float, *, center_x: float = 0.0) -> cq.Workplane:
    """CadQuery cylinder whose axis is the assembly X axis."""
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center_x - length / 2.0, 0.0, 0.0))
    )


def _cylinder_y(
    radius: float,
    length: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> cq.Workplane:
    """CadQuery cylinder whose axis is the assembly Y axis."""
    return (
        cq.Workplane("XZ")
        .center(center_x, center_z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, center_y - length / 2.0, 0.0))
    )


def _guide_body_geometry() -> cq.Workplane:
    body_len = 0.180
    body_outer_r = 0.022
    collar_outer_r = 0.029
    collar_len = 0.028
    bore_r = 0.011

    main_tube = _cylinder_x(body_outer_r, body_len)
    rear_collar = _cylinder_x(
        collar_outer_r,
        collar_len,
        center_x=-body_len / 2.0 + collar_len / 2.0,
    )
    front_collar = _cylinder_x(
        collar_outer_r,
        collar_len,
        center_x=body_len / 2.0 - collar_len / 2.0,
    )
    mounting_foot = (
        cq.Workplane("XY")
        .box(0.150, 0.078, 0.012)
        .translate((0.0, 0.0, -0.027))
    )

    guide = main_tube.union(rear_collar).union(front_collar).union(mounting_foot)

    bore_cutter = _cylinder_x(bore_r, body_len + 0.070)
    guide = guide.cut(bore_cutter)

    for x in (-0.055, 0.055):
        for y in (-0.026, 0.026):
            screw_hole = (
                cq.Workplane("XY")
                .center(x, y)
                .circle(0.0045)
                .extrude(0.040)
                .translate((0.0, 0.0, -0.045))
            )
            guide = guide.cut(screw_hole)

    return guide


def _bushing_geometry(center_x: float) -> cq.Workplane:
    ring = _cylinder_x(0.017, 0.007, center_x=center_x)
    bore = _cylinder_x(0.0104, 0.012, center_x=center_x)
    return ring.cut(bore)


def _front_clevis_geometry() -> cq.Workplane:
    cheek_len = 0.070
    cheek_thickness = 0.008
    cheek_gap = 0.018
    cheek_height = 0.034
    cheek_center_x = 0.180
    cheek_offset_y = cheek_gap / 2.0 + cheek_thickness / 2.0

    cheek_a = (
        cq.Workplane("XY")
        .box(cheek_len, cheek_thickness, cheek_height)
        .translate((cheek_center_x, cheek_offset_y, 0.0))
    )
    cheek_b = (
        cq.Workplane("XY")
        .box(cheek_len, cheek_thickness, cheek_height)
        .translate((cheek_center_x, -cheek_offset_y, 0.0))
    )
    rear_bridge = (
        cq.Workplane("XY")
        .box(0.018, cheek_gap + 2.0 * cheek_thickness, cheek_height)
        .translate((0.145, 0.0, 0.0))
    )
    threaded_socket = _cylinder_x(0.0105, 0.032, center_x=0.126)

    clevis = cheek_a.union(cheek_b).union(rear_bridge).union(threaded_socket)
    pin_hole = _cylinder_y(0.0052, 0.055, center_x=0.185)
    clevis = clevis.cut(pin_hole)
    return clevis

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_pull_plunger")

    blued_steel = model.material("blued_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.72, 1.0))
    brass = model.material("brass_bushing", rgba=(0.86, 0.63, 0.26, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.015, 0.014, 0.013, 1.0))

    guide = model.part("guide_body")
    guide.visual(
        mesh_from_cadquery(_guide_body_geometry(), "guide_body"),
        material=blued_steel,
        name="guide_body",
    )
    guide.visual(
        mesh_from_cadquery(_bushing_geometry(-0.0935), "rear_bushing"),
        material=brass,
        name="rear_bushing",
    )
    guide.visual(
        mesh_from_cadquery(_bushing_geometry(0.0935), "front_bushing"),
        material=brass,
        name="front_bushing",
    )
    for name, x in (("rear_bearing", -0.070), ("front_bearing", 0.070)):
        guide.visual(
            Box((0.012, 0.007, 0.0035)),
            origin=Origin(xyz=(x, 0.0, -0.00925)),
            material=brass,
            name=name,
        )

    plunger = model.part("plunger")
    rod_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    plunger.visual(
        Cylinder(radius=0.0075, length=0.335),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=rod_axis.rpy),
        material=polished_steel,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.0105, length=0.026),
        origin=Origin(xyz=(-0.132, 0.0, 0.0), rpy=rod_axis.rpy),
        material=polished_steel,
        name="rear_neck",
    )
    plunger.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.032,
                body_style="lobed",
                base_diameter=0.038,
                top_diameter=0.049,
                crown_radius=0.002,
                edge_radius=0.001,
            ),
            "rear_knob",
        ),
        origin=Origin(xyz=(-0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_polymer,
        name="rear_knob",
    )
    plunger.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(0.137, 0.0, 0.0), rpy=rod_axis.rpy),
        material=polished_steel,
        name="front_collar",
    )
    plunger.visual(
        mesh_from_cadquery(_front_clevis_geometry(), "front_clevis"),
        material=polished_steel,
        name="front_clevis",
    )

    model.articulation(
        "guide_to_plunger",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.040),
        motion_properties=MotionProperties(damping=4.0, friction=0.8),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_body")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("guide_to_plunger")

    ctx.check(
        "single prismatic plunger joint",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type was {slide.articulation_type}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            plunger,
            guide,
            axes="yz",
            inner_elem="rod",
            outer_elem="guide_body",
            margin=0.0,
            name="rod is radially inside guide body",
        )
        ctx.expect_overlap(
            plunger,
            guide,
            axes="x",
            elem_a="rod",
            elem_b="guide_body",
            min_overlap=0.165,
            name="retracted rod remains supported through guide",
        )
        ctx.expect_gap(
            guide,
            plunger,
            axis="x",
            positive_elem="rear_bushing",
            negative_elem="rear_knob",
            min_gap=0.030,
            name="pull knob clears rear bushing when retracted",
        )
        ctx.expect_gap(
            plunger,
            guide,
            axis="x",
            positive_elem="front_clevis",
            negative_elem="front_bushing",
            min_gap=0.010,
            name="front clevis is outside guide nose",
        )
        rest_pos = ctx.part_world_position(plunger)

    with ctx.pose({slide: 0.040}):
        ctx.expect_within(
            plunger,
            guide,
            axes="yz",
            inner_elem="rod",
            outer_elem="guide_body",
            margin=0.0,
            name="extended rod remains radially centered in guide",
        )
        ctx.expect_overlap(
            plunger,
            guide,
            axes="x",
            elem_a="rod",
            elem_b="guide_body",
            min_overlap=0.150,
            name="extended rod retains insertion in guide",
        )
        ctx.expect_gap(
            guide,
            plunger,
            axis="x",
            positive_elem="rear_bushing",
            negative_elem="rear_knob",
            min_gap=0.002,
            name="pushed knob stops before rear bushing",
        )
        extended_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive stroke pushes plunger forward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.035,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
