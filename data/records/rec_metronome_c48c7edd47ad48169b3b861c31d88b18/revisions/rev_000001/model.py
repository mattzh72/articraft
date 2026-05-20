from __future__ import annotations

import cadquery as cq
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _hollow_cylinder_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
):
    """Centered cylindrical sleeve with a real through-bore."""
    sleeve = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -0.5 * length))
    )
    return mesh_from_cadquery(sleeve, name, tolerance=0.00045, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_metronome")

    dark_base = model.material("black_lacquered_base", rgba=(0.035, 0.030, 0.025, 1.0))
    brass = model.material("brushed_brass", rgba=(0.86, 0.64, 0.28, 1.0))
    steel = model.material("polished_steel", rgba=(0.78, 0.78, 0.74, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    red_tip = model.material("red_tempo_pointer", rgba=(0.82, 0.08, 0.045, 1.0))

    base_x = 0.30
    base_y = 0.16
    base_z = 0.035
    post_x = 0.105
    post_radius = 0.0085
    post_height = 0.485
    post_center_z = base_z + post_height / 2.0
    crossbar_z = base_z + post_height
    pivot_y = -0.045
    pivot_z = crossbar_z

    frame = model.part("frame")
    frame.visual(
        Box((base_x, base_y, base_z)),
        origin=Origin(xyz=(0.0, 0.0, base_z / 2.0)),
        material=dark_base,
        name="base_plate",
    )
    for suffix, x in (("0", -post_x), ("1", post_x)):
        frame.visual(
            Cylinder(radius=post_radius, length=post_height),
            origin=Origin(xyz=(x, 0.0, post_center_z)),
            material=brass,
            name=f"side_post_{suffix}",
        )
        frame.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(x, 0.0, base_z + 0.006)),
            material=brass,
            name=f"post_socket_{suffix}",
        )
    frame.visual(
        Cylinder(radius=0.011, length=2.0 * post_x + 0.035),
        origin=Origin(xyz=(0.0, 0.0, crossbar_z), rpy=(0.0, pi / 2.0, 0.0)),
        material=brass,
        name="top_crossbar",
    )
    frame.visual(
        Cylinder(radius=0.0058, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, pivot_z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )
    frame.visual(
        Cylinder(radius=0.024, length=0.005),
        origin=Origin(xyz=(0.095, -0.052, base_z + 0.0025)),
        material=brass,
        name="knob_washer",
    )
    for suffix, x, y in (
        ("0", -0.115, -0.055),
        ("1", 0.115, -0.055),
        ("2", -0.115, 0.055),
        ("3", 0.115, 0.055),
    ):
        frame.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(x, y, -0.004)),
            material=rubber,
            name=f"rubber_foot_{suffix}",
        )

    pendulum = model.part("pendulum")
    pendulum.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.014,
            inner_radius=0.0058,
            length=0.024,
            name="pendulum_eye_mesh",
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_eye",
    )
    rod_length = 0.425
    rod_radius = 0.004
    pendulum.visual(
        Cylinder(radius=rod_radius, length=rod_length),
        origin=Origin(xyz=(0.0, 0.0, -0.012 - rod_length / 2.0)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.010, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.012 - rod_length - 0.015)),
        material=red_tip,
        name="pointer_tip",
    )

    weight = model.part("weight")
    weight.visual(
        _hollow_cylinder_mesh(
            outer_radius=0.032,
            inner_radius=0.0062,
            length=0.055,
            name="sliding_weight_mesh",
        ),
        origin=Origin(),
        material=brass,
        name="weight_barrel",
    )
    weight.visual(
        Cylinder(radius=0.0045, length=0.058),
        origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="set_screw",
    )

    winding_knob = model.part("winding_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.026,
            body_style="faceted",
            top_diameter=0.036,
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=14, depth=0.0011, width=0.0022),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008),
            center=False,
        ),
        "winding_knob_mesh",
    )
    winding_knob.visual(
        knob_mesh,
        origin=Origin(),
        material=brass,
        name="knob_cap",
    )

    model.articulation(
        "frame_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=pendulum,
        origin=Origin(xyz=(0.0, pivot_y, pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=2.6, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "pendulum_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, -0.185)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.18, lower=0.0, upper=0.205),
    )
    model.articulation(
        "frame_to_winding_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=winding_knob,
        origin=Origin(xyz=(0.095, -0.052, base_z + 0.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    winding_knob = object_model.get_part("winding_knob")
    swing = object_model.get_articulation("frame_to_pendulum")
    slide = object_model.get_articulation("pendulum_to_weight")
    wind = object_model.get_articulation("frame_to_winding_knob")

    ctx.allow_overlap(
        frame,
        pendulum,
        elem_a="pivot_pin",
        elem_b="pivot_eye",
        reason="The exposed pivot pin is intentionally captured inside the pendulum eye bearing.",
    )
    ctx.check(
        "primary mechanisms are articulated",
        swing.articulation_type == ArticulationType.REVOLUTE
        and slide.articulation_type == ArticulationType.PRISMATIC
        and wind.articulation_type == ArticulationType.CONTINUOUS,
        details=f"types={(swing.articulation_type, slide.articulation_type, wind.articulation_type)}",
    )
    ctx.expect_overlap(
        pendulum,
        frame,
        axes="xz",
        elem_a="pivot_eye",
        elem_b="pivot_pin",
        min_overlap=0.010,
        name="exposed eye is carried on the crossbar pivot pin",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        elem_a="weight_barrel",
        elem_b="rod",
        min_overlap=0.050,
        name="sliding cylindrical weight surrounds the pendulum rod",
    )
    ctx.expect_gap(
        winding_knob,
        frame,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="knob_washer",
        max_gap=0.001,
        max_penetration=0.000001,
        name="winding knob is seated on the base washer",
    )

    rest_aabb = ctx.part_world_aabb(pendulum)
    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({swing: 0.32}):
        swung_aabb = ctx.part_world_aabb(pendulum)
    with ctx.pose({slide: 0.19}):
        low_weight_pos = ctx.part_world_position(weight)
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            elem_a="weight_barrel",
            elem_b="rod",
            min_overlap=0.050,
            name="lower tempo setting remains captured on rod",
        )
    ctx.check(
        "pendulum swings laterally about the exposed pin",
        rest_aabb is not None
        and swung_aabb is not None
        and (swung_aabb[0][0] + swung_aabb[1][0]) / 2.0
        < (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        - 0.04,
        details=f"rest_aabb={rest_aabb}, swung_aabb={swung_aabb}",
    )
    ctx.check(
        "sliding weight moves downward along the rod",
        rest_weight_pos is not None
        and low_weight_pos is not None
        and low_weight_pos[2] < rest_weight_pos[2] - 0.15,
        details=f"rest={rest_weight_pos}, lowered={low_weight_pos}",
    )

    return ctx.report()


object_model = build_object_model()
