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


def _sliding_weight_mesh():
    """A tapered metronome pendulum weight with a clear vertical rod hole."""

    depth = 0.020
    height = 0.046
    bottom_width = 0.040
    top_width = 0.026
    profile = [
        (-bottom_width / 2.0, -height / 2.0),
        (bottom_width / 2.0, -height / 2.0),
        (top_width / 2.0, height / 2.0),
        (-top_width / 2.0, height / 2.0),
    ]
    wedge = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(depth)
        .translate((0.0, depth / 2.0, 0.0))
    )
    rod_clearance = (
        cq.Workplane("XY")
        .circle(0.006)
        .extrude(height + 0.020)
        .translate((0.0, 0.0, -(height + 0.020) / 2.0))
    )
    return wedge.cut(rod_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_style_mechanical_metronome")

    wood = model.material("warm_wood", rgba=(0.46, 0.25, 0.12, 1.0))
    dark_wood = model.material("dark_front", rgba=(0.16, 0.09, 0.045, 1.0))
    brass = model.material("brass", rgba=(0.86, 0.62, 0.23, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    steel = model.material("polished_steel", rgba=(0.70, 0.70, 0.66, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.220, 0.130, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=wood,
        name="base_plinth",
    )
    housing.visual(
        Box((0.180, 0.100, 0.332)),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=wood,
        name="upright_box",
    )
    housing.visual(
        Box((0.192, 0.112, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.367)),
        material=wood,
        name="top_cap",
    )
    housing.visual(
        Box((0.156, 0.012, 0.284)),
        origin=Origin(xyz=(0.0, -0.0530, 0.190)),
        material=dark_wood,
        name="flat_front_face",
    )
    housing.visual(
        Box((0.034, 0.004, 0.184)),
        origin=Origin(xyz=(-0.045, -0.0565, 0.218)),
        material=brass,
        name="tempo_scale",
    )
    for i, z in enumerate((0.144, 0.168, 0.192, 0.216, 0.240, 0.264, 0.288)):
        tick_width = 0.022 if i % 2 == 0 else 0.015
        housing.visual(
            Box((tick_width, 0.0025, 0.0022)),
            origin=Origin(xyz=(-0.045, -0.0592, z)),
            material=black,
            name=f"scale_tick_{i}",
        )
    housing.visual(
        Box((0.050, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.0585, 0.235)),
        material=brass,
        name="pivot_plate",
    )
    housing.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, -0.064, 0.235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_socket",
    )
    housing.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.094, 0.0, 0.182), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="side_collar",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0027, length=0.225),
        origin=Origin(xyz=(0.0, 0.0, 0.1125)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.010, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_hub",
    )
    pendulum.visual(
        Box((0.006, 0.004, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=steel,
        name="lower_pointer",
    )

    weight = model.part("weight")
    weight.visual(
        mesh_from_cadquery(_sliding_weight_mesh(), "sliding_wedge_weight"),
        origin=Origin(),
        material=brass,
        name="wedge",
    )
    for i, x in enumerate((-0.00435, 0.00435)):
        weight.visual(
            Box((0.0033, 0.0040, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=brass,
            name=f"rod_pad_{i}",
        )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="stem",
    )
    winding_key.visual(
        Cylinder(radius=0.022, length=0.016),
        origin=Origin(xyz=(0.034, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="round_key",
    )
    winding_key.visual(
        Box((0.006, 0.050, 0.012)),
        origin=Origin(xyz=(0.043, 0.0, 0.0)),
        material=brass,
        name="grip_bar",
    )

    model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.077, 0.235)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0, lower=-0.34, upper=0.34),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.15, lower=0.0, upper=0.105),
    )
    model.articulation(
        "key_rotation",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.101, 0.0, 0.182)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    winding_key = object_model.get_part("winding_key")
    pivot = object_model.get_articulation("pendulum_pivot")
    slide = object_model.get_articulation("weight_slide")
    key = object_model.get_articulation("key_rotation")

    ctx.check(
        "pendulum has bounded revolute swing",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and pivot.motion_limits is not None
        and pivot.motion_limits.lower < 0.0
        and pivot.motion_limits.upper > 0.0,
    )
    ctx.check(
        "weight slides upward on rod",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and slide.motion_limits is not None
        and slide.motion_limits.upper >= 0.10,
    )
    ctx.check(
        "winding key is continuous about side axis",
        key.articulation_type == ArticulationType.CONTINUOUS
        and tuple(key.axis) == (1.0, 0.0, 0.0),
    )

    ctx.expect_gap(
        housing,
        pendulum,
        axis="y",
        min_gap=0.0,
        max_gap=0.006,
        positive_elem="pivot_socket",
        negative_elem="pivot_hub",
        name="pendulum hub clears front pivot socket",
    )
    ctx.expect_gap(
        winding_key,
        housing,
        axis="x",
        min_gap=0.0,
        max_gap=0.004,
        positive_elem="stem",
        negative_elem="side_collar",
        name="winding key stem seats at side collar",
    )
    ctx.expect_overlap(
        winding_key,
        housing,
        axes="yz",
        min_overlap=0.010,
        elem_a="stem",
        elem_b="side_collar",
        name="winding key is coaxial with side collar",
    )
    ctx.expect_overlap(
        weight,
        pendulum,
        axes="z",
        min_overlap=0.040,
        elem_a="wedge",
        elem_b="rod",
        name="low weight surrounds the pendulum rod",
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({slide: 0.105}):
        ctx.expect_overlap(
            weight,
            pendulum,
            axes="z",
            min_overlap=0.040,
            elem_a="wedge",
            elem_b="rod",
            name="raised weight remains on rod",
        )
        raised_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "weight prismatic joint moves the wedge upward",
        rest_weight_pos is not None
        and raised_weight_pos is not None
        and raised_weight_pos[2] > rest_weight_pos[2] + 0.095,
        details=f"rest={rest_weight_pos}, raised={raised_weight_pos}",
    )

    with ctx.pose({pivot: 0.30}):
        swung_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "pendulum revolute joint swings the rod sideways",
        rest_weight_pos is not None
        and swung_weight_pos is not None
        and swung_weight_pos[0] > rest_weight_pos[0] + 0.018,
        details=f"rest={rest_weight_pos}, swung={swung_weight_pos}",
    )

    key_aabb_rest = ctx.part_element_world_aabb(winding_key, elem="grip_bar")
    with ctx.pose({key: math.pi / 2.0}):
        key_aabb_turn = ctx.part_element_world_aabb(winding_key, elem="grip_bar")
    if key_aabb_rest is not None and key_aabb_turn is not None:
        rest_dy = key_aabb_rest[1][1] - key_aabb_rest[0][1]
        turn_dz = key_aabb_turn[1][2] - key_aabb_turn[0][2]
    else:
        rest_dy = turn_dz = 0.0
    ctx.check(
        "winding key visibly rotates",
        rest_dy > 0.045 and turn_dz > 0.045,
        details=f"rest_aabb={key_aabb_rest}, turned_aabb={key_aabb_turn}",
    )

    return ctx.report()


object_model = build_object_model()
