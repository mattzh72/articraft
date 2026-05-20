from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_pendulum_metronome")

    wood = model.material("dark_walnut", rgba=(0.24, 0.12, 0.055, 1.0))
    endgrain = model.material("darker_walnut_edges", rgba=(0.14, 0.07, 0.035, 1.0))
    brass = model.material("aged_brass", rgba=(0.82, 0.58, 0.20, 1.0))
    steel = model.material("polished_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    black = model.material("black_enamel", rgba=(0.015, 0.013, 0.012, 1.0))
    ivory = model.material("ivory_scale", rgba=(0.92, 0.86, 0.70, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.34, 0.22, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=wood,
        name="flat_base",
    )
    housing.visual(
        Box((0.37, 0.24, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=endgrain,
        name="base_foot",
    )
    housing.visual(
        Box((0.020, 0.130, 0.580)),
        origin=Origin(xyz=(-0.090, 0.0, 0.335)),
        material=wood,
        name="side_wall_0",
    )
    housing.visual(
        Box((0.020, 0.130, 0.580)),
        origin=Origin(xyz=(0.090, 0.0, 0.335)),
        material=wood,
        name="side_wall_1",
    )
    housing.visual(
        Box((0.200, 0.018, 0.580)),
        origin=Origin(xyz=(0.0, 0.056, 0.335)),
        material=wood,
        name="rear_wall",
    )
    housing.visual(
        Box((0.180, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, -0.055, 0.080)),
        material=endgrain,
        name="front_sill",
    )
    housing.visual(
        Box((0.033, 0.132, 0.045)),
        origin=Origin(xyz=(-0.083, 0.0, 0.630)),
        material=endgrain,
        name="top_side_0",
    )
    housing.visual(
        Box((0.033, 0.132, 0.045)),
        origin=Origin(xyz=(0.083, 0.0, 0.630)),
        material=endgrain,
        name="top_side_1",
    )
    housing.visual(
        Box((0.200, 0.037, 0.045)),
        origin=Origin(xyz=(0.0, 0.047, 0.630)),
        material=endgrain,
        name="top_rear_bridge",
    )
    housing.visual(
        Box((0.068, 0.004, 0.420)),
        origin=Origin(xyz=(0.0, 0.046, 0.355)),
        material=ivory,
        name="tempo_scale",
    )
    for index, z in enumerate([0.175, 0.215, 0.255, 0.295, 0.335, 0.375, 0.415, 0.455, 0.495, 0.535]):
        tick_width = 0.050 if index % 3 == 0 else 0.034
        housing.visual(
            Box((tick_width, 0.003, 0.004)),
            origin=Origin(xyz=(0.0, 0.043, z)),
            material=black,
            name=f"scale_tick_{index}",
        )
    housing.visual(
        Cylinder(radius=0.012, length=0.124),
        origin=Origin(xyz=(0.0, -0.014, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_shaft",
    )
    housing.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.070, 0.145), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="front_pivot_boss",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0035, length=0.784),
        origin=Origin(xyz=(0.0, 0.0, 0.408)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Cylinder(radius=0.0035, length=0.049),
        origin=Origin(xyz=(0.0, 0.0, -0.0405)),
        material=steel,
        name="lower_drive_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.020, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    pendulum.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=brass,
        name="top_finial",
    )

    weight = model.part("weight")
    weight.visual(
        Cylinder(radius=0.038, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="weight_body",
    )
    weight.visual(
        Cylinder(radius=0.006, length=0.030),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="thumb_screw",
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.008, length=0.046),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="key_stem",
    )
    winding_key.visual(
        Box((0.014, 0.086, 0.033)),
        origin=Origin(xyz=(0.056, 0.0, 0.0)),
        material=brass,
        name="key_wing",
    )
    winding_key.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="key_collar",
    )

    model.articulation(
        "pendulum_swing",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.035, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.5, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, 0.0, 0.640)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.18, lower=-0.250, upper=0.080),
    )
    model.articulation(
        "key_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=winding_key,
        origin=Origin(xyz=(0.100, 0.000, 0.315)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    winding_key = object_model.get_part("winding_key")
    swing = object_model.get_articulation("pendulum_swing")
    slide = object_model.get_articulation("weight_slide")
    key_spin = object_model.get_articulation("key_spin")

    ctx.allow_overlap(
        housing,
        pendulum,
        elem_a="pivot_shaft",
        elem_b="pivot_hub",
        reason="The pendulum hub is intentionally captured around the internal pivot shaft.",
    )
    ctx.expect_within(
        housing,
        pendulum,
        axes="xz",
        inner_elem="pivot_shaft",
        outer_elem="pivot_hub",
        margin=0.001,
        name="pivot shaft sits inside hub eye",
    )
    ctx.expect_overlap(
        housing,
        pendulum,
        axes="xz",
        elem_a="pivot_shaft",
        elem_b="pivot_hub",
        min_overlap=0.020,
        name="hub and pivot shaft are coaxial",
    )

    ctx.allow_overlap(
        pendulum,
        weight,
        elem_a="rod",
        elem_b="weight_body",
        reason="The adjustable weight is represented as a sliding collar around the pendulum rod.",
    )
    ctx.expect_within(
        pendulum,
        weight,
        axes="xy",
        inner_elem="rod",
        outer_elem="weight_body",
        margin=0.002,
        name="rod remains centered through weight collar",
    )
    ctx.expect_overlap(
        pendulum,
        weight,
        axes="z",
        elem_a="rod",
        elem_b="weight_body",
        min_overlap=0.060,
        name="weight collar surrounds a vertical length of rod",
    )

    ctx.expect_gap(
        pendulum,
        housing,
        axis="z",
        positive_elem="top_finial",
        negative_elem="top_rear_bridge",
        min_gap=0.20,
        name="pendulum extends well above housing top",
    )
    ctx.expect_gap(
        weight,
        housing,
        axis="z",
        positive_elem="weight_body",
        negative_elem="top_rear_bridge",
        min_gap=0.06,
        name="resting weight is near the exposed upper rod",
    )
    ctx.expect_gap(
        winding_key,
        housing,
        axis="x",
        positive_elem="key_stem",
        negative_elem="side_wall_1",
        min_gap=0.0,
        max_gap=0.002,
        name="winding key stem seats on side wall",
    )

    rest_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({slide: -0.200}):
        low_weight_pos = ctx.part_world_position(weight)
        ctx.expect_within(
            weight,
            pendulum,
            axes="xy",
            inner_elem="weight_body",
            outer_elem="rod",
            margin=0.050,
            name="lowered weight stays on rod axis",
        )
    with ctx.pose({slide: 0.080}):
        high_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "weight slides along rod",
        rest_weight_pos is not None
        and low_weight_pos is not None
        and high_weight_pos is not None
        and low_weight_pos[2] < rest_weight_pos[2] - 0.19
        and high_weight_pos[2] > rest_weight_pos[2] + 0.07,
        details=f"rest={rest_weight_pos}, low={low_weight_pos}, high={high_weight_pos}",
    )

    rest_top = ctx.part_element_world_aabb(pendulum, elem="top_finial")
    with ctx.pose({swing: 0.160}):
        right_top = ctx.part_element_world_aabb(pendulum, elem="top_finial")
    with ctx.pose({swing: -0.160}):
        left_top = ctx.part_element_world_aabb(pendulum, elem="top_finial")

    def _center_x(aabb):
        if aabb is None:
            return None
        return 0.5 * (aabb[0][0] + aabb[1][0])

    rest_x = _center_x(rest_top)
    right_x = _center_x(right_top)
    left_x = _center_x(left_top)
    ctx.check(
        "pendulum swings side to side",
        rest_x is not None
        and right_x is not None
        and left_x is not None
        and right_x > rest_x + 0.10
        and left_x < rest_x - 0.10,
        details=f"rest_x={rest_x}, right_x={right_x}, left_x={left_x}",
    )
    ctx.check(
        "winding key is continuous rotary control",
        key_spin.articulation_type == ArticulationType.CONTINUOUS and key_spin.axis == (1.0, 0.0, 0.0),
        details=f"type={key_spin.articulation_type}, axis={key_spin.axis}",
    )

    return ctx.report()


object_model = build_object_model()
