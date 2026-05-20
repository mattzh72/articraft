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


HOUSING_WIDTH = 0.54
HOUSING_DEPTH = 0.19
HOUSING_HEIGHT = 0.105
SLOT_WIDTH = 0.410
SLOT_HEIGHT = 0.082
SLOT_CENTER_Z = 0.050
PIVOT_Y = -0.075
PIVOT_Z = 0.083
WEIGHT_CENTER_Z = -0.038


def _sliding_weight() -> cq.Workplane:
    """Flat brass slider with a close captured bore through the rod path."""

    block = cq.Workplane("XY").box(0.062, 0.016, 0.030)
    rod_clearance = cq.Workplane("XY").circle(0.0028).extrude(0.040).translate(
        (0.0, 0.0, -0.020)
    )
    # A broad front relief hints at the clamp opening while preserving one
    # connected rectangular weight.
    clamp_relief = cq.Workplane("XY").box(0.018, 0.020, 0.022).translate(
        (0.0, -0.006, 0.0)
    )
    return block.cut(rod_clearance).cut(clamp_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_wide_slot_metronome")

    walnut = model.material("warm_walnut", rgba=(0.34, 0.18, 0.08, 1.0))
    brass = model.material("brushed_brass", rgba=(0.90, 0.66, 0.24, 1.0))
    steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.75, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.500, 0.115, 0.082)),
        origin=Origin(xyz=(0.0, 0.033, 0.051)),
        material=walnut,
        name="housing_shell",
    )
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=walnut,
        name="base_plinth",
    )
    # Split top deck around a shallow rectangular recess so the key disk can sit
    # flush rather than interpenetrating the top surface.
    housing.visual(
        Box((0.354, 0.160, 0.014)),
        origin=Origin(xyz=(-0.0505, 0.0, HOUSING_HEIGHT - 0.007)),
        material=walnut,
        name="top_deck_left",
    )
    housing.visual(
        Box((0.034, 0.160, 0.014)),
        origin=Origin(xyz=(0.2105, 0.0, HOUSING_HEIGHT - 0.007)),
        material=walnut,
        name="top_deck_end",
    )
    housing.visual(
        Box((0.068, 0.052, 0.014)),
        origin=Origin(xyz=(0.160, -0.054, HOUSING_HEIGHT - 0.007)),
        material=walnut,
        name="top_deck_front",
    )
    housing.visual(
        Box((0.068, 0.052, 0.014)),
        origin=Origin(xyz=(0.160, 0.054, HOUSING_HEIGHT - 0.007)),
        material=walnut,
        name="top_deck_rear",
    )
    housing.visual(
        Box((0.044, HOUSING_DEPTH, 0.095)),
        origin=Origin(xyz=(-0.248, 0.0, 0.0575)),
        material=walnut,
        name="side_cheek_0",
    )
    housing.visual(
        Box((0.044, HOUSING_DEPTH, 0.095)),
        origin=Origin(xyz=(0.248, 0.0, 0.0575)),
        material=walnut,
        name="side_cheek_1",
    )
    housing.visual(
        Box((0.010, 0.170, 0.103)),
        origin=Origin(xyz=(-0.251, 0.0, 0.055), rpy=(0.0, 0.22, 0.0)),
        material=walnut,
        name="tapered_side_0",
    )
    housing.visual(
        Box((0.010, 0.170, 0.103)),
        origin=Origin(xyz=(0.251, 0.0, 0.055), rpy=(0.0, -0.22, 0.0)),
        material=walnut,
        name="tapered_side_1",
    )
    # Brass trim is slightly seated into the front face so it reads as mounted,
    # while the central opening remains fully clear for the pendulum.
    housing.visual(
        Box((SLOT_WIDTH + 0.030, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, -0.097, SLOT_CENTER_Z + SLOT_HEIGHT / 2 + 0.004)),
        material=brass,
        name="slot_top_trim",
    )
    housing.visual(
        Box((SLOT_WIDTH + 0.030, 0.010, 0.007)),
        origin=Origin(xyz=(0.0, -0.099, SLOT_CENTER_Z - SLOT_HEIGHT / 2 - 0.004)),
        material=brass,
        name="slot_bottom_trim",
    )
    housing.visual(
        Box((0.008, 0.010, SLOT_HEIGHT + 0.020)),
        origin=Origin(xyz=(-SLOT_WIDTH / 2 - 0.011, -0.098, SLOT_CENTER_Z)),
        material=brass,
        name="slot_end_trim_0",
    )
    housing.visual(
        Box((0.008, 0.010, SLOT_HEIGHT + 0.020)),
        origin=Origin(xyz=(SLOT_WIDTH / 2 + 0.011, -0.098, SLOT_CENTER_Z)),
        material=brass,
        name="slot_end_trim_1",
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0032, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, -0.037)),
        material=steel,
        name="rod_bar",
    )
    pendulum.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=brass,
        name="pivot_cap",
    )

    weight = model.part("weight")
    weight.visual(
        mesh_from_cadquery(_sliding_weight(), "sliding_weight", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, WEIGHT_CENTER_Z)),
        material=brass,
        name="weight_block",
    )

    key = model.part("winding_key")
    # The disk sits in a shallow top recess; the bow is a very low butterfly key
    # rather than a tall knob, matching the requested flush top-face control.
    key.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass,
        name="key_disk",
    )
    key.visual(
        Box((0.110, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brass,
        name="key_wing",
    )
    key.visual(
        Box((0.026, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brass,
        name="key_center_pad",
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=6.0, lower=-0.48, upper=0.48),
    )
    model.articulation(
        "rod_to_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=-0.012, upper=0.020),
    )
    model.articulation(
        "housing_to_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(0.160, 0.0, HOUSING_HEIGHT - 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("weight")
    key = object_model.get_part("winding_key")
    swing = object_model.get_articulation("housing_to_pendulum")
    slide = object_model.get_articulation("rod_to_weight")
    key_turn = object_model.get_articulation("housing_to_key")

    ctx.allow_overlap(
        weight,
        pendulum,
        elem_a="weight_block",
        elem_b="rod_bar",
        reason="The sliding weight is modeled as a close captured collar gripping the pendulum rod.",
    )
    ctx.expect_within(
        pendulum,
        weight,
        axes="xy",
        inner_elem="rod_bar",
        outer_elem="weight_block",
        margin=0.001,
        name="rod passes through the sliding weight collar",
    )
    ctx.expect_overlap(
        pendulum,
        weight,
        axes="z",
        elem_a="rod_bar",
        elem_b="weight_block",
        min_overlap=0.020,
        name="sliding weight remains threaded on the pendulum rod",
    )
    ctx.expect_within(
        weight,
        housing,
        axes="xz",
        inner_elem="weight_block",
        outer_elem="housing_shell",
        margin=0.002,
        name="sliding weight remains inside the front slot silhouette",
    )
    ctx.expect_within(
        pendulum,
        housing,
        axes="xz",
        inner_elem="rod_bar",
        outer_elem="housing_shell",
        margin=0.002,
        name="pendulum rod lies within the wide front opening",
    )

    def element_center(part, elem):
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    with ctx.pose({swing: -0.34}):
        left_center = element_center(weight, "weight_block")
    with ctx.pose({swing: 0.34}):
        right_center = element_center(weight, "weight_block")
    ctx.check(
        "pendulum swings side to side through the slot",
        left_center is not None
        and right_center is not None
        and abs(left_center[0] - right_center[0]) > 0.020,
        details=f"left={left_center}, right={right_center}",
    )

    rest_center = element_center(weight, "weight_block")
    with ctx.pose({slide: slide.motion_limits.upper}):
        lowered_center = element_center(weight, "weight_block")
    ctx.check(
        "sliding weight travels downward along the rod",
        rest_center is not None
        and lowered_center is not None
        and lowered_center[2] < rest_center[2] - 0.015,
        details=f"rest={rest_center}, lowered={lowered_center}",
    )

    wing_at_zero = ctx.part_element_world_aabb(key, elem="key_wing")
    with ctx.pose({key_turn: math.pi / 2}):
        wing_turned = ctx.part_element_world_aabb(key, elem="key_wing")
    ctx.check(
        "winding key turns continuously about the top face axis",
        wing_at_zero is not None
        and wing_turned is not None
        and (wing_at_zero[1][0] - wing_at_zero[0][0]) > 0.09
        and (wing_turned[1][1] - wing_turned[0][1]) > 0.09,
        details=f"zero={wing_at_zero}, turned={wing_turned}",
    )

    return ctx.report()


object_model = build_object_model()
