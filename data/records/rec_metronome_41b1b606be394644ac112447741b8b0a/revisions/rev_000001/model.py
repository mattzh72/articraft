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
    SpurGear,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _sliding_weight_shape() -> cq.Workplane:
    """A trapezoidal metronome bob with a vertical clearance hole for the rod."""
    profile = [(-0.022, -0.023), (0.022, -0.023), (0.015, 0.023), (-0.015, 0.023)]
    body = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(0.020)
        .translate((0.0, 0.010, 0.0))
    )
    rod_clearance = cq.Workplane("XY").circle(0.0048).extrude(0.070, both=True)
    return body.cut(rod_clearance)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_metronome")

    wood = model.material("dark_walnut", rgba=(0.28, 0.14, 0.055, 1.0))
    brass = model.material("brushed_brass", rgba=(0.95, 0.70, 0.28, 1.0))
    black = model.material("black_ink_marks", rgba=(0.02, 0.018, 0.014, 1.0))
    glass = model.material("slightly_smoked_glass", rgba=(0.72, 0.90, 1.0, 0.30))

    housing = model.part("housing")
    # Flat plinth and a shallow, hollow upright case; the open front and clear
    # door make the escapement and pendulum readable.
    housing.visual(Box((0.280, 0.160, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0125)), material=wood, name="base_plinth")
    housing.visual(Box((0.012, 0.090, 0.300)), origin=Origin(xyz=(-0.084, 0.0, 0.175)), material=wood, name="case_side_0")
    housing.visual(Box((0.012, 0.090, 0.300)), origin=Origin(xyz=(0.084, 0.0, 0.175)), material=wood, name="case_side_1")
    housing.visual(Box((0.180, 0.090, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.031)), material=wood, name="bottom_sill")
    housing.visual(Box((0.180, 0.090, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.319)), material=wood, name="top_cap")
    housing.visual(Box((0.180, 0.008, 0.300)), origin=Origin(xyz=(0.0, 0.041, 0.175)), material=wood, name="rear_board")
    housing.visual(Box((0.150, 0.004, 0.006)), origin=Origin(xyz=(0.0, -0.047, 0.314)), material=brass, name="front_lintel")
    housing.visual(Box((0.020, 0.004, 0.265)), origin=Origin(xyz=(-0.074, -0.047, 0.175)), material=brass, name="front_scale_0")
    housing.visual(Box((0.020, 0.004, 0.265)), origin=Origin(xyz=(0.074, -0.047, 0.175)), material=brass, name="front_scale_1")
    for i, z in enumerate((0.105, 0.140, 0.175, 0.210, 0.245, 0.280)):
        housing.visual(Box((0.010, 0.003, 0.0015)), origin=Origin(xyz=(-0.074, -0.050, z)), material=black, name=f"tempo_tick_0_{i}")
        housing.visual(Box((0.010, 0.003, 0.0015)), origin=Origin(xyz=(0.074, -0.050, z)), material=black, name=f"tempo_tick_1_{i}")

    # Brass bottom hinge leaves and the fixed side knuckles for the transparent front panel.
    housing.visual(Box((0.038, 0.024, 0.004)), origin=Origin(xyz=(-0.060, -0.054, 0.038)), material=brass, name="hinge_leaf_0")
    housing.visual(Box((0.038, 0.024, 0.004)), origin=Origin(xyz=(0.060, -0.054, 0.038)), material=brass, name="hinge_leaf_1")
    housing.visual(Box((0.070, 0.024, 0.004)), origin=Origin(xyz=(0.0, -0.054, 0.0315)), material=brass, name="hinge_saddle")
    housing.visual(Cylinder(radius=0.0045, length=0.038), origin=Origin(xyz=(-0.060, -0.063, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="hinge_knuckle_0")
    housing.visual(Cylinder(radius=0.0045, length=0.038), origin=Origin(xyz=(0.060, -0.063, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="hinge_knuckle_1")

    # Visible escapement: a toothed wheel with a hubbed axle tied back into the rear board.
    gear_shape = SpurGear(0.0022, 24, 0.008).build()
    housing.visual(mesh_from_cadquery(gear_shape, "escapement_gear"), origin=Origin(xyz=(-0.032, -0.021, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="escapement_gear")
    housing.visual(Cylinder(radius=0.004, length=0.070), origin=Origin(xyz=(-0.032, 0.010, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="gear_axle")
    housing.visual(Cylinder(radius=0.004, length=0.080), origin=Origin(xyz=(0.0, 0.002, 0.285), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="upper_pivot_shaft")
    housing.visual(Box((0.050, 0.004, 0.005)), origin=Origin(xyz=(-0.006, -0.021, 0.238), rpy=(0.0, -0.30, 0.0)), material=brass, name="escapement_anchor")

    # Side bushing for the continuous winding key.
    housing.visual(Cylinder(radius=0.010, length=0.016), origin=Origin(xyz=(0.094, 0.012, 0.225), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="key_bushing")

    front_panel = model.part("front_panel")
    front_panel.visual(Box((0.008, 0.006, 0.270)), origin=Origin(xyz=(-0.080, 0.0, 0.148)), material=wood, name="panel_side_0")
    front_panel.visual(Box((0.008, 0.006, 0.270)), origin=Origin(xyz=(0.080, 0.0, 0.148)), material=wood, name="panel_side_1")
    front_panel.visual(Box((0.168, 0.006, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.011)), material=wood, name="panel_bottom_rail")
    front_panel.visual(Box((0.168, 0.006, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.282)), material=wood, name="panel_top_rail")
    front_panel.visual(Box((0.156, 0.002, 0.242)), origin=Origin(xyz=(0.0, -0.001, 0.146)), material=glass, name="glass_pane")
    front_panel.visual(Box((0.070, 0.006, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=brass, name="panel_hinge_leaf")
    front_panel.visual(Cylinder(radius=0.0045, length=0.070), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="panel_hinge_barrel")

    pendulum = model.part("pendulum")
    pendulum.visual(Cylinder(radius=0.006, length=0.014), origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name="pivot_hub")
    pendulum.visual(Cylinder(radius=0.0022, length=0.225), origin=Origin(xyz=(0.0, -0.013, -0.1125)), material=brass, name="rod")
    pendulum.visual(Sphere(radius=0.005), origin=Origin(xyz=(0.0, -0.013, -0.225)), material=brass, name="lower_pin")

    weight = model.part("sliding_weight")
    weight.visual(mesh_from_cadquery(_sliding_weight_shape(), "sliding_weight"), material=brass, name="weight_body")
    weight.visual(Box((0.030, 0.002, 0.002)), origin=Origin(xyz=(0.0, -0.011, 0.015)), material=black, name="index_line")
    weight.visual(Box((0.0038, 0.0040, 0.010)), origin=Origin(xyz=(0.0041, 0.0, 0.0)), material=brass, name="rod_clamp_pad")

    side_key = model.part("side_key")
    side_key.visual(Cylinder(radius=0.004, length=0.040), origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="shaft")
    side_key.visual(Cylinder(radius=0.010, length=0.010), origin=Origin(xyz=(0.036, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=brass, name="round_hub")
    side_key.visual(Box((0.006, 0.045, 0.018)), origin=Origin(xyz=(0.043, 0.0, 0.0)), material=brass, name="wing")
    side_key.visual(Sphere(radius=0.006), origin=Origin(xyz=(0.043, -0.024, 0.0)), material=brass, name="wing_knob_0")
    side_key.visual(Sphere(radius=0.006), origin=Origin(xyz=(0.043, 0.024, 0.0)), material=brass, name="wing_knob_1")

    model.articulation(
        "front_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=front_panel,
        origin=Origin(xyz=(0.0, -0.063, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "pendulum_pivot",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, -0.032, 0.285)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=3.5, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "weight_slide",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=weight,
        origin=Origin(xyz=(0.0, -0.013, -0.105)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.18, lower=-0.050, upper=0.060),
    )
    model.articulation(
        "side_key_turn",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=side_key,
        origin=Origin(xyz=(0.102, 0.012, 0.225)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=8.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    panel_joint = object_model.get_articulation("front_panel_hinge")
    pendulum_joint = object_model.get_articulation("pendulum_pivot")
    weight_joint = object_model.get_articulation("weight_slide")
    key_joint = object_model.get_articulation("side_key_turn")

    ctx.check("front panel has a bottom revolute hinge", panel_joint.articulation_type == ArticulationType.REVOLUTE and tuple(panel_joint.axis) == (1.0, 0.0, 0.0))
    ctx.check("pendulum pivots on an upper shaft", pendulum_joint.articulation_type == ArticulationType.REVOLUTE and tuple(pendulum_joint.axis) == (0.0, 1.0, 0.0))
    ctx.check("wedge weight slides on the rod", weight_joint.articulation_type == ArticulationType.PRISMATIC and tuple(weight_joint.axis) == (0.0, 0.0, -1.0))
    ctx.check("side winding key turns continuously", key_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(key_joint.axis) == (1.0, 0.0, 0.0))

    housing = object_model.get_part("housing")
    panel = object_model.get_part("front_panel")
    pendulum = object_model.get_part("pendulum")
    weight = object_model.get_part("sliding_weight")
    key = object_model.get_part("side_key")

    ctx.expect_contact(pendulum, housing, elem_a="pivot_hub", elem_b="upper_pivot_shaft", contact_tol=0.001, name="pendulum hub is seated on the upper shaft")
    ctx.expect_contact(key, housing, elem_a="shaft", elem_b="key_bushing", contact_tol=0.001, name="winding key shaft meets side bushing")
    ctx.expect_within(pendulum, weight, axes="xy", inner_elem="rod", outer_elem="weight_body", margin=0.004, name="sliding weight surrounds the pendulum rod")
    ctx.expect_overlap(weight, pendulum, axes="z", elem_a="weight_body", elem_b="rod", min_overlap=0.035, name="weight remains engaged along the rod")

    with ctx.pose({panel_joint: 0.0}):
        closed_panel_aabb = ctx.part_world_aabb(panel)
    with ctx.pose({panel_joint: 1.20}):
        open_panel_aabb = ctx.part_world_aabb(panel)
    ctx.check(
        "front panel opens forward and downward",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][1] < closed_panel_aabb[0][1] - 0.040
        and open_panel_aabb[1][2] < closed_panel_aabb[1][2] - 0.040,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    with ctx.pose({pendulum_joint: 0.0}):
        vertical_aabb = ctx.part_world_aabb(pendulum)
    with ctx.pose({pendulum_joint: 0.28}):
        swung_aabb = ctx.part_world_aabb(pendulum)
    ctx.check(
        "pendulum swing changes the rod side position",
        vertical_aabb is not None
        and swung_aabb is not None
        and ((swung_aabb[0][0] + swung_aabb[1][0]) * 0.5) < ((vertical_aabb[0][0] + vertical_aabb[1][0]) * 0.5) - 0.020,
        details=f"vertical={vertical_aabb}, swung={swung_aabb}",
    )

    with ctx.pose({weight_joint: -0.040}):
        high_weight_pos = ctx.part_world_position(weight)
    with ctx.pose({weight_joint: 0.050}):
        low_weight_pos = ctx.part_world_position(weight)
    ctx.check(
        "prismatic weight travel follows the rod",
        high_weight_pos is not None and low_weight_pos is not None and low_weight_pos[2] < high_weight_pos[2] - 0.080,
        details=f"high={high_weight_pos}, low={low_weight_pos}",
    )

    return ctx.report()


object_model = build_object_model()
