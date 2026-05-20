from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _build_body_shape() -> cq.Workplane:
    flange_height = 0.006
    body_height = 0.030
    cap_height = 0.010
    spout_base_z = flange_height + body_height + cap_height - 0.002

    flange = cq.Workplane("XY").circle(0.029).extrude(flange_height)
    shank = cq.Workplane("XY").workplane(offset=-0.024).circle(0.0085).extrude(0.024)
    collar = cq.Workplane("XY").workplane(offset=flange_height).circle(0.017).extrude(body_height)
    cap = cq.Workplane("XY").workplane(offset=flange_height + body_height).circle(0.013).extrude(cap_height)

    spout_path = (
        cq.Workplane("XZ")
        .moveTo(0.0, spout_base_z)
        .lineTo(0.0, 0.155)
        .threePointArc((0.026, 0.246), (0.105, 0.230))
        .threePointArc((0.134, 0.218), (0.148, 0.182))
    )
    outer_spout = cq.Workplane("XY").workplane(offset=spout_base_z).circle(0.007).sweep(spout_path)
    inner_spout = cq.Workplane("XY").workplane(offset=spout_base_z).circle(0.0046).sweep(spout_path)
    spout = outer_spout.cut(inner_spout)

    lever_boss = cq.Workplane("XZ").circle(0.0078).extrude(-0.010).translate((0.0, 0.0235, 0.031))
    lever_bridge = cq.Workplane("XY").box(0.012, 0.010, 0.014).translate((0.0, 0.0185, 0.031))
    selector_boss = cq.Workplane("YZ").circle(0.009).extrude(0.010).translate((0.0155, 0.0, 0.022))

    return (
        flange.union(shank)
        .union(collar)
        .union(cap)
        .union(spout)
        .union(lever_boss)
        .union(lever_bridge)
        .union(selector_boss)
        .combine()
    )


def _build_lever_shaft() -> cq.Workplane:
    return cq.Workplane("XZ").circle(0.0036).extrude(-0.006).translate((0.0, 0.007, 0.0))


def _build_lever_handle() -> cq.Workplane:
    hub = cq.Workplane("XZ").circle(0.0068).extrude(-0.004).translate((0.0, 0.007, 0.0))
    neck = cq.Workplane("XY").box(0.015, 0.0052, 0.010).translate((0.010, 0.0055, 0.008))
    arm = cq.Workplane("XY").box(0.040, 0.0052, 0.008).translate((0.031, 0.0055, 0.014))
    paddle = cq.Workplane("XY").box(0.022, 0.0056, 0.011).translate((0.056, 0.0055, 0.019))
    fin = cq.Workplane("XY").box(0.010, 0.0056, 0.005).translate((0.061, 0.0055, 0.026))
    return hub.union(neck).union(arm).union(paddle).union(fin).combine()


def _build_selector_shaft() -> cq.Workplane:
    return cq.Workplane("YZ").circle(0.003).extrude(0.006)


def _build_selector_knob() -> cq.Workplane:
    skirt = cq.Workplane("YZ").circle(0.012).extrude(0.002).translate((0.0045, 0.0, 0.0))
    knob = cq.Workplane("YZ").circle(0.0105).extrude(0.007).translate((0.006, 0.0, 0.0))
    pointer = cq.Workplane("XY").box(0.0025, 0.002, 0.012).translate((0.014, 0.0, 0.013))
    return skirt.union(knob).union(pointer).combine()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="filtered_water_faucet")

    steel = model.material("steel", rgba=(0.78, 0.80, 0.84, 1.0))
    dark_knob = model.material("dark_knob", rgba=(0.16, 0.17, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material=steel,
        name="body_shell",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shaft(), "lever_shaft"),
        material=steel,
        name="lever_shaft",
    )
    lever.visual(
        mesh_from_cadquery(_build_lever_handle(), "lever_handle"),
        material=steel,
        name="lever_handle",
    )

    selector = model.part("selector")
    selector.visual(
        mesh_from_cadquery(_build_selector_shaft(), "selector_shaft"),
        material=dark_knob,
        name="selector_shaft",
    )
    selector.visual(
        mesh_from_cadquery(_build_selector_knob(), "selector_knob"),
        material=dark_knob,
        name="selector_knob",
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0225, 0.031)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.0, effort=6.0, velocity=2.5),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(0.0255, 0.0, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    selector = object_model.get_part("selector")
    lever_joint = object_model.get_articulation("body_to_lever")
    selector_joint = object_model.get_articulation("body_to_selector")

    ctx.allow_overlap(
        body,
        lever,
        elem_a="body_shell",
        elem_b="lever_shaft",
        reason="The side lever is represented with a short pivot stub nesting into the side boss.",
    )
    ctx.allow_overlap(
        body,
        lever,
        elem_a="body_shell",
        elem_b="lever_handle",
        reason="The lever hub is simplified as a close-fitting collar around the side pivot boss.",
    )
    ctx.allow_overlap(
        body,
        selector,
        elem_a="body_shell",
        elem_b="selector_shaft",
        reason="The filter selector rotates on a short shaft represented as nesting into the base collar boss.",
    )

    body_aabb = ctx.part_world_aabb(body)
    body_ok = (
        body_aabb is not None
        and body_aabb[1][2] > 0.23
        and body_aabb[1][0] > 0.14
        and body_aabb[0][2] < -0.015
    )
    ctx.check(
        "faucet keeps sink-fixture scale and reach",
        body_ok,
        details=f"body_aabb={body_aabb}",
    )

    ctx.expect_origin_gap(
        lever,
        body,
        axis="y",
        min_gap=0.012,
        max_gap=0.030,
        name="lever pivot sits on the faucet side",
    )
    ctx.expect_origin_gap(
        selector,
        body,
        axis="x",
        min_gap=0.010,
        max_gap=0.030,
        name="selector knob sits on the front collar",
    )

    with ctx.pose({lever_joint: 0.0}):
        closed_lever_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: 0.9}):
        open_lever_aabb = ctx.part_world_aabb(lever)

    ctx.check(
        "lever lifts upward on its side pivot",
        closed_lever_aabb is not None
        and open_lever_aabb is not None
        and open_lever_aabb[1][2] > closed_lever_aabb[1][2] + 0.02,
        details=f"closed={closed_lever_aabb}, open={open_lever_aabb}",
    )

    with ctx.pose({selector_joint: 0.0}):
        selector_rest_aabb = ctx.part_world_aabb(selector)
    with ctx.pose({selector_joint: math.pi / 2.0}):
        selector_turn_aabb = ctx.part_world_aabb(selector)

    rest_y = None if selector_rest_aabb is None else selector_rest_aabb[1][1] - selector_rest_aabb[0][1]
    rest_z = None if selector_rest_aabb is None else selector_rest_aabb[1][2] - selector_rest_aabb[0][2]
    turn_y = None if selector_turn_aabb is None else selector_turn_aabb[1][1] - selector_turn_aabb[0][1]
    turn_z = None if selector_turn_aabb is None else selector_turn_aabb[1][2] - selector_turn_aabb[0][2]

    ctx.check(
        "selector pointer visibly quarter-turns around its shaft",
        rest_y is not None
        and rest_z is not None
        and turn_y is not None
        and turn_z is not None
        and turn_y > rest_y + 0.003
        and rest_z > turn_z + 0.003,
        details=(
            f"rest_aabb={selector_rest_aabb}, turned_aabb={selector_turn_aabb}, "
            f"rest_spans={(rest_y, rest_z)}, turned_spans={(turn_y, turn_z)}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
