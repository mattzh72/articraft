from __future__ import annotations

import math

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
import cadquery as cq


BODY_W = 0.56
BODY_D = 0.42
BODY_H = 0.36
CORNER_R = 0.016

OPENING_X = -0.055
CAVITY_W = 0.39
CAVITY_H = 0.246
CAVITY_D = 0.335
CAVITY_BOTTOM = 0.058
CAVITY_FRONT_OVERTRAVEL = 0.003

DOOR_W = 0.404
DOOR_H = 0.246
DOOR_T = 0.018
DOOR_HINGE_Z = CAVITY_BOTTOM
DOOR_HINGE_Y = -BODY_D / 2 - DOOR_T / 2

CONTROL_X = 0.19
DIAL_Z = 0.255
BUTTON_ZS = (0.192, 0.159, 0.126)

RUNNER_W = 0.010
RUNNER_D = 0.29
RUNNER_H = 0.006
RUNNER_Z = 0.142

RACK_W = 0.386
RACK_D = 0.27
RACK_BAR = 0.006
RACK_HEIGHT = 0.020
RACK_TRAVEL = 0.13
RACK_Z = RUNNER_Z + RUNNER_H
RACK_Y = -BODY_D / 2 + 0.035

BUTTON_TRAVEL = 0.007


def _housing_shape() -> cq.Workplane:
    cavity_y = -BODY_D / 2 + CAVITY_D / 2 - CAVITY_FRONT_OVERTRAVEL
    housing = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
    housing = housing.edges("|Z").fillet(CORNER_R)

    cavity = (
        cq.Workplane("XY")
        .box(CAVITY_W, CAVITY_D, CAVITY_H, centered=(True, True, False))
        .translate((OPENING_X, cavity_y, CAVITY_BOTTOM))
    )
    housing = housing.cut(cavity)

    knob_hole = (
        cq.Workplane("XZ")
        .circle(0.013)
        .extrude(0.05)
        .translate((CONTROL_X, -BODY_D / 2 - 0.001, DIAL_Z))
    )
    housing = housing.cut(knob_hole)

    for button_z in BUTTON_ZS:
        slot = (
            cq.Workplane("XZ")
            .rect(0.034, 0.020)
            .extrude(0.045)
            .translate((CONTROL_X, -BODY_D / 2 - 0.001, button_z))
        )
        housing = housing.cut(slot)

    return housing


def _door_shape() -> cq.Workplane:
    door = cq.Workplane("XY").box(DOOR_W, DOOR_T, DOOR_H, centered=(True, True, False))
    window = (
        cq.Workplane("XY")
        .box(DOOR_W - 0.086, DOOR_T + 0.004, DOOR_H - 0.102, centered=(True, True, False))
        .translate((0.0, 0.0, 0.044))
    )
    door = door.cut(window)

    handle_bar = (
        cq.Workplane("XY")
        .box(0.24, 0.015, 0.012)
        .translate((0.0, -0.030, DOOR_H * 0.78))
    )
    door = door.union(handle_bar)

    for handle_x in (-0.075, 0.075):
        standoff = (
            cq.Workplane("XY")
            .box(0.012, 0.024, 0.018)
            .translate((handle_x, -0.019, DOOR_H * 0.78))
        )
        door = door.union(standoff)

    return door


def _dial_shape() -> cq.Workplane:
    dial = cq.Workplane("XZ").circle(0.028).extrude(0.022)
    flat_cutter = (
        cq.Workplane("XY")
        .box(0.018, 0.032, 0.080)
        .translate((0.023, 0.011, 0.0))
    )
    return dial.cut(flat_cutter)


def _button_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.048, 0.010, 0.016)
        .translate((0.0, -0.005, 0.0))
    )


def _rack_shape() -> cq.Workplane:
    rack = None

    def add_bar(current: cq.Workplane | None, bar: cq.Workplane) -> cq.Workplane:
        return bar if current is None else current.union(bar)

    side_x = RACK_W / 2 - RACK_BAR / 2
    for sign in (-1.0, 1.0):
        side_bar = (
            cq.Workplane("XY")
            .box(RACK_BAR, RACK_D, RACK_BAR, centered=(True, True, False))
            .translate((sign * side_x, RACK_D / 2, 0.0))
        )
        rack = add_bar(rack, side_bar)

    frame_width = RACK_W + 0.002

    front_bar = (
        cq.Workplane("XY")
        .box(frame_width, RACK_BAR, RACK_BAR, centered=(True, True, False))
        .translate((0.0, RACK_BAR / 2, 0.0))
    )
    rear_bar = (
        cq.Workplane("XY")
        .box(frame_width, RACK_BAR, RACK_BAR, centered=(True, True, False))
        .translate((0.0, RACK_D - RACK_BAR / 2, 0.0))
    )
    rack = add_bar(rack, front_bar)
    rack = add_bar(rack, rear_bar)

    for cross_y in (0.065, 0.130, 0.195):
        cross_bar = (
            cq.Workplane("XY")
            .box(frame_width, RACK_BAR, RACK_BAR, centered=(True, True, False))
            .translate((0.0, cross_y, 0.0))
        )
        rack = add_bar(rack, cross_bar)

    for sign in (-1.0, 1.0):
        side_lip = (
            cq.Workplane("XY")
            .box(RACK_BAR, RACK_D * 0.84, RACK_HEIGHT, centered=(True, True, False))
            .translate((sign * side_x, RACK_D * 0.49, RACK_BAR))
        )
        rack = add_bar(rack, side_lip)

    return rack


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_toaster_oven")

    model.material("brushed_steel", rgba=(0.83, 0.84, 0.86, 1.0))
    model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("rack_steel", rgba=(0.72, 0.73, 0.75, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell"),
        material="brushed_steel",
        name="housing_shell",
    )
    runner_y = -BODY_D / 2 + 0.03 + RUNNER_D / 2
    runner_offset = CAVITY_W / 2 - RUNNER_W / 2
    for index, sign in enumerate((-1.0, 1.0)):
        body.visual(
            Box((RUNNER_W, RUNNER_D, RUNNER_H)),
            origin=Origin(xyz=(OPENING_X + sign * runner_offset, runner_y, RUNNER_Z + RUNNER_H / 2)),
            material="rack_steel",
            name=f"runner_{index}",
        )

    for index, button_z in enumerate(BUTTON_ZS):
        body.visual(
            Box((0.056, 0.007, 0.022)),
            origin=Origin(xyz=(CONTROL_X, -BODY_D / 2 - 0.0035, button_z)),
            material="dark_trim",
            name=f"button_well_{index}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_shape(), "door_leaf"),
        material="dark_trim",
        name="door_leaf",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_dial_shape(), "dial_knob"),
        material="dark_trim",
        name="dial_knob",
    )

    for index in range(3):
        button = model.part(f"button_{index}")
        button.visual(
            mesh_from_cadquery(_button_shape(), f"button_cap_{index}"),
            material="dark_trim",
            name="button_cap",
        )

    rack = model.part("rack")
    rack.visual(
        mesh_from_cadquery(_rack_shape(), "rack_tray"),
        material="rack_steel",
        name="rack_tray",
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(OPENING_X, DOOR_HINGE_Y, DOOR_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.55, effort=12.0, velocity=1.4),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(CONTROL_X, -BODY_D / 2, DIAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    for index, button_z in enumerate(BUTTON_ZS):
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=f"button_{index}",
            origin=Origin(xyz=(CONTROL_X, -BODY_D / 2 - BUTTON_TRAVEL, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=3.0, velocity=0.08),
        )

    model.articulation(
        "body_to_rack",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(OPENING_X, RACK_Y, RACK_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=RACK_TRAVEL, effort=18.0, velocity=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    dial = object_model.get_part("dial")
    rack = object_model.get_part("rack")
    door_hinge = object_model.get_articulation("body_to_door")
    dial_joint = object_model.get_articulation("body_to_dial")
    rack_slide = object_model.get_articulation("body_to_rack")

    ctx.check(
        "selector dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={dial_joint.articulation_type}",
    )
    limits = dial_joint.motion_limits
    ctx.check(
        "selector dial has no stop limits",
        limits is not None and limits.lower is None and limits.upper is None,
        details=f"limits={limits}",
    )

    with ctx.pose({door_hinge: 0.0, rack_slide: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_leaf",
            elem_b="housing_shell",
            min_overlap=0.20,
            name="closed door covers the oven front opening",
        )
        ctx.expect_within(
            rack,
            body,
            axes="xz",
            inner_elem="rack_tray",
            outer_elem="housing_shell",
            margin=0.0,
            name="rack stays inside the cavity envelope at rest",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="rack_tray",
            elem_b="housing_shell",
            min_overlap=0.20,
            name="rack remains inserted at rest",
        )
        closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")
        rack_rest_pos = ctx.part_world_position(rack)

    with ctx.pose({door_hinge: 1.45, rack_slide: RACK_TRAVEL}):
        ctx.expect_overlap(
            rack,
            body,
            axes="y",
            elem_a="rack_tray",
            elem_b="housing_shell",
            min_overlap=0.12,
            name="rack still retains insertion at full extension",
        )
        ctx.expect_within(
            rack,
            body,
            axes="x",
            inner_elem="rack_tray",
            outer_elem="housing_shell",
            margin=0.0,
            name="rack stays laterally guided by the cavity when extended",
        )
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_leaf")
        rack_extended_pos = ctx.part_world_position(rack)

    ctx.check(
        "door drops outward when opened",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.10
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.08,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )
    ctx.check(
        "rack pulls forward on its runners",
        rack_rest_pos is not None
        and rack_extended_pos is not None
        and rack_extended_pos[1] < rack_rest_pos[1] - 0.08,
        details=f"rest={rack_rest_pos}, extended={rack_extended_pos}",
    )

    button_rest_positions = {}
    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_rest_positions[index] = ctx.part_world_position(button)
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} depresses inward",
            button_rest_positions[index] is not None
            and pressed_pos is not None
            and pressed_pos[1] > button_rest_positions[index][1] + 0.004,
            details=f"rest={button_rest_positions[index]}, pressed={pressed_pos}",
        )

    for active_index in range(3):
        active_joint = object_model.get_articulation(f"body_to_button_{active_index}")
        with ctx.pose({active_joint: BUTTON_TRAVEL}):
            active_pos = ctx.part_world_position(f"button_{active_index}")
            other_ok = True
            other_positions = {}
            for other_index in range(3):
                other_positions[other_index] = ctx.part_world_position(f"button_{other_index}")
                if other_index != active_index:
                    rest = button_rest_positions[other_index]
                    current = other_positions[other_index]
                    if rest is None or current is None or abs(current[1] - rest[1]) > 1e-5:
                        other_ok = False
        ctx.check(
            f"button_{active_index} moves independently",
            active_pos is not None
            and button_rest_positions[active_index] is not None
            and active_pos[1] > button_rest_positions[active_index][1] + 0.004
            and other_ok,
            details=f"rest={button_rest_positions}, posed={other_positions}",
        )

    return ctx.report()


object_model = build_object_model()
