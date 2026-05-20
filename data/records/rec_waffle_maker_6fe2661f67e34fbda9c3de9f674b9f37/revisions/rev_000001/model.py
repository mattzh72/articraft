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


BODY_W = 0.305
BODY_D = 0.245
LOWER_SLAB_H = 0.024
DECK_H = 0.017
DECK_TOP = LOWER_SLAB_H + DECK_H
SIDE_CHEEK_H = 0.048
FRONT_BROW_H = 0.046
HINGE_COVER_H = 0.050
FRONT_PANEL_FRONT_Y = -0.125
HINGE_Y = 0.103
HINGE_Z = 0.056
CONTROL_Z = 0.031
BUTTON_TRAVEL = 0.0025


def box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -(length / 2.0)))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((x, y, z))
    )


def waffle_plate(
    width: float,
    depth: float,
    thickness: float,
    groove_depth: float,
    groove_width: float,
    pitch: float,
) -> cq.Workplane:
    plate = box_shape((width, depth, thickness), (0.0, 0.0, thickness / 2.0))

    grid_x_positions = (-2, -1, 0, 1, 2)
    for idx in grid_x_positions:
        x = idx * pitch
        groove = box_shape(
            (groove_width, depth - 0.020, groove_depth),
            (x, 0.0, thickness - (groove_depth / 2.0)),
        )
        plate = plate.cut(groove)

    grid_y_positions = (-2, -1, 0, 1, 2)
    for idx in grid_y_positions:
        y = idx * pitch
        groove = box_shape(
            (width - 0.020, groove_width, groove_depth),
            (0.0, y, thickness - (groove_depth / 2.0)),
        )
        plate = plate.cut(groove)

    return plate


def build_base_shell() -> cq.Workplane:
    slab = box_shape((BODY_W, BODY_D, LOWER_SLAB_H), (0.0, 0.0, LOWER_SLAB_H / 2.0))
    deck = box_shape((0.268, 0.205, DECK_H), (0.0, -0.004, LOWER_SLAB_H + (DECK_H / 2.0)))
    cheek_y = 0.014
    cheek_z = SIDE_CHEEK_H / 2.0
    cheek_half_x = 0.129
    cheeks = box_shape((0.044, 0.175, SIDE_CHEEK_H), (-cheek_half_x, cheek_y, cheek_z)).union(
        box_shape((0.044, 0.175, SIDE_CHEEK_H), (cheek_half_x, cheek_y, cheek_z))
    )
    front_brow = box_shape((0.225, 0.042, FRONT_BROW_H), (0.0, -0.100, FRONT_BROW_H / 2.0))
    hinge_cover = box_shape((0.185, 0.034, HINGE_COVER_H), (0.0, 0.107, HINGE_COVER_H / 2.0))

    shell = slab.union(deck).union(cheeks).union(front_brow).union(hinge_cover)

    dial_hole = y_cylinder(0.0072, 0.070, (0.0, -0.098, CONTROL_Z))
    ready_hole = y_cylinder(0.0053, 0.070, (-0.058, -0.098, CONTROL_Z))
    power_hole = y_cylinder(0.0053, 0.070, (0.058, -0.098, CONTROL_Z))
    return shell.cut(dial_hole).cut(ready_hole).cut(power_hole)


def build_front_panel() -> cq.Workplane:
    panel = box_shape((0.236, 0.004, 0.042), (0.0, FRONT_PANEL_FRONT_Y + 0.002, CONTROL_Z))
    panel = panel.cut(y_cylinder(0.0082, 0.008, (0.0, FRONT_PANEL_FRONT_Y + 0.002, CONTROL_Z)))
    panel = panel.cut(y_cylinder(0.0058, 0.008, (-0.058, FRONT_PANEL_FRONT_Y + 0.002, CONTROL_Z)))
    panel = panel.cut(y_cylinder(0.0058, 0.008, (0.058, FRONT_PANEL_FRONT_Y + 0.002, CONTROL_Z)))
    return panel


def build_lid_shell() -> cq.Workplane:
    lower_shell = box_shape((0.292, 0.220, 0.022), (0.0, -0.110, 0.008))
    top_dome = box_shape((0.240, 0.166, 0.020), (0.0, -0.112, 0.028))
    front_nose = box_shape((0.190, 0.020, 0.010), (0.0, -0.221, 0.014))
    shell = lower_shell.union(top_dome).union(front_nose)
    cavity = box_shape((0.242, 0.186, 0.0175), (0.0, -0.111, 0.00575))
    return shell.cut(cavity)


def build_handle() -> cq.Workplane:
    bar = box_shape((0.150, 0.016, 0.012), (0.0, -0.229, 0.018))
    left_post = box_shape((0.020, 0.010, 0.018), (-0.050, -0.216, 0.015))
    right_post = box_shape((0.020, 0.010, 0.018), (0.050, -0.216, 0.015))
    handle = bar.union(left_post).union(right_post)
    return handle


def build_button(radius: float = 0.0095) -> cq.Workplane:
    cap = y_cylinder(radius, 0.004, (0.0, -0.127, 0.0))
    stem = y_cylinder(0.0048, 0.011, (0.0, -0.1195, 0.0))
    button = cap.union(stem)
    return button


def build_dial() -> cq.Workplane:
    bezel = y_cylinder(0.019, 0.003, (0.0, -0.1265, 0.0))
    knob = y_cylinder(0.016, 0.008, (0.0, -0.130, 0.0))
    shaft = y_cylinder(0.0062, 0.024, (0.0, -0.113, 0.0))
    pointer = box_shape((0.012, 0.0018, 0.0020), (0.0, -0.134, 0.010))
    dial = bezel.union(knob).union(shaft).union(pointer)
    return dial


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_waffle_maker")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    graphite = model.material("graphite", rgba=(0.19, 0.20, 0.22, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    plate_black = model.material("plate_black", rgba=(0.12, 0.12, 0.13, 1.0))
    handle_black = model.material("handle_black", rgba=(0.09, 0.09, 0.10, 1.0))
    ready_green = model.material("ready_green", rgba=(0.20, 0.66, 0.30, 1.0))
    power_red = model.material("power_red", rgba=(0.79, 0.18, 0.18, 1.0))
    knob_black = model.material("knob_black", rgba=(0.11, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_shell(), "base_shell"),
        material=stainless,
        name="base_shell",
    )
    base.visual(
        mesh_from_cadquery(build_front_panel(), "indicator_panel"),
        material=panel_black,
        name="indicator_panel",
    )
    base.visual(
        mesh_from_cadquery(
            waffle_plate(width=0.246, depth=0.191, thickness=0.0065, groove_depth=0.0022, groove_width=0.0035, pitch=0.037),
            "lower_plate",
        ),
        origin=Origin(xyz=(0.0, -0.004, 0.0405)),
        material=plate_black,
        name="lower_plate",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(build_lid_shell(), "lid_shell"),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=graphite,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(
            waffle_plate(width=0.246, depth=0.189, thickness=0.0065, groove_depth=0.0020, groove_width=0.0034, pitch=0.037),
            "upper_plate",
        ),
        origin=Origin(xyz=(0.0, -0.111, -0.003)),
        material=plate_black,
        name="upper_plate",
    )
    lid.visual(
        mesh_from_cadquery(build_handle(), "front_handle"),
        material=handle_black,
        name="front_handle",
    )

    ready_button = model.part("ready_button")
    ready_button.visual(
        mesh_from_cadquery(build_button(), "ready_button"),
        material=ready_green,
        name="ready_button",
    )

    power_button = model.part("power_button")
    power_button.visual(
        mesh_from_cadquery(build_button(), "power_button"),
        material=power_red,
        name="power_button",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(build_dial(), "browning_dial"),
        material=knob_black,
        name="browning_dial",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=18.0, velocity=2.0),
    )
    model.articulation(
        "ready_button_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=ready_button,
        origin=Origin(xyz=(-0.058, 0.0, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=6.0, velocity=0.08),
    )
    model.articulation(
        "power_button_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=power_button,
        origin=Origin(xyz=(0.058, 0.0, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BUTTON_TRAVEL, effort=6.0, velocity=0.08),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, CONTROL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    ready_button = object_model.get_part("ready_button")
    power_button = object_model.get_part("power_button")
    dial = object_model.get_part("dial")

    lid_hinge = object_model.get_articulation("lid_hinge")
    ready_joint = object_model.get_articulation("ready_button_slide")
    power_joint = object_model.get_articulation("power_button_slide")
    dial_joint = object_model.get_articulation("dial_spin")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            base,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.004,
            max_gap=0.007,
            name="closed plates leave a shallow batter gap",
        )
        ctx.expect_overlap(
            lid,
            base,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.175,
            name="closed plates align over the cooking area",
        )

    hinge_limits = lid_hinge.motion_limits
    if hinge_limits is not None and hinge_limits.upper is not None:
        closed_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
        with ctx.pose({lid_hinge: hinge_limits.upper}):
            opened_handle = ctx.part_element_world_aabb(lid, elem="front_handle")
        ctx.check(
            "lid opens upward on the rear hinge",
            closed_handle is not None
            and opened_handle is not None
            and opened_handle[1][2] > closed_handle[1][2] + 0.10,
            details=f"closed_handle={closed_handle}, opened_handle={opened_handle}",
        )

    ready_rest = ctx.part_world_position(ready_button)
    with ctx.pose({ready_joint: BUTTON_TRAVEL}):
        ready_pressed = ctx.part_world_position(ready_button)
    ctx.check(
        "ready button presses inward",
        ready_rest is not None
        and ready_pressed is not None
        and ready_pressed[1] > ready_rest[1] + 0.0015,
        details=f"rest={ready_rest}, pressed={ready_pressed}",
    )

    power_rest = ctx.part_world_position(power_button)
    with ctx.pose({power_joint: BUTTON_TRAVEL}):
        power_pressed = ctx.part_world_position(power_button)
    ctx.check(
        "power button presses inward",
        power_rest is not None
        and power_pressed is not None
        and power_pressed[1] > power_rest[1] + 0.0015,
        details=f"rest={power_rest}, pressed={power_pressed}",
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial uses an unbounded continuous joint",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_limits is not None
        and dial_limits.lower is None
        and dial_limits.upper is None,
        details=f"type={dial_joint.articulation_type}, limits={dial_limits}",
    )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_rotated = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates about a fixed center",
        dial_rest is not None
        and dial_rotated is not None
        and abs(dial_rotated[0] - dial_rest[0]) < 1e-6
        and abs(dial_rotated[1] - dial_rest[1]) < 1e-6
        and abs(dial_rotated[2] - dial_rest[2]) < 1e-6,
        details=f"rest={dial_rest}, rotated={dial_rotated}",
    )

    return ctx.report()


object_model = build_object_model()
