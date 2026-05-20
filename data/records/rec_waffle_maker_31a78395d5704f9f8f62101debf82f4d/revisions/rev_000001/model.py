from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

OUTER_LEN = 0.31
OUTER_WID = 0.275
BODY_RADIUS = 0.018
WALL = 0.004
FOOT_HEIGHT = 0.005
BASE_TOP = 0.074
LID_TOP = 0.066
HINGE_X = -0.146
HINGE_Z = 0.072

PLATE_SIZE = 0.228
PLATE_THICKNESS = 0.018
LOWER_PLATE_BOTTOM = 0.039
UPPER_PLATE_BOTTOM = -0.010

BASE_PANEL_X = 0.046
PANEL_LEN = 0.118
PANEL_HEIGHT = 0.048
PANEL_RECESS = 0.0025

DIAL_X = 0.076
DIAL_Z = 0.043
BUTTON_X = 0.012
UPPER_BUTTON_Z = 0.050
LOWER_BUTTON_Z = 0.032


def _y_cylinder(radius: float, length: float):
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length * 0.5))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
    )


def _z_box(length: float, width: float, height: float, *, z0: float = 0.0):
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False)).translate((0.0, 0.0, z0))


def _build_base_shell():
    shell = _z_box(OUTER_LEN, OUTER_WID, BASE_TOP - FOOT_HEIGHT, z0=FOOT_HEIGHT)
    shell = shell.edges("|Z").fillet(BODY_RADIUS)

    cavity = _z_box(
        OUTER_LEN - 2.0 * WALL,
        OUTER_WID - 2.0 * WALL,
        BASE_TOP - FOOT_HEIGHT - WALL + 0.002,
        z0=FOOT_HEIGHT + WALL,
    )
    shell = shell.cut(cavity)

    pedestal = _z_box(
        PLATE_SIZE - 0.066,
        PLATE_SIZE - 0.066,
        LOWER_PLATE_BOTTOM - (FOOT_HEIGHT + WALL),
        z0=FOOT_HEIGHT + WALL,
    )
    shell = shell.union(pedestal)

    hinge_relief = cq.Workplane("XY").box(0.020, 0.120, 0.020).translate((HINGE_X - 0.001, 0.0, 0.069))
    shell = shell.cut(hinge_relief)

    panel_recess = cq.Workplane("XY").box(PANEL_LEN, PANEL_RECESS, PANEL_HEIGHT).translate(
        (BASE_PANEL_X, OUTER_WID * 0.5 - PANEL_RECESS * 0.5, 0.039)
    )
    shell = shell.cut(panel_recess)

    control_wall_y = OUTER_WID * 0.5 - PANEL_RECESS - 0.002
    dial_hole = _y_cylinder(0.0065, 0.018).translate((DIAL_X, control_wall_y, DIAL_Z))
    upper_button_slot = cq.Workplane("XY").box(0.012, 0.018, 0.009).translate(
        (BUTTON_X, control_wall_y, UPPER_BUTTON_Z)
    )
    lower_button_slot = cq.Workplane("XY").box(0.012, 0.018, 0.009).translate(
        (BUTTON_X, control_wall_y, LOWER_BUTTON_Z)
    )
    shell = shell.cut(dial_hole).cut(upper_button_slot).cut(lower_button_slot)

    return shell


def _build_base_hinge():
    hinge = None
    for y_pos in (-0.081, 0.081):
        barrel = _y_cylinder(0.0065, 0.034).translate((HINGE_X, y_pos, HINGE_Z))
        hinge = barrel if hinge is None else hinge.union(barrel)
    return hinge


def _build_feet():
    feet = None
    for x_pos in (-0.102, 0.102):
        for y_pos in (-0.088, 0.088):
            foot = (
                cq.Workplane("XY")
                .circle(0.013)
                .extrude(FOOT_HEIGHT)
                .translate((x_pos, y_pos, 0.0))
            )
            feet = foot if feet is None else feet.union(foot)
    return feet


def _build_lid_shell():
    lid_len = 0.302
    shell = _z_box(lid_len, OUTER_WID, LID_TOP - 0.003, z0=0.003).translate((lid_len * 0.5, 0.0, 0.0))
    shell = shell.edges("|Z").fillet(BODY_RADIUS)

    cavity = _z_box(
        lid_len - 0.018,
        OUTER_WID - 2.0 * WALL,
        LID_TOP - 0.011,
        z0=0.003,
    ).translate((lid_len * 0.5 + 0.006, 0.0, 0.0))
    shell = shell.cut(cavity)

    roof_boss = _z_box(PLATE_SIZE - 0.074, PLATE_SIZE - 0.074, 0.052, z0=UPPER_PLATE_BOTTOM + PLATE_THICKNESS)
    shell = shell.union(roof_boss.translate((lid_len * 0.5 + 0.004, 0.0, 0.0)))

    for y_pos in (-0.032, 0.032):
        standoff = cq.Workplane("XY").box(0.014, 0.016, 0.020).translate((lid_len - 0.002, y_pos, 0.030))
        shell = shell.union(standoff)

    return shell


def _build_lid_hinge():
    return _y_cylinder(0.006, 0.106)


def _build_dial():
    stem = _y_cylinder(0.0052, 0.012).translate((0.0, -0.005, 0.0))
    body = _y_cylinder(0.017, 0.010).translate((0.0, 0.005, 0.0))
    face = _y_cylinder(0.015, 0.003).translate((0.0, 0.0115, 0.0))
    indicator = cq.Workplane("XY").box(0.004, 0.002, 0.007).translate((0.0, 0.0125, 0.010))
    return stem.union(body).union(face).union(indicator)


def _build_button():
    cap = cq.Workplane("XY").box(0.016, 0.009, 0.010).translate((0.0, 0.0045, 0.0))
    stem = cq.Workplane("XY").box(0.010, 0.012, 0.006).translate((0.0, -0.004, 0.0))
    return cap.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="square_waffle_iron")

    shell_silver = model.material("shell_silver", rgba=(0.73, 0.74, 0.76, 1.0))
    plate_iron = model.material("plate_iron", rgba=(0.19, 0.19, 0.20, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.07, 0.07, 0.08, 1.0))
    button_black = model.material("button_black", rgba=(0.13, 0.13, 0.14, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_shell(), "base_shell"), material=shell_silver, name="base_shell")
    base.visual(mesh_from_cadquery(_build_base_hinge(), "base_hinge"), material=shell_silver, name="base_hinge")
    base.visual(
        Box((PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_PLATE_BOTTOM + PLATE_THICKNESS * 0.5)),
        material=plate_iron,
        name="lower_plate",
    )
    base.visual(mesh_from_cadquery(_build_feet(), "feet"), material=rubber_black, name="feet")

    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_build_lid_shell(), "lid_shell"), material=shell_silver, name="lid_shell")
    lid.visual(mesh_from_cadquery(_build_lid_hinge(), "lid_hinge"), material=shell_silver, name="lid_hinge")
    lid.visual(
        Box((PLATE_SIZE, PLATE_SIZE, PLATE_THICKNESS)),
        origin=Origin(xyz=(0.155, 0.0, UPPER_PLATE_BOTTOM + PLATE_THICKNESS * 0.5)),
        material=plate_iron,
        name="upper_plate",
    )
    lid.visual(
        Box((0.018, 0.092, 0.014)),
        origin=Origin(xyz=(0.316, 0.0, 0.031)),
        material=plastic_black,
        name="handle",
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    dial = model.part("dial")
    dial.visual(mesh_from_cadquery(_build_dial(), "dial_knob"), material=plastic_black, name="knob")
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(DIAL_X, OUTER_WID * 0.5 - PANEL_RECESS, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    upper_button = model.part("upper_button")
    upper_button.visual(mesh_from_cadquery(_build_button(), "upper_button"), material=button_black, name="button")
    model.articulation(
        "base_to_upper_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_button,
        origin=Origin(xyz=(BUTTON_X, OUTER_WID * 0.5 - PANEL_RECESS, UPPER_BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    lower_button = model.part("lower_button")
    lower_button.visual(mesh_from_cadquery(_build_button(), "lower_button"), material=button_black, name="button")
    model.articulation(
        "base_to_lower_button",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_button,
        origin=Origin(xyz=(BUTTON_X, OUTER_WID * 0.5 - PANEL_RECESS, LOWER_BUTTON_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.08, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    dial = object_model.get_part("dial")
    lid = object_model.get_part("lid")
    lower_button = object_model.get_part("lower_button")
    hinge = object_model.get_articulation("base_to_lid")
    upper_button = object_model.get_part("upper_button")
    upper_button_joint = object_model.get_articulation("base_to_upper_button")
    lower_button_joint = object_model.get_articulation("base_to_lower_button")

    ctx.allow_overlap(
        base,
        dial,
        elem_a="base_shell",
        elem_b="knob",
        reason="The temperature dial is represented as a single knob proxy seated into the recessed side-wall bushing.",
    )
    ctx.allow_overlap(
        base,
        upper_button,
        elem_a="base_shell",
        elem_b="button",
        reason="The upper push button uses a simplified cap-and-plunger proxy that intentionally nests into the side-wall guide slot.",
    )
    ctx.allow_overlap(
        base,
        lower_button,
        elem_a="base_shell",
        elem_b="button",
        reason="The lower push button uses a simplified cap-and-plunger proxy that intentionally nests into the side-wall guide slot.",
    )

    ctx.expect_overlap(
        lid,
        base,
        axes="xy",
        elem_a="upper_plate",
        elem_b="lower_plate",
        min_overlap=0.18,
        name="plates overlap in plan when closed",
    )
    ctx.expect_gap(
        lid,
        base,
        axis="z",
        positive_elem="upper_plate",
        negative_elem="lower_plate",
        min_gap=0.004,
        max_gap=0.010,
        name="closed plate spacing stays realistic",
    )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="upper_plate")
    with ctx.pose({hinge: 1.55}):
        opened_aabb = ctx.part_element_world_aabb(lid, elem="upper_plate")

    ctx.check(
        "lid opens upward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.18,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.expect_origin_gap(
        dial,
        base,
        axis="y",
        min_gap=0.13,
        max_gap=0.14,
        name="temperature dial sits on the right side wall",
    )

    upper_rest = ctx.part_world_position(upper_button)
    lower_rest = ctx.part_world_position(lower_button)
    with ctx.pose({upper_button_joint: 0.004, lower_button_joint: 0.004}):
        upper_pressed = ctx.part_world_position(upper_button)
        lower_pressed = ctx.part_world_position(lower_button)

    ctx.check(
        "upper button presses inward",
        upper_rest is not None and upper_pressed is not None and upper_pressed[1] < upper_rest[1] - 0.003,
        details=f"rest={upper_rest}, pressed={upper_pressed}",
    )
    ctx.check(
        "lower button presses inward",
        lower_rest is not None and lower_pressed is not None and lower_pressed[1] < lower_rest[1] - 0.003,
        details=f"rest={lower_rest}, pressed={lower_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
