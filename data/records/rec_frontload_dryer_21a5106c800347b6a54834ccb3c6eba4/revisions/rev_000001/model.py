from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.62
BODY_D = 0.68
BODY_H = 0.86
WALL = 0.025
FRONT_Y = -BODY_D / 2.0
DOOR_Z = 0.455
DOOR_HINGE_X = -0.275
DOOR_HINGE_Y = FRONT_Y - 0.036


def _annular_cylinder(outer_radius: float, inner_radius: float, depth: float) -> cq.Workplane:
    """A circular through-ring, authored along local +Z."""

    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(depth / 2.0, both=True)
    )


def _cabinet_shell() -> cq.Workplane:
    """One connected white appliance shell with a porthole and plinth cutout."""

    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H).translate((0.0, 0.0, BODY_H / 2.0))
    inner_h = BODY_H - 0.11
    inner = (
        cq.Workplane("XY")
        .box(BODY_W - 2.0 * WALL, BODY_D - 2.0 * WALL, inner_h)
        .translate((0.0, 0.0, 0.070 + inner_h / 2.0))
    )
    shell = outer.cut(inner)

    # Circular front opening for the glass porthole.
    porthole = (
        cq.Workplane("XY")
        .cylinder(0.11, 0.232)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((0.0, FRONT_Y + WALL / 2.0, DOOR_Z))
    )
    shell = shell.cut(porthole)

    # Rectangular heat-pump filter drawer aperture in the plinth.
    drawer_opening = (
        cq.Workplane("XY")
        .box(0.470, 0.120, 0.125)
        .translate((0.0, FRONT_Y + WALL / 2.0, 0.105))
    )
    shell = shell.cut(drawer_opening)
    return shell


def _drum_mesh() -> cq.Workplane:
    """Connected stainless drum: hollow shell, lifter ribs, hub and spider spokes."""

    radius = 0.226
    inner = 0.211
    length = 0.470
    drum = _annular_cylinder(radius, inner, length)

    # Three raised lifter ribs, connected to the inner wall.
    for angle in (0.0, 120.0, 240.0):
        rib = (
            cq.Workplane("XY")
            .box(0.046, 0.024, length * 0.86)
            .translate((0.0, inner - 0.006, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        drum = drum.union(rib)

    hub = cq.Workplane("XY").cylinder(0.060, 0.047)
    drum = drum.union(hub)
    for angle in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.030, radius * 1.45, 0.026)
            .translate((0.0, radius * 0.42, 0.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        drum = drum.union(spoke)
    return drum


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heat_pump_tumble_dryer")

    white = Material("warm_white_enamel", rgba=(0.94, 0.94, 0.91, 1.0))
    satin_white = Material("satin_white_plastic", rgba=(0.98, 0.98, 0.95, 1.0))
    chrome = Material("polished_chrome", rgba=(0.80, 0.82, 0.82, 1.0))
    stainless = Material("brushed_stainless", rgba=(0.64, 0.66, 0.65, 1.0))
    dark = Material("dark_glass_black", rgba=(0.02, 0.025, 0.030, 1.0))
    smoked = Material("smoked_translucent_glass", rgba=(0.05, 0.09, 0.12, 0.42))
    graphite = Material("graphite_trim", rgba=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell", tolerance=0.0012),
        material=white,
        name="cabinet_shell",
    )
    body.visual(
        mesh_from_cadquery(_annular_cylinder(0.248, 0.232, 0.008), "black_porthole_gasket"),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, DOOR_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="porthole_gasket",
    )
    body.visual(
        Box((0.260, 0.006, 0.060)),
        origin=Origin(xyz=(-0.110, FRONT_Y - 0.003, 0.780)),
        material=dark,
        name="display_window",
    )
    body.visual(
        Box((0.032, 0.040, 0.075)),
        origin=Origin(xyz=(DOOR_HINGE_X - 0.031, FRONT_Y - 0.020, DOOR_Z + 0.170)),
        material=chrome,
        name="upper_hinge_leaf",
    )
    body.visual(
        Box((0.032, 0.040, 0.075)),
        origin=Origin(xyz=(DOOR_HINGE_X - 0.031, FRONT_Y - 0.020, DOOR_Z - 0.170)),
        material=chrome,
        name="lower_hinge_leaf",
    )
    body.visual(
        Box((0.520, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.005, 0.015)),
        material=graphite,
        name="recessed_plinth_shadow",
    )
    body.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(xyz=(0.0, 0.305, DOOR_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rear_bearing",
    )

    drum = model.part("drum")
    drum.visual(
        mesh_from_cadquery(_drum_mesh(), "drum_shell", tolerance=0.0012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="drum_shell",
    )
    drum.visual(
        Cylinder(radius=0.018, length=0.540),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="center_axle",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_annular_cylinder(0.258, 0.184, 0.045), "chrome_door_bezel"),
        origin=Origin(xyz=(0.275, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="chrome_bezel",
    )
    door.visual(
        Cylinder(radius=0.188, length=0.012),
        origin=Origin(xyz=(0.275, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked,
        name="glass_porthole",
    )
    door.visual(
        Box((0.030, 0.018, 0.150)),
        origin=Origin(xyz=(0.515, -0.030, 0.0)),
        material=satin_white,
        name="pull_grip",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=chrome,
        name="upper_barrel",
    )
    door.visual(
        Box((0.090, 0.014, 0.070)),
        origin=Origin(xyz=(0.045, 0.0, 0.170)),
        material=chrome,
        name="upper_door_leaf",
    )
    door.visual(
        Cylinder(radius=0.015, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material=chrome,
        name="lower_barrel",
    )
    door.visual(
        Box((0.090, 0.014, 0.070)),
        origin=Origin(xyz=(0.045, 0.0, -0.170)),
        material=chrome,
        name="lower_door_leaf",
    )

    filter_drawer = model.part("filter_drawer")
    filter_drawer.visual(
        Box((0.500, 0.022, 0.145)),
        origin=Origin(),
        material=satin_white,
        name="front_fascia",
    )
    filter_drawer.visual(
        Box((0.390, 0.300, 0.066)),
        origin=Origin(xyz=(0.0, 0.155, 0.000)),
        material=graphite,
        name="filter_tray",
    )
    filter_drawer.visual(
        Box((0.260, 0.006, 0.026)),
        origin=Origin(xyz=(0.0, -0.014, 0.010)),
        material=dark,
        name="drawer_grip_slot",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.043, length=0.023),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_white,
        name="dial_cap",
    )
    dial.visual(
        Box((0.006, 0.004, 0.035)),
        origin=Origin(xyz=(0.0, -0.013, 0.020)),
        material=graphite,
        name="dial_pointer",
    )

    model.articulation(
        "body_to_drum",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, DOOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=9.0),
    )
    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.6, lower=0.0, upper=1.85),
    )
    model.articulation(
        "body_to_filter_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=filter_drawer,
        origin=Origin(xyz=(0.0, FRONT_Y - 0.011, 0.105)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.185),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.205, FRONT_Y - 0.0115, 0.780)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    drawer = object_model.get_part("filter_drawer")
    dial = object_model.get_part("dial")
    drum_joint = object_model.get_articulation("body_to_drum")
    door_joint = object_model.get_articulation("body_to_door")
    drawer_joint = object_model.get_articulation("body_to_filter_drawer")

    ctx.check(
        "central drum uses a continuous revolute axle",
        drum_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(drum_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={drum_joint.articulation_type}, axis={drum_joint.axis}",
    )
    ctx.expect_within(
        drum,
        body,
        axes="xz",
        inner_elem="drum_shell",
        outer_elem="cabinet_shell",
        margin=0.0,
        name="drum fits within the appliance body envelope",
    )
    ctx.expect_gap(
        body,
        door,
        axis="y",
        positive_elem="cabinet_shell",
        negative_elem="chrome_bezel",
        min_gap=0.005,
        max_gap=0.030,
        name="closed porthole door sits proud of the front shell",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a="upper_barrel",
        elem_b="upper_hinge_leaf",
        min_overlap=0.060,
        name="upper barrel hinge aligns with its body leaf",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="z",
        elem_a="lower_barrel",
        elem_b="lower_hinge_leaf",
        min_overlap=0.060,
        name="lower barrel hinge aligns with its body leaf",
    )
    ctx.expect_gap(
        body,
        drawer,
        axis="y",
        positive_elem="cabinet_shell",
        negative_elem="front_fascia",
        min_gap=0.0,
        max_gap=0.002,
        name="filter drawer fascia contacts the plinth front",
    )
    ctx.expect_within(
        drawer,
        body,
        axes="xz",
        inner_elem="filter_tray",
        outer_elem="cabinet_shell",
        margin=0.002,
        name="filter tray is centered in the plinth opening",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="cabinet_shell",
        contact_tol=0.002,
        name="selector dial is mounted on the front panel",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(door, elem="pull_grip")
    with ctx.pose({door_joint: 1.25}):
        open_handle_aabb = ctx.part_element_world_aabb(door, elem="pull_grip")
    ctx.check(
        "door opens outward from the left hinge edge",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[0][1] < closed_handle_aabb[0][1] - 0.10,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.185}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            body,
            axes="y",
            elem_a="filter_tray",
            elem_b="cabinet_shell",
            min_overlap=0.10,
            name="extended filter drawer retains insertion in the cabinet",
        )
    ctx.check(
        "filter drawer slides outward from the plinth",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.15,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
