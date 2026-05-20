from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


DEPTH = 0.42
WIDTH = 0.58
HEIGHT = 0.32
FRONT_X = -DEPTH / 2.0

OPEN_Y = 0.045
OPEN_W = 0.43
OPEN_Z = 0.010
OPEN_H = 0.220
OPEN_BOTTOM = OPEN_Z - OPEN_H / 2.0

DOOR_W = 0.455
DOOR_H = 0.255
DOOR_THICK = 0.018
HINGE_X = FRONT_X - 0.009
HINGE_Z = OPEN_BOTTOM - 0.005

TRAY_SLOT_Z = -0.132
TRAY_Z = TRAY_SLOT_Z - 0.009


def _housing_shell() -> cq.Workplane:
    """Single continuous metal toaster-oven body with oven mouth and tray slot."""
    outer = cq.Workplane("XY").box(DEPTH, WIDTH, HEIGHT)

    oven_cavity = (
        cq.Workplane("XY")
        .box(0.390, OPEN_W, OPEN_H)
        .translate((FRONT_X - 0.020 + 0.390 / 2.0, OPEN_Y, OPEN_Z))
    )
    tray_slot = (
        cq.Workplane("XY")
        .box(0.390, 0.405, 0.028)
        .translate((FRONT_X - 0.025 + 0.390 / 2.0, OPEN_Y, TRAY_SLOT_Z))
    )
    return outer.cut(oven_cavity).cut(tray_slot)


def _door_frame() -> cq.Workplane:
    """A one-piece metal door frame whose local origin is the bottom hinge line."""
    glass_w = 0.360
    glass_h = 0.150
    glass_z = 0.130
    outer = (
        cq.Workplane("XY")
        .box(DOOR_THICK, DOOR_W, DOOR_H)
        .translate((0.0, 0.0, DOOR_H / 2.0))
    )
    window = (
        cq.Workplane("XY")
        .box(DOOR_THICK * 3.0, glass_w, glass_h)
        .translate((0.0, 0.0, glass_z))
    )
    return outer.cut(window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_convection_oven")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.70, 0.66, 1.0))
    dark = model.material("black_enamel", rgba=(0.035, 0.035, 0.038, 1.0))
    charcoal = model.material("charcoal_trim", rgba=(0.10, 0.105, 0.11, 1.0))
    glass = model.material("smoky_glass", rgba=(0.12, 0.18, 0.20, 0.38))
    chrome = model.material("chrome_wire", rgba=(0.86, 0.86, 0.82, 1.0))
    warm = model.material("warm_heater", rgba=(1.0, 0.36, 0.06, 1.0))
    white = model.material("white_markings", rgba=(0.92, 0.90, 0.82, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((DEPTH, WIDTH, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, HEIGHT / 2.0 - 0.016)),
        material=stainless,
        name="top_panel",
    )
    housing.visual(
        Box((DEPTH, WIDTH, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -HEIGHT / 2.0 + 0.007)),
        material=stainless,
        name="bottom_panel",
    )
    housing.visual(
        Box((0.032, WIDTH, HEIGHT)),
        origin=Origin(xyz=(DEPTH / 2.0 - 0.016, 0.0, 0.0)),
        material=stainless,
        name="rear_wall",
    )
    housing.visual(
        Box((DEPTH, 0.030, HEIGHT)),
        origin=Origin(xyz=(0.0, WIDTH / 2.0 - 0.015, 0.0)),
        material=stainless,
        name="side_wall",
    )
    housing.visual(
        Box((DEPTH, 0.110, HEIGHT)),
        origin=Origin(xyz=(0.0, -0.235, 0.0)),
        material=stainless,
        name="control_column",
    )
    housing.visual(
        Box((0.350, OPEN_W, 0.012)),
        origin=Origin(xyz=(-0.020, OPEN_Y, OPEN_BOTTOM - 0.006)),
        material=stainless,
        name="cavity_floor",
    )
    housing.visual(
        Box((0.030, DOOR_W, 0.040)),
        origin=Origin(xyz=(FRONT_X + 0.015, OPEN_Y, OPEN_Z + OPEN_H / 2.0 + 0.020)),
        material=stainless,
        name="front_brow",
    )
    housing.visual(
        Box((0.030, DOOR_W, 0.026)),
        origin=Origin(xyz=(FRONT_X + 0.015, OPEN_Y, OPEN_BOTTOM - 0.013)),
        material=stainless,
        name="front_sill",
    )
    housing.visual(
        Box((0.330, 0.032, 0.006)),
        origin=Origin(xyz=(-0.040, -0.164, TRAY_Z - 0.008)),
        material=stainless,
        name="tray_rail_0",
    )
    housing.visual(
        Box((0.330, 0.020, 0.006)),
        origin=Origin(xyz=(-0.040, 0.251, TRAY_Z - 0.008)),
        material=stainless,
        name="tray_rail_1",
    )
    housing.visual(
        Box((0.001, 0.095, 0.260)),
        origin=Origin(xyz=(FRONT_X - 0.0005, -0.235, 0.000)),
        material=charcoal,
        name="control_panel",
    )
    housing.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.210, 0.360),
                0.003,
                slot_size=(0.034, 0.006),
                pitch=(0.047, 0.022),
                frame=0.012,
                corner_radius=0.006,
                stagger=True,
                center=False,
            ),
            "top_vent",
        ),
        origin=Origin(xyz=(0.015, 0.030, HEIGHT / 2.0)),
        material=dark,
        name="top_vent",
    )

    # Interior rack wires and heating elements embed into the side walls, so
    # they read as mounted rather than floating inside the cavity.
    for i, x in enumerate((-0.125, -0.070, -0.015, 0.040, 0.095)):
        housing.visual(
            Cylinder(radius=0.0022, length=0.450),
            origin=Origin(xyz=(x, OPEN_Y, -0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"rack_wire_{i}",
        )
    for i, (x, z) in enumerate(((-0.040, -0.065), (0.080, 0.080))):
        housing.visual(
            Cylinder(radius=0.0055, length=0.455),
            origin=Origin(xyz=(x, OPEN_Y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm,
            name=f"heater_rod_{i}",
        )

    for i, z in enumerate((0.092, 0.012, -0.068)):
        housing.visual(
            Box((0.001, 0.030, 0.004)),
            origin=Origin(xyz=(FRONT_X - 0.0010, -0.235, z + 0.035)),
            material=white,
            name=f"dial_mark_{i}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "door_frame", tolerance=0.0008),
        material=dark,
        name="door_frame",
    )
    door.visual(
        Box((0.006, 0.382, 0.172)),
        origin=Origin(xyz=(-0.003, 0.0, 0.130)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.340),
        origin=Origin(xyz=(-0.043, 0.0, DOOR_H - 0.036), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="door_handle",
    )
    for i, y in enumerate((-0.135, 0.135)):
        door.visual(
            Box((0.024, 0.014, 0.032)),
            origin=Origin(xyz=(-0.020, y, DOOR_H - 0.036)),
            material=stainless,
            name=f"handle_standoff_{i}",
        )

    tray = model.part("crumb_tray")
    tray.visual(
        Box((0.315, 0.395, 0.010)),
        origin=Origin(xyz=(0.1575, 0.0, 0.0)),
        material=stainless,
        name="tray_pan",
    )
    tray.visual(
        Box((0.014, 0.415, 0.032)),
        origin=Origin(xyz=(-0.007, 0.0, 0.001)),
        material=stainless,
        name="front_lip",
    )
    tray.visual(
        Box((0.014, 0.150, 0.015)),
        origin=Origin(xyz=(-0.020, 0.0, 0.010)),
        material=dark,
        name="tray_pull",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.026,
            body_style="skirted",
            top_diameter=0.035,
            skirt=KnobSkirt(0.049, 0.0045, flare=0.05, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0007),
            center=False,
        ),
        "control_dial",
    )
    dial_specs = (
        ("timer_dial", 0.092),
        ("temp_dial", 0.012),
        ("function_dial", -0.068),
    )
    for name, z in dial_specs:
        dial = model.part(name)
        dial.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
            material=dark,
            name="dial_cap",
        )
        model.articulation(
            f"housing_to_{name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=dial,
            origin=Origin(xyz=(FRONT_X - 0.001, -0.235, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.25, velocity=4.0, lower=-2.35, upper=2.35),
        )

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(HINGE_X, OPEN_Y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.82),
    )
    model.articulation(
        "housing_to_crumb_tray",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=tray,
        origin=Origin(xyz=(FRONT_X - 0.006, OPEN_Y, TRAY_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    tray = object_model.get_part("crumb_tray")
    door_joint = object_model.get_articulation("housing_to_door")
    tray_joint = object_model.get_articulation("housing_to_crumb_tray")

    ctx.expect_gap(
        housing,
        door,
        axis="x",
        max_penetration=0.001,
        max_gap=0.004,
        positive_elem="front_sill",
        negative_elem="door_frame",
        name="closed door sits just in front of the housing face",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="yz",
        min_overlap=0.18,
        elem_a="door_frame",
        name="door frame covers the oven mouth footprint",
    )
    ctx.expect_within(
        tray,
        housing,
        axes="yz",
        margin=0.006,
        inner_elem="tray_pan",
        name="crumb tray pan stays within the under-opening slot envelope",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.45}):
        dropped_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door drops outward on its bottom hinge",
        closed_door_aabb is not None
        and dropped_door_aabb is not None
        and dropped_door_aabb[0][0] < closed_door_aabb[0][0] - 0.12
        and dropped_door_aabb[1][2] < closed_door_aabb[1][2] - 0.05,
        details=f"closed={closed_door_aabb}, dropped={dropped_door_aabb}",
    )

    closed_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_joint: 0.16}):
        pulled_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            housing,
            axes="x",
            min_overlap=0.08,
            elem_a="tray_pan",
            elem_b="cavity_floor",
            name="pulled crumb tray retains insertion under the oven",
        )
    ctx.check(
        "crumb tray slides out through the front",
        closed_tray_pos is not None
        and pulled_tray_pos is not None
        and pulled_tray_pos[0] < closed_tray_pos[0] - 0.12,
        details=f"closed={closed_tray_pos}, pulled={pulled_tray_pos}",
    )

    return ctx.report()


object_model = build_object_model()
