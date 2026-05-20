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


# Real countertop combination microwave proportions, in meters.
WIDTH = 0.60
DEPTH = 0.45
HEIGHT = 0.34
FRONT_Y = -DEPTH / 2.0

CAVITY_X0 = -0.265
CAVITY_X1 = 0.160
CAVITY_Z0 = 0.065
CAVITY_Z1 = 0.300
CAVITY_BACK_Y = DEPTH / 2.0 - 0.035
CAVITY_CENTER_X = (CAVITY_X0 + CAVITY_X1) / 2.0
CAVITY_CENTER_Y = (FRONT_Y + CAVITY_BACK_Y) / 2.0

DOOR_WIDTH = 0.470
DOOR_HEIGHT = 0.285
DOOR_THICKNESS = 0.030
DOOR_BOTTOM_Z = 0.028
HINGE_X = -0.285
HINGE_Y = FRONT_Y - DOOR_THICKNESS / 2.0 - 0.002

TURNTABLE_X = CAVITY_CENTER_X
TURNTABLE_Y = -0.030
TURNTABLE_Z = 0.078


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _housing_shell() -> cq.Workplane:
    """One continuous hollow carcass with a front oven opening."""
    outer = _box((WIDTH, DEPTH, HEIGHT), (0.0, 0.0, HEIGHT / 2.0))
    cavity = _box(
        (
            CAVITY_X1 - CAVITY_X0,
            CAVITY_BACK_Y - FRONT_Y + 0.020,
            CAVITY_Z1 - CAVITY_Z0,
        ),
        (
            CAVITY_CENTER_X,
            (FRONT_Y - 0.010 + CAVITY_BACK_Y) / 2.0,
            (CAVITY_Z0 + CAVITY_Z1) / 2.0,
        ),
    )
    return outer.cut(cavity)


def _door_frame() -> cq.Workplane:
    """Door slab in its hinge-line local frame, with a real window opening."""
    frame_x0 = 0.012
    frame_width = DOOR_WIDTH - frame_x0
    frame = _box(
        (frame_width, DOOR_THICKNESS, DOOR_HEIGHT),
        (frame_x0 + frame_width / 2.0, 0.0, DOOR_HEIGHT / 2.0),
    )
    window = _box((0.310, DOOR_THICKNESS * 3.0, 0.170), (0.210, 0.0, 0.155))
    return frame.cut(window)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="combination_microwave_oven")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_liner = model.material("dark_enamel_liner", rgba=(0.035, 0.038, 0.040, 1.0))
    black_glass = model.material("smoked_black_glass", rgba=(0.02, 0.025, 0.030, 0.58))
    glass = model.material("pale_glass", rgba=(0.70, 0.86, 0.95, 0.38))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    plastic = model.material("satin_black_plastic", rgba=(0.03, 0.032, 0.035, 1.0))
    led_blue = model.material("cool_display_blue", rgba=(0.08, 0.30, 0.65, 1.0))
    metal = model.material("hinge_pin_metal", rgba=(0.50, 0.51, 0.49, 1.0))

    body = model.part("housing")
    body.visual(
        mesh_from_cadquery(_housing_shell(), "housing_shell", tolerance=0.0015),
        material=stainless,
        name="shell",
    )
    body.visual(
        Box((CAVITY_X1 - CAVITY_X0, CAVITY_BACK_Y - FRONT_Y, 0.004)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, CAVITY_CENTER_Y, CAVITY_Z0 + 0.002)
        ),
        material=dark_liner,
        name="cavity_floor",
    )
    body.visual(
        Box((CAVITY_X1 - CAVITY_X0, 0.004, CAVITY_Z1 - CAVITY_Z0)),
        origin=Origin(
            xyz=(CAVITY_CENTER_X, CAVITY_BACK_Y - 0.002, (CAVITY_Z0 + CAVITY_Z1) / 2.0)
        ),
        material=dark_liner,
        name="cavity_back",
    )
    body.visual(
        Box((0.096, 0.007, 0.255)),
        origin=Origin(xyz=(0.240, FRONT_Y - 0.0035, 0.178)),
        material=plastic,
        name="control_panel",
    )
    body.visual(
        Box((0.070, 0.003, 0.032)),
        origin=Origin(xyz=(0.240, FRONT_Y - 0.0085, 0.275)),
        material=led_blue,
        name="display",
    )
    body.visual(
        Box((0.026, 0.018, 0.095)),
        origin=Origin(xyz=(HINGE_X - 0.022, HINGE_Y + 0.010, DOOR_BOTTOM_Z + 0.145)),
        material=metal,
        name="hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_BOTTOM_Z + 0.145)),
        material=metal,
        name="hinge_barrel",
    )
    body.visual(
        Cylinder(radius=0.004, length=DOOR_HEIGHT),
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_BOTTOM_Z + DOOR_HEIGHT / 2.0)),
        material=metal,
        name="hinge_pin",
    )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_door_frame(), "door_frame", tolerance=0.001),
        material=rubber,
        name="door_frame",
    )
    door.visual(
        Box((0.332, 0.006, 0.192)),
        origin=Origin(xyz=(0.210, -0.004, 0.155)),
        material=black_glass,
        name="window_glass",
    )
    door.visual(
        Box((0.026, 0.018, 0.195)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.035, -0.044, DOOR_HEIGHT / 2.0)),
        material=stainless,
        name="handle",
    )
    for z in (0.045, 0.245):
        door.visual(
            Box((0.040, 0.032, 0.026)),
            origin=Origin(xyz=(DOOR_WIDTH - 0.040, -0.025, z)),
            material=stainless,
            name=f"handle_standoff_{z:.3f}",
        )
    for z in (0.045, 0.245):
        door.visual(
            Box((0.024, 0.020, 0.050)),
            origin=Origin(xyz=(0.018, 0.0, z)),
            material=metal,
            name=f"door_hinge_leaf_{z:.3f}",
        )
        door.visual(
            Cylinder(radius=0.010, length=0.062),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=metal,
            name=f"door_hinge_barrel_{z:.3f}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.158, length=0.007),
        origin=Origin(),
        material=glass,
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.0065)),
        material=glass,
        name="center_hub",
    )
    turntable.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.112, 0.0, 0.0045)),
        material=dark_liner,
        name="index_dot",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.003, 0.033)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=stainless,
        name="dial_indicator",
    )

    button_positions = [
        (0.220, 0.218),
        (0.260, 0.218),
        (0.220, 0.184),
        (0.260, 0.184),
    ]
    buttons = []
    for i, (x, z) in enumerate(button_positions):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.030, 0.007, 0.020)),
            origin=Origin(),
            material=plastic,
            name="button_cap",
        )
        buttons.append((button, x, z))

    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, DOOR_BOTTOM_Z)),
        # The closed door extends along local +X; -Z swings the free edge out
        # toward the viewer at the negative-Y front of the appliance.
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.75, effort=18.0, velocity=1.2),
    )
    model.articulation(
        "housing_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(TURNTABLE_X, TURNTABLE_Y, TURNTABLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=3.0),
    )
    model.articulation(
        "housing_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.240, FRONT_Y - 0.016, 0.118)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=0.4, velocity=3.0),
    )
    for i, (button, x, z) in enumerate(buttons):
        model.articulation(
            f"housing_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, FRONT_Y - 0.0105, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=0.004, effort=1.5, velocity=0.08),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("housing")
    door = object_model.get_part("door")
    turntable = object_model.get_part("turntable")
    hinge = object_model.get_articulation("housing_to_door")
    bearing = object_model.get_articulation("housing_to_turntable")

    for barrel_name in ("door_hinge_barrel_0.045", "door_hinge_barrel_0.245"):
        ctx.allow_overlap(
            body,
            door,
            elem_a="hinge_pin",
            elem_b=barrel_name,
            reason="The fixed metal hinge pin is intentionally captured inside the door hinge knuckle.",
        )
        ctx.expect_within(
            body,
            door,
            axes="xy",
            inner_elem="hinge_pin",
            outer_elem=barrel_name,
            margin=0.0005,
            name=f"hinge pin stays centered in {barrel_name}",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="z",
            elem_a="hinge_pin",
            elem_b=barrel_name,
            min_overlap=0.045,
            name=f"hinge pin is retained through {barrel_name}",
        )

    ctx.check(
        "door hinge is vertical revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in hinge.axis) == (0.0, 0.0, -1.0),
        details=f"type={hinge.articulation_type}, axis={hinge.axis}",
    )
    ctx.check(
        "turntable bearing is continuous vertical",
        bearing.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in bearing.axis) == (0.0, 0.0, 1.0),
        details=f"type={bearing.articulation_type}, axis={bearing.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="shell",
            negative_elem="door_frame",
            min_gap=0.0005,
            max_gap=0.006,
            name="closed door sits just proud of the front frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_frame",
            elem_b="cavity_back",
            min_overlap=0.12,
            name="closed door spans the oven opening footprint",
        )

    rest_handle = ctx.part_element_world_aabb(door, elem="handle")
    with ctx.pose({hinge: 1.55}):
        open_handle = ctx.part_element_world_aabb(door, elem="handle")
    ctx.check(
        "door swings outward from the left hinge",
        rest_handle is not None
        and open_handle is not None
        and open_handle[0][1] < rest_handle[0][1] - 0.25,
        details=f"rest={rest_handle}, open={open_handle}",
    )

    rest_dot = ctx.part_element_world_aabb(turntable, elem="index_dot")
    with ctx.pose({bearing: math.pi / 2.0}):
        turned_dot = ctx.part_element_world_aabb(turntable, elem="index_dot")
    ctx.check(
        "turntable visibly rotates around its center bearing",
        rest_dot is not None
        and turned_dot is not None
        and turned_dot[0][0] < rest_dot[0][0] - 0.07
        and turned_dot[0][1] > rest_dot[0][1] + 0.07,
        details=f"rest={rest_dot}, turned={turned_dot}",
    )

    return ctx.report()


object_model = build_object_model()
