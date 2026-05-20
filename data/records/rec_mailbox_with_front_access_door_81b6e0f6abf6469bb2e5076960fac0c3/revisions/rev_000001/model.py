from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_mailbox")

    painted_metal = Material("dark_green_painted_metal", color=(0.05, 0.20, 0.15, 1.0))
    darker_metal = Material("shadowed_green_metal", color=(0.025, 0.10, 0.08, 1.0))
    hinge_metal = Material("blackened_steel", color=(0.015, 0.015, 0.014, 1.0))
    screw_metal = Material("dull_screw_heads", color=(0.25, 0.25, 0.23, 1.0))

    body_width = 0.36
    body_depth = 0.13
    body_height = 0.30
    sheet = 0.012

    door_width = 0.27
    door_height = 0.23
    door_thickness = 0.014
    hinge_radius = 0.008
    hinge_x = body_depth + door_thickness / 2.0 + 0.001
    hinge_z = 0.043

    housing = model.part("housing")

    # Flat wall-mounting back panel and the folded box body surrounding an open front.
    housing.visual(
        Box((sheet, body_width, body_height)),
        origin=Origin(xyz=(sheet / 2.0, 0.0, body_height / 2.0)),
        material=painted_metal,
        name="back_panel",
    )
    housing.visual(
        Box((body_depth, sheet, body_height)),
        origin=Origin(xyz=(body_depth / 2.0, body_width / 2.0 - sheet / 2.0, body_height / 2.0)),
        material=painted_metal,
        name="side_wall_0",
    )
    housing.visual(
        Box((body_depth, sheet, body_height)),
        origin=Origin(xyz=(body_depth / 2.0, -body_width / 2.0 + sheet / 2.0, body_height / 2.0)),
        material=painted_metal,
        name="side_wall_1",
    )
    housing.visual(
        Box((body_depth, body_width, sheet)),
        origin=Origin(xyz=(body_depth / 2.0, 0.0, sheet / 2.0)),
        material=painted_metal,
        name="bottom_tray",
    )
    housing.visual(
        Box((0.020, body_width, hinge_z)),
        origin=Origin(xyz=(body_depth - 0.010, 0.0, hinge_z / 2.0)),
        material=darker_metal,
        name="front_sill",
    )
    housing.visual(
        Box((0.020, 0.030, body_height)),
        origin=Origin(xyz=(body_depth - 0.010, body_width / 2.0 - 0.015, body_height / 2.0)),
        material=painted_metal,
        name="front_jamb_0",
    )
    housing.visual(
        Box((0.020, 0.030, body_height)),
        origin=Origin(xyz=(body_depth - 0.010, -body_width / 2.0 + 0.015, body_height / 2.0)),
        material=painted_metal,
        name="front_jamb_1",
    )
    housing.visual(
        Box((0.020, body_width, 0.025)),
        origin=Origin(xyz=(body_depth - 0.010, 0.0, body_height - 0.0125)),
        material=painted_metal,
        name="front_lintel",
    )
    housing.visual(
        Box((body_depth + 0.050, body_width + 0.045, 0.018)),
        origin=Origin(xyz=(body_depth / 2.0 + 0.020, 0.0, body_height + 0.009)),
        material=painted_metal,
        name="top_overhang",
    )

    # Inside screw heads make the flat back panel read as a wall-mounting plate.
    for z, name in ((0.095, "mount_screw_0"), (0.225, "mount_screw_1")):
        housing.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(sheet + 0.002, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=screw_metal,
            name=name,
        )

    # Stationary end knuckles and short leaves for a bottom piano-like hinge.
    for y, name in ((0.151, "hinge_leaf_0"), (-0.151, "hinge_leaf_1")):
        housing.visual(
            Box((0.018, 0.042, 0.014)),
            origin=Origin(xyz=(body_depth - 0.009, y, hinge_z - 0.004)),
            material=hinge_metal,
            name=name,
        )
    for y, name in ((0.151, "hinge_knuckle_0"), (-0.151, "hinge_knuckle_1")):
        housing.visual(
            Cylinder(radius=hinge_radius, length=0.040),
            origin=Origin(xyz=(hinge_x, y, hinge_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=name,
        )

    door = model.part("front_door")
    # The door part frame is on the bottom hinge axis.  In the closed pose the
    # panel rises along local +Z and forms most of the front face.
    door.visual(
        Box((door_thickness, door_width, door_height)),
        origin=Origin(xyz=(0.0, 0.0, door_height / 2.0)),
        material=painted_metal,
        name="door_panel",
    )
    door.visual(
        Cylinder(radius=hinge_radius, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="door_hinge_barrel",
    )
    door.visual(
        Box((0.010, door_width - 0.018, 0.012)),
        origin=Origin(xyz=(0.002, 0.0, 0.010)),
        material=hinge_metal,
        name="lower_hinge_leaf",
    )
    door.visual(
        Box((0.022, 0.115, 0.020)),
        origin=Origin(xyz=(door_thickness / 2.0 + 0.007, 0.0, door_height - 0.045)),
        material=hinge_metal,
        name="pull_handle",
    )
    door.visual(
        Box((0.006, 0.145, 0.032)),
        origin=Origin(xyz=(door_thickness / 2.0 + 0.003, 0.0, door_height - 0.095)),
        material=darker_metal,
        name="mail_label_plate",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=math.radians(80.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("front_door")
    hinge = object_model.get_articulation("door_hinge")

    upper = hinge.motion_limits.upper if hinge.motion_limits else None
    ctx.check(
        "front flap opens about 80 degrees",
        upper is not None and abs(upper - math.radians(80.0)) < math.radians(0.5),
        details=f"upper_limit={upper}",
    )
    ctx.expect_gap(
        door,
        housing,
        axis="x",
        min_gap=0.0005,
        max_gap=0.003,
        positive_elem="door_panel",
        negative_elem="front_sill",
        name="closed door sits just proud of front sill",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="yz",
        min_overlap=0.20,
        elem_a="door_panel",
        elem_b="back_panel",
        name="door covers most of rectangular front",
    )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: math.radians(80.0)}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_panel")

    ctx.check(
        "front flap swings outward and downward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.17
        and open_aabb[1][2] < closed_aabb[1][2] - 0.14,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
