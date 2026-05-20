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


def _annular_x_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
    center_y: float = 0.0,
    center_z: float = 0.0,
):
    """Build a centered annular puck whose axis is local +X."""
    shape = (
        cq.Workplane("YZ")
        .center(center_y, center_z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness / 2.0, both=True)
    )
    return mesh_from_cadquery(shape, name, tolerance=0.0015, angular_tolerance=0.12)


def _cabinet_shell_mesh():
    depth = 0.48
    width = 0.56
    height = 0.72
    front_opening_radius = 0.226

    cabinet = cq.Workplane("XY").box(depth, width, height)
    # Soften the white appliance silhouette without changing the basic slim box.
    cabinet = cabinet.edges("|Z").fillet(0.018)
    cabinet = cabinet.edges(">Z").fillet(0.010)
    cabinet = cabinet.edges("<Z").fillet(0.006)

    drum_tunnel = (
        cq.Workplane("YZ")
        .circle(front_opening_radius)
        .extrude(depth + 0.08, both=True)
    )
    cabinet = cabinet.cut(drum_tunnel)

    # A shallow black control/display recess in the upper fascia.
    display_recess = cq.Workplane("XY").box(0.035, 0.19, 0.055).translate(
        (0.238, -0.11, 0.245)
    )
    cabinet = cabinet.cut(display_recess)
    return mesh_from_cadquery(
        cabinet, "cabinet_shell", tolerance=0.002, angular_tolerance=0.12
    )


def _drum_mesh():
    shell = (
        cq.Workplane("YZ")
        .circle(0.187)
        .circle(0.166)
        .extrude(0.285 / 2.0, both=True)
    )
    back = cq.Workplane("YZ").circle(0.166).extrude(0.010).translate((-0.148, 0, 0))
    drum = shell.union(back)

    # Three low internal lifters make the rotating drum read as a real washer drum.
    for angle in (0.0, 120.0, 240.0):
        y = 0.176 * math.cos(math.radians(angle))
        z = 0.176 * math.sin(math.radians(angle))
        lifter = (
            cq.Workplane("XY")
            .box(0.235, 0.030, 0.040)
            .translate((0.0, y, z))
            .rotate((0, 0, 0), (1, 0, 0), angle)
        )
        drum = drum.union(lifter)
    return mesh_from_cadquery(drum, "drum_shell", tolerance=0.0015, angular_tolerance=0.10)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_front_loader")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.012, 0.012, 1.0))
    dark = model.material("dark_display", rgba=(0.02, 0.025, 0.035, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.73, 0.70, 1.0))
    chrome = model.material("chrome", rgba=(0.86, 0.87, 0.88, 1.0))
    glass = model.material("smoked_glass", rgba=(0.18, 0.30, 0.42, 0.38))

    body = model.part("body")
    body.visual(_cabinet_shell_mesh(), material=white, name="cabinet_shell")
    body.visual(
        _annular_x_mesh(
            outer_radius=0.226,
            inner_radius=0.176,
            thickness=0.020,
            name="door_gasket",
        ),
        origin=Origin(xyz=(0.242, 0.0, 0.0)),
        material=black,
        name="gasket",
    )
    body.visual(
        Box((0.014, 0.180, 0.048)),
        origin=Origin(xyz=(0.258, -0.110, 0.245)),
        material=dark,
        name="display_window",
    )
    body.visual(
        Box((0.030, 0.135, 0.032)),
        origin=Origin(xyz=(0.252, 0.085, 0.245)),
        material=white,
        name="detergent_drawer",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.024),
        origin=Origin(xyz=(0.250, 0.205, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="dial_socket",
    )
    body.visual(
        Box((0.018, 0.030, 0.350)),
        origin=Origin(xyz=(0.248, -0.260, 0.000)),
        material=white,
        name="hinge_backbone",
    )
    for index, (z, barrel_name) in enumerate(
        ((-0.135, "hinge_barrel_0"), (0.135, "hinge_barrel_1"))
    ):
        body.visual(
            Box((0.035, 0.034, 0.086)),
            origin=Origin(xyz=(0.258, -0.248, z)),
            material=white,
            name=f"hinge_arm_{index}",
        )
        body.visual(
            Cylinder(radius=0.016, length=0.092),
            origin=Origin(xyz=(0.285, -0.248, z)),
            material=chrome,
            name=barrel_name,
        )
    for index, y in enumerate((-0.205, 0.205)):
        body.visual(
            Box((0.080, 0.050, 0.025)),
            origin=Origin(xyz=(-0.155, y, -0.372)),
            material=black,
            name=f"foot_{index}",
        )
    body.visual(
        Cylinder(radius=0.040, length=0.045),
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="rear_bearing",
    )
    body.visual(
        Box((0.030, 0.026, 0.330)),
        origin=Origin(xyz=(-0.180, 0.0, 0.190)),
        material=white,
        name="upper_bearing_strut",
    )
    body.visual(
        Box((0.030, 0.026, 0.330)),
        origin=Origin(xyz=(-0.180, 0.0, -0.190)),
        material=white,
        name="lower_bearing_strut",
    )
    body.visual(
        Box((0.030, 0.330, 0.026)),
        origin=Origin(xyz=(-0.180, 0.190, 0.0)),
        material=white,
        name="side_bearing_strut",
    )

    drum = model.part("drum")
    drum.visual(_drum_mesh(), material=stainless, name="drum_shell")
    drum.visual(
        Cylinder(radius=0.022, length=0.135),
        origin=Origin(xyz=(-0.170, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="axle_shaft",
    )
    drum.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="front_hub",
    )
    for index, angle in enumerate((0.0, 120.0, 240.0)):
        y = 0.105 * math.cos(math.radians(angle))
        z = 0.105 * math.sin(math.radians(angle))
        drum.visual(
            Box((0.018, 0.200, 0.014)),
            origin=Origin(xyz=(0.145, y, z), rpy=(math.radians(angle), 0.0, 0.0)),
            material=stainless,
            name=f"front_spoke_{index}",
        )

    door = model.part("door")
    door.visual(
        _annular_x_mesh(
            outer_radius=0.220,
            inner_radius=0.156,
            thickness=0.045,
            name="door_ring",
            center_y=0.245,
        ),
        material=white,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.162, length=0.018),
        origin=Origin(xyz=(0.012, 0.245, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="door_glass",
    )
    door.visual(
        Cylinder(radius=0.016, length=0.178),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.044, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=white,
        name="hinge_leaf",
    )
    door.visual(
        Cylinder(radius=0.010, length=0.145),
        origin=Origin(xyz=(0.054, 0.425, 0.0)),
        material=chrome,
        name="latch_handle",
    )
    for index, z in enumerate((-0.044, 0.044)):
        door.visual(
            Cylinder(radius=0.006, length=0.045),
            origin=Origin(xyz=(0.033, 0.425, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"handle_post_{index}",
        )

    program_dial = model.part("program_dial")
    program_dial.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white,
        name="dial_cap",
    )
    program_dial.visual(
        Box((0.004, 0.006, 0.026)),
        origin=Origin(xyz=(0.014, 0.0, 0.015)),
        material=dark,
        name="pointer_mark",
    )

    model.articulation(
        "drum_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.020, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.285, -0.248, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=18.0, velocity=1.3),
    )
    model.articulation(
        "dial_axis",
        ArticulationType.REVOLUTE,
        parent=body,
        child=program_dial,
        origin=Origin(xyz=(0.274, 0.205, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=1.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    dial = object_model.get_part("program_dial")
    drum_axle = object_model.get_articulation("drum_axle")
    door_hinge = object_model.get_articulation("door_hinge")

    ctx.allow_overlap(
        body,
        drum,
        elem_a="rear_bearing",
        elem_b="axle_shaft",
        reason=(
            "The rotating drum axle is intentionally seated inside the rear "
            "bearing proxy that carries it."
        ),
    )
    ctx.expect_within(
        drum,
        body,
        axes="yz",
        inner_elem="axle_shaft",
        outer_elem="rear_bearing",
        margin=0.001,
        name="axle is centered in rear bearing",
    )
    ctx.expect_overlap(
        drum,
        body,
        axes="x",
        elem_a="axle_shaft",
        elem_b="rear_bearing",
        min_overlap=0.040,
        name="drum axle remains captured in bearing",
    )

    ctx.expect_gap(
        door,
        body,
        axis="x",
        positive_elem="door_ring",
        negative_elem="gasket",
        min_gap=0.004,
        max_gap=0.018,
        name="closed door sits just proud of gasket",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="hinge_barrel_0",
        contact_tol=0.001,
        name="door barrel bears on lower hinge barrel",
    )
    ctx.expect_contact(
        door,
        body,
        elem_a="hinge_barrel",
        elem_b="hinge_barrel_1",
        contact_tol=0.001,
        name="door barrel bears on upper hinge barrel",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_cap",
        elem_b="dial_socket",
        contact_tol=0.001,
        name="program dial sits on its socket",
    )

    ctx.check(
        "drum axle is front to back",
        tuple(round(v, 3) for v in drum_axle.axis) == (1.0, 0.0, 0.0),
        details=f"axis={drum_axle.axis}",
    )
    ctx.check(
        "door hinge is right-edge vertical",
        tuple(round(v, 3) for v in door_hinge.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_hinge.axis}",
    )

    closed_aabb = ctx.part_world_aabb(door)
    closed_max_x = closed_aabb[1][0] if closed_aabb is not None else None
    with ctx.pose({door_hinge: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
        open_max_x = open_aabb[1][0] if open_aabb is not None else None
    ctx.check(
        "door swings outward when opened",
        closed_max_x is not None
        and open_max_x is not None
        and open_max_x > closed_max_x + 0.09,
        details=f"closed_max_x={closed_max_x}, open_max_x={open_max_x}",
    )

    return ctx.report()


object_model = build_object_model()
