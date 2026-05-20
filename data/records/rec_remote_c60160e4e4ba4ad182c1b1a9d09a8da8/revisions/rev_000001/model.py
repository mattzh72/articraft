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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motion_sensing_gaming_remote")

    shell_white = Material("satin_white_plastic", rgba=(0.88, 0.90, 0.88, 1.0))
    seam_grey = Material("soft_grey_plastic", rgba=(0.42, 0.44, 0.44, 1.0))
    rubber_dark = Material("dark_rubber", rgba=(0.035, 0.038, 0.04, 1.0))
    black_gloss = Material("gloss_black_lens", rgba=(0.005, 0.006, 0.008, 1.0))
    led_blue = Material("pale_blue_led", rgba=(0.18, 0.55, 1.0, 0.85))

    length = 0.165
    width = 0.038
    thickness = 0.018
    half_width = width / 2.0
    bottom_z = -thickness / 2.0

    body = model.part("body")

    body_shell = (
        cq.Workplane("XY")
        .box(length, width, thickness)
        .edges()
        .fillet(0.003)
    )
    body.visual(
        mesh_from_cadquery(body_shell, "body_shell", tolerance=0.0006),
        material=shell_white,
        name="body_shell",
    )

    # Front optical window and status LEDs make the slim white body read as a
    # motion-sensing game remote without adding unrequested push-button joints.
    body.visual(
        Box((0.0012, 0.024, 0.007)),
        origin=Origin(xyz=(length / 2.0 + 0.0001, 0.0, 0.002)),
        material=black_gloss,
        name="front_sensor_window",
    )
    for i, y in enumerate((-0.0045, -0.0015, 0.0015, 0.0045)):
        body.visual(
            Box((0.0030, 0.0014, 0.0006)),
            origin=Origin(xyz=(0.045, y, thickness / 2.0 + 0.00015)),
            material=led_blue,
            name=f"status_led_{i}",
        )

    # Side saddle that the rotating navigation nub bears against.
    joystick_x = -0.022
    socket_proud = 0.0026
    socket_y = half_width + socket_proud / 2.0 - 0.0002
    body.visual(
        Box((0.026, socket_proud + 0.0004, 0.017)),
        origin=Origin(xyz=(joystick_x, socket_y, 0.0)),
        material=seam_grey,
        name="side_socket",
    )

    # Underside seam strips frame the separate battery compartment door.
    hinge_x = -0.054
    door_length = 0.082
    door_width = 0.030
    door_thickness = 0.0024
    door_gap = 0.004
    rim_z = bottom_z - 0.00025
    rim_x_center = hinge_x + door_gap + door_length / 2.0
    for i, y in enumerate((-door_width / 2.0 - 0.0012, door_width / 2.0 + 0.0012)):
        body.visual(
            Box((door_length + 0.004, 0.0008, 0.00055)),
            origin=Origin(xyz=(rim_x_center, y, rim_z)),
            material=seam_grey,
            name=f"battery_side_seam_{i}",
        )
    body.visual(
        Box((0.0008, door_width + 0.003, 0.00055)),
        origin=Origin(xyz=(hinge_x + door_gap + door_length + 0.0014, 0.0, rim_z)),
        material=seam_grey,
        name="battery_latch_seam",
    )

    # Stationary central hinge knuckle and web, mounted to the underside.
    hinge_axis_z = bottom_z - 0.0025
    body.visual(
        Box((0.008, 0.011, 0.0023)),
        origin=Origin(xyz=(hinge_x, 0.0, bottom_z - 0.0011)),
        material=shell_white,
        name="body_hinge_web",
    )
    body.visual(
        Cylinder(radius=0.0022, length=0.010),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_axis_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=shell_white,
        name="body_hinge_knuckle",
    )

    joystick = model.part("joystick_nub")
    nub_diameter = 0.016
    nub_height = 0.016
    nub_mesh = mesh_from_geometry(
        KnobGeometry(
            nub_diameter,
            nub_height,
            body_style="domed",
            edge_radius=0.0007,
            grip=KnobGrip(style="ribbed", count=18, depth=0.00045, width=0.0009),
            center=True,
        ),
        "joystick_nub",
    )
    joystick.visual(nub_mesh, material=rubber_dark, name="nub_cap")
    joystick.visual(
        Box((0.0035, 0.0012, 0.012)),
        origin=Origin(xyz=(0.0, nub_diameter / 2.0 + 0.00045, 0.0002)),
        material=seam_grey,
        name="nub_index_ridge",
    )
    model.articulation(
        "body_to_joystick",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=joystick,
        origin=Origin(xyz=(joystick_x, half_width + socket_proud + nub_diameter / 2.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.35, velocity=8.0),
    )

    door = model.part("battery_door")
    door_panel_shape = (
        cq.Workplane("XY")
        .box(door_length, door_width, door_thickness)
        .edges("|Z")
        .fillet(0.002)
    )
    axis_to_door_top = bottom_z - hinge_axis_z
    door.visual(
        mesh_from_cadquery(door_panel_shape, "battery_door_panel", tolerance=0.0005),
        origin=Origin(xyz=(door_gap + door_length / 2.0, 0.0, axis_to_door_top - door_thickness / 2.0)),
        material=shell_white,
        name="door_panel",
    )
    door.visual(
        Box((0.018, 0.004, 0.0007)),
        origin=Origin(xyz=(door_gap + door_length - 0.010, 0.0, axis_to_door_top - door_thickness - 0.00025)),
        material=seam_grey,
        name="finger_grip",
    )
    for i, y in enumerate((-0.0105, 0.0105)):
        door.visual(
            Box((0.009, 0.0066, door_thickness)),
            origin=Origin(xyz=(0.0045, y, axis_to_door_top - door_thickness / 2.0)),
            material=shell_white,
            name=f"door_hinge_tab_{i}",
        )
        door.visual(
            Cylinder(radius=0.0022, length=0.008),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=shell_white,
            name=f"door_hinge_knuckle_{i}",
        )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_axis_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    joystick = object_model.get_part("joystick_nub")
    door = object_model.get_part("battery_door")
    joystick_joint = object_model.get_articulation("body_to_joystick")
    door_joint = object_model.get_articulation("body_to_battery_door")

    ctx.check(
        "joystick joint is continuous",
        joystick_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={joystick_joint.articulation_type}",
    )
    ctx.check(
        "joystick rotates about a vertical axis",
        tuple(round(v, 6) for v in joystick_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={joystick_joint.axis}",
    )
    ctx.expect_contact(
        joystick,
        body,
        elem_a="nub_cap",
        elem_b="side_socket",
        contact_tol=0.001,
        name="side joystick is mounted against its socket",
    )

    ctx.check(
        "battery door uses a revolute hinge",
        door_joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={door_joint.articulation_type}",
    )
    ctx.expect_gap(
        body,
        door,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="body_shell",
        negative_elem="door_panel",
        name="closed battery door sits flush on the base",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xy",
        elem_a="door_panel",
        elem_b="body_shell",
        min_overlap=0.025,
        name="battery door lies within the underside footprint",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.25}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "battery door flips downward from its edge hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.045,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
