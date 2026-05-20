from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.150
BODY_WIDTH = 0.058
BODY_HEIGHT = 0.065
BODY_CENTER_Z = 0.052
BODY_TOP_Z = BODY_CENTER_Z + BODY_HEIGHT * 0.5
LENS_AXIS_Z = BODY_CENTER_Z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="palm_camcorder")

    shell_black = model.material("shell_black", rgba=(0.055, 0.058, 0.062, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.016, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.01, 0.016, 0.022, 1.0))
    blue_glass = model.material("blue_glass", rgba=(0.08, 0.20, 0.34, 0.92))
    lens_glass = model.material("lens_glass", rgba=(0.025, 0.060, 0.095, 0.82))
    white_mark = model.material("white_mark", rgba=(0.86, 0.88, 0.86, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.35, 0.36, 0.38, 1.0))

    body = model.part("body")

    rounded_body = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .edges()
        .fillet(0.008)
    )
    body.visual(
        mesh_from_cadquery(
            rounded_body,
            "rounded_body_shell",
            tolerance=0.0006,
            angular_tolerance=0.12,
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_CENTER_Z)),
        material=shell_black,
        name="body_shell",
    )

    # Front lens barrel stack: a short fixed tube with a rotating focus/zoom ring.
    lens_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    body.visual(
        Cylinder(radius=0.024, length=0.012),
        origin=Origin(xyz=(0.078, 0.0, LENS_AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_graphite,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.0176, length=0.066),
        origin=Origin(xyz=(0.105, 0.0, LENS_AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_glass,
        name="lens_core",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.026),
        origin=Origin(xyz=(0.127, 0.0, LENS_AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_graphite,
        name="front_barrel",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.003),
        origin=Origin(xyz=(0.141, 0.0, LENS_AXIS_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )

    # Rounded top handgrip and supporting posts, sized for a palm camcorder.
    body.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(-0.023, 0.0, BODY_TOP_Z + 0.012)),
        material=satin_graphite,
        name="rear_handle_post",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.026),
        origin=Origin(xyz=(0.047, 0.0, BODY_TOP_Z + 0.012)),
        material=satin_graphite,
        name="front_handle_post",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.012, 0.0, BODY_TOP_Z + 0.028), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="rounded_handgrip",
    )
    body.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(-0.033, 0.0, BODY_TOP_Z + 0.028)),
        material=rubber,
        name="rear_grip_cap",
    )
    body.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.057, 0.0, BODY_TOP_Z + 0.028)),
        material=rubber,
        name="front_grip_cap",
    )

    # Distinct rear/side details that read as a handheld camcorder shell.
    body.visual(
        Box((0.014, 0.028, 0.020)),
        origin=Origin(xyz=(-0.079, 0.0, BODY_CENTER_Z + 0.006)),
        material=dark_glass,
        name="rear_eyepiece",
    )
    body.visual(
        Box((0.060, 0.003, 0.035)),
        origin=Origin(xyz=(-0.015, -0.0305, BODY_CENTER_Z - 0.002)),
        material=rubber,
        name="side_grip_pad",
    )

    # Visible monitor hinge block on the side shell, with fixed top/bottom knuckles
    # and a central pin that captures the moving monitor knuckle.
    hinge_x = -0.034
    hinge_y = BODY_WIDTH * 0.5 + 0.014
    hinge_z = BODY_CENTER_Z
    body.visual(
        Box((0.018, 0.014, 0.060)),
        origin=Origin(xyz=(hinge_x, BODY_WIDTH * 0.5 + 0.007, hinge_z)),
        material=satin_graphite,
        name="monitor_hinge_block",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z - 0.023)),
        material=hinge_metal,
        name="lower_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=0.005, length=0.014),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z + 0.023)),
        material=hinge_metal,
        name="upper_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=0.002, length=0.060),
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        material=hinge_metal,
        name="monitor_hinge_pin",
    )

    lens_ring = model.part("lens_ring")
    ring_shell = LatheGeometry.from_shell_profiles(
        [
            (0.0220, -0.010),
            (0.0242, -0.007),
            (0.0242, 0.007),
            (0.0220, 0.010),
        ],
        [
            (0.0182, -0.010),
            (0.0182, 0.010),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    ).rotate_y(pi / 2.0)
    lens_ring.visual(
        mesh_from_geometry(ring_shell, "lens_ring_shell"),
        material=rubber,
        name="ring_shell",
    )
    lens_ring.visual(
        Box((0.011, 0.004, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.0244)),
        material=white_mark,
        name="ring_index_mark",
    )

    mode_dial = model.part("mode_dial")
    dial_geom = KnobGeometry(
        0.026,
        0.008,
        body_style="faceted",
        base_diameter=0.028,
        top_diameter=0.023,
        edge_radius=0.0008,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0007, width=0.0014),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    mode_dial.visual(
        mesh_from_geometry(dial_geom, "top_mode_dial"),
        material=satin_graphite,
        name="dial_cap",
    )

    side_monitor = model.part("side_monitor")
    side_monitor.visual(
        Cylinder(radius=0.005, length=0.026),
        material=hinge_metal,
        name="monitor_hinge_knuckle",
    )
    side_monitor.visual(
        Box((0.012, 0.008, 0.022)),
        origin=Origin(xyz=(-0.007, 0.006, 0.0)),
        material=hinge_metal,
        name="monitor_hinge_arm",
    )
    side_monitor.visual(
        Box((0.075, 0.006, 0.050)),
        origin=Origin(xyz=(-0.045, 0.011, 0.0)),
        material=shell_black,
        name="monitor_bezel",
    )
    side_monitor.visual(
        Box((0.062, 0.0018, 0.036)),
        origin=Origin(xyz=(-0.045, 0.0146, 0.0)),
        material=blue_glass,
        name="display_glass",
    )
    side_monitor.visual(
        Box((0.068, 0.0015, 0.042)),
        origin=Origin(xyz=(-0.045, 0.007, 0.0)),
        material=dark_glass,
        name="monitor_back",
    )

    model.articulation(
        "lens_ring_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.105, 0.0, LENS_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )
    model.articulation(
        "mode_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.057, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )
    model.articulation(
        "monitor_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=side_monitor,
        origin=Origin(xyz=(hinge_x, hinge_y, hinge_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=1.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")
    side_monitor = object_model.get_part("side_monitor")
    monitor_hinge = object_model.get_articulation("monitor_hinge")

    ctx.allow_overlap(
        body,
        side_monitor,
        elem_a="monitor_hinge_pin",
        elem_b="monitor_hinge_knuckle",
        reason="The visible side-monitor hinge pin is intentionally captured inside the rotating monitor knuckle.",
    )
    ctx.expect_within(
        body,
        side_monitor,
        axes="xy",
        inner_elem="monitor_hinge_pin",
        outer_elem="monitor_hinge_knuckle",
        margin=0.0005,
        name="monitor hinge pin is centered in the moving knuckle",
    )
    ctx.expect_overlap(
        body,
        side_monitor,
        axes="z",
        elem_a="monitor_hinge_pin",
        elem_b="monitor_hinge_knuckle",
        min_overlap=0.020,
        name="monitor hinge pin passes through the moving knuckle",
    )

    ctx.expect_within(
        body,
        lens_ring,
        axes="yz",
        inner_elem="lens_core",
        outer_elem="ring_shell",
        margin=0.001,
        name="lens ring is concentric around the lens barrel",
    )
    ctx.expect_gap(
        lens_ring,
        body,
        axis="x",
        positive_elem="ring_shell",
        negative_elem="lens_mount",
        min_gap=0.002,
        name="rotating lens ring clears the rear lens mount",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="body_shell",
        max_gap=0.001,
        max_penetration=0.0005,
        name="mode dial sits on the top shell",
    )
    ctx.expect_gap(
        side_monitor,
        body,
        axis="y",
        positive_elem="monitor_bezel",
        negative_elem="monitor_hinge_block",
        min_gap=0.001,
        name="folded monitor is outside the visible hinge block",
    )

    closed_aabb = ctx.part_world_aabb(side_monitor)
    with ctx.pose({monitor_hinge: 1.45}):
        open_aabb = ctx.part_world_aabb(side_monitor)
        ctx.expect_gap(
            side_monitor,
            body,
            axis="y",
            positive_elem="monitor_bezel",
            negative_elem="body_shell",
            min_gap=0.010,
            name="opened monitor swings outward from the side shell",
        )

    if closed_aabb is not None and open_aabb is not None:
        closed_y = (closed_aabb[0][1] + closed_aabb[1][1]) * 0.5
        open_y = (open_aabb[0][1] + open_aabb[1][1]) * 0.5
        ctx.check(
            "monitor center moves outward on vertical hinge",
            open_y > closed_y + 0.020,
            details=f"closed_y={closed_y:.4f}, open_y={open_y:.4f}",
        )
    else:
        ctx.fail("monitor center moves outward on vertical hinge", "monitor AABBs were unavailable")

    return ctx.report()


object_model = build_object_model()
