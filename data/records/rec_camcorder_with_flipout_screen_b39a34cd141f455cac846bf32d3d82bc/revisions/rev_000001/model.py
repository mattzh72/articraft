from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _ribbed_tube_geometry(
    *,
    inner_radius: float,
    outer_radius: float,
    length: float,
    radial_segments: int = 96,
    ribs: int = 32,
) -> MeshGeometry:
    """Closed hollow tube centered on local Z, with shallow circumferential grip ribs."""
    geom = MeshGeometry()
    z0 = -length / 2.0
    z1 = length / 2.0
    amp = (outer_radius - inner_radius) * 0.22

    outer0 = []
    outer1 = []
    inner0 = []
    inner1 = []
    for i in range(radial_segments):
        theta = 2.0 * math.pi * i / radial_segments
        # Small alternating peaks make the focus ring read as rubberized/ribbed.
        ridge = 0.5 + 0.5 * math.cos(ribs * theta)
        r_outer = outer_radius + amp * ridge
        x = math.cos(theta)
        y = math.sin(theta)
        outer0.append(geom.add_vertex(r_outer * x, r_outer * y, z0))
        outer1.append(geom.add_vertex(r_outer * x, r_outer * y, z1))
        inner0.append(geom.add_vertex(inner_radius * x, inner_radius * y, z0))
        inner1.append(geom.add_vertex(inner_radius * x, inner_radius * y, z1))

    for i in range(radial_segments):
        j = (i + 1) % radial_segments
        # Outer wall.
        geom.add_face(outer0[i], outer0[j], outer1[j])
        geom.add_face(outer0[i], outer1[j], outer1[i])
        # Inner wall, winding reversed.
        geom.add_face(inner0[i], inner1[j], inner0[j])
        geom.add_face(inner0[i], inner1[i], inner1[j])
        # Rear annular cap.
        geom.add_face(inner0[i], inner0[j], outer0[j])
        geom.add_face(inner0[i], outer0[j], outer0[i])
        # Front annular cap.
        geom.add_face(inner1[i], outer1[j], inner1[j])
        geom.add_face(inner1[i], outer1[i], outer1[j])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prosumer_camcorder")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    charcoal = model.material("charcoal_shell", rgba=(0.075, 0.080, 0.085, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    lens_glass = model.material("coated_lens_glass", rgba=(0.02, 0.055, 0.09, 0.62))
    screen_glass = model.material("blue_black_screen", rgba=(0.01, 0.025, 0.035, 1.0))
    screen_bezel = model.material("screen_bezel", rgba=(0.025, 0.027, 0.030, 1.0))
    silver = model.material("brushed_silver", rgba=(0.45, 0.45, 0.43, 1.0))
    white = model.material("white_mark", rgba=(0.92, 0.92, 0.84, 1.0))

    body = model.part("body")
    # Prosumer handheld-camera scale: a deep body with a shoulder-like rear mass.
    body.visual(
        Box((0.250, 0.120, 0.130)),
        origin=Origin(xyz=(0.000, 0.000, 0.075)),
        material=charcoal,
        name="deep_body_shell",
    )
    body.visual(
        Box((0.145, 0.100, 0.030)),
        origin=Origin(xyz=(-0.030, 0.000, 0.153)),
        material=matte_black,
        name="top_shell_ridge",
    )
    body.visual(
        Box((0.050, 0.085, 0.040)),
        origin=Origin(xyz=(-0.120, 0.000, 0.100)),
        material=dark_rubber,
        name="rear_viewfinder_block",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.020),
        origin=Origin(xyz=(-0.153, 0.000, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="rear_eye_cup",
    )

    # Long lens barrel projects from the front, with stepped housings and glass.
    body.visual(
        Cylinder(radius=0.057, length=0.045),
        origin=Origin(xyz=(0.137, 0.000, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.043, length=0.175),
        origin=Origin(xyz=(0.220, 0.000, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="long_lens_barrel",
    )
    body.visual(
        Cylinder(radius=0.054, length=0.048),
        origin=Origin(xyz=(0.324, 0.000, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="front_lens_hood",
    )
    body.visual(
        Cylinder(radius=0.037, length=0.004),
        origin=Origin(xyz=(0.350, 0.000, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )

    # Top carry handle and supports; the mode dial sits behind it on the top shell.
    body.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(0.060, 0.000, 0.175)),
        material=matte_black,
        name="front_handle_post",
    )
    body.visual(
        Box((0.030, 0.040, 0.060)),
        origin=Origin(xyz=(-0.030, 0.000, 0.175)),
        material=matte_black,
        name="rear_handle_post",
    )
    body.visual(
        Box((0.125, 0.042, 0.026)),
        origin=Origin(xyz=(0.015, 0.000, 0.211)),
        material=matte_black,
        name="top_handle",
    )

    # Side handgrip/strap on the monitor side, forward of the screen.
    body.visual(
        Box((0.085, 0.018, 0.095)),
        origin=Origin(xyz=(0.030, 0.069, 0.080)),
        material=dark_rubber,
        name="side_handgrip",
    )
    body.visual(
        Box((0.016, 0.014, 0.100)),
        origin=Origin(xyz=(0.082, 0.067, 0.080)),
        material=dark_rubber,
        name="grip_front_anchor",
    )
    body.visual(
        Box((0.016, 0.014, 0.100)),
        origin=Origin(xyz=(-0.022, 0.067, 0.080)),
        material=dark_rubber,
        name="grip_rear_anchor",
    )

    # Fixed hinge leaf on the side shell for the flip-out monitor.
    body.visual(
        Cylinder(radius=0.004, length=0.080),
        origin=Origin(xyz=(-0.052, 0.067, 0.082)),
        material=silver,
        name="monitor_hinge_pin",
    )
    body.visual(
        Box((0.016, 0.009, 0.086)),
        origin=Origin(xyz=(-0.052, 0.062, 0.082)),
        material=silver,
        name="monitor_hinge_leaf",
    )

    # A few small controls on the camera body, kept fixed because they are not
    # the prompt's primary articulated controls.
    body.visual(
        Box((0.016, 0.006, 0.010)),
        origin=Origin(xyz=(-0.090, -0.063, 0.137)),
        material=dark_rubber,
        name="rear_button_0",
    )
    body.visual(
        Box((0.016, 0.006, 0.010)),
        origin=Origin(xyz=(-0.064, -0.063, 0.137)),
        material=dark_rubber,
        name="rear_button_1",
    )

    lens_ring = model.part("lens_ring")
    ring_mesh = mesh_from_geometry(
        _ribbed_tube_geometry(inner_radius=0.0430, outer_radius=0.0485, length=0.044),
        "ribbed_lens_ring",
    )
    lens_ring.visual(
        ring_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="ribbed_lens_ring",
    )
    model.articulation(
        "body_to_lens_ring",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.215, 0.000, 0.080)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    monitor = model.part("monitor")
    # The monitor is a real hinged slab with thickness, a hinge barrel and a
    # raised bezel around a recessed screen, not a painted rectangle on the body.
    monitor_barrel_mesh = mesh_from_geometry(
        _ribbed_tube_geometry(
            inner_radius=0.0040,
            outer_radius=0.0067,
            length=0.082,
            radial_segments=64,
            ribs=0,
        ),
        "monitor_barrel",
    )
    monitor.visual(
        monitor_barrel_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=silver,
        name="monitor_barrel",
    )
    monitor.visual(
        Box((0.012, 0.012, 0.074)),
        origin=Origin(xyz=(-0.006, 0.006, 0.000)),
        material=screen_bezel,
        name="monitor_hinge_block",
    )
    monitor.visual(
        Box((0.108, 0.011, 0.072)),
        origin=Origin(xyz=(-0.061, 0.012, 0.000)),
        material=screen_bezel,
        name="monitor_panel_case",
    )
    monitor.visual(
        Box((0.093, 0.003, 0.054)),
        origin=Origin(xyz=(-0.062, 0.0185, 0.000)),
        material=screen_glass,
        name="display_glass",
    )
    monitor.visual(
        Box((0.006, 0.004, 0.064)),
        origin=Origin(xyz=(-0.112, 0.0175, 0.000)),
        material=matte_black,
        name="screen_side_bezel_0",
    )
    monitor.visual(
        Box((0.006, 0.004, 0.064)),
        origin=Origin(xyz=(-0.012, 0.0175, 0.000)),
        material=matte_black,
        name="screen_side_bezel_1",
    )
    monitor.visual(
        Box((0.102, 0.004, 0.005)),
        origin=Origin(xyz=(-0.062, 0.0175, 0.032)),
        material=matte_black,
        name="screen_top_bezel",
    )
    monitor.visual(
        Box((0.102, 0.004, 0.005)),
        origin=Origin(xyz=(-0.062, 0.0175, -0.032)),
        material=matte_black,
        name="screen_bottom_bezel",
    )
    model.articulation(
        "body_to_monitor",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(-0.052, 0.074, 0.082)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    mode_dial = model.part("mode_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.014,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="ribbed", count=22, depth=0.0010, width=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "top_mode_dial",
    )
    mode_dial.visual(
        dial_mesh,
        material=dark_rubber,
        name="top_mode_dial",
    )
    mode_dial.visual(
        Box((0.006, 0.020, 0.002)),
        origin=Origin(xyz=(0.000, 0.014, 0.015)),
        material=white,
        name="dial_index_mark",
    )
    model.articulation(
        "body_to_mode_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.082, 0.000, 0.168)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.2, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    monitor = object_model.get_part("monitor")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")
    monitor_hinge = object_model.get_articulation("body_to_monitor")
    ring_joint = object_model.get_articulation("body_to_lens_ring")
    dial_joint = object_model.get_articulation("body_to_mode_dial")

    ctx.allow_overlap(
        body,
        lens_ring,
        elem_a="long_lens_barrel",
        elem_b="ribbed_lens_ring",
        reason=(
            "The rotating focus ring is intentionally captured around the fixed "
            "lens barrel; the simplified exact collision proxy treats the hidden "
            "annular bearing fit as interpenetration."
        ),
    )
    ctx.expect_contact(
        mode_dial,
        body,
        elem_a="top_mode_dial",
        elem_b="top_shell_ridge",
        contact_tol=0.002,
        name="mode dial sits on top shell behind handle",
    )
    ctx.expect_within(
        lens_ring,
        body,
        axes="yz",
        inner_elem="ribbed_lens_ring",
        outer_elem="long_lens_barrel",
        margin=0.007,
        name="lens ring stays coaxial with lens barrel",
    )
    ctx.expect_overlap(
        lens_ring,
        body,
        axes="x",
        elem_a="ribbed_lens_ring",
        elem_b="long_lens_barrel",
        min_overlap=0.035,
        name="lens ring encircles a real barrel length",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="z",
        elem_a="monitor_barrel",
        elem_b="monitor_hinge_pin",
        min_overlap=0.070,
        name="monitor hinge barrels share a vertical pin line",
    )

    closed_aabb = ctx.part_element_world_aabb(monitor, elem="display_glass")
    closed_screen_y = None
    if closed_aabb is not None:
        closed_screen_y = 0.5 * (closed_aabb[0][1] + closed_aabb[1][1])
    with ctx.pose({monitor_hinge: 1.2, ring_joint: math.pi / 2.0, dial_joint: math.pi / 3.0}):
        open_aabb = ctx.part_element_world_aabb(monitor, elem="display_glass")
        open_screen_y = None
        if open_aabb is not None:
            open_screen_y = 0.5 * (open_aabb[0][1] + open_aabb[1][1])
        ctx.expect_origin_distance(
            monitor,
            body,
            axes="xy",
            min_dist=0.065,
            name="flip monitor swings outward on side hinge",
        )
        ctx.expect_within(
            lens_ring,
            body,
            axes="yz",
            inner_elem="ribbed_lens_ring",
            outer_elem="long_lens_barrel",
            margin=0.007,
            name="rotated lens ring remains coaxial",
        )
    ctx.check(
        "monitor opens away from side body",
        closed_screen_y is not None
        and open_screen_y is not None
        and open_screen_y > closed_screen_y + 0.030,
        details=f"closed_screen_y={closed_screen_y}, open_screen_y={open_screen_y}",
    )

    return ctx.report()


object_model = build_object_model()
