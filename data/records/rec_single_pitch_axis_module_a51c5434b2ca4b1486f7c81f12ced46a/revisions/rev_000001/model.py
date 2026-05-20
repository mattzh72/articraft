from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="range_hood_flap")

    # Fixed hood frame (root part)
    hood_frame = model.part("hood_frame")

    # Frame body: chamfered rectangular box (X: width, Y: depth, Z: height)
    frame_body_cq = (
        cq.Workplane("XY")
        .box(0.9, 0.1, 0.2)
        .edges()
        .chamfer(0.005)
    )
    hood_frame.visual(
        mesh_from_cadquery(frame_body_cq, "frame_body_geom"),
        origin=Origin(xyz=(0.0, 0.05, 0.1)),
        name="frame_body",
        color=(0.7, 0.7, 0.7),
    )

    # Hinge rod: cylinder along X-axis at top front edge of frame
    hinge_rod_cq = (
        cq.Workplane("YZ")  # Y-Z workplane (X=constant)
        .workplane(offset=-0.45)  # Start extrusion at X=-0.45
        .center(0.1, 0.2)  # Y=0.1 (front face), Z=0.2 (top edge) in workplane
        .circle(0.01)  # Rod radius 0.01m
        .extrude(0.9)  # Extrude 0.9m along X to X=0.45 (total length 0.9m)
    )
    hood_frame.visual(
        mesh_from_cadquery(hinge_rod_cq, "hinge_rod_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="hinge_rod",
        color=(0.7, 0.7, 0.7),
    )

    # Baffle flap (articulated child part)
    baffle_flap = model.part("baffle_flap")

    # Create single connected flap with integrated handle lip using profile extrusion
    # Workplane XZ: X=width, Z=height (flap hangs from top)
    flap_with_lip_cq = (
        cq.Workplane("XZ")  # X=width, Z=height
        .rect(0.85, 0.18, forConstruction=True)  # Main flap: 0.85m wide, 0.18m tall
        .rect(0.8, 0.01, forConstruction=True)  # Handle lip: 0.8m wide, 0.01m tall
        # Position handle at bottom of flap (Z=-0.18 to Z=-0.19)
        .center(0, -0.18 - 0.005)  # Center of handle: Z=-0.185 (midpoint of -0.18 and -0.19)
        .rect(0.8, 0.01)
        # Position main flap: Z=-0.09 (center of 0.18m tall flap from Z=0 to Z=-0.18)
        .center(0, 0.09 + 0.005)  # Reset to flap center: Z=-0.09
        .rect(0.85, 0.18)
        # Extrude along Y-axis (thickness of flap: 0.02m)
        .extrude(0.02)
        # Chamfer all edges
        .edges()
        .chamfer(0.003)
    )
    baffle_flap.visual(
        mesh_from_cadquery(flap_with_lip_cq, "flap_with_lip_geom"),
        # Y offset: world min Y=0.1 + 0.026 -0.015 = 0.111 → gap=0.011m
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        name="flap_with_lip",
        color=(0.6, 0.6, 0.6),
    )

    # Revolute joint for tilting flap (pitch around X-axis)
    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=hood_frame,
        child=baffle_flap,
        origin=Origin(xyz=(0.0, 0.1, 0.2)),  # Hinge axis position (top front of frame)
        axis=(1.0, 0.0, 0.0),  # Rotation around X-axis (right-hand rule)
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=-1.047,  # ~60 degrees open (tilted downward)
            upper=0.0,  # Fully closed
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("hood_frame")
    flap = object_model.get_part("baffle_flap")
    joint = object_model.get_articulation("frame_to_flap")

    # Main mechanism checks
    ctx.check(
        "joint is revolute type",
        joint.articulation_type == ArticulationType.REVOLUTE,
        details=f"Actual joint type: {joint.articulation_type}",
    )
    ctx.check(
        "joint axis aligns with hinge rod (X-axis)",
        joint.axis == (1.0, 0.0, 0.0),
        details=f"Actual joint axis: {joint.axis}",
    )
    ctx.check(
        "joint motion limits are correct",
        (joint.motion_limits.lower, joint.motion_limits.upper) == (-1.047, 0.0),
        details=f"Limits: lower={joint.motion_limits.lower}, upper={joint.motion_limits.upper}",
    )

    # Closed pose (q=0) checks
    with ctx.pose({joint: 0.0}):
        # Flap should be positioned in front of frame with small gap
        ctx.expect_gap(
            positive_link=flap,
            negative_link=frame,
            axis="y",
            min_gap=0.005,
            max_gap=0.015,
            positive_elem="flap_with_lip",
            negative_elem="frame_body",
            name="closed flap gap from frame front",
        )
        # Hinge rod and flap panel should be in contact (intentional overlap)
        ctx.expect_contact(
            link_a=frame,
            link_b=flap,
            elem_a="hinge_rod",
            elem_b="flap_with_lip",
            contact_tol=0.015,
            name="hinge rod contacts flap at closed pose",
        )

    # Open pose (max extension) check
    closed_bottom_z = None
    open_bottom_z = None
    with ctx.pose({joint: 0.0}):  # Closed pose
        closed_aabb = ctx.part_element_world_aabb(flap, elem="flap_with_lip")
        if closed_aabb:
            closed_bottom_z = closed_aabb[0][2]  # Min Z of flap (z is third element)
    with ctx.pose({joint: joint.motion_limits.lower}):  # Open pose (lower limit = tilted downward)
        open_aabb = ctx.part_element_world_aabb(flap, elem="flap_with_lip")
        if open_aabb:
            open_bottom_z = open_aabb[0][2]
    ctx.check(
        "flap tilts upward when open (top-hinged behavior)",
        closed_bottom_z is not None and open_bottom_z is not None and open_bottom_z > closed_bottom_z,
        details=f"Closed bottom Z: {closed_bottom_z}, Open bottom Z: {open_bottom_z}",
    )

    # Allow intentional overlap between hinge rod and flap panel (captured pivot)
    ctx.allow_overlap(
        link_a="hood_frame",
        link_b="baffle_flap",
        elem_a="hinge_rod",
        elem_b="flap_with_lip",
        reason="Hinge rod is captured in flap's top edge pivot, small intentional overlap",
    )

    # Visible details check: flap includes integrated handle lip
    flap = object_model.get_part("baffle_flap")
    with ctx.pose({joint: 0.0}):  # Check at rest pose
        flap_aabb = ctx.part_element_world_aabb(flap, elem="flap_with_lip")
        if flap_aabb:
            flap_height = flap_aabb[1][2] - flap_aabb[0][2]  # max z - min z
            ctx.check(
                "flap has correct total height including handle",
                abs(flap_height - 0.19) < 0.01,
                details=f"Flap height: {flap_height:.3f}m, expected ~0.19m",
            )
        else:
            ctx.fail("flap_height_check", "Could not get flap AABB")

    return ctx.report()


object_model = build_object_model()
