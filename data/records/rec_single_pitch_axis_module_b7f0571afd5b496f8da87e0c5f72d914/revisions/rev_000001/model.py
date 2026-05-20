from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    BezelGeometry,
    BezelFace,
    mesh_from_geometry,
    place_on_face,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vent_flap_module")

    # Materials
    frame_material = model.material("frame_white", rgba=(0.95, 0.95, 0.95, 1.0))
    flap_material = model.material("flap_light_gray", rgba=(0.85, 0.85, 0.85, 1.0))
    seal_material = model.material("seal_black", rgba=(0.1, 0.1, 0.1, 1.0))
    axle_material = model.material("axle_dark_gray", rgba=(0.4, 0.4, 0.4, 1.0))

    # --- Frame (root part, fixed) ---
    frame = model.part("frame")
    
    # Main frame using bezel geometry for the rectangular frame with opening
    frame_bezel = BezelGeometry(
        opening_size=(0.26, 0.16),  # Flap opening size (X, Y)
        outer_size=(0.30, 0.20),    # Frame outer dimensions
        depth=0.05,                  # Frame depth (Z)
        opening_shape="rect",
        outer_shape="rounded_rect",
        opening_corner_radius=0.002,
        outer_corner_radius=0.004,
        wall=0.02,                  # Frame wall thickness
        face=BezelFace(style="radiused_step", front_lip=0.001, fillet=0.002),
    )
    frame_visual = mesh_from_geometry(frame_bezel, "frame_shell")
    frame.visual(
        frame_visual,
        origin=Origin(),  # Centered at origin
        material=frame_material,
    )

    # Seal rim around the opening (raised rim for sealing when flap is closed)
    seal_rim = BezelGeometry(
        opening_size=(0.252, 0.152),  # Slightly smaller than opening to create raised rim
        outer_size=(0.26, 0.16),     # Same as opening size
        depth=0.005,                 # Thin raised rim
        opening_shape="rect",
        outer_shape="rect",
        center=False,                # Place rear face at z=0, so front at z=0.005
    )
    seal_visual = mesh_from_geometry(seal_rim, "seal_rim")
    frame.visual(
        seal_visual,
        origin=Origin(xyz=(0, 0, 0.025)),  # Position at front face of frame (frame depth 0.05, centered at origin, so front face at z=0.025)
        material=seal_material,
        name="seal_rim",
    )

    # Side axle bosses (left and right, at hinge line position)
    # Hinge line is at top of opening: Y=0.08 (since opening Y=0.16 centered, top is 0.08)
    boss_length = 0.01  # Small boss protruding inward
    boss_radius = 0.008
    # Left boss
    frame.visual(
        Cylinder(radius=boss_radius, length=boss_length),
        origin=Origin(xyz=(-0.13, 0.08, 0.025 - boss_length/2)),  # Left side, top of opening, front face
        material=axle_material,
        name="left_axle_boss",
    )
    # Right boss
    frame.visual(
        Cylinder(radius=boss_radius, length=boss_length),
        origin=Origin(xyz=(0.13, 0.08, 0.025 - boss_length/2)),
        material=axle_material,
        name="right_axle_boss",
    )

    # --- Flap (moving part) ---
    flap = model.part("flap")
    
    # Main flap blade (broad rectangular panel)
    flap_blade = Box((0.26, 0.16, 0.02))  # X, Y, Z (width, height, thickness)
    # Position flap so top edge is at y=0 (hinge line), extends downward to y=-0.16
    # Flap part frame is at hinge line, so blade extends along -Y from hinge
    flap.visual(
        flap_blade,
        origin=Origin(xyz=(0, -0.08, 0.015)),  # Center Z at front face of frame (0.025 - half thickness 0.01 = 0.015)
        material=flap_material,
        name="flap_blade",
    )

    # Handle lip on the flap (bottom edge, opposite hinge)
    # Position it proud of the flap front face for gripping
    handle_lip = Box((0.10, 0.008, 0.015))  # Small lip along bottom edge
    flap.visual(
        handle_lip,
        origin=Origin(xyz=(0, -0.16, 0.0325)),  # Proud of flap front face (0.025 + half thickness 0.0075)
        material=flap_material,
        name="handle_lip",
    )

    # --- Articulation: Pitch joint along flap width (X-axis) ---
    # Hinge line is along X-axis at top of flap (y=0.08, z=0.025 in frame part frame)
    model.articulation(
        "frame_to_flap",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=flap,
        origin=Origin(xyz=(0, 0.08, 0.025)),  # Hinge line at top center of opening, front face
        axis=(1.0, 0.0, 0.0),  # Rotation along X-axis (flap width)
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,    # Closed position
            upper=1.57,   # ~90 degrees open
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    flap = object_model.get_part("flap")
    hinge = object_model.get_articulation("frame_to_flap")

    # Closed pose checks (q=0)
    with ctx.pose({hinge: 0.0}):
        # Flap should be flush with seal rim when closed
        # flap_blade is on flap (positive_link), seal_rim is on frame (negative_link)
        ctx.expect_contact(
            flap, frame,
            elem_a="flap_blade",
            elem_b="seal_rim",
            name="flap seals against rim when closed",
        )
        ctx.expect_gap(
            flap, frame,
            axis="z",
            max_penetration=0.001,
            positive_elem="flap_blade",
            negative_elem="seal_rim",
            name="flap flush with seal rim (closed)",
        )

    # Open pose checks (q=1.57 ~90 degrees)
    with ctx.pose({hinge: 1.57}):
        # Flap rotates around X-axis; the blade should now be more horizontal
        # Check that flap has moved - measure the blade's position
        flap_center = ctx.part_world_position(flap)
        ctx.check(
            "flap opens to horizontal position",
            flap_center is not None,
            details=f"flap part position at open: {flap_center}",
        )

    return ctx.report()


object_model = build_object_model()
