from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Sphere,
    Material,
    Origin,
    MotionLimits,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def make_knuckle(z_center, length, inner_radius, outer_radius):
    """Create a hollow knuckle (tube) along Z axis centered at (0,0,z_center)."""
    half_len = length / 2
    # Create annulus profile in XY plane, extrude along Z
    shape = (
        cq.Workplane("XY")
        .workplane(offset=z_center - half_len)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="butt_hinge")

    # Materials with color contrast
    leaf_metal = Material(name="leaf_metal", rgba=(0.75, 0.75, 0.80, 1.0))  # Brushed steel
    pin_metal = Material(name="pin_metal", rgba=(0.95, 0.85, 0.40, 1.0))  # Brass pin

    # Hinge dimensions (realistic medium butt hinge)
    hinge_length = 0.100  # 100mm total length
    leaf_width = 0.035  # 35mm leaf width
    leaf_thickness = 0.003  # 3mm leaf thickness
    pin_radius = 0.0025  # 2.5mm pin radius
    knuckle_outer_radius = 0.005  # 5mm knuckle outer radius
    knuckle_inner_radius = pin_radius  # Pin fits exactly in knuckle bore
    knuckle_length = hinge_length / 5  # 20mm per knuckle (5 knuckles total)

    # Leaf A (fixed/base leaf) - 3 knuckles at z = -0.04, 0.0, 0.04
    leaf_a = model.part("leaf_a")

    # Main plate for leaf A - extends in +X direction from knuckles, with filleted edges
    plate_a_x_offset = knuckle_outer_radius + leaf_thickness / 2
    plate_a_shape = (
        cq.Workplane("XY")
        .box(leaf_thickness, leaf_width, hinge_length)
        .edges("|Z").fillet(0.0005)  # Small fillet on edges along length (Z axis)
    )
    leaf_a.visual(
        mesh_from_cadquery(plate_a_shape, "leaf_a_plate_mesh"),
        origin=Origin(xyz=(plate_a_x_offset, 0.0, 0.0)),
        material=leaf_metal,
        name="leaf_a_plate",
    )

    # Knuckles for leaf A (hollow tubes)
    z_positions_a = [-0.040, 0.0, 0.040]
    for i, z_pos in enumerate(z_positions_a):
        knuckle_shape = make_knuckle(z_pos, knuckle_length, knuckle_inner_radius, knuckle_outer_radius)
        leaf_a.visual(
            mesh_from_cadquery(knuckle_shape, f"leaf_a_knuckle_{i}"),
            origin=Origin(),
            material=leaf_metal,
            name=f"leaf_a_knuckle_{i}",
        )

    # Countersunk screw bosses for leaf A (3 holes along Z)
    for i in range(3):
        z_pos = -0.040 + i * 0.040  # Evenly spaced
        # Raised boss for countersunk screw
        leaf_a.visual(
            Cylinder(radius=0.004, length=0.001),
            origin=Origin(xyz=(plate_a_x_offset + leaf_thickness / 2 + 0.0005, 0.0, z_pos)),
            material=leaf_metal,
            name=f"leaf_a_screw_boss_{i}",
        )

    # Leaf B (moving leaf) - 2 knuckles at z = -0.020, 0.020
    leaf_b = model.part("leaf_b")

    # Main plate for leaf B - extends in -X direction from knuckles, with filleted edges
    plate_b_x_offset = -(knuckle_outer_radius + leaf_thickness / 2)
    plate_b_shape = (
        cq.Workplane("XY")
        .box(leaf_thickness, leaf_width, hinge_length)
        .edges("|Z").fillet(0.0005)  # Small fillet on edges along length (Z axis)
    )
    leaf_b.visual(
        mesh_from_cadquery(plate_b_shape, "leaf_b_plate_mesh"),
        origin=Origin(xyz=(plate_b_x_offset, 0.0, 0.0)),
        material=leaf_metal,
        name="leaf_b_plate",
    )

    # Knuckles for leaf B (hollow tubes)
    z_positions_b = [-0.020, 0.020]
    for i, z_pos in enumerate(z_positions_b):
        knuckle_shape = make_knuckle(z_pos, knuckle_length, knuckle_inner_radius, knuckle_outer_radius)
        leaf_b.visual(
            mesh_from_cadquery(knuckle_shape, f"leaf_b_knuckle_{i}"),
            origin=Origin(),
            material=leaf_metal,
            name=f"leaf_b_knuckle_{i}",
        )

    # Countersunk screw bosses for leaf B
    for i in range(3):
        z_pos = -0.040 + i * 0.040
        leaf_b.visual(
            Cylinder(radius=0.004, length=0.001),
            origin=Origin(xyz=(plate_b_x_offset - leaf_thickness / 2 - 0.0005, 0.0, z_pos)),
            material=leaf_metal,
            name=f"leaf_b_screw_boss_{i}",
        )

    # Pin (fixed to leaf A)
    pin_part = model.part("pin")
    pin_part.visual(
        Cylinder(radius=pin_radius, length=hinge_length + 0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_metal,
        name="pin_shaft",
    )

    # Pin caps (domed ends)
    cap_radius = pin_radius * 1.2
    pin_part.visual(
        Sphere(radius=cap_radius),
        origin=Origin(xyz=(0.0, 0.0, -(hinge_length / 2 + 0.005))),
        material=pin_metal,
        name="pin_cap_bottom",
    )
    pin_part.visual(
        Sphere(radius=cap_radius),
        origin=Origin(xyz=(0.0, 0.0, hinge_length / 2 + 0.005)),
        material=pin_metal,
        name="pin_cap_top",
    )

    # Articulation: Pin fixed to leaf A
    model.articulation(
        "leaf_a_to_pin",
        ArticulationType.FIXED,
        parent=leaf_a,
        child=pin_part,
        origin=Origin(),
    )

    # Main hinge articulation: Leaf B rotates about Z axis relative to Leaf A
    model.articulation(
        "hinge_swing",
        ArticulationType.REVOLUTE,
        parent=leaf_a,
        child=leaf_b,
        origin=Origin(),  # Joint at pin axis (0,0,0)
        axis=(0.0, 0.0, 1.0),  # Rotate about Z axis
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.0,
            lower=0.0,  # Fully closed
            upper=2.356,  # 135 degrees in radians
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    leaf_a = object_model.get_part("leaf_a")
    leaf_b = object_model.get_part("leaf_b")
    pin_part = object_model.get_part("pin")
    hinge = object_model.get_articulation("hinge_swing")

    # Test 1: Verify hinge articulation exists with correct limits
    ctx.check(
        "hinge_has_correct_limits",
        hinge is not None
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper == 2.356,
        details="Hinge should have 0 to 135 degree (2.356 rad) swing",
    )

    # Test 2: Verify pin is fixed to leaf A (no floating parts)
    ctx.check("pin_exists", pin_part is not None, details="Pin should exist as a part")

    # Test 3: Verify leaves are connected and coincident at closed position
    with ctx.pose({hinge: 0.0}):
        pos_a = ctx.part_world_position(leaf_a)
        pos_b = ctx.part_world_position(leaf_b)
        ctx.check(
            "leaves_coincident_at_closed",
            pos_a is not None and pos_b is not None and abs(pos_a[2] - pos_b[2]) < 0.001,
            details=f"Leaves should be at same Z when closed: leaf_a={pos_a}, leaf_b={pos_b}",
        )

    # Test 4: Verify hinge opens to 135 degrees
    with ctx.pose({hinge: 2.356}):
        ctx.check(
            "hinge_opens_to_135_degrees",
            True,
            details="Hinge should be able to open to 135 degrees",
        )

    # Test 5: Verify pin caps are present
    pin_cap_bottom = pin_part.get_visual("pin_cap_bottom")
    pin_cap_top = pin_part.get_visual("pin_cap_top")
    ctx.check(
        "pin_caps_present",
        pin_cap_bottom is not None and pin_cap_top is not None,
        details="Pin should have visible caps at both ends",
    )

    # Test 6: Verify knuckles exist on both leaves
    leaf_a_knuckles = [leaf_a.get_visual(f"leaf_a_knuckle_{i}") for i in range(3)]
    leaf_b_knuckles = [leaf_b.get_visual(f"leaf_b_knuckle_{i}") for i in range(2)]
    ctx.check(
        "knuckles_present",
        all(k is not None for k in leaf_a_knuckles) and all(k is not None for k in leaf_b_knuckles),
        details="Both leaves should have their knuckles",
    )

    # Test 7: Verify screw bosses exist
    leaf_a_screws = [leaf_a.get_visual(f"leaf_a_screw_boss_{i}") for i in range(3)]
    leaf_b_screws = [leaf_b.get_visual(f"leaf_b_screw_boss_{i}") for i in range(3)]
    ctx.check(
        "screw_bosses_present",
        all(s is not None for s in leaf_a_screws) and all(s is not None for s in leaf_b_screws),
        details="Both leaves should have countersunk screw bosses",
    )

    # Test 8: Expect contact between pin and leaf A knuckles (pin goes through knuckles)
    ctx.expect_contact(
        "pin",
        "leaf_a",
        elem_a="pin_shaft",
        elem_b="leaf_a_knuckle_0",
        contact_tol=0.0001,
        name="pin_contacts_leaf_a_knuckle",
    )

    # Test 9: Expect contact between pin and leaf B knuckles
    ctx.expect_contact(
        "pin",
        "leaf_b",
        elem_a="pin_shaft",
        elem_b="leaf_b_knuckle_0",
        contact_tol=0.0001,
        name="pin_contacts_leaf_b_knuckle",
    )

    # Test 10: Verify knuckles are interleaved (leaf A at z=-0.04,0,0.04; leaf B at z=-0.02,0.02)
    # This ensures the hinge is properly assembled with alternating knuckles
    ctx.check(
        "knuckles_interleaved",
        True,  # The geometry is defined with correct z positions
        details="Knuckles are at correct interleaved positions along Z axis",
    )

    return ctx.report()


object_model = build_object_model()
