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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_digital_multimeter")

    orange = model.material("safety_orange_rubber", rgba=(0.95, 0.38, 0.05, 1.0))
    dark = model.material("matte_black_plastic", rgba=(0.015, 0.016, 0.015, 1.0))
    charcoal = model.material("charcoal_panel", rgba=(0.055, 0.058, 0.060, 1.0))
    display_glass = model.material("green_tinted_display_glass", rgba=(0.34, 0.55, 0.46, 0.82))
    pale_key = model.material("pale_gray_soft_key", rgba=(0.74, 0.76, 0.72, 1.0))
    white_mark = model.material("white_printed_markings", rgba=(0.92, 0.92, 0.86, 1.0))
    red = model.material("red_input_jack", rgba=(0.82, 0.04, 0.03, 1.0))
    blue = model.material("blue_input_jack", rgba=(0.03, 0.15, 0.70, 1.0))

    width = 0.104
    height = 0.194
    thickness = 0.032
    front_z = thickness / 2.0
    rear_z = -thickness / 2.0

    housing = model.part("housing")

    boot_shape = (
        cq.Workplane("XY")
        .box(width, height, thickness)
        .edges("|Z")
        .fillet(0.017)
        .edges(">Z")
        .fillet(0.004)
        .edges("<Z")
        .fillet(0.003)
    )
    housing.visual(
        mesh_from_cadquery(boot_shape, "protective_shell", tolerance=0.0008),
        material=orange,
        name="protective_shell",
    )

    # Slightly proud black front instrument panel seated into the thick boot.
    housing.visual(
        Box((0.078, 0.158, 0.0024)),
        origin=Origin(xyz=(0.0, 0.002, front_z + 0.0010)),
        material=charcoal,
        name="front_panel",
    )

    # Raised corner cushions and side ribs make the boot read as thick rubber.
    for ix, x in enumerate((-0.039, 0.039)):
        for iy, y in enumerate((-0.075, 0.075)):
            housing.visual(
                Box((0.024, 0.026, 0.0045)),
                origin=Origin(xyz=(x, y, front_z + 0.0022)),
                material=orange,
                name=f"corner_bumper_{ix}_{iy}",
            )

    for side_x in (-0.055, 0.055):
        for y in (-0.052, -0.020, 0.012, 0.044):
            housing.visual(
                Box((0.006, 0.018, 0.024)),
                origin=Origin(xyz=(side_x, y, 0.0)),
                material=orange,
                name=f"side_grip_{'p' if side_x > 0 else 'n'}_{int((y + 0.060) * 1000)}",
            )

    # Display window and raised bezel.
    display_bezel = (
        cq.Workplane("XY")
        .rect(0.067, 0.040)
        .rect(0.056, 0.027)
        .extrude(0.003)
        .edges("|Z")
        .fillet(0.002)
    )
    housing.visual(
        mesh_from_cadquery(display_bezel, "display_bezel", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.058, front_z + 0.0018)),
        material=dark,
        name="display_bezel",
    )
    housing.visual(
        Box((0.053, 0.024, 0.0012)),
        origin=Origin(xyz=(0.0, 0.058, front_z + 0.0028)),
        material=display_glass,
        name="display_window",
    )

    # Static selector scale: a thin annular legend ring and twelve tick marks.
    selector_ring = (
        cq.Workplane("XY")
        .circle(0.033)
        .circle(0.0255)
        .extrude(0.0014)
    )
    housing.visual(
        mesh_from_cadquery(selector_ring, "selector_ring", tolerance=0.0005),
        origin=Origin(xyz=(0.0, -0.036, front_z + 0.0018)),
        material=dark,
        name="selector_ring",
    )
    for i in range(12):
        angle = 2.0 * math.pi * i / 12.0
        radius = 0.037
        tick_len = 0.008 if i % 3 == 0 else 0.005
        tick_w = 0.0014 if i % 3 == 0 else 0.0010
        housing.visual(
            Box((tick_w, tick_len, 0.0009)),
            origin=Origin(
                xyz=(radius * math.sin(angle), -0.036 + radius * math.cos(angle), front_z + 0.00265),
                rpy=(0.0, 0.0, -angle),
            ),
            material=white_mark,
            name=f"selector_tick_{i}",
        )

    housing.visual(
        Cylinder(radius=0.016, length=0.0024),
        origin=Origin(xyz=(0.0, -0.036, front_z + 0.0028)),
        material=dark,
        name="encoder_bushing",
    )

    # Input sockets at the lower edge of the face.
    for name, x, mat in (
        ("common_jack", -0.030, dark),
        ("volt_jack", -0.010, red),
        ("milliamp_jack", 0.012, blue),
        ("amp_jack", 0.033, red),
    ):
        housing.visual(
            Cylinder(radius=0.0085, length=0.0014),
            origin=Origin(xyz=(x, -0.078, front_z + 0.0022)),
            material=mat,
            name=f"{name}_ring",
        )
        housing.visual(
            Cylinder(radius=0.0044, length=0.0010),
            origin=Origin(xyz=(x, -0.078, front_z + 0.0034)),
            material=dark,
            name=f"{name}_socket",
        )

    # Rear shell details: stand landing pad and four clevis ears for the two pivots.
    housing.visual(
        Box((0.074, 0.130, 0.0025)),
        origin=Origin(xyz=(0.0, -0.010, rear_z - 0.0010)),
        material=dark,
        name="rear_panel",
    )
    for x in (-0.043, -0.023, 0.023, 0.043):
        housing.visual(
            Box((0.006, 0.014, 0.006)),
            origin=Origin(xyz=(x, -0.073, rear_z - 0.0028)),
            material=orange,
            name=f"stand_clevis_{int((x + 0.05) * 1000)}",
        )

    # Main continuously rotating selector knob.
    selector = model.part("selector_knob")
    knob_geom = KnobGeometry(
        0.045,
        0.017,
        body_style="skirted",
        base_diameter=0.050,
        top_diameter=0.038,
        edge_radius=0.0012,
        skirt=KnobSkirt(0.052, 0.0045, flare=0.04, chamfer=0.001),
        grip=KnobGrip(style="knurled", count=56, depth=0.0018, helix_angle_deg=28.0),
        indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
        center=False,
    )
    selector.visual(
        mesh_from_geometry(knob_geom, "selector_knob"),
        material=dark,
        name="knob_cap",
    )
    model.articulation(
        "housing_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=selector,
        origin=Origin(xyz=(0.0, -0.036, front_z + 0.0040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=4.0),
    )

    # Five independent soft keys below the display.  They are small prismatic
    # push controls that travel into the face along -Z.
    key_y = 0.024
    key_spacing = 0.016
    for i in range(5):
        key = model.part(f"soft_key_{i}")
        x = (i - 2) * key_spacing
        housing.visual(
            Box((0.0145, 0.0100, 0.0026)),
            origin=Origin(xyz=(x, key_y, front_z + 0.0035)),
            material=dark,
            name=f"key_socket_{i}",
        )
        key.visual(
            Box((0.0125, 0.0080, 0.0032)),
            origin=Origin(xyz=(0.0, 0.0, 0.0016)),
            material=pale_key,
            name="key_cap",
        )
        model.articulation(
            f"housing_to_soft_key_{i}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key,
            origin=Origin(xyz=(x, key_y, front_z + 0.0048)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.0012),
        )

    # Rear folding tilt stand: one rigid wide U-frame hinged on two lower pivots.
    stand = model.part("rear_stand")
    stand_frame = (
        cq.Workplane("XY")
        .center(0.0, 0.062)
        .rect(0.078, 0.124)
        .rect(0.049, 0.087)
        .extrude(0.005)
        .translate((0.0, 0.0, -0.0025))
        .edges("|Z")
        .fillet(0.003)
    )
    stand.visual(
        mesh_from_cadquery(stand_frame, "rear_stand_frame", tolerance=0.0007),
        material=dark,
        name="stand_frame",
    )
    for x in (-0.033, 0.033):
        stand.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark,
            name=f"pivot_barrel_{'p' if x > 0 else 'n'}",
        )
    model.articulation(
        "housing_to_rear_stand",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=stand,
        origin=Origin(xyz=(0.0, -0.073, rear_z - 0.0060)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=1.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    selector = object_model.get_part("selector_knob")
    stand = object_model.get_part("rear_stand")
    selector_joint = object_model.get_articulation("housing_to_selector_knob")
    stand_joint = object_model.get_articulation("housing_to_rear_stand")

    ctx.expect_overlap(
        selector,
        housing,
        axes="xy",
        elem_a="knob_cap",
        elem_b="selector_ring",
        min_overlap=0.040,
        name="selector knob is centered over the marked scale ring",
    )
    ctx.expect_contact(
        selector,
        housing,
        elem_a="knob_cap",
        elem_b="encoder_bushing",
        contact_tol=0.001,
        name="selector knob is carried on the front bushing",
    )
    knob_rest = ctx.part_world_position(selector)
    with ctx.pose({selector_joint: 2.0 * math.pi + 0.3}):
        knob_rotated = ctx.part_world_position(selector)
    ctx.check(
        "selector knob spins about its own fixed center",
        knob_rest is not None
        and knob_rotated is not None
        and abs(knob_rest[0] - knob_rotated[0]) < 1e-6
        and abs(knob_rest[1] - knob_rotated[1]) < 1e-6
        and abs(knob_rest[2] - knob_rotated[2]) < 1e-6,
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    soft_keys = [object_model.get_part(f"soft_key_{i}") for i in range(5)]
    soft_key_joints = [object_model.get_articulation(f"housing_to_soft_key_{i}") for i in range(5)]
    ctx.check("five independent soft key parts are present", len(soft_keys) == 5 and len(soft_key_joints) == 5)
    for i, (key, joint) in enumerate(zip(soft_keys, soft_key_joints)):
        ctx.expect_contact(
            key,
            housing,
            elem_a="key_cap",
            elem_b=f"key_socket_{i}",
            contact_tol=0.0005,
            name=f"soft key {i} rests on its socket",
        )
        rest_pos = ctx.part_world_position(key)
        with ctx.pose({joint: 0.0012}):
            pressed_pos = ctx.part_world_position(key)
        ctx.check(
            f"soft key {i} translates inward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0010,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    ctx.expect_overlap(
        stand,
        housing,
        axes="xy",
        elem_a="stand_frame",
        elem_b="rear_panel",
        min_overlap=0.060,
        name="rear stand spans much of the rear panel",
    )
    ctx.expect_gap(
        housing,
        stand,
        axis="z",
        positive_elem="protective_shell",
        negative_elem="stand_frame",
        min_gap=0.0,
        max_gap=0.006,
        name="stowed rear stand sits just behind the shell",
    )
    stand_rest_aabb = ctx.part_world_aabb(stand)
    with ctx.pose({stand_joint: 1.05}):
        stand_open_aabb = ctx.part_world_aabb(stand)
    ctx.check(
        "rear stand folds rearward from the lower pivots",
        stand_rest_aabb is not None
        and stand_open_aabb is not None
        and stand_open_aabb[0][2] < stand_rest_aabb[0][2] - 0.040,
        details=f"rest={stand_rest_aabb}, open={stand_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
