from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    Material,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="push_button_plunger")

    # Materials with realistic colors
    housing_material = Material(name="housing_dark", rgba=(0.2, 0.2, 0.25, 1.0))
    button_material = Material(name="button_red", rgba=(0.85, 0.15, 0.15, 1.0))
    stem_material = Material(name="stem_silver", rgba=(0.7, 0.7, 0.75, 1.0))
    collar_material = Material(name="collar_black", rgba=(0.15, 0.15, 0.15, 1.0))

    # ---- FIXED HOUSING ----
    housing = model.part("housing")

    # Main housing body dimensions
    housing_outer_radius = 0.020
    housing_height = 0.060
    housing_wall_thickness = 0.003

    # Create hollow housing cylinder
    housing_shape = (
        cq.Workplane("XY")
        .workplane()
        .circle(housing_outer_radius)
        .circle(housing_outer_radius - housing_wall_thickness)
        .extrude(housing_height)
    )

    # Add a rectangular viewing slot in the side
    slot_width = 0.012
    slot_height = 0.040

    # Create a box cutter for the slot
    slot_cutter = (
        cq.Workplane("YZ")
        .workplane(offset=-housing_outer_radius - 0.001)
        .box(housing_outer_radius * 2 + 0.002, slot_height, slot_width)
        .translate((0, 0, housing_height / 2))
    )

    housing_shape = housing_shape.cut(slot_cutter)

    # Add chamfered edges to housing
    housing_shape = housing_shape.edges("|Z").chamfer(0.001)

    housing.visual(
        mesh_from_cadquery(housing_shape, "housing_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=housing_material,
        name="housing_shell",
    )

    # Guide collar - ring at the top of housing
    collar_outer_radius = housing_outer_radius + 0.002
    collar_inner_radius = 0.008
    collar_height = 0.008

    collar_shape = (
        cq.Workplane("XY")
        .workplane()
        .circle(collar_outer_radius)
        .circle(collar_inner_radius)
        .extrude(collar_height)
    )

    housing.visual(
        mesh_from_cadquery(collar_shape, "guide_collar"),
        origin=Origin(xyz=(0.0, 0.0, housing_height)),
        material=collar_material,
        name="guide_collar",
    )

    # ---- MOVING BUTTON ASSEMBLY ----
    button_assembly = model.part("button_assembly")

    # Button cap dimensions
    cap_radius = 0.018
    cap_height = 0.012

    # Create button cap as a cylinder with a domed top
    # Dome using sphere intersection
    cap_cyl = cq.Workplane("XY").workplane().circle(cap_radius).extrude(cap_height)

    # Add dome on top
    dome = (
        cq.Workplane("XY")
        .workplane(offset=cap_height)
        .sphere(cap_radius)
    )
    # Keep only the top hemisphere
    keep_box = cq.Workplane("XY").box(
        cap_radius * 2.2, cap_radius * 2.2, cap_height * 2, centered=(True, True, False)
    )
    dome = dome.intersect(keep_box.translate((0, 0, cap_height)))

    cap_shape = cap_cyl.union(dome)
    cap_shape = cap_shape.edges("<Z or |Z").chamfer(0.0005)

    # Cap is positioned so its BOTTOM is at z=0 (part frame origin)
    # Cap extends from z=0 to z=cap_height
    button_assembly.visual(
        mesh_from_cadquery(cap_shape, "button_cap"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=button_material,
        name="button_cap",
    )

    # Internal stem - extends down from the cap into the housing
    stem_radius = 0.006
    stem_length = 0.050

    stem_shape = cq.Workplane("XY").circle(stem_radius).extrude(stem_length)

    # Add decorative ring grooves (spring-like appearance)
    groove_depth = 0.001
    groove_width = 0.001
    num_grooves = 6
    groove_start = 0.010
    groove_end = 0.040
    groove_spacing = (groove_end - groove_start) / (num_grooves + 1)

    for i in range(num_grooves):
        z_center = groove_start + (i + 1) * groove_spacing
        # Create groove by cutting annular ring
        groove_cutter = (
            cq.Workplane("XY")
            .workplane(offset=z_center - groove_width / 2)
            .circle(stem_radius)
            .workplane(offset=groove_width)
            .circle(stem_radius - groove_depth)
            .loft()
        )
        stem_shape = stem_shape.cut(groove_cutter)

    # Stem extends DOWN from the cap (part frame at bottom of cap)
    # So stem origin is at z = -stem_length (goes down from part frame)
    button_assembly.visual(
        mesh_from_cadquery(stem_shape, "stem"),
        origin=Origin(xyz=(0.0, 0.0, -stem_length)),
        material=stem_material,
        name="stem",
    )

    # ---- ARTICULATION ----
    # The button sits on top of the housing at rest
    # Articulation origin is at the bottom of the button cap
    # At q=0: button is at rest (extended, cap sitting on collar)
    # At q=press_travel: button is pressed (moved down by press_travel)

    press_travel = 0.008

    model.articulation(
        "housing_to_button",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=button_assembly,
        # Joint at the resting position (bottom of button cap sits on top of collar)
        # Collar top is at z = housing_height + collar_height = 0.060 + 0.008 = 0.068
        origin=Origin(xyz=(0.0, 0.0, housing_height + collar_height)),
        axis=(0.0, 0.0, -1.0),  # Positive q presses DOWN (negative Z)
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.15,
            lower=0.0,  # Rest position (extended)
            upper=press_travel,  # Max press distance
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    button = object_model.get_part("button_assembly")
    press_joint = object_model.get_articulation("housing_to_button")

    # Test 1: Check that housing is the root part
    root_parts = object_model.root_parts()
    ctx.check(
        "housing_is_root",
        len(root_parts) == 1 and root_parts[0].name == "housing",
        details=f"Root parts: {[p.name for p in root_parts]}",
    )

    # Test 2: Check prismatic joint configuration
    ctx.check(
        "joint_is_prismatic",
        press_joint.articulation_type == ArticulationType.PRISMATIC,
        details=f"Joint type: {press_joint.articulation_type}",
    )

    # Test 3: Check axis is along -Z (for press-down)
    ctx.check(
        "joint_axis_is_correct",
        press_joint.axis == (0.0, 0.0, -1.0),
        details=f"Joint axis: {press_joint.axis}",
    )

    # Test 4: Check travel limits are realistic (5-15mm)
    limits = press_joint.motion_limits
    if limits and limits.lower is not None and limits.upper is not None:
        travel = limits.upper - limits.lower
        ctx.check(
            "realistic_travel_distance",
            0.005 <= travel <= 0.020,
            details=f"Travel: {travel:.3f}m ({travel*1000:.1f}mm)",
        )

    # Test 5: Check button cap is above housing at rest position
    with ctx.pose({press_joint: 0.0}):
        ctx.expect_gap(
            button,
            housing,
            axis="z",
            positive_elem="button_cap",
            negative_elem="housing_shell",
            min_gap=0.002,
            name="button_cap_above_housing_at_rest",
        )

    # Test 6: Check button cap is closer to housing when pressed
    with ctx.pose({press_joint: limits.upper if limits else 0.008}):
        ctx.expect_gap(
            button,
            housing,
            axis="z",
            positive_elem="button_cap",
            negative_elem="housing_shell",
            max_gap=0.015,
            name="button_cap_near_housing_when_pressed",
        )

    # Test 7: Verify the button moves DOWN when pressed
    rest_pos = None
    pressed_pos = None
    with ctx.pose({press_joint: 0.0}):
        rest_pos = ctx.part_world_position(button)
    with ctx.pose({press_joint: limits.upper if limits else 0.008}):
        pressed_pos = ctx.part_world_position(button)

    if rest_pos and pressed_pos:
        z_travel = pressed_pos[2] - rest_pos[2]
        ctx.check(
            "button_moves_down_when_pressed",
            z_travel < -0.005,  # Should move down (negative Z)
            details=f"Z travel: {z_travel:.4f}m",
        )

    # Test 8: Check that critical visuals exist
    housing_shell = housing.get_visual("housing_shell")
    button_cap = button.get_visual("button_cap")
    stem = button.get_visual("stem")
    collar = housing.get_visual("guide_collar")

    ctx.check("housing_shell_exists", housing_shell is not None)
    ctx.check("button_cap_exists", button_cap is not None)
    ctx.check("stem_exists", stem is not None)
    ctx.check("collar_exists", collar is not None)

    # Test 9: Check materials are assigned
    ctx.check(
        "button_cap_has_material",
        button_cap.material is not None,
    )

    # Test 10: Check button cap is centered over housing (XY alignment)
    with ctx.pose({press_joint: 0.0}):
        ctx.expect_origin_distance(
            button,
            housing,
            axes="xy",
            max_dist=0.002,
            name="button_centered_over_housing",
        )

    # Test 11: Check stem is inside housing (overlaps in XY)
    with ctx.pose({press_joint: 0.0}):
        ctx.expect_overlap(
            button,
            housing,
            axes="xy",
            elem_a="stem",
            elem_b="housing_shell",
            min_overlap=0.004,
            name="stem_inside_housing",
        )

    # Allow overlap between stem and housing (intentional nested fit)
    ctx.allow_overlap(
        "button_assembly",
        "housing",
        elem_a="stem",
        elem_b="housing_shell",
        reason="Stem is intentionally inside the housing for the push-button mechanism",
    )

    return ctx.report()


object_model = build_object_model()
