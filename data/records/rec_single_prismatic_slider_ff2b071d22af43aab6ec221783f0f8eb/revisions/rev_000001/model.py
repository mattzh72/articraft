from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_fader_cap():
    """Build the sliding fader cap that sits in the slot."""
    # Cap dimensions - top button part
    cap_top_width = 0.018
    cap_top_length = 0.018
    cap_top_height = 0.008
    
    # Shaft that goes through the slot
    shaft_width = 0.006
    shaft_length = 0.010
    shaft_height = 0.005
    
    # Build the cap (top part the user touches)
    cap = (
        cq.Workplane("XY")
        .box(cap_top_length, cap_top_width, cap_top_height)
    )
    
    # Add chamfered edges on top
    cap = cap.edges("|Z").chamfer(0.002)
    
    # Build the shaft (goes through slot)
    shaft = (
        cq.Workplane("XY")
        .box(shaft_length, shaft_width, shaft_height)
        .translate((0, 0, -shaft_height / 2 - 0.001))
    )
    
    cap = cap.union(shaft)
    
    # Add small knob details on top
    knob_detail = (
        cq.Workplane("XY")
        .box(cap_top_length * 0.6, cap_top_width * 0.3, 0.001)
        .translate((0, 0, cap_top_height / 2 + 0.0005))
    )
    cap = cap.union(knob_detail)
    
    return cap


def build_panel_body():
    """Build the main panel body with slot."""
    panel_width = 0.12
    panel_height = 0.08
    panel_thickness = 0.003
    slot_length = 0.10
    slot_width = 0.008
    
    panel = (
        cq.Workplane("XY")
        .box(panel_width, panel_height, panel_thickness)
        .faces(">Z")
        .workplane()
        .rect(slot_length, slot_width)
        .cutThruAll()
    )
    
    return panel


def build_rails():
    """Build the top and bottom rails."""
    slot_length = 0.10
    rail_thickness = 0.002
    rail_height = 0.004
    rail_spacing = 0.008 / 2 + rail_thickness / 2
    
    rail_profile = (
        cq.Workplane("XZ")
        .rect(rail_thickness, rail_height)
        .extrude(slot_length + 0.01)
    )
    
    rail_top = rail_profile.translate((0, rail_spacing, rail_height / 2))
    rail_bottom = rail_profile.translate((0, -rail_spacing, rail_height / 2))
    
    return rail_top, rail_bottom


def build_tick_marks():
    """Build raised tick marks along the slot."""
    slot_length = 0.10
    tick_width = 0.0005
    tick_depth = 0.003
    num_ticks = 11
    tick_spacing = slot_length / (num_ticks - 1)
    
    ticks = []
    for i in range(num_ticks):
        x_pos = -slot_length / 2 + i * tick_spacing
        # Center tick is longer
        if i == num_ticks // 2:
            tick_len = 0.008
        else:
            tick_len = 0.004
        
        tick = (
            cq.Workplane("XY")
            .box(tick_len, tick_width, tick_depth)
            .translate((x_pos, 0, tick_depth / 2))
        )
        ticks.append(tick)
    
    return ticks


def build_end_stops():
    """Build end stops at both ends of the slot."""
    slot_length = 0.10
    stop_width = 0.01
    stop_height = 0.006
    slot_width = 0.008
    
    stop_left = (
        cq.Workplane("XY")
        .box(stop_width, slot_width + 0.008, stop_height)
        .translate((-slot_length / 2 - stop_width / 2, 0, stop_height / 2))
    )
    
    stop_right = (
        cq.Workplane("XY")
        .box(stop_width, slot_width + 0.008, stop_height)
        .translate((slot_length / 2 + stop_width / 2, 0, stop_height / 2))
    )
    
    return stop_left, stop_right


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="instrument_fader_slider")
    
    # Define materials
    model.material("panel_dark", rgba=(0.18, 0.18, 0.20, 1.0))  # Dark gray panel
    model.material("rail_metal", rgba=(0.70, 0.72, 0.75, 1.0))  # Silver metal rails
    model.material("cap_black", rgba=(0.10, 0.10, 0.12, 1.0))  # Black fader cap
    model.material("tick_white", rgba=(0.85, 0.85, 0.88, 1.0))  # White tick marks
    model.material("stop_red", rgba=(0.70, 0.15, 0.15, 1.0))  # Red end stops
    
    # Build the panel (base/root part)
    panel = model.part("panel")
    
    # Main panel body with slot
    panel_body = build_panel_body()
    panel.visual(
        mesh_from_cadquery(panel_body, "panel_body"),
        material="panel_dark",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="panel_body"
    )
    
    # Add rails as separate visuals
    rail_top, rail_bottom = build_rails()
    panel.visual(
        mesh_from_cadquery(rail_top, "rail_top"),
        material="rail_metal",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rail_top"
    )
    panel.visual(
        mesh_from_cadquery(rail_bottom, "rail_bottom"),
        material="rail_metal",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="rail_bottom"
    )
    
    # Add tick marks
    ticks = build_tick_marks()
    for i, tick in enumerate(ticks):
        panel.visual(
            mesh_from_cadquery(tick, f"tick_{i}"),
            material="tick_white",
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            name=f"tick_{i}"
        )
    
    # Add end stops
    stop_left, stop_right = build_end_stops()
    panel.visual(
        mesh_from_cadquery(stop_left, "stop_left"),
        material="stop_red",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="stop_left"
    )
    panel.visual(
        mesh_from_cadquery(stop_right, "stop_right"),
        material="stop_red",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="stop_right"
    )
    
    # Build the fader cap (slider)
    fader_cap = model.part("fader_cap")
    cap_shape = build_fader_cap()
    fader_cap.visual(
        mesh_from_cadquery(cap_shape, "fader_cap_body"),
        material="cap_black",
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),  # Positioned through the slot
        name="fader_cap_body"
    )
    
    # Add a small indicator line on the cap for visual reference
    indicator = (
        cq.Workplane("XY")
        .box(0.012, 0.001, 0.001)
        .translate((0, 0, 0.008))
    )
    fader_cap.visual(
        mesh_from_cadquery(indicator, "cap_indicator"),
        material="tick_white",
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        name="cap_indicator"
    )
    
    # Create prismatic joint for fader slider
    # Travel range: -0.04 to +0.04 meters (80mm total travel)
    model.articulation(
        "panel_to_fader",
        ArticulationType.PRISMATIC,
        parent=panel,
        child=fader_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Center of the slot
        axis=(1.0, 0.0, 0.0),  # Slide along X-axis
        motion_limits=MotionLimits(
            lower=-0.04,  # Left end stop
            upper=0.04,   # Right end stop
            effort=10.0,
            velocity=0.5,
        ),
    )
    
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    
    panel = object_model.get_part("panel")
    fader_cap = object_model.get_part("fader_cap")
    slider_joint = object_model.get_articulation("panel_to_fader")
    
    # Test 1: Check that panel and fader are properly defined
    ctx.check("panel_exists", panel is not None, "Panel part not found")
    ctx.check("fader_cap_exists", fader_cap is not None, "Fader cap part not found")
    ctx.check("slider_joint_exists", slider_joint is not None, "Slider joint not found")
    
    # Test 2: Check prismatic joint axis and limits
    ctx.check(
        "joint_axis_correct",
        slider_joint.axis == (1.0, 0.0, 0.0),
        f"Expected axis (1,0,0), got {slider_joint.axis}"
    )
    ctx.check(
        "joint_limits_symmetric",
        slider_joint.motion_limits.lower == -0.04 and slider_joint.motion_limits.upper == 0.04,
        f"Expected limits (-0.04, 0.04), got ({slider_joint.motion_limits.lower}, {slider_joint.motion_limits.upper})"
    )
    
    # Test 3: Check that cap is seated through panel opening (allow overlap)
    # The fader cap shaft intentionally passes through the panel slot
    ctx.allow_overlap(
        "panel",
        "fader_cap",
        reason="Fader cap shaft intentionally passes through panel slot opening",
        elem_a="panel_body",
        elem_b="fader_cap_body",
    )
    
    # Test 4: Verify cap maintains contact/seating with panel at rest position
    with ctx.pose({slider_joint: 0.0}):
        ctx.expect_contact(
            panel,
            fader_cap,
            elem_a="panel_body",
            elem_b="fader_cap_body",
            name="cap_seated_at_center",
        )
        # Check cap is within panel bounds on Y axis
        ctx.expect_within(
            fader_cap,
            panel,
            axes="y",
            margin=0.02,
            name="cap_within_panel_y",
        )
        # Check cap shaft fits through slot (overlap on Y axis)
        ctx.expect_overlap(
            panel,
            fader_cap,
            axes="y",
            min_overlap=0.002,
            name="cap_shaft_fits_slot_width",
        )
    
    # Test 5: Verify cap moves correctly to the right (positive direction)
    with ctx.pose({slider_joint: 0.04}):
        pos_right = ctx.part_world_position(fader_cap)
        ctx.check(
            "cap_moves_right",
            pos_right is not None and pos_right[0] > 0.03,
            f"Cap should move right, position: {pos_right}"
        )
        ctx.expect_within(
            fader_cap,
            panel,
            axes="y",
            margin=0.02,
            name="cap_within_panel_y_at_right",
        )
    
    # Test 6: Verify cap moves correctly to the left (negative direction)
    with ctx.pose({slider_joint: -0.04}):
        pos_left = ctx.part_world_position(fader_cap)
        ctx.check(
            "cap_moves_left",
            pos_left is not None and pos_left[0] < -0.03,
            f"Cap should move left, position: {pos_left}"
        )
        ctx.expect_within(
            fader_cap,
            panel,
            axes="y",
            margin=0.02,
            name="cap_within_panel_y_at_left",
        )
    
    # Test 7: Check that cap stays within panel on non-slide axes during travel
    ctx.expect_within(
        fader_cap,
        panel,
        axes="y",
        margin=0.02,
        name="cap_centered_on_y_axis",
    )
    
    # Test 8: Verify the fader has proper travel range
    with ctx.pose({slider_joint: 0.0}):
        center_pos = ctx.part_world_position(fader_cap)
    with ctx.pose({slider_joint: 0.04}):
        right_pos = ctx.part_world_position(fader_cap)
    with ctx.pose({slider_joint: -0.04}):
        left_pos = ctx.part_world_position(fader_cap)
    
    ctx.check(
        "travel_range_correct",
        right_pos[0] - left_pos[0] > 0.075,  # At least 75mm travel
        f"Expected ~80mm travel, got {right_pos[0] - left_pos[0]}"
    )
    
    return ctx.report()


object_model = build_object_model()
