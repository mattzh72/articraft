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
    Inertial,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_rectangular_guide")

    # Materials with contrasting colors
    sleeve_material = model.material("sleeve_dark", rgba=(0.25, 0.27, 0.30, 1.0))
    inner_material = model.material("inner_blue", rgba=(0.20, 0.45, 0.75, 1.0))
    tab_material = model.material("tab_red", rgba=(0.70, 0.20, 0.20, 1.0))
    stop_material = model.material("stop_yellow", rgba=(0.85, 0.75, 0.15, 1.0))

    # ========== FIXED SLEEVE (root part) ==========
    sleeve = model.part("sleeve")

    # Create hollow rectangular sleeve using CadQuery
    sleeve_length = 0.300  # 300mm long
    sleeve_width = 0.040   # 40mm Y dimension
    sleeve_height = 0.020  # 20mm Z dimension
    wall_thickness = 0.002  # 2mm wall

    # Create outer box with chamfered edges
    outer = cq.Workplane("XY").box(sleeve_length, sleeve_width, sleeve_height)

    # Add chamfers to outer edges (1mm chamfer)
    try:
        outer = outer.edges("|Z").chamfer(0.001)
    except Exception:
        pass  # If chamfer fails, continue without it

    # Cut inner cavity
    inner_length = sleeve_length - 0.004
    inner_width = sleeve_width - 2 * wall_thickness
    inner_height = sleeve_height - 2 * wall_thickness

    hollow_sleeve = (
        outer
        .faces(">Z")
        .workplane()
        .workplane(offset=-wall_thickness)
        .rect(inner_length, inner_width)
        .cutBlind(-inner_height)
    )

    sleeve.visual(
        mesh_from_cadquery(hollow_sleeve, "sleeve_body"),
        origin=Origin(xyz=(sleeve_length / 2, 0.0, sleeve_height / 2)),
        material=sleeve_material,
        name="sleeve_shell",
    )

    # Add decorative slots on top face as separate small visuals
    num_slots = 8
    slot_spacing = (sleeve_length - 0.040) / num_slots
    slot_width = 0.003
    slot_depth = 0.0015  # Shallow slot depth

    for i in range(num_slots):
        x_pos = -sleeve_length / 2 + 0.020 + i * slot_spacing
        sleeve.visual(
            Box((slot_width, sleeve_width + 0.002, slot_depth)),
            origin=Origin(xyz=(sleeve_length / 2 + x_pos, 0.0, sleeve_height + slot_depth / 2)),
            material=sleeve_material,
            name=f"slot_{i}",
        )

    # Back plate with travel stop (yellow) - at the back (-X) end
    sleeve.visual(
        Box((0.004, inner_width, inner_height)),
        origin=Origin(xyz=(-0.002, 0.0, sleeve_height / 2)),
        material=stop_material,
        name="travel_stop",
    )

    sleeve.inertial = Inertial.from_geometry(
        Box((sleeve_length, sleeve_width, sleeve_height)),
        mass=0.8,
        origin=Origin(xyz=(sleeve_length / 2, 0.0, sleeve_height / 2)),
    )

    # ========== SLIDING INNER BAR ==========
    inner_bar = model.part("inner_bar")

    # Inner bar dimensions - fits inside sleeve with 2mm clearance total
    bar_length = 0.600  # 600mm total - ensures minimum insertion at full extension
    bar_width = 0.036   # 36mm Y (4mm total clearance in sleeve)
    bar_height = 0.016  # 16mm Z (4mm total clearance in sleeve)

    # Bar visual: center at part frame origin
    bar_body_visual = (
        cq.Workplane("XY")
        .box(bar_length, bar_width, bar_height)
        .edges("|X").chamfer(0.001)  # Chamfer edges along length
    )

    inner_bar.visual(
        mesh_from_cadquery(bar_body_visual, "bar_body_geom"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=inner_material,
        name="bar_body",
    )

    # Front pull tab (red, contrasting color) - at the front (+X) end of the bar
    tab_width = 0.050
    tab_height = 0.030
    tab_depth = 0.008

    inner_bar.visual(
        Box((tab_depth, tab_width, tab_height)),
        origin=Origin(xyz=(bar_length / 2 + tab_depth / 2, 0.0, (tab_height - bar_height) / 2)),
        material=tab_material,
        name="pull_tab",
    )

    # Add grip ridges to pull tab
    grip_spacing = 0.006
    for i in range(4):
        grip_y = -0.015 + i * grip_spacing
        inner_bar.visual(
            Box((tab_depth + 0.002, 0.004, 0.002)),
            origin=Origin(xyz=(bar_length / 2 + tab_depth / 2, grip_y, tab_height + 0.001)),
            material=tab_material,
            name=f"grip_{i}",
        )

    # Stop feature on the inner bar (back end, -X) - yellow
    inner_bar.visual(
        Box((0.006, bar_width - 0.002, bar_height + 0.002)),
        origin=Origin(xyz=(-bar_length / 2 + 0.003, 0.0, 0.0)),
        material=stop_material,
        name="bar_stop",
    )

    # Inertial for inner bar
    total_length = bar_length + tab_depth
    inner_bar.inertial = Inertial.from_geometry(
        Box((total_length, bar_width, bar_height)),
        mass=0.5,
        origin=Origin(xyz=(tab_depth / 2, 0.0, 0.0)),
    )

    # ========== PRISMATIC JOINT ==========
    # Joint at sleeve front face (entry point)
    travel_distance = 0.200  # 200mm travel

    # Joint origin in sleeve part frame (at sleeve front face)
    joint_x_in_sleeve = sleeve_length / 2

    model.articulation(
        "sleeve_to_bar",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=inner_bar,
        origin=Origin(xyz=(joint_x_in_sleeve, 0.0, sleeve_height / 2)),
        axis=(1.0, 0.0, 0.0),  # Slides along +X
        motion_limits=MotionLimits(
            lower=0.0,
            upper=travel_distance,
            effort=50.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    inner_bar = object_model.get_part("inner_bar")
    sleeve_to_bar = object_model.get_articulation("sleeve_to_bar")

    # Test 1: At rest (q=0), inner bar should overlap with sleeve in X (insertion)
    ctx.expect_overlap(
        inner_bar,
        sleeve,
        axes="x",
        min_overlap=0.250,
        elem_a="bar_body",
        elem_b="sleeve_shell",
        name="bar remains inserted in sleeve at rest",
    )

    # Test 2: At rest, bar should be centered in sleeve (Y and Z axes)
    ctx.expect_within(
        inner_bar,
        sleeve,
        axes="yz",
        margin=0.003,
        elem_a="bar_body",
        elem_b="sleeve_shell",
        name="bar centered in sleeve at rest",
    )

    # Test 3: At full extension, bar should still be partly inserted
    with ctx.pose({sleeve_to_bar: sleeve_to_bar.motion_limits.upper}):
        ctx.expect_overlap(
            inner_bar,
            sleeve,
            axes="x",
            min_overlap=0.080,
            elem_a="bar_body",
            elem_b="sleeve_shell",
            name="bar still inserted at full extension",
        )
        ctx.expect_within(
            inner_bar,
            sleeve,
            axes="yz",
            margin=0.003,
            elem_a="bar_body",
            elem_b="sleeve_shell",
            name="bar still centered at full extension",
        )

    # Test 4: Check that the bar actually extends along +X
    rest_pos = ctx.part_world_position(inner_bar)
    with ctx.pose({sleeve_to_bar: sleeve_to_bar.motion_limits.upper}):
        extended_pos = ctx.part_world_position(inner_bar)

    ctx.check(
        "bar extends along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.05,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    # Test 5: Check pull tab exists
    pull_tab = inner_bar.get_visual("pull_tab")
    ctx.check(
        "pull tab exists",
        pull_tab is not None,
        details="Pull tab visual not found on inner_bar",
    )

    # Test 6: Allow intentional overlap between bar and sleeve (seated/nested fit)
    ctx.allow_overlap(
        "sleeve",
        "inner_bar",
        reason="Inner bar is intentionally nested inside the sleeve for telescoping action. "
               "The bar slides inside the sleeve with small clearance.",
        elem_a="sleeve_shell",
        elem_b="bar_body",
    )

    # Test 7: Verify travel stop is present
    travel_stop = sleeve.get_visual("travel_stop")
    ctx.check(
        "travel stop exists",
        travel_stop is not None,
        details="Travel stop visual not found on sleeve",
    )

    # Test 8: Verify bar stop is present
    bar_stop = inner_bar.get_visual("bar_stop")
    ctx.check(
        "bar stop exists",
        bar_stop is not None,
        details="Bar stop visual not found on inner_bar",
    )

    return ctx.report()


object_model = build_object_model()
