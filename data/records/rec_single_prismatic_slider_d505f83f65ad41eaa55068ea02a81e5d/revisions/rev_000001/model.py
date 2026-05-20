from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_drawer_slide")

    # Materials
    dark_gray_metal = Material(name="dark_gray_metal", color=(0.2, 0.2, 0.2))
    light_gray_metal = Material(name="light_gray_metal", color=(0.8, 0.8, 0.8))
    black_plastic = Material(name="black_plastic", color=(0.1, 0.1, 0.1))
    blue_plastic = Material(name="blue_plastic", color=(0.2, 0.4, 0.8))

    # ==================== Outer Rail (Root Part) ====================
    outer_rail = model.part("outer_rail")

    # Outer rail dimensions
    rail_length = 0.5  # X axis length (m)
    rail_outer_width = 0.10  # Y axis total width (m)
    rail_wall_thickness = 0.01  # Wall thickness (m)
    rail_inner_height = 0.04  # Z axis inner height (m)
    rail_back_wall_thickness = 0.01  # Z thickness of back wall (m)

    # Create U-channel profile in YZ plane
    rail_profile = (
        cq.Workplane("YZ")
        # Back wall (full width, thickness rail_back_wall_thickness)
        .moveTo(-rail_outer_width/2, 0)
        .lineTo(rail_outer_width/2, 0)
        .lineTo(rail_outer_width/2, rail_back_wall_thickness)
        .lineTo(-rail_outer_width/2, rail_back_wall_thickness)
        .close()
        # Left side wall
        .moveTo(-rail_outer_width/2, rail_back_wall_thickness)
        .lineTo(-rail_outer_width/2 + rail_wall_thickness, rail_back_wall_thickness)
        .lineTo(-rail_outer_width/2 + rail_wall_thickness, rail_back_wall_thickness + rail_inner_height)
        .lineTo(-rail_outer_width/2, rail_back_wall_thickness + rail_inner_height)
        .close()
        # Right side wall
        .moveTo(rail_outer_width/2 - rail_wall_thickness, rail_back_wall_thickness)
        .lineTo(rail_outer_width/2, rail_back_wall_thickness)
        .lineTo(rail_outer_width/2, rail_back_wall_thickness + rail_inner_height)
        .lineTo(rail_outer_width/2 - rail_wall_thickness, rail_back_wall_thickness + rail_inner_height)
        .close()
    )

    # Extrude along X axis for full rail length
    outer_rail_solid = rail_profile.extrude(rail_length)

    # Add side grooves (0.005m deep, 0.005m wide, centered vertically)
    groove_width = 0.005
    groove_z_center = rail_back_wall_thickness + rail_inner_height / 2

    # Left side groove
    left_groove = (
        cq.Workplane("YZ")
        .rect(rail_wall_thickness, groove_width)
        .translate((-rail_outer_width/2 + rail_wall_thickness/2, groove_z_center))
        .extrude(rail_length)
    )
    outer_rail_solid = outer_rail_solid.cut(left_groove)

    # Right side groove
    right_groove = (
        cq.Workplane("YZ")
        .rect(rail_wall_thickness, groove_width)
        .translate((rail_outer_width/2 - rail_wall_thickness/2, groove_z_center))
        .extrude(rail_length)
    )
    outer_rail_solid = outer_rail_solid.cut(right_groove)

    # Add end stops to outer rail solid (ensures mesh connectivity)
    end_stop_thickness = 0.01  # X thickness
    end_stop_y_size = rail_outer_width - 2 * rail_wall_thickness  # 0.08m
    end_stop_z_size = rail_inner_height  # 0.04m
    end_stop_z_center = rail_back_wall_thickness + end_stop_z_size / 2  # 0.03m

    # Back end stop (at X=0.01, just inside the rail channel)
    back_stop_x = 0.01
    back_stop = (
        cq.Workplane("XY")
        .box(end_stop_thickness, end_stop_y_size, end_stop_z_size)
        .translate((back_stop_x + end_stop_thickness / 2, 0, end_stop_z_center))
    )
    # Front end stop (at X=0.49, near end of rail)
    front_stop_x = 0.49 - end_stop_thickness
    front_stop = (
        cq.Workplane("XY")
        .box(end_stop_thickness, end_stop_y_size, end_stop_z_size)
        .translate((front_stop_x + end_stop_thickness / 2, 0, end_stop_z_center))
    )

    outer_rail_solid = outer_rail_solid.union(back_stop).union(front_stop)

    # Add outer rail body visual (now includes end stops)
    # The CadQuery solid has X from 0 to 0.5 in local coords, center at 0.25
    # To have world X range [0, 0.5], set visual origin X = 0 (so world center = 0 + 0.25 = 0.25)
    # For Z: local center is 0.025, want world channel center at 0.03, so visual origin Z = 0.03 - 0.025 = 0.005
    outer_rail.visual(
        mesh_from_cadquery(outer_rail_solid, "outer_rail_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),  # X=0 for correct X range
        name="outer_rail_body",
        material=dark_gray_metal,
    )

    # ==================== Inner Carriage (Moving Part) ====================
    inner_carriage = model.part("inner_carriage")

    # Carriage dimensions
    carriage_length = 0.2  # X axis length (m)
    carriage_y_size = (rail_outer_width - 2*rail_wall_thickness) - 0.004  # 0.076m (2mm clearance per side)
    carriage_z_size = rail_inner_height - 0.004  # 0.036m (2mm clearance top/bottom)

    # Create carriage body with roller mounts as one solid
    # Carriage local origin at center of carriage body
    # The carriage will be positioned so it's centered in Y and Z within the rail channel
    carriage_solid = cq.Workplane("XY").box(carriage_length, carriage_y_size, carriage_z_size)

    # Roller dimensions and positions
    roller_radius = 0.004
    roller_height = 0.01  # Along Y axis
    # Rollers should be positioned at the Y edges of the carriage (in the grooves)
    roller_y_offset = carriage_y_size / 2  # Y position at edge of carriage body
    roller_z_pos = 0.0  # Z center of carriage (carriage is centered in Z)
    roller_x_positions = [-0.05, 0.05]  # Local X positions along carriage (centered)

    # Add roller mounts (small cylinders that connect rollers to carriage)
    for x_pos in roller_x_positions:
        # Left side roller mount
        left_mount = (
            cq.Workplane("XY")
            .cylinder(roller_height, roller_radius)
            .rotate((0,0,0), (1,0,0), 90)  # Rotate so cylinder axis is along Y
            .translate((x_pos, -roller_y_offset, roller_z_pos))
        )
        carriage_solid = carriage_solid.union(left_mount)

        # Right side roller mount
        right_mount = (
            cq.Workplane("XY")
            .cylinder(roller_height, roller_radius)
            .rotate((0,0,0), (1,0,0), 90)  # Rotate so cylinder axis is along Y
            .translate((x_pos, roller_y_offset, roller_z_pos))
        )
        carriage_solid = carriage_solid.union(right_mount)

    # Add main carriage visual (includes roller mounts)
    # Carriage local origin at center, positioned so carriage is centered in rail channel
    inner_carriage.visual(
        mesh_from_cadquery(carriage_solid, "carriage_body"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),  # Local origin at center of carriage
        name="carriage_body",
        material=light_gray_metal,
    )

    # Add visible roller blocks (positioned on the mounts, extending into rail grooves)
    for i, x_pos in enumerate(roller_x_positions):
        # Left side roller (Y negative, extends toward left groove)
        inner_carriage.visual(
            Cylinder(radius=roller_radius, length=roller_height),
            origin=Origin(xyz=(x_pos, -roller_y_offset, roller_z_pos)),
            name=f"roller_left_{i}",
            material=black_plastic,
        )
        # Right side roller (Y positive, extends toward right groove)
        inner_carriage.visual(
            Cylinder(radius=roller_radius, length=roller_height),
            origin=Origin(xyz=(x_pos, roller_y_offset, roller_z_pos)),
            name=f"roller_right_{i}",
            material=black_plastic,
        )

    # Handle tab (front of carriage, extending beyond carriage body)
    handle_depth = 0.02  # X size
    handle_width = 0.03  # Y size
    handle_height = 0.01  # Z size
    inner_carriage.visual(
        Box((handle_depth, handle_width, handle_height)),
        origin=Origin(xyz=(carriage_length/2 + handle_depth/2, 0.0, carriage_z_size/2 + handle_height/2)),
        name="handle_tab",
        material=blue_plastic,
    )

    # ==================== Prismatic Articulation ====================
    # Joint positioned at back of rail (X=0.11), centered in Y (0.0) and Z (rail channel center)
    # Rail channel center Z = rail_back_wall_thickness + rail_inner_height/2 = 0.01 + 0.02 = 0.03
    # Carriage is centered in Y and Z relative to joint
    joint_x = 0.11  # Position where carriage back face touches back end stop at q=0
    joint_z = rail_back_wall_thickness + rail_inner_height/2  # 0.03
    model.articulation(
        name="rail_to_carriage",
        articulation_type=ArticulationType.PRISMATIC,
        parent=outer_rail,
        child=inner_carriage,
        origin=Origin(xyz=(joint_x, 0.0, joint_z)),
        axis=(1.0, 0.0, 0.0),  # Along X axis
        motion_limits=MotionLimits(effort=10.0, velocity=0.5, lower=0.0, upper=0.19),  # 0.19m travel
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_rail = object_model.get_part("outer_rail")
    inner_carriage = object_model.get_part("inner_carriage")
    joint = object_model.get_articulation("rail_to_carriage")

    # Allow inner_carriage to be isolated (it's nested inside rail with clearances)
    ctx.allow_isolated_part(
        "inner_carriage",
        reason="Inner carriage is nested inside outer rail channel with clearances; connected via prismatic joint"
    )

    # Allow intentional overlap between nested carriage and rail
    ctx.allow_overlap(
        "inner_carriage",
        "outer_rail",
        reason="Inner carriage is intentionally nested inside outer rail channel with small clearances",
        elem_a="carriage_body",
        elem_b="outer_rail_body",
    )
    # Allow overlap for rollers (intentionally in rail grooves)
    for side in ["left", "right"]:
        for i in range(2):
            ctx.allow_overlap(
                "inner_carriage",
                "outer_rail",
                reason="Rollers are intentionally positioned inside rail grooves",
                elem_a=f"roller_{side}_{i}",
                elem_b="outer_rail_body",
            )

    # Test 1: Retracted position (q=0) - carriage at back of rail
    with ctx.pose({joint: 0.0}):
        # At q=0: carriage center at X=0.11, spans X=0.01 to 0.21
        # Rail spans X=0 to 0.5
        # Carriage should be contained within rail on YZ axes
        ctx.expect_within(
            inner_carriage,
            outer_rail,
            axes="yz",
            margin=0.003,  # Allow 3mm margin for clearances
            name="retracted_contained_yz"
        )
        # Carriage overlaps with rail on X axis (from X=0.01 to 0.21, rail from 0 to 0.5)
        ctx.expect_overlap(
            inner_carriage,
            outer_rail,
            axes="x",
            min_overlap=0.1,  # At least 0.1m overlap (handle extends to ~0.23)
            name="retracted_overlap_x"
        )

    # Test 2: Extended position (q=0.19) - carriage 0.19m forward
    with ctx.pose({joint: 0.19}):
        # At q=0.19: carriage center at X=0.3, spans X=0.2 to 0.4
        # Still contained on YZ
        ctx.expect_within(
            inner_carriage,
            outer_rail,
            axes="yz",
            margin=0.003,
            name="extended_contained_yz"
        )
        # Still retained in rail
        ctx.expect_overlap(
            inner_carriage,
            outer_rail,
            axes="x",
            min_overlap=0.1,
            name="extended_overlap_x"
        )

    # Test 3: Travel distance verification
    rest_pos = ctx.part_world_position(inner_carriage)
    with ctx.pose({joint: 0.19}):
        extended_pos = ctx.part_world_position(inner_carriage)
    ctx.check(
        "travel_distance",
        abs(extended_pos[0] - rest_pos[0] - 0.19) < 0.001,
        details=f"Travel: {extended_pos[0]-rest_pos[0]:.3f}m (expected 0.19m)"
    )

    # Test 4: Carriage moves in +X direction
    ctx.check(
        "carriage_moves_forward",
        extended_pos[0] > rest_pos[0] + 0.1,  # Should move at least 0.1m forward
        details=f"Movement: {extended_pos[0]-rest_pos[0]:.3f}m in +X direction"
    )

    # Test 5: All rollers present (4 total: 2 left, 2 right)
    for side in ["left", "right"]:
        for i in range(2):
            roller_name = f"roller_{side}_{i}"
            ctx.check(
                f"roller_exists_{roller_name}",
                inner_carriage.get_visual(roller_name) is not None,
            )

    # Test 6: Handle tab present and positioned at front of carriage
    with ctx.pose({joint: 0.0}):
        handle = inner_carriage.get_visual("handle_tab")
        ctx.check("handle_tab_exists", handle is not None)
        if handle:
            handle_aabb = ctx.part_element_world_aabb(inner_carriage, elem="handle_tab")
            carriage_aabb = ctx.part_element_world_aabb(inner_carriage, elem="carriage_body")
            # Handle should be in front of carriage body
            ctx.check(
                "handle_at_front",
                handle_aabb[0][0] >= carriage_aabb[1][0] - 0.001,
                details=f"Handle X min: {handle_aabb[0][0]:.3f}, Carriage X max: {carriage_aabb[1][0]:.3f}"
            )

    # Test 7: Verify outer rail body exists
    outer_rail_body = outer_rail.get_visual("outer_rail_body")
    ctx.check("outer_rail_body_exists", outer_rail_body is not None)

    return ctx.report()


object_model = build_object_model()