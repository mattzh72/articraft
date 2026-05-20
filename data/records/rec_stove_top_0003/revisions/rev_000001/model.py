from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_induction_cooktop", assets=ASSETS)

    stone = model.material("stone_quartz", rgba=(0.80, 0.80, 0.78, 1.0))
    glass_black = model.material("glass_black", rgba=(0.08, 0.08, 0.09, 1.0))
    zone_gray = model.material("zone_gray", rgba=(0.35, 0.36, 0.39, 0.65))
    pod_gray = model.material("pod_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    button_gray = model.material("button_gray", rgba=(0.62, 0.64, 0.67, 1.0))

    island_top = model.part("island_top")

    top_width = 1.60
    top_depth = 0.90
    top_thickness = 0.03

    opening_width = 0.764
    opening_depth = 0.504
    support_ledge_width = 0.018
    support_ledge_thickness = 0.004
    glass_width = 0.760
    glass_depth = 0.500
    glass_thickness = 0.008
    glass_support_z = top_thickness - glass_thickness

    side_band = (top_width - opening_width) / 2.0
    end_band = (top_depth - opening_depth) / 2.0

    island_top.visual(
        Box((side_band, top_depth, top_thickness)),
        origin=Origin(xyz=(-(opening_width / 2.0 + side_band / 2.0), 0.0, top_thickness / 2.0)),
        material=stone,
        name="left_counter_span",
    )
    island_top.visual(
        Box((side_band, top_depth, top_thickness)),
        origin=Origin(xyz=((opening_width / 2.0 + side_band / 2.0), 0.0, top_thickness / 2.0)),
        material=stone,
        name="right_counter_span",
    )
    island_top.visual(
        Box((opening_width, end_band, top_thickness)),
        origin=Origin(xyz=(0.0, -(opening_depth / 2.0 + end_band / 2.0), top_thickness / 2.0)),
        material=stone,
        name="front_counter_span",
    )
    island_top.visual(
        Box((opening_width, end_band, top_thickness)),
        origin=Origin(xyz=(0.0, (opening_depth / 2.0 + end_band / 2.0), top_thickness / 2.0)),
        material=stone,
        name="rear_counter_span",
    )

    ledge_z = glass_support_z - support_ledge_thickness / 2.0
    inner_ledge_span_x = opening_width - 2.0 * support_ledge_width
    inner_ledge_span_y = opening_depth - 2.0 * support_ledge_width

    island_top.visual(
        Box((support_ledge_width, opening_depth, support_ledge_thickness)),
        origin=Origin(xyz=(-(opening_width / 2.0 - support_ledge_width / 2.0), 0.0, ledge_z)),
        material=stone,
        name="left_support_ledge",
    )
    island_top.visual(
        Box((support_ledge_width, opening_depth, support_ledge_thickness)),
        origin=Origin(xyz=((opening_width / 2.0 - support_ledge_width / 2.0), 0.0, ledge_z)),
        material=stone,
        name="right_support_ledge",
    )
    island_top.visual(
        Box((inner_ledge_span_x, support_ledge_width, support_ledge_thickness)),
        origin=Origin(xyz=(0.0, -(opening_depth / 2.0 - support_ledge_width / 2.0), ledge_z)),
        material=stone,
        name="front_support_ledge",
    )
    island_top.visual(
        Box((inner_ledge_span_x, support_ledge_width, support_ledge_thickness)),
        origin=Origin(xyz=(0.0, (opening_depth / 2.0 - support_ledge_width / 2.0), ledge_z)),
        material=stone,
        name="rear_support_ledge",
    )

    cooktop = model.part("cooktop_glass")
    cooktop.visual(
        Box((glass_width, glass_depth, glass_thickness)),
        origin=Origin(xyz=(0.0, 0.0, glass_thickness / 2.0)),
        material=glass_black,
        name="glass_panel",
    )
    model.articulation(
        "island_to_cooktop",
        ArticulationType.FIXED,
        parent=island_top,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, glass_support_z)),
    )

    zone_specs = (
        ("zone_rear_left", -0.152, 0.115, 0.094),
        ("zone_rear_right", 0.152, 0.115, 0.094),
        ("zone_front_left", -0.152, -0.115, 0.084),
        ("zone_front_right", 0.152, -0.115, 0.084),
    )
    zone_thickness = 0.0004
    for name, x_pos, y_pos, radius in zone_specs:
        zone = model.part(name)
        zone.visual(
            Cylinder(radius=radius, length=zone_thickness),
            origin=Origin(xyz=(0.0, 0.0, zone_thickness / 2.0)),
            material=zone_gray,
            name="zone_disc",
        )
        model.articulation(
            f"cooktop_to_{name}",
            ArticulationType.FIXED,
            parent=cooktop,
            child=zone,
            origin=Origin(xyz=(x_pos, y_pos, glass_thickness)),
        )

    pod_width = 0.054
    pod_depth = 0.054
    pod_height = 0.014
    wall_thickness = 0.0035
    top_plate_thickness = 0.002
    bottom_plate_thickness = 0.002
    hole_size = 0.015
    cap_size = 0.014
    stem_size = 0.010
    shoulder_size = 0.019
    button_travel = 0.002

    pod_x = glass_width / 2.0 - pod_width / 2.0 - 0.018
    pod_y = -(glass_depth / 2.0 - pod_depth / 2.0 - 0.018)

    control_pod = model.part("control_pod")
    control_pod.visual(
        Box((pod_width, pod_depth, bottom_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_plate_thickness / 2.0)),
        material=pod_gray,
        name="bottom_plate",
    )
    wall_height = pod_height - bottom_plate_thickness
    wall_z = bottom_plate_thickness + wall_height / 2.0
    control_pod.visual(
        Box((wall_thickness, pod_depth, wall_height)),
        origin=Origin(xyz=(-(pod_width / 2.0 - wall_thickness / 2.0), 0.0, wall_z)),
        material=pod_gray,
        name="left_wall",
    )
    control_pod.visual(
        Box((wall_thickness, pod_depth, wall_height)),
        origin=Origin(xyz=((pod_width / 2.0 - wall_thickness / 2.0), 0.0, wall_z)),
        material=pod_gray,
        name="right_wall",
    )
    control_pod.visual(
        Box((pod_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -(pod_depth / 2.0 - wall_thickness / 2.0), wall_z)),
        material=pod_gray,
        name="front_wall",
    )
    control_pod.visual(
        Box((pod_width - 2.0 * wall_thickness, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, (pod_depth / 2.0 - wall_thickness / 2.0), wall_z)),
        material=pod_gray,
        name="rear_wall",
    )

    frame_side = (pod_width - 2.0 * hole_size - 0.004) / 2.0
    frame_end = (pod_depth - 2.0 * hole_size - 0.004) / 2.0
    bezel_z = pod_height - top_plate_thickness / 2.0
    hole_offset_x = hole_size / 2.0 + 0.004 / 2.0
    hole_offset_y = hole_size / 2.0 + 0.004 / 2.0

    control_pod.visual(
        Box((frame_side, pod_depth, top_plate_thickness)),
        origin=Origin(xyz=(-(pod_width / 2.0 - frame_side / 2.0), 0.0, bezel_z)),
        material=pod_gray,
        name="left_bezel",
    )
    control_pod.visual(
        Box((0.004, pod_depth, top_plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bezel_z)),
        material=pod_gray,
        name="center_bezel_rib",
    )
    control_pod.visual(
        Box((frame_side, pod_depth, top_plate_thickness)),
        origin=Origin(xyz=((pod_width / 2.0 - frame_side / 2.0), 0.0, bezel_z)),
        material=pod_gray,
        name="right_bezel",
    )
    for horiz_name, x_pos, y_pos, size_y in (
        ("front_left_bezel", -hole_offset_x, pod_depth / 2.0 - frame_end / 2.0, frame_end),
        ("front_right_bezel", hole_offset_x, pod_depth / 2.0 - frame_end / 2.0, frame_end),
        ("middle_left_bezel", -hole_offset_x, 0.0, 0.004),
        ("middle_right_bezel", hole_offset_x, 0.0, 0.004),
        ("rear_left_bezel", -hole_offset_x, -(pod_depth / 2.0 - frame_end / 2.0), frame_end),
        ("rear_right_bezel", hole_offset_x, -(pod_depth / 2.0 - frame_end / 2.0), frame_end),
    ):
        control_pod.visual(
            Box((hole_size, size_y, top_plate_thickness)),
            origin=Origin(xyz=(x_pos, y_pos, bezel_z)),
            material=pod_gray,
            name=horiz_name,
        )

    model.articulation(
        "cooktop_to_control_pod",
        ArticulationType.FIXED,
        parent=cooktop,
        child=control_pod,
        origin=Origin(xyz=(pod_x, pod_y, glass_thickness)),
    )

    button_specs = (
        ("button_front_left", -hole_offset_x, -hole_offset_y),
        ("button_front_right", hole_offset_x, -hole_offset_y),
        ("button_rear_left", -hole_offset_x, hole_offset_y),
        ("button_rear_right", hole_offset_x, hole_offset_y),
    )
    for name, x_pos, y_pos in button_specs:
        button = model.part(name)
        button.visual(
            Box((cap_size, cap_size, 0.002)),
            origin=Origin(xyz=(0.0, 0.0, 0.001)),
            material=button_gray,
            name="cap",
        )
        button.visual(
            Box((stem_size, stem_size, 0.0045)),
            origin=Origin(xyz=(0.0, 0.0, -0.00125)),
            material=button_gray,
            name="upper_stem",
        )
        button.visual(
            Box((shoulder_size, shoulder_size, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, -0.0025)),
            material=button_gray,
            name="shoulder_stop",
        )
        button.visual(
            Box((stem_size, stem_size, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.0065)),
            material=button_gray,
            name="lower_stem",
        )
        button.visual(
            Box((shoulder_size, shoulder_size, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, -0.0100)),
            material=button_gray,
            name="bottom_stop",
        )
        model.articulation(
            f"control_pod_to_{name}",
            ArticulationType.PRISMATIC,
            parent=control_pod,
            child=button,
            origin=Origin(xyz=(x_pos, y_pos, pod_height)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.05,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    island_top = object_model.get_part("island_top")
    cooktop = object_model.get_part("cooktop_glass")
    control_pod = object_model.get_part("control_pod")
    zones = {
        name: object_model.get_part(name)
        for name in (
            "zone_rear_left",
            "zone_rear_right",
            "zone_front_left",
            "zone_front_right",
        )
    }
    buttons = {
        name: object_model.get_part(name)
        for name in (
            "button_front_left",
            "button_front_right",
            "button_rear_left",
            "button_rear_right",
        )
    }
    button_joints = {
        name: object_model.get_articulation(f"control_pod_to_{name}")
        for name in buttons
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(cooktop, island_top, name="cooktop_supported_by_countertop")
    ctx.expect_within(control_pod, cooktop, axes="xy", margin=0.0, name="control_pod_within_cooktop_plan")
    ctx.expect_contact(control_pod, cooktop, name="control_pod_mounted_to_glass")

    island_aabb = ctx.part_world_aabb(island_top)
    cooktop_aabb = ctx.part_world_aabb(cooktop)
    if island_aabb is not None and cooktop_aabb is not None:
        island_top_z = island_aabb[1][2]
        cooktop_top_z = cooktop_aabb[1][2]
        ctx.check(
            "cooktop_flush_with_countertop",
            abs(cooktop_top_z - island_top_z) <= 1e-6,
            details=f"cooktop top z={cooktop_top_z:.6f}, countertop top z={island_top_z:.6f}",
        )
    else:
        ctx.fail("cooktop_flush_with_countertop", "missing world AABB for island or cooktop")

    pod_pos = ctx.part_world_position(control_pod)
    if pod_pos is not None:
        ctx.check(
            "control_pod_front_right_position",
            pod_pos[0] > 0.25 and pod_pos[1] < -0.15,
            details=f"control pod position={pod_pos}",
        )
    else:
        ctx.fail("control_pod_front_right_position", "missing control pod world position")

    for zone_name, zone in zones.items():
        ctx.expect_contact(zone, cooktop, name=f"{zone_name}_printed_on_glass")
        ctx.expect_within(zone, cooktop, axes="xy", margin=0.0, name=f"{zone_name}_inside_glass_footprint")

    zone_positions = {name: ctx.part_world_position(zone) for name, zone in zones.items()}
    for zone_name, zone_pos in zone_positions.items():
        if zone_pos is None:
            ctx.fail(f"{zone_name}_position_available", "missing zone world position")
            continue
        x_pos, y_pos, _ = zone_pos
        if "left" in zone_name:
            ctx.check(f"{zone_name}_left_side", x_pos < 0.0, details=f"x={x_pos:.6f}")
        else:
            ctx.check(f"{zone_name}_right_side", x_pos > 0.0, details=f"x={x_pos:.6f}")
        if "front" in zone_name:
            ctx.check(f"{zone_name}_front_row", y_pos < 0.0, details=f"y={y_pos:.6f}")
        else:
            ctx.check(f"{zone_name}_rear_row", y_pos > 0.0, details=f"y={y_pos:.6f}")

    for button_name, button in buttons.items():
        joint = button_joints[button_name]
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{button_name}_limits_present", "button joint is missing finite prismatic limits")
            continue

        with ctx.pose({joint: limits.lower}):
            ctx.expect_contact(button, control_pod, name=f"{button_name}_rest_support_contact")
            ctx.expect_within(button, control_pod, axes="xy", margin=0.0, name=f"{button_name}_rest_within_pod")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_name}_rest_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{button_name}_rest_no_floating")
            rest_pos = ctx.part_world_position(button)

        with ctx.pose({joint: limits.upper}):
            ctx.expect_contact(button, control_pod, name=f"{button_name}_pressed_support_contact")
            ctx.expect_within(button, control_pod, axes="xy", margin=0.0, name=f"{button_name}_pressed_within_pod")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{button_name}_pressed_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{button_name}_pressed_no_floating")
            pressed_pos = ctx.part_world_position(button)

        if rest_pos is None or pressed_pos is None:
            ctx.fail(f"{button_name}_motion_measured", "missing button world position in one or both poses")
            continue

        delta_x = abs(rest_pos[0] - pressed_pos[0])
        delta_y = abs(rest_pos[1] - pressed_pos[1])
        delta_z = rest_pos[2] - pressed_pos[2]
        ctx.check(
            f"{button_name}_normal_motion_only",
            delta_x <= 1e-6 and delta_y <= 1e-6 and abs(delta_z - limits.upper) <= 1e-6,
            details=(
                f"delta_x={delta_x:.6f}, delta_y={delta_y:.6f}, "
                f"delta_z={delta_z:.6f}, expected={limits.upper:.6f}"
            ),
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="button_pose_sweep_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
