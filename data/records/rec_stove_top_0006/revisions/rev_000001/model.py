from __future__ import annotations

import math

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _arc_points_2d(
    cx: float,
    cy: float,
    radius: float,
    start_angle: float,
    end_angle: float,
    *,
    segments: int = 8,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(start_angle + (end_angle - start_angle) * index / segments),
            cy + radius * math.sin(start_angle + (end_angle - start_angle) * index / segments),
        )
        for index in range(segments + 1)
    ]


def _cooktop_slab_profile(
    width: float,
    depth: float,
    radius: float,
    *,
    round_front: bool = False,
    round_back: bool = False,
) -> list[tuple[float, float]]:
    half_width = width / 2.0
    half_depth = depth / 2.0
    radius = min(radius, half_width, half_depth)

    points: list[tuple[float, float]] = []

    if round_front:
        points.append((-half_width + radius, -half_depth))
        points.append((half_width - radius, -half_depth))
        points.extend(
            _arc_points_2d(
                half_width - radius,
                -half_depth + radius,
                radius,
                -math.pi / 2.0,
                0.0,
            )[1:]
        )
    else:
        points.append((-half_width, -half_depth))
        points.append((half_width, -half_depth))

    if round_back:
        points.append((half_width, half_depth - radius))
        points.extend(
            _arc_points_2d(
                half_width - radius,
                half_depth - radius,
                radius,
                0.0,
                math.pi / 2.0,
            )[1:]
        )
        points.append((-half_width + radius, half_depth))
        points.extend(
            _arc_points_2d(
                -half_width + radius,
                half_depth - radius,
                radius,
                math.pi / 2.0,
                math.pi,
            )[1:]
        )
    else:
        points.append((half_width, half_depth))
        points.append((-half_width, half_depth))

    if round_front:
        points.append((-half_width, -half_depth + radius))
        points.extend(
            _arc_points_2d(
                -half_width + radius,
                -half_depth + radius,
                radius,
                math.pi,
                3.0 * math.pi / 2.0,
            )[1:-1]
        )
    else:
        points.append((-half_width, -half_depth))

    return points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceramic_cooktop_with_push_buttons", assets=ASSETS)

    counter_stone = model.material("counter_stone", rgba=(0.77, 0.76, 0.73, 1.0))
    ceramic_glass = model.material("ceramic_glass", rgba=(0.07, 0.07, 0.08, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.48, 0.50, 0.53, 0.35))
    button_finish = model.material("button_finish", rgba=(0.63, 0.65, 0.68, 1.0))
    button_shadow = model.material("button_shadow", rgba=(0.18, 0.19, 0.21, 1.0))

    counter_width = 0.920
    counter_depth = 0.700
    counter_thickness = 0.040

    opening_width = 0.596
    opening_depth = 0.516

    cooktop_width = 0.590
    cooktop_depth = 0.510
    cooktop_thickness = 0.006
    cooktop_center_z = counter_thickness - cooktop_thickness / 2.0

    front_strip_depth = 0.052
    front_rail_depth = 0.012
    button_slot_depth = 0.028
    back_rail_depth = 0.012
    rear_panel_depth = cooktop_depth - front_strip_depth

    cooktop_front_edge = -cooktop_depth / 2.0
    front_rail_center_y = cooktop_front_edge + front_rail_depth / 2.0
    button_center_y = cooktop_front_edge + front_rail_depth + button_slot_depth / 2.0
    back_rail_center_y = (
        cooktop_front_edge + front_rail_depth + button_slot_depth + back_rail_depth / 2.0
    )
    rear_panel_center_y = cooktop_front_edge + front_strip_depth + rear_panel_depth / 2.0

    button_width = 0.094
    button_depth = button_slot_depth
    button_stem_height = cooktop_thickness
    button_cap_width = 0.088
    button_cap_depth = 0.022
    button_cap_height = 0.002
    button_travel = 0.002

    side_margin = 0.048
    web_width = 0.006
    button_centers_x = (-0.200, -0.100, 0.000, 0.100, 0.200)
    web_centers_x = (-0.150, -0.050, 0.050, 0.150)
    shoulder_center_offset = cooktop_width / 2.0 - side_margin / 2.0
    cooktop_corner_radius = 0.022

    support_thickness = 0.006
    support_center_z = counter_thickness - cooktop_thickness - support_thickness / 2.0
    support_side_width = 0.015
    support_front_depth = 0.015

    assembly = model.part("countertop_assembly")

    front_counter_band_depth = (counter_depth - opening_depth) / 2.0
    side_counter_band_width = (counter_width - opening_width) / 2.0

    assembly.visual(
        Box((counter_width, front_counter_band_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(counter_depth / 2.0 - front_counter_band_depth / 2.0),
                counter_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="counter_front_band",
    )
    assembly.visual(
        Box((counter_width, front_counter_band_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                counter_depth / 2.0 - front_counter_band_depth / 2.0,
                counter_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="counter_back_band",
    )
    assembly.visual(
        Box((side_counter_band_width, opening_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                -(counter_width / 2.0 - side_counter_band_width / 2.0),
                0.0,
                counter_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="counter_left_band",
    )
    assembly.visual(
        Box((side_counter_band_width, opening_depth, counter_thickness)),
        origin=Origin(
            xyz=(
                counter_width / 2.0 - side_counter_band_width / 2.0,
                0.0,
                counter_thickness / 2.0,
            )
        ),
        material=counter_stone,
        name="counter_right_band",
    )

    assembly.visual(
        Box((opening_width, support_front_depth, support_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(opening_depth / 2.0 - support_front_depth / 2.0),
                support_center_z,
            )
        ),
        material=button_shadow,
        name="support_front_lip",
    )
    assembly.visual(
        Box((opening_width, support_front_depth, support_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                opening_depth / 2.0 - support_front_depth / 2.0,
                support_center_z,
            )
        ),
        material=button_shadow,
        name="support_back_lip",
    )
    assembly.visual(
        Box((support_side_width, opening_depth, support_thickness)),
        origin=Origin(
            xyz=(
                -(opening_width / 2.0 - support_side_width / 2.0),
                0.0,
                support_center_z,
            )
        ),
        material=button_shadow,
        name="support_left_lip",
    )
    assembly.visual(
        Box((support_side_width, opening_depth, support_thickness)),
        origin=Origin(
            xyz=(
                opening_width / 2.0 - support_side_width / 2.0,
                0.0,
                support_center_z,
            )
        ),
        material=button_shadow,
        name="support_right_lip",
    )

    rear_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            _cooktop_slab_profile(
                cooktop_width,
                rear_panel_depth,
                cooktop_corner_radius,
                round_back=True,
            ),
            cooktop_thickness,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("cooktop_rear_panel.obj"),
    )
    front_rail_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            _cooktop_slab_profile(
                cooktop_width,
                front_rail_depth,
                cooktop_corner_radius,
                round_front=True,
            ),
            cooktop_thickness,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("cooktop_front_rail.obj"),
    )
    assembly.visual(
        rear_panel_mesh,
        origin=Origin(
            xyz=(0.0, rear_panel_center_y, counter_thickness - cooktop_thickness)
        ),
        material=ceramic_glass,
        name="cooktop_rear_panel",
    )
    assembly.visual(
        front_rail_mesh,
        origin=Origin(
            xyz=(0.0, front_rail_center_y, counter_thickness - cooktop_thickness)
        ),
        material=ceramic_glass,
        name="control_front_rail",
    )
    assembly.visual(
        Box((cooktop_width, back_rail_depth, cooktop_thickness)),
        origin=Origin(xyz=(0.0, back_rail_center_y, cooktop_center_z)),
        material=ceramic_glass,
        name="control_back_rail",
    )
    assembly.visual(
        Box((side_margin, button_slot_depth, cooktop_thickness)),
        origin=Origin(xyz=(-shoulder_center_offset, button_center_y, cooktop_center_z)),
        material=ceramic_glass,
        name="control_left_shoulder",
    )
    assembly.visual(
        Box((side_margin, button_slot_depth, cooktop_thickness)),
        origin=Origin(xyz=(shoulder_center_offset, button_center_y, cooktop_center_z)),
        material=ceramic_glass,
        name="control_right_shoulder",
    )
    for web_index, web_center_x in enumerate(web_centers_x, start=1):
        assembly.visual(
            Box((web_width, button_slot_depth, cooktop_thickness)),
            origin=Origin(xyz=(web_center_x, button_center_y, cooktop_center_z)),
            material=ceramic_glass,
            name=f"control_web_{web_index}",
        )

    zone_specs = (
        ("zone_front_left", -0.155, -0.060, 0.082),
        ("zone_front_right", 0.155, -0.060, 0.082),
        ("zone_rear_left", -0.155, 0.135, 0.092),
        ("zone_rear_right", 0.155, 0.135, 0.092),
    )
    for zone_name, zone_x, zone_y, radius in zone_specs:
        assembly.visual(
            Cylinder(radius=radius, length=0.0004),
            origin=Origin(xyz=(zone_x, zone_y, counter_thickness + 0.0002)),
            material=zone_mark,
            name=zone_name,
        )
        assembly.visual(
            Cylinder(radius=radius * 0.46, length=0.0004),
            origin=Origin(xyz=(zone_x, zone_y, counter_thickness + 0.00025)),
            material=zone_mark,
            name=f"{zone_name}_inner",
        )

    assembly.inertial = Inertial.from_geometry(
        Box((counter_width, counter_depth, counter_thickness)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, counter_thickness / 2.0)),
    )

    button_cap_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                button_cap_width,
                button_cap_depth,
                0.005,
                corner_segments=10,
            ),
            button_cap_height,
            cap=True,
            closed=True,
        ),
        ASSETS.mesh_path("cooktop_button_cap.obj"),
    )

    for button_index, button_center_x in enumerate(button_centers_x, start=1):
        button_part = model.part(f"button_{button_index}")
        button_part.visual(
            Box((button_width, button_depth, button_stem_height)),
            origin=Origin(xyz=(0.0, 0.0, button_stem_height / 2.0)),
            material=button_shadow,
            name="plunger",
        )
        button_part.visual(
            button_cap_mesh,
            origin=Origin(xyz=(0.0, 0.0, button_stem_height)),
            material=button_finish,
            name="button_cap",
        )
        button_part.inertial = Inertial.from_geometry(
            Box((button_width, button_depth, button_stem_height + button_cap_height)),
            mass=0.022,
            origin=Origin(
                xyz=(0.0, 0.0, (button_stem_height + button_cap_height) / 2.0)
            ),
        )
        model.articulation(
            f"button_{button_index}_press",
            ArticulationType.PRISMATIC,
            parent=assembly,
            child=button_part,
            origin=Origin(
                xyz=(
                    button_center_x,
                    button_center_y,
                    counter_thickness - cooktop_thickness,
                )
            ),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.03,
                lower=0.0,
                upper=button_travel,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    assembly = object_model.get_part("countertop_assembly")
    buttons = [object_model.get_part(f"button_{index}") for index in range(1, 6)]
    joints = [object_model.get_articulation(f"button_{index}_press") for index in range(1, 6)]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "single_root_countertop_assembly",
        len(object_model.root_parts()) == 1,
        details=f"root parts: {[part.name for part in object_model.root_parts()]}",
    )
    ctx.check(
        "five_button_parts_present",
        len(buttons) == 5,
        details=f"found {len(buttons)} button parts",
    )
    ctx.check(
        "five_button_joints_present",
        len(joints) == 5,
        details=f"found {len(joints)} button articulations",
    )

    assembly_aabb = ctx.part_world_aabb(assembly)
    if assembly_aabb is not None:
        assembly_min, assembly_max = assembly_aabb
        width = assembly_max[0] - assembly_min[0]
        depth = assembly_max[1] - assembly_min[1]
        thickness = assembly_max[2] - assembly_min[2]
        ctx.check(
            "countertop_overall_size_realistic",
            0.88 <= width <= 0.94 and 0.68 <= depth <= 0.72 and 0.040 <= thickness <= 0.043,
            details=f"assembly size {(width, depth, thickness)}",
        )
    else:
        ctx.fail("countertop_overall_size_realistic", "assembly AABB unavailable")

    leftmost_button = buttons[0]
    rightmost_button = buttons[-1]
    ctx.expect_origin_distance(
        leftmost_button,
        rightmost_button,
        axes="x",
        min_dist=0.395,
        max_dist=0.405,
        name="buttons_span_most_of_control_strip",
    )
    for index in range(4):
        ctx.expect_origin_distance(
            buttons[index],
            buttons[index + 1],
            axes="x",
            min_dist=0.099,
            max_dist=0.101,
            name=f"button_{index + 1}_to_button_{index + 2}_pitch",
        )
        ctx.expect_origin_distance(
            buttons[index],
            buttons[index + 1],
            axes="y",
            min_dist=0.0,
            max_dist=0.001,
            name=f"button_{index + 1}_to_button_{index + 2}_aligned_in_y",
        )

    for index, joint in enumerate(joints, start=1):
        limits = joint.motion_limits
        ctx.check(
            f"button_{index}_joint_axis",
            tuple(joint.axis) == (0.0, 0.0, -1.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"button_{index}_joint_limits",
            limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and 0.0015 <= limits.upper <= 0.0030,
            details=f"limits={limits}",
        )

        with ctx.pose({joint: 0.0}):
            ctx.expect_contact(
                buttons[index - 1],
                assembly,
                contact_tol=5e-5,
                name=f"button_{index}_rest_contact",
            )

        if limits is not None and limits.upper is not None:
            with ctx.pose({joint: limits.upper}):
                ctx.expect_contact(
                    buttons[index - 1],
                    assembly,
                    contact_tol=5e-5,
                    name=f"button_{index}_pressed_contact",
                )
                ctx.fail_if_parts_overlap_in_current_pose(
                    name=f"button_{index}_pressed_no_overlap"
                )
                ctx.fail_if_isolated_parts(
                    contact_tol=5e-5,
                    name=f"button_{index}_pressed_no_floating",
                )

    zone_names = (
        "zone_front_left",
        "zone_front_right",
        "zone_rear_left",
        "zone_rear_right",
    )
    zone_centers: dict[str, tuple[float, float, float]] = {}
    for zone_name in zone_names:
        zone_aabb = ctx.part_element_world_aabb(assembly, elem=zone_name)
        if zone_aabb is None:
            ctx.fail(f"{zone_name}_present", "zone AABB unavailable")
            continue
        zone_min, zone_max = zone_aabb
        zone_centers[zone_name] = (
            (zone_min[0] + zone_max[0]) / 2.0,
            (zone_min[1] + zone_max[1]) / 2.0,
            (zone_min[2] + zone_max[2]) / 2.0,
        )

    if len(zone_centers) == 4:
        ctx.check(
            "radiant_zones_left_right_symmetry",
            abs(zone_centers["zone_front_left"][0] + zone_centers["zone_front_right"][0]) <= 0.005
            and abs(zone_centers["zone_rear_left"][0] + zone_centers["zone_rear_right"][0]) <= 0.005,
            details=f"zone centers={zone_centers}",
        )
        ctx.check(
            "radiant_zones_front_rear_stagger",
            zone_centers["zone_rear_left"][1] > zone_centers["zone_front_left"][1]
            and zone_centers["zone_rear_right"][1] > zone_centers["zone_front_right"][1],
            details=f"zone centers={zone_centers}",
        )

    pressed_pose = {
        joint: joint.motion_limits.upper
        for joint in joints
        if joint.motion_limits is not None and joint.motion_limits.upper is not None
    }
    with ctx.pose(pressed_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_buttons_pressed_no_overlap")
        ctx.fail_if_isolated_parts(
            contact_tol=5e-5,
            name="all_buttons_pressed_no_floating",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=32, name="button_travel_clearance")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=32,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
