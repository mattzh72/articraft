from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="induction_cooktop_cabinet", assets=ASSETS)

    stone = model.material("counter_stone", rgba=(0.78, 0.78, 0.76, 1.0))
    cabinet_paint = model.material("cabinet_paint", rgba=(0.70, 0.72, 0.68, 1.0))
    cabinet_shadow = model.material("cabinet_shadow", rgba=(0.32, 0.34, 0.33, 1.0))
    cooktop_glass = model.material("cooktop_glass", rgba=(0.08, 0.09, 0.10, 0.96))
    appliance_dark = model.material("appliance_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    zone_mark = model.material("zone_mark", rgba=(0.42, 0.46, 0.48, 0.55))
    control_strip_finish = model.material("control_strip_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    button_finish = model.material("button_finish", rgba=(0.76, 0.77, 0.79, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.82, 0.83, 0.84, 1.0))

    cabinet_width = 1.12
    cabinet_depth = 0.60
    cabinet_height = 0.72
    panel_thickness = 0.018
    back_thickness = 0.012
    toe_kick_height = 0.10
    top_rail_height = 0.06

    countertop_width = 1.12
    countertop_depth = 0.64
    countertop_thickness = 0.04
    cutout_width = 0.86
    cutout_depth = 0.50

    cooktop_width = 0.90
    cooktop_depth = 0.52
    cooktop_glass_thickness = 0.008
    cooktop_body_width = 0.85
    cooktop_body_depth = 0.47
    cooktop_body_depth_z = 0.06

    control_strip_width = 0.64
    control_strip_depth = 0.044
    control_support_height = 0.008
    control_plate_thickness = 0.004
    control_strip_y = -0.208

    button_radius = 0.011
    button_height = 0.009
    button_travel = 0.002
    button_x_positions = (-0.255, -0.200, -0.145, 0.060, 0.205)

    door_width = 0.552
    door_thickness = 0.020
    door_height = cabinet_height - toe_kick_height
    handle_offset_from_free_edge = 0.075
    handle_span = 0.14

    def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
        half_w = width * 0.5
        half_d = depth * 0.5
        return [
            (-half_w, -half_d),
            (half_w, -half_d),
            (half_w, half_d),
            (-half_w, half_d),
        ]

    def _circle_profile(
        radius: float,
        *,
        segments: int = 32,
        center: tuple[float, float] = (0.0, 0.0),
    ) -> list[tuple[float, float]]:
        cx, cy = center
        return [
            (
                cx + radius * math.cos(2.0 * math.pi * index / segments),
                cy + radius * math.sin(2.0 * math.pi * index / segments),
            )
            for index in range(segments)
        ]

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    cabinet_body = model.part("cabinet_body")
    cabinet_body.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                -(cabinet_width * 0.5 - panel_thickness * 0.5),
                0.0,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="left_side",
    )
    cabinet_body.visual(
        Box((panel_thickness, cabinet_depth, cabinet_height)),
        origin=Origin(
            xyz=(
                cabinet_width * 0.5 - panel_thickness * 0.5,
                0.0,
                cabinet_height * 0.5,
            )
        ),
        material=cabinet_paint,
        name="right_side",
    )
    cabinet_body.visual(
        Box((cabinet_width - 2.0 * panel_thickness, cabinet_depth, panel_thickness)),
        origin=Origin(xyz=(0.0, 0.0, panel_thickness * 0.5)),
        material=cabinet_shadow,
        name="bottom_panel",
    )
    cabinet_body.visual(
        Box((cabinet_width - 2.0 * panel_thickness, back_thickness, cabinet_height - panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                cabinet_depth * 0.5 - back_thickness * 0.5,
                panel_thickness + (cabinet_height - panel_thickness) * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="back_panel",
    )
    cabinet_body.visual(
        Box((cabinet_width - 2.0 * panel_thickness, panel_thickness, toe_kick_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth * 0.5 - panel_thickness * 0.5),
                toe_kick_height * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="toe_kick",
    )
    cabinet_body.visual(
        Box((cabinet_width - 2.0 * panel_thickness, panel_thickness, top_rail_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(cabinet_depth * 0.5 - panel_thickness * 0.5),
                cabinet_height - top_rail_height * 0.5,
            )
        ),
        material=cabinet_shadow,
        name="top_rail",
    )
    cabinet_body.inertial = Inertial.from_geometry(
        Box((cabinet_width, cabinet_depth, cabinet_height)),
        mass=58.0,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height * 0.5)),
    )

    countertop = model.part("countertop")
    countertop_mesh = _save_mesh(
        "countertop_slab.obj",
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(countertop_width, countertop_depth, 0.010, corner_segments=10),
            [rounded_rect_profile(cutout_width, cutout_depth, 0.026, corner_segments=10)],
            height=countertop_thickness,
            center=False,
        ),
    )
    countertop.visual(
        countertop_mesh,
        material=stone,
        name="counter_slab",
    )
    countertop.inertial = Inertial.from_geometry(
        Box((countertop_width, countertop_depth, countertop_thickness)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, countertop_thickness * 0.5)),
    )
    model.articulation(
        "cabinet_to_countertop",
        ArticulationType.FIXED,
        parent=cabinet_body,
        child=countertop,
        origin=Origin(xyz=(0.0, 0.0, cabinet_height)),
    )

    cooktop = model.part("cooktop")
    cooktop.visual(
        _save_mesh(
            "cooktop_glass_top.obj",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(cooktop_width, cooktop_depth, 0.022, corner_segments=10),
                cooktop_glass_thickness,
            ),
        ),
        material=cooktop_glass,
        name="glass_top",
    )
    cooktop.visual(
        _save_mesh(
            "cooktop_body_shell.obj",
            ExtrudeGeometry.from_z0(
                rounded_rect_profile(cooktop_body_width, cooktop_body_depth, 0.018, corner_segments=10),
                cooktop_body_depth_z,
            ),
        ),
        origin=Origin(xyz=(0.0, 0.0, -cooktop_body_depth_z)),
        material=appliance_dark,
        name="cooktop_body",
    )
    for name, x_pos, y_pos, diameter in (
        ("zone_rear_left", -0.250, 0.135, 0.170),
        ("zone_front_left", -0.215, -0.110, 0.205),
        ("zone_center", 0.000, 0.010, 0.240),
        ("zone_rear_right", 0.220, 0.140, 0.180),
        ("zone_front_right", 0.235, -0.120, 0.165),
    ):
        cooktop.visual(
            Cylinder(radius=diameter * 0.5, length=0.0008),
            origin=Origin(xyz=(x_pos, y_pos, cooktop_glass_thickness + 0.0004)),
            material=zone_mark,
            name=name,
        )
    cooktop.inertial = Inertial.from_geometry(
        Box((cooktop_width, cooktop_depth, cooktop_glass_thickness + cooktop_body_depth_z)),
        mass=13.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                -(cooktop_body_depth_z - cooktop_glass_thickness) * 0.5,
            )
        ),
    )
    model.articulation(
        "countertop_to_cooktop",
        ArticulationType.FIXED,
        parent=countertop,
        child=cooktop,
        origin=Origin(xyz=(0.0, 0.0, countertop_thickness)),
    )

    control_face_geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(control_strip_width, control_strip_depth, 0.010, corner_segments=10),
        [_circle_profile(button_radius, center=(x_pos, 0.0)) for x_pos in button_x_positions],
        height=control_plate_thickness,
        center=True,
    )
    control_face_geom.translate(0.0, 0.0, control_support_height + control_plate_thickness * 0.5)
    control_face_mesh = _save_mesh("cooktop_control_strip.obj", control_face_geom)
    button_mesh = _save_mesh(
        "cooktop_button.obj",
        ExtrudeGeometry.from_z0(_circle_profile(button_radius), button_height),
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        control_face_mesh,
        material=control_strip_finish,
        name="control_face",
    )
    control_strip.visual(
        Box((0.024, control_strip_depth, control_support_height)),
        origin=Origin(xyz=(-0.306, 0.0, control_support_height * 0.5)),
        material=control_strip_finish,
        name="left_support",
    )
    control_strip.visual(
        Box((0.024, control_strip_depth, control_support_height)),
        origin=Origin(xyz=(0.306, 0.0, control_support_height * 0.5)),
        material=control_strip_finish,
        name="right_support",
    )
    control_strip.visual(
        Box((control_strip_width, 0.008, control_support_height)),
        origin=Origin(
            xyz=(
                0.0,
                control_strip_depth * 0.5 - 0.004,
                control_support_height * 0.5,
            )
        ),
        material=control_strip_finish,
        name="rear_spine",
    )
    control_strip.inertial = Inertial.from_geometry(
        Box((control_strip_width, control_strip_depth, control_support_height + control_plate_thickness)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, (control_support_height + control_plate_thickness) * 0.5)),
    )
    model.articulation(
        "cooktop_to_control_strip",
        ArticulationType.FIXED,
        parent=cooktop,
        child=control_strip,
        origin=Origin(xyz=(0.0, control_strip_y, cooktop_glass_thickness)),
    )

    for button_index, x_pos in enumerate(button_x_positions, start=1):
        button = model.part(f"button_{button_index}")
        button.visual(
            button_mesh,
            material=button_finish,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((button_radius * 2.0, button_radius * 2.0, button_height)),
            mass=0.05,
            origin=Origin(xyz=(0.0, 0.0, button_height * 0.5)),
        )
        model.articulation(
            f"control_strip_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(x_pos, 0.0, button_travel)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.05,
                lower=0.0,
                upper=button_travel,
            ),
        )

    left_door = model.part("left_door")
    left_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(door_width * 0.5, -door_thickness * 0.5, door_height * 0.5)),
        material=cabinet_paint,
        name="door_panel",
    )
    left_handle_x = door_width - handle_offset_from_free_edge
    for z_pos in (0.24, 0.38):
        left_door.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(left_handle_x, -0.029, z_pos),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=handle_metal,
            name=f"standoff_{int(z_pos * 100)}",
        )
    left_door.visual(
        Cylinder(radius=0.006, length=handle_span),
        origin=Origin(xyz=(left_handle_x, -0.038, door_height * 0.5)),
        material=handle_metal,
        name="pull_handle",
    )
    left_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.05, door_height)),
        mass=8.0,
        origin=Origin(xyz=(door_width * 0.5, -0.020, door_height * 0.5)),
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((door_width, door_thickness, door_height)),
        origin=Origin(xyz=(-door_width * 0.5, -door_thickness * 0.5, door_height * 0.5)),
        material=cabinet_paint,
        name="door_panel",
    )
    right_handle_x = -(door_width - handle_offset_from_free_edge)
    for z_pos in (0.24, 0.38):
        right_door.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(right_handle_x, -0.029, z_pos),
                rpy=(math.pi * 0.5, 0.0, 0.0),
            ),
            material=handle_metal,
            name=f"standoff_{int(z_pos * 100)}",
        )
    right_door.visual(
        Cylinder(radius=0.006, length=handle_span),
        origin=Origin(xyz=(right_handle_x, -0.038, door_height * 0.5)),
        material=handle_metal,
        name="pull_handle",
    )
    right_door.inertial = Inertial.from_geometry(
        Box((door_width, 0.05, door_height)),
        mass=8.0,
        origin=Origin(xyz=(-door_width * 0.5, -0.020, door_height * 0.5)),
    )

    model.articulation(
        "cabinet_to_left_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=left_door,
        origin=Origin(xyz=(-cabinet_width * 0.5, -cabinet_depth * 0.5, toe_kick_height)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.0,
            lower=0.0,
            upper=1.92,
        ),
    )
    model.articulation(
        "cabinet_to_right_door",
        ArticulationType.REVOLUTE,
        parent=cabinet_body,
        child=right_door,
        origin=Origin(xyz=(cabinet_width * 0.5, -cabinet_depth * 0.5, toe_kick_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.0,
            lower=0.0,
            upper=1.92,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
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

    cabinet_body = object_model.get_part("cabinet_body")
    countertop = object_model.get_part("countertop")
    cooktop = object_model.get_part("cooktop")
    control_strip = object_model.get_part("control_strip")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    button_parts = [object_model.get_part(f"button_{index}") for index in range(1, 6)]

    glass_top = cooktop.get_visual("glass_top")
    cooktop_body = cooktop.get_visual("cooktop_body")
    zones = [
        cooktop.get_visual("zone_rear_left"),
        cooktop.get_visual("zone_front_left"),
        cooktop.get_visual("zone_center"),
        cooktop.get_visual("zone_rear_right"),
        cooktop.get_visual("zone_front_right"),
    ]
    control_face = control_strip.get_visual("control_face")
    left_door_panel = left_door.get_visual("door_panel")
    right_door_panel = right_door.get_visual("door_panel")

    left_hinge = object_model.get_articulation("cabinet_to_left_door")
    right_hinge = object_model.get_articulation("cabinet_to_right_door")
    button_joints = [
        object_model.get_articulation(f"control_strip_to_button_{index}")
        for index in range(1, 6)
    ]

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    def _axis_is_close(axis, expected):
        return all(abs(axis[i] - expected[i]) < 1e-9 for i in range(3))

    ctx.expect_contact(countertop, cabinet_body, name="countertop_supported_by_cabinet")
    ctx.expect_contact(cooktop, countertop, name="cooktop_seated_in_countertop")
    ctx.expect_contact(control_strip, cooktop, name="control_strip_mounted_to_cooktop")
    ctx.expect_contact(left_door, cabinet_body, name="left_door_closed_contact")
    ctx.expect_contact(right_door, cabinet_body, name="right_door_closed_contact")

    countertop_aabb = ctx.part_world_aabb(countertop)
    cabinet_aabb = ctx.part_world_aabb(cabinet_body)
    if countertop_aabb is None or cabinet_aabb is None:
        ctx.fail("main_body_aabbs_present", "countertop or cabinet AABB unavailable")
    else:
        countertop_width = countertop_aabb[1][0] - countertop_aabb[0][0]
        cabinet_width = cabinet_aabb[1][0] - cabinet_aabb[0][0]
        ctx.check(
            "broad_cabinet_proportions",
            countertop_width > 1.05 and cabinet_width > 1.05,
            f"countertop_width={countertop_width:.3f}, cabinet_width={cabinet_width:.3f}",
        )

    cooktop_aabb = ctx.part_world_aabb(cooktop)
    if cooktop_aabb is None:
        ctx.fail("cooktop_aabb_present", "cooktop AABB unavailable")
    else:
        cooktop_width = cooktop_aabb[1][0] - cooktop_aabb[0][0]
        cooktop_depth = cooktop_aabb[1][1] - cooktop_aabb[0][1]
        ctx.check(
            "cooktop_is_low_profile",
            cooktop_width > 0.85 and cooktop_depth > 0.48,
            f"cooktop footprint={cooktop_width:.3f} x {cooktop_depth:.3f}",
        )

    zone_names_ok = len({zone.name for zone in zones}) == 5
    ctx.check("five_named_cooking_zones", zone_names_ok, "expected five distinct cooking-zone visuals")
    glass_aabb = ctx.part_element_world_aabb(cooktop, elem=glass_top)
    if glass_aabb is None:
        ctx.fail("glass_top_aabb_present", "glass top AABB unavailable")
    else:
        glass_width = glass_aabb[1][0] - glass_aabb[0][0]
        glass_depth = glass_aabb[1][1] - glass_aabb[0][1]
        zone_inside = True
        zone_details: list[str] = []
        for zone in zones:
            zone_aabb = ctx.part_element_world_aabb(cooktop, elem=zone)
            if zone_aabb is None:
                zone_inside = False
                zone_details.append(f"{zone.name}:missing")
                continue
            within_x = glass_aabb[0][0] <= zone_aabb[0][0] and zone_aabb[1][0] <= glass_aabb[1][0]
            within_y = glass_aabb[0][1] <= zone_aabb[0][1] and zone_aabb[1][1] <= glass_aabb[1][1]
            zone_inside = zone_inside and within_x and within_y
            zone_details.append(f"{zone.name}:{within_x and within_y}")
        ctx.check(
            "zones_within_glass_surface",
            zone_inside and glass_width > 0.85 and glass_depth > 0.50,
            ", ".join(zone_details),
        )

    ctx.check(
        "left_door_vertical_hinge_axis",
        _axis_is_close(left_hinge.axis, (0.0, 0.0, -1.0)),
        f"axis={left_hinge.axis}",
    )
    ctx.check(
        "right_door_vertical_hinge_axis",
        _axis_is_close(right_hinge.axis, (0.0, 0.0, 1.0)),
        f"axis={right_hinge.axis}",
    )

    button_rest_positions = [ctx.part_world_position(button) for button in button_parts]
    if any(position is None for position in button_rest_positions):
        ctx.fail("button_positions_present", "one or more button positions unavailable")
    else:
        button_rest_positions = [position for position in button_rest_positions if position is not None]
        x_gaps = [
            button_rest_positions[index + 1][0] - button_rest_positions[index][0]
            for index in range(len(button_rest_positions) - 1)
        ]
        left_heavy = (
            button_rest_positions[2][0] < 0.0 < button_rest_positions[3][0]
            and x_gaps[0] < 0.07
            and x_gaps[1] < 0.07
            and x_gaps[2] > 0.16
            and x_gaps[3] > 0.10
        )
        ctx.check("left_heavy_button_layout", left_heavy, f"x_gaps={x_gaps}")

    for button_index, (button, joint) in enumerate(zip(button_parts, button_joints, strict=True), start=1):
        ctx.check(
            f"button_{button_index}_axis",
            _axis_is_close(joint.axis, (0.0, 0.0, -1.0)),
            f"axis={joint.axis}",
        )
        ctx.expect_contact(button, control_strip, name=f"button_{button_index}_rest_contact")
        ctx.expect_overlap(
            button,
            control_strip,
            axes="xy",
            min_overlap=0.020,
            name=f"button_{button_index}_xy_alignment",
        )
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            rest_pos = ctx.part_world_position(button)
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
                ctx.expect_contact(button, control_strip, name=f"button_{button_index}_pressed_contact")
                pressed_pos = ctx.part_world_position(button)
            if rest_pos is None or pressed_pos is None:
                ctx.fail(f"button_{button_index}_travel_measurement", "button origin unavailable")
            else:
                ctx.check(
                    f"button_{button_index}_presses_inward",
                    pressed_pos[2] < rest_pos[2] - 0.0015,
                    f"rest_z={rest_pos[2]:.4f}, pressed_z={pressed_pos[2]:.4f}",
                )

    for door, door_panel, hinge, side_name in (
        (left_door, left_door_panel, left_hinge, "left"),
        (right_door, right_door_panel, right_hinge, "right"),
    ):
        limits = hinge.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(f"{side_name}_door_limits_present", "door joint limits missing")
            continue
        closed_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)
        with ctx.pose({hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge.name}_lower_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{hinge.name}_lower_no_floating")
            ctx.expect_contact(door, cabinet_body, name=f"{side_name}_door_closed_hinge_contact")
        with ctx.pose({hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{hinge.name}_upper_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{hinge.name}_upper_no_floating")
            ctx.expect_contact(door, cabinet_body, name=f"{side_name}_door_open_hinge_contact")
            open_panel_aabb = ctx.part_element_world_aabb(door, elem=door_panel)
        if closed_panel_aabb is None or open_panel_aabb is None:
            ctx.fail(f"{side_name}_door_motion_measurement", "door panel AABB unavailable")
        else:
            closed_center = _aabb_center(closed_panel_aabb)
            open_center = _aabb_center(open_panel_aabb)
            if closed_center is None or open_center is None:
                ctx.fail(f"{side_name}_door_motion_center", "door center unavailable")
            else:
                ctx.check(
                    f"{side_name}_door_swings_outward",
                    open_center[1] < closed_center[1] - 0.18,
                    f"closed_center={closed_center}, open_center={open_center}",
                )

    if (
        left_hinge.motion_limits is not None
        and right_hinge.motion_limits is not None
        and left_hinge.motion_limits.upper is not None
        and right_hinge.motion_limits.upper is not None
    ):
        with ctx.pose(
            {
                left_hinge: left_hinge.motion_limits.upper,
                right_hinge: right_hinge.motion_limits.upper,
            }
        ):
            ctx.fail_if_parts_overlap_in_current_pose(name="both_doors_open_no_overlap")
            ctx.fail_if_isolated_parts(name="both_doors_open_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
