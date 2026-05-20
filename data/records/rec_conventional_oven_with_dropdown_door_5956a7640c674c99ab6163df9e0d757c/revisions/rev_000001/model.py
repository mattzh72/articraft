from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_tunnel_oven")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.28, 0.30, 0.33, 1.0))
    conveyor_black = model.material("conveyor_black", rgba=(0.12, 0.12, 0.11, 1.0))
    roller_grey = model.material("roller_grey", rgba=(0.52, 0.55, 0.58, 1.0))
    door_blue = model.material("door_blue", rgba=(0.20, 0.37, 0.63, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.10, 1.0))
    sight_glass = model.material("sight_glass", rgba=(0.55, 0.72, 0.82, 0.30))

    body_length = 3.40
    body_width = 1.00
    body_height = 0.90
    leg_height = 0.45
    wall_t = 0.04
    body_half_x = body_length * 0.5
    body_half_y = body_width * 0.5
    body_bottom_z = leg_height
    body_top_z = leg_height + body_height
    floor_center_z = body_bottom_z + wall_t * 0.5
    roof_center_z = body_top_z - wall_t * 0.5
    shell_center_z = body_bottom_z + body_height * 0.5

    end_opening_width = 0.82
    end_opening_bottom = 0.66
    end_opening_top = 0.86
    end_column_w = (body_width - end_opening_width) * 0.5

    service_opening_width = 1.35
    service_opening_center_x = -0.45
    service_opening_bottom = 0.60
    service_opening_top = 1.18
    service_left_edge = service_opening_center_x - service_opening_width * 0.5
    service_right_edge = service_opening_center_x + service_opening_width * 0.5

    roller_center_x = 1.40
    roller_center_z = 0.76
    roller_radius = 0.075
    roller_length = 0.80
    belt_width = 0.76
    belt_thickness = 0.014
    belt_wrap_radius = roller_radius + belt_thickness * 0.5

    chassis = model.part("chassis")
    chassis.visual(
        Box((body_length, body_width, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_center_z)),
        material=stainless,
        name="floor_panel",
    )
    chassis.visual(
        Box((body_length, body_width, wall_t)),
        origin=Origin(xyz=(0.0, 0.0, roof_center_z)),
        material=stainless,
        name="roof_panel",
    )
    chassis.visual(
        Box((body_length, wall_t, body_height)),
        origin=Origin(xyz=(0.0, -body_half_y + wall_t * 0.5, shell_center_z)),
        material=stainless,
        name="rear_wall",
    )
    chassis.visual(
        Box((body_half_x + service_left_edge, wall_t, body_height)),
        origin=Origin(
            xyz=(
                (-body_half_x + service_left_edge) * 0.5,
                body_half_y - wall_t * 0.5,
                shell_center_z,
            )
        ),
        material=stainless,
        name="front_left_wall",
    )
    chassis.visual(
        Box((body_half_x - service_right_edge, wall_t, body_height)),
        origin=Origin(
            xyz=(
                (service_right_edge + body_half_x) * 0.5,
                body_half_y - wall_t * 0.5,
                shell_center_z,
            )
        ),
        material=stainless,
        name="front_right_wall",
    )
    chassis.visual(
        Box((service_opening_width, wall_t, service_opening_bottom - body_bottom_z)),
        origin=Origin(
            xyz=(
                service_opening_center_x,
                body_half_y - wall_t * 0.5,
                (body_bottom_z + service_opening_bottom) * 0.5,
            )
        ),
        material=stainless,
        name="front_service_sill",
    )
    chassis.visual(
        Box((service_opening_width, wall_t, body_top_z - service_opening_top)),
        origin=Origin(
            xyz=(
                service_opening_center_x,
                body_half_y - wall_t * 0.5,
                (service_opening_top + body_top_z) * 0.5,
            )
        ),
        material=stainless,
        name="front_service_header",
    )
    chassis.visual(
        Box((wall_t, body_width, body_height)),
        origin=Origin(xyz=(-body_half_x + wall_t * 0.5, 0.0, shell_center_z)),
        material=stainless,
        name="left_end_top_frame",
    )
    chassis.visual(
        Box((wall_t, body_width, body_height)),
        origin=Origin(xyz=(body_half_x - wall_t * 0.5, 0.0, shell_center_z)),
        material=stainless,
        name="right_end_top_frame",
    )

    for x_sign, prefix in ((-1.0, "inlet"), (1.0, "outlet")):
        x_center = x_sign * (body_half_x - wall_t * 0.5)
        chassis.visual(
            Box((wall_t, body_width, end_opening_bottom - body_bottom_z)),
            origin=Origin(
                xyz=(x_center, 0.0, (body_bottom_z + end_opening_bottom) * 0.5)
            ),
            material=stainless,
            name=f"{prefix}_lower_sill",
        )
        chassis.visual(
            Box((wall_t, body_width, body_top_z - end_opening_top)),
            origin=Origin(xyz=(x_center, 0.0, (end_opening_top + body_top_z) * 0.5)),
            material=stainless,
            name=f"{prefix}_header",
        )
        chassis.visual(
            Box((wall_t, end_column_w, end_opening_top - end_opening_bottom)),
            origin=Origin(
                xyz=(
                    x_center,
                    -(end_opening_width + end_column_w) * 0.5,
                    (end_opening_bottom + end_opening_top) * 0.5,
                )
            ),
            material=stainless,
            name=f"{prefix}_left_post",
        )
        chassis.visual(
            Box((wall_t, end_column_w, end_opening_top - end_opening_bottom)),
            origin=Origin(
                xyz=(
                    x_center,
                    (end_opening_width + end_column_w) * 0.5,
                    (end_opening_bottom + end_opening_top) * 0.5,
                )
            ),
            material=stainless,
            name=f"{prefix}_right_post",
        )

    top_run_center_z = roller_center_z + roller_radius + belt_thickness * 0.5
    top_support_center_z = top_run_center_z - belt_thickness * 0.5 - 0.01
    support_side_y = belt_width * 0.5 + 0.03
    chassis.visual(
        Box((2.60, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, -support_side_y, top_support_center_z)),
        material=dark_frame,
        name="rear_belt_support",
    )
    chassis.visual(
        Box((2.60, 0.03, 0.02)),
        origin=Origin(xyz=(0.0, support_side_y, top_support_center_z)),
        material=dark_frame,
        name="front_belt_support",
    )
    support_post_height = top_support_center_z - 0.01 - (floor_center_z + wall_t * 0.5)
    support_post_center_z = (top_support_center_z - 0.01 + floor_center_z + wall_t * 0.5) * 0.5
    for idx, x_pos in enumerate((-0.95, 0.0, 0.95), start=1):
        chassis.visual(
            Box((0.06, 0.03, support_post_height)),
            origin=Origin(xyz=(x_pos, -support_side_y, support_post_center_z)),
            material=dark_frame,
            name=f"rear_support_post_{idx}",
        )
        chassis.visual(
            Box((0.06, 0.03, support_post_height)),
            origin=Origin(xyz=(x_pos, support_side_y, support_post_center_z)),
            material=dark_frame,
            name=f"front_support_post_{idx}",
        )
    bearing_post_height = (roller_center_z - 0.07) - (floor_center_z + wall_t * 0.5)
    bearing_post_center_z = ((roller_center_z - 0.07) + floor_center_z + wall_t * 0.5) * 0.5
    for x_pos, prefix in ((-roller_center_x, "inlet"), (roller_center_x, "outlet")):
        chassis.visual(
            Box((0.10, 0.04, 0.14)),
            origin=Origin(xyz=(x_pos, -0.42, roller_center_z)),
            material=dark_frame,
            name=f"{prefix}_rear_bearing_block",
        )
        chassis.visual(
            Box((0.10, 0.04, 0.14)),
            origin=Origin(xyz=(x_pos, 0.42, roller_center_z)),
            material=dark_frame,
            name=f"{prefix}_front_bearing_block",
        )
        chassis.visual(
            Box((0.08, 0.04, bearing_post_height)),
            origin=Origin(xyz=(x_pos, -0.42, bearing_post_center_z)),
            material=dark_frame,
            name=f"{prefix}_rear_bearing_post",
        )
        chassis.visual(
            Box((0.08, 0.04, bearing_post_height)),
            origin=Origin(xyz=(x_pos, 0.42, bearing_post_center_z)),
            material=dark_frame,
            name=f"{prefix}_front_bearing_post",
        )

    leg_size = 0.08
    foot_size = 0.14
    leg_x = 1.45
    leg_y = 0.38
    for i, (sx, sy) in enumerate(
        ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)),
        start=1,
    ):
        chassis.visual(
            Box((leg_size, leg_size, leg_height)),
            origin=Origin(xyz=(sx * leg_x, sy * leg_y, leg_height * 0.5)),
            material=dark_frame,
            name=f"leg_{i}",
        )
        chassis.visual(
            Box((foot_size, foot_size, 0.02)),
            origin=Origin(xyz=(sx * leg_x, sy * leg_y, 0.01)),
            material=dark_frame,
            name=f"foot_{i}",
        )
    chassis.visual(
        Box((2.82, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, leg_y, 0.18)),
        material=dark_frame,
        name="front_lower_brace",
    )
    chassis.visual(
        Box((2.82, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, -leg_y, 0.18)),
        material=dark_frame,
        name="rear_lower_brace",
    )
    chassis.visual(
        Box((0.04, 0.68, 0.05)),
        origin=Origin(xyz=(-leg_x, 0.0, 0.18)),
        material=dark_frame,
        name="left_lower_brace",
    )
    chassis.visual(
        Box((0.04, 0.68, 0.05)),
        origin=Origin(xyz=(leg_x, 0.0, 0.18)),
        material=dark_frame,
        name="right_lower_brace",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_top_z)),
        mass=320.0,
        origin=Origin(xyz=(0.0, 0.0, body_top_z * 0.5)),
    )

    front_track = model.part("front_track")
    rail_length = 2.90
    upper_rail_center = (0.15, 0.61, 1.22)
    lower_rail_center = (0.15, 0.61, 0.57)
    front_track.visual(
        Box((rail_length, 0.06, 0.05)),
        origin=Origin(xyz=upper_rail_center),
        material=dark_frame,
        name="upper_rail",
    )
    front_track.visual(
        Box((rail_length, 0.06, 0.05)),
        origin=Origin(xyz=lower_rail_center),
        material=dark_frame,
        name="lower_rail",
    )
    for idx, x_pos in enumerate((-1.10, -0.40, 0.30, 1.00), start=1):
        front_track.visual(
            Box((0.08, 0.08, 0.70)),
            origin=Origin(xyz=(x_pos, 0.54, 0.895)),
            material=dark_frame,
            name=f"upright_{idx}",
        )
        front_track.visual(
            Box((0.10, 0.08, 0.06)),
            origin=Origin(xyz=(x_pos, 0.54, upper_rail_center[2])),
            material=dark_frame,
            name=f"upper_standoff_{idx}",
        )
        front_track.visual(
            Box((0.10, 0.08, 0.06)),
            origin=Origin(xyz=(x_pos, 0.54, lower_rail_center[2])),
            material=dark_frame,
            name=f"lower_standoff_{idx}",
        )
    for prefix, z_pos in (("upper", upper_rail_center[2]), ("lower", lower_rail_center[2])):
        front_track.visual(
            Box((0.04, 0.09, 0.09)),
            origin=Origin(xyz=(-1.28, 0.615, z_pos)),
            material=dark_frame,
            name=f"{prefix}_left_stop",
        )
        front_track.visual(
            Box((0.04, 0.09, 0.09)),
            origin=Origin(xyz=(1.58, 0.615, z_pos)),
            material=dark_frame,
            name=f"{prefix}_right_stop",
        )
    front_track.inertial = Inertial.from_geometry(
        Box((rail_length, 0.16, 0.75)),
        mass=24.0,
        origin=Origin(xyz=(0.15, 0.58, 0.895)),
    )

    model.articulation(
        "chassis_to_front_track",
        ArticulationType.FIXED,
        parent=chassis,
        child=front_track,
    )

    sliding_door = model.part("sliding_door")
    sliding_door.visual(
        Box((1.48, 0.03, 0.62)),
        material=door_blue,
        name="door_panel",
    )
    sliding_door.visual(
        Box((1.34, 0.012, 0.08)),
        origin=Origin(xyz=(0.0, 0.021, 0.23)),
        material=door_blue,
        name="top_stiffener",
    )
    sliding_door.visual(
        Box((1.34, 0.012, 0.08)),
        origin=Origin(xyz=(0.0, 0.021, -0.23)),
        material=door_blue,
        name="bottom_stiffener",
    )
    sliding_door.visual(
        Box((0.08, 0.012, 0.50)),
        origin=Origin(xyz=(-0.48, 0.021, 0.0)),
        material=door_blue,
        name="left_stiffener",
    )
    sliding_door.visual(
        Box((0.08, 0.012, 0.50)),
        origin=Origin(xyz=(0.10, 0.021, 0.0)),
        material=door_blue,
        name="center_stiffener",
    )
    sliding_door.visual(
        Box((0.08, 0.012, 0.50)),
        origin=Origin(xyz=(0.55, 0.021, 0.0)),
        material=door_blue,
        name="right_stiffener",
    )
    sliding_door.visual(
        Box((0.42, 0.010, 0.18)),
        origin=Origin(xyz=(-0.20, 0.0, 0.06)),
        material=sight_glass,
        name="inspection_window",
    )
    sliding_door.visual(
        Box((0.12, 0.05, 0.03)),
        origin=Origin(xyz=(-0.56, -0.04, 0.305)),
        material=dark_frame,
        name="top_left_shoe",
    )
    sliding_door.visual(
        Box((0.12, 0.05, 0.03)),
        origin=Origin(xyz=(0.56, -0.04, 0.305)),
        material=dark_frame,
        name="top_right_shoe",
    )
    sliding_door.visual(
        Box((0.12, 0.05, 0.03)),
        origin=Origin(xyz=(-0.56, -0.04, -0.265)),
        material=dark_frame,
        name="bottom_left_shoe",
    )
    sliding_door.visual(
        Box((0.12, 0.05, 0.03)),
        origin=Origin(xyz=(0.56, -0.04, -0.265)),
        material=dark_frame,
        name="bottom_right_shoe",
    )
    sliding_door.visual(
        Box((0.04, 0.03, 0.05)),
        origin=Origin(xyz=(0.58, 0.030, 0.115)),
        material=handle_black,
        name="handle_upper_mount",
    )
    sliding_door.visual(
        Box((0.04, 0.03, 0.05)),
        origin=Origin(xyz=(0.58, 0.030, -0.115)),
        material=handle_black,
        name="handle_lower_mount",
    )
    sliding_door.visual(
        Cylinder(radius=0.013, length=0.18),
        origin=Origin(xyz=(0.58, 0.052, 0.0)),
        material=handle_black,
        name="handle_grip",
    )
    sliding_door.inertial = Inertial.from_geometry(
        Box((1.48, 0.08, 0.65)),
        mass=48.0,
        origin=Origin(xyz=(0.0, 0.03, 0.0)),
    )

    door_closed_origin = Origin(xyz=(service_opening_center_x, 0.655, 0.875))
    model.articulation(
        "track_to_door",
        ArticulationType.PRISMATIC,
        parent=front_track,
        child=sliding_door,
        origin=door_closed_origin,
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=1.35,
        ),
    )

    conveyor_belt = model.part("conveyor_belt")
    conveyor_belt.visual(
        Box((2.95, belt_width, belt_thickness)),
        origin=Origin(xyz=(0.0, 0.0, roller_center_z + roller_radius + belt_thickness * 0.5)),
        material=conveyor_black,
        name="top_run",
    )
    conveyor_belt.visual(
        Box((2.95, belt_width, belt_thickness)),
        origin=Origin(xyz=(0.0, 0.0, roller_center_z - roller_radius - belt_thickness * 0.5)),
        material=conveyor_black,
        name="bottom_return",
    )
    conveyor_belt.visual(
        Box((belt_thickness, belt_width, roller_radius * 2.0)),
        origin=Origin(
            xyz=(-roller_center_x - roller_radius - belt_thickness * 0.5, 0.0, roller_center_z)
        ),
        material=conveyor_black,
        name="inlet_return",
    )
    conveyor_belt.visual(
        Box((belt_thickness, belt_width, roller_radius * 2.0)),
        origin=Origin(
            xyz=(roller_center_x + roller_radius + belt_thickness * 0.5, 0.0, roller_center_z)
        ),
        material=conveyor_black,
        name="outlet_return",
    )
    conveyor_belt.inertial = Inertial.from_geometry(
        Box((2.95, belt_width, 0.18)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, roller_center_z)),
    )
    model.articulation(
        "chassis_to_belt",
        ArticulationType.FIXED,
        parent=chassis,
        child=conveyor_belt,
    )

    inlet_roller = model.part("inlet_roller")
    inlet_roller.visual(
        Cylinder(radius=roller_radius, length=roller_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_grey,
        name="roller_drum",
    )
    inlet_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=roller_radius, length=roller_length),
        mass=12.0,
    )
    model.articulation(
        "chassis_to_inlet_roller",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=inlet_roller,
        origin=Origin(xyz=(-roller_center_x, 0.0, roller_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )

    outlet_roller = model.part("outlet_roller")
    outlet_roller.visual(
        Cylinder(radius=roller_radius, length=roller_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=roller_grey,
        name="roller_drum",
    )
    outlet_roller.inertial = Inertial.from_geometry(
        Cylinder(radius=roller_radius, length=roller_length),
        mass=12.0,
    )
    model.articulation(
        "chassis_to_outlet_roller",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=outlet_roller,
        origin=Origin(xyz=(roller_center_x, 0.0, roller_center_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    chassis = object_model.get_part("chassis")
    front_track = object_model.get_part("front_track")
    sliding_door = object_model.get_part("sliding_door")
    conveyor_belt = object_model.get_part("conveyor_belt")
    inlet_roller = object_model.get_part("inlet_roller")
    outlet_roller = object_model.get_part("outlet_roller")

    door_slide = object_model.get_articulation("track_to_door")
    inlet_spin = object_model.get_articulation("chassis_to_inlet_roller")
    outlet_spin = object_model.get_articulation("chassis_to_outlet_roller")

    ctx.expect_contact(
        front_track,
        chassis,
        contact_tol=0.001,
        name="front guide track mounts to the oven housing",
    )
    ctx.expect_within(
        conveyor_belt,
        chassis,
        axes="yz",
        margin=0.02,
        name="conveyor belt stays inside the tunnel envelope",
    )
    ctx.expect_contact(
        conveyor_belt,
        inlet_roller,
        elem_a="top_run",
        elem_b="roller_drum",
        contact_tol=0.001,
        name="top conveyor run contacts the inlet roller drum",
    )
    ctx.expect_contact(
        conveyor_belt,
        outlet_roller,
        elem_a="top_run",
        elem_b="roller_drum",
        contact_tol=0.001,
        name="top conveyor run contacts the outlet roller drum",
    )

    with ctx.pose({door_slide: 0.0}):
        ctx.expect_contact(
            sliding_door,
            front_track,
            elem_a="top_left_shoe",
            elem_b="upper_rail",
            contact_tol=0.001,
            name="closed door top carriage bears on the upper rail",
        )
        ctx.expect_contact(
            sliding_door,
            front_track,
            elem_a="bottom_left_shoe",
            elem_b="lower_rail",
            contact_tol=0.001,
            name="closed door bottom carriage bears on the lower rail",
        )
        ctx.expect_gap(
            sliding_door,
            chassis,
            axis="y",
            min_gap=0.12,
            max_gap=0.15,
            positive_elem="door_panel",
            name="door panel rides proud of the oven front face",
        )
        closed_pos = ctx.part_world_position(sliding_door)

    with ctx.pose({door_slide: 1.35}):
        ctx.expect_contact(
            sliding_door,
            front_track,
            elem_a="top_right_shoe",
            elem_b="upper_rail",
            contact_tol=0.001,
            name="open door remains supported by the upper guide rail",
        )
        ctx.expect_contact(
            sliding_door,
            front_track,
            elem_a="bottom_right_shoe",
            elem_b="lower_rail",
            contact_tol=0.001,
            name="open door remains supported by the lower guide rail",
        )
        open_pos = ctx.part_world_position(sliding_door)

    ctx.check(
        "service door slides laterally to the right",
        closed_pos is not None
        and open_pos is not None
        and open_pos[0] > closed_pos[0] + 1.20,
        details=f"closed={closed_pos}, open={open_pos}",
    )
    ctx.check(
        "conveyor rollers use continuous rotation joints across oven width",
        inlet_spin.joint_type == ArticulationType.CONTINUOUS
        and outlet_spin.joint_type == ArticulationType.CONTINUOUS
        and tuple(inlet_spin.axis) == (0.0, 1.0, 0.0)
        and tuple(outlet_spin.axis) == (0.0, 1.0, 0.0),
        details=(
            f"inlet_type={inlet_spin.joint_type}, inlet_axis={inlet_spin.axis}, "
            f"outlet_type={outlet_spin.joint_type}, outlet_axis={outlet_spin.axis}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
