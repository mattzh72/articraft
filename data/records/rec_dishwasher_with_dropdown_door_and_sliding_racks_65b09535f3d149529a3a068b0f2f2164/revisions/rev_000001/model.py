from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cyl(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_static_rails(tub, z, prefix, material):
    # Rail ledges are part of the tub and slightly bury into the side walls so
    # they read as real supported slide tracks rather than floating strips.
    for i, x in enumerate((-0.264, 0.264)):
        _box(tub, f"{prefix}_rail_{i}", (0.022, 0.510, 0.018), (x, 0.0, z), material)


def _add_wire_rack(part, prefix, width, depth, basket_height, material):
    side_x = width * 0.5
    front_y = -depth * 0.5
    rear_y = depth * 0.5

    # Rectangular sliding runners and basket rims.
    for i, x in enumerate((-side_x, side_x)):
        _box(part, f"{prefix}_runner_{i}", (0.018, depth, 0.016), (x, 0.0, 0.0), material)
        _box(part, f"{prefix}_top_side_{i}", (0.014, depth, 0.012), (x, 0.0, basket_height), material)
    for i, y in enumerate((front_y, rear_y)):
        _box(part, f"{prefix}_bottom_cross_{i}", (width, 0.016, 0.016), (0.0, y, 0.0), material)
        _box(part, f"{prefix}_top_cross_{i}", (width, 0.014, 0.012), (0.0, y, basket_height), material)

    for ix, x in enumerate((-side_x, side_x)):
        for iy, y in enumerate((front_y, rear_y)):
            _box(
                part,
                f"{prefix}_corner_post_{ix}_{iy}",
                (0.012, 0.012, basket_height),
                (x, y, basket_height * 0.5),
                material,
            )

    # Open wire-grid floor: rods cross and overlap at intersections so the rack
    # part remains one physically connected wire basket.
    for row, y in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        if abs(y) < depth * 0.48:
            _box(part, f"{prefix}_grid_cross_{row}", (width, 0.007, 0.008), (0.0, y, 0.010), material)
    for col, x in enumerate((-0.16, -0.08, 0.0, 0.08, 0.16)):
        if abs(x) < width * 0.48:
            _box(part, f"{prefix}_grid_long_{col}", (0.007, depth, 0.008), (x, 0.0, 0.010), material)

    # Plate tines rise from grid intersections and stay attached to the floor.
    tine_positions = [
        (-0.16, -0.08),
        (-0.08, -0.08),
        (0.0, -0.08),
        (0.08, -0.08),
        (0.16, -0.08),
        (-0.16, 0.08),
        (-0.08, 0.08),
        (0.0, 0.08),
        (0.08, 0.08),
        (0.16, 0.08),
        (-0.08, 0.16),
        (0.08, 0.16),
    ]
    for i, (x, y) in enumerate(tine_positions):
        if abs(x) < width * 0.42 and abs(y) < depth * 0.42:
            _box(part, f"{prefix}_tine_{i}", (0.007, 0.007, 0.085), (x, y, 0.052), material)


def _add_spray_arm(part, prefix, material, dark_material):
    _cyl(part, f"{prefix}_hub", 0.035, 0.038, (0.0, 0.0, 0.0), material)
    _box(part, f"{prefix}_arm_long", (0.460, 0.034, 0.012), (0.0, 0.0, 0.0), material)
    _box(part, f"{prefix}_arm_short", (0.034, 0.230, 0.010), (0.0, 0.0, 0.002), material)
    for i, (x, y) in enumerate(((-0.16, 0.0), (-0.07, 0.0), (0.08, 0.0), (0.18, 0.0), (0.0, -0.08), (0.0, 0.08))):
        _box(part, f"{prefix}_nozzle_{i}", (0.024, 0.006, 0.004), (x, y, 0.006), dark_material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_tall_tub_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.70, 1.0))
    dark = model.material("black_glass", rgba=(0.01, 0.012, 0.014, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    rack_coating = model.material("dark_gray_nylon", rgba=(0.10, 0.11, 0.12, 1.0))
    light_plastic = model.material("light_gray_plastic", rgba=(0.72, 0.74, 0.74, 1.0))
    white_mark = model.material("white_marking", rgba=(0.92, 0.94, 0.92, 1.0))
    blue_cap = model.material("translucent_blue_cap", rgba=(0.08, 0.22, 0.55, 1.0))

    # Root tub: full built-in scale, thin-wall open front, visible hollow cavity.
    tub = model.part("tub")
    _box(tub, "left_wall", (0.025, 0.600, 0.780), (-0.2875, 0.0, 0.460), stainless)
    _box(tub, "right_wall", (0.025, 0.600, 0.780), (0.2875, 0.0, 0.460), stainless)
    _box(tub, "back_wall", (0.600, 0.025, 0.780), (0.0, 0.2875, 0.460), stainless)
    _box(tub, "bottom_pan", (0.600, 0.600, 0.030), (0.0, 0.0, 0.055), stainless)
    _box(tub, "top_panel", (0.600, 0.600, 0.030), (0.0, 0.0, 0.855), stainless)
    _box(tub, "lower_hinge_leaf", (0.500, 0.028, 0.020), (0.0, -0.299, 0.070), stainless)
    _box(tub, "front_left_stile", (0.026, 0.030, 0.760), (-0.287, -0.285, 0.465), stainless)
    _box(tub, "front_right_stile", (0.026, 0.030, 0.760), (0.287, -0.285, 0.465), stainless)
    _box(tub, "front_top_rail", (0.600, 0.030, 0.026), (0.0, -0.285, 0.835), stainless)
    _box(tub, "front_bottom_sill", (0.600, 0.028, 0.024), (0.0, -0.285, 0.115), stainless)
    _box(tub, "gasket_top", (0.540, 0.012, 0.020), (0.0, -0.290, 0.805), rubber)
    _box(tub, "gasket_bottom", (0.540, 0.012, 0.018), (0.0, -0.290, 0.125), rubber)
    _box(tub, "gasket_left", (0.018, 0.012, 0.680), (-0.255, -0.290, 0.465), rubber)
    _box(tub, "gasket_right", (0.018, 0.012, 0.680), (0.255, -0.290, 0.465), rubber)
    _cyl(tub, "sump_well", 0.085, 0.018, (0.0, 0.055, 0.075), light_plastic)
    _cyl(tub, "filter_screen", 0.060, 0.010, (0.0, 0.055, 0.089), dark)
    _cyl(tub, "lower_spray_standpipe", 0.025, 0.053, (0.0, 0.020, 0.1095), light_plastic)
    _box(tub, "upper_spray_feed_tube", (0.026, 0.230, 0.026), (0.0, 0.170, 0.421), light_plastic)

    _add_static_rails(tub, 0.220, "lower", stainless)
    _add_static_rails(tub, 0.485, "upper", stainless)
    _add_static_rails(tub, 0.655, "tray", stainless)

    # Broad door is authored open at q=0 so the hollow tub and rack levels are visible.
    # At the upper limit it closes upward against the front gasket.
    door = model.part("door")
    _box(door, "outer_slab", (0.610, 0.760, 0.050), (0.0, -0.405, 0.0), stainless)
    _box(door, "inner_liner_pan", (0.545, 0.620, 0.010), (0.0, -0.390, 0.029), stainless)
    _box(door, "liner_left_wall", (0.018, 0.610, 0.022), (-0.282, -0.390, 0.044), stainless)
    _box(door, "liner_right_wall", (0.018, 0.610, 0.022), (0.282, -0.390, 0.044), stainless)
    _box(door, "liner_top_wall", (0.560, 0.018, 0.022), (0.0, -0.695, 0.044), stainless)
    _box(door, "liner_bottom_wall", (0.560, 0.018, 0.022), (0.0, -0.085, 0.044), stainless)
    _box(door, "detergent_cup", (0.145, 0.090, 0.022), (0.125, -0.255, 0.044), light_plastic)
    _box(door, "detergent_lid_seam", (0.125, 0.006, 0.004), (0.125, -0.218, 0.054), dark)
    _cyl(door, "bottom_hinge_bar", 0.012, 0.500, (0.0, -0.020, 0.0), stainless, rpy=(0.0, pi / 2.0, 0.0))

    _box(door, "control_bank", (0.545, 0.092, 0.008), (0.0, -0.705, -0.0285), dark)
    _box(door, "handle_recess", (0.440, 0.066, 0.006), (0.0, -0.600, -0.0285), dark)
    _box(door, "handle_bar", (0.420, 0.014, 0.014), (0.0, -0.600, -0.044), stainless)
    _box(door, "handle_mount_0", (0.018, 0.050, 0.024), (-0.205, -0.600, -0.036), stainless)
    _box(door, "handle_mount_1", (0.018, 0.050, 0.024), (0.205, -0.600, -0.036), stainless)

    door_hinge = model.articulation(
        "tub_to_door",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.305, 0.070)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.0, lower=0.0, upper=1.45),
    )

    # Three independent rack levels on supported side rails.
    lower_rack = model.part("lower_rack")
    _add_wire_rack(lower_rack, "lower", 0.488, 0.460, 0.140, rack_coating)
    upper_rack = model.part("upper_rack")
    _add_wire_rack(upper_rack, "upper", 0.488, 0.440, 0.125, rack_coating)
    tray = model.part("utensil_tray")
    _add_wire_rack(tray, "tray", 0.488, 0.410, 0.060, rack_coating)
    for i, x in enumerate((-0.14, -0.07, 0.0, 0.07, 0.14)):
        _box(tray, f"divider_{i}", (0.006, 0.360, 0.050), (x, 0.0, 0.031), rack_coating)

    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.005, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.430),
    )
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.005, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=65.0, velocity=0.35, lower=0.0, upper=0.400),
    )
    model.articulation(
        "tub_to_utensil_tray",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=tray,
        origin=Origin(xyz=(0.0, -0.005, 0.670)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.30, lower=0.0, upper=0.360),
    )

    # Continuous rotating spray arms on central hubs inside the tub.
    lower_spray = model.part("lower_spray_arm")
    _add_spray_arm(lower_spray, "lower_spray", light_plastic, dark)
    model.articulation(
        "tub_to_lower_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray,
        origin=Origin(xyz=(0.0, 0.020, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    upper_spray = model.part("upper_spray_arm")
    _add_spray_arm(upper_spray, "upper_spray", light_plastic, dark)
    model.articulation(
        "tub_to_upper_spray_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_spray,
        origin=Origin(xyz=(0.0, 0.020, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    # Top control bank: one continuous dial and six independent prismatic push buttons.
    dial = model.part("dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.070,
            0.018,
            body_style="cylindrical",
            grip=KnobGrip(style="ribbed", count=24, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "cycle_dial",
    )
    dial.visual(dial_mesh, material=light_plastic, name="dial_cap")
    _box(dial, "dial_pointer", (0.006, 0.028, 0.003), (0.0, 0.014, -0.0085), white_mark)
    model.articulation(
        "door_to_dial",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=dial,
        origin=Origin(xyz=(0.0, -0.705, -0.0415)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    button_xs = (-0.225, -0.165, -0.105, 0.105, 0.165, 0.225)
    for i, x in enumerate(button_xs):
        button = model.part(f"option_button_{i}")
        _box(button, "button_cap", (0.046, 0.026, 0.012), (0.0, 0.0, 0.0), light_plastic)
        _box(button, "button_label", (0.024, 0.003, 0.0015), (0.0, 0.0, -0.0058), white_mark)
        model.articulation(
            f"door_to_option_button_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.705, -0.0385)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    # Rinse-aid cap is a separate continuously rotating part on the inner door liner.
    cap = model.part("rinse_cap")
    _cyl(cap, "cap_disk", 0.036, 0.010, (0.0, 0.0, 0.005), blue_cap)
    _box(cap, "cap_grip", (0.014, 0.054, 0.006), (0.0, 0.0, 0.013), light_plastic)
    model.articulation(
        "door_to_rinse_cap",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cap,
        origin=Origin(xyz=(-0.135, -0.285, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0),
    )

    # Keep a handle to avoid the appearance that the hinge variable is unused in
    # this construction-heavy function; tests retrieve by name.
    _ = door_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door = object_model.get_part("door")
    tub = object_model.get_part("tub")
    door_hinge = object_model.get_articulation("tub_to_door")

    # The authored default is the lowered loading position; the upper limit is
    # the closed built-in front.
    open_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.45}):
        closed_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door closes upward from lowered hinge",
        open_aabb is not None
        and closed_aabb is not None
        and open_aabb[1][2] < 0.16
        and closed_aabb[1][2] > 0.78,
        details=f"open_aabb={open_aabb}, closed_aabb={closed_aabb}",
    )

    # Rack slides move outward toward the user and still retain insertion on
    # their supported rails at full travel.
    for rack_name, joint_name, runner_name, rail_name, travel in (
        ("lower_rack", "tub_to_lower_rack", "lower_runner_1", "lower_rail_1", 0.430),
        ("upper_rack", "tub_to_upper_rack", "upper_runner_1", "upper_rail_1", 0.400),
        ("utensil_tray", "tub_to_utensil_tray", "tray_runner_1", "tray_rail_1", 0.360),
    ):
        rack = object_model.get_part(rack_name)
        joint = object_model.get_articulation(joint_name)
        rest_pos = ctx.part_world_position(rack)
        ctx.expect_overlap(
            rack,
            tub,
            axes="y",
            elem_a=runner_name,
            elem_b=rail_name,
            min_overlap=0.36,
            name=f"{rack_name} runner overlaps its rail when stowed",
        )
        with ctx.pose({joint: travel}):
            extended_pos = ctx.part_world_position(rack)
            ctx.expect_overlap(
                rack,
                tub,
                axes="y",
                elem_a=runner_name,
                elem_b=rail_name,
                min_overlap=0.045,
                name=f"{rack_name} retains rail insertion when extended",
            )
        ctx.check(
            f"{rack_name} slides outward",
            rest_pos is not None and extended_pos is not None and extended_pos[1] < rest_pos[1] - 0.25,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    continuous_joints = (
        "door_to_dial",
        "door_to_rinse_cap",
        "tub_to_lower_spray_arm",
        "tub_to_upper_spray_arm",
    )
    for joint_name in continuous_joints:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"type={joint.articulation_type}",
        )

    for i in range(6):
        joint = object_model.get_articulation(f"door_to_option_button_{i}")
        ctx.check(
            f"option_button_{i} is prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC,
            details=f"type={joint.articulation_type}",
        )

    return ctx.report()


object_model = build_object_model()
