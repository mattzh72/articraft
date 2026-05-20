from __future__ import annotations

import math

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
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material: Material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_wire_rack(
    part,
    *,
    half_width: float,
    half_depth: float,
    height: float,
    wire_radius: float,
    material: Material,
    channel_offset: float,
) -> None:
    """Build a connected open dishwasher rack from round wire members."""
    bottom_z = 0.0
    top_z = height
    xs = [-half_width, -half_width * 0.5, 0.0, half_width * 0.5, half_width]
    ys = [-half_depth, -half_depth * 0.5, 0.0, half_depth * 0.5, half_depth]

    # Perimeter bottom and top baskets.
    corners_bottom = [
        (-half_width, -half_depth, bottom_z),
        (half_width, -half_depth, bottom_z),
        (half_width, half_depth, bottom_z),
        (-half_width, half_depth, bottom_z),
    ]
    corners_top = [(x, y, top_z) for x, y, _ in corners_bottom]
    for corners in (corners_bottom, corners_top):
        for i in range(4):
            _add_member(part, corners[i], corners[(i + 1) % 4], wire_radius * 1.25, material)

    # Vertical corner posts and two intermediate rear posts make the basket read tall and rigid.
    for x, y, _ in corners_bottom:
        _add_member(part, (x, y, bottom_z), (x, y, top_z), wire_radius * 1.15, material)
    for x in (-half_width * 0.45, half_width * 0.45):
        _add_member(part, (x, half_depth, bottom_z), (x, half_depth, top_z), wire_radius, material)

    # Open bottom grate.
    for x in xs[1:-1]:
        _add_member(part, (x, -half_depth, bottom_z), (x, half_depth, bottom_z), wire_radius * 0.82, material)
    for y in ys[1:-1]:
        _add_member(part, (-half_width, y, bottom_z), (half_width, y, bottom_z), wire_radius * 0.82, material)

    # Front and rear side wires, intentionally open between them.
    for z in (height * 0.36, height * 0.68):
        _add_member(part, (-half_width, -half_depth, z), (half_width, -half_depth, z), wire_radius * 0.9, material)
        _add_member(part, (-half_width, half_depth, z), (half_width, half_depth, z), wire_radius * 0.9, material)
        _add_member(part, (-half_width, -half_depth, z), (-half_width, half_depth, z), wire_radius * 0.9, material)
        _add_member(part, (half_width, -half_depth, z), (half_width, half_depth, z), wire_radius * 0.9, material)

    # Dish tines welded to the bottom grid.
    tine_count = 7
    for i in range(tine_count):
        x = -half_width * 0.65 + i * (half_width * 1.30 / (tine_count - 1))
        for y in (-half_depth * 0.28, half_depth * 0.10):
            start_y = -half_depth * 0.5 if y < 0.0 else 0.0
            _add_member(
                part,
                (x, start_y, bottom_z),
                (x + 0.018, start_y + 0.018, top_z * 0.62),
                wire_radius * 0.62,
                material,
            )

    # Heavy sliding side channels that line up with the tub rails.
    for x in (-half_width - channel_offset, half_width + channel_offset):
        side = -1.0 if x < 0.0 else 1.0
        _add_member(part, (x, -half_depth * 0.93, height * 0.17), (x, half_depth * 0.93, height * 0.17), wire_radius * 1.55, material)
        _add_member(part, (x, -half_depth * 0.88, height * 0.32), (x, half_depth * 0.88, height * 0.32), wire_radius * 1.15, material)
        for y in (-half_depth, 0.0, half_depth):
            _add_member(part, (side * half_width, y, height * 0.17), (x, y, height * 0.17), wire_radius * 0.9, material)
            _add_member(part, (side * half_width, y, height * 0.32), (x, y, height * 0.32), wire_radius * 0.75, material)
        for y_side, y_channel in ((-half_depth, -half_depth * 0.93), (half_depth, half_depth * 0.93)):
            _add_member(part, (side * half_width, y_side, height * 0.17), (x, y_channel, height * 0.17), wire_radius * 0.9, material)
        for y_side, y_channel in ((-half_depth, -half_depth * 0.88), (half_depth, half_depth * 0.88)):
            _add_member(part, (side * half_width, y_side, height * 0.32), (x, y_channel, height * 0.32), wire_radius * 0.75, material)


def _add_tub_rail_pair(tub, *, z: float, material: Material, name_prefix: str) -> None:
    for side, x in (("near", -0.286), ("far", 0.286)):
        tub.visual(
            Box((0.018, 0.500, 0.024)),
            origin=Origin(xyz=(x, -0.010, z)),
            material=material,
            name=f"{name_prefix}_{side}_rail",
        )
        tub.visual(
            Box((0.030, 0.035, 0.042)),
            origin=Origin(xyz=(x, -0.285, z - 0.006)),
            material=material,
            name=f"{name_prefix}_{side}_front_stop",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.69, 1.0))
    dark_stainless = model.material("shadowed_stainless", rgba=(0.48, 0.49, 0.48, 1.0))
    liner_steel = model.material("liner_steel", rgba=(0.78, 0.80, 0.79, 1.0))
    rack_coating = model.material("rack_coating", rgba=(0.88, 0.89, 0.87, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.06, 0.065, 0.07, 1.0))
    black_glass = model.material("black_glass", rgba=(0.02, 0.025, 0.03, 1.0))
    detergent_blue = model.material("detergent_blue", rgba=(0.28, 0.48, 0.72, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.02, 0.022, 0.022, 1.0))

    tub = model.part("tub")
    # Open front stainless tub and outer cabinet: side, back, top, and bottom walls.
    tub.visual(Box((0.025, 0.605, 0.790)), origin=Origin(xyz=(-0.305, 0.000, 0.430)), material=stainless, name="side_wall_0")
    tub.visual(Box((0.025, 0.605, 0.790)), origin=Origin(xyz=(0.305, 0.000, 0.430)), material=stainless, name="side_wall_1")
    tub.visual(Box((0.610, 0.025, 0.790)), origin=Origin(xyz=(0.000, 0.302, 0.430)), material=stainless, name="back_wall")
    tub.visual(Box((0.610, 0.605, 0.035)), origin=Origin(xyz=(0.000, 0.000, 0.0175)), material=stainless, name="bottom_pan")
    tub.visual(Box((0.610, 0.605, 0.035)), origin=Origin(xyz=(0.000, 0.000, 0.8425)), material=stainless, name="top_panel")
    # Rolled front flange makes the tub read open and hollow rather than a solid block.
    tub.visual(Box((0.035, 0.035, 0.825)), origin=Origin(xyz=(-0.312, -0.318, 0.430)), material=dark_stainless, name="front_stile_0")
    tub.visual(Box((0.035, 0.035, 0.825)), origin=Origin(xyz=(0.312, -0.318, 0.430)), material=dark_stainless, name="front_stile_1")
    tub.visual(Box((0.625, 0.035, 0.035)), origin=Origin(xyz=(0.000, -0.318, 0.845)), material=dark_stainless, name="front_header")
    tub.visual(Box((0.625, 0.035, 0.035)), origin=Origin(xyz=(0.000, -0.318, 0.035)), material=dark_stainless, name="front_sill")
    tub.visual(Box((0.020, 0.009, 0.680)), origin=Origin(xyz=(-0.286, -0.332, 0.435)), material=seal_rubber, name="gasket_side_0")
    tub.visual(Box((0.020, 0.009, 0.680)), origin=Origin(xyz=(0.286, -0.332, 0.435)), material=seal_rubber, name="gasket_side_1")
    tub.visual(Box((0.572, 0.009, 0.020)), origin=Origin(xyz=(0.000, -0.332, 0.780)), material=seal_rubber, name="gasket_top")
    tub.visual(Box((0.572, 0.009, 0.020)), origin=Origin(xyz=(0.000, -0.332, 0.115)), material=seal_rubber, name="gasket_bottom")

    _add_tub_rail_pair(tub, z=0.230, material=dark_stainless, name_prefix="lower")
    _add_tub_rail_pair(tub, z=0.525, material=dark_stainless, name_prefix="upper")

    # Water feed plumbing and two central vertical supports for rotating spray arms.
    tub.visual(Cylinder(radius=0.014, length=0.120), origin=Origin(xyz=(0.000, 0.000, 0.095)), material=dark_stainless, name="lower_spray_post")
    tub.visual(Cylinder(radius=0.014, length=0.445), origin=Origin(xyz=(0.000, 0.288, 0.290)), material=dark_stainless, name="rear_feed_pipe")
    tub.visual(Cylinder(radius=0.011, length=0.236), origin=Origin(xyz=(0.000, 0.170, 0.446), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_stainless, name="upper_feed_pipe")
    tub.visual(Box((0.022, 0.070, 0.008)), origin=Origin(xyz=(0.000, 0.035, 0.438)), material=dark_stainless, name="upper_feed_connector")
    tub.visual(Cylinder(radius=0.014, length=0.055), origin=Origin(xyz=(0.000, 0.000, 0.423)), material=dark_stainless, name="upper_spray_post")

    lower_rack = model.part("lower_rack")
    _add_wire_rack(lower_rack, half_width=0.238, half_depth=0.238, height=0.220, wire_radius=0.0042, material=rack_coating, channel_offset=0.034)

    upper_rack = model.part("upper_rack")
    _add_wire_rack(upper_rack, half_width=0.218, half_depth=0.205, height=0.235, wire_radius=0.0038, material=rack_coating, channel_offset=0.054)
    # Extra tall glass supports in the upper rack.
    for x in (-0.145, -0.090, -0.035, 0.035, 0.090, 0.145):
        _add_member(upper_rack, (x, -0.1025, 0.0), (x + 0.020, -0.084, 0.190), 0.0026, rack_coating)

    lower_spray_arm = model.part("lower_spray_arm")
    lower_spray_arm.visual(Cylinder(radius=0.028, length=0.018), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=dark_plastic, name="hub")
    lower_spray_arm.visual(Box((0.455, 0.034, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=dark_plastic, name="spray_blade")
    lower_spray_arm.visual(Cylinder(radius=0.018, length=0.034), origin=Origin(xyz=(-0.228, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="rounded_tip_0")
    lower_spray_arm.visual(Cylinder(radius=0.018, length=0.034), origin=Origin(xyz=(0.228, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="rounded_tip_1")
    for x in (-0.155, -0.075, 0.075, 0.155):
        lower_spray_arm.visual(Box((0.026, 0.006, 0.004)), origin=Origin(xyz=(x, 0.018 if x < 0 else -0.018, 0.019)), material=liner_steel, name=f"nozzle_{x:+.2f}")

    upper_spray_arm = model.part("upper_spray_arm")
    upper_spray_arm.visual(Cylinder(radius=0.023, length=0.016), origin=Origin(xyz=(0.0, 0.0, 0.008)), material=dark_plastic, name="hub")
    upper_spray_arm.visual(Box((0.385, 0.030, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=dark_plastic, name="spray_blade")
    upper_spray_arm.visual(Cylinder(radius=0.016, length=0.030), origin=Origin(xyz=(-0.193, 0.0, 0.009), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="rounded_tip_0")
    upper_spray_arm.visual(Cylinder(radius=0.016, length=0.030), origin=Origin(xyz=(0.193, 0.0, 0.009), rpy=(math.pi / 2.0, 0.0, 0.0)), material=dark_plastic, name="rounded_tip_1")

    door = model.part("door")
    door.visual(Box((0.612, 0.046, 0.730)), origin=Origin(xyz=(0.000, -0.024, 0.365)), material=stainless, name="outer_panel")
    door.visual(Box((0.612, 0.020, 0.118)), origin=Origin(xyz=(0.000, -0.0555, 0.668)), material=dark_stainless, name="control_fascia")
    door.visual(Box((0.530, 0.006, 0.580)), origin=Origin(xyz=(0.000, 0.002, 0.385)), material=liner_steel, name="inner_liner_sheet")
    door.visual(Box((0.018, 0.018, 0.590)), origin=Origin(xyz=(-0.255, 0.011, 0.385)), material=liner_steel, name="liner_side_rib_0")
    door.visual(Box((0.018, 0.018, 0.590)), origin=Origin(xyz=(0.255, 0.011, 0.385)), material=liner_steel, name="liner_side_rib_1")
    door.visual(Box((0.520, 0.018, 0.020)), origin=Origin(xyz=(0.000, 0.011, 0.105)), material=liner_steel, name="liner_lower_rib")
    door.visual(Box((0.520, 0.018, 0.020)), origin=Origin(xyz=(0.000, 0.011, 0.660)), material=liner_steel, name="liner_upper_rib")
    door.visual(Box((0.160, 0.012, 0.105)), origin=Origin(xyz=(-0.150, 0.010, 0.345)), material=detergent_blue, name="detergent_cup")
    door.visual(Box((0.178, 0.006, 0.020)), origin=Origin(xyz=(-0.150, 0.010, 0.400)), material=liner_steel, name="cup_hinge_mount")

    detergent_lid = model.part("detergent_lid")
    detergent_lid.visual(Cylinder(radius=0.007, length=0.124), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=detergent_blue, name="hinge_barrel")
    detergent_lid.visual(Box((0.130, 0.007, 0.082)), origin=Origin(xyz=(0.000, 0.004, -0.041)), material=detergent_blue, name="lid_panel")
    detergent_lid.visual(Box((0.098, 0.003, 0.020)), origin=Origin(xyz=(0.000, 0.008, -0.067)), material=liner_steel, name="latch_ridge")

    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(Cylinder(radius=0.043, length=0.028), origin=Origin(xyz=(0.0, 0.0, 0.014)), material=dark_plastic, name="knob_skirt")
    cycle_knob.visual(Cylinder(radius=0.033, length=0.010), origin=Origin(xyz=(0.0, 0.0, 0.033)), material=black_glass, name="knob_face")
    cycle_knob.visual(Box((0.006, 0.046, 0.004)), origin=Origin(xyz=(0.0, 0.012, 0.040)), material=liner_steel, name="pointer_mark")

    start_button = model.part("start_button")
    start_button.visual(Cylinder(radius=0.018, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.006)), material=black_glass, name="button_cap")

    delay_button = model.part("delay_button")
    delay_button.visual(Cylinder(radius=0.016, length=0.012), origin=Origin(xyz=(0.0, 0.0, 0.006)), material=black_glass, name="button_cap")

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.000, -0.335, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.8, lower=0.0, upper=1.52),
    )
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.000, -0.020, 0.205)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.35, lower=0.0, upper=0.405),
    )
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.000, -0.020, 0.490)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.350),
    )
    model.articulation(
        "lower_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.000, 0.000, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "upper_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.000, 0.000, 0.450)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "detergent_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_lid,
        origin=Origin(xyz=(-0.150, 0.021, 0.400)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    front_control_rpy = (math.pi / 2.0, 0.0, 0.0)
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cycle_knob,
        origin=Origin(xyz=(-0.180, -0.0655, 0.682), rpy=front_control_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=6.0),
    )
    model.articulation(
        "start_push",
        ArticulationType.PRISMATIC,
        parent=door,
        child=start_button,
        origin=Origin(xyz=(0.110, -0.0655, 0.682), rpy=front_control_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=-0.006, upper=0.0),
    )
    model.articulation(
        "delay_push",
        ArticulationType.PRISMATIC,
        parent=door,
        child=delay_button,
        origin=Origin(xyz=(0.185, -0.0655, 0.682), rpy=front_control_rpy),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=0.05, lower=-0.006, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    lower_spray_arm = object_model.get_part("lower_spray_arm")
    detergent_lid = object_model.get_part("detergent_lid")
    cycle_knob = object_model.get_part("cycle_knob")
    start_button = object_model.get_part("start_button")

    door_hinge = object_model.get_articulation("door_hinge")
    lower_slide = object_model.get_articulation("lower_rack_slide")
    upper_slide = object_model.get_articulation("upper_rack_slide")
    detergent_hinge = object_model.get_articulation("detergent_hinge")
    start_push = object_model.get_articulation("start_push")

    ctx.expect_origin_gap(tub, door, axis="y", min_gap=0.30, name="closed door hinge is in front of tub")
    ctx.expect_within(lower_rack, tub, axes="xz", margin=0.02, name="lower rack fits inside tub width and height")
    ctx.expect_within(upper_rack, tub, axes="xz", margin=0.02, name="upper rack fits inside tub width and height")
    ctx.expect_contact(lower_spray_arm, tub, elem_a="hub", elem_b="lower_spray_post", contact_tol=0.004, name="lower spray arm sits on central support")
    ctx.expect_overlap(cycle_knob, door, axes="xz", min_overlap=0.040, name="cycle knob mounted on fascia")

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward and outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.15
        and open_door_aabb[1][2] < closed_door_aabb[1][2] - 0.15,
        details=f"closed={closed_door_aabb}, opened={open_door_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({lower_slide: 0.32, upper_slide: 0.28}):
        lower_extended = ctx.part_world_position(lower_rack)
        upper_extended = ctx.part_world_position(upper_rack)
    ctx.check(
        "racks slide toward the user",
        lower_rest is not None
        and upper_rest is not None
        and lower_extended is not None
        and upper_extended is not None
        and lower_extended[1] < lower_rest[1] - 0.25
        and upper_extended[1] < upper_rest[1] - 0.20,
        details=f"lower {lower_rest}->{lower_extended}, upper {upper_rest}->{upper_extended}",
    )

    lid_closed_aabb = ctx.part_world_aabb(detergent_lid)
    with ctx.pose({detergent_hinge: 0.9}):
        lid_open_aabb = ctx.part_world_aabb(detergent_lid)
    ctx.check(
        "detergent lid swings out from inner door",
        lid_closed_aabb is not None and lid_open_aabb is not None and lid_open_aabb[1][1] > lid_closed_aabb[1][1] + 0.020,
        details=f"closed={lid_closed_aabb}, open={lid_open_aabb}",
    )

    button_rest = ctx.part_world_position(start_button)
    with ctx.pose({start_push: -0.005}):
        button_pressed = ctx.part_world_position(start_button)
    ctx.check(
        "start button depresses into fascia",
        button_rest is not None and button_pressed is not None and button_pressed[1] > button_rest[1] + 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
