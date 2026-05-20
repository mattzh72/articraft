from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Part,
    TestContext,
    TestReport,
)


def _box(part: Part, name: str, size, xyz, material) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _rack_wirework(
    part: Part,
    *,
    width: float,
    depth: float,
    wall_height: float,
    wire: float,
    runner_z: float,
    material,
    shallow: bool = False,
) -> None:
    """Open dishwasher basket made from intersecting coated-wire bars."""
    half_w = width / 2.0
    half_d = depth / 2.0

    # Sliding shoes / runners that sit on the fixed guide supports.
    _box(part, "runner_0", (0.024, depth + 0.035, 0.012), (-(half_w + 0.018), 0.0, runner_z), material)
    _box(part, "runner_1", (0.024, depth + 0.035, 0.012), ((half_w + 0.018), 0.0, runner_z), material)
    for i, sx in enumerate((-1.0, 1.0)):
        for j, y in enumerate((-half_d + 0.045, half_d - 0.045)):
            _box(part, f"runner_bridge_{i}_{j}", (0.042, 0.010, 0.035), (sx * (half_w + 0.006), y, runner_z + 0.014), material)

    # Bottom perimeter and open grid floor.
    z0 = 0.0
    _box(part, "front_floor_wire", (width, wire, wire), (0.0, -half_d, z0), material)
    _box(part, "rear_floor_wire", (width, wire, wire), (0.0, half_d, z0), material)
    _box(part, "side_floor_wire_0", (wire, depth, wire), (-half_w, 0.0, z0), material)
    _box(part, "side_floor_wire_1", (wire, depth, wire), (half_w, 0.0, z0), material)

    for idx, y in enumerate((-0.15, -0.075, 0.0, 0.075, 0.15)):
        _box(part, f"floor_cross_{idx}", (width, wire, wire), (0.0, y, z0), material)
    for idx, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        _box(part, f"floor_long_{idx}", (wire, depth, wire), (x, 0.0, z0 + 0.001), material)

    # Side walls and upper rim.
    for i, sx in enumerate((-1.0, 1.0)):
        _box(part, f"upper_side_wire_{i}", (wire, depth, wire), (sx * half_w, 0.0, wall_height), material)
        for j, y in enumerate((-half_d, -half_d / 2.0, 0.0, half_d / 2.0, half_d)):
            _box(part, f"side_upright_{i}_{j}", (wire, wire, wall_height), (sx * half_w, y, wall_height / 2.0), material)
    _box(part, "upper_front_wire", (width, wire, wire), (0.0, -half_d, wall_height), material)
    _box(part, "upper_rear_wire", (width, wire, wire), (0.0, half_d, wall_height), material)
    for idx, x in enumerate((-half_w, -half_w / 2.0, 0.0, half_w / 2.0, half_w)):
        _box(part, f"front_upright_{idx}", (wire, wire, wall_height), (x, -half_d, wall_height / 2.0), material)
        _box(part, f"rear_upright_{idx}", (wire, wire, wall_height), (x, half_d, wall_height / 2.0), material)

    if shallow:
        # Flat cutlery dividers rather than plate tines.
        for idx, x in enumerate((-0.15, -0.05, 0.05, 0.15)):
            _box(part, f"cutlery_divider_{idx}", (wire, depth - 0.035, wall_height * 0.75), (x, 0.0, wall_height * 0.38), material)
    else:
        # Upright plate tines; each tine intersects the grid floor.
        for row, y in enumerate((-0.15, -0.075, 0.0, 0.075)):
            for col, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
                _box(part, f"tine_{row}_{col}", (wire, wire, 0.072), (x, y, 0.036), material)


def _button_part(model: ArticulatedObject, door: Part, name: str, x: float, width: float, material) -> Part:
    button = model.part(name)
    button.visual(
        Box((width, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=material,
        name="cap",
    )
    model.articulation(
        f"door_to_{name}",
        ArticulationType.PRISMATIC,
        parent=door,
        child=button,
        origin=Origin(xyz=(x, 0.0, 0.818)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.08, lower=0.0, upper=0.006),
    )
    return button


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panel_ready_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    dark = model.material("dark_shadow", rgba=(0.035, 0.037, 0.04, 1.0))
    panel_mat = model.material("plain_panel", rgba=(0.86, 0.84, 0.78, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.012, 0.014, 1.0))
    wire_mat = model.material("gray_coated_wire", rgba=(0.23, 0.25, 0.26, 1.0))
    guide_mat = model.material("rail_plastic", rgba=(0.16, 0.17, 0.18, 1.0))
    green = model.material("start_green", rgba=(0.05, 0.52, 0.22, 1.0))
    red = model.material("cancel_red", rgba=(0.60, 0.08, 0.06, 1.0))
    button_gray = model.material("button_gray", rgba=(0.11, 0.12, 0.13, 1.0))

    # One connected fixed appliance shell: thin hollow tub, front rim, guides,
    # and water-feed supports.  The open front is covered by the articulated door.
    tub = model.part("tub")
    _box(tub, "floor_pan", (0.62, 0.60, 0.070), (0.0, 0.010, 0.035), stainless)
    _box(tub, "ceiling_panel", (0.62, 0.60, 0.030), (0.0, 0.010, 0.835), stainless)
    _box(tub, "rear_wall", (0.62, 0.030, 0.790), (0.0, 0.295, 0.445), stainless)
    _box(tub, "side_wall_0", (0.025, 0.60, 0.790), (-0.2975, 0.010, 0.445), stainless)
    _box(tub, "side_wall_1", (0.025, 0.60, 0.790), (0.2975, 0.010, 0.445), stainless)
    _box(tub, "front_sill", (0.62, 0.026, 0.055), (0.0, -0.290, 0.080), stainless)
    _box(tub, "front_header", (0.62, 0.026, 0.060), (0.0, -0.290, 0.805), stainless)
    _box(tub, "front_stile_0", (0.038, 0.026, 0.735), (-0.291, -0.290, 0.445), stainless)
    _box(tub, "front_stile_1", (0.038, 0.026, 0.735), (0.291, -0.290, 0.445), stainless)
    _box(tub, "door_gasket", (0.555, 0.010, 0.018), (0.0, -0.284, 0.765), rubber)

    # Fixed slide shelves/rails, connected into the side walls.
    rack_levels = {
        "lower": 0.220,
        "upper": 0.480,
        "cutlery": 0.675,
    }
    for level, z in rack_levels.items():
        for i, sx in enumerate((-1.0, 1.0)):
            _box(
                tub,
                f"{level}_guide_{i}",
                (0.080, 0.505, 0.012),
                (sx * 0.247, 0.0, z - 0.022),
                guide_mat,
            )
            _box(
                tub,
                f"{level}_guide_lip_{i}",
                (0.018, 0.505, 0.030),
                (sx * 0.279, 0.0, z - 0.010),
                guide_mat,
            )

    # Water feed, lower pedestal, and upper-arm support.
    tub.visual(Cylinder(radius=0.030, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.083)), material=stainless, name="lower_hub_pedestal")
    _box(tub, "rear_water_column", (0.055, 0.020, 0.395), (0.0, 0.278, 0.260), stainless)
    _box(tub, "upper_feed_arm", (0.050, 0.285, 0.030), (0.0, 0.140, 0.370), stainless)
    tub.visual(Cylinder(radius=0.024, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.382)), material=stainless, name="upper_hub_pedestal")

    # Dropdown door with a plain panel-ready exterior and a concealed top-edge
    # control strip.
    door = model.part("door")
    _box(door, "outer_panel", (0.596, 0.055, 0.790), (0.0, 0.0, 0.395), panel_mat)
    _box(door, "inner_liner", (0.545, 0.018, 0.640), (0.0, 0.035, 0.360), stainless)
    _box(door, "handle_shadow", (0.420, 0.004, 0.034), (0.0, -0.029, 0.685), dark)
    door.visual(
        Box((0.545, 0.052, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.804)),
        material=dark,
        name="control_edge",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.570),
        origin=Origin(xyz=(0.0, 0.0, -0.003), rpy=(0.0, pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    model.articulation(
        "tub_to_door",
        ArticulationType.REVOLUTE,
        parent=tub,
        child=door,
        origin=Origin(xyz=(0.0, -0.3275, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.75, lower=0.0, upper=1.78),
    )

    # Three sliding racks, each riding on its own guide supports.
    lower_rack = model.part("lower_rack")
    _rack_wirework(
        lower_rack,
        width=0.470,
        depth=0.455,
        wall_height=0.130,
        wire=0.006,
        runner_z=-0.010,
        material=wire_mat,
    )
    model.articulation(
        "tub_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=lower_rack,
        origin=Origin(xyz=(0.0, 0.0, rack_levels["lower"])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.385),
    )

    upper_rack = model.part("upper_rack")
    _rack_wirework(
        upper_rack,
        width=0.465,
        depth=0.445,
        wall_height=0.110,
        wire=0.0055,
        runner_z=-0.010,
        material=wire_mat,
    )
    model.articulation(
        "tub_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=upper_rack,
        origin=Origin(xyz=(0.0, 0.0, rack_levels["upper"])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.35, lower=0.0, upper=0.360),
    )

    cutlery_tray = model.part("cutlery_tray")
    _rack_wirework(
        cutlery_tray,
        width=0.485,
        depth=0.445,
        wall_height=0.055,
        wire=0.0045,
        runner_z=-0.010,
        material=wire_mat,
        shallow=True,
    )
    model.articulation(
        "tub_to_cutlery_tray",
        ArticulationType.PRISMATIC,
        parent=tub,
        child=cutlery_tray,
        origin=Origin(xyz=(0.0, 0.0, rack_levels["cutlery"])),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=0.330),
    )

    # Two independent continuous spray arms on vertical hubs.
    lower_arm = model.part("lower_wash_arm")
    lower_arm.visual(Cylinder(radius=0.026, length=0.030), origin=Origin(xyz=(0.0, 0.0, 0.015)), material=stainless, name="hub")
    _box(lower_arm, "spray_blade", (0.462, 0.036, 0.012), (0.0, 0.0, 0.035), stainless)
    _box(lower_arm, "jet_slots", (0.360, 0.006, 0.003), (0.0, -0.017, 0.040), dark)
    model.articulation(
        "tub_to_lower_wash_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=lower_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )

    upper_arm = model.part("upper_wash_arm")
    upper_arm.visual(Cylinder(radius=0.022, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.013)), material=stainless, name="hub")
    _box(upper_arm, "spray_blade", (0.390, 0.030, 0.010), (0.0, 0.0, 0.030), stainless)
    _box(upper_arm, "jet_slots", (0.300, 0.005, 0.003), (0.0, -0.014, 0.034), dark)
    model.articulation(
        "tub_to_upper_wash_arm",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.394)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=15.0),
    )

    # Concealed top-edge push controls.  They sit proud at rest and travel into
    # the control edge when pressed.
    _button_part(model, door, "start_button", -0.205, 0.050, green)
    _button_part(model, door, "cancel_button", -0.145, 0.050, red)
    for idx, x in enumerate((-0.055, 0.005, 0.065)):
        _button_part(model, door, f"program_button_{idx}", x, 0.046, button_gray)
    _button_part(model, door, "latch_button", 0.190, 0.070, button_gray)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    tub = object_model.get_part("tub")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("tub_to_door")

    closed_panel = ctx.part_element_world_aabb(door, elem="outer_panel")
    with ctx.pose({hinge: 1.45}):
        open_panel = ctx.part_element_world_aabb(door, elem="outer_panel")
    ctx.check(
        "door drops forward on lower hinge",
        closed_panel is not None
        and open_panel is not None
        and open_panel[0][1] < closed_panel[0][1] - 0.45
        and open_panel[1][2] < closed_panel[1][2] - 0.30,
        details=f"closed={closed_panel}, open={open_panel}",
    )

    # Rack runners are seated on their own fixed guides and retain insertion
    # when pulled outward.
    rack_specs = (
        ("lower_rack", "tub_to_lower_rack", "lower_guide_0", 0.385),
        ("upper_rack", "tub_to_upper_rack", "upper_guide_0", 0.360),
        ("cutlery_tray", "tub_to_cutlery_tray", "cutlery_guide_0", 0.330),
    )
    for rack_name, joint_name, guide_name, travel in rack_specs:
        rack = object_model.get_part(rack_name)
        joint = object_model.get_articulation(joint_name)
        ctx.expect_gap(
            rack,
            tub,
            axis="z",
            positive_elem="runner_0",
            negative_elem=guide_name,
            max_gap=0.002,
            max_penetration=0.0005,
            name=f"{rack_name} runner rests on guide",
        )
        ctx.expect_overlap(
            rack,
            tub,
            axes="xy",
            elem_a="runner_0",
            elem_b=guide_name,
            min_overlap=0.020,
            name=f"{rack_name} runner overlaps guide footprint",
        )
        rest = ctx.part_world_position(rack)
        with ctx.pose({hinge: 1.45, joint: travel}):
            extended = ctx.part_world_position(rack)
            ctx.expect_overlap(
                rack,
                tub,
                axes="y",
                elem_a="runner_0",
                elem_b=guide_name,
                min_overlap=0.070,
                name=f"{rack_name} remains retained at full slide",
            )
        ctx.check(
            f"{rack_name} slides out toward front",
            rest is not None and extended is not None and extended[1] < rest[1] - travel + 0.010,
            details=f"rest={rest}, extended={extended}",
        )

    # Spray arms rotate continuously about vertical hubs: the long blade swaps
    # from X-span to Y-span at a quarter turn.
    for arm_name, joint_name in (
        ("lower_wash_arm", "tub_to_lower_wash_arm"),
        ("upper_wash_arm", "tub_to_upper_wash_arm"),
    ):
        arm = object_model.get_part(arm_name)
        joint = object_model.get_articulation(joint_name)
        at_zero = ctx.part_element_world_aabb(arm, elem="spray_blade")
        with ctx.pose({joint: pi / 2.0}):
            at_quarter = ctx.part_element_world_aabb(arm, elem="spray_blade")
        ctx.check(
            f"{arm_name} turns around vertical hub",
            at_zero is not None
            and at_quarter is not None
            and (at_zero[1][0] - at_zero[0][0]) > 0.30
            and (at_quarter[1][1] - at_quarter[0][1]) > 0.30,
            details=f"zero={at_zero}, quarter={at_quarter}",
        )

    # Push buttons intentionally enter the control-edge recess when pressed.
    button_names = (
        "start_button",
        "cancel_button",
        "program_button_0",
        "program_button_1",
        "program_button_2",
        "latch_button",
    )
    for button_name in button_names:
        button = object_model.get_part(button_name)
        joint = object_model.get_articulation(f"door_to_{button_name}")
        ctx.allow_overlap(
            button,
            door,
            elem_a="cap",
            elem_b="control_edge",
            reason="Push button cap deliberately travels into the concealed top-edge control recess.",
        )
        rest = ctx.part_world_position(button)
        ctx.expect_gap(
            button,
            door,
            axis="z",
            positive_elem="cap",
            negative_elem="control_edge",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"{button_name} sits proud on control edge",
        )
        with ctx.pose({joint: 0.006}):
            pressed = ctx.part_world_position(button)
            ctx.expect_gap(
                button,
                door,
                axis="z",
                positive_elem="cap",
                negative_elem="control_edge",
                max_penetration=0.007,
                name=f"{button_name} presses into control edge",
            )
        ctx.check(
            f"{button_name} moves inward",
            rest is not None and pressed is not None and pressed[2] < rest[2] - 0.004,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
