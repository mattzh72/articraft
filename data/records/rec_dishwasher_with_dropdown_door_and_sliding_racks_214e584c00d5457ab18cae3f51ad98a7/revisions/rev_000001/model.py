from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


WIDTH = 0.64
DEPTH = 0.66
HEIGHT = 0.86


def _add_rack(part, *, width: float, depth: float, height: float, rod: float, material: str) -> None:
    """Wire basket rack made from physically connected coated rods."""
    half_w = width * 0.5
    half_d = depth * 0.5
    # Bottom rectangle.
    part.visual(Box((width, rod, rod)), origin=Origin(xyz=(0.0, -half_d, rod * 0.5)), material=material, name="front_bottom_rail")
    part.visual(Box((width, rod, rod)), origin=Origin(xyz=(0.0, half_d, rod * 0.5)), material=material, name="rear_bottom_rail")
    part.visual(Box((rod, depth, rod)), origin=Origin(xyz=(-half_w, 0.0, rod * 0.5)), material=material, name="side_bottom_rail_0")
    part.visual(Box((rod, depth, rod)), origin=Origin(xyz=(half_w, 0.0, rod * 0.5)), material=material, name="side_bottom_rail_1")

    # Top rectangle and corner posts make the basket read as a deep rack.
    part.visual(Box((width, rod, rod)), origin=Origin(xyz=(0.0, -half_d, height)), material=material, name="front_top_rail")
    part.visual(Box((width, rod, rod)), origin=Origin(xyz=(0.0, half_d, height)), material=material, name="rear_top_rail")
    part.visual(Box((rod, depth, rod)), origin=Origin(xyz=(-half_w, 0.0, height)), material=material, name="side_top_rail_0")
    part.visual(Box((rod, depth, rod)), origin=Origin(xyz=(half_w, 0.0, height)), material=material, name="side_top_rail_1")
    for xi, x in enumerate((-half_w, half_w)):
        for yi, y in enumerate((-half_d, half_d)):
            part.visual(
                Box((rod, rod, height)),
                origin=Origin(xyz=(x, y, height * 0.5)),
                material=material,
                name=f"corner_post_{xi}_{yi}",
            )

    # Bottom dish-support grid. These rods overlap the perimeter rails, so the
    # authored rack is one supported assembly rather than loose floating wires.
    for index, x in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        part.visual(
            Box((rod * 0.75, depth, rod * 0.75)),
            origin=Origin(xyz=(x, 0.0, 0.010)),
            material=material,
            name=f"long_grid_{index}",
        )
    for index, y in enumerate((-0.18, -0.09, 0.0, 0.09, 0.18)):
        part.visual(
            Box((width, rod * 0.75, rod * 0.75)),
            origin=Origin(xyz=(0.0, y, 0.010)),
            material=material,
            name=f"cross_grid_{index}",
        )


def _add_cutlery_caddy(part, *, material: str) -> None:
    width = 0.16
    depth = 0.15
    height = 0.13
    rod = 0.008
    half_w = width * 0.5
    half_d = depth * 0.5
    part.visual(Box((width, depth, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=material, name="perforated_floor")
    part.visual(Box((width, rod, height)), origin=Origin(xyz=(0.0, -half_d, height * 0.5)), material=material, name="front_wall")
    part.visual(Box((width, rod, height)), origin=Origin(xyz=(0.0, half_d, height * 0.5)), material=material, name="rear_wall")
    part.visual(Box((rod, depth, height)), origin=Origin(xyz=(-half_w, 0.0, height * 0.5)), material=material, name="side_wall_0")
    part.visual(Box((rod, depth, height)), origin=Origin(xyz=(half_w, 0.0, height * 0.5)), material=material, name="side_wall_1")
    for index, x in enumerate((-0.048, -0.016, 0.016, 0.048)):
        part.visual(Box((0.004, depth, height * 0.80)), origin=Origin(xyz=(x, 0.0, height * 0.44)), material=material, name=f"divider_{index}")
    part.visual(Box((0.10, 0.010, 0.010)), origin=Origin(xyz=(0.0, -0.066, height + 0.010)), material=material, name="front_handle_bar")
    part.visual(Box((0.010, 0.010, 0.070)), origin=Origin(xyz=(-0.050, -0.066, 0.100)), material=material, name="handle_post_0")
    part.visual(Box((0.010, 0.010, 0.070)), origin=Origin(xyz=(0.050, -0.066, 0.100)), material=material, name="handle_post_1")
    # Two hooked feet bear on the lower rack front rail while the caddy slides.
    part.visual(Box((0.010, 0.012, 0.050)), origin=Origin(xyz=(-0.058, -0.075, -0.025)), material=material, name="rail_hook_0")
    part.visual(Box((0.010, 0.012, 0.050)), origin=Origin(xyz=(0.058, -0.075, -0.025)), material=material, name="rail_hook_1")


def _add_tine_bank(part, *, material: str) -> None:
    # Local frame is on the hinge/pivot line.
    part.visual(
        Cylinder(radius=0.006, length=0.23),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="pivot_rod",
    )
    for index, x in enumerate((-0.085, -0.050, -0.015, 0.020, 0.055, 0.090)):
        part.visual(
            Box((0.006, 0.006, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.060)),
            material=material,
            name=f"folding_tine_{index}",
        )
    part.visual(Box((0.22, 0.006, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.120)), material=material, name="top_tie_rod")


def _add_spray_arm(part, *, span: float, material: str, accent: str) -> None:
    part.visual(Cylinder(radius=0.034, length=0.024), origin=Origin(), material=material, name="central_hub")
    part.visual(Box((span, 0.055, 0.014)), origin=Origin(), material=material, name="spray_blade")
    part.visual(Box((0.055, span * 0.62, 0.012)), origin=Origin(), material=material, name="short_blade")
    for index, x in enumerate((-span * 0.34, -span * 0.18, span * 0.18, span * 0.34)):
        part.visual(
            Cylinder(radius=0.006, length=0.005),
            origin=Origin(xyz=(x, 0.018 if index % 2 else -0.018, 0.009)),
            material=accent,
            name=f"spray_nozzle_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_dishwasher")

    model.material("white_enamel", rgba=(0.86, 0.87, 0.84, 1.0))
    model.material("stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    model.material("dark_plastic", rgba=(0.04, 0.045, 0.05, 1.0))
    model.material("black_glass", rgba=(0.02, 0.025, 0.03, 1.0))
    model.material("rack_coating", rgba=(0.91, 0.93, 0.92, 1.0))
    model.material("caddy_gray", rgba=(0.42, 0.45, 0.47, 1.0))
    model.material("rubber_button", rgba=(0.12, 0.13, 0.14, 1.0))
    model.material("nozzle_black", rgba=(0.01, 0.01, 0.012, 1.0))

    cabinet = model.part("cabinet")
    # Open-front hollow wash chamber: side walls, floor, ceiling, back panel and
    # a raised front flange leave clear, visible interior space for the racks.
    cabinet.visual(Box((0.035, DEPTH, 0.74)), origin=Origin(xyz=(-0.3125, 0.0, 0.49)), material="white_enamel", name="side_wall_0")
    cabinet.visual(Box((0.035, DEPTH, 0.74)), origin=Origin(xyz=(0.3125, 0.0, 0.49)), material="white_enamel", name="side_wall_1")
    cabinet.visual(Box((WIDTH, 0.035, 0.74)), origin=Origin(xyz=(0.0, 0.3025, 0.49)), material="stainless", name="back_wall")
    cabinet.visual(Box((WIDTH, DEPTH, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.1125)), material="stainless", name="tub_floor")
    cabinet.visual(Box((WIDTH, DEPTH, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.8375)), material="white_enamel", name="top_panel")
    cabinet.visual(Box((0.040, 0.035, 0.70)), origin=Origin(xyz=(-0.312, -0.332, 0.47)), material="white_enamel", name="front_jamb_0")
    cabinet.visual(Box((0.040, 0.035, 0.70)), origin=Origin(xyz=(0.312, -0.332, 0.47)), material="white_enamel", name="front_jamb_1")
    cabinet.visual(Box((WIDTH, 0.035, 0.060)), origin=Origin(xyz=(0.0, -0.332, 0.800)), material="white_enamel", name="front_lintel")
    cabinet.visual(Box((WIDTH, 0.035, 0.055)), origin=Origin(xyz=(0.0, -0.332, 0.075)), material="white_enamel", name="front_sill")
    cabinet.visual(Box((0.58, 0.050, 0.080)), origin=Origin(xyz=(0.0, -0.372, 0.040)), material="dark_plastic", name="toe_kick")
    cabinet.visual(
        Cylinder(radius=0.012, length=0.600),
        origin=Origin(xyz=(0.0, -0.400, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="stainless",
        name="hinge_pin",
    )
    for index, x in enumerate((-0.292, 0.292)):
        cabinet.visual(
            Box((0.025, 0.080, 0.026)),
            origin=Origin(xyz=(x, -0.365, 0.098)),
            material="stainless",
            name=f"hinge_bracket_{index}",
        )

    # Rack slide rails mounted to the side walls.
    for zi, z in enumerate((0.275, 0.595)):
        cabinet.visual(Box((0.020, 0.48, 0.020)), origin=Origin(xyz=(-0.285, 0.005, z)), material="stainless", name=f"rack_slide_0_{zi}")
        cabinet.visual(Box((0.020, 0.48, 0.020)), origin=Origin(xyz=(0.285, 0.005, z)), material="stainless", name=f"rack_slide_1_{zi}")

    # Central supports for the two rotating spray arms.
    cabinet.visual(Cylinder(radius=0.025, length=0.055), origin=Origin(xyz=(0.0, 0.0, 0.1625)), material="stainless", name="lower_spray_stand")
    cabinet.visual(Box((0.040, 0.290, 0.035)), origin=Origin(xyz=(0.0, 0.145, 0.485)), material="stainless", name="upper_spray_feed")
    cabinet.visual(Cylinder(radius=0.018, length=0.046), origin=Origin(xyz=(0.0, 0.0, 0.475)), material="stainless", name="upper_spray_drop")

    door = model.part("door")
    door.visual(Box((0.600, 0.690, 0.050)), origin=Origin(xyz=(0.0, -0.361, 0.025)), material="white_enamel", name="broad_panel")
    door.visual(Box((0.520, 0.540, 0.006)), origin=Origin(xyz=(0.0, -0.370, 0.054)), material="stainless", name="inner_liner")
    door.visual(Box((0.560, 0.075, 0.060)), origin=Origin(xyz=(0.0, -0.665, 0.040)), material="black_glass", name="top_control_band")
    door.visual(Box((0.420, 0.030, 0.018)), origin=Origin(xyz=(0.0, -0.150, -0.009)), material="stainless", name="front_handle")
    door.visual(
        Cylinder(radius=0.018, length=0.540),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material="stainless",
        name="hinge_knuckle",
    )
    model.articulation(
        "lower_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -0.400, 0.110)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=0.0, upper=1.50),
    )

    # Top-edge control band: five push buttons arranged around a central selector knob.
    button_xs = (-0.225, -0.145, -0.070, 0.105, 0.185)
    for index, x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(Box((0.052, 0.018, 0.026)), origin=Origin(xyz=(0.0, -0.009, 0.0)), material="rubber_button", name="button_cap")
        model.articulation(
            f"door_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.7025, 0.045)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.06, lower=0.0, upper=0.006),
        )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.032, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_plastic",
        name="knob_cap",
    )
    selector_knob.visual(Box((0.006, 0.006, 0.040)), origin=Origin(xyz=(0.0, -0.026, 0.0)), material="stainless", name="pointer_mark")
    model.articulation(
        "door_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=selector_knob,
        origin=Origin(xyz=(0.0, -0.7025, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    upper_rack = model.part("upper_rack")
    _add_rack(upper_rack, width=0.538, depth=0.49, height=0.13, rod=0.012, material="rack_coating")
    model.articulation(
        "upper_rack_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.005, 0.560)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.35, lower=0.0, upper=0.42),
    )

    lower_rack = model.part("lower_rack")
    _add_rack(lower_rack, width=0.538, depth=0.51, height=0.16, rod=0.012, material="rack_coating")
    for index, x in enumerate((-0.125, 0.125)):
        lower_rack.visual(
            Box((0.240, 0.010, 0.0325)),
            origin=Origin(xyz=(x, 0.055, 0.01625)),
            material="rack_coating",
            name=f"tine_pivot_saddle_{index}",
        )
    model.articulation(
        "lower_rack_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.005, 0.235)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.47),
    )

    cutlery_caddy = model.part("cutlery_caddy")
    _add_cutlery_caddy(cutlery_caddy, material="caddy_gray")
    model.articulation(
        "caddy_slide",
        ArticulationType.PRISMATIC,
        parent=lower_rack,
        child=cutlery_caddy,
        origin=Origin(xyz=(-0.105, -0.168, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=-0.060, upper=0.230),
    )

    for index, x in enumerate((-0.125, 0.125)):
        tine_bank = model.part(f"tine_bank_{index}")
        _add_tine_bank(tine_bank, material="rack_coating")
        model.articulation(
            f"lower_rack_to_tine_bank_{index}",
            ArticulationType.REVOLUTE,
            parent=lower_rack,
            child=tine_bank,
            origin=Origin(xyz=(x, 0.055, 0.0385)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=1.4, lower=0.0, upper=1.45),
        )

    lower_spray_arm = model.part("lower_spray_arm")
    _add_spray_arm(lower_spray_arm, span=0.46, material="stainless", accent="nozzle_black")
    model.articulation(
        "lower_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=lower_spray_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.202)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    upper_spray_arm = model.part("upper_spray_arm")
    _add_spray_arm(upper_spray_arm, span=0.38, material="stainless", accent="nozzle_black")
    model.articulation(
        "upper_spray_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=upper_spray_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.440)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    upper_rack = object_model.get_part("upper_rack")
    lower_rack = object_model.get_part("lower_rack")
    cutlery_caddy = object_model.get_part("cutlery_caddy")

    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        reason="The stainless hinge pin is intentionally captured inside the door hinge knuckle.",
    )
    ctx.expect_within(
        cabinet,
        door,
        axes="yz",
        inner_elem="hinge_pin",
        outer_elem="hinge_knuckle",
        margin=0.001,
        name="hinge pin is captured in door knuckle",
    )
    ctx.expect_overlap(
        cabinet,
        door,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_knuckle",
        min_overlap=0.50,
        name="hinge pin spans the door knuckle",
    )

    ctx.check("five separate push buttons", all(object_model.get_part(f"button_{i}") is not None for i in range(5)))
    ctx.check(
        "all named mechanisms exist",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "lower_hinge",
                "upper_rack_slide",
                "lower_rack_slide",
                "caddy_slide",
                "lower_spray_spin",
                "upper_spray_spin",
                "door_to_selector_knob",
            )
        ),
    )

    # The racks sit in the hollow chamber without filling the vertical space
    # between them.
    ctx.expect_within(upper_rack, cabinet, axes="x", margin=0.010, name="upper rack fits between side walls")
    ctx.expect_within(lower_rack, cabinet, axes="x", margin=0.010, name="lower rack fits between side walls")
    ctx.expect_gap(upper_rack, lower_rack, axis="z", min_gap=0.10, name="open wash space between racks")
    ctx.expect_gap(lower_rack, cabinet, axis="z", positive_elem="front_top_rail", negative_elem="tub_floor", min_gap=0.04, name="lower rack clears tub floor")

    lower_hinge = object_model.get_articulation("lower_hinge")
    lower_rack_slide = object_model.get_articulation("lower_rack_slide")
    upper_rack_slide = object_model.get_articulation("upper_rack_slide")
    caddy_slide = object_model.get_articulation("caddy_slide")
    tine_0 = object_model.get_articulation("lower_rack_to_tine_bank_0")

    door_rest_aabb = ctx.part_element_world_aabb(door, elem="top_control_band")
    with ctx.pose({lower_hinge: 1.45}):
        door_raised_aabb = ctx.part_element_world_aabb(door, elem="top_control_band")
    ctx.check(
        "door rotates upward from lower hinge",
        door_rest_aabb is not None
        and door_raised_aabb is not None
        and door_raised_aabb[1][2] > door_rest_aabb[1][2] + 0.45,
        details=f"rest={door_rest_aabb}, raised={door_raised_aabb}",
    )

    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_rack_slide: 0.35, upper_rack_slide: 0.28}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_within(upper_rack, cabinet, axes="x", margin=0.010, name="extended upper rack stays on side rails")
        ctx.expect_within(lower_rack, cabinet, axes="x", margin=0.010, name="extended lower rack stays on side rails")
    ctx.check(
        "racks slide outward",
        lower_rest is not None and lower_extended is not None and lower_extended[1] < lower_rest[1] - 0.25,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    caddy_rest = ctx.part_world_position(cutlery_caddy)
    with ctx.pose({caddy_slide: 0.18}):
        caddy_shifted = ctx.part_world_position(cutlery_caddy)
    ctx.check(
        "cutlery caddy slides along front rail",
        caddy_rest is not None and caddy_shifted is not None and caddy_shifted[0] > caddy_rest[0] + 0.15,
        details=f"rest={caddy_rest}, shifted={caddy_shifted}",
    )

    tine_bank = object_model.get_part("tine_bank_0")
    tine_rest = ctx.part_world_position(tine_bank)
    with ctx.pose({tine_0: 1.20}):
        tine_folded = ctx.part_world_position(tine_bank)
    ctx.check(
        "fold-down tine bank rotates on lower rack pivot",
        tine_rest is not None and tine_folded is not None,
        details=f"rest={tine_rest}, folded={tine_folded}",
    )

    return ctx.report()


object_model = build_object_model()
