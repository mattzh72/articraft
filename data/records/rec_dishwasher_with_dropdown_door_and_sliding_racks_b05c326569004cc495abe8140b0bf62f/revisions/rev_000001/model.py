from __future__ import annotations

from math import pi

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_undercounter_dishwasher")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.17, 0.18, 0.19, 1.0))
    rack_metal = model.material("rack_metal", rgba=(0.86, 0.88, 0.90, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.12, 0.13, 1.0))
    control_gray = model.material("control_gray", rgba=(0.42, 0.44, 0.46, 1.0))
    button_blue = model.material("button_blue", rgba=(0.19, 0.48, 0.78, 1.0))
    button_amber = model.material("button_amber", rgba=(0.88, 0.63, 0.20, 1.0))
    detergent_blue = model.material("detergent_blue", rgba=(0.44, 0.59, 0.84, 1.0))

    width = 0.62
    depth = 0.68
    height = 0.84
    wall = 0.018
    base_h = 0.06
    top_t = 0.018

    body = model.part("body")
    body.visual(
        Box((depth, wall, height - base_h)),
        origin=Origin(xyz=(-depth / 2, width / 2 - wall / 2, base_h + (height - base_h) / 2)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((depth, wall, height - base_h)),
        origin=Origin(xyz=(-depth / 2, -width / 2 + wall / 2, base_h + (height - base_h) / 2)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((wall, width, height - base_h)),
        origin=Origin(xyz=(-depth + wall / 2, 0.0, base_h + (height - base_h) / 2)),
        material=stainless,
        name="rear_wall",
    )
    body.visual(
        Box((depth, width, top_t)),
        origin=Origin(xyz=(-depth / 2, 0.0, height - top_t / 2)),
        material=stainless,
        name="top_panel",
    )
    body.visual(
        Box((depth, width, wall)),
        origin=Origin(xyz=(-depth / 2, 0.0, base_h + wall / 2)),
        material=dark_trim,
        name="bottom_pan",
    )
    body.visual(
        Box((0.56, 0.026, 0.018)),
        origin=Origin(xyz=(-0.34, width / 2 - wall - 0.011, 0.26)),
        material=control_gray,
        name="guide_0",
    )
    body.visual(
        Box((0.56, 0.026, 0.018)),
        origin=Origin(xyz=(-0.34, -width / 2 + wall + 0.011, 0.26)),
        material=control_gray,
        name="guide_1",
    )
    body.visual(
        Cylinder(radius=0.092, length=0.052),
        origin=Origin(xyz=(-0.34, 0.0, base_h + wall + 0.026)),
        material=control_gray,
        name="sump",
    )
    body.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(-0.34, 0.0, base_h + wall + 0.061)),
        material=dark_trim,
        name="sump_cap",
    )

    door = model.part("door")
    door_w = width - 0.04
    door_h = height - base_h - 0.024
    shell_t = 0.048
    door.visual(
        Box((0.004, door_w, door_h)),
        origin=Origin(xyz=(0.037, 0.0, door_h / 2)),
        material=stainless,
        name="outer_skin",
    )
    door.visual(
        Box((0.004, door_w - 0.06, door_h - 0.030)),
        origin=Origin(xyz=(-0.010, 0.0, door_h / 2 + 0.001)),
        material=stainless,
        name="inner_panel",
    )
    door.visual(
        Box((0.034, 0.030, door_h)),
        origin=Origin(xyz=(0.017, door_w / 2 - 0.015, door_h / 2)),
        material=stainless,
        name="side_hem_0",
    )
    door.visual(
        Box((0.034, 0.030, door_h)),
        origin=Origin(xyz=(0.017, -door_w / 2 + 0.015, door_h / 2)),
        material=stainless,
        name="side_hem_1",
    )
    door.visual(
        Box((shell_t, door_w, 0.034)),
        origin=Origin(xyz=(0.013, 0.0, door_h - 0.017)),
        material=stainless,
        name="top_hem",
    )
    door.visual(
        Box((0.032, door_w, 0.030)),
        origin=Origin(xyz=(0.016, 0.0, 0.015)),
        material=stainless,
        name="bottom_hem",
    )
    door.visual(
        Box((0.012, door_w * 0.92, 0.115)),
        origin=Origin(xyz=(0.031, 0.0, door_h - 0.058)),
        material=control_black,
        name="control_panel",
    )
    door.visual(
        Box((0.012, 0.180, 0.155)),
        origin=Origin(xyz=(-0.008, 0.105, 0.365)),
        material=control_gray,
        name="detergent_housing",
    )

    rack = model.part("rack")
    rack_w = 0.48
    rack_d = 0.52
    rack_h = 0.19
    bar = 0.014
    front_clearance = 0.04
    rack.visual(
        Box((bar, rack_w, bar)),
        origin=Origin(xyz=(-front_clearance - bar / 2, 0.0, bar / 2)),
        material=rack_metal,
        name="front_bar",
    )
    rack.visual(
        Box((bar, rack_w, bar)),
        origin=Origin(xyz=(-front_clearance - rack_d + bar / 2, 0.0, bar / 2)),
        material=rack_metal,
        name="rear_bar",
    )
    rack.visual(
        Box((rack_d, bar, bar)),
        origin=Origin(xyz=(-front_clearance - rack_d / 2, rack_w / 2 - bar / 2, bar / 2)),
        material=rack_metal,
        name="right_lower_rail",
    )
    rack.visual(
        Box((rack_d, bar, bar)),
        origin=Origin(xyz=(-front_clearance - rack_d / 2, -rack_w / 2 + bar / 2, bar / 2)),
        material=rack_metal,
        name="left_lower_rail",
    )
    rack.visual(
        Box((bar, rack_w, bar)),
        origin=Origin(xyz=(-front_clearance - bar / 2, 0.0, rack_h - bar / 2)),
        material=rack_metal,
        name="front_top_bar",
    )
    rack.visual(
        Box((bar, rack_w, bar)),
        origin=Origin(xyz=(-front_clearance - rack_d + bar / 2, 0.0, rack_h - bar / 2)),
        material=rack_metal,
        name="rear_top_bar",
    )
    rack.visual(
        Box((rack_d, bar, bar)),
        origin=Origin(xyz=(-front_clearance - rack_d / 2, rack_w / 2 - bar / 2, rack_h - bar / 2)),
        material=rack_metal,
        name="right_upper_rail",
    )
    rack.visual(
        Box((rack_d, bar, bar)),
        origin=Origin(xyz=(-front_clearance - rack_d / 2, -rack_w / 2 + bar / 2, rack_h - bar / 2)),
        material=rack_metal,
        name="left_upper_rail",
    )
    rack.visual(
        Box((rack_d, bar, rack_h - bar)),
        origin=Origin(xyz=(-front_clearance - rack_d / 2, rack_w / 2 - bar / 2, rack_h / 2)),
        material=rack_metal,
        name="right_side_runner",
    )
    rack.visual(
        Box((rack_d, bar, rack_h - bar)),
        origin=Origin(xyz=(-front_clearance - rack_d / 2, -rack_w / 2 + bar / 2, rack_h / 2)),
        material=rack_metal,
        name="left_side_runner",
    )
    for ix, x in enumerate((-front_clearance - bar / 2, -front_clearance - rack_d + bar / 2)):
        for iy, y in enumerate((rack_w / 2 - bar / 2, -rack_w / 2 + bar / 2)):
            rack.visual(
                Box((bar, bar, rack_h)),
                origin=Origin(xyz=(x, y, rack_h / 2)),
                material=rack_metal,
                name=f"post_{ix}_{iy}",
            )
    for idx, y in enumerate((-0.12, 0.0, 0.12)):
        rack.visual(
            Box((rack_d - 2 * bar, bar * 0.75, bar * 0.75)),
            origin=Origin(xyz=(-front_clearance - rack_d / 2, y, bar * 0.375)),
            material=rack_metal,
            name=f"floor_slats_{idx}",
        )
    for idx, x in enumerate((-0.18, -0.30, -0.42)):
        rack.visual(
            Box((bar * 0.75, rack_w - 2 * bar, bar * 0.75)),
            origin=Origin(xyz=(x, 0.0, bar * 0.375)),
            material=rack_metal,
            name=f"cross_slats_{idx}",
        )
    rack.visual(
        Box((0.48, 0.050, 0.014)),
        origin=Origin(xyz=(-front_clearance - 0.26, 0.252, 0.096)),
        material=rack_metal,
        name="runner_0",
    )
    rack.visual(
        Box((0.48, 0.050, 0.014)),
        origin=Origin(xyz=(-front_clearance - 0.26, -0.252, 0.096)),
        material=rack_metal,
        name="runner_1",
    )

    detergent_flap = model.part("detergent_flap")
    detergent_flap.visual(
        Box((0.008, 0.145, 0.112)),
        origin=Origin(xyz=(-0.004, 0.0, -0.056)),
        material=detergent_blue,
        name="flap_panel",
    )
    detergent_flap.visual(
        Box((0.016, 0.022, 0.010)),
        origin=Origin(xyz=(-0.008, 0.0, -0.106)),
        material=control_gray,
        name="pull_tab",
    )

    wash_arm = model.part("wash_arm")
    wash_arm.visual(
        Cylinder(radius=0.028, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=control_gray,
        name="hub",
    )
    wash_arm.visual(
        Box((0.040, 0.340, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=control_gray,
        name="spray_bar",
    )
    wash_arm.visual(
        Box((0.055, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, 0.150, 0.010)),
        material=control_gray,
        name="tip_0",
    )
    wash_arm.visual(
        Box((0.055, 0.050, 0.010)),
        origin=Origin(xyz=(0.0, -0.150, 0.010)),
        material=control_gray,
        name="tip_1",
    )

    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(
        Cylinder(radius=0.046, length=0.026),
        origin=Origin(xyz=(0.013, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=control_black,
        name="knob_body",
    )
    cycle_knob.visual(
        Box((0.012, 0.010, 0.034)),
        origin=Origin(xyz=(0.027, 0.0, 0.0)),
        material=control_gray,
        name="indicator_ridge",
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.007, 0.0, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=button_blue,
        name="button_cap",
    )

    indicator_button_0 = model.part("indicator_button_0")
    indicator_button_0.visual(
        Box((0.012, 0.022, 0.022)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=button_amber,
        name="button_cap",
    )

    indicator_button_1 = model.part("indicator_button_1")
    indicator_button_1.visual(
        Box((0.012, 0.022, 0.022)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=button_blue,
        name="button_cap",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, 0.0, base_h)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "rack_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=rack,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.35, lower=0.0, upper=0.26),
    )
    model.articulation(
        "detergent_hinge",
        ArticulationType.REVOLUTE,
        parent=door,
        child=detergent_flap,
        origin=Origin(xyz=(-0.013, 0.105, 0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.70),
    )
    model.articulation(
        "wash_arm_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=wash_arm,
        origin=Origin(xyz=(-0.34, 0.0, base_h + wall + 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "cycle_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cycle_knob,
        origin=Origin(xyz=(0.037, -0.170, door_h - 0.058)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "start_button_press",
        ArticulationType.PRISMATIC,
        parent=door,
        child=start_button,
        origin=Origin(xyz=(0.037, 0.045, door_h - 0.060)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.06, lower=0.0, upper=0.010),
    )
    model.articulation(
        "indicator_button_0_press",
        ArticulationType.PRISMATIC,
        parent=door,
        child=indicator_button_0,
        origin=Origin(xyz=(0.037, 0.150, door_h - 0.058)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.008),
    )
    model.articulation(
        "indicator_button_1_press",
        ArticulationType.PRISMATIC,
        parent=door,
        child=indicator_button_1,
        origin=Origin(xyz=(0.037, 0.190, door_h - 0.058)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.05, lower=0.0, upper=0.008),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rack = object_model.get_part("rack")
    detergent_flap = object_model.get_part("detergent_flap")
    start_button = object_model.get_part("start_button")
    indicator_button_0 = object_model.get_part("indicator_button_0")
    indicator_button_1 = object_model.get_part("indicator_button_1")

    door_hinge = object_model.get_articulation("door_hinge")
    rack_slide = object_model.get_articulation("rack_slide")
    detergent_hinge = object_model.get_articulation("detergent_hinge")
    start_button_press = object_model.get_articulation("start_button_press")
    indicator_button_0_press = object_model.get_articulation("indicator_button_0_press")
    indicator_button_1_press = object_model.get_articulation("indicator_button_1_press")

    door_upper = door_hinge.motion_limits.upper if door_hinge.motion_limits else None
    rack_upper = rack_slide.motion_limits.upper if rack_slide.motion_limits else None
    detergent_upper = detergent_hinge.motion_limits.upper if detergent_hinge.motion_limits else None
    start_upper = start_button_press.motion_limits.upper if start_button_press.motion_limits else None
    ind0_upper = indicator_button_0_press.motion_limits.upper if indicator_button_0_press.motion_limits else None
    ind1_upper = indicator_button_1_press.motion_limits.upper if indicator_button_1_press.motion_limits else None

    with ctx.pose({door_hinge: 0.0, rack_slide: 0.0, detergent_hinge: 0.0}):
        ctx.expect_overlap(
            door,
            body,
            axes="yz",
            min_overlap=0.56,
            name="closed door covers the chamber opening",
        )
        ctx.expect_within(
            rack,
            body,
            axes="y",
            margin=0.03,
            name="rack sits between the body side walls",
        )
        ctx.expect_overlap(
            rack,
            body,
            axes="x",
            min_overlap=0.48,
            name="rack starts deep inside the wash chamber",
        )

    if rack_upper is not None:
        rest_pos = ctx.part_world_position(rack)
        with ctx.pose({rack_slide: rack_upper}):
            extended_pos = ctx.part_world_position(rack)
            ctx.expect_within(
                rack,
                body,
                axes="y",
                margin=0.03,
                name="extended rack stays centered on the guides",
            )
            ctx.expect_overlap(
                rack,
                body,
                axes="x",
                min_overlap=0.22,
                name="extended rack retains insertion into the chamber",
            )
        ctx.check(
            "rack slides outward for loading",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[0] > rest_pos[0] + 0.20,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    if door_upper is not None:
        with ctx.pose({door_hinge: door_upper}):
            door_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door drops into a horizontal loading shelf",
            door_aabb is not None and door_aabb[1][0] > 0.70 and door_aabb[1][2] < 0.20,
            details=f"door_aabb={door_aabb}",
        )

    if detergent_upper is not None:
        with ctx.pose({detergent_hinge: 0.0}):
            flap_closed = ctx.part_world_aabb(detergent_flap)
        with ctx.pose({detergent_hinge: detergent_upper}):
            flap_open = ctx.part_world_aabb(detergent_flap)
        ctx.check(
            "detergent flap opens into the wash chamber",
            flap_closed is not None
            and flap_open is not None
            and flap_open[0][0] < flap_closed[0][0] - 0.04,
            details=f"closed={flap_closed}, open={flap_open}",
        )

    if start_upper is not None and ind0_upper is not None and ind1_upper is not None:
        start_rest = ctx.part_world_position(start_button)
        ind0_rest = ctx.part_world_position(indicator_button_0)
        ind1_rest = ctx.part_world_position(indicator_button_1)
        with ctx.pose(
            {
                start_button_press: start_upper,
                indicator_button_0_press: ind0_upper,
                indicator_button_1_press: ind1_upper,
            }
        ):
            start_pressed = ctx.part_world_position(start_button)
            ind0_pressed = ctx.part_world_position(indicator_button_0)
            ind1_pressed = ctx.part_world_position(indicator_button_1)
        ctx.check(
            "front panel buttons depress inward independently",
            start_rest is not None
            and start_pressed is not None
            and ind0_rest is not None
            and ind0_pressed is not None
            and ind1_rest is not None
            and ind1_pressed is not None
            and start_pressed[0] < start_rest[0] - 0.006
            and ind0_pressed[0] < ind0_rest[0] - 0.004
            and ind1_pressed[0] < ind1_rest[0] - 0.004,
            details=(
                f"start_rest={start_rest}, start_pressed={start_pressed}, "
                f"ind0_rest={ind0_rest}, ind0_pressed={ind0_pressed}, "
                f"ind1_rest={ind1_rest}, ind1_pressed={ind1_pressed}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
