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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_transfer_nose")

    wall = Material("matte_wall", rgba=(0.72, 0.72, 0.68, 1.0))
    plate_paint = Material("dark_powder_coat", rgba=(0.06, 0.07, 0.075, 1.0))
    hinge_steel = Material("blackened_hinge_steel", rgba=(0.015, 0.016, 0.018, 1.0))
    carrier_paint = Material("blue_carrier_paint", rgba=(0.02, 0.14, 0.42, 1.0))
    slide_steel = Material("brushed_slide_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    tab_finish = Material("safety_orange_tab", rgba=(0.95, 0.38, 0.04, 1.0))

    backplate = model.part("backplate")
    # The root frame is on the hinge centerline, just in front of the wall plate.
    backplate.visual(
        Box((0.012, 0.46, 0.52)),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=wall,
        name="wall_backer",
    )
    backplate.visual(
        Box((0.040, 0.36, 0.42)),
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
        material=plate_paint,
        name="backplate_slab",
    )
    for i, (y, z) in enumerate(((-0.135, -0.155), (0.135, -0.155), (-0.135, 0.155), (0.135, 0.155))):
        backplate.visual(
            Cylinder(radius=0.018, length=0.009),
            origin=Origin(xyz=(0.0035, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"mount_bolt_{i}",
        )
    for z in (-0.065, 0.065):
        backplate.visual(
            Box((0.024, 0.086, 0.070)),
            origin=Origin(xyz=(0.010, 0.0, z)),
            material=plate_paint,
            name=f"hinge_leaf_{'lower' if z < 0 else 'upper'}",
        )
        backplate.visual(
            Cylinder(radius=0.024, length=0.070),
            origin=Origin(xyz=(0.025, 0.0, z)),
            material=hinge_steel,
            name=f"fixed_barrel_{'lower' if z < 0 else 'upper'}",
        )

    carrier = model.part("carrier")
    carrier.visual(
        Cylinder(radius=0.021, length=0.060),
        origin=Origin(),
        material=hinge_steel,
        name="swing_barrel",
    )
    carrier.visual(
        Box((0.130, 0.082, 0.064)),
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        material=carrier_paint,
        name="hinge_yoke",
    )
    for y, name in ((-0.049, "root_web_0"), (0.049, "root_web_1")):
        carrier.visual(
            Box((0.044, 0.018, 0.060)),
            origin=Origin(xyz=(0.122, y, 0.0)),
            material=carrier_paint,
            name=name,
        )
    carrier.visual(
        Box((0.500, 0.135, 0.016)),
        origin=Origin(xyz=(0.370, 0.0, 0.058)),
        material=carrier_paint,
        name="upper_rail",
    )
    carrier.visual(
        Box((0.500, 0.135, 0.016)),
        origin=Origin(xyz=(0.370, 0.0, -0.058)),
        material=carrier_paint,
        name="lower_rail",
    )
    for y, name in ((-0.063, "side_rail_0"), (0.063, "side_rail_1")):
        carrier.visual(
            Box((0.500, 0.014, 0.102)),
            origin=Origin(xyz=(0.370, y, 0.0)),
            material=carrier_paint,
            name=name,
        )
    carrier.visual(
        Box((0.030, 0.148, 0.016)),
        origin=Origin(xyz=(0.620, 0.0, 0.058)),
        material=carrier_paint,
        name="front_upper_bridge",
    )
    carrier.visual(
        Box((0.030, 0.148, 0.016)),
        origin=Origin(xyz=(0.620, 0.0, -0.058)),
        material=carrier_paint,
        name="front_lower_bridge",
    )
    for y, name in ((-0.063, "front_side_0"), (0.063, "front_side_1")):
        carrier.visual(
            Box((0.030, 0.014, 0.102)),
            origin=Origin(xyz=(0.620, y, 0.0)),
            material=carrier_paint,
            name=name,
        )

    slider = model.part("slider")
    slider.visual(
        Box((0.420, 0.050, 0.032)),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=slide_steel,
        name="main_bar",
    )
    slider.visual(
        Box((0.060, 0.070, 0.060)),
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        material=slide_steel,
        name="nose_block",
    )
    for z, name in ((0.035, "upper_clevis"), (-0.035, "lower_clevis")):
        slider.visual(
            Box((0.080, 0.078, 0.012)),
            origin=Origin(xyz=(0.365, 0.0, z)),
            material=slide_steel,
            name=name,
        )
    for z, name in ((0.046, "upper_pin_cap"), (-0.046, "lower_pin_cap")):
        slider.visual(
            Cylinder(radius=0.023, length=0.010),
            origin=Origin(xyz=(0.380, 0.0, z)),
            material=hinge_steel,
            name=name,
        )

    tab = model.part("tab")
    tab.visual(
        Cylinder(radius=0.024, length=0.058),
        origin=Origin(),
        material=hinge_steel,
        name="pivot_boss",
    )
    tab.visual(
        Box((0.140, 0.036, 0.010)),
        origin=Origin(xyz=(0.070, 0.0, 0.0)),
        material=tab_finish,
        name="tool_tongue",
    )
    tab.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=tab_finish,
        name="rounded_tip",
    )

    model.articulation(
        "carrier_hinge",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=carrier,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.95, upper=0.95),
    )
    model.articulation(
        "slider_travel",
        ArticulationType.PRISMATIC,
        parent=carrier,
        child=slider,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.20),
    )
    model.articulation(
        "tab_pivot",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=tab,
        origin=Origin(xyz=(0.380, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carrier = object_model.get_part("carrier")
    slider = object_model.get_part("slider")
    tab = object_model.get_part("tab")
    carrier_hinge = object_model.get_articulation("carrier_hinge")
    slider_travel = object_model.get_articulation("slider_travel")
    tab_pivot = object_model.get_articulation("tab_pivot")

    ctx.check(
        "three primary mechanisms are articulated",
        carrier_hinge.articulation_type == ArticulationType.REVOLUTE
        and slider_travel.articulation_type == ArticulationType.PRISMATIC
        and tab_pivot.articulation_type == ArticulationType.REVOLUTE,
        details="Expected carrier revolute, slider prismatic, and tab revolute joints.",
    )
    ctx.expect_within(
        slider,
        carrier,
        axes="yz",
        inner_elem="main_bar",
        margin=0.003,
        name="sliding bar is captured inside carrier channel",
    )
    ctx.expect_overlap(
        slider,
        carrier,
        axes="x",
        elem_a="main_bar",
        min_overlap=0.25,
        name="collapsed slider remains retained in carrier",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({slider_travel: 0.20}):
        ctx.expect_overlap(
            slider,
            carrier,
            axes="x",
            elem_a="main_bar",
            min_overlap=0.13,
            name="extended slider keeps retained insertion",
        )
        extended_slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "prismatic member translates out of the carrier",
        rest_slider_pos is not None
        and extended_slider_pos is not None
        and extended_slider_pos[0] > rest_slider_pos[0] + 0.18,
        details=f"rest={rest_slider_pos}, extended={extended_slider_pos}",
    )

    rest_slider_pos = ctx.part_world_position(slider)
    with ctx.pose({carrier_hinge: 0.60}):
        swung_slider_pos = ctx.part_world_position(slider)
    ctx.check(
        "carrier hinge swings the nose sideways",
        rest_slider_pos is not None
        and swung_slider_pos is not None
        and swung_slider_pos[1] > rest_slider_pos[1] + 0.10,
        details=f"rest={rest_slider_pos}, swung={swung_slider_pos}",
    )

    rest_tip_aabb = ctx.part_element_world_aabb(tab, elem="rounded_tip")
    with ctx.pose({tab_pivot: 0.65}):
        turned_tip_aabb = ctx.part_element_world_aabb(tab, elem="rounded_tip")
    if rest_tip_aabb is not None and turned_tip_aabb is not None:
        rest_tip_y = 0.5 * (rest_tip_aabb[0][1] + rest_tip_aabb[1][1])
        turned_tip_y = 0.5 * (turned_tip_aabb[0][1] + turned_tip_aabb[1][1])
    else:
        rest_tip_y = turned_tip_y = None
    ctx.check(
        "tip tab pivots around its vertical pin",
        rest_tip_y is not None and turned_tip_y is not None and turned_tip_y > rest_tip_y + 0.06,
        details=f"rest_tip_y={rest_tip_y}, turned_tip_y={turned_tip_y}",
    )

    return ctx.report()


object_model = build_object_model()
