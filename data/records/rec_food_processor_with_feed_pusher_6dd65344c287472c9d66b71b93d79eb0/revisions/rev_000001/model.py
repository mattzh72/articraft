from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_W = 0.315
BODY_D = 0.220
COUPLING_X = -0.050
COUPLING_Y = 0.020
COUPLING_Z = 0.112
DISC_BASE_Z = 0.144
FEED_X = 0.040
FEED_Y = 0.000
FEED_TOP_Z = 0.191


def _body_shell() -> cq.Workplane:
    lower = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, 0.058, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
    )
    upper = (
        cq.Workplane("XY")
        .box(0.238, 0.172, 0.042, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.012, 0.058))
    )
    collar = (
        cq.Workplane("XY")
        .circle(0.036)
        .extrude(0.018)
        .translate((COUPLING_X, COUPLING_Y, 0.086))
    )
    toe = (
        cq.Workplane("XY")
        .box(0.230, 0.045, 0.020, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.008, -0.065, 0.012))
    )
    return lower.union(upper).union(collar).union(toe)


def _bowl_shell() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .circle(0.050)
        .workplane(offset=0.026)
        .circle(0.106)
        .workplane(offset=0.040)
        .circle(0.118)
        .workplane(offset=0.030)
        .circle(0.120)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .circle(0.040)
        .workplane(offset=0.010)
        .circle(0.076)
        .workplane(offset=0.020)
        .circle(0.101)
        .workplane(offset=0.030)
        .circle(0.110)
        .workplane(offset=0.028)
        .circle(0.112)
        .loft(combine=True)
    )
    top_open = cq.Workplane("XY").circle(0.150).extrude(0.040).translate((0.0, 0.0, 0.092))
    skirt = (
        cq.Workplane("XY")
        .circle(0.052)
        .circle(0.036)
        .extrude(0.018)
        .translate((0.0, 0.0, -0.008))
    )
    floor_bridge = cq.Workplane("XY").circle(0.040).circle(0.012).extrude(0.008)
    spindle = cq.Workplane("XY").circle(0.020).circle(0.012).extrude(0.022)
    lid_plate = cq.Workplane("XY").circle(0.120).extrude(0.011).translate((0.0, 0.0, 0.092))
    feed_cut = cq.Workplane("XY").box(0.080, 0.052, 0.020, centered=(True, True, False)).translate(
        (FEED_X, FEED_Y, 0.092)
    )
    return outer.cut(inner).cut(top_open).union(skirt).union(floor_bridge).union(spindle).union(lid_plate).cut(feed_cut)


def _feed_tube() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(0.108, 0.078, 0.090, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((FEED_X, FEED_Y, 0.101))
    )
    inner = (
        cq.Workplane("XY")
        .box(0.078, 0.050, 0.100, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
        .translate((FEED_X, FEED_Y, 0.097))
    )
    return outer.cut(inner)


def _disc_shell() -> cq.Workplane:
    plate = cq.Workplane("XY").circle(0.086).extrude(0.004)
    hub = cq.Workplane("XY").circle(0.020).extrude(0.024)
    crown = cq.Workplane("XY").circle(0.040).circle(0.020).extrude(0.006).translate((0.0, 0.0, 0.004))
    bore = cq.Workplane("XY").circle(0.010).extrude(0.040).translate((0.0, 0.0, -0.004))
    return plate.union(hub).union(crown).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prep_food_processor")

    housing = model.material("housing", rgba=(0.84, 0.86, 0.89, 1.0))
    housing_trim = model.material("housing_trim", rgba=(0.67, 0.69, 0.72, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    bowl_clear = model.material("bowl_clear", rgba=(0.72, 0.78, 0.82, 0.34))
    pusher_white = model.material("pusher_white", rgba=(0.95, 0.96, 0.97, 1.0))
    steel = model.material("steel", rgba=(0.84, 0.86, 0.88, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_body_shell(), "food_processor_base"), material=housing, name="base_shell")
    base.visual(
        Box((0.122, 0.012, 0.056)),
        origin=Origin(xyz=(0.034, -0.104, 0.056)),
        material=housing_trim,
        name="control_panel",
    )
    base.visual(
        Cylinder(radius=0.0085, length=0.060),
        origin=Origin(xyz=(COUPLING_X, COUPLING_Y, 0.134)),
        material=steel,
        name="shaft",
    )

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(_bowl_shell(), "food_processor_bowl"), material=bowl_clear, name="bowl_shell")
    bowl.visual(mesh_from_cadquery(_feed_tube(), "food_processor_feed_tube"), material=bowl_clear, name="feed_tube")
    bowl.visual(
        Box((0.028, 0.072, 0.112)),
        origin=Origin(xyz=(0.133, 0.000, 0.072)),
        material=dark_plastic,
        name="handle",
    )

    disc = model.part("disc")
    disc.visual(mesh_from_cadquery(_disc_shell(), "food_processor_disc"), material=steel, name="disc_shell")
    disc.visual(
        Box((0.078, 0.008, 0.004)),
        origin=Origin(xyz=(0.026, 0.000, 0.006), rpy=(0.0, 0.0, math.radians(18.0))),
        material=steel,
        name="blade",
    )
    disc.visual(
        Box((0.020, 0.012, 0.004)),
        origin=Origin(xyz=(-0.038, 0.000, 0.006), rpy=(0.0, 0.0, math.radians(18.0))),
        material=steel,
        name="counterweight",
    )

    large_pusher = model.part("large_pusher")
    large_pusher.visual(
        Box((0.005, 0.049, 0.102)),
        origin=Origin(xyz=(-0.031, 0.000, -0.029)),
        material=pusher_white,
        name="wall_left",
    )
    large_pusher.visual(
        Box((0.005, 0.049, 0.102)),
        origin=Origin(xyz=(0.031, 0.000, -0.029)),
        material=pusher_white,
        name="wall_right",
    )
    large_pusher.visual(
        Box((0.061, 0.005, 0.102)),
        origin=Origin(xyz=(0.000, -0.0205, -0.029)),
        material=pusher_white,
        name="wall_front",
    )
    large_pusher.visual(
        Box((0.061, 0.005, 0.102)),
        origin=Origin(xyz=(0.000, 0.0205, -0.029)),
        material=pusher_white,
        name="wall_back",
    )
    large_pusher.visual(
        Box((0.018, 0.045, 0.016)),
        origin=Origin(xyz=(-0.0265, 0.000, 0.008)),
        material=pusher_white,
        name="cap_left",
    )
    large_pusher.visual(
        Box((0.018, 0.045, 0.016)),
        origin=Origin(xyz=(0.0265, 0.000, 0.008)),
        material=pusher_white,
        name="cap_right",
    )
    large_pusher.visual(
        Box((0.037, 0.008, 0.016)),
        origin=Origin(xyz=(0.000, -0.0215, 0.008)),
        material=pusher_white,
        name="cap_front",
    )
    large_pusher.visual(
        Box((0.037, 0.008, 0.016)),
        origin=Origin(xyz=(0.000, 0.0215, 0.008)),
        material=pusher_white,
        name="cap_back",
    )

    small_pusher = model.part("small_pusher")
    small_pusher.visual(
        Cylinder(radius=0.018, length=0.102),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=pusher_white,
        name="small_plunger",
    )
    small_pusher.visual(
        Cylinder(radius=0.0175, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=pusher_white,
        name="small_cap",
    )
    small_pusher.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=pusher_white,
        name="small_button",
    )

    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_skirt",
    )
    selector_knob.visual(
        Cylinder(radius=0.020, length=0.020),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="knob_body",
    )
    selector_knob.visual(
        Box((0.005, 0.010, 0.010)),
        origin=Origin(xyz=(0.012, -0.021, 0.0)),
        material=housing_trim,
        name="indicator",
    )

    for index, x_pos in enumerate((0.005, 0.034)):
        switch = model.part(f"switch_{index}")
        switch.visual(
            Box((0.022, 0.014, 0.028)),
            origin=Origin(xyz=(0.0, -0.007, 0.0)),
            material=dark_plastic,
            name="rocker_shell",
        )
        model.articulation(
            f"base_to_switch_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=switch,
            origin=Origin(xyz=(x_pos, -0.110, 0.056)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=4.0,
                lower=-0.24,
                upper=0.24,
            ),
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(COUPLING_X, COUPLING_Y, COUPLING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0),
    )
    model.articulation(
        "base_to_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(COUPLING_X, COUPLING_Y, DISC_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=15.0),
    )
    model.articulation(
        "bowl_to_large_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=large_pusher,
        origin=Origin(xyz=(FEED_X, FEED_Y, FEED_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.18, lower=0.0, upper=0.055),
    )
    model.articulation(
        "large_to_small_pusher",
        ArticulationType.PRISMATIC,
        parent=large_pusher,
        child=small_pusher,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.16, lower=0.0, upper=0.050),
    )
    model.articulation(
        "base_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=selector_knob,
        origin=Origin(xyz=(0.074, -0.110, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bowl = object_model.get_part("bowl")
    disc = object_model.get_part("disc")
    large_pusher = object_model.get_part("large_pusher")
    small_pusher = object_model.get_part("small_pusher")
    selector_knob = object_model.get_part("selector_knob")
    switch_0 = object_model.get_part("switch_0")
    switch_1 = object_model.get_part("switch_1")

    bowl_lock = object_model.get_articulation("base_to_bowl")
    disc_spin = object_model.get_articulation("base_to_disc")
    large_slide = object_model.get_articulation("bowl_to_large_pusher")
    small_slide = object_model.get_articulation("large_to_small_pusher")
    knob_spin = object_model.get_articulation("base_to_selector_knob")
    switch_joint_0 = object_model.get_articulation("base_to_switch_0")
    switch_joint_1 = object_model.get_articulation("base_to_switch_1")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    ctx.expect_overlap(
        disc,
        bowl,
        axes="xy",
        min_overlap=0.16,
        elem_a="disc_shell",
        elem_b="bowl_shell",
        name="disc stays centered under bowl shell",
    )

    handle_rest = _aabb_center(ctx.part_element_world_aabb(bowl, elem="handle"))
    with ctx.pose({bowl_lock: 0.35}):
        handle_twist = _aabb_center(ctx.part_element_world_aabb(bowl, elem="handle"))
    ctx.check(
        "bowl_twist_moves_handle",
        handle_rest is not None
        and handle_twist is not None
        and math.hypot(handle_rest[0] - handle_twist[0], handle_rest[1] - handle_twist[1]) > 0.03,
        details=f"rest={handle_rest}, twist={handle_twist}",
    )

    blade_rest = _aabb_center(ctx.part_element_world_aabb(disc, elem="blade"))
    with ctx.pose({disc_spin: 1.2}):
        blade_spun = _aabb_center(ctx.part_element_world_aabb(disc, elem="blade"))
    ctx.check(
        "disc_rotation_moves_blade",
        blade_rest is not None
        and blade_spun is not None
        and abs(blade_rest[0] - blade_spun[0]) > 0.01
        and abs(blade_rest[1] - blade_spun[1]) > 0.01,
        details=f"rest={blade_rest}, spun={blade_spun}",
    )

    large_rest = ctx.part_world_position(large_pusher)
    with ctx.pose({large_slide: 0.055}):
        large_pressed = ctx.part_world_position(large_pusher)
        ctx.expect_gap(
            large_pusher,
            disc,
            axis="z",
            min_gap=0.015,
            elem_b="blade",
            name="large pusher clears slicing disc when pressed",
        )
    ctx.check(
        "large_pusher_moves_down",
        large_rest is not None and large_pressed is not None and large_pressed[2] < large_rest[2] - 0.040,
        details=f"rest={large_rest}, pressed={large_pressed}",
    )

    small_rest = ctx.part_world_position(small_pusher)
    with ctx.pose({small_slide: 0.050}):
        small_pressed = ctx.part_world_position(small_pusher)
        ctx.expect_overlap(
            small_pusher,
            large_pusher,
            axes="z",
            min_overlap=0.030,
            name="small pusher stays nested in the large pusher",
        )
        ctx.expect_gap(
            small_pusher,
            disc,
            axis="z",
            min_gap=0.018,
            elem_a="small_plunger",
            elem_b="blade",
            name="small pusher clears slicing disc at full travel",
        )
    ctx.check(
        "small_pusher_moves_down",
        small_rest is not None and small_pressed is not None and small_pressed[2] < small_rest[2] - 0.035,
        details=f"rest={small_rest}, pressed={small_pressed}",
    )

    indicator_rest = _aabb_center(ctx.part_element_world_aabb(selector_knob, elem="indicator"))
    with ctx.pose({knob_spin: math.pi / 2.0}):
        indicator_quarter = _aabb_center(ctx.part_element_world_aabb(selector_knob, elem="indicator"))
    ctx.check(
        "selector_knob_rotates_indicator",
        indicator_rest is not None
        and indicator_quarter is not None
        and abs(indicator_rest[0] - indicator_quarter[0]) > 0.008
        and abs(indicator_rest[2] - indicator_quarter[2]) > 0.008,
        details=f"rest={indicator_rest}, quarter={indicator_quarter}",
    )

    for switch_part, switch_joint, label in (
        (switch_0, switch_joint_0, "switch_0"),
        (switch_1, switch_joint_1, "switch_1"),
    ):
        with ctx.pose({switch_joint: -0.24}):
            low_center = _aabb_center(ctx.part_element_world_aabb(switch_part, elem="rocker_shell"))
        with ctx.pose({switch_joint: 0.24}):
            high_center = _aabb_center(ctx.part_element_world_aabb(switch_part, elem="rocker_shell"))
        ctx.check(
            f"{label}_rocks_about_pivot",
            low_center is not None
            and high_center is not None
            and abs(low_center[2] - high_center[2]) > 0.002,
            details=f"low={low_center}, high={high_center}",
        )

    return ctx.report()


object_model = build_object_model()
