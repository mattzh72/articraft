from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _tube_mesh(name: str, outer_radius: float, inner_radius: float, height: float):
    """Open-ended vertical annular tube in local +Z."""
    tube = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    return mesh_from_cadquery(tube, name, tolerance=0.0008, angular_tolerance=0.08)


def _bowl_shell_mesh():
    outer_radius = 0.145
    inner_radius = 0.125
    height = 0.340
    bottom = 0.018
    shaft_hole = 0.026

    outer = cq.Workplane("XY").circle(outer_radius).extrude(height)
    open_volume = (
        cq.Workplane("XY")
        .workplane(offset=bottom)
        .circle(inner_radius)
        .extrude(height - bottom + 0.004)
    )
    center_bore = cq.Workplane("XY").circle(shaft_hole).extrude(bottom + 0.006)
    shell = outer.cut(open_volume).cut(center_bore)
    return mesh_from_cadquery(shell, "hollow_bowl_shell", tolerance=0.0008, angular_tolerance=0.08)


def _lid_mesh():
    lid_radius = 0.153
    height = 0.027
    feed_center_y = 0.065
    feed_hole_radius = 0.040

    disk = cq.Workplane("XY").circle(lid_radius).extrude(height)
    feed_opening = (
        cq.Workplane("XY")
        .center(0.0, feed_center_y)
        .circle(feed_hole_radius)
        .extrude(height + 0.006)
    )
    lid = disk.cut(feed_opening)
    return mesh_from_cadquery(lid, "lid_with_feed_opening", tolerance=0.0008, angular_tolerance=0.08)


def _blade_hub_mesh():
    hub = cq.Workplane("XY").circle(0.034).circle(0.015).extrude(0.046)
    blade_a = (
        cq.Workplane("XY")
        .box(0.130, 0.026, 0.006)
        .translate((0.040, 0.0, 0.040))
        .rotate((0, 0, 0), (0, 0, 1), 10)
    )
    blade_b = (
        cq.Workplane("XY")
        .box(0.130, 0.026, 0.006)
        .translate((-0.040, 0.0, 0.030))
        .rotate((0, 0, 0), (0, 0, 1), 190)
    )
    carrier = hub.union(blade_a).union(blade_b)
    return mesh_from_cadquery(carrier, "rotating_blade_hub", tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tall_cylindrical_food_processor")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("charcoal_controls", rgba=(0.05, 0.055, 0.060, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    transparent = model.material("clear_blue_tinted_plastic", rgba=(0.68, 0.88, 1.0, 0.36))
    smoky = model.material("smoky_clear_plastic", rgba=(0.40, 0.52, 0.60, 0.46))
    steel = model.material("brushed_stainless_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    label_white = model.material("white_print", rgba=(0.96, 0.96, 0.90, 1.0))
    red = model.material("red_pulse_accent", rgba=(0.75, 0.05, 0.03, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.190, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
        material=warm_white,
        name="round_base_body",
    )
    base.visual(
        Box((0.245, 0.066, 0.105)),
        origin=Origin(xyz=(0.0, -0.170, 0.069)),
        material=warm_white,
        name="front_fascia_block",
    )
    base.visual(
        Box((0.205, 0.006, 0.088)),
        origin=Origin(xyz=(0.0, -0.201, 0.073)),
        material=dark,
        name="front_control_panel",
    )
    base.visual(
        Cylinder(radius=0.123, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.136)),
        material=dark,
        name="bowl_lock_socket",
    )
    base.visual(
        Cylinder(radius=0.016, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, 0.182)),
        material=steel,
        name="center_shaft",
    )
    for foot_x, foot_y, foot_name in (
        (-0.110, -0.090, "foot_0"),
        (0.110, -0.090, "foot_1"),
        (-0.105, 0.100, "foot_2"),
        (0.105, 0.100, "foot_3"),
    ):
        base.visual(
            Cylinder(radius=0.024, length=0.012),
            origin=Origin(xyz=(foot_x, foot_y, 0.006)),
            material=rubber,
            name=foot_name,
        )

    bowl = model.part("bowl")
    bowl.visual(
        _tube_mesh("twist_lock_collar", 0.106, 0.030, 0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=dark,
        name="twist_lock_collar",
    )
    for angle_deg, tab_name in ((0, "lock_tab_0"), (120, "lock_tab_1"), (240, "lock_tab_2")):
        angle = math.radians(angle_deg)
        bowl.visual(
            Box((0.050, 0.018, 0.014)),
            origin=Origin(
                xyz=(0.112 * math.cos(angle), 0.112 * math.sin(angle), 0.014),
                rpy=(0.0, 0.0, angle),
            ),
            material=dark,
            name=tab_name,
        )
    bowl.visual(
        _bowl_shell_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=transparent,
        name="hollow_bowl_shell",
    )
    bowl.visual(
        _lid_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.365)),
        material=smoky,
        name="lid_with_feed_opening",
    )
    bowl.visual(
        Box((0.012, 0.086, 0.230)),
        origin=Origin(xyz=(0.047, 0.065, 0.501)),
        material=transparent,
        name="feed_side_wall_0",
    )
    bowl.visual(
        Box((0.012, 0.086, 0.230)),
        origin=Origin(xyz=(-0.047, 0.065, 0.501)),
        material=transparent,
        name="feed_side_wall_1",
    )
    bowl.visual(
        Box((0.106, 0.012, 0.230)),
        origin=Origin(xyz=(0.0, 0.102, 0.501)),
        material=transparent,
        name="feed_end_wall_0",
    )
    bowl.visual(
        Box((0.106, 0.012, 0.230)),
        origin=Origin(xyz=(0.0, 0.028, 0.501)),
        material=transparent,
        name="feed_end_wall_1",
    )
    bowl.visual(
        Box((0.046, 0.052, 0.026)),
        origin=Origin(xyz=(0.154, 0.0, 0.145)),
        material=dark,
        name="handle_lower_mount",
    )
    bowl.visual(
        Box((0.046, 0.052, 0.026)),
        origin=Origin(xyz=(0.154, 0.0, 0.292)),
        material=dark,
        name="handle_upper_mount",
    )
    bowl.visual(
        Box((0.026, 0.050, 0.170)),
        origin=Origin(xyz=(0.188, 0.0, 0.218)),
        material=dark,
        name="side_handle_grip",
    )

    blade_hub = model.part("blade_hub")
    blade_hub.visual(
        _blade_hub_mesh(),
        origin=Origin(),
        material=steel,
        name="blade_hub_and_blades",
    )

    main_pusher = model.part("main_pusher")
    main_pusher.visual(
        Box((0.022, 0.062, 0.240)),
        origin=Origin(xyz=(0.030, 0.0, -0.095)),
        material=smoky,
        name="main_side_wall_0",
    )
    main_pusher.visual(
        Box((0.022, 0.062, 0.240)),
        origin=Origin(xyz=(-0.030, 0.0, -0.095)),
        material=dark,
        name="main_side_wall_1",
    )
    main_pusher.visual(
        Box((0.082, 0.012, 0.240)),
        origin=Origin(xyz=(0.0, 0.025, -0.095)),
        material=smoky,
        name="main_end_wall_0",
    )
    main_pusher.visual(
        Box((0.082, 0.012, 0.240)),
        origin=Origin(xyz=(0.0, -0.025, -0.095)),
        material=smoky,
        name="main_end_wall_1",
    )
    main_pusher.visual(
        Box((0.037, 0.092, 0.018)),
        origin=Origin(xyz=(0.0375, 0.0, 0.011)),
        material=dark,
        name="main_flange_side_0",
    )
    main_pusher.visual(
        Box((0.037, 0.092, 0.018)),
        origin=Origin(xyz=(-0.0375, 0.0, 0.011)),
        material=dark,
        name="main_flange_side_1",
    )
    main_pusher.visual(
        Box((0.112, 0.027, 0.018)),
        origin=Origin(xyz=(0.0, 0.0325, 0.011)),
        material=dark,
        name="main_flange_end_0",
    )
    main_pusher.visual(
        Box((0.112, 0.027, 0.018)),
        origin=Origin(xyz=(0.0, -0.0325, 0.011)),
        material=dark,
        name="main_flange_end_1",
    )
    main_pusher.visual(
        Box((0.018, 0.006, 0.018)),
        origin=Origin(xyz=(0.0, -0.047, 0.011)),
        material=label_white,
        name="pusher_align_mark",
    )

    mini_pusher = model.part("mini_pusher")
    mini_pusher.visual(
        Cylinder(radius=0.019, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=smoky,
        name="mini_pusher_stem",
    )
    mini_pusher.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=smoky,
        name="mini_pusher_neck",
    )
    mini_pusher.visual(
        Cylinder(radius=0.027, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=dark,
        name="mini_pusher_cap",
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.038, length=0.026),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="speed_dial_body",
    )
    speed_dial.visual(
        Box((0.006, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, -0.028, 0.010)),
        material=label_white,
        name="speed_pointer_line",
    )

    pulse_lever = model.part("pulse_lever")
    pulse_lever.visual(
        Box((0.036, 0.018, 0.078)),
        origin=Origin(xyz=(0.0, -0.009, -0.038)),
        material=red,
        name="pulse_paddle",
    )
    pulse_lever.visual(
        Cylinder(radius=0.012, length=0.042),
        origin=Origin(xyz=(0.0, -0.014, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="pulse_pivot_pin",
    )

    model.articulation(
        "bowl_twist",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.2),
    )
    model.articulation(
        "blade_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade_hub,
        origin=Origin(xyz=(0.0, 0.0, 0.198)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=50.0),
    )
    model.articulation(
        "main_pusher_slide",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=main_pusher,
        origin=Origin(xyz=(0.0, 0.065, 0.622)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "mini_pusher_slide",
        ArticulationType.PRISMATIC,
        parent=main_pusher,
        child=mini_pusher,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.16, lower=0.0, upper=0.080),
    )
    model.articulation(
        "speed_dial_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_dial,
        origin=Origin(xyz=(-0.060, -0.204, 0.075)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=6.0),
    )
    model.articulation(
        "pulse_lever_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pulse_lever,
        origin=Origin(xyz=(0.065, -0.204, 0.101)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.38, upper=0.38),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    main_pusher = object_model.get_part("main_pusher")
    mini_pusher = object_model.get_part("mini_pusher")
    blade_hub = object_model.get_part("blade_hub")

    bowl_twist = object_model.get_articulation("bowl_twist")
    blade_spin = object_model.get_articulation("blade_spin")
    main_slide = object_model.get_articulation("main_pusher_slide")
    mini_slide = object_model.get_articulation("mini_pusher_slide")
    dial_spin = object_model.get_articulation("speed_dial_spin")
    pulse_pivot = object_model.get_articulation("pulse_lever_pivot")

    ctx.check(
        "primary mechanisms are articulated",
        bowl_twist.articulation_type == ArticulationType.CONTINUOUS
        and blade_spin.articulation_type == ArticulationType.CONTINUOUS
        and main_slide.articulation_type == ArticulationType.PRISMATIC
        and mini_slide.articulation_type == ArticulationType.PRISMATIC
        and dial_spin.articulation_type == ArticulationType.CONTINUOUS
        and pulse_pivot.articulation_type == ArticulationType.REVOLUTE,
        details="Expected continuous bowl/blade/dial, prismatic pushers, and revolute pulse lever.",
    )
    ctx.check(
        "pulse lever has short travel",
        pulse_pivot.motion_limits is not None
        and pulse_pivot.motion_limits.lower == -0.38
        and pulse_pivot.motion_limits.upper == 0.38,
        details=f"limits={pulse_pivot.motion_limits}",
    )

    ctx.allow_overlap(
        base,
        blade_hub,
        elem_a="center_shaft",
        elem_b="blade_hub_and_blades",
        reason="The rotating blade hub is intentionally captured on the center shaft with a tiny hidden shaft/bore interference.",
    )
    ctx.expect_overlap(
        base,
        blade_hub,
        axes="z",
        elem_a="center_shaft",
        elem_b="blade_hub_and_blades",
        min_overlap=0.020,
        name="blade hub is retained on center shaft",
    )

    ctx.expect_gap(
        bowl,
        main_pusher,
        axis="x",
        positive_elem="feed_side_wall_0",
        negative_elem="main_side_wall_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="main pusher touches right feed guide",
    )
    ctx.expect_gap(
        main_pusher,
        bowl,
        axis="x",
        positive_elem="main_side_wall_1",
        negative_elem="feed_side_wall_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="main pusher touches left feed guide",
    )
    ctx.expect_gap(
        bowl,
        main_pusher,
        axis="y",
        positive_elem="feed_end_wall_0",
        negative_elem="main_end_wall_0",
        max_gap=0.001,
        max_penetration=1e-6,
        name="main pusher touches front feed guide",
    )
    ctx.expect_gap(
        main_pusher,
        bowl,
        axis="y",
        positive_elem="main_end_wall_1",
        negative_elem="feed_end_wall_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="main pusher touches rear feed guide",
    )
    ctx.expect_overlap(
        main_pusher,
        bowl,
        axes="z",
        elem_a="main_side_wall_0",
        elem_b="feed_side_wall_0",
        min_overlap=0.150,
        name="main pusher remains inserted in feed chimney",
    )

    ctx.expect_gap(
        main_pusher,
        mini_pusher,
        axis="x",
        positive_elem="main_side_wall_0",
        negative_elem="mini_pusher_stem",
        max_gap=0.001,
        max_penetration=0.0,
        name="mini pusher touches right nested guide",
    )
    ctx.expect_gap(
        mini_pusher,
        main_pusher,
        axis="x",
        positive_elem="mini_pusher_stem",
        negative_elem="main_side_wall_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="mini pusher touches left nested guide",
    )
    ctx.expect_gap(
        main_pusher,
        mini_pusher,
        axis="y",
        positive_elem="main_end_wall_0",
        negative_elem="mini_pusher_stem",
        max_gap=0.001,
        max_penetration=1e-6,
        name="mini pusher touches front nested guide",
    )
    ctx.expect_gap(
        mini_pusher,
        main_pusher,
        axis="y",
        positive_elem="mini_pusher_stem",
        negative_elem="main_end_wall_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="mini pusher touches rear nested guide",
    )
    ctx.expect_overlap(
        mini_pusher,
        main_pusher,
        axes="z",
        elem_a="mini_pusher_stem",
        elem_b="main_side_wall_0",
        min_overlap=0.130,
        name="mini pusher remains inserted in nested sleeve",
    )
    ctx.expect_within(
        blade_hub,
        bowl,
        axes="xy",
        inner_elem="blade_hub_and_blades",
        outer_elem="hollow_bowl_shell",
        margin=0.0,
        name="rotary blade fits within bowl diameter",
    )

    rest_main = ctx.part_world_position(main_pusher)
    rest_mini = ctx.part_world_position(mini_pusher)
    with ctx.pose({main_slide: 0.100, mini_slide: 0.080}):
        extended_main = ctx.part_world_position(main_pusher)
        extended_mini = ctx.part_world_position(mini_pusher)
        ctx.expect_overlap(
            main_pusher,
            bowl,
            axes="z",
            elem_a="main_side_wall_0",
            elem_b="feed_side_wall_0",
            min_overlap=0.080,
            name="extended main pusher is still captured",
        )
        ctx.expect_overlap(
            mini_pusher,
            main_pusher,
            axes="z",
            elem_a="mini_pusher_stem",
            elem_b="main_side_wall_0",
            min_overlap=0.055,
            name="extended mini pusher is still captured",
        )
    ctx.check(
        "pusher slides move upward",
        rest_main is not None
        and extended_main is not None
        and rest_mini is not None
        and extended_mini is not None
        and extended_main[2] > rest_main[2] + 0.095
        and extended_mini[2] > rest_mini[2] + 0.170,
        details=f"rest_main={rest_main}, extended_main={extended_main}, rest_mini={rest_mini}, extended_mini={extended_mini}",
    )

    return ctx.report()


object_model = build_object_model()
