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


BASE_DEPTH = 0.280
BASE_WIDTH = 0.270
BASE_HEIGHT = 0.090
POD_DEPTH = 0.180
POD_WIDTH = 0.220
POD_HEIGHT = 0.058
POD_CENTER_X = -0.032


def _build_chassis_shell():
    base = (
        cq.Workplane("XY")
        .box(BASE_DEPTH, BASE_WIDTH, BASE_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.012)
        .edges(">Z")
        .fillet(0.008)
    )

    drawer_cavity = (
        cq.Workplane("XY")
        .box(0.245, 0.220, 0.054, centered=(True, True, False))
        .translate((0.028, 0.0, 0.014))
    )
    base = base.cut(drawer_cavity)

    pod = (
        cq.Workplane("XY")
        .box(POD_DEPTH, POD_WIDTH, POD_HEIGHT, centered=(True, True, False))
        .translate((POD_CENTER_X, 0.0, BASE_HEIGHT - 0.002))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.012)
    )
    pod_cap = (
        cq.Workplane("XY")
        .box(0.090, 0.170, 0.018, centered=(True, True, False))
        .translate((-0.068, 0.0, BASE_HEIGHT + POD_HEIGHT - 0.004))
        .edges("|Z")
        .fillet(0.010)
        .edges(">Z")
        .fillet(0.006)
    )

    return base.union(pod).union(pod_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boutique_cash_register")

    body = model.material("body", rgba=(0.83, 0.81, 0.77, 1.0))
    trim = model.material("trim", rgba=(0.32, 0.31, 0.30, 1.0))
    button_dark = model.material("button_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    button_accent = model.material("button_accent", rgba=(0.46, 0.44, 0.32, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.35, 0.39, 0.55))
    metal = model.material("metal", rgba=(0.72, 0.73, 0.75, 1.0))
    key_black = model.material("key_black", rgba=(0.11, 0.12, 0.13, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        mesh_from_cadquery(_build_chassis_shell(), "cash_register_chassis"),
        material=body,
        name="shell",
    )
    chassis.visual(
        Box((0.214, 0.236, 0.006)),
        origin=Origin(xyz=(0.028, 0.0, 0.012)),
        material=trim,
        name="drawer_track_floor",
    )
    chassis.visual(
        Box((0.112, 0.116, 0.004)),
        origin=Origin(xyz=(-0.006, 0.0, 0.146)),
        material=trim,
        name="keypad_deck",
    )
    for x_pos in (-0.104, 0.104):
        for y_pos in (-0.102, 0.102):
            chassis.visual(
                Cylinder(radius=0.010, length=0.006),
                origin=Origin(xyz=(x_pos, y_pos, 0.003)),
                material=trim,
                name=f"foot_{int((x_pos > 0) * 2 + (y_pos > 0))}",
            )
    chassis.visual(
        Box((0.014, 0.108, 0.020)),
        origin=Origin(xyz=(-0.117, 0.0, 0.153)),
        material=trim,
        name="display_bracket",
    )
    chassis.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(xyz=(-0.117, 0.0, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="display_hinge_barrel",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.210, 0.214, 0.048)),
        material=body,
        name="drawer_body",
    )
    drawer.visual(
        Box((0.008, 0.228, 0.060)),
        origin=Origin(xyz=(0.109, 0.0, 0.0)),
        material=body,
        name="drawer_face",
    )
    drawer.visual(
        Box((0.010, 0.020, 0.022)),
        origin=Origin(xyz=(0.112, -0.048, 0.0)),
        material=trim,
        name="handle_post_0",
    )
    drawer.visual(
        Box((0.010, 0.020, 0.022)),
        origin=Origin(xyz=(0.112, 0.048, 0.0)),
        material=trim,
        name="handle_post_1",
    )
    drawer.visual(
        Cylinder(radius=0.008, length=0.118),
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim,
        name="handle_grip",
    )

    display = model.part("display")
    display.visual(
        Box((0.062, 0.104, 0.012)),
        origin=Origin(xyz=(0.032, 0.0, 0.016)),
        material=trim,
        name="display_housing",
    )
    display.visual(
        Box((0.050, 0.090, 0.002)),
        origin=Origin(xyz=(0.036, 0.0, 0.023)),
        material=display_glass,
        name="display_screen",
    )
    for y_pos, index in ((-0.032, 0), (0.032, 1)):
        display.visual(
            Box((0.010, 0.018, 0.016)),
            origin=Origin(xyz=(0.009, y_pos, 0.008)),
            material=trim,
            name=f"display_arm_{index}",
        )
    for y_pos, index in ((-0.032, 0), (0.032, 1)):
        display.visual(
            Cylinder(radius=0.004, length=0.028),
            origin=Origin(xyz=(0.006, y_pos, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=f"display_knuckle_{index}",
        )

    mode_switch = model.part("mode_switch")
    mode_switch.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="switch_bezel",
    )
    mode_switch.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=metal,
        name="switch_barrel",
    )
    mode_switch.visual(
        Box((0.020, 0.005, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.016)),
        material=key_black,
        name="key_head",
    )
    mode_switch.visual(
        Box((0.010, 0.0025, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=metal,
        name="key_blade",
    )
    mode_switch.visual(
        Box((0.004, 0.014, 0.006)),
        origin=Origin(xyz=(-0.003, 0.0, 0.016)),
        material=key_black,
        name="key_bow",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=chassis,
        child=drawer,
        origin=Origin(xyz=(0.036, 0.0, 0.039)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.18, lower=0.0, upper=0.086),
    )
    model.articulation(
        "display_hinge",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=display,
        origin=Origin(xyz=(-0.117, 0.0, 0.165)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "mode_switch_spin",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=mode_switch,
        origin=Origin(xyz=(0.038, -0.095, 0.146)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )

    key_x_positions = (-0.034, -0.006, 0.022)
    key_y_positions = (0.040, 0.013, -0.014, -0.041)
    for row_index, y_pos in enumerate(key_y_positions):
        for col_index, x_pos in enumerate(key_x_positions):
            key = model.part(f"key_{row_index}_{col_index}")
            key.visual(
                Box((0.020, 0.020, 0.006)),
                origin=Origin(xyz=(0.0, 0.0, 0.003)),
                material=button_accent if row_index == 3 and col_index == 2 else button_dark,
                name="keycap",
            )
            key.visual(
                Box((0.012, 0.012, 0.004)),
                origin=Origin(xyz=(0.0, 0.0, 0.000)),
                material=button_accent if row_index == 3 and col_index == 2 else button_dark,
                name="plunger",
            )
            model.articulation(
                f"keypad_to_key_{row_index}_{col_index}",
                ArticulationType.PRISMATIC,
                parent=chassis,
                child=key,
                origin=Origin(xyz=(x_pos, y_pos, 0.148)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    effort=3.0,
                    velocity=0.06,
                    lower=0.0,
                    upper=0.0025,
                ),
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    chassis = object_model.get_part("chassis")
    drawer = object_model.get_part("drawer")
    display = object_model.get_part("display")
    mode_switch = object_model.get_part("mode_switch")
    nearest_key = object_model.get_part("key_3_2")
    drawer_slide = object_model.get_articulation("drawer_slide")
    display_hinge = object_model.get_articulation("display_hinge")
    mode_switch_spin = object_model.get_articulation("mode_switch_spin")

    drawer_limits = drawer_slide.motion_limits
    display_limits = display_hinge.motion_limits

    ctx.expect_overlap(drawer, chassis, axes="yz", min_overlap=0.040, name="drawer aligns within chassis opening")

    rest_drawer_pos = ctx.part_world_position(drawer)
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_overlap(drawer, chassis, axes="yz", min_overlap=0.040, name="drawer stays aligned when extended")
            ctx.expect_overlap(drawer, chassis, axes="x", min_overlap=0.120, name="drawer keeps retained insertion")
            extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends forward",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[0] > rest_drawer_pos[0] + 0.080,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    closed_display_aabb = ctx.part_world_aabb(display)
    ctx.check(
        "display sits at rear edge",
        closed_display_aabb is not None and closed_display_aabb[0][0] < -0.110,
        details=f"closed_aabb={closed_display_aabb}",
    )
    if display_limits is not None and display_limits.upper is not None:
        with ctx.pose({display_hinge: display_limits.upper}):
            opened_display_aabb = ctx.part_world_aabb(display)
        ctx.check(
            "display flips upward",
            closed_display_aabb is not None
            and opened_display_aabb is not None
            and opened_display_aabb[1][2] > closed_display_aabb[1][2] + 0.035,
            details=f"closed_aabb={closed_display_aabb}, opened_aabb={opened_display_aabb}",
        )

    switch_aabb_0 = ctx.part_world_aabb(mode_switch)
    with ctx.pose({mode_switch_spin: math.pi / 2.0}):
        switch_aabb_90 = ctx.part_world_aabb(mode_switch)

    def _xy_size(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return (maxs[0] - mins[0], maxs[1] - mins[1])

    switch_size_0 = _xy_size(switch_aabb_0)
    switch_size_90 = _xy_size(switch_aabb_90)
    ctx.expect_origin_distance(
        mode_switch,
        nearest_key,
        axes="xy",
        min_dist=0.040,
        name="mode switch stays separate from keypad",
    )
    ctx.check(
        "mode switch visibly rotates",
        switch_size_0 is not None
        and switch_size_90 is not None
        and abs(switch_size_0[0] - switch_size_90[0]) > 0.0035,
        details=f"size_0={switch_size_0}, size_90={switch_size_90}",
    )

    return ctx.report()


object_model = build_object_model()
