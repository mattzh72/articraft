from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.22
BASE_DEPTH = 0.21
BASE_TOP_Z = 0.132
PAD_TOP_Z = 0.140
PITCHER_HEIGHT = 0.255
PANEL_Y = 0.102
CONTROL_Y = 0.104


def _base_housing_shape() -> cq.Workplane:
    side_profile = [
        (-0.110, 0.000),
        (-0.110, 0.022),
        (-0.094, 0.110),
        (-0.018, 0.132),
        (0.048, 0.132),
        (0.086, 0.116),
        (0.100, 0.060),
        (0.100, 0.000),
    ]
    housing = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(BASE_WIDTH)
        .translate((-BASE_WIDTH * 0.5, 0.0, 0.0))
        .edges("|X")
        .fillet(0.010)
    )
    return housing


def _pitcher_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .rect(0.108, 0.104)
        .workplane(offset=PITCHER_HEIGHT)
        .rect(0.154, 0.146)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .rect(0.100, 0.096)
        .workplane(offset=PITCHER_HEIGHT - 0.007)
        .rect(0.146, 0.138)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.006))
    )
    return outer.cut(inner)


def _pitcher_handle_shape() -> cq.Workplane:
    loop = (
        cq.Workplane("YZ")
        .rect(0.052, 0.176)
        .extrude(0.016)
        .cut(
            cq.Workplane("YZ")
            .rect(0.026, 0.116)
            .extrude(0.020)
            .translate((-0.002, 0.0, 0.0))
        )
        .translate((0.073, 0.0, 0.146))
    )
    upper_mount = cq.Workplane("XY").box(0.018, 0.026, 0.018).translate((0.065, 0.0, 0.206))
    lower_mount = cq.Workplane("XY").box(0.018, 0.028, 0.020).translate((0.062, 0.0, 0.088))
    return loop.union(upper_mount).union(lower_mount)


def _blade_shape() -> cq.Workplane:
    hub = cq.Workplane("XY").circle(0.012).extrude(0.006).translate((0.0, 0.0, -0.003))

    blade_x_pos = (
        cq.Workplane("XY")
        .box(0.040, 0.010, 0.0016)
        .translate((0.020, 0.0, 0.002))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 12.0)
    )
    blade_x_neg = (
        cq.Workplane("XY")
        .box(0.040, 0.010, 0.0016)
        .translate((-0.020, 0.0, 0.002))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)
    )
    blade_y_pos = (
        cq.Workplane("XY")
        .box(0.010, 0.040, 0.0016)
        .translate((0.0, 0.020, 0.0018))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -12.0)
    )
    blade_y_neg = (
        cq.Workplane("XY")
        .box(0.010, 0.040, 0.0016)
        .translate((0.0, -0.020, 0.0018))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 12.0)
    )
    return hub.union(blade_x_pos).union(blade_x_neg).union(blade_y_pos).union(blade_y_neg)


def _pitcher_collar_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.116, 0.112, 0.018)
        .cut(cq.Workplane("XY").box(0.090, 0.086, 0.022))
        .translate((0.0, 0.0, 0.009))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smoothie_blender")

    body_dark = model.material("body_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    panel_black = model.material("panel_black", rgba=(0.08, 0.09, 0.10, 1.0))
    trim_black = model.material("trim_black", rgba=(0.05, 0.05, 0.06, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.74, 0.75, 0.77, 1.0))
    indicator_white = model.material("indicator_white", rgba=(0.94, 0.95, 0.96, 1.0))
    button_green = model.material("button_green", rgba=(0.18, 0.50, 0.33, 1.0))
    button_orange = model.material("button_orange", rgba=(0.78, 0.43, 0.12, 1.0))
    pitcher_clear = model.material("pitcher_clear", rgba=(0.84, 0.92, 0.98, 0.35))
    lid_dark = model.material("lid_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.70, 0.73, 0.76, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_housing_shape(), "blender_base"),
        material=body_dark,
        name="housing",
    )
    base.visual(
        Box((0.100, 0.100, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, PAD_TOP_Z - 0.004)),
        material=trim_black,
        name="seat_pad",
    )
    base.visual(
        Box((0.132, 0.004, 0.056)),
        origin=Origin(xyz=(0.0, PANEL_Y, 0.056)),
        material=panel_black,
        name="panel",
    )
    base.visual(
        Box((0.110, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.082, 0.004)),
        material=trim_black,
        name="rear_foot",
    )

    pitcher = model.part("pitcher")
    pitcher.visual(
        mesh_from_cadquery(_pitcher_shell_shape(), "pitcher_shell"),
        material=pitcher_clear,
        name="shell",
    )
    pitcher.visual(
        mesh_from_cadquery(_pitcher_collar_shape(), "pitcher_collar"),
        material=lid_dark,
        name="collar",
    )
    pitcher.visual(
        mesh_from_cadquery(_pitcher_handle_shape(), "pitcher_handle"),
        material=lid_dark,
        name="handle",
    )
    pitcher.visual(
        Box((0.148, 0.140, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, PITCHER_HEIGHT + 0.003)),
        material=lid_dark,
        name="lid",
    )
    pitcher.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(
            xyz=(0.0, 0.0, PITCHER_HEIGHT + 0.017),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=lid_dark,
        name="cap",
    )

    dial = model.part("dial")
    timer_knob = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.018,
            body_style="skirted",
            top_diameter=0.030,
            center=False,
        ),
        "timer_dial",
    )
    dial.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="shaft",
    )
    dial.visual(
        timer_knob,
        origin=Origin(xyz=(0.0, 0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_silver,
        name="knob",
    )
    dial.visual(
        Box((0.002, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, 0.030, 0.007)),
        material=indicator_white,
        name="pointer",
    )

    for index, x_pos in enumerate((0.020, 0.058)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.028, 0.008, 0.018)),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=button_green if index == 0 else button_orange,
            name="cap",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_Y, 0.042)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.10,
                lower=0.0,
                upper=0.004,
            ),
        )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_shape(), "blade_assembly"),
        material=blade_steel,
        name="hub",
    )
    blade.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=blade_steel,
        name="shaft",
    )

    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.0, PAD_TOP_Z)),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(-0.046, CONTROL_Y, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )
    model.articulation(
        "pitcher_to_blade",
        ArticulationType.CONTINUOUS,
        parent=pitcher,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=80.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")
    blade = object_model.get_part("blade")

    dial_joint = object_model.get_articulation("base_to_dial")
    button_0_joint = object_model.get_articulation("base_to_button_0")
    button_1_joint = object_model.get_articulation("base_to_button_1")
    blade_joint = object_model.get_articulation("pitcher_to_blade")

    ctx.allow_overlap(
        blade,
        pitcher,
        elem_a="shaft",
        elem_b="shell",
        reason="The blade spindle intentionally passes through the simplified pitcher-bottom bearing seat.",
    )

    ctx.expect_gap(
        pitcher,
        base,
        axis="z",
        positive_elem="shell",
        negative_elem="seat_pad",
        max_gap=0.001,
        max_penetration=0.0,
        name="pitcher seats on the motor pad",
    )
    ctx.expect_overlap(
        pitcher,
        base,
        axes="xy",
        elem_a="shell",
        elem_b="seat_pad",
        min_overlap=0.090,
        name="pitcher footprint stays over the base pad",
    )

    ctx.expect_gap(
        dial,
        base,
        axis="y",
        positive_elem="shaft",
        negative_elem="panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="dial shaft starts at the front control panel",
    )
    ctx.expect_gap(
        button_0,
        base,
        axis="y",
        positive_elem="cap",
        negative_elem="panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="button_0 sits proud of the panel face",
    )
    ctx.expect_gap(
        button_1,
        base,
        axis="y",
        positive_elem="cap",
        negative_elem="panel",
        max_gap=0.0005,
        max_penetration=0.0,
        name="button_1 sits proud of the panel face",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        outer_elem="shell",
        margin=0.020,
        name="blade stays within the pitcher body footprint",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_0_joint: 0.004}):
        pressed_button_0 = ctx.part_world_position(button_0)
        other_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.004}):
        pressed_button_1 = ctx.part_world_position(button_1)
        other_button_0 = ctx.part_world_position(button_0)

    ctx.check(
        "button_0 depresses independently",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and other_button_1 is not None
        and pressed_button_0[1] < rest_button_0[1] - 0.003
        and math.isclose(other_button_1[1], rest_button_1[1], abs_tol=1e-6),
        details=f"rest0={rest_button_0}, pressed0={pressed_button_0}, rest1={rest_button_1}, other1={other_button_1}",
    )
    ctx.check(
        "button_1 depresses independently",
        rest_button_1 is not None
        and pressed_button_1 is not None
        and rest_button_0 is not None
        and other_button_0 is not None
        and pressed_button_1[1] < rest_button_1[1] - 0.003
        and math.isclose(other_button_0[1], rest_button_0[1], abs_tol=1e-6),
        details=f"rest1={rest_button_1}, pressed1={pressed_button_1}, rest0={rest_button_0}, other0={other_button_0}",
    )

    rest_dial = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi * 0.75}):
        turned_dial = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates without shifting off the panel",
        rest_dial is not None
        and turned_dial is not None
        and all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(rest_dial, turned_dial)),
        details=f"rest={rest_dial}, turned={turned_dial}",
    )

    rest_blade = ctx.part_world_position(blade)
    with ctx.pose({blade_joint: math.pi * 0.5}):
        spun_blade = ctx.part_world_position(blade)
    ctx.check(
        "blade spins in place under the pitcher",
        rest_blade is not None
        and spun_blade is not None
        and all(math.isclose(a, b, abs_tol=1e-6) for a, b in zip(rest_blade, spun_blade)),
        details=f"rest={rest_blade}, spun={spun_blade}",
    )

    return ctx.report()


object_model = build_object_model()
