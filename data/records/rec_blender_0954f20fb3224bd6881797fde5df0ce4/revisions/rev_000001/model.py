from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_WIDTH = 0.24
BASE_DEPTH = 0.29
BASE_HEIGHT = 0.16
PITCHER_HEIGHT = 0.295
HOOD_WIDTH = 0.212
HOOD_DEPTH = 0.232
HOOD_HEIGHT = 0.375
HOOD_WALL = 0.006


def _base_shape() -> object:
    lower = cq.Workplane("XY").box(BASE_WIDTH, BASE_DEPTH, 0.105).translate((0.0, 0.0, 0.0525))
    upper = cq.Workplane("XY").box(0.192, 0.220, 0.055).translate((0.0, 0.008, 0.1325))
    body = lower.union(upper)
    body = body.edges("|Z").fillet(0.018)
    body = body.edges(">Z").fillet(0.010)
    return body


def _pitcher_body_shape() -> object:
    outer = (
        cq.Workplane("XY")
        .rect(0.090, 0.090)
        .workplane(offset=PITCHER_HEIGHT)
        .rect(0.152, 0.152)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY")
        .rect(0.078, 0.078)
        .workplane(offset=PITCHER_HEIGHT - 0.012)
        .rect(0.140, 0.140)
        .loft(combine=True)
        .translate((0.0, 0.0, 0.008))
    )
    vessel = outer.cut(inner)
    pour_cut = cq.Workplane("XY").box(0.040, 0.030, 0.020).translate((0.0, -0.082, PITCHER_HEIGHT - 0.002))
    grip = cq.Workplane("XY").circle(0.012).extrude(0.155).translate((0.0, -0.106, 0.090))
    upper_arm = cq.Workplane("XY").box(0.026, 0.044, 0.020).translate((0.0, -0.084, 0.214))
    lower_arm = cq.Workplane("XY").box(0.024, 0.042, 0.036).translate((0.0, -0.084, 0.092))
    return vessel.cut(pour_cut).union(grip).union(upper_arm).union(lower_arm)


def _pitcher_lid_shape() -> object:
    lid = cq.Workplane("XY").box(0.120, 0.120, 0.012).translate((0.0, 0.0, 0.006))
    cap = cq.Workplane("XY").circle(0.018).extrude(0.010).translate((0.0, 0.0, 0.012))
    return lid.union(cap).edges("|Z").fillet(0.004)


def _hood_shape() -> object:
    outer = cq.Workplane("XY").box(HOOD_WIDTH, HOOD_DEPTH, HOOD_HEIGHT).translate((0.0, -HOOD_DEPTH / 2.0, HOOD_HEIGHT / 2.0))
    outer = outer.edges("|Z").fillet(0.015)
    outer = outer.edges(">Z").fillet(0.012)
    inner = (
        cq.Workplane("XY")
        .box(HOOD_WIDTH - 2.0 * HOOD_WALL, HOOD_DEPTH - 2.0 * HOOD_WALL, HOOD_HEIGHT - HOOD_WALL)
        .translate((0.0, -HOOD_DEPTH / 2.0, (HOOD_HEIGHT - HOOD_WALL) / 2.0))
    )
    shell = outer.cut(inner)
    top_window = cq.Workplane("XY").box(0.090, 0.120, 0.030).translate((0.0, -0.110, 0.292))
    return shell.cut(top_window)


def _blade_shape() -> object:
    hub = cq.Workplane("XY").circle(0.016).extrude(0.008)
    lock_nut = cq.Workplane("XY").polygon(6, 0.026).extrude(0.004).translate((0.0, 0.0, 0.008))
    long_blade = (
        cq.Workplane("XY")
        .box(0.038, 0.010, 0.002)
        .translate((0.014, 0.0, 0.009))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 16.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 12.0)
    )
    long_blade_back = (
        cq.Workplane("XY")
        .box(0.038, 0.010, 0.002)
        .translate((-0.014, 0.0, 0.009))
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -16.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 12.0)
    )
    short_blade = (
        cq.Workplane("XY")
        .box(0.030, 0.010, 0.002)
        .translate((0.0, 0.012, 0.011))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -14.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 78.0)
    )
    short_blade_back = (
        cq.Workplane("XY")
        .box(0.030, 0.010, 0.002)
        .translate((0.0, -0.012, 0.011))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 14.0)
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 78.0)
    )
    return hub.union(lock_nut).union(long_blade).union(long_blade_back).union(short_blade).union(short_blade_back)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_quiet_blender")

    base_black = model.material("base_black", rgba=(0.12, 0.13, 0.14, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.08, 0.10, 0.11, 1.0))
    metal = model.material("metal", rgba=(0.78, 0.80, 0.82, 1.0))
    clear_poly = model.material("clear_poly", rgba=(0.78, 0.89, 0.96, 0.28))
    rubber = model.material("rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_base_shape(), "blender_base"), material=base_black, name="body")
    base.visual(
        Box((0.124, 0.006, 0.046)),
        origin=Origin(xyz=(0.0, -0.103, 0.116)),
        material=panel_dark,
        name="front_panel",
    )
    base.visual(
        Box((0.094, 0.094, 0.004)),
        origin=Origin(xyz=(0.0, 0.004, BASE_HEIGHT + 0.002)),
        material=rubber,
        name="seat_pad",
    )
    for foot_x in (-0.085, 0.085):
        for foot_y in (-0.100, 0.100):
            base.visual(
                Cylinder(radius=0.013, length=0.010),
                origin=Origin(xyz=(foot_x, foot_y, 0.005)),
                material=rubber,
                name=f"foot_{int((foot_x > 0))}_{int((foot_y > 0))}",
            )
    for bracket_x in (-0.078, 0.078):
        base.visual(
            Box((0.014, 0.030, 0.028)),
            origin=Origin(xyz=(bracket_x, 0.118, BASE_HEIGHT + 0.014)),
            material=panel_dark,
            name=f"hinge_bracket_{int(bracket_x > 0)}",
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT + 0.028)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.092)),
    )

    pitcher = model.part("pitcher")
    pitcher.visual(mesh_from_cadquery(_pitcher_body_shape(), "pitcher_body"), material=clear_poly, name="body")
    pitcher.visual(
        mesh_from_cadquery(_pitcher_lid_shape(), "pitcher_lid"),
        origin=Origin(xyz=(0.0, 0.0, PITCHER_HEIGHT)),
        material=rubber,
        name="lid",
    )
    pitcher.inertial = Inertial.from_geometry(
        Box((0.152, 0.218, PITCHER_HEIGHT + 0.018)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -0.018, 0.150)),
    )
    model.articulation(
        "base_to_pitcher",
        ArticulationType.FIXED,
        parent=base,
        child=pitcher,
        origin=Origin(xyz=(0.0, 0.004, BASE_HEIGHT + 0.004)),
    )

    hood = model.part("hood")
    hood.visual(mesh_from_cadquery(_hood_shape(), "sound_hood"), material=clear_poly, name="shell")
    hood.visual(
        Box((0.104, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -HOOD_DEPTH - 0.004, 0.214)),
        material=panel_dark,
        name="pull_bar",
    )
    hood.inertial = Inertial.from_geometry(
        Box((HOOD_WIDTH, HOOD_DEPTH, HOOD_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, -HOOD_DEPTH / 2.0, HOOD_HEIGHT / 2.0)),
    )
    model.articulation(
        "base_to_hood",
        ArticulationType.REVOLUTE,
        parent=base,
        child=hood,
        origin=Origin(xyz=(0.0, 0.106, BASE_HEIGHT)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.20),
    )

    speed_knob = model.part("speed_knob")
    speed_knob.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="shaft",
    )
    speed_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.050,
                0.026,
                body_style="skirted",
                top_diameter=0.038,
                skirt=KnobSkirt(0.058, 0.006, flare=0.08),
                grip=KnobGrip(style="fluted", count=16, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
                center=False,
            ),
            "speed_knob",
        ),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=base_black,
        name="knob",
    )
    speed_knob.visual(
        Box((0.012, 0.006, 0.004)),
        origin=Origin(xyz=(0.030, 0.0, 0.024)),
        material=metal,
        name="pointer_fin",
    )
    speed_knob.inertial = Inertial.from_geometry(
        Box((0.040, 0.058, 0.058)),
        mass=0.14,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )
    model.articulation(
        "base_to_speed_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=speed_knob,
        origin=Origin(xyz=(BASE_WIDTH / 2.0 - 0.001, 0.028, 0.104)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    blade = model.part("blade")
    blade.visual(mesh_from_cadquery(_blade_shape(), "blade_set"), material=metal, name="blade_set")
    blade.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="coupler",
    )
    blade.visual(
        Box((0.012, 0.010, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, 0.010)),
        material=metal,
        name="tip",
    )
    blade.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )
    model.articulation(
        "base_to_blade",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=blade,
        origin=Origin(xyz=(0.0, 0.004, BASE_HEIGHT + 0.022)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=45.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    pitcher = object_model.get_part("pitcher")
    hood = object_model.get_part("hood")
    speed_knob = object_model.get_part("speed_knob")
    blade = object_model.get_part("blade")
    hood_joint = object_model.get_articulation("base_to_hood")
    knob_joint = object_model.get_articulation("base_to_speed_knob")
    blade_joint = object_model.get_articulation("base_to_blade")

    ctx.allow_overlap(
        blade,
        pitcher,
        elem_a="coupler",
        elem_b="body",
        reason="The blade spindle is intentionally represented as passing through the pitcher floor and gasket seat.",
    )

    ctx.expect_gap(
        pitcher,
        base,
        axis="z",
        positive_elem="body",
        negative_elem="seat_pad",
        min_gap=0.0,
        max_gap=0.004,
        name="pitcher sits on the base pad",
    )
    ctx.expect_overlap(
        hood,
        pitcher,
        axes="xy",
        elem_a="shell",
        elem_b="body",
        min_overlap=0.135,
        name="hood covers the pitcher footprint",
    )
    ctx.expect_gap(
        hood,
        base,
        axis="z",
        positive_elem="shell",
        negative_elem="body",
        max_penetration=0.0,
        min_gap=0.0,
        name="hood shell does not sink into the base",
    )

    closed_pull = ctx.part_element_world_aabb(hood, elem="pull_bar")
    with ctx.pose({hood_joint: 1.0}):
        open_pull = ctx.part_element_world_aabb(hood, elem="pull_bar")

    hood_opens = (
        closed_pull is not None
        and open_pull is not None
        and open_pull[0][2] > closed_pull[1][2] + 0.05
        and open_pull[0][1] > closed_pull[0][1] + 0.05
    )
    ctx.check(
        "hood opens upward from the rear hinge",
        hood_opens,
        details=f"closed_pull={closed_pull}, open_pull={open_pull}",
    )

    knob_rest = ctx.part_element_world_aabb(speed_knob, elem="pointer_fin")
    with ctx.pose({knob_joint: math.pi / 2.0}):
        knob_quarter = ctx.part_element_world_aabb(speed_knob, elem="pointer_fin")
    knob_rotates = (
        knob_rest is not None
        and knob_quarter is not None
        and abs(knob_quarter[0][2] - knob_rest[0][2]) > 0.015
        and abs(knob_quarter[0][1] - knob_rest[0][1]) > 0.015
    )
    ctx.check(
        "speed knob rotates on its side shaft",
        knob_rotates,
        details=f"rest={knob_rest}, quarter={knob_quarter}",
    )

    blade_rest = ctx.part_element_world_aabb(blade, elem="tip")
    with ctx.pose({blade_joint: math.pi / 2.0}):
        blade_quarter = ctx.part_element_world_aabb(blade, elem="tip")
    blade_spins = (
        blade_rest is not None
        and blade_quarter is not None
        and abs(blade_quarter[0][0] - blade_rest[0][0]) > 0.012
        and abs(blade_quarter[0][1] - blade_rest[0][1]) > 0.012
    )
    ctx.check(
        "blade spins inside the pitcher",
        blade_spins,
        details=f"rest={blade_rest}, quarter={blade_quarter}",
    )
    ctx.expect_within(
        blade,
        pitcher,
        axes="xy",
        elem_a="blade_set",
        elem_b="body",
        margin=0.030,
        name="blade footprint stays within the pitcher walls",
    )

    return ctx.report()


object_model = build_object_model()
