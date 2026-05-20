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


def _box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]):
    """CadQuery cylinder aligned to the model X axis."""
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - length / 2.0, center[1], center[2]))
    )


def _make_housing_geometry():
    body = _box((0.180, 0.108, 0.145), (0.0, 0.0, 0.0905))

    base = _box((0.205, 0.125, 0.018), (0.0, 0.0, 0.009))
    housing = body.union(base)

    # Raised cast front ring around the pencil entry.
    housing = housing.union(_cylinder_x(0.030, 0.017, (-0.0955, 0.0, 0.108)))

    # Rear bearing boss for the hand crank.
    housing = housing.union(_cylinder_x(0.020, 0.017, (0.0955, 0.0, 0.108)))

    # Drawer frame lip and selector rails are cast into the housing front.
    housing = housing.union(_box((0.012, 0.084, 0.006), (-0.094, 0.0, 0.071)))
    housing = housing.union(_box((0.012, 0.084, 0.006), (-0.094, 0.0, 0.018)))
    housing = housing.union(_box((0.012, 0.006, 0.050), (-0.094, -0.041, 0.044)))
    housing = housing.union(_box((0.012, 0.006, 0.050), (-0.094, 0.041, 0.044)))

    housing = housing.union(_box((0.012, 0.004, 0.047), (-0.094, -0.052, 0.108)))
    housing = housing.union(_box((0.012, 0.004, 0.047), (-0.094, -0.028, 0.108)))

    # True voids: pencil tunnel, shavings-drawer pocket, selector slot,
    # and rear shaft clearance.  The pencil tunnel is a blind cylindrical
    # bore with real depth into the mechanism area, not a painted-on circle.
    housing = housing.cut(_cylinder_x(0.0185, 0.146, (-0.038, 0.0, 0.108)))
    housing = housing.cut(_box((0.104, 0.076, 0.050), (-0.057, 0.0, 0.045)))
    housing = housing.cut(_box((0.032, 0.010, 0.044), (-0.095, -0.040, 0.108)))
    housing = housing.cut(_cylinder_x(0.0105, 0.034, (0.096, 0.0, 0.108)))

    return housing


def _make_drawer_geometry():
    drawer = _box((0.008, 0.070, 0.040), (-0.007, 0.0, 0.0))
    drawer = drawer.union(_box((0.086, 0.070, 0.004), (0.039, 0.0, -0.019)))
    drawer = drawer.union(_box((0.086, 0.005, 0.034), (0.039, -0.0355, -0.004)))
    drawer = drawer.union(_box((0.086, 0.005, 0.034), (0.039, 0.0355, -0.004)))
    drawer = drawer.union(_box((0.004, 0.062, 0.030), (0.080, 0.0, -0.006)))
    return drawer


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vintage_desktop_pencil_sharpener")

    model.material("cast_green", rgba=(0.10, 0.24, 0.18, 1.0))
    model.material("dark_shadow", rgba=(0.01, 0.01, 0.009, 1.0))
    model.material("brushed_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    model.material("dull_nickel", rgba=(0.38, 0.40, 0.39, 1.0))
    model.material("drawer_metal", rgba=(0.30, 0.32, 0.31, 1.0))
    model.material("warm_wood", rgba=(0.55, 0.32, 0.15, 1.0))
    model.material("black_rubber", rgba=(0.025, 0.025, 0.023, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_make_housing_geometry(), "cast_housing", tolerance=0.0007),
        material="cast_green",
        name="cast_shell",
    )
    housing.visual(
        Cylinder(radius=0.0165, length=0.004),
        origin=Origin(xyz=(0.034, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="dark_shadow",
        name="port_depth_shadow",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_make_drawer_geometry(), "shavings_drawer", tolerance=0.0007),
        material="drawer_metal",
        name="drawer_tray",
    )
    drawer.visual(
        Cylinder(radius=0.006, length=0.036),
        origin=Origin(xyz=(-0.012, 0.0, -0.003), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="black_rubber",
        name="drawer_pull",
    )

    crank = model.part("crank")
    x_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    crank.visual(
        Cylinder(radius=0.0105, length=0.040),
        origin=Origin(xyz=(0.020, 0.0, 0.0), rpy=x_axis.rpy),
        material="brushed_steel",
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.018, length=0.008),
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=x_axis.rpy),
        material="dull_nickel",
        name="hub",
    )
    crank.visual(
        Box((0.007, 0.012, 0.070)),
        origin=Origin(xyz=(0.046, 0.0, 0.037)),
        material="brushed_steel",
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.060, 0.0, 0.074), rpy=x_axis.rpy),
        material="brushed_steel",
        name="handle_post",
    )
    crank.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(xyz=(0.082, 0.0, 0.074), rpy=x_axis.rpy),
        material="warm_wood",
        name="wood_grip",
    )

    selector = model.part("selector")
    selector.visual(
        Box((0.008, 0.014, 0.016)),
        origin=Origin(xyz=(-0.007, 0.0, 0.0)),
        material="dull_nickel",
        name="thumb_tab",
    )
    selector.visual(
        Cylinder(radius=0.0050, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="guide_pin",
    )

    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(-0.094, 0.0, 0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.25, lower=0.0, upper=0.065),
    )
    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=crank,
        origin=Origin(xyz=(0.090, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=8.0),
    )
    model.articulation(
        "selector_slide",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=selector,
        origin=Origin(xyz=(-0.095, -0.040, 0.091)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.12, lower=0.0, upper=0.028),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    selector = object_model.get_part("selector")

    drawer_slide = object_model.get_articulation("drawer_slide")
    crank_spin = object_model.get_articulation("crank_spin")
    selector_slide = object_model.get_articulation("selector_slide")

    ctx.allow_overlap(
        drawer,
        housing,
        elem_a="drawer_tray",
        elem_b="cast_shell",
        reason=(
            "The shavings drawer is intentionally represented as a sliding tray "
            "retained inside the front pocket of the simplified cast shell."
        ),
    )
    ctx.allow_overlap(
        crank,
        housing,
        elem_a="shaft",
        elem_b="cast_shell",
        reason=(
            "The crank shaft is intentionally captured through the rear bearing "
            "boss so the continuously rotating handle has a physical support."
        ),
    )

    ctx.check(
        "crank uses continuous rotation",
        crank_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={crank_spin.articulation_type}",
    )

    ctx.expect_within(
        drawer,
        housing,
        axes="yz",
        inner_elem="drawer_tray",
        outer_elem="cast_shell",
        margin=0.003,
        name="drawer stays aligned in front pocket",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="x",
        elem_a="drawer_tray",
        elem_b="cast_shell",
        min_overlap=0.050,
        name="closed drawer retains long insertion",
    )
    ctx.expect_overlap(
        crank,
        housing,
        axes="x",
        elem_a="shaft",
        elem_b="cast_shell",
        min_overlap=0.010,
        name="crank shaft remains seated in rear bearing",
    )
    ctx.expect_within(
        crank,
        housing,
        axes="yz",
        inner_elem="shaft",
        outer_elem="cast_shell",
        margin=0.002,
        name="crank shaft is centered in bearing height and width",
    )
    ctx.expect_within(
        selector,
        housing,
        axes="y",
        inner_elem="guide_pin",
        outer_elem="cast_shell",
        margin=0.002,
        name="selector pin remains beside the port",
    )

    drawer_rest = ctx.part_world_position(drawer)
    selector_rest = ctx.part_world_position(selector)
    with ctx.pose({drawer_slide: 0.065, selector_slide: 0.028, crank_spin: math.tau}):
        ctx.expect_overlap(
            drawer,
            housing,
            axes="x",
            elem_a="drawer_tray",
            elem_b="cast_shell",
            min_overlap=0.020,
            name="extended drawer remains captured",
        )
        ctx.expect_within(
            selector,
            housing,
            axes="y",
            inner_elem="guide_pin",
            outer_elem="cast_shell",
            margin=0.002,
            name="raised selector remains in guide",
        )
        drawer_extended = ctx.part_world_position(drawer)
        selector_raised = ctx.part_world_position(selector)

    ctx.check(
        "drawer translates outward from front",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] < drawer_rest[0] - 0.050,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )
    ctx.check(
        "selector travels along short vertical guide",
        selector_rest is not None
        and selector_raised is not None
        and selector_raised[2] > selector_rest[2] + 0.020,
        details=f"rest={selector_rest}, raised={selector_raised}",
    )

    return ctx.report()


object_model = build_object_model()
