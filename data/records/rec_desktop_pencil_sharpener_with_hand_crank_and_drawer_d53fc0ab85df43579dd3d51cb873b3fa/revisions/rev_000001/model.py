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


def _housing_shape() -> cq.Workplane:
    body_length = 0.152
    body_width = 0.092
    body_height = 0.090
    housing_bottom = 0.011

    housing = (
        cq.Workplane("XY")
        .box(body_length, body_width, body_height)
        .translate((0.0, 0.0, housing_bottom + body_height / 2.0))
        .edges("|Z")
        .fillet(0.010)
    )

    cavity = cq.Workplane("XY").box(0.126, 0.066, 0.064).translate((-0.006, 0.0, 0.050))
    drawer_opening = cq.Workplane("XY").box(0.026, 0.064, 0.042).translate((0.063, 0.0, 0.036))
    feed_bore = cq.Workplane("YZ").circle(0.0048).extrude(0.046).translate((0.030, 0.0, 0.074))
    feed_counterbore = cq.Workplane("YZ").circle(0.0078).extrude(0.010).translate((0.066, 0.0, 0.074))
    shaft_hole = cq.Workplane("XZ").circle(0.0058).extrude(0.030).translate((0.000, 0.028, 0.061))

    return housing.cut(cavity).cut(drawer_opening).cut(feed_bore).cut(feed_counterbore).cut(shaft_hole)


def _cup_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.055)
        .extrude(0.012)
        .cut(cq.Workplane("XY").circle(0.040).extrude(0.010).translate((0.0, 0.0, 0.001)))
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="suction_base_pencil_sharpener")

    housing_red = model.material("housing_red", rgba=(0.57, 0.10, 0.10, 1.0))
    drawer_dark = model.material("drawer_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    metal = model.material("metal", rgba=(0.72, 0.73, 0.76, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))
    lever_black = model.material("lever_black", rgba=(0.08, 0.08, 0.08, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_housing_shape(), "sharpener_housing"),
        material=housing_red,
        name="housing",
    )
    body.visual(
        mesh_from_cadquery(_cup_shape(), "sharpener_cup"),
        material=rubber_black,
        name="cup",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.0, 0.050, 0.061), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="mount_boss",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((0.089, 0.056, 0.003)),
        origin=Origin(xyz=(-0.0435, 0.0, -0.0135)),
        material=drawer_dark,
        name="tray",
    )
    drawer.visual(
        Box((0.089, 0.003, 0.028)),
        origin=Origin(xyz=(-0.0435, 0.0275, -0.0005)),
        material=drawer_dark,
        name="wall_0",
    )
    drawer.visual(
        Box((0.089, 0.003, 0.028)),
        origin=Origin(xyz=(-0.0435, -0.0275, -0.0005)),
        material=drawer_dark,
        name="wall_1",
    )
    drawer.visual(
        Box((0.003, 0.056, 0.028)),
        origin=Origin(xyz=(-0.0875, 0.0, -0.0005)),
        material=drawer_dark,
        name="back",
    )
    drawer.visual(
        Box((0.008, 0.066, 0.042)),
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        material=drawer_dark,
        name="front_panel",
    )
    drawer.visual(
        Box((0.010, 0.026, 0.010)),
        origin=Origin(xyz=(0.009, 0.0, -0.006)),
        material=metal,
        name="pull",
    )

    crank = model.part("crank")
    crank.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="shaft",
    )
    crank.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="hub",
    )
    crank.visual(
        Cylinder(radius=0.004, length=0.036),
        origin=Origin(xyz=(0.018, 0.014, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="arm",
    )
    crank.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.036, 0.014, -0.012)),
        material=metal,
        name="drop",
    )
    crank.visual(
        Cylinder(radius=0.0065, length=0.020),
        origin=Origin(xyz=(0.036, 0.014, -0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="grip",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.0045, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lever_black,
        name="barrel",
    )
    lever.visual(
        Box((0.020, 0.024, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, -0.004)),
        material=lever_black,
        name="arm",
    )
    lever.visual(
        Box((0.010, 0.024, 0.010)),
        origin=Origin(xyz=(0.023, 0.0, -0.009)),
        material=lever_black,
        name="toe",
    )

    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.076, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.050, effort=40.0, velocity=0.20),
    )
    model.articulation(
        "body_to_crank",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=crank,
        origin=Origin(xyz=(0.0, 0.055, 0.061)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.008, 0.0, 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.80, effort=6.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    crank = object_model.get_part("crank")
    lever = object_model.get_part("lever")

    drawer_slide = object_model.get_articulation("body_to_drawer")
    crank_spin = object_model.get_articulation("body_to_crank")
    lever_pivot = object_model.get_articulation("body_to_lever")

    drawer_limits = drawer_slide.motion_limits
    lever_limits = lever_pivot.motion_limits

    ctx.expect_within(
        drawer,
        body,
        axes="yz",
        inner_elem="tray",
        margin=0.004,
        name="drawer tray stays centered in the housing",
    )
    ctx.expect_gap(
        drawer,
        body,
        axis="x",
        positive_elem="front_panel",
        max_gap=0.002,
        max_penetration=0.0,
        name="drawer front seats against the housing",
    )

    drawer_rest = ctx.part_world_position(drawer)
    if drawer_limits is not None and drawer_limits.upper is not None:
        with ctx.pose({drawer_slide: drawer_limits.upper}):
            ctx.expect_within(
                drawer,
                body,
                axes="yz",
                inner_elem="tray",
                margin=0.004,
                name="drawer tray stays guided when open",
            )
            ctx.expect_overlap(
                drawer,
                body,
                axes="x",
                elem_a="tray",
                min_overlap=0.030,
                name="drawer remains retained at full extension",
            )
            drawer_open = ctx.part_world_position(drawer)
        ctx.check(
            "drawer slides forward",
            drawer_rest is not None and drawer_open is not None and drawer_open[0] > drawer_rest[0] + 0.040,
            details=f"rest={drawer_rest}, open={drawer_open}",
        )

    grip_rest = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    with ctx.pose({crank_spin: math.pi / 2.0}):
        grip_quarter = _aabb_center(ctx.part_element_world_aabb(crank, elem="grip"))
    ctx.check(
        "crank grip orbits around the shaft",
        grip_rest is not None
        and grip_quarter is not None
        and (
            abs(grip_quarter[0] - grip_rest[0]) > 0.020
            or abs(grip_quarter[2] - grip_rest[2]) > 0.020
        ),
        details=f"rest={grip_rest}, quarter_turn={grip_quarter}",
    )

    if lever_limits is not None and lever_limits.upper is not None:
        toe_rest = _aabb_center(ctx.part_element_world_aabb(lever, elem="toe"))
        with ctx.pose({lever_pivot: lever_limits.upper}):
            toe_locked = _aabb_center(ctx.part_element_world_aabb(lever, elem="toe"))
        ctx.check(
            "lock lever pivots beneath the base",
            toe_rest is not None
            and toe_locked is not None
            and toe_locked[2] < toe_rest[2] - 0.008
            and abs(toe_locked[0] - toe_rest[0]) > 0.004,
            details=f"rest={toe_rest}, locked={toe_locked}",
        )

    return ctx.report()


object_model = build_object_model()
