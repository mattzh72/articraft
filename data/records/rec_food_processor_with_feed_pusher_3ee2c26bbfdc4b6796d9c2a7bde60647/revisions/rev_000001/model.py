from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
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

BASE_W = 0.260
BASE_D = 0.210
BASE_H = 0.180
FRONT_Y = -BASE_D / 2.0

BOWL_X = 0.000
BOWL_Y = 0.028
BOWL_BOTTOM_Z = BASE_H

TUBE_X = 0.020
TUBE_Y = 0.056
TUBE_TOP_Z = 0.305

BUTTON_LAYOUT = (
    (-0.026, 0.096),
    (-0.026, 0.074),
    (-0.026, 0.052),
    (0.008, 0.085),
    (0.008, 0.063),
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _make_base_shape():
    base_shell = _cq_box((BASE_W, BASE_D, BASE_H), (0.0, 0.0, BASE_H / 2.0))
    for x_pos, z_pos in BUTTON_LAYOUT:
        button_well = _cq_box((0.024, 0.034, 0.014), (x_pos, FRONT_Y + 0.013, z_pos))
        base_shell = base_shell.cut(button_well)
    return base_shell


def _make_bowl_shape():
    bowl_outer = cq.Workplane("XY").circle(0.103).extrude(0.150)
    bowl_inner = cq.Workplane("XY").circle(0.0985).extrude(0.146).translate((0.0, 0.0, 0.004))
    bowl_body = bowl_outer.cut(bowl_inner)
    bowl_body = bowl_body.cut(cq.Workplane("XY").circle(0.034).extrude(0.020))

    rim = (
        cq.Workplane("XY")
        .circle(0.106)
        .extrude(0.008)
        .cut(cq.Workplane("XY").circle(0.097).extrude(0.008))
        .translate((0.0, 0.0, 0.142))
    )
    lock_collar = (
        cq.Workplane("XY")
        .circle(0.056)
        .extrude(0.016)
        .cut(cq.Workplane("XY").circle(0.034).extrude(0.016))
    )

    handle_top = _cq_box((0.034, 0.044, 0.018), (0.113, 0.0, 0.136))
    handle_bottom = _cq_box((0.026, 0.038, 0.018), (0.106, 0.0, 0.045))
    handle_spine = _cq_box((0.018, 0.024, 0.100), (0.126, 0.0, 0.090))

    return bowl_body.union(rim).union(lock_collar).union(handle_top).union(handle_bottom).union(handle_spine)


def _make_feed_tube_shape():
    tube_outer = _cq_box((0.086, 0.070, 0.165), (TUBE_X, TUBE_Y, 0.2225))
    lower_flange = _cq_box((0.100, 0.084, 0.016), (TUBE_X, TUBE_Y, 0.142))
    feed_tube = tube_outer.union(lower_flange)
    feed_tube_void = _cq_box((0.072, 0.056, 0.190), (TUBE_X, TUBE_Y, 0.225))
    return feed_tube.cut(feed_tube_void)


def _make_disc_shape():
    disc_plate = cq.Workplane("XY").circle(0.078).extrude(0.004).translate((0.0, 0.0, -0.002))
    center_hub = cq.Workplane("XY").circle(0.020).extrude(0.018).translate((0.0, 0.0, -0.009))
    blade_a = _cq_box((0.066, 0.008, 0.004), (0.016, 0.0, 0.004)).rotate((0, 0, 0), (0, 0, 1), 28)
    blade_b = _cq_box((0.066, 0.008, 0.004), (-0.016, 0.0, 0.004)).rotate((0, 0, 0), (0, 0, 1), 208)
    return disc_plate.union(center_hub).union(blade_a).union(blade_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="food_processor")

    base_white = model.material("base_white", rgba=(0.88, 0.88, 0.84, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.56, 0.58, 0.60, 1.0))
    smoky_clear = model.material("smoky_clear", rgba=(0.73, 0.80, 0.84, 0.34))
    pusher_gray = model.material("pusher_gray", rgba=(0.24, 0.26, 0.28, 1.0))
    metal = model.material("metal", rgba=(0.70, 0.72, 0.74, 1.0))
    knob_black = model.material("knob_black", rgba=(0.12, 0.12, 0.13, 1.0))
    button_gray = model.material("button_gray", rgba=(0.74, 0.76, 0.78, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "base_shell"), material=base_white, name="base_shell")
    base.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(BOWL_X, BOWL_Y, BASE_H + 0.009)),
        material=trim_gray,
        name="drive_collar",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.115),
        origin=Origin(xyz=(BOWL_X, BOWL_Y, BASE_H + 0.0575)),
        material=metal,
        name="drive_shaft",
    )

    bowl = model.part("bowl")
    bowl.visual(mesh_from_cadquery(_make_bowl_shape(), "bowl_body"), material=smoky_clear, name="bowl_body")
    bowl.visual(mesh_from_cadquery(_make_feed_tube_shape(), "feed_tube"), material=smoky_clear, name="feed_tube")

    model.articulation(
        "base_to_bowl",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(BOWL_X, BOWL_Y, BOWL_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Box((0.066, 0.050, 0.186)),
        origin=Origin(xyz=(0.0, 0.0, -0.093)),
        material=pusher_gray,
        name="pusher_shaft",
    )
    pusher.visual(
        Box((0.082, 0.066, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=pusher_gray,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.050, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=pusher_gray,
        name="pusher_ridge",
    )

    model.articulation(
        "bowl_to_pusher",
        ArticulationType.PRISMATIC,
        parent=bowl,
        child=pusher,
        origin=Origin(xyz=(TUBE_X, TUBE_Y, TUBE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.110),
    )

    disc = model.part("disc")
    disc.visual(mesh_from_cadquery(_make_disc_shape(), "slicing_disc"), material=metal, name="disc_body")
    model.articulation(
        "base_to_disc",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=disc,
        origin=Origin(xyz=(BOWL_X, BOWL_Y, BASE_H + 0.056)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=18.0),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.022,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.046, 0.006, flare=0.05),
                grip=KnobGrip(style="fluted", count=16, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "selector_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial_body",
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.068, FRONT_Y, 0.074)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=8.0),
    )

    for index, (x_pos, z_pos) in enumerate(BUTTON_LAYOUT):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.024, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.012, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.016, 0.0)),
            material=button_gray,
            name="button_stem",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, FRONT_Y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.06, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    pusher = object_model.get_part("pusher")
    disc = object_model.get_part("disc")
    button_2 = object_model.get_part("button_2")

    pusher_joint = object_model.get_articulation("bowl_to_pusher")
    button_joint = object_model.get_articulation("base_to_button_2")

    ctx.allow_overlap(
        base,
        disc,
        elem_a="drive_shaft",
        elem_b="disc_body",
        reason="The slicing disc is intentionally represented as riding on the center shaft through its hub.",
    )
    for index in range(5):
        ctx.allow_overlap(
            base,
            f"button_{index}",
            elem_a="base_shell",
            elem_b="button_cap",
            reason="Each push button is intentionally represented seated inside a front-panel button well.",
        )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="bowl_body",
        negative_elem="base_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="bowl seats on the base top",
    )
    ctx.expect_overlap(
        bowl,
        base,
        axes="xy",
        elem_a="bowl_body",
        elem_b="drive_collar",
        min_overlap=0.050,
        name="bowl stays centered over the coupling collar",
    )
    ctx.expect_within(
        pusher,
        bowl,
        axes="xy",
        inner_elem="pusher_shaft",
        outer_elem="feed_tube",
        margin=0.002,
        name="pusher stays inside the feed tube footprint at rest",
    )
    ctx.expect_overlap(
        pusher,
        bowl,
        axes="z",
        elem_a="pusher_shaft",
        elem_b="feed_tube",
        min_overlap=0.120,
        name="pusher remains engaged in the feed tube at rest",
    )
    ctx.expect_within(
        disc,
        bowl,
        axes="xy",
        inner_elem="disc_body",
        outer_elem="bowl_body",
        margin=0.010,
        name="slicing disc stays inside the bowl footprint",
    )

    rest_pusher_pos = ctx.part_world_position(pusher)
    with ctx.pose({pusher_joint: 0.110}):
        ctx.expect_within(
            pusher,
            bowl,
            axes="xy",
            inner_elem="pusher_shaft",
            outer_elem="feed_tube",
            margin=0.002,
            name="pusher stays centered when pressed",
        )
        ctx.expect_overlap(
            pusher,
            bowl,
            axes="z",
            elem_a="pusher_shaft",
            elem_b="feed_tube",
            min_overlap=0.050,
            name="pressed pusher still retains insertion",
        )
        pressed_pusher_pos = ctx.part_world_position(pusher)
    ctx.check(
        "pusher moves downward into the tube",
        rest_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < rest_pusher_pos[2] - 0.08,
        details=f"rest={rest_pusher_pos}, pressed={pressed_pusher_pos}",
    )

    rest_button_pos = ctx.part_world_position(button_2)
    with ctx.pose({button_joint: 0.004}):
        pressed_button_pos = ctx.part_world_position(button_2)
    ctx.check(
        "button pushes inward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[1] > rest_button_pos[1] + 0.003,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
