from __future__ import annotations

from math import pi

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


def _x_cylinder(radius: float, length: float, center_x: float) -> cq.Workplane:
    """CadQuery cylinder aligned with the syringe barrel's local X axis."""
    return (
        cq.Workplane("XY")
        .cylinder(length, radius)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
        .translate((center_x, 0.0, 0.0))
    )


def _barrel_frame_shape() -> cq.Workplane:
    barrel_len = 0.120
    outer_r = 0.019
    inner_r = 0.0163

    tube = _x_cylinder(outer_r, barrel_len, barrel_len / 2.0).cut(
        _x_cylinder(inner_r, barrel_len + 0.008, barrel_len / 2.0)
    )

    rear_ring = _x_cylinder(0.023, 0.012, -0.004).cut(
        _x_cylinder(inner_r + 0.001, 0.018, -0.004)
    )

    finger_flange = (
        cq.Workplane("XY")
        .box(0.009, 0.095, 0.013)
        .edges("|X")
        .fillet(0.004)
        .translate((-0.006, 0.0, 0.0))
        .cut(_x_cylinder(inner_r + 0.001, 0.020, -0.006))
    )

    front_collared_end = _x_cylinder(0.017, 0.012, 0.123)
    nozzle_neck = _x_cylinder(0.008, 0.025, 0.1413)
    nozzle_tip = _x_cylinder(0.0042, 0.020, 0.1635)
    nozzle_bore = _x_cylinder(0.0020, 0.060, 0.150)

    return (
        tube.union(rear_ring)
        .union(finger_flange)
        .union(front_collared_end)
        .union(nozzle_neck)
        .union(nozzle_tip)
        .cut(nozzle_bore)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hand_syringe")

    clear_polycarbonate = model.material(
        "clear_polycarbonate", rgba=(0.72, 0.90, 1.0, 0.34)
    )
    ink = model.material("blue_graduation_ink", rgba=(0.05, 0.18, 0.55, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    plunger_plastic = model.material("white_plunger_plastic", rgba=(0.88, 0.91, 0.94, 1.0))

    barrel = model.part("barrel")
    barrel.visual(
        mesh_from_cadquery(_barrel_frame_shape(), "barrel_frame", tolerance=0.0006),
        material=clear_polycarbonate,
        name="barrel_frame",
    )

    for i, x in enumerate((0.026, 0.038, 0.050, 0.062, 0.074, 0.086, 0.098)):
        major = i % 2 == 0
        barrel.visual(
            Box((0.0012, 0.022 if major else 0.014, 0.0010)),
            origin=Origin(xyz=(x, 0.0, 0.0192)),
            material=ink,
            name=f"graduation_{i}",
        )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.0034, length=0.140),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="rod",
    )
    plunger.visual(
        Cylinder(radius=0.01635, length=0.018),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="plunger_head",
    )
    plunger.visual(
        Cylinder(radius=0.0065, length=0.012),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="head_stem",
    )
    plunger.visual(
        Cylinder(radius=0.0175, length=0.008),
        origin=Origin(xyz=(-0.089, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=plunger_plastic,
        name="thumb_pad",
    )

    model.articulation(
        "barrel_to_plunger",
        ArticulationType.PRISMATIC,
        parent=barrel,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.18, lower=0.0, upper=0.035),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    plunger = object_model.get_part("plunger")
    slide = object_model.get_articulation("barrel_to_plunger")

    ctx.allow_overlap(
        barrel,
        plunger,
        elem_a="barrel_frame",
        elem_b="plunger_head",
        reason=(
            "The rubber plunger head is intentionally modeled with a tiny "
            "radial compression seal against the transparent barrel bore."
        ),
    )

    ctx.check(
        "plunger uses short compact travel",
        slide.motion_limits is not None and 0.02 <= slide.motion_limits.upper <= 0.04,
        details=f"limits={slide.motion_limits}",
    )
    ctx.expect_within(
        plunger,
        barrel,
        axes="yz",
        inner_elem="plunger_head",
        outer_elem="barrel_frame",
        margin=0.002,
        name="rubber head is centered in barrel bore",
    )
    ctx.expect_overlap(
        plunger,
        barrel,
        axes="x",
        elem_a="plunger_head",
        elem_b="barrel_frame",
        min_overlap=0.015,
        name="rubber head sits inside barrel length",
    )

    rest_pos = ctx.part_world_position(plunger)
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            plunger,
            barrel,
            axes="yz",
            inner_elem="plunger_head",
            outer_elem="barrel_frame",
            margin=0.002,
            name="advanced rubber head stays centered",
        )
        ctx.expect_overlap(
            plunger,
            barrel,
            axes="x",
            elem_a="plunger_head",
            elem_b="barrel_frame",
            min_overlap=0.015,
            name="advanced rubber head remains retained",
        )
        advanced_pos = ctx.part_world_position(plunger)

    ctx.check(
        "positive travel presses plunger toward nozzle",
        rest_pos is not None
        and advanced_pos is not None
        and advanced_pos[0] > rest_pos[0] + 0.03,
        details=f"rest={rest_pos}, advanced={advanced_pos}",
    )

    return ctx.report()


object_model = build_object_model()
