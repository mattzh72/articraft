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


def _cq_centered_cylinder(radius: float, length: float, axis: str = "z") -> cq.Workplane:
    cyl = cq.Workplane("XY").circle(radius).extrude(length, both=True)
    if axis == "x":
        return cyl.rotate((0, 0, 0), (0, 1, 0), 90)
    if axis == "y":
        return cyl.rotate((0, 0, 0), (1, 0, 0), 90)
    return cyl


def _cq_tube(outer_radius: float, inner_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _binder_ring_shape() -> cq.Workplane:
    height = 0.032
    ring = (
        cq.Workplane("XY")
        .circle(0.024)
        .circle(0.0140)
        .extrude(height, both=True)
    )
    # A real quick-release collar has a saw slot and two clamp ears.
    slot = cq.Workplane("XY").box(0.028, 0.012, height * 1.4).translate((0.019, 0.0, 0.0))
    lug_a = cq.Workplane("XY").box(0.026, 0.008, 0.025).translate((0.028, 0.017, 0.0))
    lug_b = cq.Workplane("XY").box(0.026, 0.008, 0.025).translate((0.028, -0.017, 0.0))
    body = ring.cut(slot).union(lug_a).union(lug_b)
    pivot_hole = _cq_centered_cylinder(0.0033, 0.070, "y").translate((0.036, 0.0, 0.0))
    return body.cut(pivot_hole)


def _quick_lever_shape() -> cq.Workplane:
    barrel = _cq_centered_cylinder(0.0052, 0.008, "y")
    pin = _cq_centered_cylinder(0.0035, 0.046, "y")
    neck = cq.Workplane("XY").box(0.014, 0.008, 0.018).translate((0.007, 0.0, -0.010))
    blade = (
        cq.Workplane("XY")
        .box(0.018, 0.008, 0.078)
        .translate((0.020, 0.0, -0.050))
        .edges("|Y")
        .fillet(0.002)
    )
    cam_lobe = _cq_centered_cylinder(0.007, 0.006, "y").translate((0.004, 0.0, -0.003))
    return barrel.union(pin).union(neck).union(blade).union(cam_lobe)


def _post_head_shape() -> cq.Workplane:
    neck = _cq_centered_cylinder(0.016, 0.030, "z").translate((0.0, 0.0, 0.186))
    cradle = (
        cq.Workplane("XY")
        .box(0.090, 0.060, 0.014)
        .translate((0.0, 0.0, 0.205))
        .edges("|Z")
        .fillet(0.006)
    )
    rail_seat_0 = _cq_centered_cylinder(0.0042, 0.082, "x").translate((0.0, -0.022, 0.212))
    rail_seat_1 = _cq_centered_cylinder(0.0042, 0.082, "x").translate((0.0, 0.022, 0.212))
    return neck.union(cradle).cut(rail_seat_0).cut(rail_seat_1)


def _clamp_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(0.096, 0.060, 0.010)
        .edges("|Z")
        .fillet(0.006)
    )
    for x in (-0.027, 0.027):
        cutter = _cq_centered_cylinder(0.0035, 0.020, "z").translate((x, 0.0, 0.0))
        plate = plate.cut(cutter)
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dropper_seatpost")

    black = model.material("black_anodized", color=(0.015, 0.016, 0.018, 1.0))
    satin = model.material("satin_aluminum", color=(0.72, 0.70, 0.66, 1.0))
    gunmetal = model.material("gunmetal", color=(0.18, 0.19, 0.20, 1.0))
    brass = model.material("bronze_stanchion", color=(0.78, 0.61, 0.36, 1.0))
    rubber = model.material("black_rubber", color=(0.004, 0.004, 0.004, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        mesh_from_cadquery(_cq_tube(0.014, 0.0118, 0.360), "outer_tube", tolerance=0.0005),
        material=black,
        name="outer_tube",
    )
    lower_body.visual(
        mesh_from_cadquery(_cq_tube(0.01185, 0.0103, 0.014), "lower_bushing", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=rubber,
        name="lower_bushing",
    )
    lower_body.visual(
        mesh_from_cadquery(_cq_tube(0.01185, 0.0103, 0.014), "upper_bushing", tolerance=0.0005),
        origin=Origin(xyz=(0.0, 0.0, 0.306)),
        material=rubber,
        name="upper_bushing",
    )

    binder_ring = model.part("binder_ring")
    binder_ring.visual(
        mesh_from_cadquery(_binder_ring_shape(), "binder_ring", tolerance=0.0005),
        material=gunmetal,
        name="split_collar",
    )
    model.articulation(
        "lower_to_binder",
        ArticulationType.FIXED,
        parent=lower_body,
        child=binder_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.344)),
    )

    quick_lever = model.part("quick_lever")
    quick_lever.visual(
        mesh_from_cadquery(_quick_lever_shape(), "quick_lever", tolerance=0.0005),
        material=satin,
        name="cam_lever",
    )
    model.articulation(
        "binder_to_lever",
        ArticulationType.REVOLUTE,
        parent=binder_ring,
        child=quick_lever,
        origin=Origin(xyz=(0.036, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.0, lower=0.0, upper=1.55),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0105, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brass,
        name="stanchion",
    )
    inner_post.visual(
        mesh_from_cadquery(_post_head_shape(), "post_head", tolerance=0.0005),
        material=black,
        name="saddle_cradle",
    )
    model.articulation(
        "lower_to_inner",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.100),
    )

    clamp_plate = model.part("clamp_plate")
    clamp_plate.visual(
        mesh_from_cadquery(_clamp_plate_shape(), "clamp_plate", tolerance=0.0005),
        material=black,
        name="upper_plate",
    )
    model.articulation(
        "inner_to_plate",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
    )

    for idx, y in enumerate((-0.022, 0.022)):
        rail = model.part(f"rail_{idx}")
        rail.visual(
            Cylinder(radius=0.004, length=0.168),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin,
            name="round_rail",
        )
        model.articulation(
            f"inner_to_rail_{idx}",
            ArticulationType.FIXED,
            parent=inner_post,
            child=rail,
            origin=Origin(xyz=(0.0, y, 0.216)),
        )

    for idx, x in enumerate((-0.027, 0.027)):
        bolt = model.part(f"bolt_{idx}")
        bolt.visual(
            Cylinder(radius=0.0027, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, -0.008)),
            material=satin,
            name="bolt_shank",
        )
        bolt.visual(
            Cylinder(radius=0.0062, length=0.005),
            origin=Origin(xyz=(0.0, 0.0, 0.0075)),
            material=satin,
            name="bolt_head",
        )
        bolt.visual(
            Box((0.010, 0.0022, 0.0012)),
            origin=Origin(xyz=(0.0, 0.0, 0.0102)),
            material=gunmetal,
            name="hex_slot",
        )
        model.articulation(
            f"plate_to_bolt_{idx}",
            ArticulationType.CONTINUOUS,
            parent=clamp_plate,
            child=bolt,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_body")
    inner = object_model.get_part("inner_post")
    plate = object_model.get_part("clamp_plate")
    rail_0 = object_model.get_part("rail_0")
    lever = object_model.get_part("quick_lever")
    slide = object_model.get_articulation("lower_to_inner")
    lever_joint = object_model.get_articulation("binder_to_lever")

    ctx.allow_overlap(
        "binder_ring",
        "lower_body",
        elem_a="split_collar",
        elem_b="outer_tube",
        reason="The quick-release collar is modeled as a tightly clamped sleeve gripping the lower body tube.",
    )
    ctx.allow_overlap(
        "binder_ring",
        "quick_lever",
        elem_a="split_collar",
        elem_b="cam_lever",
        reason="The quick-release lever axle is intentionally captured through the collar ear holes.",
    )
    for bushing_name in ("lower_bushing", "upper_bushing"):
        ctx.allow_overlap(
            "lower_body",
            "inner_post",
            elem_a=bushing_name,
            elem_b="stanchion",
            reason="Sliding guide bushings are intentionally shown with slight compression around the telescoping stanchion.",
        )
    for bolt_name in ("bolt_0", "bolt_1"):
        ctx.allow_overlap(
            bolt_name,
            "inner_post",
            elem_a="bolt_shank",
            elem_b="saddle_cradle",
            reason="The saddle-clamp bolt shank is represented as threaded into the lower cradle.",
        )

    ctx.expect_within(
        inner,
        lower,
        axes="xy",
        inner_elem="stanchion",
        outer_elem="outer_tube",
        margin=0.0,
        name="stanchion centered inside lower tube bore",
    )
    ctx.expect_overlap(
        "binder_ring",
        lower,
        axes="z",
        elem_a="split_collar",
        elem_b="outer_tube",
        min_overlap=0.025,
        name="binder collar surrounds upper lower-body tube",
    )
    ctx.expect_overlap(
        "binder_ring",
        lever,
        axes="y",
        elem_a="split_collar",
        elem_b="cam_lever",
        min_overlap=0.030,
        name="quick release axle spans both clamp ears",
    )
    for bushing_name in ("lower_bushing", "upper_bushing"):
        ctx.expect_overlap(
            lower,
            inner,
            axes="z",
            elem_a=bushing_name,
            elem_b="stanchion",
            min_overlap=0.012,
            name=f"{bushing_name} retains stanchion contact",
        )
    for bolt_name in ("bolt_0", "bolt_1"):
        ctx.expect_overlap(
            bolt_name,
            inner,
            axes="z",
            elem_a="bolt_shank",
            elem_b="saddle_cradle",
            min_overlap=0.006,
            name=f"{bolt_name} threads into saddle cradle",
        )
    ctx.expect_overlap(
        inner,
        lower,
        axes="z",
        elem_a="stanchion",
        elem_b="outer_tube",
        min_overlap=0.18,
        name="collapsed post remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({slide: 0.100}):
        ctx.expect_overlap(
            inner,
            lower,
            axes="z",
            elem_a="stanchion",
            elem_b="outer_tube",
            min_overlap=0.10,
            name="extended post keeps retained insertion",
        )
        extended_pos = ctx.part_world_position(inner)
    ctx.check(
        "prismatic dropper extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.095,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        plate,
        rail_0,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0005,
        positive_elem="upper_plate",
        negative_elem="round_rail",
        name="upper clamp plate seats on saddle rail",
    )

    closed_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: 1.20}):
        open_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "quick release lever swings outward",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][0] > closed_aabb[1][0] + 0.030,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
