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


def _cylinder_between(start: tuple[float, float, float], end: tuple[float, float, float], radius: float):
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    origin = Origin(
        xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
        rpy=(0.0, pitch, yaw),
    )
    return Cylinder(radius=radius, length=length), origin


def _tube_shell(outer_radius: float, inner_radius: float, length: float, z0: float):
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(length + 0.004).translate((0.0, 0.0, -0.002))
    return outer.cut(inner).translate((0.0, 0.0, z0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="school_orchestra_stand")

    black = model.material("satin_black", rgba=(0.01, 0.01, 0.012, 1.0))
    dark = model.material("dark_sheet_metal", rgba=(0.035, 0.038, 0.04, 1.0))
    silver = model.material("brushed_chrome", rgba=(0.66, 0.68, 0.66, 1.0))
    rubber = model.material("black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))

    base = model.part("tripod_base")
    base.visual(
        mesh_from_cadquery(_tube_shell(0.030, 0.0205, 0.50, 0.22), "hub_tube"),
        material=black,
        name="hub_tube",
    )
    base.visual(
        Cylinder(radius=0.055, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=black,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.200)),
        material=black,
        name="lower_collar",
    )
    base.visual(
        Box((0.120, 0.032, 0.040)),
        origin=Origin(xyz=(0.000, -0.040, 0.545)),
        material=black,
        name="height_clamp",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, -0.073, 0.545), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="clamp_knob",
    )

    for i, angle in enumerate((math.radians(90.0), math.radians(210.0), math.radians(330.0))):
        root = (0.040 * math.cos(angle), 0.040 * math.sin(angle), 0.235)
        foot = (0.440 * math.cos(angle), 0.440 * math.sin(angle), 0.026)
        leg_geom, leg_origin = _cylinder_between(root, foot, 0.012)
        base.visual(leg_geom, origin=leg_origin, material=black, name=f"leg_{i}")
        base.visual(
            Cylinder(radius=0.045, length=0.018),
            origin=Origin(xyz=(foot[0], foot[1], 0.009)),
            material=rubber,
            name=f"foot_{i}",
        )

    mast = model.part("inner_mast")
    mast.visual(
        Cylinder(radius=0.0155, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=silver,
        name="inner_tube",
    )
    mast.visual(
        Cylinder(radius=0.018, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.448)),
        material=black,
        name="top_plug",
    )
    mast.visual(
        Cylinder(radius=0.021, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.280)),
        material=black,
        name="guide_bushing",
    )

    head = model.part("tilt_head")
    head.visual(
        Cylinder(radius=0.026, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black,
        name="mast_collar",
    )
    head.visual(
        Cylinder(radius=0.016, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=black,
        name="head_stem",
    )
    head.visual(
        Box((0.170, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=black,
        name="rear_bridge",
    )
    head.visual(
        Box((0.018, 0.022, 0.090)),
        origin=Origin(xyz=(0.075, 0.0, 0.135)),
        material=black,
        name="yoke_plate_0",
    )
    head.visual(
        Box((0.018, 0.022, 0.090)),
        origin=Origin(xyz=(-0.075, 0.0, 0.135)),
        material=black,
        name="yoke_plate_1",
    )
    head.visual(
        Cylinder(radius=0.012, length=0.650),
        origin=Origin(xyz=(0.0, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=silver,
        name="hinge_pin",
    )
    head.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.333, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="tilt_knob_0",
    )
    head.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(-0.333, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="tilt_knob_1",
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.580, 0.012, 0.360)),
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        material=dark,
        name="panel_shell",
    )
    desk.visual(
        Box((0.610, 0.022, 0.026)),
        origin=Origin(xyz=(0.0, -0.030, 0.185)),
        material=black,
        name="top_rim",
    )
    desk.visual(
        Box((0.610, 0.075, 0.030)),
        origin=Origin(xyz=(0.0, -0.060, -0.192)),
        material=black,
        name="bottom_ledge",
    )
    desk.visual(
        Box((0.022, 0.018, 0.370)),
        origin=Origin(xyz=(0.301, -0.030, 0.0)),
        material=black,
        name="side_rim_0",
    )
    desk.visual(
        Box((0.022, 0.018, 0.370)),
        origin=Origin(xyz=(-0.301, -0.030, 0.0)),
        material=black,
        name="side_rim_1",
    )
    desk.visual(
        Box((0.105, 0.018, 0.036)),
        origin=Origin(xyz=(0.0, -0.023, 0.0)),
        material=black,
        name="hinge_strap",
    )
    desk.visual(
        Cylinder(radius=0.018, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="hinge_socket",
    )

    for i, x in enumerate((-0.185, 0.185)):
        clip = model.part(f"clip_{i}")
        clip.visual(
            Cylinder(radius=0.009, length=0.030),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=silver,
            name="pivot_pin",
        )
        clip.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name="pivot_cap",
        )
        clip.visual(
            Box((0.036, 0.006, 0.090)),
            origin=Origin(xyz=(0.0, -0.015, -0.047)),
            material=black,
            name="spring_tab",
        )
        clip.visual(
            Box((0.050, 0.007, 0.016)),
            origin=Origin(xyz=(0.0, -0.018, -0.090)),
            material=black,
            name="finger_pad",
        )

        model.articulation(
            f"desk_to_clip_{i}",
            ArticulationType.REVOLUTE,
            parent=desk,
            child=clip,
            origin=Origin(xyz=(x, -0.030, 0.195)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=3.0, lower=-0.95, upper=0.95),
        )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.720)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.25, lower=0.0, upper=0.22),
    )
    model.articulation(
        "head_mount",
        ArticulationType.FIXED,
        parent=mast,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
    )
    model.articulation(
        "desk_tilt",
        ArticulationType.REVOLUTE,
        parent=head,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.8, lower=-0.80, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("tripod_base")
    mast = object_model.get_part("inner_mast")
    head = object_model.get_part("tilt_head")
    desk = object_model.get_part("desk")
    mast_slide = object_model.get_articulation("mast_slide")
    desk_tilt = object_model.get_articulation("desk_tilt")

    ctx.allow_overlap(
        head,
        desk,
        elem_a="hinge_pin",
        elem_b="hinge_socket",
        reason="The visible tilt-head pin is intentionally captured by the desk hinge socket.",
    )
    ctx.allow_overlap(
        base,
        mast,
        elem_a="hub_tube",
        elem_b="guide_bushing",
        reason="A small guide bushing lightly seats the sliding mast against the inside of the hub tube.",
    )
    for i in (0, 1):
        ctx.allow_overlap(
            f"clip_{i}",
            desk,
            elem_a="pivot_pin",
            elem_b="top_rim",
            reason="Each page clip has a short pivot pin passing through the desk top rim.",
        )

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="hub_tube",
        margin=0.002,
        name="mast centered in hub tube",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="hub_tube",
        min_overlap=0.30,
        name="mast retained in hub tube when lowered",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="guide_bushing",
        elem_b="hub_tube",
        min_overlap=0.035,
        name="guide bushing remains inside hub tube",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 0.22}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="hub_tube",
            margin=0.002,
            name="raised mast remains centered in hub tube",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="hub_tube",
            min_overlap=0.12,
            name="raised mast keeps retained insertion",
        )
        raised_mast_pos = ctx.part_world_position(mast)

    ctx.check(
        "mast slide moves desk upward",
        rest_mast_pos is not None
        and raised_mast_pos is not None
        and raised_mast_pos[2] > rest_mast_pos[2] + 0.20,
        details=f"rest={rest_mast_pos}, raised={raised_mast_pos}",
    )

    ctx.expect_overlap(
        head,
        desk,
        axes="x",
        elem_a="hinge_pin",
        elem_b="hinge_socket",
        min_overlap=0.08,
        name="desk hinge socket captures tilt pin",
    )
    for i in (0, 1):
        ctx.expect_overlap(
            f"clip_{i}",
            desk,
            axes="yz",
            elem_a="pivot_pin",
            elem_b="top_rim",
            min_overlap=0.006,
            name=f"clip {i} pivot passes through top rim",
        )

    top_rest = ctx.part_element_world_aabb(desk, elem="top_rim")
    with ctx.pose({desk_tilt: -0.55}):
        top_tilted = ctx.part_element_world_aabb(desk, elem="top_rim")
    if top_rest is not None and top_tilted is not None:
        rest_y = (top_rest[0][1] + top_rest[1][1]) * 0.5
        tilted_y = (top_tilted[0][1] + top_tilted[1][1]) * 0.5
        ctx.check(
            "desk tilt leans top rearward",
            tilted_y > rest_y + 0.05,
            details=f"rest_y={rest_y:.3f}, tilted_y={tilted_y:.3f}",
        )
    else:
        ctx.fail("desk tilt leans top rearward", "top rim AABB unavailable")

    return ctx.report()


object_model = build_object_model()
