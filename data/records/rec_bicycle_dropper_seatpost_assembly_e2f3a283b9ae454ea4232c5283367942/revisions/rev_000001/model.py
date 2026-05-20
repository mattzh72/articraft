from __future__ import annotations

import cadquery as cq
import math

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


def _hollow_cylinder_mesh(
    outer_radius: float,
    inner_radius: float,
    length: float,
    name: str,
):
    """Closed tube/ring mesh, authored in meters with its base at local z=0."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.08)


def _rounded_plate_with_holes(name: str):
    plate = (
        cq.Workplane("XY")
        .box(0.090, 0.060, 0.008)
        .edges("|Z")
        .fillet(0.007)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.026, 0.0), (0.026, 0.0)])
        .hole(0.007)
    )
    return mesh_from_cadquery(plate, name, tolerance=0.0006, angular_tolerance=0.08)


def _saddle_pad_mesh(name: str):
    # A tapered racing saddle planform: narrow nose, broad rear, rounded edges.
    outline = [
        (0.155, 0.000),
        (0.130, 0.023),
        (0.045, 0.034),
        (-0.075, 0.065),
        (-0.135, 0.074),
        (-0.158, 0.058),
        (-0.158, -0.058),
        (-0.135, -0.074),
        (-0.075, -0.065),
        (0.045, -0.034),
        (0.130, -0.023),
    ]
    pad = cq.Workplane("XY").polyline(outline).close().extrude(0.026)
    pad = pad.edges("|Z").fillet(0.012)
    pad = pad.edges(">Z").fillet(0.004)
    return mesh_from_cadquery(pad, name, tolerance=0.0008, angular_tolerance=0.10)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_bolt_collar_dropper_seatpost")

    matte_black = model.material("matte_black", rgba=(0.005, 0.006, 0.007, 1.0))
    satin_black = model.material("satin_black", rgba=(0.03, 0.032, 0.035, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.18, 0.18, 0.17, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.60, 0.55, 1.0))
    bolt_dark = model.material("black_oxide", rgba=(0.02, 0.018, 0.016, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        _hollow_cylinder_mesh(0.0160, 0.0142, 0.430, "straight_outer_tube"),
        material=matte_black,
        name="outer_sleeve",
    )
    outer_tube.visual(
        _hollow_cylinder_mesh(0.0270, 0.0158, 0.032, "seat_tube_binder_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=gunmetal,
        name="binder_ring",
    )
    # Split clamp ears and pinch bolt on the binder ring.  They are rigidly part
    # of the seat-tube clamp and visibly clamp the straight sleeve.
    for i, x in enumerate((-0.012, 0.012)):
        outer_tube.visual(
            Box((0.007, 0.028, 0.026)),
            origin=Origin(xyz=(x, -0.028, 0.061)),
            material=gunmetal,
            name=f"binder_ear_{i}",
        )
    outer_tube.visual(
        Cylinder(radius=0.0035, length=0.044),
        origin=Origin(xyz=(0.0, -0.041, 0.061), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="binder_bolt",
    )
    outer_tube.visual(
        Box((0.004, 0.010, 0.034)),
        origin=Origin(xyz=(0.0, -0.017, 0.061)),
        material=bolt_dark,
        name="binder_split",
    )

    lock_collar = model.part("lock_collar")
    lock_collar.visual(
        _hollow_cylinder_mesh(0.0255, 0.0138, 0.035, "locking_collar_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=gunmetal,
        name="collar_ring",
    )
    lock_collar.visual(
        Box((0.006, 0.020, 0.028)),
        origin=Origin(xyz=(-0.010, -0.032, 0.000)),
        material=gunmetal,
        name="pivot_ear_0",
    )
    lock_collar.visual(
        Box((0.006, 0.020, 0.028)),
        origin=Origin(xyz=(0.010, -0.032, 0.000)),
        material=gunmetal,
        name="pivot_ear_1",
    )
    lock_collar.visual(
        Box((0.018, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, -0.020, 0.000)),
        material=bolt_dark,
        name="collar_split",
    )

    model.articulation(
        "outer_to_collar",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=lock_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.4475)),
    )

    release_lever = model.part("release_lever")
    release_lever.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="lever_pivot",
    )
    release_lever.visual(
        Box((0.010, 0.082, 0.010)),
        origin=Origin(xyz=(0.0, -0.046, -0.002)),
        material=satin_black,
        name="lever_blade",
    )
    release_lever.visual(
        Box((0.020, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, -0.092, -0.004)),
        material=dark_rubber,
        name="lever_paddle",
    )
    model.articulation(
        "collar_to_lever",
        ArticulationType.REVOLUTE,
        parent=lock_collar,
        child=release_lever,
        origin=Origin(xyz=(0.0, -0.032, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0, lower=0.0, upper=1.05),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0125, length=0.570),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=satin_black,
        name="inner_shaft",
    )
    inner_post.visual(
        Cylinder(radius=0.0124, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=satin_black,
        name="collar_contact_sleeve",
    )
    inner_post.visual(
        Cylinder(radius=0.0180, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0380)),
        material=gunmetal,
        name="actuator_shoulder",
    )
    inner_post.visual(
        Cylinder(radius=0.0170, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.369)),
        material=gunmetal,
        name="top_head",
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.160),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.0165, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=gunmetal,
        name="post_cap",
    )
    saddle_clamp.visual(
        Box((0.034, 0.038, 0.036)),
        origin=Origin(xyz=(0.012, 0.0, 0.030)),
        material=gunmetal,
        name="clamp_yoke",
    )
    saddle_clamp.visual(
        Box((0.088, 0.034, 0.008)),
        origin=Origin(xyz=(0.016, 0.0, 0.050)),
        material=gunmetal,
        name="upper_bridge",
    )
    saddle_clamp.visual(
        Box((0.076, 0.010, 0.006)),
        origin=Origin(xyz=(0.016, -0.022, 0.044)),
        material=gunmetal,
        name="rail_cradle_0",
    )
    saddle_clamp.visual(
        Box((0.076, 0.010, 0.006)),
        origin=Origin(xyz=(0.016, 0.022, 0.044)),
        material=gunmetal,
        name="rail_cradle_1",
    )
    saddle_clamp.visual(
        _rounded_plate_with_holes("two_bolt_clamp_plate"),
        origin=Origin(xyz=(0.016, 0.0, 0.058)),
        material=gunmetal,
        name="clamp_plate",
    )
    model.articulation(
        "post_to_clamp",
        ArticulationType.FIXED,
        parent=inner_post,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.378)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        _saddle_pad_mesh("narrow_black_saddle"),
        origin=Origin(xyz=(0.020, 0.0, 0.092)),
        material=dark_rubber,
        name="saddle_pad",
    )
    saddle.visual(
        Box((0.145, 0.013, 0.002)),
        origin=Origin(xyz=(0.020, 0.0, 0.119)),
        material=matte_black,
        name="relief_channel",
    )
    for i, y in enumerate((-0.022, 0.022)):
        saddle.visual(
            Cylinder(radius=0.0030, length=0.170),
            origin=Origin(xyz=(0.016, y, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
            name=f"saddle_rail_{i}",
        )
        for j, x in enumerate((-0.068, 0.092)):
            saddle.visual(
                Cylinder(radius=0.0023, length=0.040),
                origin=Origin(xyz=(x, y, 0.073)),
                material=brushed_steel,
                name=f"rail_strut_{i}_{j}",
            )
    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent=saddle_clamp,
        child=saddle,
        origin=Origin(),
    )

    for idx, x in enumerate((-0.010, 0.042)):
        bolt = model.part(f"clamp_bolt_{idx}")
        bolt.visual(
            Cylinder(radius=0.0022, length=0.032),
            origin=Origin(xyz=(0.0, 0.0, -0.018)),
            material=brushed_steel,
            name="bolt_shank",
        )
        bolt.visual(
            Cylinder(radius=0.0065, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.001)),
            material=brushed_steel,
            name="bolt_head",
        )
        bolt.visual(
            Box((0.010, 0.002, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0020)),
            material=bolt_dark,
            name="hex_slot",
        )
        model.articulation(
            f"clamp_to_bolt_{idx}",
            ArticulationType.REVOLUTE,
            parent=saddle_clamp,
            child=bolt,
            origin=Origin(xyz=(x, 0.0, 0.066)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0, lower=-math.pi, upper=math.pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    inner = object_model.get_part("inner_post")
    collar = object_model.get_part("lock_collar")
    lever = object_model.get_part("release_lever")
    clamp = object_model.get_part("saddle_clamp")
    saddle = object_model.get_part("saddle")
    slide = object_model.get_articulation("outer_to_inner")
    lever_joint = object_model.get_articulation("collar_to_lever")

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="inner_shaft",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="inner shaft stays concentric in outer sleeve",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="inner_shaft",
        elem_b="outer_sleeve",
        min_overlap=0.150,
        name="collapsed post has deep insertion",
    )
    ctx.expect_contact(
        collar,
        outer,
        elem_a="collar_ring",
        elem_b="outer_sleeve",
        contact_tol=0.002,
        name="locking collar is seated on outer tube",
    )
    ctx.expect_contact(
        saddle,
        clamp,
        elem_a="saddle_rail_0",
        elem_b="rail_cradle_0",
        contact_tol=0.002,
        name="first saddle rail sits in cradle",
    )
    ctx.expect_contact(
        saddle,
        clamp,
        elem_a="saddle_rail_1",
        elem_b="rail_cradle_1",
        contact_tol=0.002,
        name="second saddle rail sits in cradle",
    )

    rest_saddle = ctx.part_world_aabb(saddle)
    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({slide: 0.160, lever_joint: 1.05}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="inner_shaft",
            outer_elem="outer_sleeve",
            margin=0.002,
            name="extended shaft remains concentric",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="inner_shaft",
            elem_b="outer_sleeve",
            min_overlap=0.045,
            name="extended post retains insertion",
        )
        raised_saddle = ctx.part_world_aabb(saddle)
        lowered_lever = ctx.part_world_aabb(lever)

    ctx.check(
        "prismatic joint raises saddle",
        rest_saddle is not None
        and raised_saddle is not None
        and raised_saddle[1][2] > rest_saddle[1][2] + 0.150,
        details=f"rest={rest_saddle}, raised={raised_saddle}",
    )
    ctx.check(
        "lever rotates downward to release collar",
        rest_lever is not None
        and lowered_lever is not None
        and lowered_lever[0][2] < rest_lever[0][2] - 0.050,
        details=f"rest={rest_lever}, lowered={lowered_lever}",
    )

    for idx in (0, 1):
        bolt_joint = object_model.get_articulation(f"clamp_to_bolt_{idx}")
        ctx.check(
            f"clamp bolt {idx} is independently turnable",
            bolt_joint.motion_limits is not None
            and bolt_joint.motion_limits.lower <= -math.pi
            and bolt_joint.motion_limits.upper >= math.pi,
        )

    return ctx.report()


object_model = build_object_model()
