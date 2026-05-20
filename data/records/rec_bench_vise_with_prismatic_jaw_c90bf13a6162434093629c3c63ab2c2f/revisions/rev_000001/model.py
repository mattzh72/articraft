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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _v_block_body(width: float, depth: float, height: float) -> object:
    """Rectangular jaw block with a real V-groove cut along its width."""
    groove_width = depth * 0.70
    groove_depth = height * 0.34
    body = cq.Workplane("XY").box(width, depth, height)
    cutter = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-groove_width / 2.0, height / 2.0 + 0.006),
                (0.0, height / 2.0 - groove_depth),
                (groove_width / 2.0, height / 2.0 + 0.006),
            ]
        )
        .close()
        .extrude(width + 0.020, both=True)
    )
    return body.cut(cutter)


def _centered_tube(outer_radius: float, inner_radius: float, length: float) -> object:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )


def _hex_ring(outer_radius: float, inner_radius: float, length: float) -> object:
    points = [
        (
            outer_radius * math.cos(math.pi / 6.0 + i * math.tau / 6.0),
            outer_radius * math.sin(math.pi / 6.0 + i * math.tau / 6.0),
        )
        for i in range(6)
    ]
    body = (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    bore = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.010)
        .translate((0.0, 0.0, -(length + 0.010) / 2.0))
    )
    return body.cut(bore)


def _helix(radius: float, radius_eps: float, pitch: float, height: float, d: float = 0.0):
    turns = height / pitch

    def func(t: float) -> tuple[float, float, float]:
        z = height * t
        r = radius + radius_eps
        angle = math.tau * turns * t
        if t < 0.04:
            r = radius + radius_eps * math.sin(math.pi * t / 0.08)
        elif t > 0.96:
            r = radius + radius_eps * math.sin(math.pi * (1.0 - t) / 0.08)
        return (r * math.cos(angle), r * math.sin(angle), z + d)

    return func


def _threaded_shaft(core_radius: float, thread_radius: float, pitch: float, length: float) -> object:
    """A compact helical ridge on a cylindrical leadscrew core."""
    flank = pitch * 0.18
    core = cq.Workplane("XY").circle(core_radius).extrude(length)
    root_a = cq.Workplane("XY").parametricCurve(_helix(core_radius * 0.98, 0.0, pitch, length, -flank)).val()
    root_b = cq.Workplane("XY").parametricCurve(_helix(core_radius * 0.98, 0.0, pitch, length, flank)).val()
    crest_a = cq.Workplane("XY").parametricCurve(_helix(thread_radius, 0.0, pitch, length, -flank * 0.25)).val()
    crest_b = cq.Workplane("XY").parametricCurve(_helix(thread_radius, 0.0, pitch, length, flank * 0.25)).val()
    faces = [
        cq.Face.makeRuledSurface(root_a, root_b),
        cq.Face.makeRuledSurface(root_a, crest_a),
        cq.Face.makeRuledSurface(root_b, crest_b),
        cq.Face.makeRuledSurface(crest_a, crest_b),
    ]
    thread = cq.Solid.makeSolid(cq.Shell.makeShell(faces))
    return core.union(thread)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cross_drilling_vise")

    cast_iron = model.material("blue_gray_cast_iron", rgba=(0.22, 0.29, 0.34, 1.0))
    dark_iron = model.material("dark_burnished_steel", rgba=(0.06, 0.065, 0.07, 1.0))
    polished = model.material("polished_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    black = model.material("black_recess", rgba=(0.005, 0.005, 0.006, 1.0))
    handle_red = model.material("red_handle_knobs", rgba=(0.65, 0.05, 0.035, 1.0))

    base = model.part("base_plate")
    base.visual(
        Box((0.66, 0.48, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=cast_iron,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(_centered_tube(0.205, 0.188, 0.002), "swivel_shadow_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=dark_iron,
        name="swivel_shadow_ring",
    )
    for idx, (x, y) in enumerate(((-0.24, -0.16), (0.24, -0.16), (-0.24, 0.16), (0.24, 0.16))):
        base.visual(
            Box((0.105, 0.030, 0.002)),
            origin=Origin(xyz=(x, y, 0.036)),
            material=black,
            name=f"mount_slot_{idx}",
        )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.185, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cast_iron,
        name="turntable_disk",
    )
    turntable.visual(
        Box((0.48, 0.40, 0.032)),
        origin=Origin(xyz=(0.0, -0.040, 0.041)),
        material=cast_iron,
        name="rectangular_bed",
    )
    turntable.visual(
        Cylinder(radius=0.0105, length=0.365),
        origin=Origin(xyz=(-0.115, -0.040, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="guide_bar_0",
    )
    turntable.visual(
        Cylinder(radius=0.0105, length=0.365),
        origin=Origin(xyz=(0.115, -0.040, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="guide_bar_1",
    )
    for idx, x in enumerate((-0.115, 0.115)):
        turntable.visual(
            Box((0.054, 0.036, 0.052)),
            origin=Origin(xyz=(x, -0.220, 0.0825)),
            material=cast_iron,
            name=f"front_bar_lug_{idx}",
        )
    turntable.visual(
        mesh_from_cadquery(_centered_tube(0.024, 0.0105, 0.038), "front_screw_bearing"),
        origin=Origin(xyz=(0.0, -0.215, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="front_screw_bearing",
    )
    turntable.visual(
        Box((0.060, 0.036, 0.018)),
        origin=Origin(xyz=(0.0, -0.215, 0.064)),
        material=cast_iron,
        name="bearing_pedestal",
    )
    turntable.visual(
        Box((0.335, 0.058, 0.018)),
        origin=Origin(xyz=(0.0, 0.105, 0.128)),
        material=cast_iron,
        name="rear_saddle",
    )
    for idx, x in enumerate((-0.160, 0.160)):
        turntable.visual(
            Box((0.038, 0.052, 0.063)),
            origin=Origin(xyz=(x, 0.105, 0.0875)),
            material=cast_iron,
            name=f"rear_support_{idx}",
        )
    turntable.visual(
        mesh_from_cadquery(_v_block_body(0.335, 0.058, 0.090), "fixed_v_block"),
        origin=Origin(xyz=(0.0, 0.105, 0.180)),
        material=cast_iron,
        name="fixed_v_block",
    )
    turntable.visual(
        Box((0.330, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.072, 0.180)),
        material=dark_iron,
        name="fixed_jaw_face",
    )

    moving = model.part("moving_jaw")
    moving.visual(
        mesh_from_cadquery(_v_block_body(0.335, 0.058, 0.090), "moving_v_block"),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=cast_iron,
        name="moving_v_block",
    )
    moving.visual(
        Box((0.335, 0.058, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.126)),
        material=cast_iron,
        name="moving_saddle",
    )
    moving.visual(
        Box((0.330, 0.010, 0.030)),
        origin=Origin(xyz=(0.0, 0.033, 0.180)),
        material=dark_iron,
        name="moving_jaw_face",
    )
    for idx, x in enumerate((-0.115, 0.115)):
        moving.visual(
            mesh_from_cadquery(_centered_tube(0.024, 0.0145, 0.058), f"guide_bushing_{idx}"),
            origin=Origin(xyz=(x, 0.0, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_iron,
            name=f"guide_bushing_{idx}",
        )
        moving.visual(
            Box((0.012, 0.050, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.0815)),
            material=dark_iron,
            name=f"guide_wear_pad_{idx}",
        )
    moving.visual(
        mesh_from_cadquery(_hex_ring(0.031, 0.0165, 0.040), "lead_nut"),
        origin=Origin(xyz=(0.0, 0.0, 0.095), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="lead_nut",
    )

    leadscrew = model.part("leadscrew")
    leadscrew.visual(
        mesh_from_cadquery(_threaded_shaft(0.0105, 0.0130, 0.026, 0.355), "threaded_shaft", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="threaded_shaft",
    )
    for idx, y in enumerate(0.065 + i * 0.024 for i in range(11)):
        leadscrew.visual(
            Cylinder(radius=0.0128, length=0.0022),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=polished,
            name=f"thread_ridge_{idx}",
        )
    leadscrew.visual(
        mesh_from_cadquery(_hex_ring(0.021, 0.0001, 0.038), "hex_drive_hub"),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="hex_drive_hub",
    )
    leadscrew.visual(
        Cylinder(radius=0.0075, length=0.185),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="t_bar",
    )
    leadscrew.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(-0.097, -0.012, 0.0)),
        material=handle_red,
        name="handle_knob_0",
    )
    leadscrew.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.097, -0.012, 0.0)),
        material=handle_red,
        name="handle_knob_1",
    )

    model.articulation(
        "base_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.0),
    )
    model.articulation(
        "turntable_to_moving_jaw",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=moving,
        origin=Origin(xyz=(0.0, -0.045, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.06, lower=0.0, upper=0.085),
    )
    model.articulation(
        "turntable_to_leadscrew",
        ArticulationType.CONTINUOUS,
        parent=turntable,
        child=leadscrew,
        origin=Origin(xyz=(0.0, -0.255, 0.095)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    turntable = object_model.get_part("turntable")
    moving = object_model.get_part("moving_jaw")
    leadscrew = object_model.get_part("leadscrew")
    swivel = object_model.get_articulation("base_to_turntable")
    slide = object_model.get_articulation("turntable_to_moving_jaw")

    ctx.allow_overlap(
        leadscrew,
        turntable,
        elem_a="threaded_shaft",
        elem_b="front_screw_bearing",
        reason="The rotating leadscrew is intentionally captured inside the front bearing sleeve proxy.",
    )

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="base_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel plate sits on base",
    )
    ctx.expect_within(
        leadscrew,
        turntable,
        axes="xz",
        inner_elem="threaded_shaft",
        outer_elem="front_screw_bearing",
        margin=0.0,
        name="leadscrew centered in front bearing",
    )
    ctx.expect_overlap(
        leadscrew,
        turntable,
        axes="y",
        elem_a="threaded_shaft",
        elem_b="front_screw_bearing",
        min_overlap=0.030,
        name="leadscrew remains retained in bearing",
    )
    for idx in range(2):
        ctx.expect_within(
            turntable,
            moving,
            axes="xz",
            inner_elem=f"guide_bar_{idx}",
            outer_elem=f"guide_bushing_{idx}",
            margin=0.0,
            name=f"guide bar {idx} runs through moving bushing",
        )
        ctx.expect_overlap(
            moving,
            turntable,
            axes="y",
            elem_a=f"guide_bushing_{idx}",
            elem_b=f"guide_bar_{idx}",
            min_overlap=0.045,
            name=f"moving bushing {idx} remains on guide bar",
        )
    ctx.expect_within(
        leadscrew,
        moving,
        axes="xz",
        inner_elem="threaded_shaft",
        outer_elem="lead_nut",
        margin=0.0,
        name="lead screw passes through moving nut",
    )
    ctx.expect_overlap(
        leadscrew,
        moving,
        axes="y",
        elem_a="threaded_shaft",
        elem_b="lead_nut",
        min_overlap=0.030,
        name="lead screw engages moving nut",
    )

    rest_pos = ctx.part_world_position(moving)
    with ctx.pose({slide: 0.085}):
        open_pos = ctx.part_world_position(moving)

    with ctx.pose({slide: 0.085, swivel: 0.75}):
        for idx in range(2):
            ctx.expect_overlap(
                moving,
                turntable,
                axes="y",
                elem_a=f"guide_bushing_{idx}",
                elem_b=f"guide_bar_{idx}",
                min_overlap=0.045,
                name=f"opened jaw keeps guide engagement {idx}",
            )
        ctx.expect_overlap(
            leadscrew,
            moving,
            axes="y",
            elem_a="threaded_shaft",
            elem_b="lead_nut",
            min_overlap=0.030,
            name="opened jaw keeps lead screw engagement",
        )
    ctx.check(
        "front jaw slides outward",
        rest_pos is not None and open_pos is not None and open_pos[1] < rest_pos[1] - 0.075,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
