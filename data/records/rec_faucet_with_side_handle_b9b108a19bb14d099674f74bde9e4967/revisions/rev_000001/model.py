from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _hollow_cylinder(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(length)
        .cut(cq.Workplane("XY").circle(inner_radius).extrude(length))
    )


def _spout_arch_shell() -> cq.Workplane:
    path = (
        cq.Workplane("XZ")
        .moveTo(0.0, 0.018)
        .lineTo(0.0, 0.180)
        .threePointArc((0.070, 0.346), (0.160, 0.200))
        .wire()
    )
    outer = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.018))
        .circle(0.0115)
        .sweep(path)
    )
    inner = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, 0.018))
        .circle(0.0084)
        .sweep(path)
    )
    return outer.cut(inner)


def _spout_receiver_shell() -> cq.Workplane:
    return (
        _hollow_cylinder(0.0135, 0.0088, 0.125)
        .translate((0.160, 0.0, 0.075))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bar_prep_faucet")

    chrome = model.material("chrome", rgba=(0.83, 0.85, 0.88, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.21, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.026, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=chrome,
        name="deck_plate",
    )
    base.visual(
        Cylinder(radius=0.0185, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=chrome,
        name="body_shell",
    )
    base.visual(
        Cylinder(radius=0.0155, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=chrome,
        name="socket_cap",
    )
    base.visual(
        Cylinder(radius=0.011, length=0.007),
        origin=Origin(xyz=(0.0, 0.021, 0.030), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="handle_pad",
    )

    spout = model.part("spout")
    spout.visual(
        mesh_from_cadquery(_hollow_cylinder(0.0150, 0.0112, 0.018), "spout_collar"),
        material=chrome,
        name="spout_collar",
    )
    spout.visual(
        mesh_from_cadquery(_spout_arch_shell(), "spout_arch"),
        material=chrome,
        name="spout_arch",
    )
    spout.visual(
        mesh_from_cadquery(_spout_receiver_shell(), "spout_receiver"),
        material=chrome,
        name="spout_receiver",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0032, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_pivot",
    )
    handle.visual(
        Cylinder(radius=0.0029, length=0.0558),
        origin=Origin(
            xyz=(0.0, 0.035, 0.007),
            rpy=(-math.atan2(0.054, 0.014), 0.0, 0.0),
        ),
        material=chrome,
        name="handle_arm",
    )
    handle.visual(
        Cylinder(radius=0.0034, length=0.018),
        origin=Origin(xyz=(0.0, 0.071, 0.014), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_grip",
    )

    spray_head = model.part("spray_head")
    spray_head.visual(
        Cylinder(radius=0.0042, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, -0.0925)),
        material=dark_trim,
        name="inner_hose",
    )
    spray_head.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        material=chrome,
        name="spray_collar",
    )
    spray_head.visual(
        Cylinder(radius=0.0108, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.150)),
        material=dark_trim,
        name="spray_grip",
    )
    spray_head.visual(
        Cylinder(radius=0.0075, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.175)),
        material=chrome,
        name="spray_tip",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0),
    )

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0245, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    model.articulation(
        "spray_pull",
        ArticulationType.PRISMATIC,
        parent=spout,
        child=spray_head,
        origin=Origin(xyz=(0.160, 0.0, 0.200)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.18,
            lower=0.0,
            upper=0.060,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    spray_head = object_model.get_part("spray_head")
    swivel = object_model.get_articulation("spout_swivel")
    handle_pivot = object_model.get_articulation("handle_pivot")
    spray_pull = object_model.get_articulation("spray_pull")
    handle_upper = handle_pivot.motion_limits.upper if handle_pivot.motion_limits is not None else 1.0
    spray_upper = spray_pull.motion_limits.upper if spray_pull.motion_limits is not None else 0.060

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))

    ctx.expect_contact(
        spout,
        base,
        elem_a="spout_collar",
        elem_b="socket_cap",
        name="spout collar seats on the base socket",
    )
    ctx.expect_contact(
        handle,
        base,
        elem_a="handle_pivot",
        elem_b="handle_pad",
        name="side handle sits on its pivot pad",
    )

    ctx.expect_within(
        spray_head,
        spout,
        axes="xy",
        inner_elem="inner_hose",
        outer_elem="spout_receiver",
        margin=0.0,
        name="hose stays centered in the spray receiver",
    )
    ctx.expect_overlap(
        spray_head,
        spout,
        axes="z",
        elem_a="inner_hose",
        elem_b="spout_receiver",
        min_overlap=0.11,
        name="spray head remains deeply inserted at rest",
    )
    ctx.expect_gap(
        spout,
        spray_head,
        axis="z",
        positive_elem="spout_receiver",
        negative_elem="spray_collar",
        min_gap=0.0,
        max_gap=0.004,
        name="spray collar parks just below the receiver opening",
    )

    receiver_rest = aabb_center(ctx.part_element_world_aabb(spout, elem="spout_receiver"))
    with ctx.pose({swivel: math.pi / 2.0}):
        receiver_turned = aabb_center(ctx.part_element_world_aabb(spout, elem="spout_receiver"))
        ctx.expect_contact(
            spout,
            base,
            elem_a="spout_collar",
            elem_b="socket_cap",
            name="spout collar stays seated while swiveling",
        )

    ctx.check(
        "spout rotates about the vertical axis",
        receiver_rest is not None
        and receiver_turned is not None
        and receiver_rest[0] > 0.14
        and abs(receiver_turned[1]) > 0.14
        and abs(receiver_rest[2] - receiver_turned[2]) < 1e-6,
        details=f"rest={receiver_rest}, turned={receiver_turned}",
    )

    handle_rest = ctx.part_world_aabb(handle)
    with ctx.pose({handle_pivot: handle_upper}):
        handle_open = ctx.part_world_aabb(handle)
    ctx.check(
        "side handle lifts upward",
        handle_rest is not None
        and handle_open is not None
        and handle_open[1][2] > handle_rest[1][2] + 0.02,
        details=f"rest={handle_rest}, open={handle_open}",
    )

    spray_rest = ctx.part_world_position(spray_head)
    with ctx.pose({spray_pull: spray_upper}):
        spray_extended = ctx.part_world_position(spray_head)
        ctx.expect_within(
            spray_head,
            spout,
            axes="xy",
            inner_elem="inner_hose",
            outer_elem="spout_receiver",
            margin=0.0,
            name="hose stays centered when the spray head is extended",
        )
        ctx.expect_overlap(
            spray_head,
            spout,
            axes="z",
            elem_a="inner_hose",
            elem_b="spout_receiver",
            min_overlap=0.06,
            name="extended spray head keeps hose insertion in the receiver",
        )

    ctx.check(
        "spray head pulls downward",
        spray_rest is not None and spray_extended is not None and spray_extended[2] < spray_rest[2] - 0.05,
        details=f"rest={spray_rest}, extended={spray_extended}",
    )

    return ctx.report()


object_model = build_object_model()
