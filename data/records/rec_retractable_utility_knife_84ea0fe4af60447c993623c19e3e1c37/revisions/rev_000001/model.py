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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_box_cutter")

    shell_red = model.material("molded_red_shell", rgba=(0.78, 0.08, 0.035, 1.0))
    rubber = model.material("black_rubber", rgba=(0.018, 0.018, 0.018, 1.0))
    dark = model.material("shadow_black", rgba=(0.002, 0.002, 0.002, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.74, 0.69, 1.0))
    blade_edge = model.material("honed_edge", rgba=(0.92, 0.92, 0.88, 1.0))
    guard_plastic = model.material("translucent_smoke_guard", rgba=(1.0, 0.62, 0.10, 0.48))

    length = 0.125
    width = 0.044
    thickness = 0.018

    body = cq.Workplane("XY").box(length, width, thickness)
    body = body.edges("|Z").fillet(0.007)
    body = body.edges(">Z or <Z").fillet(0.0015)

    blade_tunnel = (
        cq.Workplane("XY")
        .box(0.112, 0.017, 0.007)
        .translate((0.010, 0.0, 0.0))
    )
    thumb_slot = (
        cq.Workplane("XY")
        .box(0.092, 0.011, 0.016)
        .translate((-0.006, 0.0, 0.0055))
    )
    body = body.cut(blade_tunnel).cut(thumb_slot)

    shell = model.part("shell")
    shell.visual(
        mesh_from_cadquery(body, "rounded_handle_shell", tolerance=0.0008),
        material=shell_red,
        name="handle_shell",
    )

    # Raised rubber grip islands are slightly proud of the broad side faces.
    shell.visual(
        Box((0.066, 0.003, 0.011)),
        origin=Origin(xyz=(-0.015, width / 2.0 + 0.0012, -0.0005)),
        material=rubber,
        name="side_grip_0",
    )
    shell.visual(
        Box((0.066, 0.003, 0.011)),
        origin=Origin(xyz=(-0.015, -width / 2.0 - 0.0012, -0.0005)),
        material=rubber,
        name="side_grip_1",
    )
    shell.visual(
        Box((0.003, 0.016, 0.006)),
        origin=Origin(xyz=(-(length / 2.0) + 0.012, 0.0, -0.0005)),
        material=dark,
        name="rear_channel_stop",
    )

    hinge_x = length / 2.0 + 0.0015
    hinge_z = thickness / 2.0 + 0.005
    shell.visual(
        Cylinder(radius=0.0025, length=0.046),
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    for idx, y in enumerate((-0.020, 0.020)):
        shell.visual(
            Box((0.010, 0.004, 0.008)),
            origin=Origin(xyz=(length / 2.0 - 0.001, y, thickness / 2.0 + 0.0025)),
            material=shell_red,
            name=f"hinge_ear_{idx}",
        )

    carrier = model.part("blade_carrier")
    carrier.visual(
        Box((0.064, 0.012, 0.003)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=steel,
        name="carrier_rail",
    )
    blade_profile = (
        cq.Workplane("XY")
        .polyline(
            [
                (-0.041, -0.006),
                (0.039, -0.006),
                (0.058, 0.0),
                (0.039, 0.006),
                (-0.041, 0.006),
            ]
        )
        .close()
        .extrude(0.0016)
        .translate((0.0, 0.0, -0.0008))
    )
    carrier.visual(
        mesh_from_cadquery(blade_profile, "trapezoid_blade", tolerance=0.0004),
        material=blade_edge,
        name="blade",
    )
    carrier.visual(
        Box((0.008, 0.006, 0.009)),
        origin=Origin(xyz=(-0.018, 0.0, 0.006)),
        material=dark,
        name="thumb_stem",
    )
    carrier.visual(
        Box((0.024, 0.018, 0.006)),
        origin=Origin(xyz=(-0.018, 0.0, 0.012)),
        material=rubber,
        name="thumb_slider",
    )

    guard = model.part("guard_cover")
    guard.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=guard_plastic,
        name="hinge_barrel",
    )
    guard.visual(
        Box((0.014, 0.032, 0.005)),
        origin=Origin(xyz=(0.004, 0.0, -0.005)),
        material=guard_plastic,
        name="upper_bridge",
    )
    guard.visual(
        Box((0.006, 0.038, 0.026)),
        origin=Origin(xyz=(0.010, 0.0, -0.014)),
        material=guard_plastic,
        name="front_shield",
    )
    guard.visual(
        Box((0.020, 0.040, 0.005)),
        origin=Origin(xyz=(0.017, 0.0, -0.027)),
        material=guard_plastic,
        name="guard_lip",
    )

    model.articulation(
        "shell_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=carrier,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.026, effort=35.0, velocity=0.20),
    )
    model.articulation(
        "shell_to_guard_cover",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=guard,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=2.0, velocity=2.5),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    shell = object_model.get_part("shell")
    carrier = object_model.get_part("blade_carrier")
    guard = object_model.get_part("guard_cover")
    slide = object_model.get_articulation("shell_to_blade_carrier")
    hinge = object_model.get_articulation("shell_to_guard_cover")

    ctx.allow_overlap(
        shell,
        guard,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        reason="The guard barrel is intentionally clipped around the fixed nose hinge pin.",
    )
    ctx.expect_contact(
        shell,
        guard,
        elem_a="hinge_pin",
        elem_b="hinge_barrel",
        contact_tol=0.002,
        name="guard barrel remains on the nose pin",
    )

    ctx.expect_within(
        carrier,
        shell,
        axes="y",
        inner_elem="carrier_rail",
        outer_elem="handle_shell",
        margin=0.004,
        name="blade carrier is centered in the handle channel",
    )
    ctx.expect_overlap(
        carrier,
        shell,
        axes="x",
        elem_a="carrier_rail",
        elem_b="handle_shell",
        min_overlap=0.040,
        name="retracted carrier stays retained inside the short handle",
    )

    rest_pos = ctx.part_world_position(carrier)
    with ctx.pose({slide: 0.026}):
        extended_pos = ctx.part_world_position(carrier)
        ctx.expect_overlap(
            carrier,
            shell,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_shell",
            min_overlap=0.026,
            name="extended carrier remains inserted in the shell",
        )
    ctx.check(
        "blade carrier slides toward the nose",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.020,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    closed_guard = ctx.part_element_world_aabb(guard, elem="guard_lip")
    with ctx.pose({hinge: 1.20}):
        open_guard = ctx.part_element_world_aabb(guard, elem="guard_lip")
    ctx.check(
        "guard cover flips upward from the nose",
        closed_guard is not None
        and open_guard is not None
        and open_guard[0][2] > closed_guard[0][2] + 0.015,
        details=f"closed={closed_guard}, open={open_guard}",
    )

    return ctx.report()


object_model = build_object_model()
