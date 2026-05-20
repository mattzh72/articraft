from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="column_mounted_radial_arm")

    dark_iron = Material("dark_iron", color=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = Material("brushed_steel", color=(0.63, 0.66, 0.67, 1.0))
    blue_casting = Material("blue_casting", color=(0.08, 0.20, 0.43, 1.0))
    safety_yellow = Material("safety_yellow", color=(0.95, 0.63, 0.08, 1.0))
    black_rubber = Material("black_rubber", color=(0.015, 0.015, 0.015, 1.0))
    red_carriage = Material("red_carriage", color=(0.72, 0.08, 0.05, 1.0))

    post = model.part("post")
    post.visual(
        Box((0.62, 0.62, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_iron,
        name="floor_plate",
    )
    post.visual(
        Cylinder(radius=0.075, length=1.142),
        origin=Origin(xyz=(0.0, 0.0, 0.629)),
        material=brushed_steel,
        name="column",
    )
    post.visual(
        Cylinder(radius=0.12, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 1.186)),
        material=dark_iron,
        name="top_bearing_seat",
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.145, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=blue_casting,
        name="turntable",
    )
    turret.visual(
        Cylinder(radius=0.105, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=blue_casting,
        name="rotary_hub",
    )
    turret.visual(
        Box((1.25, 0.12, 0.10)),
        origin=Origin(xyz=(0.72, 0.0, 0.20)),
        material=safety_yellow,
        name="beam_web",
    )
    turret.visual(
        Box((1.18, 0.022, 0.032)),
        origin=Origin(xyz=(0.72, -0.047, 0.136)),
        material=dark_iron,
        name="lower_rail_0",
    )
    turret.visual(
        Box((1.18, 0.022, 0.032)),
        origin=Origin(xyz=(0.72, 0.047, 0.136)),
        material=dark_iron,
        name="lower_rail_1",
    )
    turret.visual(
        Box((0.06, 0.18, 0.16)),
        origin=Origin(xyz=(0.13, 0.0, 0.17)),
        material=blue_casting,
        name="inner_stop",
    )
    turret.visual(
        Box((0.06, 0.18, 0.18)),
        origin=Origin(xyz=(1.355, 0.0, 0.18)),
        material=blue_casting,
        name="end_stop",
    )

    truck = model.part("truck")
    truck.visual(
        Box((0.22, 0.17, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=red_carriage,
        name="carriage_plate",
    )
    truck.visual(
        Box((0.22, 0.025, 0.11)),
        origin=Origin(xyz=(0.0, -0.085, -0.015)),
        material=red_carriage,
        name="side_cheek_0",
    )
    truck.visual(
        Box((0.22, 0.025, 0.11)),
        origin=Origin(xyz=(0.0, 0.085, -0.015)),
        material=red_carriage,
        name="side_cheek_1",
    )
    truck.visual(
        Cylinder(radius=0.025, length=0.19),
        origin=Origin(xyz=(-0.06, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="roller_0",
    )
    truck.visual(
        Cylinder(radius=0.025, length=0.19),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_rubber,
        name="roller_1",
    )
    truck.visual(
        Box((0.12, 0.10, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, -0.147)),
        material=red_carriage,
        name="tool_mount",
    )

    model.articulation(
        "post_to_turret",
        ArticulationType.REVOLUTE,
        parent=post,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "turret_to_truck",
        ArticulationType.PRISMATIC,
        parent=turret,
        child=truck,
        origin=Origin(xyz=(0.35, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.82),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post = object_model.get_part("post")
    turret = object_model.get_part("turret")
    truck = object_model.get_part("truck")
    swing = object_model.get_articulation("post_to_turret")
    travel = object_model.get_articulation("turret_to_truck")

    ctx.check(
        "turret is vertical revolute",
        swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(swing.axis) == (0.0, 0.0, 1.0),
        details=f"type={swing.articulation_type}, axis={swing.axis}",
    )
    ctx.check(
        "truck is beam-axis prismatic",
        travel.articulation_type == ArticulationType.PRISMATIC
        and tuple(travel.axis) == (1.0, 0.0, 0.0),
        details=f"type={travel.articulation_type}, axis={travel.axis}",
    )
    ctx.expect_contact(
        turret,
        post,
        elem_a="turntable",
        elem_b="top_bearing_seat",
        contact_tol=1e-5,
        name="turret rests on post bearing",
    )
    ctx.expect_contact(
        truck,
        turret,
        elem_a="roller_0",
        elem_b="lower_rail_0",
        contact_tol=1e-5,
        name="truck roller rides rail",
    )
    ctx.expect_overlap(
        truck,
        turret,
        axes="x",
        elem_a="carriage_plate",
        elem_b="beam_web",
        min_overlap=0.15,
        name="truck stays under beam at rest",
    )

    rest_pos = ctx.part_world_position(truck)
    with ctx.pose({travel: 0.82}):
        ctx.expect_contact(
            truck,
            turret,
            elem_a="roller_1",
            elem_b="lower_rail_1",
            contact_tol=1e-5,
            name="extended truck still rides rail",
        )
        ctx.expect_overlap(
            truck,
            turret,
            axes="x",
            elem_a="carriage_plate",
            elem_b="beam_web",
            min_overlap=0.15,
            name="truck remains carried by beam at full travel",
        )
        extended_pos = ctx.part_world_position(truck)
    ctx.check(
        "truck translates outward along beam",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.80,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({swing: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(truck)
    ctx.check(
        "turret swings truck around post",
        rest_pos is not None
        and rotated_pos is not None
        and abs(rotated_pos[0]) < 0.02
        and rotated_pos[1] > rest_pos[0] - 0.02,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
