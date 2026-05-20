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
    model = ArticulatedObject(name="guide_plunger_output_flap")

    painted_iron = model.material("painted_iron", color=(0.16, 0.19, 0.21, 1.0))
    dark_recess = model.material("dark_bore", color=(0.02, 0.022, 0.024, 1.0))
    bright_steel = model.material("polished_steel", color=(0.74, 0.74, 0.70, 1.0))
    yellow_flap = model.material("safety_yellow", color=(0.95, 0.67, 0.12, 1.0))
    rubber = model.material("black_rubber", color=(0.03, 0.03, 0.028, 1.0))

    guide_block = model.part("guide_block")
    guide_block.visual(
        Box((0.205, 0.128, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=painted_iron,
        name="base_flange",
    )
    guide_block.visual(
        Box((0.190, 0.104, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=painted_iron,
        name="guide_top",
    )
    guide_block.visual(
        Box((0.190, 0.104, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=painted_iron,
        name="guide_bottom",
    )
    guide_block.visual(
        Box((0.190, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, -0.039, 0.0)),
        material=painted_iron,
        name="guide_side_0",
    )
    guide_block.visual(
        Box((0.190, 0.026, 0.050)),
        origin=Origin(xyz=(0.0, 0.039, 0.0)),
        material=painted_iron,
        name="guide_side_1",
    )
    guide_block.visual(
        Box((0.014, 0.030, 0.030)),
        origin=Origin(xyz=(0.102, 0.0, 0.027)),
        material=dark_recess,
        name="front_upper_bore_liner",
    )
    guide_block.visual(
        Box((0.014, 0.030, 0.030)),
        origin=Origin(xyz=(-0.102, 0.0, 0.027)),
        material=dark_recess,
        name="rear_upper_bore_liner",
    )
    for prefix, x in (("front", 0.102), ("rear", -0.102)):
        guide_block.visual(
            Box((0.014, 0.030, 0.030)),
            origin=Origin(xyz=(x, 0.0, -0.027)),
            material=dark_recess,
            name=f"{prefix}_lower_bore_liner",
        )
        guide_block.visual(
            Box((0.014, 0.030, 0.024)),
            origin=Origin(xyz=(x, -0.027, 0.0)),
            material=dark_recess,
            name=f"{prefix}_bore_side_0",
        )
        guide_block.visual(
            Box((0.014, 0.030, 0.024)),
            origin=Origin(xyz=(x, 0.027, 0.0)),
            material=dark_recess,
            name=f"{prefix}_bore_side_1",
        )
    for i, x in enumerate((-0.074, 0.074)):
        for j, y in enumerate((-0.048, 0.048)):
            guide_block.visual(
                Cylinder(radius=0.008, length=0.003),
                origin=Origin(xyz=(x, y, -0.037), rpy=(0.0, 0.0, 0.0)),
                material=dark_recess,
                name=f"mount_bolt_{i}_{j}",
            )

    plunger_rod = model.part("plunger_rod")
    plunger_rod.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="rod_shaft",
    )
    plunger_rod.visual(
        Cylinder(radius=0.025, length=0.026),
        origin=Origin(xyz=(-0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="rear_push_cap",
    )
    plunger_rod.visual(
        Cylinder(radius=0.019, length=0.024),
        origin=Origin(xyz=(0.141, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="front_collar",
    )
    plunger_rod.visual(
        Box((0.030, 0.070, 0.026)),
        origin=Origin(xyz=(0.153, 0.0, 0.0)),
        material=bright_steel,
        name="yoke_bridge",
    )
    for name, y in (("yoke_cheek_0", -0.035), ("yoke_cheek_1", 0.035)):
        plunger_rod.visual(
            Box((0.044, 0.010, 0.036)),
            origin=Origin(xyz=(0.174, y, 0.0)),
            material=bright_steel,
            name=name,
        )
    plunger_rod.visual(
        Cylinder(radius=0.0045, length=0.084),
        origin=Origin(xyz=(0.174, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="hinge_pin",
    )

    output_flap = model.part("output_flap")
    output_flap.visual(
        Cylinder(radius=0.0085, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="flap_barrel",
    )
    output_flap.visual(
        Box((0.010, 0.066, 0.072)),
        origin=Origin(xyz=(0.006, 0.0, -0.043)),
        material=yellow_flap,
        name="paddle_plate",
    )
    output_flap.visual(
        Box((0.014, 0.070, 0.006)),
        origin=Origin(xyz=(0.006, 0.0, -0.079)),
        material=rubber,
        name="soft_edge",
    )

    model.articulation(
        "guide_to_plunger",
        ArticulationType.PRISMATIC,
        parent=guide_block,
        child=plunger_rod,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.055),
    )

    model.articulation(
        "plunger_to_flap",
        ArticulationType.REVOLUTE,
        parent=plunger_rod,
        child=output_flap,
        origin=Origin(xyz=(0.174, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=4.0, lower=0.0, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_block")
    rod = object_model.get_part("plunger_rod")
    flap = object_model.get_part("output_flap")
    slide = object_model.get_articulation("guide_to_plunger")
    hinge = object_model.get_articulation("plunger_to_flap")

    ctx.allow_overlap(
        rod,
        flap,
        elem_a="hinge_pin",
        elem_b="flap_barrel",
        reason="The solid pin is intentionally represented captured inside the flap hinge barrel.",
    )

    ctx.expect_within(
        rod,
        flap,
        axes="xz",
        inner_elem="hinge_pin",
        outer_elem="flap_barrel",
        margin=0.0005,
        name="hinge pin stays inside barrel diameter",
    )
    ctx.expect_overlap(
        rod,
        flap,
        axes="y",
        elem_a="hinge_pin",
        elem_b="flap_barrel",
        min_overlap=0.050,
        name="hinge pin spans flap barrel",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_within(
            rod,
            guide,
            axes="yz",
            inner_elem="rod_shaft",
            margin=0.0,
            name="rod is centered in the fixed guide bore",
        )
        ctx.expect_overlap(
            rod,
            guide,
            axes="x",
            elem_a="rod_shaft",
            min_overlap=0.150,
            name="rod remains deeply guided when retracted",
        )
        rest_rod_pos = ctx.part_world_position(rod)
        rest_plate_aabb = ctx.part_element_world_aabb(flap, elem="paddle_plate")

    with ctx.pose({slide: 0.055, hinge: 0.99}):
        ctx.expect_within(
            rod,
            guide,
            axes="yz",
            inner_elem="rod_shaft",
            margin=0.0,
            name="extended rod stays aligned in the bore",
        )
        ctx.expect_overlap(
            rod,
            guide,
            axes="x",
            elem_a="rod_shaft",
            min_overlap=0.105,
            name="rod retains insertion at full stroke",
        )
        extended_rod_pos = ctx.part_world_position(rod)
        extended_plate_aabb = ctx.part_element_world_aabb(flap, elem="paddle_plate")

    ctx.check(
        "plunger translates forward",
        rest_rod_pos is not None
        and extended_rod_pos is not None
        and extended_rod_pos[0] > rest_rod_pos[0] + 0.050,
        details=f"rest={rest_rod_pos}, extended={extended_rod_pos}",
    )
    ctx.check(
        "paddle rotates forward with plunger stroke",
        rest_plate_aabb is not None
        and extended_plate_aabb is not None
        and extended_plate_aabb[1][0] > rest_plate_aabb[1][0] + 0.080
        and extended_plate_aabb[0][2] > rest_plate_aabb[0][2] + 0.020,
        details=f"rest={rest_plate_aabb}, extended={extended_plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
