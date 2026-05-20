from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drill_press_vise")

    model.material("cast_iron_blue", color=(0.18, 0.25, 0.30, 1.0))
    model.material("dark_cast_iron", color=(0.09, 0.11, 0.12, 1.0))
    model.material("machined_steel", color=(0.70, 0.72, 0.70, 1.0))
    model.material("dark_slot", color=(0.015, 0.016, 0.018, 1.0))
    model.material("black_grip", color=(0.02, 0.02, 0.018, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.350, 0.180, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="cast_iron_blue",
        name="base_plate",
    )

    # Four dark, shallow mounting slots on the flat drill-press table base.
    for i, (x, y) in enumerate(
        ((-0.115, -0.065), (-0.115, 0.065), (0.115, -0.065), (0.115, 0.065))
    ):
        base.visual(
            Box((0.055, 0.014, 0.002)),
            origin=Origin(xyz=(x, y, 0.027)),
            material="dark_slot",
            name=f"mount_slot_{i}",
        )

    # Parallel machined rails that carry the movable jaw.
    for i, y in enumerate((-0.052, 0.052)):
        base.visual(
            Box((0.220, 0.018, 0.010)),
            origin=Origin(xyz=(-0.025, y, 0.031)),
            material="machined_steel",
            name=f"guide_rail_{i}",
        )

    # The fixed rear jaw is a connected bridge-and-cheek casting with an open
    # screw tunnel through the centerline under the gripping plate.
    for i, y in enumerate((-0.052, 0.052)):
        base.visual(
            Box((0.072, 0.036, 0.064)),
            origin=Origin(xyz=(0.095, y, 0.058)),
            material="cast_iron_blue",
            name=f"rear_cheek_{i}",
        )
    base.visual(
        Box((0.072, 0.150, 0.030)),
        origin=Origin(xyz=(0.095, 0.0, 0.090)),
        material="cast_iron_blue",
        name="rear_bridge",
    )
    base.visual(
        Box((0.007, 0.132, 0.034)),
        origin=Origin(xyz=(0.0565, 0.0, 0.091)),
        material="machined_steel",
        name="rear_jaw_plate",
    )
    for i, z in enumerate((0.081, 0.091, 0.101)):
        base.visual(
            Box((0.003, 0.126, 0.0025)),
            origin=Origin(xyz=(0.052, 0.0, z)),
            material="dark_cast_iron",
            name=f"rear_tooth_{i}",
        )

    front_jaw = model.part("front_jaw")
    # A movable C-shaped casting: side shoes ride on the rails, while the open
    # center clears the lead screw.
    for i, y in enumerate((-0.052, 0.052)):
        front_jaw.visual(
            Box((0.060, 0.038, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0275)),
            material="cast_iron_blue",
            name=f"slide_cheek_{i}",
        )
        front_jaw.visual(
            Box((0.070, 0.024, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.005)),
            material="dark_cast_iron",
            name=f"rail_shoe_{i}",
        )
    front_jaw.visual(
        Box((0.060, 0.146, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material="cast_iron_blue",
        name="slide_bridge",
    )
    # Split bearing yoke around the lead screw collar.  The gap between the two
    # lugs leaves the screw visible while giving the rotating screw a real
    # support path back into the sliding jaw casting.
    for i, y in enumerate((-0.022, 0.022)):
        front_jaw.visual(
            Box((0.026, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, y, 0.026)),
            material="cast_iron_blue",
            name=f"bearing_yoke_{i}",
        )
    front_jaw.visual(
        Box((0.007, 0.128, 0.034)),
        origin=Origin(xyz=(0.0335, 0.0, 0.091)),
        material="machined_steel",
        name="front_jaw_plate",
    )
    for i, z in enumerate((0.081, 0.091, 0.101)):
        front_jaw.visual(
            Box((0.003, 0.122, 0.0025)),
            origin=Origin(xyz=(0.038, 0.0, z)),
            material="dark_cast_iron",
            name=f"front_tooth_{i}",
        )

    lead_screw = model.part("lead_screw")
    lead_screw.visual(
        Cylinder(radius=0.006, length=0.178),
        origin=Origin(xyz=(0.093, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="screw_shaft",
    )
    for i, x in enumerate((0.050, 0.065, 0.080, 0.095, 0.110, 0.125, 0.140, 0.155)):
        lead_screw.visual(
            Cylinder(radius=0.0069, length=0.003),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_cast_iron",
            name=f"thread_rib_{i}",
        )
    lead_screw.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.043, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="thrust_collar",
    )
    lead_screw.visual(
        Cylinder(radius=0.013, length=0.026),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="machined_steel",
        name="handle_hub",
    )
    lead_screw.visual(
        Cylinder(radius=0.0055, length=0.116),
        origin=Origin(xyz=(-0.022, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="machined_steel",
        name="handle_bar",
    )
    for i, y in enumerate((-0.064, 0.064)):
        lead_screw.visual(
            Sphere(radius=0.009),
            origin=Origin(xyz=(-0.022, y, 0.0)),
            material="black_grip",
            name=f"handle_knob_{i}",
        )

    model.articulation(
        "base_to_front_jaw",
        ArticulationType.PRISMATIC,
        parent=base,
        child=front_jaw,
        origin=Origin(xyz=(-0.070, 0.0, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=150.0, velocity=0.08, lower=0.0, upper=0.080),
    )
    model.articulation(
        "front_jaw_to_lead_screw",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=lead_screw,
        origin=Origin(xyz=(-0.043, 0.0, 0.020)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    front_jaw = object_model.get_part("front_jaw")
    lead_screw = object_model.get_part("lead_screw")
    slide = object_model.get_articulation("base_to_front_jaw")
    handle = object_model.get_articulation("front_jaw_to_lead_screw")

    ctx.expect_gap(
        front_jaw,
        base,
        axis="z",
        positive_elem="slide_cheek_0",
        negative_elem="guide_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="front jaw shoe rests on guide rail",
    )
    ctx.expect_overlap(
        front_jaw,
        base,
        axes="xy",
        elem_a="slide_cheek_0",
        elem_b="guide_rail_0",
        min_overlap=0.012,
        name="front jaw is carried by the rail footprint",
    )

    ctx.expect_gap(
        base,
        front_jaw,
        axis="x",
        positive_elem="rear_jaw_plate",
        negative_elem="front_jaw_plate",
        min_gap=0.070,
        max_gap=0.100,
        name="vise jaws start open",
    )

    rest_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({slide: 0.080}):
        ctx.expect_gap(
            base,
            front_jaw,
            axis="x",
            positive_elem="rear_jaw_plate",
            negative_elem="front_jaw_plate",
            min_gap=0.004,
            max_gap=0.018,
            name="front jaw closes toward fixed jaw",
        )
        closed_pos = ctx.part_world_position(front_jaw)

    ctx.check(
        "prismatic jaw advances along the vise length",
        rest_pos is not None
        and closed_pos is not None
        and closed_pos[0] > rest_pos[0] + 0.075,
        details=f"rest={rest_pos}, closed={closed_pos}",
    )

    rest_aabb = ctx.part_world_aabb(lead_screw)
    with ctx.pose({handle: math.pi / 2.0}):
        turned_aabb = ctx.part_world_aabb(lead_screw)
    if rest_aabb is not None and turned_aabb is not None:
        rest_y = rest_aabb[1][1] - rest_aabb[0][1]
        rest_z = rest_aabb[1][2] - rest_aabb[0][2]
        turned_y = turned_aabb[1][1] - turned_aabb[0][1]
        turned_z = turned_aabb[1][2] - turned_aabb[0][2]
        handle_rotates = turned_z > rest_z + 0.050 and turned_y < rest_y - 0.050
    else:
        handle_rotates = False
    ctx.check(
        "T handle rotates continuously about the screw",
        handle_rotates,
        details=f"rest_aabb={rest_aabb}, turned_aabb={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
