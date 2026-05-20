from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_drawer_slide")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.64, 0.62, 1.0))
    dark = model.material("blackened_pocket", rgba=(0.03, 0.035, 0.035, 1.0))
    screw = model.material("dark_cap_screw", rgba=(0.015, 0.015, 0.017, 1.0))
    plate_mat = model.material("anodized_plate", rgba=(0.34, 0.37, 0.40, 1.0))

    ground = model.part("ground_channel")
    # Fixed low-profile C-channel, 560 mm long, open through the center.
    ground.visual(
        Box((0.560, 0.075, 0.004)),
        origin=Origin(xyz=(0.280, 0.0, 0.002)),
        material=zinc,
        name="bottom_web",
    )
    for side, y in (("side_0", -0.0355), ("side_1", 0.0355)):
        ground.visual(
            Box((0.560, 0.004, 0.023)),
            origin=Origin(xyz=(0.280, y, 0.0155)),
            material=zinc,
            name=f"{side}_wall",
        )
    for side, y in (("lip_0", -0.0275), ("lip_1", 0.0275)):
        ground.visual(
            Box((0.560, 0.012, 0.003)),
            origin=Origin(xyz=(0.280, y, 0.0255)),
            material=zinc,
            name=f"{side}_return",
        )
    # Small end stops live on the side lanes, leaving the central sliding path clear.
    for i, x in enumerate((0.020, 0.540)):
        for side, y in (("a", -0.0295), ("b", 0.0295)):
            ground.visual(
                Box((0.018, 0.010, 0.010)),
                origin=Origin(xyz=(x, y, 0.009)),
                material=zinc,
                name=f"stop_{i}_{side}",
            )
    # Dark roller pockets and small side rollers, mounted into the channel walls.
    for i, x in enumerate((0.135, 0.425)):
        for side, y in (("a", -0.0378), ("b", 0.0378)):
            ground.visual(
                Box((0.070, 0.001, 0.012)),
                origin=Origin(xyz=(x, y, 0.015)),
                material=dark,
                name=f"pocket_{i}_{side}",
            )
    ground.visual(
        Cylinder(radius=0.0055, length=0.0155),
        origin=Origin(xyz=(0.135, -0.02575, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="roller_0_a",
    )
    ground.visual(
        Cylinder(radius=0.0055, length=0.0155),
        origin=Origin(xyz=(0.135, 0.02575, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="roller_0_b",
    )
    ground.visual(
        Cylinder(radius=0.0055, length=0.0155),
        origin=Origin(xyz=(0.425, -0.02575, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="roller_1_a",
    )
    ground.visual(
        Cylinder(radius=0.0055, length=0.0155),
        origin=Origin(xyz=(0.425, 0.02575, 0.015), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="roller_1_b",
    )
    for i, x in enumerate((0.085, 0.475)):
        for side, y in (("a", -0.0325), ("b", 0.0325)):
            ground.visual(
                Cylinder(radius=0.0036, length=0.002),
                origin=Origin(xyz=(x, y, 0.02775)),
                material=screw,
                name=f"fixed_screw_{i}_{side}",
            )

    inner = model.part("inner_slide")
    # Nested moving slide: a smaller straight channel with its own guide lips.
    inner.visual(
        Box((0.460, 0.033, 0.003)),
        origin=Origin(xyz=(0.230, 0.0, 0.0015)),
        material=zinc,
        name="slide_web",
    )
    inner.visual(
        Box((0.460, 0.003, 0.015)),
        origin=Origin(xyz=(0.230, -0.0165, 0.0105)),
        material=zinc,
        name="side_0_rib",
    )
    inner.visual(
        Box((0.460, 0.003, 0.015)),
        origin=Origin(xyz=(0.230, 0.0165, 0.0105)),
        material=zinc,
        name="side_1_rib",
    )
    for side, y in (("lip_0", -0.0118), ("lip_1", 0.0118)):
        inner.visual(
            Box((0.460, 0.007, 0.0025)),
            origin=Origin(xyz=(0.230, y, 0.0188)),
            material=zinc,
            name=f"{side}_guide",
        )
    for i, x in enumerate((0.055, 0.405)):
        inner.visual(
            Box((0.020, 0.021, 0.006)),
            origin=Origin(xyz=(x, 0.0, 0.006)),
            material=dark,
            name=f"travel_stop_{i}",
        )
    for i, x in enumerate((0.120, 0.330)):
        for side, y in (("a", -0.0118), ("b", 0.0118)):
            inner.visual(
                Cylinder(radius=0.0032, length=0.0018),
                origin=Origin(xyz=(x, y, 0.0206)),
                material=screw,
                name=f"slide_screw_{i}_{side}",
            )

    plate = model.part("equipment_plate")
    # The plate rides on a retained tongue inside the moving slide and sits just above
    # the channel lips, like a small equipment mounting tab on a drawer slide.
    plate.visual(
        Box((0.220, 0.030, 0.006)),
        origin=Origin(xyz=(-0.080, 0.0, 0.010)),
        material=plate_mat,
        name="carrier_tongue",
    )
    plate.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(0.005, 0.0, 0.018)),
        material=plate_mat,
        name="riser",
    )
    plate.visual(
        Box((0.105, 0.055, 0.006)),
        origin=Origin(xyz=(0.058, 0.0, 0.026)),
        material=plate_mat,
        name="mount_plate",
    )
    for i, x in enumerate((0.030, 0.086)):
        for side, y in (("a", -0.018), ("b", 0.018)):
            plate.visual(
                Cylinder(radius=0.0038, length=0.002),
                origin=Origin(xyz=(x, y, 0.030)),
                material=screw,
                name=f"plate_screw_{i}_{side}",
            )

    model.articulation(
        "channel_to_slide",
        ArticulationType.PRISMATIC,
        parent=ground,
        child=inner,
        origin=Origin(xyz=(0.075, 0.0, 0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.220),
        motion_properties=MotionProperties(damping=2.0, friction=0.4),
    )
    model.articulation(
        "slide_to_plate",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=plate,
        origin=Origin(xyz=(0.375, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.120),
        motion_properties=MotionProperties(damping=1.5, friction=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ground = object_model.get_part("ground_channel")
    inner = object_model.get_part("inner_slide")
    plate = object_model.get_part("equipment_plate")
    slide_joint = object_model.get_articulation("channel_to_slide")
    plate_joint = object_model.get_articulation("slide_to_plate")

    ctx.check(
        "serial prismatic slide limits",
        slide_joint.motion_limits is not None
        and plate_joint.motion_limits is not None
        and slide_joint.motion_limits.lower == 0.0
        and slide_joint.motion_limits.upper == 0.220
        and plate_joint.motion_limits.lower == 0.0
        and plate_joint.motion_limits.upper == 0.120,
        details="Both moving members should extend along the same positive X guide axis.",
    )

    rest_inner_pos = ctx.part_world_position(inner)
    rest_plate_pos = ctx.part_world_position(plate)

    ctx.expect_within(
        inner,
        ground,
        axes="yz",
        margin=0.0,
        name="nested slide stays inside channel section",
    )
    ctx.expect_overlap(
        inner,
        ground,
        axes="x",
        min_overlap=0.40,
        name="collapsed slide remains deeply inserted",
    )
    ctx.expect_contact(
        ground,
        inner,
        elem_a="roller_0_b",
        elem_b="side_1_rib",
        contact_tol=0.00002,
        name="fixed roller supports inner rib at rest",
    )
    ctx.expect_within(
        plate,
        inner,
        axes="yz",
        inner_elem="carrier_tongue",
        margin=0.0,
        name="equipment tongue is captured in slide section",
    )
    ctx.expect_overlap(
        plate,
        inner,
        axes="x",
        elem_a="carrier_tongue",
        elem_b="slide_web",
        min_overlap=0.18,
        name="collapsed plate tongue remains retained",
    )
    ctx.expect_contact(
        plate,
        inner,
        elem_a="carrier_tongue",
        elem_b="side_1_rib",
        contact_tol=0.00002,
        name="plate tongue bears on guide rib at rest",
    )

    with ctx.pose({slide_joint: 0.220, plate_joint: 0.120}):
        ctx.expect_within(
            inner,
            ground,
            axes="yz",
            margin=0.0,
            name="extended slide stays inside channel section",
        )
        ctx.expect_overlap(
            inner,
            ground,
            axes="x",
            min_overlap=0.24,
            name="extended slide keeps practical channel overlap",
        )
        ctx.expect_contact(
            ground,
            inner,
            elem_a="roller_1_b",
            elem_b="side_1_rib",
            contact_tol=0.00002,
            name="front roller supports inner rib when extended",
        )
        ctx.expect_within(
            plate,
            inner,
            axes="yz",
            inner_elem="carrier_tongue",
            margin=0.0,
            name="extended equipment tongue remains captured",
        )
        ctx.expect_overlap(
            plate,
            inner,
            axes="x",
            elem_a="carrier_tongue",
            elem_b="slide_web",
            min_overlap=0.14,
            name="extended plate tongue keeps retained insertion",
        )
        ctx.expect_contact(
            plate,
            inner,
            elem_a="carrier_tongue",
            elem_b="side_1_rib",
            contact_tol=0.00002,
            name="plate tongue still bears on guide rib extended",
        )
        extended_inner_pos = ctx.part_world_position(inner)
        extended_plate_pos = ctx.part_world_position(plate)

    ctx.check(
        "inner slide translates along guide axis",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[0] > rest_inner_pos[0] + 0.20,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )
    ctx.check(
        "equipment plate extends serially",
        rest_plate_pos is not None
        and extended_plate_pos is not None
        and extended_plate_pos[0] > rest_plate_pos[0] + 0.32,
        details=f"rest={rest_plate_pos}, extended={extended_plate_pos}",
    )

    return ctx.report()


object_model = build_object_model()
