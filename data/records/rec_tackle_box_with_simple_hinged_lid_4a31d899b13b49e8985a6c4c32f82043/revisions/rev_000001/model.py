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
    model = ArticulatedObject(name="plain_hinged_tackle_box")

    dark_green = Material("dark_green_plastic", rgba=(0.05, 0.24, 0.12, 1.0))
    lid_green = Material("lid_green_plastic", rgba=(0.07, 0.32, 0.16, 1.0))
    tray_gray = Material("matte_gray_tray", rgba=(0.46, 0.50, 0.46, 1.0))
    hinge_metal = Material("dull_hinge_pin", rgba=(0.55, 0.57, 0.54, 1.0))

    body = model.part("body")

    # A rectangular open box: bottom slab and four walls leave the top hollow.
    body.visual(
        Box((0.500, 0.260, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_green,
        name="bottom",
    )
    body.visual(
        Box((0.500, 0.014, 0.122)),
        origin=Origin(xyz=(0.0, -0.123, 0.079)),
        material=dark_green,
        name="front_wall",
    )
    body.visual(
        Box((0.500, 0.014, 0.122)),
        origin=Origin(xyz=(0.0, 0.123, 0.079)),
        material=dark_green,
        name="rear_wall",
    )
    body.visual(
        Box((0.014, 0.232, 0.122)),
        origin=Origin(xyz=(-0.243, 0.0, 0.079)),
        material=dark_green,
        name="side_wall_0",
    )
    body.visual(
        Box((0.014, 0.232, 0.122)),
        origin=Origin(xyz=(0.243, 0.0, 0.079)),
        material=dark_green,
        name="side_wall_1",
    )

    # Fixed molded tray details inside the body.  They are low enough to remain
    # inside the box and touch the bottom slab so the tray reads as integral.
    body.visual(
        Box((0.010, 0.200, 0.056)),
        origin=Origin(xyz=(-0.080, 0.0, 0.046)),
        material=tray_gray,
        name="long_divider_0",
    )
    body.visual(
        Box((0.010, 0.200, 0.056)),
        origin=Origin(xyz=(0.080, 0.0, 0.046)),
        material=tray_gray,
        name="long_divider_1",
    )
    body.visual(
        Box((0.462, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, -0.046, 0.041)),
        material=tray_gray,
        name="cross_divider_0",
    )
    body.visual(
        Box((0.462, 0.010, 0.046)),
        origin=Origin(xyz=(0.0, 0.046, 0.041)),
        material=tray_gray,
        name="cross_divider_1",
    )

    # Two rear hinge knuckles fixed to the body, with simple molded leaves
    # connecting them to the rear wall.
    for suffix, x in (("0", -0.180), ("1", 0.180)):
        body.visual(
            Box((0.100, 0.008, 0.015)),
            origin=Origin(xyz=(x, 0.134, 0.1375)),
            material=dark_green,
            name=f"hinge_leaf_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.100),
            origin=Origin(xyz=(x, 0.137, 0.153), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_knuckle_{suffix}",
        )

    lid = model.part("lid")

    # The lid frame is centered on the rear hinge pin axis.  At the closed pose
    # the long, thin panel extends forward from that axis and rests on the box
    # walls.
    lid.visual(
        Box((0.520, 0.274, 0.018)),
        origin=Origin(xyz=(0.0, -0.143, -0.004)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.012, 0.274, 0.022)),
        origin=Origin(xyz=(-0.266, -0.143, -0.024)),
        material=lid_green,
        name="side_lip_0",
    )
    lid.visual(
        Box((0.012, 0.274, 0.022)),
        origin=Origin(xyz=(0.266, -0.143, -0.024)),
        material=lid_green,
        name="side_lip_1",
    )
    lid.visual(
        Box((0.520, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, -0.286, -0.024)),
        material=lid_green,
        name="front_lip",
    )
    lid.visual(
        Cylinder(radius=0.0075, length=0.200),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="hinge_barrel",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.137, 0.153)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.check(
        "single rear lid hinge",
        len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (-1.0, 0.0, 0.0),
        details=f"articulations={object_model.articulations}",
    )

    limits = hinge.motion_limits
    ctx.check(
        "lid has realistic opening range",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 1.5 <= limits.upper <= 2.1,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=0.0001,
            name="closed lid rests on front wall",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="front_wall",
            min_overlap=0.010,
            name="closed lid covers the box mouth",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({hinge: 1.20}):
        open_aabb = ctx.part_world_aabb(lid)
        ctx.expect_origin_gap(
            lid,
            body,
            axis="z",
            min_gap=0.0,
            name="opened lid lifts above body origin",
        )

    ctx.check(
        "opening motion lifts the front edge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
