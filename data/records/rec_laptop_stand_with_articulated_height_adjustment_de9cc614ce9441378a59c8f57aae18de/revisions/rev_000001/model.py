from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    graphite = Material("graphite_powder_coat", color=(0.07, 0.075, 0.08, 1.0))
    black = Material("matte_black_rubber", color=(0.005, 0.005, 0.004, 1.0))
    aluminum = Material("brushed_aluminum", color=(0.72, 0.74, 0.73, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.38, 0.29, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Box((0.060, 0.260, 0.018)),
        origin=Origin(xyz=(-0.135, 0.0, 0.027)),
        material=graphite,
        name="rear_spine",
    )
    for ix, x in enumerate((-0.125, 0.125)):
        for iy, y in enumerate((-0.095, 0.095)):
            base.visual(
                Box((0.070, 0.048, 0.006)),
                origin=Origin(xyz=(x, y, -0.003)),
                material=black,
                name=f"rubber_foot_{ix}_{iy}",
            )

    sleeve_height = 0.130
    sleeve_center_z = 0.018 + sleeve_height / 2.0
    sleeve_outer = 0.042
    sleeve_wall = 0.006
    sleeve_x = -0.135
    post_ys = (-0.105, 0.105)
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x - sleeve_outer / 2.0 + sleeve_wall / 2.0, post_ys[0], sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_0_rear_wall",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x + sleeve_outer / 2.0 - sleeve_wall / 2.0, post_ys[0], sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_0_front_wall",
    )
    base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x, post_ys[0] - sleeve_outer / 2.0 + sleeve_wall / 2.0, sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_0_outer_wall",
    )
    base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x, post_ys[0] + sleeve_outer / 2.0 - sleeve_wall / 2.0, sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_0_inner_wall",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x - sleeve_outer / 2.0 + sleeve_wall / 2.0, post_ys[1], sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_1_rear_wall",
    )
    base.visual(
        Box((sleeve_wall, sleeve_outer, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x + sleeve_outer / 2.0 - sleeve_wall / 2.0, post_ys[1], sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_1_front_wall",
    )
    base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x, post_ys[1] - sleeve_outer / 2.0 + sleeve_wall / 2.0, sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_1_outer_wall",
    )
    base.visual(
        Box((sleeve_outer - 2.0 * sleeve_wall, sleeve_wall, sleeve_height)),
        origin=Origin(
            xyz=(sleeve_x, post_ys[1] + sleeve_outer / 2.0 - sleeve_wall / 2.0, sleeve_center_z)
        ),
        material=graphite,
        name="sleeve_1_inner_wall",
    )

    posts = []
    for i in range(2):
        post = model.part(f"post_{i}")
        post.visual(
            Box((0.024, 0.024, 0.215)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=aluminum,
            name="inner_post",
        )
        post.visual(
            Box((0.032, 0.046, 0.034)),
            origin=Origin(xyz=(-0.018, 0.0, 0.119)),
            material=graphite,
            name="head_block",
        )
        post.visual(
            Cylinder(radius=0.012, length=0.048),
            origin=Origin(xyz=(-0.018, 0.0, 0.119), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=black,
            name="hinge_bushing",
        )
        posts.append(post)

    tray = model.part("tray")
    tray.visual(
        Cylinder(radius=0.014, length=0.250),
        origin=Origin(xyz=(0.0, 0.105, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hinge_barrel",
    )
    tray.visual(
        Box((0.030, 0.230, 0.008)),
        origin=Origin(xyz=(0.015, 0.105, -0.011)),
        material=graphite,
        name="hinge_leaf",
    )
    tray.visual(
        Box((0.300, 0.230, 0.012)),
        origin=Origin(xyz=(0.166, 0.105, -0.014)),
        material=graphite,
        name="tray_panel",
    )
    tray.visual(
        Box((0.018, 0.230, 0.045)),
        origin=Origin(xyz=(0.307, 0.105, 0.0145)),
        material=graphite,
        name="front_lip",
    )
    for y, name in ((-0.006, "side_rail_0"), (0.216, "side_rail_1")):
        tray.visual(
            Box((0.270, 0.012, 0.022)),
            origin=Origin(xyz=(0.166, y, 0.003)),
            material=graphite,
            name=name,
        )
    for y, name in ((0.044, "rubber_strip_0"), (0.166, "rubber_strip_1")):
        tray.visual(
            Box((0.205, 0.018, 0.004)),
            origin=Origin(xyz=(0.172, y, -0.006)),
            material=black,
            name=name,
        )

    post_limits = MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.080)
    model.articulation(
        "base_to_post_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=posts[0],
        origin=Origin(xyz=(sleeve_x, post_ys[0], 0.018 + sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=post_limits,
    )
    model.articulation(
        "base_to_post_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=posts[1],
        origin=Origin(xyz=(sleeve_x, post_ys[1], 0.018 + sleeve_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=post_limits,
        mimic=Mimic("base_to_post_0"),
    )
    model.articulation(
        "post_0_to_tray",
        ArticulationType.REVOLUTE,
        parent=posts[0],
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.119)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.0, lower=0.0, upper=0.58),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    post_0 = object_model.get_part("post_0")
    post_1 = object_model.get_part("post_1")
    tray = object_model.get_part("tray")
    post_slide = object_model.get_articulation("base_to_post_0")
    tray_hinge = object_model.get_articulation("post_0_to_tray")

    for post in (post_0, post_1):
        ctx.allow_overlap(
            post,
            tray,
            elem_a="hinge_bushing",
            elem_b="hinge_barrel",
            reason="The transverse hinge barrel is intentionally captured through the post head bushing.",
        )
        ctx.allow_overlap(
            post,
            tray,
            elem_a="head_block",
            elem_b="hinge_barrel",
            reason="The transverse hinge barrel passes through the solid proxy of the post head.",
        )
        ctx.expect_overlap(
            post,
            tray,
            axes="xyz",
            elem_a="hinge_bushing",
            elem_b="hinge_barrel",
            min_overlap=0.008,
            name=f"{post.name} bushing captures hinge barrel",
        )
        ctx.expect_overlap(
            post,
            tray,
            axes="xyz",
            elem_a="head_block",
            elem_b="hinge_barrel",
            min_overlap=0.010,
            name=f"{post.name} head surrounds hinge barrel",
        )

    with ctx.pose({post_slide: 0.0, tray_hinge: 0.0}):
        rest_0 = ctx.part_world_position(post_0)
        rest_1 = ctx.part_world_position(post_1)
        rest_front = ctx.part_element_world_aabb(tray, elem="front_lip")
        ctx.expect_overlap(
            post_0,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_0_front_wall",
            min_overlap=0.090,
            name="post_0 remains inserted when collapsed",
        )
        ctx.expect_overlap(
            post_1,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_1_front_wall",
            min_overlap=0.090,
            name="post_1 remains inserted when collapsed",
        )

    with ctx.pose({post_slide: 0.080, tray_hinge: 0.0}):
        high_0 = ctx.part_world_position(post_0)
        high_1 = ctx.part_world_position(post_1)
        ctx.expect_overlap(
            post_0,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_0_front_wall",
            min_overlap=0.020,
            name="post_0 remains captured when raised",
        )
        ctx.expect_overlap(
            post_1,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="sleeve_1_front_wall",
            min_overlap=0.020,
            name="post_1 remains captured when raised",
        )

    ctx.check(
        "posts slide upward together",
        rest_0 is not None
        and rest_1 is not None
        and high_0 is not None
        and high_1 is not None
        and abs((high_0[2] - rest_0[2]) - 0.080) < 1.0e-6
        and abs((high_1[2] - rest_1[2]) - 0.080) < 1.0e-6
        and abs(high_0[2] - high_1[2]) < 1.0e-6
        and abs(high_0[0] - rest_0[0]) < 1.0e-6
        and abs(high_1[0] - rest_1[0]) < 1.0e-6,
        details=f"rest_0={rest_0}, rest_1={rest_1}, high_0={high_0}, high_1={high_1}",
    )

    with ctx.pose({post_slide: 0.0, tray_hinge: 0.58}):
        tilted_front = ctx.part_element_world_aabb(tray, elem="front_lip")

    ctx.check(
        "tray tilts upward about rear transverse hinge",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[1][2] > rest_front[1][2] + 0.12,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    return ctx.report()


object_model = build_object_model()
