from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_laptop_stand")

    dark_metal = model.material("dark_anodized_aluminum", rgba=(0.08, 0.085, 0.09, 1.0))
    satin_metal = model.material("brushed_aluminum", rgba=(0.58, 0.60, 0.60, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))

    # Coordinate convention: +X points toward the front lip of the tray, +Y spans
    # the stand width, and +Z is vertical.  The hinge line is the shared
    # horizontal Y axis through the rear post heads.
    base_depth = 0.34
    base_width = 0.40
    base_thickness = 0.016
    base_center_x = 0.040
    rear_x = -0.105
    post_half_span = 0.150
    sleeve_height = 0.140
    sleeve_top_z = base_thickness + sleeve_height
    sleeve_outer = 0.044
    sleeve_inner = 0.028
    sleeve_wall = (sleeve_outer - sleeve_inner) / 2.0
    hinge_z = 0.105

    base = model.part("base")
    base.visual(
        Box((base_depth, base_width, base_thickness)),
        origin=Origin(xyz=(base_center_x, 0.0, base_thickness / 2.0)),
        material=dark_metal,
        name="base_plate",
    )
    # A raised rear rail ties the two sleeve sockets into the base casting.
    base.visual(
        Box((0.024, 0.350, 0.020)),
        origin=Origin(xyz=(rear_x - 0.030, 0.0, base_thickness + 0.010)),
        material=dark_metal,
        name="rear_stiffener",
    )

    sleeve_specs = (
        (
            -post_half_span,
            "sleeve_0_rear_wall",
            "sleeve_0_front_wall",
            "sleeve_0_side_wall_0",
            "sleeve_0_side_wall_1",
        ),
        (
            post_half_span,
            "sleeve_1_rear_wall",
            "sleeve_1_front_wall",
            "sleeve_1_side_wall_0",
            "sleeve_1_side_wall_1",
        ),
    )
    for y, rear_wall, front_wall, side_wall_0, side_wall_1 in sleeve_specs:
        # Four wall boxes form each fixed outer sleeve.  The sleeve remains open
        # so the sliding post tube has real clearance instead of being modeled as
        # a solid-overlapping proxy.
        base.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_height)),
            origin=Origin(
                xyz=(rear_x - sleeve_inner / 2.0 - sleeve_wall / 2.0, y, base_thickness + sleeve_height / 2.0)
            ),
            material=dark_metal,
            name=rear_wall,
        )
        base.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_height)),
            origin=Origin(
                xyz=(rear_x + sleeve_inner / 2.0 + sleeve_wall / 2.0, y, base_thickness + sleeve_height / 2.0)
            ),
            material=dark_metal,
            name=front_wall,
        )
        base.visual(
            Box((sleeve_inner, sleeve_wall, sleeve_height)),
            origin=Origin(
                xyz=(rear_x, y - sleeve_inner / 2.0 - sleeve_wall / 2.0, base_thickness + sleeve_height / 2.0)
            ),
            material=dark_metal,
            name=side_wall_0,
        )
        base.visual(
            Box((sleeve_inner, sleeve_wall, sleeve_height)),
            origin=Origin(
                xyz=(rear_x, y + sleeve_inner / 2.0 + sleeve_wall / 2.0, base_thickness + sleeve_height / 2.0)
            ),
            material=dark_metal,
            name=side_wall_1,
        )

    for x in (-0.095, 0.175):
        for y in (-0.155, 0.155):
            base.visual(
                Cylinder(radius=0.020, length=0.006),
                origin=Origin(xyz=(x, y, -0.003)),
                material=rubber,
                name=f"rubber_foot_{x}_{y}",
            )

    posts = []
    for index, y in enumerate((-post_half_span, post_half_span)):
        post = model.part(f"post_{index}")
        post.visual(
            Box((0.028, 0.028, 0.200)),
            # The tube extends well below the sleeve lip so it remains captured
            # after the short vertical travel.
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=satin_metal,
            name="inner_tube",
        )
        post.visual(
            Box((0.046, 0.046, 0.027)),
            origin=Origin(xyz=(0.0, 0.0, 0.0835)),
            material=dark_metal,
            name="head_pad",
        )
        post.visual(
            Box((0.030, 0.030, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, 0.061)),
            material=satin_metal,
            name="lock_collar",
        )
        posts.append(post)

        model.articulation(
            f"base_to_post_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=post,
            origin=Origin(xyz=(rear_x, y, sleeve_top_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=120.0, velocity=0.08, lower=0.0, upper=0.065),
            mimic=Mimic("base_to_post_0") if index == 1 else None,
        )

    tray = model.part("tray")
    tray_panel = SlotPatternPanelGeometry(
        (0.300, 0.350),
        0.010,
        slot_size=(0.070, 0.009),
        pitch=(0.095, 0.032),
        frame=0.020,
        corner_radius=0.008,
        stagger=True,
    )
    tray.visual(
        mesh_from_geometry(tray_panel, "vented_tray_panel"),
        origin=Origin(xyz=(0.185, 0.0, -0.015)),
        material=dark_metal,
        name="vented_panel",
    )
    tray.visual(
        Cylinder(radius=0.008, length=0.350),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="hinge_bar",
    )
    tray.visual(
        Box((0.026, 0.350, 0.006)),
        origin=Origin(xyz=(0.013, 0.0, -0.009)),
        material=satin_metal,
        name="hinge_leaf",
    )
    tray.visual(
        Box((0.054, 0.060, 0.018)),
        origin=Origin(xyz=(0.023, 0.0, -0.007)),
        material=satin_metal,
        name="center_hinge_bridge",
    )
    tray.visual(
        Box((0.018, 0.335, 0.035)),
        origin=Origin(xyz=(0.333, 0.0, 0.0045)),
        material=dark_metal,
        name="front_lip",
    )
    for y, rail_name in ((-0.180, "side_rail_0"), (0.180, "side_rail_1")):
        tray.visual(
            Box((0.285, 0.010, 0.018)),
            origin=Origin(xyz=(0.185, y, -0.008)),
            material=dark_metal,
            name=rail_name,
        )
    for y, strip_name in ((-0.075, "rubber_strip_0"), (0.075, "rubber_strip_1")):
        tray.visual(
            Box((0.210, 0.014, 0.003)),
            origin=Origin(xyz=(0.195, y, -0.0085)),
            material=rubber,
            name=strip_name,
        )

    model.articulation(
        "post_to_tray",
        ArticulationType.REVOLUTE,
        parent=posts[0],
        child=tray,
        # The parent is post_0 at the negative-Y sleeve; offset the joint origin
        # to the midpoint of the shared hinge line between both post heads.
        origin=Origin(xyz=(0.0, post_half_span, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post_0 = object_model.get_part("post_0")
    post_1 = object_model.get_part("post_1")
    tray = object_model.get_part("tray")
    post_slide = object_model.get_articulation("base_to_post_0")
    tray_hinge = object_model.get_articulation("post_to_tray")

    # Both post tubes remain inserted in their fixed sleeves at rest.
    ctx.expect_overlap(
        post_0,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="sleeve_0_front_wall",
        min_overlap=0.120,
        name="post 0 retained in sleeve",
    )
    ctx.expect_overlap(
        post_1,
        base,
        axes="z",
        elem_a="inner_tube",
        elem_b="sleeve_1_front_wall",
        min_overlap=0.120,
        name="post 1 retained in sleeve",
    )

    # The single tray hinge bar rests on both post heads along the same line.
    ctx.expect_gap(
        tray,
        post_0,
        axis="z",
        positive_elem="hinge_bar",
        negative_elem="head_pad",
        max_gap=0.002,
        max_penetration=1e-5,
        name="hinge bar seated on post 0 head",
    )
    ctx.expect_gap(
        tray,
        post_1,
        axis="z",
        positive_elem="hinge_bar",
        negative_elem="head_pad",
        max_gap=0.002,
        max_penetration=1e-5,
        name="hinge bar seated on post 1 head",
    )

    rest_0 = ctx.part_world_position(post_0)
    rest_1 = ctx.part_world_position(post_1)
    ctx.check(
        "post origins aligned at rest",
        rest_0 is not None and rest_1 is not None and abs(rest_0[2] - rest_1[2]) < 1e-6,
        details=f"post_0={rest_0}, post_1={rest_1}",
    )

    with ctx.pose({post_slide: 0.065}):
        ext_0 = ctx.part_world_position(post_0)
        ext_1 = ctx.part_world_position(post_1)
        ctx.check(
            "matched posts extend together",
            ext_0 is not None
            and ext_1 is not None
            and rest_0 is not None
            and ext_0[2] > rest_0[2] + 0.060
            and abs(ext_0[2] - ext_1[2]) < 1e-6,
            details=f"rest={rest_0}, post_0={ext_0}, post_1={ext_1}",
        )
        ctx.expect_overlap(
            post_0,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="sleeve_0_front_wall",
            min_overlap=0.055,
            name="post 0 retained when raised",
        )
        ctx.expect_overlap(
            post_1,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="sleeve_1_front_wall",
            min_overlap=0.055,
            name="post 1 retained when raised",
        )

    front_rest = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_hinge: 0.50}):
        front_up = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tray_hinge: -0.35}):
        front_down = ctx.part_element_world_aabb(tray, elem="front_lip")
    ctx.check(
        "tray rotates about rear hinge line",
        front_rest is not None
        and front_up is not None
        and front_down is not None
        and front_up[1][2] > front_rest[1][2] + 0.050
        and front_down[0][2] < front_rest[0][2] - 0.030,
        details=f"rest={front_rest}, up={front_up}, down={front_down}",
    )

    return ctx.report()


object_model = build_object_model()
