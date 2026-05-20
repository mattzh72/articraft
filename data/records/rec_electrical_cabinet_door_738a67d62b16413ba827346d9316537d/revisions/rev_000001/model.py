from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_wall_consumer_unit")

    plastic = model.material("warm_white_plastic", rgba=(0.88, 0.86, 0.80, 1.0))
    shadow = model.material("recess_shadow", rgba=(0.06, 0.07, 0.08, 1.0))
    translucent = model.material("smoked_translucent_cover", rgba=(0.46, 0.55, 0.62, 0.42))
    dark = model.material("dark_grey_latch", rgba=(0.03, 0.035, 0.04, 1.0))
    breaker_grey = model.material("breaker_grey", rgba=(0.70, 0.71, 0.69, 1.0))
    metal = model.material("dull_galvanized_metal", rgba=(0.62, 0.64, 0.62, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.385, 0.235),
                outer_size=(0.475, 0.320),
                depth=0.025,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.010,
                outer_corner_radius=0.014,
            ),
            "front_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0125, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="front_frame",
    )
    # Recessed back-box walls: the frame sits flush at y=0 and the box recedes
    # into the wall in +Y.
    housing.visual(
        Box((0.385, 0.006, 0.235)),
        origin=Origin(xyz=(0.0, 0.090, 0.0)),
        material=shadow,
        name="back_panel",
    )
    for x in (-0.198, 0.198):
        housing.visual(
            Box((0.012, 0.070, 0.235)),
            origin=Origin(xyz=(x, 0.055, 0.0)),
            material=plastic,
            name=f"side_wall_{0 if x < 0 else 1}",
        )
    for z in (-0.123, 0.123):
        housing.visual(
            Box((0.385, 0.070, 0.012)),
            origin=Origin(xyz=(0.0, 0.055, z)),
            material=plastic,
            name=f"end_wall_{0 if z < 0 else 1}",
        )

    housing.visual(
        Box((0.330, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.083, 0.022)),
        material=metal,
        name="din_rail",
    )
    breaker_xs = [-0.110, -0.055, 0.0, 0.055, 0.110]
    for i, x in enumerate(breaker_xs):
        housing.visual(
            Box((0.050, 0.045, 0.104)),
            origin=Origin(xyz=(x, 0.060, -0.002)),
            material=breaker_grey,
            name=f"breaker_body_{i}",
        )

    # Fixed hinge knuckles are on the housing; alternating moving knuckles are
    # part of the cover.  All knuckles share the same top horizontal hinge axis.
    hinge_y = -0.012
    hinge_z = 0.150
    for i, x in enumerate((-0.078, 0.078)):
        housing.visual(
            Box((0.060, 0.008, 0.012)),
            origin=Origin(xyz=(x, -0.004, hinge_z + 0.011)),
            material=plastic,
            name=f"hinge_mount_{i}",
        )
        housing.visual(
            Cylinder(radius=0.008, length=0.060),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name=f"fixed_knuckle_{i}",
        )

    housing.visual(
        Box((0.072, 0.016, 0.014)),
        origin=Origin(xyz=(0.0, -0.008, -0.130)),
        material=dark,
        name="latch_keeper",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_geometry(
            BezelGeometry(
                opening_size=(0.335, 0.175),
                outer_size=(0.415, 0.255),
                depth=0.010,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.010,
                outer_corner_radius=0.016,
            ),
            "cover_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.1325), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plastic,
        name="cover_frame",
    )
    cover.visual(
        Box((0.348, 0.004, 0.188)),
        origin=Origin(xyz=(0.0, -0.001, -0.1325)),
        material=translucent,
        name="cover_window",
    )
    for i, (x, length) in enumerate(((-0.150, 0.060), (0.0, 0.072), (0.150, 0.060))):
        cover.visual(
            Cylinder(radius=0.008, length=length),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name=f"cover_knuckle_{i}",
        )

    latch_pivot = (0.0, -0.014, -0.225)
    for i, x in enumerate((-0.027, 0.027)):
        cover.visual(
            Box((0.014, 0.010, 0.018)),
            origin=Origin(xyz=(x, -0.009, -0.225)),
            material=plastic,
            name=f"latch_ear_support_{i}",
        )
        cover.visual(
            Cylinder(radius=0.005, length=0.014),
            origin=Origin(xyz=(x, latch_pivot[1], latch_pivot[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=plastic,
            name=f"latch_ear_{i}",
        )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.005, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark,
        name="pivot_barrel",
    )
    latch.visual(
        Box((0.060, 0.007, 0.045)),
        origin=Origin(xyz=(0.0, -0.006, -0.024)),
        material=dark,
        name="latch_paddle",
    )
    latch.visual(
        Box((0.030, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.001, -0.050)),
        material=dark,
        name="snap_tooth",
    )

    for i, x in enumerate(breaker_xs):
        switch = model.part(f"switch_{i}")
        switch.visual(
            Box((0.018, 0.010, 0.042)),
            origin=Origin(xyz=(0.0, -0.005, -0.021)),
            material=dark,
            name="switch_paddle",
        )
        model.articulation(
            f"breaker_switch_{i}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=switch,
            origin=Origin(xyz=(x, 0.0375, 0.026)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=6.0, lower=0.0, upper=0.55),
        )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=cover,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=cover,
        child=latch,
        origin=Origin(xyz=latch_pivot),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    cover = object_model.get_part("cover")
    latch = object_model.get_part("latch")
    cover_hinge = object_model.get_articulation("cover_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.expect_gap(
        housing,
        cover,
        axis="y",
        positive_elem="front_frame",
        negative_elem="cover_frame",
        min_gap=0.004,
        name="closed cover stands proud of flush frame",
    )
    ctx.expect_overlap(
        cover,
        housing,
        axes="xz",
        elem_a="cover_frame",
        elem_b="front_frame",
        min_overlap=0.20,
        name="cover spans the rectangular housing frame",
    )
    ctx.expect_within(
        latch,
        cover,
        axes="x",
        inner_elem="latch_paddle",
        outer_elem="cover_frame",
        margin=0.0,
        name="snap latch is centered within cover width",
    )

    closed_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_frame")
    with ctx.pose({cover_hinge: 1.20}):
        open_cover_aabb = ctx.part_element_world_aabb(cover, elem="cover_frame")
    ctx.check(
        "cover hinge opens outward from wall",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][1] < closed_cover_aabb[0][1] - 0.10,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    closed_latch_aabb = ctx.part_world_aabb(latch)
    with ctx.pose({latch_pivot: 0.40}):
        open_latch_aabb = ctx.part_world_aabb(latch)
    ctx.check(
        "snap latch rotates outward on its pivot",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and open_latch_aabb[0][1] < closed_latch_aabb[0][1] - 0.010,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
