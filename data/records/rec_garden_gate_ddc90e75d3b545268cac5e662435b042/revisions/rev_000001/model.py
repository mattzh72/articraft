from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="single_leaf_garden_gate")

    weathered_wood = Material("weathered_wood", rgba=(0.50, 0.32, 0.18, 1.0))
    end_grain = Material("end_grain", rgba=(0.38, 0.23, 0.12, 1.0))
    dark_iron = Material("blackened_iron", rgba=(0.02, 0.018, 0.015, 1.0))
    concrete = Material("concrete_footing", rgba=(0.48, 0.46, 0.42, 1.0))

    posts = model.part("posts")
    # A low footing ties the two square posts into one grounded support assembly.
    posts.visual(
        Box((1.46, 0.18, 0.055)),
        origin=Origin(xyz=(0.66, 0.0, 0.0275)),
        material=concrete,
        name="footing",
    )
    for x, name in ((0.0, "hinge_post"), (1.32, "latch_post")):
        posts.visual(
            Box((0.12, 0.12, 1.35)),
            origin=Origin(xyz=(x, 0.0, 0.675)),
            material=weathered_wood,
            name=name,
        )
        posts.visual(
            Box((0.15, 0.15, 0.055)),
            origin=Origin(xyz=(x, 0.0, 1.3775)),
            material=end_grain,
            name=f"{name}_cap",
        )

    # Fixed halves of the two strap hinges on the hinge post.
    hinge_axis_x = 0.095
    for i, zc in enumerate((0.42, 0.92)):
        posts.visual(
            Box((0.010, 0.080, 0.190)),
            origin=Origin(xyz=(0.064, -0.001, zc)),
            material=dark_iron,
            name=f"post_hinge_plate_{i}",
        )
        for suffix, kz in (("lower", zc - 0.0605), ("upper", zc + 0.0605)):
            posts.visual(
                Cylinder(radius=0.020, length=0.047),
                origin=Origin(xyz=(hinge_axis_x, 0.0, kz)),
                material=dark_iron,
                name=f"post_hinge_knuckle_{i}_{suffix}",
            )
            posts.visual(
                Box((0.043, 0.018, 0.034)),
                origin=Origin(xyz=(0.078, 0.0, kz)),
                material=dark_iron,
                name=f"post_hinge_web_{i}_{suffix}",
            )

    # Receiver/keeper for the thumb latch on the opposite post.
    posts.visual(
        Box((0.010, 0.085, 0.145)),
        origin=Origin(xyz=(1.258, -0.020, 0.755)),
        material=dark_iron,
        name="latch_keeper_plate",
    )
    posts.visual(
        Box((0.038, 0.030, 0.028)),
        origin=Origin(xyz=(1.237, -0.045, 0.735)),
        material=dark_iron,
        name="latch_keeper_lip",
    )

    leaf = model.part("leaf")
    # Wooden rectangular leaf frame.
    leaf.visual(
        Box((0.070, 0.045, 0.980)),
        origin=Origin(xyz=(0.075, 0.0, 0.650)),
        material=weathered_wood,
        name="hinge_stile",
    )
    leaf.visual(
        Box((0.070, 0.045, 0.980)),
        origin=Origin(xyz=(1.080, 0.0, 0.650)),
        material=weathered_wood,
        name="latch_stile",
    )
    leaf.visual(
        Box((1.075, 0.045, 0.070)),
        origin=Origin(xyz=(0.580, 0.0, 1.105)),
        material=weathered_wood,
        name="top_rail",
    )
    leaf.visual(
        Box((1.075, 0.045, 0.070)),
        origin=Origin(xyz=(0.580, 0.0, 0.195)),
        material=weathered_wood,
        name="bottom_rail",
    )
    leaf.visual(
        Box((1.010, 0.040, 0.055)),
        origin=Origin(xyz=(0.580, 0.0, 0.650)),
        material=weathered_wood,
        name="middle_rail",
    )
    # Diagonal brace gives the garden gate a believable sag-resistant build.
    leaf.visual(
        Box((1.110, 0.038, 0.050)),
        origin=Origin(xyz=(0.580, 0.0, 0.650), rpy=(0.0, -0.750, 0.0)),
        material=weathered_wood,
        name="diagonal_brace",
    )
    for i, x in enumerate((0.220, 0.365, 0.510, 0.655, 0.800, 0.945)):
        leaf.visual(
            Box((0.040, 0.032, 0.860)),
            origin=Origin(xyz=(x, 0.0, 0.650)),
            material=weathered_wood,
            name=f"picket_{i}",
        )

    # Moving halves of the two strap hinges: barrel knuckles and long straps.
    for i, zc in enumerate((0.42, 0.92)):
        leaf.visual(
            Cylinder(radius=0.018, length=0.074),
            origin=Origin(xyz=(0.0, 0.0, zc)),
            material=dark_iron,
            name=f"gate_hinge_barrel_{i}",
        )
        leaf.visual(
            Box((0.460, 0.012, 0.045)),
            origin=Origin(xyz=(0.230, -0.022, zc)),
            material=dark_iron,
            name=f"strap_hinge_{i}",
        )

    # Thumb-latch housing and latch bar fixed to the moving leaf.
    leaf.visual(
        Box((0.170, 0.024, 0.110)),
        origin=Origin(xyz=(1.015, -0.034, 0.795)),
        material=dark_iron,
        name="latch_housing",
    )
    leaf.visual(
        Box((0.180, 0.018, 0.024)),
        origin=Origin(xyz=(1.025, -0.043, 0.735)),
        material=dark_iron,
        name="latch_bar",
    )

    thumb = model.part("thumb")
    thumb.visual(
        Cylinder(radius=0.009, length=0.055),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="thumb_pin",
    )
    thumb.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="thumb_boss",
    )
    thumb.visual(
        Box((0.030, 0.012, 0.038)),
        origin=Origin(xyz=(0.000, -0.034, -0.024)),
        material=dark_iron,
        name="thumb_neck",
    )
    thumb.visual(
        Box((0.110, 0.012, 0.036)),
        origin=Origin(xyz=(0.030, -0.034, -0.052)),
        material=dark_iron,
        name="thumb_paddle",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=posts,
        child=leaf,
        origin=Origin(xyz=(hinge_axis_x, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.65),
    )
    model.articulation(
        "thumb_pin",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=thumb,
        origin=Origin(xyz=(1.015, -0.058, 0.795)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    posts = object_model.get_part("posts")
    leaf = object_model.get_part("leaf")
    thumb = object_model.get_part("thumb")
    leaf_hinge = object_model.get_articulation("leaf_hinge")
    thumb_pin = object_model.get_articulation("thumb_pin")

    ctx.allow_overlap(
        leaf,
        thumb,
        elem_a="latch_housing",
        elem_b="thumb_pin",
        reason="The thumb-latch pin is intentionally captured through the latch housing.",
    )
    ctx.expect_within(
        thumb,
        leaf,
        axes="xz",
        inner_elem="thumb_pin",
        outer_elem="latch_housing",
        margin=0.010,
        name="thumb pin is centered in latch housing",
    )
    ctx.expect_overlap(
        thumb,
        leaf,
        axes="y",
        elem_a="thumb_pin",
        elem_b="latch_housing",
        min_overlap=0.010,
        name="thumb pin passes through latch housing",
    )

    with ctx.pose({leaf_hinge: 0.0, thumb_pin: 0.0}):
        ctx.expect_gap(
            posts,
            leaf,
            axis="x",
            positive_elem="latch_keeper_lip",
            negative_elem="latch_bar",
            min_gap=-0.001,
            max_gap=0.012,
            name="latch bar closes against keeper",
        )
        ctx.expect_overlap(
            leaf,
            posts,
            axes="z",
            elem_a="latch_bar",
            elem_b="latch_keeper_lip",
            min_overlap=0.010,
            name="latch bar is vertically aligned with keeper",
        )

    closed_latch_aabb = ctx.part_element_world_aabb(leaf, elem="latch_bar")
    with ctx.pose({leaf_hinge: 1.20}):
        open_latch_aabb = ctx.part_element_world_aabb(leaf, elem="latch_bar")
        ctx.expect_gap(
            leaf,
            posts,
            axis="y",
            positive_elem="latch_bar",
            negative_elem="footing",
            min_gap=0.18,
            name="opened leaf swings outward from posts",
        )
    ctx.check(
        "leaf hinge has vertical revolute axis",
        tuple(round(v, 6) for v in leaf_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={leaf_hinge.axis}",
    )
    ctx.check(
        "thumb pin has transverse revolute axis",
        tuple(round(v, 6) for v in thumb_pin.axis) == (0.0, 1.0, 0.0),
        details=f"axis={thumb_pin.axis}",
    )
    ctx.check(
        "latch end moves outward when opened",
        closed_latch_aabb is not None
        and open_latch_aabb is not None
        and ((open_latch_aabb[0][1] + open_latch_aabb[1][1]) / 2.0)
        > ((closed_latch_aabb[0][1] + closed_latch_aabb[1][1]) / 2.0) + 0.25,
        details=f"closed={closed_latch_aabb}, open={open_latch_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
