from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rising_security_bollard")

    steel = model.material("polished_stainless_steel", rgba=(0.76, 0.78, 0.78, 1.0))
    dark_steel = model.material("shadowed_lock_slot", rgba=(0.04, 0.045, 0.05, 1.0))
    guide_polymer = model.material("black_guide_polymer", rgba=(0.015, 0.015, 0.014, 1.0))
    concrete = model.material("sawn_gray_pavement", rgba=(0.42, 0.42, 0.39, 1.0))

    base = model.part("pavement_sleeve")

    pavement = LatheGeometry(
        [
            (0.235, -0.120),
            (0.600, -0.120),
            (0.600, 0.000),
            (0.235, 0.000),
        ],
        segments=96,
    )
    base.visual(
        mesh_from_geometry(pavement, "pavement_with_round_opening"),
        material=concrete,
        name="pavement_opening",
    )

    sleeve_wall = LatheGeometry(
        [
            (0.138, -0.38),
            (0.198, -0.38),
            (0.198, 0.035),
            (0.138, 0.035),
        ],
        segments=80,
    )
    base.visual(
        mesh_from_geometry(sleeve_wall, "below_grade_sleeve_wall"),
        material=steel,
        name="sleeve_wall",
    )

    collar_ring = LatheGeometry(
        [
            (0.138, 0.000),
            (0.205, 0.000),
            (0.242, 0.016),
            (0.242, 0.064),
            (0.205, 0.080),
            (0.138, 0.080),
        ],
        segments=96,
    )
    base.visual(
        mesh_from_geometry(collar_ring, "beveled_sleeve_collar"),
        material=steel,
        name="collar_ring",
    )

    # Four small polymer guide pads visibly explain how the sliding post is carried by the sleeve.
    base.visual(
        Box((0.039, 0.040, 0.080)),
        origin=Origin(xyz=(0.1225, 0.0, 0.040)),
        material=guide_polymer,
        name="guide_pad_x_pos",
    )
    base.visual(
        Box((0.039, 0.040, 0.080)),
        origin=Origin(xyz=(-0.1225, 0.0, 0.040)),
        material=guide_polymer,
        name="guide_pad_x_neg",
    )
    base.visual(
        Box((0.040, 0.039, 0.080)),
        origin=Origin(xyz=(0.0, 0.1225, 0.040)),
        material=guide_polymer,
        name="guide_pad_y_pos",
    )
    base.visual(
        Box((0.040, 0.039, 0.080)),
        origin=Origin(xyz=(0.0, -0.1225, 0.040)),
        material=guide_polymer,
        name="guide_pad_y_neg",
    )

    # A compact raised boss at the pavement-edge side of the collar carries the latch hinge.
    base.visual(
        Box((0.18, 0.22, 0.074)),
        origin=Origin(xyz=(0.265, 0.0, 0.037)),
        material=steel,
        name="latch_boss",
    )
    for suffix, y in (("0", -0.076), ("1", 0.076)):
        base.visual(
            Box((0.045, 0.026, 0.055)),
            origin=Origin(xyz=(0.290, y, 0.096)),
            material=steel,
            name=f"hinge_lug_{suffix}",
        )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.105, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=steel,
        name="post_body",
    )

    latch_cover = model.part("latch_cover")
    latch_cover.visual(
        Box((0.156, 0.120, 0.018)),
        origin=Origin(xyz=(-0.078, 0.0, 0.0)),
        material=steel,
        name="cover_plate",
    )
    latch_cover.visual(
        Cylinder(radius=0.018, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )

    lock_cap = model.part("lock_cap")
    lock_cap.visual(
        Cylinder(radius=0.116, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=steel,
        name="cap_disk",
    )
    lock_cap.visual(
        Box((0.112, 0.024, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=dark_steel,
        name="key_slot",
    )

    model.articulation(
        "sleeve_to_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=-0.55, upper=0.18),
    )

    model.articulation(
        "collar_to_latch_cover",
        ArticulationType.REVOLUTE,
        parent=base,
        child=latch_cover,
        origin=Origin(xyz=(0.290, 0.0, 0.096)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    model.articulation(
        "post_to_lock_cap",
        ArticulationType.CONTINUOUS,
        parent=post,
        child=lock_cap,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    base = object_model.get_part("pavement_sleeve")
    post = object_model.get_part("post")
    cover = object_model.get_part("latch_cover")
    cap = object_model.get_part("lock_cap")
    post_slide = object_model.get_articulation("sleeve_to_post")
    cover_hinge = object_model.get_articulation("collar_to_latch_cover")

    for pad_name in (
        "guide_pad_x_pos",
        "guide_pad_x_neg",
        "guide_pad_y_pos",
        "guide_pad_y_neg",
    ):
        ctx.allow_overlap(
            base,
            post,
            elem_a=pad_name,
            elem_b="post_body",
            reason="The polymer guide pad is intentionally modeled with a tiny compressed sliding contact against the post.",
        )

    ctx.expect_gap(
        base,
        post,
        axis="x",
        positive_elem="guide_pad_x_pos",
        negative_elem="post_body",
        max_gap=0.001,
        max_penetration=0.004,
        name="positive x guide pad is a light sliding fit",
    )
    ctx.expect_gap(
        post,
        base,
        axis="x",
        positive_elem="post_body",
        negative_elem="guide_pad_x_neg",
        max_gap=0.001,
        max_penetration=0.004,
        name="negative x guide pad is a light sliding fit",
    )
    ctx.expect_gap(
        base,
        post,
        axis="y",
        positive_elem="guide_pad_y_pos",
        negative_elem="post_body",
        max_gap=0.001,
        max_penetration=0.004,
        name="positive y guide pad is a light sliding fit",
    )
    ctx.expect_gap(
        post,
        base,
        axis="y",
        positive_elem="post_body",
        negative_elem="guide_pad_y_neg",
        max_gap=0.001,
        max_penetration=0.004,
        name="negative y guide pad is a light sliding fit",
    )

    ctx.expect_overlap(
        post,
        base,
        axes="z",
        elem_a="post_body",
        elem_b="sleeve_wall",
        min_overlap=0.18,
        name="post remains deeply retained in the sleeve at rest",
    )
    ctx.expect_gap(
        cover,
        base,
        axis="z",
        positive_elem="cover_plate",
        negative_elem="collar_ring",
        min_gap=0.004,
        max_gap=0.012,
        name="closed latch cover sits just above the beveled collar",
    )
    ctx.expect_contact(
        cap,
        post,
        elem_a="cap_disk",
        elem_b="post_body",
        contact_tol=0.001,
        name="rotating lock cap is seated on the round post top",
    )

    rest_post = ctx.part_world_position(post)
    rest_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({post_slide: 0.18, cover_hinge: 1.15}):
        raised_post = ctx.part_world_position(post)
        raised_cover_aabb = ctx.part_world_aabb(cover)
        ctx.expect_overlap(
            post,
            base,
            axes="z",
            elem_a="post_body",
            elem_b="collar_ring",
            min_overlap=0.05,
            name="raised post still has retained insertion through the collar",
        )

    ctx.check(
        "post prismatic motion travels upward",
        rest_post is not None
        and raised_post is not None
        and raised_post[2] > rest_post[2] + 0.15,
        details=f"rest={rest_post}, raised={raised_post}",
    )
    ctx.check(
        "latch cover hinge lifts the free edge",
        rest_cover_aabb is not None
        and raised_cover_aabb is not None
        and raised_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.08,
        details=f"rest_aabb={rest_cover_aabb}, raised_aabb={raised_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
