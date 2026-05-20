from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vehicle_barrier_with_breakout_section")

    post_gray = model.material("post_gray", rgba=(0.26, 0.27, 0.30, 1.0))
    head_gray = model.material("head_gray", rgba=(0.36, 0.37, 0.40, 1.0))
    boom_white = model.material("boom_white", rgba=(0.96, 0.97, 0.95, 1.0))
    stripe_red = model.material("stripe_red", rgba=(0.80, 0.12, 0.10, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.10, 0.10, 0.11, 1.0))

    boom_height = 0.09
    inner_width = 0.05
    outer_width = 0.02
    breakout_axis_y = -(inner_width + outer_width) / 4.0
    outer_center_y = -breakout_axis_y

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.11, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=post_gray,
        name="post_shaft",
    )
    post.visual(
        Cylinder(radius=0.16, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=head_gray,
        name="post_base_flange",
    )
    post.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=head_gray,
        name="post_head_collar",
    )
    post.visual(
        Box((0.18, 0.14, 0.12)),
        origin=Origin(xyz=(0.08, 0.0, 1.03)),
        material=head_gray,
        name="post_head_housing",
    )
    post.visual(
        Cylinder(radius=0.03, length=0.16),
        origin=Origin(xyz=(0.17, 0.0, 1.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material=head_gray,
        name="post_hinge_barrel",
    )
    post.visual(
        Box((0.04, 0.08, 0.08)),
        origin=Origin(xyz=(0.19, 0.0, 1.03)),
        material=head_gray,
        name="post_clevis_face",
    )
    post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.13, length=1.05),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
    )

    inner_arm = model.part("inner_arm")
    inner_arm.visual(
        Box((1.68, inner_width, boom_height)),
        origin=Origin(xyz=(0.96, 0.0, -0.045)),
        material=boom_white,
        name="inner_body",
    )
    inner_arm.visual(
        Box((0.12, 0.06, 0.08)),
        origin=Origin(xyz=(0.09, 0.0, -0.04)),
        material=head_gray,
        name="inner_root_clamp",
    )
    inner_arm.visual(
        Box((0.04, 0.05, 0.04)),
        origin=Origin(xyz=(0.02, 0.0, -0.02)),
        material=head_gray,
        name="inner_hinge_ear",
    )
    inner_arm.visual(
        Box((0.10, 0.03, 0.12)),
        origin=Origin(xyz=(1.85, -0.01, -0.06)),
        material=head_gray,
        name="inner_knuckle_block",
    )
    for idx, x_pos in enumerate((0.42, 0.92, 1.42), start=1):
        inner_arm.visual(
            Box((0.18, inner_width + 0.002, boom_height + 0.002)),
            origin=Origin(xyz=(x_pos, 0.0, -0.045)),
            material=stripe_red,
            name=f"inner_stripe_{idx}",
        )
    inner_arm.inertial = Inertial.from_geometry(
        Box((1.90, 0.07, 0.12)),
        mass=13.0,
        origin=Origin(xyz=(0.95, 0.0, 0.0)),
    )

    outer_arm = model.part("outer_arm")
    outer_arm.visual(
        Box((0.10, outer_width, 0.11)),
        origin=Origin(xyz=(0.05, outer_center_y, -0.055)),
        material=head_gray,
        name="outer_root_block",
    )
    outer_arm.visual(
        Box((1.70, outer_width, 0.08)),
        origin=Origin(xyz=(0.95, outer_center_y, -0.04)),
        material=boom_white,
        name="outer_body",
    )
    outer_arm.visual(
        Box((0.09, outer_width + 0.006, 0.10)),
        origin=Origin(xyz=(1.755, outer_center_y, -0.05)),
        material=bumper_black,
        name="outer_tip_cap",
    )
    for idx, x_pos in enumerate((0.34, 0.80, 1.26), start=1):
        outer_arm.visual(
            Box((0.16, outer_width + 0.002, 0.082)),
            origin=Origin(xyz=(x_pos, outer_center_y, -0.04)),
            material=stripe_red,
            name=f"outer_stripe_{idx}",
        )
    outer_arm.inertial = Inertial.from_geometry(
        Box((1.80, 0.03, 0.11)),
        mass=7.0,
        origin=Origin(xyz=(0.90, outer_center_y, 0.0)),
    )

    model.articulation(
        "post_to_inner_arm",
        ArticulationType.REVOLUTE,
        parent=post,
        child=inner_arm,
        origin=Origin(xyz=(0.21, 0.0, 1.03)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.9,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "inner_to_outer_breakout",
        ArticulationType.REVOLUTE,
        parent=inner_arm,
        child=outer_arm,
        origin=Origin(xyz=(1.90, breakout_axis_y, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.5,
            lower=0.0,
            upper=pi,
        ),
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

    post = object_model.get_part("post")
    inner_arm = object_model.get_part("inner_arm")
    outer_arm = object_model.get_part("outer_arm")
    main_hinge = object_model.get_articulation("post_to_inner_arm")
    breakout_hinge = object_model.get_articulation("inner_to_outer_breakout")

    ctx.check(
        "barrier assembly parts exist",
        all(part is not None for part in (post, inner_arm, outer_arm)),
        details="Expected post, inner arm, and breakout outer arm.",
    )

    with ctx.pose({main_hinge: 0.0, breakout_hinge: 0.0}):
        ctx.expect_contact(
            inner_arm,
            outer_arm,
            elem_a="inner_knuckle_block",
            elem_b="outer_root_block",
            name="breakout knuckle mates in the straight barrier pose",
        )
        ctx.expect_overlap(
            inner_arm,
            outer_arm,
            axes="yz",
            elem_a="inner_body",
            elem_b="outer_body",
            min_overlap=0.018,
            name="straight breakout section stays aligned with the inner boom",
        )
        closed_outer = ctx.part_element_world_aabb(outer_arm, elem="outer_body")

    with ctx.pose({main_hinge: 1.20, breakout_hinge: 0.0}):
        opened_outer = ctx.part_element_world_aabb(outer_arm, elem="outer_body")

    ctx.check(
        "main boom lifts upward from the post head",
        closed_outer is not None
        and opened_outer is not None
        and opened_outer[1][2] > closed_outer[1][2] + 1.0,
        details=f"closed_outer={closed_outer}, opened_outer={opened_outer}",
    )

    with ctx.pose({main_hinge: 0.0, breakout_hinge: pi}):
        ctx.expect_gap(
            inner_arm,
            outer_arm,
            axis="y",
            positive_elem="inner_body",
            negative_elem="outer_body",
            max_gap=0.002,
            max_penetration=0.0,
            name="folded breakout section lies flush to the inner arm side",
        )
        ctx.expect_overlap(
            inner_arm,
            outer_arm,
            axes="xz",
            elem_a="inner_body",
            elem_b="outer_body",
            min_overlap=0.075,
            name="folded breakout section stacks back along the inner arm",
        )
        folded_outer = ctx.part_element_world_aabb(outer_arm, elem="outer_body")

    ctx.check(
        "breakout articulation shortens the deployed reach for emergency through-access",
        closed_outer is not None
        and folded_outer is not None
        and folded_outer[1][0] < closed_outer[1][0] - 1.5,
        details=f"closed_outer={closed_outer}, folded_outer={folded_outer}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
