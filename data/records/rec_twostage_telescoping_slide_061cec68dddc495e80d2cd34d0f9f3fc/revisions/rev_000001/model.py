from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide")

    dark_steel = model.material("dark_burnished_steel", color=(0.12, 0.13, 0.14, 1.0))
    bright_steel = model.material("bright_zinc_plated_steel", color=(0.62, 0.64, 0.62, 1.0))
    black_acetal = model.material("black_acetal_glide", color=(0.015, 0.015, 0.014, 1.0))
    screw_black = model.material("blackened_fasteners", color=(0.03, 0.032, 0.032, 1.0))

    outer = model.part("outer_body")
    # Grounded mounting plate and an open, C-shaped outer slide channel.
    outer.visual(
        Box((0.68, 0.12, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_steel,
        name="mounting_foot",
    )
    outer.visual(
        Box((0.56, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.035, 0.0175)),
        material=dark_steel,
        name="side_pedestal_0",
    )
    outer.visual(
        Box((0.56, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, -0.035, 0.0175)),
        material=dark_steel,
        name="side_pedestal_1",
    )
    outer.visual(
        Box((0.62, 0.090, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=dark_steel,
        name="bottom_web",
    )
    outer.visual(
        Box((0.62, 0.012, 0.057)),
        origin=Origin(xyz=(0.0, 0.045, 0.054)),
        material=dark_steel,
        name="side_wall_0",
    )
    outer.visual(
        Box((0.62, 0.012, 0.057)),
        origin=Origin(xyz=(0.0, -0.045, 0.054)),
        material=dark_steel,
        name="side_wall_1",
    )
    outer.visual(
        Box((0.62, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, 0.031, 0.087)),
        material=dark_steel,
        name="top_lip_0",
    )
    outer.visual(
        Box((0.62, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.031, 0.087)),
        material=dark_steel,
        name="top_lip_1",
    )
    outer.visual(
        Box((0.012, 0.090, 0.065)),
        origin=Origin(xyz=(-0.316, 0.0, 0.058)),
        material=dark_steel,
        name="rear_stop",
    )
    for idx, (x, y) in enumerate(
        ((-0.245, -0.043), (-0.245, 0.043), (0.245, -0.043), (0.245, 0.043))
    ):
        outer.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, y, 0.013)),
            material=screw_black,
            name=f"mount_screw_{idx}",
        )

    intermediate = model.part("intermediate_member")
    # The child frame is located at the front mouth of the outer channel.
    # The rail extends backward from that frame so it remains retained when extended.
    intermediate.visual(
        Box((0.52, 0.046, 0.028)),
        origin=Origin(xyz=(-0.220, 0.0, 0.0)),
        material=bright_steel,
        name="intermediate_rail",
    )
    intermediate.visual(
        Box((0.050, 0.060, 0.040)),
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=bright_steel,
        name="front_pull",
    )
    intermediate.visual(
        Box((0.160, 0.012, 0.014)),
        origin=Origin(xyz=(-0.090, 0.0285, 0.020)),
        material=black_acetal,
        name="top_glide_0_0",
    )
    intermediate.visual(
        Box((0.160, 0.012, 0.014)),
        origin=Origin(xyz=(-0.090, -0.0285, 0.020)),
        material=black_acetal,
        name="top_glide_0_1",
    )
    intermediate.visual(
        Box((0.160, 0.014, 0.012)),
        origin=Origin(xyz=(-0.090, 0.017, -0.019)),
        material=black_acetal,
        name="bottom_glide_0_0",
    )
    intermediate.visual(
        Box((0.160, 0.014, 0.012)),
        origin=Origin(xyz=(-0.090, -0.017, -0.019)),
        material=black_acetal,
        name="bottom_glide_0_1",
    )
    intermediate.visual(
        Box((0.160, 0.012, 0.014)),
        origin=Origin(xyz=(-0.360, 0.0285, 0.020)),
        material=black_acetal,
        name="top_glide_1_0",
    )
    intermediate.visual(
        Box((0.160, 0.012, 0.014)),
        origin=Origin(xyz=(-0.360, -0.0285, 0.020)),
        material=black_acetal,
        name="top_glide_1_1",
    )
    intermediate.visual(
        Box((0.160, 0.014, 0.012)),
        origin=Origin(xyz=(-0.360, 0.017, -0.019)),
        material=black_acetal,
        name="bottom_glide_1_0",
    )
    intermediate.visual(
        Box((0.160, 0.014, 0.012)),
        origin=Origin(xyz=(-0.360, -0.017, -0.019)),
        material=black_acetal,
        name="bottom_glide_1_1",
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=intermediate,
        origin=Origin(xyz=(0.300, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.280),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_body")
    intermediate = object_model.get_part("intermediate_member")
    slide = object_model.get_articulation("outer_to_intermediate")

    ctx.expect_overlap(
        intermediate,
        outer,
        axes="x",
        elem_a="intermediate_rail",
        elem_b="bottom_web",
        min_overlap=0.45,
        name="collapsed rail is deeply retained in outer channel",
    )
    ctx.expect_gap(
        intermediate,
        outer,
        axis="z",
        positive_elem="intermediate_rail",
        negative_elem="bottom_web",
        min_gap=0.009,
        max_gap=0.014,
        name="rail clears the bottom web",
    )
    ctx.expect_gap(
        outer,
        intermediate,
        axis="z",
        positive_elem="top_lip_0",
        negative_elem="intermediate_rail",
        min_gap=0.010,
        max_gap=0.018,
        name="rail clears the retaining lip",
    )
    ctx.expect_gap(
        outer,
        intermediate,
        axis="y",
        positive_elem="side_wall_0",
        negative_elem="intermediate_rail",
        min_gap=0.014,
        max_gap=0.020,
        name="rail has side clearance to outer wall",
    )
    ctx.expect_contact(
        intermediate,
        outer,
        elem_a="top_glide_0_0",
        elem_b="top_lip_0",
        contact_tol=0.001,
        name="front top glide bears on retaining lip",
    )
    ctx.expect_contact(
        intermediate,
        outer,
        elem_a="bottom_glide_0_0",
        elem_b="bottom_web",
        contact_tol=0.001,
        name="front lower glide bears on channel web",
    )

    rest_position = ctx.part_world_position(intermediate)
    with ctx.pose({slide: 0.280}):
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="x",
            elem_a="intermediate_rail",
            elem_b="bottom_web",
            min_overlap=0.19,
            name="extended rail keeps clear retained overlap",
        )
        ctx.expect_contact(
            intermediate,
            outer,
            elem_a="top_glide_1_0",
            elem_b="top_lip_0",
            contact_tol=0.001,
            name="rear top glide supports the extended member",
        )
        extended_position = ctx.part_world_position(intermediate)

    ctx.check(
        "prismatic joint extends from the front",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 0.25,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


object_model = build_object_model()
