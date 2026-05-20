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
    model = ArticulatedObject(name="nested_two_stage_runner")

    zinc = model.material("brushed_zinc", rgba=(0.62, 0.66, 0.68, 1.0))
    dark = model.material("black_plastic", rgba=(0.02, 0.022, 0.025, 1.0))
    blue = model.material("blue_anodized_runner", rgba=(0.14, 0.28, 0.58, 1.0))
    steel = model.material("bright_steel", rgba=(0.80, 0.82, 0.80, 1.0))

    outer = model.part("outer_section")
    # Fixed C-channel with mounting flanges.  X is the extension direction.
    outer.visual(
        Box((0.72, 0.120, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=zinc,
        name="bottom_web",
    )
    for side, y in (("near", -0.056), ("far", 0.056)):
        outer.visual(
            Box((0.72, 0.008, 0.070)),
            origin=Origin(xyz=(0.0, y, 0.035)),
            material=zinc,
            name=f"{side}_side_wall",
        )
        outer.visual(
            Box((0.72, 0.026, 0.008)),
            origin=Origin(xyz=(0.0, y * 0.77, 0.066)),
            material=zinc,
            name=f"{side}_top_lip",
        )
        outer.visual(
            Box((0.72, 0.040, 0.006)),
            origin=Origin(xyz=(0.0, -0.080 if y < 0.0 else 0.080, 0.003)),
            material=zinc,
            name=f"{side}_mount_flange",
        )

    for i, x in enumerate((-0.27, -0.09, 0.09, 0.27)):
        for side, y in (("near", -0.080), ("far", 0.080)):
            outer.visual(
                Cylinder(radius=0.011, length=0.004),
                origin=Origin(xyz=(x, y, 0.0078)),
                material=dark,
                name=f"{side}_screw_{i}",
            )

    middle = model.part("middle_runner")
    # The middle runner is a nested slide member: the side bearing shoes touch
    # the inner faces of the fixed channel while a raised saddle carries the
    # output runner.
    middle.visual(
        Box((0.62, 0.076, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=blue,
        name="middle_web",
    )
    middle.visual(
        Box((0.62, 0.040, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0345)),
        material=blue,
        name="center_saddle",
    )
    for side, y in (("near", -0.045), ("far", 0.045)):
        middle.visual(
            Box((0.62, 0.014, 0.046)),
            origin=Origin(xyz=(0.0, y, 0.037)),
            material=blue,
            name=f"{side}_side_shoe",
        )
        middle.visual(
            Box((0.56, 0.004, 0.024)),
            origin=Origin(xyz=(0.0, y * 1.111, 0.035)),
            material=dark,
            name=f"{side}_wear_pad",
        )
    for x, label in ((-0.295, "rear"), (0.295, "front")):
        middle.visual(
            Box((0.030, 0.076, 0.012)),
            origin=Origin(xyz=(x, 0.0, 0.026)),
            material=dark,
            name=f"{label}_stop_pad",
        )

    output = model.part("output_runner")
    # Output runner rides on the middle saddle and carries the load mounting nose.
    output.visual(
        Box((0.56, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=steel,
        name="output_rail",
    )
    output.visual(
        Box((0.050, 0.052, 0.018)),
        origin=Origin(xyz=(0.305, 0.0, 0.047)),
        material=steel,
        name="mounting_nose",
    )
    for x, label in ((-0.20, "rear"), (0.18, "front")):
        output.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(x, 0.0, 0.058)),
            material=dark,
            name=f"{label}_threaded_insert",
        )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.24),
    )
    model.articulation(
        "middle_to_output",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=output,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.24),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_section")
    middle = object_model.get_part("middle_runner")
    output = object_model.get_part("output_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_output = object_model.get_articulation("middle_to_output")

    prismatic_count = sum(
        1
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.PRISMATIC
    )
    ctx.check(
        "two serial prismatic joints",
        prismatic_count == 2
        and outer_to_middle.parent == "outer_section"
        and outer_to_middle.child == "middle_runner"
        and middle_to_output.parent == "middle_runner"
        and middle_to_output.child == "output_runner",
        details=f"prismatic_count={prismatic_count}",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_output: 0.0}):
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.001,
            name="middle runner nests inside fixed channel",
        )
        ctx.expect_within(
            output,
            middle,
            axes="yz",
            margin=0.002,
            name="output runner is carried inside middle runner envelope",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.55,
            name="collapsed outer-middle length overlap",
        )
        ctx.expect_overlap(
            output,
            middle,
            axes="x",
            min_overlap=0.50,
            name="collapsed middle-output length overlap",
        )
        ctx.expect_overlap(
            output,
            outer,
            axes="x",
            min_overlap=0.50,
            name="collapsed all sections overlap",
        )
        collapsed_output_pos = ctx.part_world_position(output)

    with ctx.pose({outer_to_middle: 0.24, middle_to_output: 0.24}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.36,
            name="extended outer-middle retained overlap",
        )
        ctx.expect_overlap(
            output,
            middle,
            axes="x",
            min_overlap=0.30,
            name="extended middle-output retained overlap",
        )
        ctx.expect_overlap(
            output,
            outer,
            axes="x",
            min_overlap=0.12,
            name="extended all sections still overlap",
        )
        extended_middle_pos = ctx.part_world_position(middle)
        extended_output_pos = ctx.part_world_position(output)

    ctx.check(
        "output advances by combined travel",
        collapsed_output_pos is not None
        and extended_middle_pos is not None
        and extended_output_pos is not None
        and extended_middle_pos[0] > 0.23
        and extended_output_pos[0] > extended_middle_pos[0] + 0.23,
        details=(
            f"collapsed_output={collapsed_output_pos}, "
            f"extended_middle={extended_middle_pos}, extended_output={extended_output_pos}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
