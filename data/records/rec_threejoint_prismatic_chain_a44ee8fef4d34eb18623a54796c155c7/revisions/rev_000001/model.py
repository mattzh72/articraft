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
    model = ArticulatedObject(name="triple_runner_transfer_stack")

    anodized_blue = model.material("anodized_blue", color=(0.10, 0.22, 0.38, 1.0))
    anodized_clear = model.material("brushed_aluminum", color=(0.70, 0.72, 0.70, 1.0))
    dark_steel = model.material("dark_steel", color=(0.07, 0.075, 0.08, 1.0))
    black = model.material("black_fasteners", color=(0.01, 0.01, 0.012, 1.0))
    output_orange = model.material("output_stage_orange", color=(0.95, 0.48, 0.12, 1.0))

    # Root: a floor-mounted open U-channel.  It is deliberately longer and
    # wider than the nested runners, with a pair of exposed lower guide rails.
    outer = model.part("outer_channel")
    outer.visual(
        Box((1.28, 0.36, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=anodized_blue,
        name="base_plate",
    )
    for i, y in enumerate((-0.166, 0.166)):
        outer.visual(
            Box((1.20, 0.028, 0.105)),
            origin=Origin(xyz=(0.0, y, 0.0775)),
            material=anodized_blue,
            name=f"side_wall_{i}",
        )
    for rail_name, y in (("outer_rail_0", -0.09), ("outer_rail_1", 0.09)):
        outer.visual(
            Box((1.15, 0.022, 0.024)),
            origin=Origin(xyz=(0.0, y, 0.037)),
            material=dark_steel,
            name=rail_name,
        )
    outer.visual(
        Box((0.045, 0.34, 0.090)),
        origin=Origin(xyz=(-0.62, 0.0, 0.070)),
        material=anodized_blue,
        name="rear_stop",
    )
    for ix, x in enumerate((-0.47, 0.47)):
        for iy, y in enumerate((-0.135, 0.135)):
            outer.visual(
                Box((0.13, 0.060, 0.020)),
                origin=Origin(xyz=(x, y, -0.010)),
                material=dark_steel,
                name=f"mount_foot_{ix}_{iy}",
            )

    # First runner: a broad carriage riding on the outer channel rails, with
    # its own smaller rail set on top for the second moving stage.
    first = model.part("first_runner")
    first.visual(
        Box((0.62, 0.22, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=anodized_clear,
        name="first_carriage",
    )
    for shoe_name, y in (("first_shoe_0", -0.09), ("first_shoe_1", 0.09)):
        first.visual(
            Box((0.58, 0.038, 0.020)),
            origin=Origin(xyz=(0.0, y, -0.005)),
            material=dark_steel,
            name=shoe_name,
        )
    for rail_name, y in (("first_top_rail_0", -0.075), ("first_top_rail_1", 0.075)):
        first.visual(
            Box((0.56, 0.018, 0.030)),
            origin=Origin(xyz=(0.0, y, 0.045)),
            material=dark_steel,
            name=rail_name,
        )
    for x in (-0.22, 0.22):
        for y in (-0.070, 0.070):
            first.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, y, 0.032)),
                material=black,
                name=f"first_bolt_{x:+.2f}_{y:+.2f}",
            )

    # Second runner: a narrower slide carried by the first runner.
    second = model.part("second_runner")
    second.visual(
        Box((0.42, 0.145, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
        material=anodized_blue,
        name="second_carriage",
    )
    for shoe_name, y in (("second_shoe_0", -0.075), ("second_shoe_1", 0.075)):
        second.visual(
            Box((0.39, 0.026, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.005)),
            material=dark_steel,
            name=shoe_name,
        )
    for rail_name, y in (("second_top_rail_0", -0.045), ("second_top_rail_1", 0.045)):
        second.visual(
            Box((0.34, 0.014, 0.023)),
            origin=Origin(xyz=(0.0, y, 0.0405)),
            material=dark_steel,
            name=rail_name,
        )
    for x in (-0.15, 0.15):
        for y in (-0.045, 0.045):
            second.visual(
                Cylinder(radius=0.0075, length=0.004),
                origin=Origin(xyz=(x, y, 0.031)),
                material=black,
                name=f"second_bolt_{x:+.2f}_{y:+.2f}",
            )

    # Final output stage: a compact tooling/payload plate on the last slide.
    output = model.part("output_stage")
    output.visual(
        Box((0.24, 0.12, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=output_orange,
        name="tooling_plate",
    )
    output.visual(
        Box((0.12, 0.060, 0.035)),
        origin=Origin(xyz=(0.030, 0.0, 0.0375)),
        material=output_orange,
        name="raised_pad",
    )
    for shoe_name, y in (("output_shoe_0", -0.045), ("output_shoe_1", 0.045)):
        output.visual(
            Box((0.20, 0.020, 0.014)),
            origin=Origin(xyz=(0.0, y, -0.007)),
            material=dark_steel,
            name=shoe_name,
        )
    for x in (-0.075, 0.075):
        for y in (-0.030, 0.030):
            output.visual(
                Cylinder(radius=0.006, length=0.003),
                origin=Origin(xyz=(x, y, 0.0215)),
                material=black,
                name=f"output_bolt_{x:+.3f}_{y:+.3f}",
            )

    # Three serial prismatic joints.  Each child frame is placed at its nested
    # carriage center at q=0 and travels along +X; the upper limits preserve
    # substantial runner-to-rail engagement while the serial stack produces a
    # long transfer stroke at the orange output plate.
    model.articulation(
        "outer_to_first",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=first,
        origin=Origin(xyz=(-0.25, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.45, lower=0.0, upper=0.45),
    )
    model.articulation(
        "first_to_second",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(-0.12, 0.0, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.40, lower=0.0, upper=0.30),
    )
    model.articulation(
        "second_to_output",
        ArticulationType.PRISMATIC,
        parent=second,
        child=output,
        origin=Origin(xyz=(-0.04, 0.0, 0.066)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.18),
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

    outer = object_model.get_part("outer_channel")
    first = object_model.get_part("first_runner")
    second = object_model.get_part("second_runner")
    output = object_model.get_part("output_stage")
    j1 = object_model.get_articulation("outer_to_first")
    j2 = object_model.get_articulation("first_to_second")
    j3 = object_model.get_articulation("second_to_output")

    ctx.check(
        "three serial prismatic joints",
        len(object_model.articulations) == 3
        and all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (j1, j2, j3)
        ),
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )

    # Each runner is centered across the carrying channel and has bearing shoes
    # touching the rail it rides on, so the visual support path is grounded.
    ctx.expect_within(
        first,
        outer,
        axes="y",
        margin=0.002,
        name="first runner fits inside the outer channel width",
    )
    ctx.expect_gap(
        first,
        outer,
        axis="z",
        positive_elem="first_shoe_0",
        negative_elem="outer_rail_0",
        max_penetration=0.000001,
        max_gap=0.00001,
        name="first shoe touches outer rail",
    )
    ctx.expect_gap(
        second,
        first,
        axis="z",
        positive_elem="second_shoe_0",
        negative_elem="first_top_rail_0",
        max_penetration=0.000001,
        max_gap=0.00001,
        name="second shoe touches first rail",
    )
    ctx.expect_gap(
        output,
        second,
        axis="z",
        positive_elem="output_shoe_0",
        negative_elem="second_top_rail_0",
        max_penetration=0.000001,
        max_gap=0.00001,
        name="output shoe touches second rail",
    )

    rest_pos = ctx.part_world_position(output)
    with ctx.pose({j1: 0.45, j2: 0.30, j3: 0.18}):
        extended_pos = ctx.part_world_position(output)
        ctx.expect_overlap(
            first,
            outer,
            axes="x",
            elem_a="first_shoe_0",
            elem_b="outer_rail_0",
            min_overlap=0.25,
            name="first runner remains engaged with outer rail at full stroke",
        )
        ctx.expect_overlap(
            second,
            first,
            axes="x",
            elem_a="second_shoe_0",
            elem_b="first_top_rail_0",
            min_overlap=0.18,
            name="second runner remains engaged with first rail at full stroke",
        )
        ctx.expect_overlap(
            output,
            second,
            axes="x",
            elem_a="output_shoe_0",
            elem_b="second_top_rail_0",
            min_overlap=0.08,
            name="output stage remains engaged with second rail at full stroke",
        )

    ctx.check(
        "serial stack extends the output stage",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.85,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
