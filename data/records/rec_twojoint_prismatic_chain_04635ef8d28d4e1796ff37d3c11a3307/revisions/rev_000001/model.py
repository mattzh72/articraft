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
    model = ArticulatedObject(name="two_stage_transfer_axis")

    dark_anodized = model.material("dark_anodized", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_hardware", rgba=(0.015, 0.015, 0.018, 1.0))
    machined_aluminum = model.material("machined_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.50, 0.52, 0.54, 1.0))

    base = model.part("base")
    base.visual(
        Box((1.20, 0.36, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_anodized,
        name="base_plate",
    )
    for i, x in enumerate((-0.46, 0.46)):
        for j, y in enumerate((-0.135, 0.135)):
            base.visual(
                Box((0.16, 0.055, 0.020)),
                origin=Origin(xyz=(x, y, -0.009)),
                material=black,
                name=f"mount_foot_{i}_{j}",
            )
    base.visual(
        Box((1.05, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, -0.115, 0.0625)),
        material=ground_steel,
        name="base_rail_0",
    )
    base.visual(
        Box((1.05, 0.035, 0.025)),
        origin=Origin(xyz=(0.0, 0.115, 0.0625)),
        material=ground_steel,
        name="base_rail_1",
    )
    for i, y in enumerate((-0.115, 0.115)):
        for k, x in enumerate((-0.42, -0.20, 0.02, 0.24, 0.46)):
            base.visual(
                Cylinder(radius=0.010, length=0.004),
                origin=Origin(xyz=(x, y, 0.077)),
                material=black,
                name=f"base_rail_bolt_{i}_{k}",
            )
    base.visual(
        Box((1.12, 0.060, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=machined_aluminum,
        name="center_rib",
    )

    first_slide = model.part("first_slide")
    first_slide.visual(
        Box((0.42, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, -0.115, 0.009)),
        material=machined_aluminum,
        name="saddle_shoe_0",
    )
    first_slide.visual(
        Box((0.42, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.115, 0.009)),
        material=machined_aluminum,
        name="saddle_shoe_1",
    )
    first_slide.visual(
        Box((0.50, 0.270, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0355)),
        material=machined_aluminum,
        name="saddle_plate",
    )
    first_slide.visual(
        Box((0.46, 0.040, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_anodized,
        name="center_boss",
    )
    first_slide.visual(
        Box((0.46, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, -0.070, 0.062)),
        material=ground_steel,
        name="stage_rail_0",
    )
    first_slide.visual(
        Box((0.46, 0.025, 0.018)),
        origin=Origin(xyz=(0.0, 0.070, 0.062)),
        material=ground_steel,
        name="stage_rail_1",
    )
    for i, y in enumerate((-0.070, 0.070)):
        for k, x in enumerate((-0.16, 0.0, 0.16)):
            first_slide.visual(
                Cylinder(radius=0.007, length=0.003),
                origin=Origin(xyz=(x, y, 0.0725)),
                material=black,
                name=f"stage_rail_bolt_{i}_{k}",
            )

    terminal_slide = model.part("terminal_slide")
    terminal_slide.visual(
        Box((0.24, 0.045, 0.014)),
        origin=Origin(xyz=(0.0, -0.070, 0.007)),
        material=machined_aluminum,
        name="terminal_shoe_0",
    )
    terminal_slide.visual(
        Box((0.24, 0.045, 0.014)),
        origin=Origin(xyz=(0.0, 0.070, 0.007)),
        material=machined_aluminum,
        name="terminal_shoe_1",
    )
    terminal_slide.visual(
        Box((0.28, 0.160, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=machined_aluminum,
        name="terminal_body",
    )
    terminal_slide.visual(
        Box((0.025, 0.190, 0.090)),
        origin=Origin(xyz=(0.1525, 0.0, 0.059)),
        material=dark_anodized,
        name="end_plate",
    )
    for y in (-0.055, 0.055):
        terminal_slide.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(0.0, y, 0.046)),
            material=black,
            name=f"terminal_cap_screw_{'n' if y < 0 else 'p'}",
        )

    model.articulation(
        "base_to_first",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first_slide,
        origin=Origin(xyz=(-0.20, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.40, lower=0.0, upper=0.35),
    )
    model.articulation(
        "first_to_terminal",
        ArticulationType.PRISMATIC,
        parent=first_slide,
        child=terminal_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.30, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first_slide = object_model.get_part("first_slide")
    terminal_slide = object_model.get_part("terminal_slide")
    base_to_first = object_model.get_articulation("base_to_first")
    first_to_terminal = object_model.get_articulation("first_to_terminal")

    ctx.check(
        "two serial prismatic joints",
        base_to_first.articulation_type == ArticulationType.PRISMATIC
        and first_to_terminal.articulation_type == ArticulationType.PRISMATIC
        and base_to_first.parent == "base"
        and base_to_first.child == "first_slide"
        and first_to_terminal.parent == "first_slide"
        and first_to_terminal.child == "terminal_slide",
        details=(
            f"{base_to_first.articulation_type=} {first_to_terminal.articulation_type=} "
            f"{base_to_first.parent}->{base_to_first.child}, "
            f"{first_to_terminal.parent}->{first_to_terminal.child}"
        ),
    )

    ctx.expect_gap(
        first_slide,
        base,
        axis="z",
        positive_elem="saddle_shoe_0",
        negative_elem="base_rail_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="first slide shoe rides on base rail",
    )
    ctx.expect_overlap(
        first_slide,
        base,
        axes="xy",
        elem_a="saddle_shoe_0",
        elem_b="base_rail_0",
        min_overlap=0.030,
        name="first slide is seated over the base rail",
    )
    ctx.expect_gap(
        terminal_slide,
        first_slide,
        axis="z",
        positive_elem="terminal_shoe_0",
        negative_elem="stage_rail_0",
        max_gap=0.001,
        max_penetration=0.000001,
        name="terminal slide shoe rides on first slide rail",
    )
    ctx.expect_overlap(
        terminal_slide,
        first_slide,
        axes="xy",
        elem_a="terminal_shoe_0",
        elem_b="stage_rail_0",
        min_overlap=0.020,
        name="terminal slide is seated over the first slide rail",
    )

    rest_first = ctx.part_world_position(first_slide)
    rest_terminal = ctx.part_world_position(terminal_slide)
    with ctx.pose({base_to_first: 0.35, first_to_terminal: 0.16}):
        ctx.expect_gap(
            first_slide,
            base,
            axis="z",
            positive_elem="saddle_shoe_0",
            negative_elem="base_rail_0",
            max_gap=0.001,
            max_penetration=0.0,
            name="first slide stays on base rail at full travel",
        )
        ctx.expect_overlap(
            first_slide,
            base,
            axes="xy",
            elem_a="saddle_shoe_0",
            elem_b="base_rail_0",
            min_overlap=0.030,
            name="first slide keeps rail engagement at full travel",
        )
        ctx.expect_gap(
            terminal_slide,
            first_slide,
            axis="z",
            positive_elem="terminal_shoe_0",
            negative_elem="stage_rail_0",
            max_gap=0.001,
            max_penetration=0.000001,
            name="terminal slide stays on first slide rail at full travel",
        )
        ctx.expect_overlap(
            terminal_slide,
            first_slide,
            axes="xy",
            elem_a="terminal_shoe_0",
            elem_b="stage_rail_0",
            min_overlap=0.020,
            name="terminal slide keeps rail engagement at full travel",
        )
        extended_first = ctx.part_world_position(first_slide)
        extended_terminal = ctx.part_world_position(terminal_slide)

    ctx.check(
        "slides extend along the transfer axis",
        rest_first is not None
        and rest_terminal is not None
        and extended_first is not None
        and extended_terminal is not None
        and extended_first[0] > rest_first[0] + 0.30
        and extended_terminal[0] > rest_terminal[0] + 0.46,
        details=(
            f"first {rest_first}->{extended_first}, "
            f"terminal {rest_terminal}->{extended_terminal}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
