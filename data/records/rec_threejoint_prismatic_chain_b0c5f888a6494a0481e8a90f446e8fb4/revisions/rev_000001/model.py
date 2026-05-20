from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_three_stage_linear_stack")

    anodized_black = Material("anodized_black", rgba=(0.035, 0.038, 0.043, 1.0))
    dark_edge = Material("dark_edge", rgba=(0.010, 0.012, 0.014, 1.0))
    satin_aluminum = Material("satin_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    blue_anodized = Material("blue_anodized", rgba=(0.10, 0.28, 0.55, 1.0))
    orange_end = Material("orange_end", rgba=(0.95, 0.38, 0.08, 1.0))
    polymer = Material("black_polymer", rgba=(0.025, 0.025, 0.022, 1.0))
    screw = Material("brushed_screw_heads", rgba=(0.42, 0.43, 0.42, 1.0))

    outer = model.part("outer_body")
    outer.visual(
        Box((0.500, 0.140, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=anodized_black,
        name="base_plate",
    )
    for index, y in enumerate((-0.061, 0.061)):
        outer.visual(
            Box((0.465, 0.018, 0.050)),
            origin=Origin(xyz=(0.0, y, 0.043)),
            material=anodized_black,
            name=f"guide_rail_{index}",
        )
        outer.visual(
            Box((0.090, 0.019, 0.004)),
            origin=Origin(xyz=(-0.170, y, 0.070)),
            material=screw,
            name=f"rail_wear_strip_{index}_0",
        )
        outer.visual(
            Box((0.090, 0.019, 0.004)),
            origin=Origin(xyz=(0.145, y, 0.070)),
            material=screw,
            name=f"rail_wear_strip_{index}_1",
        )
    outer.visual(
        Box((0.030, 0.140, 0.044)),
        origin=Origin(xyz=(-0.250, 0.0, 0.040)),
        material=dark_edge,
        name="rear_stop",
    )
    outer.visual(
        Box((0.030, 0.140, 0.026)),
        origin=Origin(xyz=(0.250, 0.0, 0.031)),
        material=dark_edge,
        name="front_stop",
    )

    middle = model.part("middle_carriage")
    for index, y in enumerate((-0.048, 0.048)):
        middle.visual(
            Box((0.330, 0.020, 0.012)),
            origin=Origin(xyz=(0.0, y, 0.006)),
            material=polymer,
            name=f"lower_pad_{index}",
        )
    middle.visual(
        Box((0.350, 0.095, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=satin_aluminum,
        name="main_plate",
    )
    middle.visual(
        Box((0.350, 0.020, 0.018)),
        origin=Origin(xyz=(-0.168, 0.0, 0.041)),
        material=satin_aluminum,
        name="rear_bridge",
    )
    middle.visual(
        Box((0.350, 0.020, 0.018)),
        origin=Origin(xyz=(0.168, 0.0, 0.041)),
        material=satin_aluminum,
        name="front_bridge",
    )
    for index, y in enumerate((-0.038, 0.038)):
        middle.visual(
            Box((0.305, 0.012, 0.022)),
            origin=Origin(xyz=(0.0, y, 0.043)),
            material=satin_aluminum,
            name=f"upper_rail_{index}",
        )
        middle.visual(
            Box((0.055, 0.014, 0.004)),
            origin=Origin(xyz=(-0.095, y, 0.056)),
            material=screw,
            name=f"upper_wear_strip_{index}_0",
        )
        middle.visual(
            Box((0.055, 0.014, 0.004)),
            origin=Origin(xyz=(0.115, y, 0.056)),
            material=screw,
            name=f"upper_wear_strip_{index}_1",
        )

    inner = model.part("inner_carriage")
    for index, y in enumerate((-0.034, 0.034)):
        inner.visual(
            Box((0.235, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, y, 0.005)),
            material=polymer,
            name=f"lower_pad_{index}",
        )
    inner.visual(
        Box((0.255, 0.060, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=blue_anodized,
        name="main_plate",
    )
    inner.visual(
        Box((0.255, 0.014, 0.014)),
        origin=Origin(xyz=(-0.120, 0.0, 0.033)),
        material=blue_anodized,
        name="rear_bridge",
    )
    inner.visual(
        Box((0.255, 0.014, 0.014)),
        origin=Origin(xyz=(0.120, 0.0, 0.033)),
        material=blue_anodized,
        name="front_bridge",
    )
    for index, y in enumerate((-0.025, 0.025)):
        inner.visual(
            Box((0.215, 0.010, 0.016)),
            origin=Origin(xyz=(0.0, y, 0.034)),
            material=blue_anodized,
            name=f"upper_rail_{index}",
        )

    terminal = model.part("terminal_slide")
    for index, y in enumerate((-0.022, 0.022)):
        terminal.visual(
            Box((0.140, 0.010, 0.008)),
            origin=Origin(xyz=(0.0, y, 0.004)),
            material=polymer,
            name=f"lower_pad_{index}",
        )
    terminal.visual(
        Box((0.165, 0.038, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_aluminum,
        name="slide_tongue",
    )
    terminal.visual(
        Box((0.022, 0.048, 0.026)),
        origin=Origin(xyz=(0.083, 0.0, 0.021)),
        material=orange_end,
        name="end_tab",
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(-0.055, 0.0, 0.068)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.160),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(-0.025, 0.0, 0.054)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.30, lower=0.0, upper=0.120),
    )
    model.articulation(
        "inner_to_terminal",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=terminal,
        origin=Origin(xyz=(-0.015, 0.0, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.25, lower=0.0, upper=0.080),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [
        object_model.get_articulation("outer_to_middle"),
        object_model.get_articulation("middle_to_inner"),
        object_model.get_articulation("inner_to_terminal"),
    ]
    ctx.check(
        "three serial prismatic joints",
        len(joints) == 3
        and all(joint.articulation_type == ArticulationType.PRISMATIC for joint in joints)
        and [joint.parent for joint in joints] == ["outer_body", "middle_carriage", "inner_carriage"]
        and [joint.child for joint in joints] == ["middle_carriage", "inner_carriage", "terminal_slide"],
        details=f"joints={[joint.name for joint in joints]}",
    )

    outer = object_model.get_part("outer_body")
    middle = object_model.get_part("middle_carriage")
    inner = object_model.get_part("inner_carriage")
    terminal = object_model.get_part("terminal_slide")

    ctx.expect_contact(
        middle,
        outer,
        elem_a="lower_pad_1",
        elem_b="guide_rail_1",
        name="middle carriage bears on outer guide rail",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="lower_pad_1",
        elem_b="upper_rail_1",
        name="inner carriage bears on middle guide rail",
    )
    ctx.expect_contact(
        terminal,
        inner,
        elem_a="lower_pad_1",
        elem_b="upper_rail_1",
        name="terminal slide bears on inner guide rail",
    )

    rest_terminal = ctx.part_world_position(terminal)
    with ctx.pose(
        {
            joints[0]: joints[0].motion_limits.upper,
            joints[1]: joints[1].motion_limits.upper,
            joints[2]: joints[2].motion_limits.upper,
        }
    ):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.25,
            elem_a="lower_pad_1",
            elem_b="guide_rail_1",
            name="extended middle remains inserted in outer body",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.12,
            elem_a="lower_pad_1",
            elem_b="upper_rail_1",
            name="extended inner remains inserted in middle carriage",
        )
        ctx.expect_overlap(
            terminal,
            inner,
            axes="x",
            min_overlap=0.06,
            elem_a="lower_pad_1",
            elem_b="upper_rail_1",
            name="extended terminal slide remains inserted in inner carriage",
        )
        extended_terminal = ctx.part_world_position(terminal)

    ctx.check(
        "terminal slide extends along stack axis",
        rest_terminal is not None
        and extended_terminal is not None
        and extended_terminal[0] > rest_terminal[0] + 0.30,
        details=f"rest={rest_terminal}, extended={extended_terminal}",
    )

    return ctx.report()


object_model = build_object_model()
