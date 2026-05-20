from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="linear_positioning_module")

    model.material("anodized_black", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("ground_steel", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("dark_steel", rgba=(0.15, 0.16, 0.17, 1.0))
    model.material("rubber_stop", rgba=(0.03, 0.03, 0.025, 1.0))
    model.material("warning_orange", rgba=(0.95, 0.42, 0.05, 1.0))

    base_rail = model.part("base_rail")
    base_rail.visual(
        Box((1.20, 0.26, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="anodized_black",
        name="base_plate",
    )
    base_rail.visual(
        Box((1.10, 0.085, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material="dark_steel",
        name="center_rib",
    )
    base_rail.visual(
        Box((1.05, 0.034, 0.045)),
        origin=Origin(xyz=(0.0, -0.073, 0.0615)),
        material="ground_steel",
        name="guide_rail_0",
    )
    base_rail.visual(
        Box((1.05, 0.034, 0.045)),
        origin=Origin(xyz=(0.0, 0.073, 0.0615)),
        material="ground_steel",
        name="guide_rail_1",
    )
    for label, x, bumper_x in (("low", -0.570, -0.536), ("high", 0.570, 0.536)):
        base_rail.visual(
            Box((0.050, 0.205, 0.120)),
            origin=Origin(xyz=(x, 0.0, 0.098)),
            material="brushed_aluminum",
            name=f"{label}_end_block",
        )
        base_rail.visual(
            Box((0.018, 0.160, 0.060)),
            origin=Origin(xyz=(bumper_x, 0.0, 0.106)),
            material="rubber_stop",
            name=f"{label}_bumper",
        )
    for x in (-0.43, -0.14, 0.14, 0.43):
        base_rail.visual(
            Cylinder(radius=0.010, length=0.006),
            origin=Origin(xyz=(x, 0.0, 0.077)),
            material="dark_steel",
            name=f"rail_screw_{x:+.2f}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.210, 0.205, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="brushed_aluminum",
        name="saddle_plate",
    )
    for index, y in enumerate((-0.108, 0.108)):
        carriage.visual(
            Box((0.180, 0.036, 0.070)),
            origin=Origin(xyz=(0.0, -0.109 if y < 0.0 else 0.109, -0.008)),
            material="dark_steel",
            name=f"slide_shoe_{index}",
        )
    for index, y in enumerate((-0.073, 0.073)):
        carriage.visual(
            Box((0.160, 0.044, 0.018)),
            origin=Origin(xyz=(0.0, y, -0.047)),
            material="ground_steel",
            name=f"bearing_pad_{index}",
        )
    carriage.visual(
        Box((0.110, 0.160, 0.036)),
        origin=Origin(xyz=(0.020, 0.0, 0.043)),
        material="brushed_aluminum",
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.104, 0.104)):
        carriage.visual(
            Box((0.080, 0.032, 0.190)),
            origin=Origin(xyz=(0.020, y, 0.115)),
            material="brushed_aluminum",
            name=f"pivot_cheek_{index}",
        )
        carriage.visual(
            Cylinder(radius=0.045, length=0.018),
            origin=Origin(xyz=(0.020, -0.124 if y < 0.0 else 0.124, 0.180), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="dark_steel",
            name=f"outer_bushing_{index}",
        )
        carriage.visual(
            Box((0.040, 0.020, 0.036)),
            origin=Origin(xyz=(0.070, -0.124 if y < 0.0 else 0.124, 0.222)),
            material="warning_orange",
            name=f"rotary_stop_{index}",
        )

    elbow_bracket = model.part("elbow_bracket")
    elbow_bracket.visual(
        Cylinder(radius=0.052, length=0.145),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="pivot_hub",
    )
    for index, y in enumerate((-0.081, 0.081)):
        elbow_bracket.visual(
            Cylinder(radius=0.062, length=0.0155),
            origin=Origin(xyz=(0.0, -0.08025 if y < 0.0 else 0.08025, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
            material="brushed_aluminum",
            name=f"pivot_cap_{index}",
        )
    elbow_bracket.visual(
        Box((0.300, 0.085, 0.050)),
        origin=Origin(xyz=(0.150, 0.0, 0.0)),
        material="brushed_aluminum",
        name="elbow_arm",
    )
    for index, y in enumerate((-0.049, 0.049)):
        elbow_bracket.visual(
            Box((0.250, 0.018, 0.026)),
            origin=Origin(xyz=(0.165, y, 0.035)),
            material="brushed_aluminum",
            name=f"side_web_{index}",
        )
    elbow_bracket.visual(
        Box((0.200, 0.130, 0.026)),
        origin=Origin(xyz=(0.390, 0.0, 0.035)),
        material="dark_steel",
        name="lower_nose_sleeve",
    )
    elbow_bracket.visual(
        Box((0.200, 0.130, 0.026)),
        origin=Origin(xyz=(0.390, 0.0, 0.115)),
        material="dark_steel",
        name="upper_nose_sleeve",
    )
    for name, y in (("side_nose_sleeve_0", -0.071), ("side_nose_sleeve_1", 0.071)):
        elbow_bracket.visual(
            Box((0.200, 0.026, 0.106)),
            origin=Origin(xyz=(0.390, y, 0.075)),
            material="dark_steel",
            name=name,
        )
    elbow_bracket.visual(
        Box((0.020, 0.070, 0.024)),
        origin=Origin(xyz=(0.302, 0.0, 0.140)),
        material="warning_orange",
        name="rear_nose_stop",
    )
    elbow_bracket.visual(
        Box((0.020, 0.070, 0.024)),
        origin=Origin(xyz=(0.478, 0.0, 0.140)),
        material="warning_orange",
        name="front_nose_stop",
    )

    tool_nose = model.part("tool_nose")
    tool_nose.visual(
        Box((0.460, 0.045, 0.054)),
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        material="ground_steel",
        name="telescoping_ram",
    )
    tool_nose.visual(
        Box((0.040, 0.075, 0.055)),
        origin=Origin(xyz=(-0.108, 0.0, 0.0)),
        material="warning_orange",
        name="rear_stop_tab",
    )
    tool_nose.visual(
        Cylinder(radius=0.025, length=0.070),
        origin=Origin(xyz=(0.385, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="tool_holder",
    )
    tool_nose.visual(
        Cylinder(radius=0.012, length=0.044),
        origin=Origin(xyz=(0.437, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="probe_tip",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base_rail,
        child=carriage,
        origin=Origin(xyz=(-0.350, 0.0, 0.140)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.35, lower=0.0, upper=0.550),
    )
    model.articulation(
        "carriage_to_elbow",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=elbow_bracket,
        origin=Origin(xyz=(0.020, 0.0, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=-0.18, upper=0.65),
    )
    model.articulation(
        "elbow_to_nose",
        ArticulationType.PRISMATIC,
        parent=elbow_bracket,
        child=tool_nose,
        origin=Origin(xyz=(0.280, 0.0, 0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.100),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_rail = object_model.get_part("base_rail")
    carriage = object_model.get_part("carriage")
    elbow_bracket = object_model.get_part("elbow_bracket")
    tool_nose = object_model.get_part("tool_nose")
    carriage_slide = object_model.get_articulation("base_to_carriage")
    elbow_pivot = object_model.get_articulation("carriage_to_elbow")
    nose_slide = object_model.get_articulation("elbow_to_nose")

    ctx.expect_gap(
        carriage,
        base_rail,
        axis="z",
        positive_elem="saddle_plate",
        negative_elem="guide_rail_1",
        min_gap=0.025,
        name="carriage saddle clears the fixed guide rail",
    )
    ctx.expect_within(
        tool_nose,
        elbow_bracket,
        axes="y",
        inner_elem="telescoping_ram",
        outer_elem="upper_nose_sleeve",
        margin=0.0,
        name="nose ram remains centered laterally in the guide sleeve",
    )
    ctx.expect_gap(
        elbow_bracket,
        tool_nose,
        axis="z",
        positive_elem="upper_nose_sleeve",
        negative_elem="telescoping_ram",
        min_gap=-0.0005,
        max_gap=0.002,
        name="upper sleeve bears on the guided nose ram",
    )
    ctx.expect_overlap(
        tool_nose,
        elbow_bracket,
        axes="x",
        elem_a="telescoping_ram",
        elem_b="upper_nose_sleeve",
        min_overlap=0.150,
        name="collapsed nose keeps retained insertion inside sleeve",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({carriage_slide: 0.550}):
        extended_carriage = ctx.part_world_position(carriage)
        ctx.expect_gap(
            carriage,
            base_rail,
            axis="z",
            positive_elem="saddle_plate",
            negative_elem="guide_rail_1",
            min_gap=0.025,
            name="travel end keeps carriage saddle clear above rail",
        )

    ctx.check(
        "carriage slides along rail",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.50
        and abs(extended_carriage[1] - rest_carriage[1]) < 0.001,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    rest_elbow = ctx.part_world_position(tool_nose)
    with ctx.pose({elbow_pivot: 0.65}):
        raised_elbow = ctx.part_world_position(tool_nose)
        ctx.expect_gap(
            elbow_bracket,
            carriage,
            axis="z",
            positive_elem="elbow_arm",
            negative_elem="yoke_bridge",
            min_gap=0.040,
            name="raised elbow arm clears the support yoke bridge",
        )
    with ctx.pose({elbow_pivot: -0.18}):
        ctx.expect_gap(
            tool_nose,
            base_rail,
            axis="z",
            min_gap=0.030,
            name="lower elbow limit keeps nose above rail and covers",
        )

    ctx.check(
        "elbow pivot raises nose at upper limit",
        rest_elbow is not None and raised_elbow is not None and raised_elbow[2] > rest_elbow[2] + 0.08,
        details=f"rest={rest_elbow}, raised={raised_elbow}",
    )

    rest_nose = ctx.part_world_position(tool_nose)
    with ctx.pose({nose_slide: 0.100}):
        extended_nose = ctx.part_world_position(tool_nose)
        ctx.expect_overlap(
            tool_nose,
            elbow_bracket,
            axes="x",
            elem_a="telescoping_ram",
            elem_b="upper_nose_sleeve",
            min_overlap=0.120,
            name="extended nose remains retained in sleeve",
        )
        ctx.expect_gap(
            elbow_bracket,
            tool_nose,
            axis="x",
            positive_elem="rear_nose_stop",
            negative_elem="rear_stop_tab",
            min_gap=-0.0005,
            max_gap=0.002,
            name="rear stop tab meets sleeve stop at full nose travel",
        )

    ctx.check(
        "tool nose extends along local module axis",
        rest_nose is not None and extended_nose is not None and extended_nose[0] > rest_nose[0] + 0.090,
        details=f"rest={rest_nose}, extended={extended_nose}",
    )

    return ctx.report()


object_model = build_object_model()
