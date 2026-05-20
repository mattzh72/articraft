from __future__ import annotations

import math

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
    model = ArticulatedObject(name="precision_extension_module")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.07, 0.08, 0.09, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.64, 0.66, 0.67, 1.0))
    hardcoat = model.material("hardcoat_carriage", rgba=(0.22, 0.25, 0.27, 1.0))
    tool_steel = model.material("tool_steel", rgba=(0.42, 0.43, 0.43, 1.0))
    bronze = model.material("bronze_wear_strip", rgba=(0.72, 0.53, 0.25, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))

    def add_screw_z(part, name: str, x: float, y: float, top_z: float, radius: float, length: float) -> None:
        # A very small sink keeps the cap screw visibly seated in the machined face.
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(x, y, top_z + length * 0.5 - 0.0007)),
            material=tool_steel,
            name=name,
        )

    outer = model.part("outer_body")
    outer.visual(
        Box((0.88, 0.20, 0.024)),
        origin=Origin(xyz=(0.44, 0.0, 0.020)),
        material=anodized,
        name="outer_bed",
    )
    for i, x in enumerate((0.12, 0.76)):
        for j, y in enumerate((-0.074, 0.074)):
            outer.visual(
                Box((0.11, 0.060, 0.012)),
                origin=Origin(xyz=(x, y, 0.006)),
                material=rubber,
                name=f"ground_foot_{i}_{j}",
            )
    outer.visual(
        Box((0.84, 0.026, 0.066)),
        origin=Origin(xyz=(0.43, -0.087, 0.064)),
        material=anodized,
        name="outer_side_rail_0",
    )
    outer.visual(
        Box((0.76, 0.008, 0.018)),
        origin=Origin(xyz=(0.47, -0.046, 0.050)),
        material=bronze,
        name="outer_wear_strip_0",
    )
    outer.visual(
        Box((0.74, 0.014, 0.017)),
        origin=Origin(xyz=(0.48, -0.052, 0.1045)),
        material=anodized,
        name="outer_keeper_0",
    )
    outer.visual(
        Box((0.74, 0.022, 0.017)),
        origin=Origin(xyz=(0.48, -0.064, 0.1045)),
        material=anodized,
        name="outer_keeper_web_0",
    )
    outer.visual(
        Box((0.84, 0.026, 0.066)),
        origin=Origin(xyz=(0.43, 0.087, 0.064)),
        material=anodized,
        name="outer_side_rail_1",
    )
    outer.visual(
        Box((0.76, 0.008, 0.018)),
        origin=Origin(xyz=(0.47, 0.046, 0.050)),
        material=bronze,
        name="outer_wear_strip_1",
    )
    outer.visual(
        Box((0.74, 0.014, 0.017)),
        origin=Origin(xyz=(0.48, 0.052, 0.1045)),
        material=anodized,
        name="outer_keeper_1",
    )
    outer.visual(
        Box((0.74, 0.022, 0.017)),
        origin=Origin(xyz=(0.48, 0.064, 0.1045)),
        material=anodized,
        name="outer_keeper_web_1",
    )
    for i, x in enumerate((0.055, 0.835)):
        for side, y in enumerate((-0.060, 0.060)):
            outer.visual(
                Box((0.030, 0.030, 0.038)),
                origin=Origin(xyz=(x, y, 0.055)),
                material=rubber,
                name=f"outer_buffer_{i}_{side}",
            )
    for side, y in enumerate((-0.087, 0.087)):
        for i, x in enumerate((0.13, 0.32, 0.53, 0.72)):
            add_screw_z(outer, f"outer_screw_{side}_{i}", x, y, 0.097, 0.0075, 0.005)

    middle = model.part("middle_carriage")
    middle.visual(
        Box((0.66, 0.080, 0.028)),
        origin=Origin(xyz=(0.09, 0.0, 0.046)),
        material=brushed,
        name="middle_beam",
    )
    middle.visual(
        Box((0.62, 0.010, 0.018)),
        origin=Origin(xyz=(0.09, -0.032, 0.069)),
        material=hardcoat,
        name="middle_guide_strip_0",
    )
    add_screw_z(middle, "middle_screw_0_0", -0.14, -0.032, 0.078, 0.0047, 0.004)
    add_screw_z(middle, "middle_screw_0_1", 0.07, -0.032, 0.078, 0.0047, 0.004)
    add_screw_z(middle, "middle_screw_0_2", 0.28, -0.032, 0.078, 0.0047, 0.004)
    middle.visual(
        Box((0.62, 0.010, 0.018)),
        origin=Origin(xyz=(0.09, 0.032, 0.069)),
        material=hardcoat,
        name="middle_guide_strip_1",
    )
    add_screw_z(middle, "middle_screw_1_0", -0.14, 0.032, 0.078, 0.0047, 0.004)
    add_screw_z(middle, "middle_screw_1_1", 0.07, 0.032, 0.078, 0.0047, 0.004)
    add_screw_z(middle, "middle_screw_1_2", 0.28, 0.032, 0.078, 0.0047, 0.004)
    for side, y in enumerate((-0.026, 0.026)):
        middle.visual(
            Box((0.024, 0.010, 0.026)),
            origin=Origin(xyz=(0.425, y, 0.066)),
            material=rubber,
            name=f"middle_end_stop_{side}",
        )

    inner = model.part("inner_carriage")
    inner.visual(
        Box((0.46, 0.040, 0.020)),
        origin=Origin(xyz=(0.05, 0.0, 0.070)),
        material=hardcoat,
        name="inner_beam",
    )
    inner.visual(
        Box((0.42, 0.006, 0.014)),
        origin=Origin(xyz=(0.05, -0.017, 0.087)),
        material=brushed,
        name="inner_guide_strip_0",
    )
    add_screw_z(inner, "inner_screw_0_0", -0.09, -0.017, 0.094, 0.0024, 0.003)
    add_screw_z(inner, "inner_screw_0_1", 0.11, -0.017, 0.094, 0.0024, 0.003)
    add_screw_z(inner, "inner_screw_0_2", 0.24, -0.017, 0.094, 0.0024, 0.003)
    inner.visual(
        Box((0.42, 0.006, 0.014)),
        origin=Origin(xyz=(0.05, 0.017, 0.087)),
        material=brushed,
        name="inner_guide_strip_1",
    )
    add_screw_z(inner, "inner_screw_1_0", -0.09, 0.017, 0.094, 0.0024, 0.003)
    add_screw_z(inner, "inner_screw_1_1", 0.11, 0.017, 0.094, 0.0024, 0.003)
    add_screw_z(inner, "inner_screw_1_2", 0.24, 0.017, 0.094, 0.0024, 0.003)
    for side, y in enumerate((-0.0235, 0.0235)):
        inner.visual(
            Box((0.018, 0.008, 0.018)),
            origin=Origin(xyz=(0.283, y, 0.078)),
            material=rubber,
            name=f"inner_end_stop_{side}",
        )

    nose = model.part("nose_plate")
    nose.visual(
        Box((0.19, 0.026, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, 0.087)),
        material=tool_steel,
        name="nose_tongue",
    )
    nose.visual(
        Box((0.030, 0.075, 0.060)),
        origin=Origin(xyz=(0.144, 0.0, 0.089)),
        material=brushed,
        name="front_plate",
    )
    for i, y in enumerate((-0.023, 0.023)):
        for j, z in enumerate((0.076, 0.102)):
            nose.visual(
                Cylinder(radius=0.0040, length=0.006),
                origin=Origin(
                    xyz=(0.1605, y, z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=tool_steel,
                name=f"nose_screw_{i}_{j}",
            )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.38, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.18, lower=0.0, upper=0.28),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.24, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.16, lower=0.0, upper=0.18),
    )
    model.articulation(
        "inner_to_nose",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=nose,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.12, lower=0.0, upper=0.08),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_body")
    middle = object_model.get_part("middle_carriage")
    inner = object_model.get_part("inner_carriage")
    nose = object_model.get_part("nose_plate")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")
    inner_to_nose = object_model.get_articulation("inner_to_nose")

    ctx.check(
        "serial chain has three slide joints",
        len(object_model.articulations) == 3
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in object_model.articulations),
        details=f"articulations={[j.name for j in object_model.articulations]}",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0, inner_to_nose: 0.0}):
        ctx.expect_gap(
            middle,
            outer,
            axis="z",
            positive_elem="middle_beam",
            negative_elem="outer_bed",
            max_gap=0.0005,
            max_penetration=0.000001,
            name="middle beam rides on outer bed",
        )
        ctx.expect_gap(
            inner,
            middle,
            axis="z",
            positive_elem="inner_beam",
            negative_elem="middle_beam",
            max_gap=0.0005,
            max_penetration=0.000001,
            name="inner beam rides on middle beam",
        )
        ctx.expect_gap(
            nose,
            inner,
            axis="z",
            positive_elem="nose_tongue",
            negative_elem="inner_beam",
            max_gap=0.0005,
            max_penetration=0.000001,
            name="nose tongue rides on inner beam",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_beam",
            elem_b="outer_bed",
            min_overlap=0.55,
            name="collapsed middle has deep outer engagement",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_beam",
            elem_b="middle_beam",
            min_overlap=0.34,
            name="collapsed inner has deep middle engagement",
        )
        ctx.expect_overlap(
            nose,
            inner,
            axes="x",
            elem_a="nose_tongue",
            elem_b="inner_beam",
            min_overlap=0.16,
            name="collapsed nose tongue remains captured",
        )
        ctx.expect_gap(
            nose,
            inner,
            axis="x",
            positive_elem="front_plate",
            negative_elem="inner_beam",
            min_gap=0.015,
            name="nose plate clears inner end at rest",
        )

    rest_middle = ctx.part_world_position(middle)
    rest_inner = ctx.part_world_position(inner)
    rest_nose = ctx.part_world_position(nose)

    with ctx.pose({outer_to_middle: 0.28, middle_to_inner: 0.18, inner_to_nose: 0.08}):
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="middle_beam",
            elem_b="outer_bed",
            min_overlap=0.44,
            name="extended middle retains outer engagement",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="inner_beam",
            elem_b="middle_beam",
            min_overlap=0.17,
            name="extended inner retains middle engagement",
        )
        ctx.expect_overlap(
            nose,
            inner,
            axes="x",
            elem_a="nose_tongue",
            elem_b="inner_beam",
            min_overlap=0.08,
            name="extended nose tongue remains captured",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="y",
            inner_elem="middle_beam",
            outer_elem="outer_bed",
            margin=0.0,
            name="middle stays between outer side rails",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="y",
            inner_elem="inner_beam",
            outer_elem="middle_beam",
            margin=0.0,
            name="inner stays centered on middle guide",
        )
        ctx.expect_within(
            nose,
            inner,
            axes="y",
            inner_elem="nose_tongue",
            outer_elem="inner_beam",
            margin=0.0,
            name="nose tongue stays centered in inner guide",
        )
        ctx.expect_gap(
            outer,
            middle,
            axis="y",
            positive_elem="outer_wear_strip_1",
            negative_elem="middle_beam",
            min_gap=0.001,
            name="middle clears positive outer wear strip",
        )
        ctx.expect_gap(
            middle,
            outer,
            axis="y",
            positive_elem="middle_beam",
            negative_elem="outer_wear_strip_0",
            min_gap=0.001,
            name="middle clears negative outer wear strip",
        )
        ctx.expect_gap(
            middle,
            inner,
            axis="y",
            positive_elem="middle_guide_strip_1",
            negative_elem="inner_beam",
            min_gap=0.005,
            name="inner clears positive middle guide",
        )
        ctx.expect_gap(
            inner,
            middle,
            axis="y",
            positive_elem="inner_beam",
            negative_elem="middle_guide_strip_0",
            min_gap=0.005,
            name="inner clears negative middle guide",
        )
        ctx.expect_gap(
            inner,
            nose,
            axis="y",
            positive_elem="inner_guide_strip_1",
            negative_elem="nose_tongue",
            min_gap=0.0005,
            name="nose clears positive inner guide",
        )
        ctx.expect_gap(
            nose,
            inner,
            axis="y",
            positive_elem="nose_tongue",
            negative_elem="inner_guide_strip_0",
            min_gap=0.0005,
            name="nose clears negative inner guide",
        )
        ctx.expect_gap(
            outer,
            middle,
            axis="z",
            positive_elem="outer_keeper_0",
            negative_elem="middle_screw_0_1",
            min_gap=0.010,
            name="middle screw heads clear outer keeper",
        )
        ctx.expect_gap(
            nose,
            inner,
            axis="x",
            positive_elem="front_plate",
            negative_elem="inner_beam",
            min_gap=0.09,
            name="extended nose plate clears inner end",
        )
        ext_middle = ctx.part_world_position(middle)
        ext_inner = ctx.part_world_position(inner)
        ext_nose = ctx.part_world_position(nose)

    ctx.check(
        "all stages extend along positive x",
        rest_middle is not None
        and rest_inner is not None
        and rest_nose is not None
        and ext_middle is not None
        and ext_inner is not None
        and ext_nose is not None
        and ext_middle[0] > rest_middle[0] + 0.27
        and ext_inner[0] > rest_inner[0] + 0.45
        and ext_nose[0] > rest_nose[0] + 0.53,
        details=f"rest={(rest_middle, rest_inner, rest_nose)}, extended={(ext_middle, ext_inner, ext_nose)}",
    )

    return ctx.report()


object_model = build_object_model()
