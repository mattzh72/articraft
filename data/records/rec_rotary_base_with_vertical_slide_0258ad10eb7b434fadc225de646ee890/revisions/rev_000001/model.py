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
    model = ArticulatedObject(name="low_profile_rotary_lift")

    dark_metal = model.material("dark_metal", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.58, 1.0))
    satin_black = model.material("satin_black", rgba=(0.015, 0.016, 0.018, 1.0))
    warm_column = model.material("warm_column", rgba=(0.35, 0.36, 0.34, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.025, 0.025, 0.023, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.32, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_metal,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.255, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=satin_black,
        name="bearing_shadow",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.245, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=brushed_steel,
        name="table_disk",
    )
    turntable.visual(
        Cylinder(radius=0.105, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_metal,
        name="center_boss",
    )
    turntable.visual(
        Box((0.120, 0.090, 0.280)),
        origin=Origin(xyz=(0.0, -0.100, 0.180)),
        material=warm_column,
        name="tower_column",
    )
    turntable.visual(
        Box((0.088, 0.008, 0.250)),
        origin=Origin(xyz=(0.0, -0.059, 0.172)),
        material=satin_black,
        name="front_guide",
    )
    turntable.visual(
        Box((0.148, 0.120, 0.018)),
        origin=Origin(xyz=(0.0, -0.100, 0.049)),
        material=dark_metal,
        name="tower_foot",
    )

    lift_stage = model.part("lift_stage")
    lift_stage.visual(
        Box((0.056, 0.030, 0.240)),
        origin=Origin(xyz=(0.0, 0.015, 0.120)),
        material=brushed_steel,
        name="slide_rail",
    )
    lift_stage.visual(
        Box((0.078, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, 0.039, 0.154)),
        material=dark_metal,
        name="carriage_face",
    )
    lift_stage.visual(
        Box((0.160, 0.160, 0.026)),
        origin=Origin(xyz=(0.0, 0.095, 0.253)),
        material=rubber,
        name="square_pad",
    )
    lift_stage.visual(
        Box((0.070, 0.070, 0.018)),
        origin=Origin(xyz=(0.0, 0.047, 0.232)),
        material=brushed_steel,
        name="pad_neck",
    )

    model.articulation(
        "table_spin",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "stage_lift",
        ArticulationType.PRISMATIC,
        parent=turntable,
        child=lift_stage,
        origin=Origin(xyz=(0.0, -0.055, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.100,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    lift_stage = object_model.get_part("lift_stage")
    spin = object_model.get_articulation("table_spin")
    lift = object_model.get_articulation("stage_lift")

    ctx.check(
        "turntable uses revolute joint",
        spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"joint type is {spin.articulation_type}",
    )
    ctx.check(
        "lifting head uses prismatic joint",
        lift.articulation_type == ArticulationType.PRISMATIC,
        details=f"joint type is {lift.articulation_type}",
    )

    ctx.expect_gap(
        turntable,
        base,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="table_disk",
        negative_elem="base_disk",
        name="turntable disk sits on squat base",
    )
    ctx.expect_gap(
        lift_stage,
        turntable,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="slide_rail",
        negative_elem="tower_column",
        name="slide rail rides on tower face",
    )
    ctx.expect_overlap(
        lift_stage,
        turntable,
        axes="z",
        min_overlap=0.12,
        elem_a="slide_rail",
        elem_b="tower_column",
        name="slide remains engaged with tower",
    )

    rest_pos = ctx.part_world_position(lift_stage)
    with ctx.pose({lift: 0.100}):
        ctx.expect_gap(
            lift_stage,
            turntable,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="slide_rail",
            negative_elem="tower_column",
            name="raised slide still follows tower face",
        )
        ctx.expect_overlap(
            lift_stage,
            turntable,
            axes="z",
            min_overlap=0.12,
            elem_a="slide_rail",
            elem_b="tower_column",
            name="raised slide retains guided insertion",
        )
        raised_pos = ctx.part_world_position(lift_stage)

    ctx.check(
        "prismatic joint lifts the square pad",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.09,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
