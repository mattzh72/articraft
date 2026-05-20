from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.48, 0.53, 0.55, 1.0))
    dark_steel = model.material("dark_powder_coated_steel", rgba=(0.05, 0.07, 0.08, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    concrete = model.material("poured_concrete", rgba=(0.55, 0.53, 0.49, 1.0))
    warning_yellow = model.material("travel_mark_yellow", rgba=(0.95, 0.72, 0.08, 1.0))

    support = model.part("support_frame")
    support.visual(
        Box((4.50, 0.36, 0.10)),
        origin=Origin(xyz=(0.55, 0.0, 0.05)),
        material=concrete,
        name="concrete_sill",
    )
    support.visual(
        Box((0.13, 0.13, 1.84)),
        origin=Origin(xyz=(-1.35, 0.0, 0.97)),
        material=galvanized,
        name="closed_post",
    )
    support.visual(
        Box((0.13, 0.13, 1.84)),
        origin=Origin(xyz=(2.65, 0.0, 0.97)),
        material=galvanized,
        name="track_post",
    )
    support.visual(
        Box((4.35, 0.22, 0.045)),
        origin=Origin(xyz=(0.55, 0.0, 1.79)),
        material=galvanized,
        name="top_cap",
    )
    support.visual(
        Box((4.35, 0.035, 0.18)),
        origin=Origin(xyz=(0.55, 0.085, 1.6775)),
        material=galvanized,
        name="top_lip_y_positive",
    )
    support.visual(
        Box((4.35, 0.035, 0.18)),
        origin=Origin(xyz=(0.55, -0.085, 1.6775)),
        material=galvanized,
        name="top_lip_y_negative",
    )
    support.visual(
        Box((4.20, 0.15, 0.03)),
        origin=Origin(xyz=(0.55, 0.0, 0.125)),
        material=galvanized,
        name="bottom_floor",
    )
    support.visual(
        Box((4.20, 0.035, 0.16)),
        origin=Origin(xyz=(0.55, 0.065, 0.18)),
        material=galvanized,
        name="bottom_lip_y_positive",
    )
    support.visual(
        Box((4.20, 0.035, 0.16)),
        origin=Origin(xyz=(0.55, -0.065, 0.18)),
        material=galvanized,
        name="bottom_lip_y_negative",
    )
    support.visual(
        Box((0.10, 0.14, 0.22)),
        origin=Origin(xyz=(-1.235, 0.0, 0.95)),
        material=galvanized,
        name="latch_receiver",
    )
    support.visual(
        Box((0.035, 0.30, 0.16)),
        origin=Origin(xyz=(-1.18, 0.0, 0.205)),
        material=rubber,
        name="closed_stop",
    )
    support.visual(
        Box((0.035, 0.30, 0.16)),
        origin=Origin(xyz=(2.26, 0.0, 0.205)),
        material=rubber,
        name="travel_stop",
    )
    support.visual(
        Box((4.05, 0.012, 0.012)),
        origin=Origin(xyz=(0.55, -0.135, 0.245)),
        material=warning_yellow,
        name="exposed_travel_line",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((2.10, 0.065, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_steel,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((2.10, 0.065, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        material=dark_steel,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.08, 0.065, 1.24)),
        origin=Origin(xyz=(-1.05, 0.0, 0.715)),
        material=dark_steel,
        name="end_stile_0",
    )
    gate_leaf.visual(
        Box((0.08, 0.065, 1.24)),
        origin=Origin(xyz=(1.05, 0.0, 0.715)),
        material=dark_steel,
        name="end_stile_1",
    )

    for index, x in enumerate((-0.78, -0.52, -0.26, 0.0, 0.26, 0.52, 0.78)):
        gate_leaf.visual(
            Box((0.035, 0.045, 1.13)),
            origin=Origin(xyz=(x, 0.0, 0.715)),
            material=dark_steel,
            name=f"security_bar_{index}",
        )

    brace_angle = math.atan2(1.00, 1.90)
    gate_leaf.visual(
        Box((2.15, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.715), rpy=(0.0, -brace_angle, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )
    gate_leaf.visual(
        Box((0.18, 0.075, 0.26)),
        origin=Origin(xyz=(-0.82, -0.045, 0.78)),
        material=dark_steel,
        name="lock_box",
    )
    gate_leaf.visual(
        Box((1.90, 0.050, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="bottom_shoe",
    )
    gate_leaf.visual(
        Box((0.055, 0.040, 0.20)),
        origin=Origin(xyz=(-0.68, 0.0, 0.075)),
        material=dark_steel,
        name="shoe_bracket_0",
    )
    gate_leaf.visual(
        Box((0.055, 0.040, 0.20)),
        origin=Origin(xyz=(0.68, 0.0, 0.075)),
        material=dark_steel,
        name="shoe_bracket_1",
    )
    gate_leaf.visual(
        Box((0.065, 0.030, 0.18)),
        origin=Origin(xyz=(-0.62, 0.0, 1.395)),
        material=dark_steel,
        name="roller_hanger_0",
    )
    gate_leaf.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(xyz=(-0.62, 0.0, 1.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_0",
    )
    gate_leaf.visual(
        Box((0.065, 0.030, 0.18)),
        origin=Origin(xyz=(0.62, 0.0, 1.395)),
        material=dark_steel,
        name="roller_hanger_1",
    )
    gate_leaf.visual(
        Cylinder(radius=0.045, length=0.070),
        origin=Origin(xyz=(0.62, 0.0, 1.46), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="top_roller_1",
    )

    model.articulation(
        "leaf_slide",
        ArticulationType.PRISMATIC,
        parent=support,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.1675)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("leaf_slide")

    ctx.expect_gap(
        support,
        gate_leaf,
        axis="y",
        positive_elem="top_lip_y_positive",
        negative_elem="top_roller_0",
        min_gap=0.020,
        max_gap=0.045,
        name="top roller has visible clearance to positive channel lip",
    )
    ctx.expect_gap(
        gate_leaf,
        support,
        axis="y",
        positive_elem="top_roller_0",
        negative_elem="top_lip_y_negative",
        min_gap=0.020,
        max_gap=0.045,
        name="top roller has visible clearance to negative channel lip",
    )
    ctx.expect_gap(
        gate_leaf,
        support,
        axis="z",
        positive_elem="bottom_shoe",
        negative_elem="bottom_floor",
        max_gap=0.001,
        max_penetration=0.0,
        name="bottom shoe rides on the guide floor",
    )
    ctx.expect_overlap(
        gate_leaf,
        support,
        axes="x",
        elem_a="bottom_shoe",
        elem_b="bottom_floor",
        min_overlap=1.5,
        name="closed leaf is captured along the exposed bottom guide",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 1.10}):
        ctx.expect_overlap(
            gate_leaf,
            support,
            axes="x",
            elem_a="bottom_shoe",
            elem_b="bottom_floor",
            min_overlap=1.5,
            name="open leaf remains captured along the exposed bottom guide",
        )
        ctx.expect_gap(
            support,
            gate_leaf,
            axis="y",
            positive_elem="top_lip_y_positive",
            negative_elem="top_roller_1",
            min_gap=0.020,
            max_gap=0.045,
            name="open top roller stays in the upper channel",
        )
        extended_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "leaf translates along the track axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 1.0,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
