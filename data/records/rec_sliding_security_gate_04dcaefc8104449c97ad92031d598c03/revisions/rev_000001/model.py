from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized_steel", color=(0.62, 0.66, 0.67, 1.0))
    dark_steel = model.material("dark_powder_coat", color=(0.05, 0.06, 0.055, 1.0))
    guide_yellow = model.material("safety_yellow", color=(0.95, 0.72, 0.10, 1.0))
    concrete = model.material("concrete", color=(0.48, 0.47, 0.43, 1.0))

    fixed = model.part("fixed_frame")

    fixed.visual(
        Box((4.90, 0.36, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=concrete,
        name="concrete_sill",
    )
    fixed.visual(
        Box((4.70, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0675)),
        material=galvanized,
        name="bottom_pan",
    )
    for name, y in (("front_bottom_lip", 0.11), ("rear_bottom_lip", -0.11)):
        fixed.visual(
            Box((4.70, 0.03, 0.08)),
            origin=Origin(xyz=(0.0, y, 0.125)),
            material=galvanized,
            name=name,
        )

    for name, x in (("receiver_post", -2.25), ("support_post", 2.25)):
        fixed.visual(
            Box((0.18, 0.18, 1.95)),
            origin=Origin(xyz=(x, 0.0, 0.975)),
            material=dark_steel,
            name=name,
        )

    fixed.visual(
        Box((4.70, 0.26, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 1.94)),
        material=galvanized,
        name="top_track_roof",
    )
    for name, y in (("front_track_lip", 0.1125), ("rear_track_lip", -0.1125)):
        fixed.visual(
            Box((4.70, 0.035, 0.18)),
            origin=Origin(xyz=(0.0, y, 1.83)),
            material=galvanized,
            name=name,
        )

    fixed.visual(
        Box((0.10, 0.05, 0.55)),
        origin=Origin(xyz=(-2.12, -0.115, 0.96)),
        material=guide_yellow,
        name="receiver_catch",
    )

    gate = model.part("gate_leaf")
    leaf_len = 2.70
    gate.visual(
        Box((leaf_len, 0.06, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=dark_steel,
        name="bottom_rail",
    )
    gate.visual(
        Box((leaf_len, 0.06, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
        material=dark_steel,
        name="top_rail",
    )
    for name, x in (("end_stile_0", -1.35), ("end_stile_1", 1.35)):
        gate.visual(
            Box((0.08, 0.06, 1.31)),
            origin=Origin(xyz=(x, 0.0, 0.915)),
            material=dark_steel,
            name=name,
        )

    gate.visual(
        Box((2.48, 0.05, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=dark_steel,
        name="middle_rail",
    )
    for index, x in enumerate((-0.96, -0.64, -0.32, 0.0, 0.32, 0.64, 0.96)):
        gate.visual(
            Box((0.035, 0.045, 1.20)),
            origin=Origin(xyz=(x, 0.0, 0.915)),
            material=dark_steel,
            name=f"bar_{index}",
        )

    diag_dx = 2.42
    diag_dz = 1.17
    diag_len = math.sqrt(diag_dx * diag_dx + diag_dz * diag_dz)
    diag_angle = math.atan2(diag_dz, diag_dx)
    gate.visual(
        Box((diag_len, 0.045, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.915), rpy=(0.0, -diag_angle, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )

    gate.visual(
        Box((2.60, 0.15, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.82)),
        material=galvanized,
        name="top_shoe",
    )
    for index, x in enumerate((-0.90, 0.90)):
        gate.visual(
            Box((0.07, 0.045, 0.25)),
            origin=Origin(xyz=(x, 0.0, 1.685)),
            material=galvanized,
            name=f"top_hanger_{index}",
        )

    gate.visual(
        Box((2.60, 0.15, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=galvanized,
        name="bottom_shoe",
    )
    for index, x in enumerate((-0.90, 0.90)):
        gate.visual(
            Box((0.08, 0.05, 0.18)),
            origin=Origin(xyz=(x, 0.0, 0.20)),
            material=galvanized,
            name=f"bottom_stem_{index}",
        )

    gate.visual(
        Box((0.16, 0.05, 0.08)),
        origin=Origin(xyz=(-1.43, -0.05, 1.02)),
        material=guide_yellow,
        name="latch_tongue",
    )

    model.articulation(
        "frame_to_leaf",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=gate,
        origin=Origin(xyz=(-0.55, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=250.0, velocity=0.45, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_leaf")

    def check_nested_guides(prefix: str) -> None:
        ctx.expect_within(
            gate,
            fixed,
            axes="x",
            inner_elem="top_shoe",
            outer_elem="top_track_roof",
            margin=0.0,
            name=f"{prefix} top shoe remains under full track length",
        )
        ctx.expect_overlap(
            gate,
            fixed,
            axes="x",
            elem_a="top_shoe",
            elem_b="top_track_roof",
            min_overlap=2.40,
            name=f"{prefix} top shoe keeps retained insertion",
        )
        ctx.expect_gap(
            fixed,
            gate,
            axis="y",
            positive_elem="front_track_lip",
            negative_elem="top_shoe",
            min_gap=0.012,
            max_gap=0.035,
            name=f"{prefix} front top guide clearance is tight",
        )
        ctx.expect_gap(
            gate,
            fixed,
            axis="y",
            positive_elem="top_shoe",
            negative_elem="rear_track_lip",
            min_gap=0.012,
            max_gap=0.035,
            name=f"{prefix} rear top guide clearance is tight",
        )
        ctx.expect_gap(
            fixed,
            gate,
            axis="z",
            positive_elem="top_track_roof",
            negative_elem="top_shoe",
            min_gap=0.045,
            max_gap=0.075,
            name=f"{prefix} top shoe clears roof inside channel",
        )
        ctx.expect_gap(
            fixed,
            gate,
            axis="y",
            positive_elem="front_bottom_lip",
            negative_elem="bottom_shoe",
            min_gap=0.012,
            max_gap=0.035,
            name=f"{prefix} front bottom guide clearance is tight",
        )
        ctx.expect_gap(
            gate,
            fixed,
            axis="y",
            positive_elem="bottom_shoe",
            negative_elem="rear_bottom_lip",
            min_gap=0.012,
            max_gap=0.035,
            name=f"{prefix} rear bottom guide clearance is tight",
        )
        ctx.expect_gap(
            gate,
            fixed,
            axis="z",
            positive_elem="bottom_shoe",
            negative_elem="bottom_pan",
            min_gap=0.0,
            max_gap=0.002,
            name=f"{prefix} bottom shoe bears on pan",
        )

    check_nested_guides("closed")
    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: 1.30}):
        check_nested_guides("open")
        extended_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate leaf translates along track",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 1.20
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
