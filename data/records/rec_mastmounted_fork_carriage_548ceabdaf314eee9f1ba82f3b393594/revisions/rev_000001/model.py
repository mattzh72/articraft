from __future__ import annotations

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
import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_forklift_mast")

    dark_steel = Material("dark_steel", rgba=(0.08, 0.09, 0.09, 1.0))
    worn_steel = Material("worn_steel", rgba=(0.55, 0.56, 0.54, 1.0))
    carriage_orange = Material("carriage_orange", rgba=(0.95, 0.36, 0.04, 1.0))
    fork_steel = Material("fork_steel", rgba=(0.12, 0.12, 0.11, 1.0))
    rubber = Material("rubber", rgba=(0.015, 0.015, 0.013, 1.0))

    mast = model.part("mast")

    # Two compact C-channel uprights.  The webs sit outboard, with front and rear
    # flanges wrapping inward so the pair reads as forklift mast rails.
    mast.visual(
        Box((0.035, 0.18, 1.55)),
        origin=Origin(xyz=(-0.38, 0.0, 0.775)),
        material=dark_steel,
        name="left_web",
    )
    mast.visual(
        Box((0.11, 0.035, 1.55)),
        origin=Origin(xyz=(-0.325, -0.075, 0.775)),
        material=dark_steel,
        name="left_front_flange",
    )
    mast.visual(
        Box((0.11, 0.035, 1.55)),
        origin=Origin(xyz=(-0.325, 0.075, 0.775)),
        material=dark_steel,
        name="left_rear_flange",
    )
    mast.visual(
        Box((0.028, 0.012, 1.38)),
        origin=Origin(xyz=(-0.265, -0.096, 0.79)),
        material=worn_steel,
        name="left_wear_strip",
    )
    mast.visual(
        Box((0.035, 0.18, 1.55)),
        origin=Origin(xyz=(0.38, 0.0, 0.775)),
        material=dark_steel,
        name="right_web",
    )
    mast.visual(
        Box((0.11, 0.035, 1.55)),
        origin=Origin(xyz=(0.325, -0.075, 0.775)),
        material=dark_steel,
        name="right_front_flange",
    )
    mast.visual(
        Box((0.11, 0.035, 1.55)),
        origin=Origin(xyz=(0.325, 0.075, 0.775)),
        material=dark_steel,
        name="right_rear_flange",
    )
    mast.visual(
        Box((0.028, 0.012, 1.38)),
        origin=Origin(xyz=(0.265, -0.096, 0.79)),
        material=worn_steel,
        name="right_wear_strip",
    )

    mast.visual(
        Box((0.84, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
        material=dark_steel,
        name="top_crosshead",
    )
    mast.visual(
        Box((0.78, 0.15, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_steel,
        name="bottom_tie",
    )
    mast.visual(
        Box((0.64, 0.035, 0.07)),
        origin=Origin(xyz=(0.0, 0.085, 0.78)),
        material=dark_steel,
        name="rear_tie_bar",
    )
    for sx in (-0.12, 0.12):
        mast.visual(
            Cylinder(radius=0.055, length=0.035),
            origin=Origin(xyz=(sx, -0.115, 1.55), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name=f"top_sheave_{0 if sx < 0 else 1}",
        )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.62, 0.060, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=carriage_orange,
        name="lower_bar",
    )
    carriage.visual(
        Box((0.62, 0.060, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.32)),
        material=carriage_orange,
        name="upper_bar",
    )
    for sx in (-0.29, 0.29):
        carriage.visual(
            Box((0.050, 0.070, 0.42)),
            origin=Origin(xyz=(sx, 0.0, 0.16)),
            material=carriage_orange,
            name=f"side_plate_{0 if sx < 0 else 1}",
        )
    carriage.visual(
        Box((0.045, 0.025, 0.18)),
        origin=Origin(xyz=(-0.265, 0.0305, 0.16)),
        material=worn_steel,
        name="slide_pad_0",
    )
    carriage.visual(
        Box((0.045, 0.025, 0.18)),
        origin=Origin(xyz=(0.265, 0.0305, 0.16)),
        material=worn_steel,
        name="slide_pad_1",
    )

    # Load backrest: a connected guard frame rising from the carriage.
    for i, sx in enumerate((-0.22, 0.0, 0.22)):
        carriage.visual(
            Box((0.035, 0.045, 0.50)),
            origin=Origin(xyz=(sx, -0.015, 0.61)),
            material=carriage_orange,
            name=f"backrest_post_{i}",
        )
    carriage.visual(
        Box((0.56, 0.045, 0.060)),
        origin=Origin(xyz=(0.0, -0.015, 0.86)),
        material=carriage_orange,
        name="backrest_top",
    )
    carriage.visual(
        Box((0.50, 0.035, 0.050)),
        origin=Origin(xyz=(0.0, -0.020, 0.51)),
        material=carriage_orange,
        name="backrest_mid",
    )

    # Paired forks are authored as rigid parts of the carriage, not separate
    # joints.  Each shank overlaps the lower bar and the horizontal tine so the
    # L-shaped fork reads as a mounted steel weldment.
    for i, sx in enumerate((-0.18, 0.18)):
        carriage.visual(
            Box((0.085, 0.10, 0.34)),
            origin=Origin(xyz=(sx, -0.080, -0.14)),
            material=fork_steel,
            name=f"fork_{i}_shank",
        )
        carriage.visual(
            Box((0.085, 0.68, 0.065)),
            origin=Origin(xyz=(sx, -0.445, -0.295)),
            material=fork_steel,
            name=f"fork_{i}_tine",
        )
        carriage.visual(
            Box((0.085, 0.18, 0.038)),
            origin=Origin(xyz=(sx, -0.875, -0.282)),
            material=fork_steel,
            name=f"fork_{i}_toe",
        )
        carriage.visual(
            Cylinder(radius=0.040, length=0.030),
            origin=Origin(xyz=(sx, 0.025, 0.27), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"guide_roller_{i}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, -0.145, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6000.0, velocity=0.45, lower=0.0, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    ctx.check(
        "single vertical prismatic lift",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper == 0.65,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={lift.motion_limits}",
    )
    ctx.check(
        "forks rigidly authored on carriage",
        all(
            carriage.get_visual(name) is not None
            for name in ("fork_0_tine", "fork_1_tine", "fork_0_shank", "fork_1_shank")
        ),
        details="Both fork tines and shanks should be visual elements of the carriage part.",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            min_gap=0.010,
            max_gap=0.070,
            positive_elem="left_front_flange",
            negative_elem="upper_bar",
            name="carriage face clears mast rails",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a="slide_pad_0",
            elem_b="left_wear_strip",
            name="carriage slide pad bears on rail",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="x",
            min_overlap=0.50,
            elem_a="upper_bar",
            elem_b="top_crosshead",
            name="carriage spans between mast uprights",
        )
        rest_pos = ctx.part_world_position(carriage)

    with ctx.pose({lift: 0.65}):
        ctx.expect_gap(
            carriage,
            mast,
            axis="z",
            min_gap=0.020,
            positive_elem="backrest_top",
            negative_elem="bottom_tie",
            name="raised carriage is above base tie",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage rises along mast",
        rest_pos is not None
        and raised_pos is not None
        and abs((raised_pos[2] - rest_pos[2]) - 0.65) < 1e-6,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
