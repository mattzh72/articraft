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


HINGE_RPY = (-1.57079632679, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_service_arm")

    dark_steel = Material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
    painted_blue = Material("painted_blue", color=(0.05, 0.23, 0.55, 1.0))
    safety_orange = Material("safety_orange", color=(0.95, 0.42, 0.08, 1.0))
    black_rubber = Material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.30, 0.34, 0.040)),
        origin=Origin(xyz=(0.000, 0.000, 0.020)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.035, 0.250, 0.185)),
        origin=Origin(xyz=(-0.075, 0.000, 0.130)),
        material=dark_steel,
        name="back_web",
    )
    base.visual(
        Box((0.130, 0.035, 0.265)),
        origin=Origin(xyz=(0.000, -0.105, 0.170)),
        material=dark_steel,
        name="shoulder_cheek_0",
    )
    base.visual(
        Box((0.130, 0.035, 0.265)),
        origin=Origin(xyz=(0.000, 0.105, 0.170)),
        material=dark_steel,
        name="shoulder_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.000, -0.124, 0.240), rpy=HINGE_RPY),
        material=dark_steel,
        name="pin_cap_0",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.000, 0.124, 0.240), rpy=HINGE_RPY),
        material=dark_steel,
        name="pin_cap_1",
    )

    first = model.part("first_link")
    first.visual(
        Cylinder(radius=0.055, length=0.175),
        origin=Origin(rpy=HINGE_RPY),
        material=painted_blue,
        name="shoulder_boss",
    )
    first.visual(
        Box((0.620, 0.055, 0.050)),
        origin=Origin(xyz=(0.340, 0.000, 0.000)),
        material=painted_blue,
        name="long_bar",
    )
    first.visual(
        Box((0.085, 0.155, 0.052)),
        origin=Origin(xyz=(0.660, 0.000, 0.000)),
        material=painted_blue,
        name="elbow_bridge",
    )
    for index, y in enumerate((-0.067, 0.067)):
        first.visual(
            Box((0.145, 0.026, 0.052)),
            origin=Origin(xyz=(0.730, y, 0.000)),
            material=painted_blue,
            name=f"elbow_fork_{index}",
        )
        first.visual(
            Cylinder(radius=0.045, length=0.028),
            origin=Origin(xyz=(0.750, y, 0.000), rpy=HINGE_RPY),
            material=painted_blue,
            name=f"elbow_knuckle_{index}",
        )

    second = model.part("second_link")
    second.visual(
        Cylinder(radius=0.043, length=0.108),
        origin=Origin(rpy=HINGE_RPY),
        material=safety_orange,
        name="elbow_boss",
    )
    second.visual(
        Box((0.360, 0.050, 0.042)),
        origin=Origin(xyz=(0.205, 0.000, 0.000)),
        material=safety_orange,
        name="short_bar",
    )
    second.visual(
        Box((0.080, 0.058, 0.050)),
        origin=Origin(xyz=(0.405, 0.000, 0.000)),
        material=safety_orange,
        name="pad_mount",
    )

    pad = model.part("end_pad")
    pad.visual(
        Box((0.090, 0.040, 0.034)),
        origin=Origin(xyz=(0.045, 0.000, 0.000)),
        material=dark_steel,
        name="pad_neck",
    )
    pad.visual(
        Box((0.026, 0.026, 0.058)),
        origin=Origin(xyz=(0.075, 0.000, -0.028)),
        material=dark_steel,
        name="pad_stem",
    )
    pad.visual(
        Cylinder(radius=0.060, length=0.018),
        origin=Origin(xyz=(0.075, 0.000, -0.066)),
        material=black_rubber,
        name="rubber_disc",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=first,
        origin=Origin(xyz=(0.000, 0.000, 0.240)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.4, lower=-1.0, upper=1.2),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first,
        child=second,
        origin=Origin(xyz=(0.750, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=0.0, upper=2.45),
    )
    model.articulation(
        "pad_mount",
        ArticulationType.FIXED,
        parent=second,
        child=pad,
        origin=Origin(xyz=(0.445, 0.000, 0.000)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_bracket")
    first = object_model.get_part("first_link")
    second = object_model.get_part("second_link")
    pad = object_model.get_part("end_pad")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check(
        "hinge axes are parallel",
        tuple(shoulder.axis) == (0.0, -1.0, 0.0) and tuple(elbow.axis) == (0.0, -1.0, 0.0),
        details=f"shoulder_axis={shoulder.axis}, elbow_axis={elbow.axis}",
    )
    ctx.expect_origin_gap(
        second,
        first,
        axis="x",
        min_gap=0.72,
        max_gap=0.78,
        name="elbow sits at end of long first link",
    )
    ctx.expect_origin_gap(
        pad,
        second,
        axis="x",
        min_gap=0.42,
        max_gap=0.47,
        name="end pad sits at end of shorter second link",
    )
    ctx.expect_overlap(
        first,
        base,
        axes="xz",
        elem_a="shoulder_boss",
        elem_b="shoulder_cheek_0",
        min_overlap=0.050,
        name="shoulder boss captured by base clevis",
    )
    ctx.expect_within(
        second,
        first,
        axes="y",
        inner_elem="elbow_boss",
        outer_elem="elbow_bridge",
        margin=0.0,
        name="elbow boss sits between first-link fork cheeks",
    )

    rest_pad = ctx.part_world_position(pad)
    with ctx.pose({elbow: 1.65}):
        folded_pad = ctx.part_world_position(pad)
    ctx.check(
        "elbow folds end pad back and upward in the arm plane",
        rest_pad is not None
        and folded_pad is not None
        and folded_pad[0] < rest_pad[0] - 0.35
        and folded_pad[2] > rest_pad[2] + 0.35
        and abs(folded_pad[1] - rest_pad[1]) < 0.001,
        details=f"rest={rest_pad}, folded={folded_pad}",
    )

    return ctx.report()


object_model = build_object_model()
