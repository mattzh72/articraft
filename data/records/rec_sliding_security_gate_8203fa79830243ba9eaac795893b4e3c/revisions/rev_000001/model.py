from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    frame_steel = model.material("frame_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    gate_steel = model.material("gate_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.11, 0.12, 1.0))

    fixed_frame = model.part("fixed_frame")
    fixed_frame.visual(
        Box((0.10, 0.10, 2.25)),
        origin=Origin(xyz=(-1.25, 0.0, 1.125)),
        material=frame_steel,
        name="left_post",
    )
    fixed_frame.visual(
        Box((0.10, 0.10, 2.25)),
        origin=Origin(xyz=(1.25, 0.0, 1.125)),
        material=frame_steel,
        name="right_post",
    )
    fixed_frame.visual(
        Box((2.60, 0.08, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 2.19)),
        material=frame_steel,
        name="header_beam",
    )
    fixed_frame.visual(
        Box((2.70, 0.07, 0.012)),
        origin=Origin(xyz=(0.0, 0.11, 2.171)),
        material=frame_steel,
        name="track_top",
    )
    fixed_frame.visual(
        Box((2.70, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.079, 2.135)),
        material=frame_steel,
        name="track_back_wall",
    )
    fixed_frame.visual(
        Box((2.70, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.141, 2.135)),
        material=frame_steel,
        name="track_front_wall",
    )
    for index, x in enumerate((-1.00, -0.10, 0.80)):
        fixed_frame.visual(
            Box((0.08, 0.035, 0.08)),
            origin=Origin(xyz=(x, 0.0575, 2.145)),
            material=frame_steel,
            name=f"track_bracket_{index}",
        )
    fixed_frame.inertial = Inertial.from_geometry(
        Box((2.80, 0.16, 2.30)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.04, 1.15)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((1.30, 0.034, 0.024)),
        material=gate_steel,
        name="carriage_beam",
    )
    for index, x in enumerate((-0.40, 0.40)):
        gate_leaf.visual(
            Box((0.038, 0.014, 0.018)),
            origin=Origin(xyz=(x, 0.0, 0.003)),
            material=gate_steel,
            name=f"roller_mount_{index}",
        )
        gate_leaf.visual(
            Box((0.020, 0.010, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.014)),
            material=gate_steel,
            name=f"roller_yoke_{index}",
        )
        gate_leaf.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(x, 0.0, 0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=latch_black,
            name=f"wheel_{index}",
        )
    for index, x in enumerate((-0.36, 0.36)):
        gate_leaf.visual(
            Box((0.06, 0.014, 0.106)),
            origin=Origin(xyz=(x, 0.0, -0.065)),
            material=gate_steel,
            name=f"hanger_{index}",
        )
    gate_leaf.visual(
        Box((1.20, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.143)),
        material=gate_steel,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.20, 0.04, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -2.030)),
        material=gate_steel,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.05, 0.04, 1.88)),
        origin=Origin(xyz=(-0.575, 0.0, -1.093)),
        material=gate_steel,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.05, 0.04, 1.88)),
        origin=Origin(xyz=(0.575, 0.0, -1.093)),
        material=gate_steel,
        name="right_stile",
    )
    for index, x in enumerate((-0.40, -0.24, -0.08, 0.08, 0.24, 0.40)):
        gate_leaf.visual(
            Box((0.02, 0.02, 1.86)),
            origin=Origin(xyz=(x, 0.0, -1.093)),
            material=gate_steel,
            name=f"bar_{index}",
        )
    gate_leaf.visual(
        Box((0.10, 0.045, 0.18)),
        origin=Origin(xyz=(0.515, 0.0, -1.093)),
        material=latch_black,
        name="lock_case",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.35, 0.05, 2.10)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, -1.03)),
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=fixed_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.55, 0.11, 2.135)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.35,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_frame = object_model.get_part("fixed_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")
    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else 1.10

    ctx.expect_contact(
        gate_leaf,
        fixed_frame,
        elem_a="wheel_0",
        elem_b="track_top",
        name="left trolley wheel bears on the top track at rest",
    )
    ctx.expect_contact(
        gate_leaf,
        fixed_frame,
        elem_a="wheel_1",
        elem_b="track_top",
        name="right trolley wheel bears on the top track at rest",
    )

    ctx.expect_gap(
        gate_leaf,
        fixed_frame,
        axis="y",
        positive_elem="carriage_beam",
        negative_elem="track_back_wall",
        min_gap=0.009,
        max_gap=0.012,
        name="carriage clears back track wall at rest",
    )
    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="y",
        positive_elem="track_front_wall",
        negative_elem="carriage_beam",
        min_gap=0.009,
        max_gap=0.012,
        name="carriage clears front track wall at rest",
    )
    ctx.expect_gap(
        fixed_frame,
        gate_leaf,
        axis="z",
        positive_elem="track_top",
        negative_elem="carriage_beam",
        min_gap=0.017,
        max_gap=0.021,
        name="carriage hangs below top track plate",
    )
    ctx.expect_overlap(
        gate_leaf,
        fixed_frame,
        axes="x",
        elem_a="carriage_beam",
        elem_b="track_top",
        min_overlap=1.28,
        name="closed gate keeps a long carriage overlap in the track",
    )

    closed_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: slide_upper}):
        ctx.expect_contact(
            gate_leaf,
            fixed_frame,
            elem_a="wheel_0",
            elem_b="track_top",
            name="left trolley wheel stays supported at full open",
        )
        ctx.expect_contact(
            gate_leaf,
            fixed_frame,
            elem_a="wheel_1",
            elem_b="track_top",
            name="right trolley wheel stays supported at full open",
        )
        ctx.expect_gap(
            gate_leaf,
            fixed_frame,
            axis="y",
            positive_elem="carriage_beam",
            negative_elem="track_back_wall",
            min_gap=0.009,
            max_gap=0.012,
            name="carriage clears back track wall fully open",
        )
        ctx.expect_gap(
            fixed_frame,
            gate_leaf,
            axis="y",
            positive_elem="track_front_wall",
            negative_elem="carriage_beam",
            min_gap=0.009,
            max_gap=0.012,
            name="carriage clears front track wall fully open",
        )
        ctx.expect_overlap(
            gate_leaf,
            fixed_frame,
            axes="x",
            elem_a="carriage_beam",
            elem_b="track_top",
            min_overlap=1.28,
            name="open gate still keeps generous track engagement",
        )
        open_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf slides left to open",
        closed_pos is not None and open_pos is not None and open_pos[0] < closed_pos[0] - 1.05,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
