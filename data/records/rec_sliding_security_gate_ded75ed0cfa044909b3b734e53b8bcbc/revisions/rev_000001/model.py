from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TRACK_LENGTH = 2.82
TRACK_CENTER_X = 0.16
TRACK_TOP_Z = 2.208
TRACK_CLEAR_Z = 2.08
POST_HEIGHT = 2.20
LEFT_POST_X = -1.18
RIGHT_POST_X = 1.50
JOINT_X_CLOSED = 1.37
TROLLEY_Z = 2.175
GATE_WIDTH = 1.35
GATE_HEIGHT = 1.80
GATE_TRAVEL = 1.23


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    galvanized = model.material("galvanized", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.30, 0.33, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.10, 0.10, POST_HEIGHT)),
        origin=Origin(xyz=(LEFT_POST_X, 0.0, POST_HEIGHT / 2.0)),
        material=dark_steel,
        name="left_post",
    )
    support_frame.visual(
        Box((0.10, 0.10, POST_HEIGHT)),
        origin=Origin(xyz=(RIGHT_POST_X, 0.0, POST_HEIGHT / 2.0)),
        material=dark_steel,
        name="right_post",
    )
    support_frame.visual(
        Box((TRACK_LENGTH, 0.14, 0.016)),
        origin=Origin(xyz=(TRACK_CENTER_X, 0.0, TRACK_TOP_Z)),
        material=dark_steel,
        name="track_top_plate",
    )
    support_frame.visual(
        Box((TRACK_LENGTH, 0.014, TRACK_TOP_Z - TRACK_CLEAR_Z)),
        origin=Origin(
            xyz=(TRACK_CENTER_X, 0.063, (TRACK_TOP_Z - 0.008 + TRACK_CLEAR_Z) / 2.0)
        ),
        material=dark_steel,
        name="track_front_lip",
    )
    support_frame.visual(
        Box((TRACK_LENGTH, 0.014, TRACK_TOP_Z - TRACK_CLEAR_Z)),
        origin=Origin(
            xyz=(TRACK_CENTER_X, -0.063, (TRACK_TOP_Z - 0.008 + TRACK_CLEAR_Z) / 2.0)
        ),
        material=dark_steel,
        name="track_back_lip",
    )
    support_frame.visual(
        Box((0.05, 0.05, 1.70)),
        origin=Origin(xyz=(1.425, 0.0, 0.95)),
        material=galvanized,
        name="strike_plate",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((2.78, 0.14, POST_HEIGHT)),
        mass=80.0,
        origin=Origin(xyz=(0.16, 0.0, POST_HEIGHT / 2.0)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.10, 0.04, 0.05)),
        material=dark_steel,
        name="right_trolley",
    )
    gate_leaf.visual(
        Box((0.10, 0.04, 0.05)),
        origin=Origin(xyz=(-1.05, 0.0, 0.0)),
        material=dark_steel,
        name="left_trolley",
    )
    gate_leaf.visual(
        Box((0.024, 0.016, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=galvanized,
        name="right_hanger",
    )
    gate_leaf.visual(
        Box((0.024, 0.016, 0.22)),
        origin=Origin(xyz=(-1.05, 0.0, -0.135)),
        material=galvanized,
        name="left_hanger",
    )
    gate_leaf.visual(
        Box((GATE_WIDTH, 0.04, 0.06)),
        origin=Origin(xyz=(-GATE_WIDTH / 2.0, 0.0, -0.27)),
        material=galvanized,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((GATE_WIDTH, 0.04, 0.06)),
        origin=Origin(xyz=(-GATE_WIDTH / 2.0, 0.0, -2.01)),
        material=galvanized,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.06, 0.04, 1.68)),
        origin=Origin(xyz=(-0.03, 0.0, -1.14)),
        material=galvanized,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((0.06, 0.04, 1.68)),
        origin=Origin(xyz=(-GATE_WIDTH + 0.03, 0.0, -1.14)),
        material=galvanized,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((GATE_WIDTH - 0.12, 0.03, 0.05)),
        origin=Origin(xyz=(-GATE_WIDTH / 2.0, 0.0, -1.14)),
        material=galvanized,
        name="mid_rail",
    )
    gate_leaf.visual(
        Box((0.05, 0.03, 1.68)),
        origin=Origin(xyz=(-0.42, 0.0, -1.14)),
        material=galvanized,
        name="inner_bar_a",
    )
    gate_leaf.visual(
        Box((0.05, 0.03, 1.68)),
        origin=Origin(xyz=(-0.68, 0.0, -1.14)),
        material=galvanized,
        name="inner_bar_b",
    )
    gate_leaf.visual(
        Box((0.05, 0.03, 1.68)),
        origin=Origin(xyz=(-0.94, 0.0, -1.14)),
        material=galvanized,
        name="inner_bar_c",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((GATE_WIDTH, 0.05, 2.05)),
        mass=34.0,
        origin=Origin(xyz=(-GATE_WIDTH / 2.0, 0.0, -1.025)),
    )

    model.articulation(
        "frame_to_gate",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=(JOINT_X_CLOSED, 0.0, TROLLEY_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.6,
            lower=0.0,
            upper=GATE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate")

    ctx.expect_contact(
        frame,
        gate_leaf,
        elem_a="track_top_plate",
        elem_b="right_trolley",
        name="right trolley bears directly on the track top",
    )
    ctx.expect_contact(
        frame,
        gate_leaf,
        elem_a="track_top_plate",
        elem_b="left_trolley",
        name="left trolley bears directly on the track top",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="y",
        inner_elem="right_trolley",
        outer_elem="track_top_plate",
        margin=0.0,
        name="right trolley remains laterally captured by the track width",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="y",
        inner_elem="left_trolley",
        outer_elem="track_top_plate",
        margin=0.0,
        name="left trolley remains laterally captured by the track width",
    )
    ctx.expect_gap(
        frame,
        gate_leaf,
        axis="x",
        positive_elem="strike_plate",
        negative_elem="right_stile",
        min_gap=0.02,
        max_gap=0.05,
        name="closed gate leaf sits just off the receiver strike plate",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: GATE_TRAVEL}):
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="x",
            positive_elem="strike_plate",
            negative_elem="right_stile",
            min_gap=1.0,
            name="open gate clears the receiver side",
        )
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="x",
            elem_a="right_trolley",
            elem_b="track_top_plate",
            min_overlap=0.08,
            name="right trolley remains under the track at full open",
        )
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="x",
            elem_a="left_trolley",
            elem_b="track_top_plate",
            min_overlap=0.08,
            name="left trolley remains under the track at full open",
        )
        open_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf slides left to open",
        rest_pos is not None and open_pos is not None and open_pos[0] < rest_pos[0] - 1.0,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
