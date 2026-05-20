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
    model = ArticulatedObject(name="drafting_table")

    # BASE
    base = model.part("base")
    base.visual(Box((0.096, 0.096, 0.9)), origin=Origin(xyz=(0.0, 0.0, 0.45)), name="upright")
    base.visual(Box((0.6, 0.05, 0.05)), origin=Origin(xyz=(0.0, 0.0, 0.025)), name="crossbar")
    base.visual(Box((0.05, 0.6, 0.05)), origin=Origin(xyz=(-0.3, 0.0, 0.025)), name="left_foot")
    base.visual(Box((0.05, 0.6, 0.05)), origin=Origin(xyz=(0.3, 0.0, 0.025)), name="right_foot")

    # CARRIAGE
    carriage = model.part("carriage")
    # sleeve walls (inner hole 0.1 x 0.1)
    carriage.visual(Box((0.12, 0.01, 0.2)), origin=Origin(xyz=(0.0, -0.055, 0.1)), name="sleeve_front")
    carriage.visual(Box((0.12, 0.01, 0.2)), origin=Origin(xyz=(0.0, 0.055, 0.1)), name="sleeve_back")
    carriage.visual(Box((0.01, 0.1, 0.2)), origin=Origin(xyz=(-0.055, 0.0, 0.1)), name="sleeve_left")
    carriage.visual(Box((0.01, 0.1, 0.2)), origin=Origin(xyz=(0.055, 0.0, 0.1)), name="sleeve_right")
    # tilt frame
    carriage.visual(Box((0.315, 0.05, 0.05)), origin=Origin(xyz=(-0.2175, 0.0, 0.175)), name="left_beam")
    carriage.visual(Box((0.315, 0.05, 0.05)), origin=Origin(xyz=(0.2175, 0.0, 0.175)), name="right_beam")
    carriage.visual(Box((0.05, 0.05, 0.5)), origin=Origin(xyz=(-0.4, 0.0, 0.45)), name="left_arm")
    carriage.visual(Box((0.05, 0.05, 0.5)), origin=Origin(xyz=(0.4, 0.0, 0.45)), name="right_arm")
    carriage.visual(Cylinder(radius=0.01, length=0.03), origin=Origin(xyz=(-0.365, 0.0, 0.65), rpy=(0.0, 1.5708, 0.0)), name="left_pin")
    carriage.visual(Cylinder(radius=0.01, length=0.03), origin=Origin(xyz=(0.365, 0.0, 0.65), rpy=(0.0, 1.5708, 0.0)), name="right_pin")

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.4)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.0, lower=0.0, upper=0.3),
    )

    # BOARD
    board = model.part("board")
    board.visual(Box((1.0, 0.7, 0.02)), origin=Origin(xyz=(0.0, 0.0, 0.08)), name="surface")
    board.visual(Box((0.02, 0.1, 0.08)), origin=Origin(xyz=(-0.36, 0.0, 0.04)), name="left_bracket")
    board.visual(Box((0.02, 0.1, 0.08)), origin=Origin(xyz=(0.36, 0.0, 0.04)), name="right_bracket")

    model.articulation(
        "carriage_to_board",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=board,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.0, lower=0.0, upper=1.57),
    )

    # KNOB
    knob = model.part("knob")
    knob.visual(Cylinder(radius=0.03, length=0.04), origin=Origin(xyz=(0.02, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="head")
    knob.visual(Cylinder(radius=0.01, length=0.025), origin=Origin(xyz=(-0.0125, 0.0, 0.0), rpy=(0.0, 1.5708, 0.0)), name="shaft")

    model.articulation(
        "carriage_to_knob",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=knob,
        origin=Origin(xyz=(0.425, 0.0, 0.65)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0, lower=-3.14, upper=3.14),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Allowances
    ctx.allow_isolated_part("carriage", reason="The carriage slides on the upright with a small clearance gap.")
    ctx.allow_isolated_part("board", reason="The board is mounted to the floating carriage.")
    ctx.allow_isolated_part("knob", reason="The knob is mounted to the floating carriage.")

    ctx.allow_overlap(
        "carriage", "board",
        elem_a="left_pin", elem_b="left_bracket",
        reason="The pivot pin is intentionally captured inside the board bracket."
    )
    ctx.allow_overlap(
        "carriage", "board",
        elem_a="right_pin", elem_b="right_bracket",
        reason="The pivot pin is intentionally captured inside the board bracket."
    )
    ctx.allow_overlap(
        "knob", "carriage",
        elem_a="shaft", elem_b="right_arm",
        reason="The locking knob shaft is inserted into the carriage arm."
    )

    # Exact checks
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    board = object_model.get_part("board")
    knob = object_model.get_part("knob")

    # Sleeve bounds the upright
    ctx.expect_gap(carriage, base, axis="y", positive_elem="sleeve_back", negative_elem="upright", min_gap=0.001)
    ctx.expect_gap(base, carriage, axis="y", positive_elem="upright", negative_elem="sleeve_front", min_gap=0.001)

    # Board is above arms
    ctx.expect_gap(board, carriage, axis="z", positive_elem="surface", negative_elem="left_arm", min_gap=0.0)
    ctx.expect_gap(board, carriage, axis="z", positive_elem="surface", negative_elem="right_arm", min_gap=0.0)

    # Knob is outside right arm
    ctx.expect_gap(knob, carriage, axis="x", positive_elem="head", negative_elem="right_arm", max_penetration=1e-5)

    # Poses
    slide_joint = object_model.get_articulation("base_to_carriage")
    tilt_joint = object_model.get_articulation("carriage_to_board")

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide_joint: 0.3}):
        ext_pos = ctx.part_world_position(carriage)
        ctx.check("carriage moves up", ext_pos is not None and rest_pos is not None and ext_pos[2] > rest_pos[2] + 0.2)
        ctx.expect_overlap(carriage, base, axes="z", elem_a="sleeve_left", elem_b="upright", min_overlap=0.05)

    with ctx.pose({tilt_joint: 1.57}):
        # Check tilt clears the upright
        ctx.expect_gap(base, board, axis="y", positive_elem="upright", negative_elem="surface", min_gap=0.01)

    return ctx.report()

object_model = build_object_model()
