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
    model = ArticulatedObject(name="bench_xz_transfer_stage")

    # 1. Base rail (fixed)
    base = model.part("base")
    base.visual(
        Box((0.60, 0.08, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        name="base_rail"
    )

    # 2. Horizontal carriage (moves along X)
    carriage = model.part("carriage")
    # Saddle top
    carriage.visual(
        Box((0.12, 0.10, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        name="saddle_top"
    )
    # Saddle wrap front
    carriage.visual(
        Box((0.12, 0.01, 0.02)),
        origin=Origin(xyz=(0.0, 0.045, 0.03)),
        name="saddle_wrap_front"
    )
    # Saddle wrap back
    carriage.visual(
        Box((0.12, 0.01, 0.02)),
        origin=Origin(xyz=(0.0, -0.045, 0.03)),
        name="saddle_wrap_back"
    )
    # Upright mast
    carriage.visual(
        Box((0.06, 0.06, 0.40)),
        origin=Origin(xyz=(0.0, -0.02, 0.26)),
        name="mast"
    )

    # 3. Vertical head (moves along Z)
    head = model.part("head")
    # Head front block
    head.visual(
        Box((0.10, 0.06, 0.08)),
        origin=Origin(xyz=(0.0, 0.04, 0.26)),
        name="head_front"
    )
    # Head wrap left
    head.visual(
        Box((0.01, 0.03, 0.08)),
        origin=Origin(xyz=(0.035, -0.005, 0.26)),
        name="head_wrap_left"
    )
    # Head wrap right
    head.visual(
        Box((0.01, 0.03, 0.08)),
        origin=Origin(xyz=(-0.035, -0.005, 0.26)),
        name="head_wrap_right"
    )

    # Articulations
    model.articulation(
        name="base_to_carriage",
        articulation_type=ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.24, upper=0.24),
    )

    model.articulation(
        name="carriage_to_head",
        articulation_type=ArticulationType.PRISMATIC,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.15, upper=0.15),
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("head")

    x_joint = object_model.get_articulation("base_to_carriage")
    z_joint = object_model.get_articulation("carriage_to_head")

    # Check that carriage straddles the base rail
    ctx.expect_within(
        carriage,
        base,
        axes="y",
        inner_elem="saddle_top",
        outer_elem="base_rail",
        margin=0.015, # Saddle top Y is +/-0.05, rail Y is +/-0.04. So margin of 0.01 is expected.
        name="carriage sits on top of rail in Y"
    )
    
    ctx.expect_overlap(
        carriage,
        base,
        axes="x",
        elem_a="saddle_top",
        elem_b="base_rail",
        min_overlap=0.10,
        name="carriage overlaps rail in X"
    )

    ctx.expect_contact(
        carriage,
        base,
        elem_a="saddle_top",
        elem_b="base_rail",
        name="carriage contacts rail"
    )

    # Check that head straddles the mast
    ctx.expect_contact(
        head,
        carriage,
        elem_a="head_front",
        elem_b="mast",
        name="head contacts mast"
    )

    with ctx.pose({x_joint: 0.24}):
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            elem_a="saddle_top",
            elem_b="base_rail",
            min_overlap=0.10,
            name="carriage retains overlap at max X"
        )
    
    with ctx.pose({z_joint: 0.15}):
        ctx.expect_overlap(
            head,
            carriage,
            axes="z",
            elem_a="head_front",
            elem_b="mast",
            min_overlap=0.06,
            name="head retains overlap at max Z"
        )

    return ctx.report()

object_model = build_object_model()
