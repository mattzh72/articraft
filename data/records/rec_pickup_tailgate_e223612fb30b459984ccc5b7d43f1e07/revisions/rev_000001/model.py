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
    model = ArticulatedObject(name="pickup_tailgate")

    bed = model.part("bed")
    # Bed floor
    bed.visual(
        Box((1.0, 1.7, 0.1)),
        origin=Origin(xyz=(0.5, 0.0, 0.05)),
        name="bed_floor"
    )
    # Front wall
    bed.visual(
        Box((0.1, 1.7, 0.6)),
        origin=Origin(xyz=(0.95, 0.0, 0.4)),
        name="front_wall"
    )
    # Left side
    bed.visual(
        Box((1.0, 0.1, 0.6)),
        origin=Origin(xyz=(0.5, 0.8, 0.4)),
        name="left_side"
    )
    # Right side
    bed.visual(
        Box((1.0, 0.1, 0.6)),
        origin=Origin(xyz=(0.5, -0.8, 0.4)),
        name="right_side"
    )
    # Left striker pin
    bed.visual(
        Cylinder(radius=0.01, length=0.02),
        origin=Origin(xyz=(0.0, 0.74, 0.6), rpy=(1.5708, 0.0, 0.0)),
        name="left_striker"
    )
    # Right striker pin
    bed.visual(
        Cylinder(radius=0.01, length=0.02),
        origin=Origin(xyz=(0.0, -0.74, 0.6), rpy=(1.5708, 0.0, 0.0)),
        name="right_striker"
    )

    tailgate = model.part("tailgate")
    # Outer panel
    tailgate.visual(
        Box((0.04, 1.46, 0.6)),
        origin=Origin(xyz=(-0.02, 0.0, 0.3)),
        name="outer_panel"
    )
    # Inner base panel
    tailgate.visual(
        Box((0.02, 1.36, 0.55)),
        origin=Origin(xyz=(0.01, 0.0, 0.3)),
        name="inner_base"
    )
    # Inner rims (framing)
    tailgate.visual(
        Box((0.02, 0.05, 0.55)),
        origin=Origin(xyz=(0.03, 0.655, 0.3)),
        name="inner_left_rim"
    )
    tailgate.visual(
        Box((0.02, 0.05, 0.55)),
        origin=Origin(xyz=(0.03, -0.655, 0.3)),
        name="inner_right_rim"
    )
    tailgate.visual(
        Box((0.02, 1.26, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.05)),
        name="inner_bottom_rim"
    )
    tailgate.visual(
        Box((0.02, 1.26, 0.05)),
        origin=Origin(xyz=(0.03, 0.0, 0.55)),
        name="inner_top_rim"
    )
    # Top cap
    tailgate.visual(
        Box((0.08, 1.48, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
        name="top_cap"
    )
    # Left latch
    tailgate.visual(
        Box((0.04, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, 0.74, 0.5)),
        name="left_latch"
    )
    # Right latch
    tailgate.visual(
        Box((0.04, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, -0.74, 0.5)),
        name="right_latch"
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=bed,
        child=tailgate,
        origin=Origin(xyz=(0.0, 0.0, 0.1)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.5708, effort=50.0, velocity=2.0)
    )

    return model

def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    bed = object_model.get_part("bed")
    tailgate = object_model.get_part("tailgate")
    hinge = object_model.get_articulation("tailgate_hinge")

    ctx.allow_overlap(
        bed,
        tailgate,
        elem_a="left_striker",
        elem_b="left_latch",
        reason="The tailgate latch intentionally captures the bed side striker pin."
    )
    ctx.allow_overlap(
        bed,
        tailgate,
        elem_a="right_striker",
        elem_b="right_latch",
        reason="The tailgate latch intentionally captures the bed side striker pin."
    )

    # Rest pose (closed) checks
    ctx.expect_overlap(bed, tailgate, axes="y", elem_a="left_striker", elem_b="left_latch", min_overlap=0.01)
    ctx.expect_overlap(bed, tailgate, axes="y", elem_a="right_striker", elem_b="right_latch", min_overlap=0.01)

    ctx.expect_gap(bed, tailgate, axis="x", positive_elem="bed_floor", negative_elem="outer_panel", min_gap=0.0)

    # Pose check (open)
    with ctx.pose({hinge: 1.5708}):
        ctx.expect_gap(bed, tailgate, axis="x", max_penetration=0.001, positive_elem="bed_floor", negative_elem="outer_panel", name="Tailgate clears the bed floor when open")

    return ctx.report()

object_model = build_object_model()
