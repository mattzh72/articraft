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

    powder_black = model.material("powder_black", rgba=(0.16, 0.17, 0.18, 1.0))
    galvanized = model.material("galvanized", rgba=(0.67, 0.69, 0.70, 1.0))
    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.61, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.84, 0.69, 0.14, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.28, 0.24, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="left_footing",
    )
    support_frame.visual(
        Box((0.32, 0.26, 0.12)),
        origin=Origin(xyz=(3.18, 0.0, 0.06)),
        material=concrete,
        name="right_footing",
    )
    support_frame.visual(
        Box((0.08, 0.10, 2.00)),
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        material=powder_black,
        name="left_post",
    )
    support_frame.visual(
        Box((0.10, 0.12, 2.15)),
        origin=Origin(xyz=(3.18, 0.0, 1.075)),
        material=powder_black,
        name="right_post",
    )
    support_frame.visual(
        Box((3.02, 0.036, 0.014)),
        origin=Origin(xyz=(1.53, 0.0, 0.140)),
        material=galvanized,
        name="ground_rail",
    )
    support_frame.visual(
        Box((3.18, 0.09, 0.02)),
        origin=Origin(xyz=(1.59, 0.0, 2.06)),
        material=galvanized,
        name="track_top",
    )
    support_frame.visual(
        Box((3.18, 0.012, 0.06)),
        origin=Origin(xyz=(1.59, -0.039, 2.02)),
        material=galvanized,
        name="track_front_wall",
    )
    support_frame.visual(
        Box((3.18, 0.012, 0.06)),
        origin=Origin(xyz=(1.59, 0.039, 2.02)),
        material=galvanized,
        name="track_back_wall",
    )
    support_frame.visual(
        Box((0.02, 0.09, 0.06)),
        origin=Origin(xyz=(3.17, 0.0, 2.02)),
        material=safety_yellow,
        name="end_cap",
    )
    support_frame.visual(
        Box((0.01, 0.07, 0.16)),
        origin=Origin(xyz=(0.035, 0.0, 1.02)),
        material=galvanized,
        name="latch_receiver",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((3.30, 0.28, 2.18)),
        mass=65.0,
        origin=Origin(xyz=(1.60, 0.0, 1.09)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="left_roller",
    )
    gate_leaf.visual(
        Cylinder(radius=0.018, length=0.022),
        origin=Origin(xyz=(1.25, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="right_roller",
    )
    gate_leaf.visual(
        Box((0.03, 0.012, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, -0.13)),
        material=powder_black,
        name="left_hanger",
    )
    gate_leaf.visual(
        Box((0.03, 0.012, 0.28)),
        origin=Origin(xyz=(1.25, 0.0, -0.13)),
        material=powder_black,
        name="right_hanger",
    )
    gate_leaf.visual(
        Box((1.50, 0.045, 0.05)),
        origin=Origin(xyz=(0.625, 0.0, -0.25)),
        material=powder_black,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.50, 0.045, 0.06)),
        origin=Origin(xyz=(0.625, 0.0, -1.82)),
        material=powder_black,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(xyz=(0.18, 0.0, -1.823), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="front_wheel",
    )
    gate_leaf.visual(
        Cylinder(radius=0.042, length=0.028),
        origin=Origin(xyz=(1.07, 0.0, -1.823), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="rear_wheel",
    )
    gate_leaf.visual(
        Box((0.05, 0.045, 1.62)),
        origin=Origin(xyz=(-0.10, 0.0, -1.035)),
        material=powder_black,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.05, 0.045, 1.62)),
        origin=Origin(xyz=(1.35, 0.0, -1.035)),
        material=powder_black,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((1.36, 0.03, 0.04)),
        origin=Origin(xyz=(0.625, 0.0, -1.035)),
        material=powder_black,
        name="mid_rail",
    )
    for index, x_pos in enumerate((0.06, 0.23, 0.40, 0.57, 0.74, 0.91, 1.08, 1.25), start=1):
        gate_leaf.visual(
            Box((0.018, 0.018, 1.56)),
            origin=Origin(xyz=(x_pos, 0.0, -1.035)),
            material=galvanized,
            name=f"picket_{index}",
        )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.50, 0.06, 1.86)),
        mass=24.0,
        origin=Origin(xyz=(0.625, 0.0, -0.93)),
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.18, 0.0, 2.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.55,
            lower=0.0,
            upper=1.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("support_frame")
    gate = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("gate_slide")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            gate,
            frame,
            axis="x",
            positive_elem="left_stile",
            negative_elem="left_post",
            min_gap=0.01,
            max_gap=0.04,
            name="gate closes near the latch post without intersecting it",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="z",
            positive_elem="track_top",
            negative_elem="left_roller",
            min_gap=0.012,
            max_gap=0.030,
            name="left roller rides beneath the top track",
        )
        ctx.expect_gap(
            frame,
            gate,
            axis="z",
            positive_elem="track_top",
            negative_elem="right_roller",
            min_gap=0.012,
            max_gap=0.030,
            name="right roller rides beneath the top track",
        )

    rest_pos = ctx.part_world_position(gate)
    with ctx.pose({slide: upper}):
        ctx.expect_gap(
            frame,
            gate,
            axis="x",
            positive_elem="end_cap",
            negative_elem="right_roller",
            min_gap=0.01,
            max_gap=0.03,
            name="end cap visually limits the fully open travel",
        )
        open_pos = ctx.part_world_position(gate)

    ctx.check(
        "gate leaf translates to the right when opened",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 1.6,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
