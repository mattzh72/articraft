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

    galvanized = model.material("galvanized", rgba=(0.62, 0.65, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.10, 0.11, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((0.10, 0.10, 2.40)),
        origin=Origin(xyz=(0.05, 0.0, 1.20)),
        material=dark_steel,
        name="left_post",
    )
    support_frame.visual(
        Box((0.08, 0.08, 2.10)),
        origin=Origin(xyz=(2.45, 0.0, 1.05)),
        material=dark_steel,
        name="receiver_post",
    )
    support_frame.visual(
        Box((3.70, 0.10, 0.10)),
        origin=Origin(xyz=(1.85, 0.0, 2.15)),
        material=dark_steel,
        name="header_beam",
    )
    support_frame.visual(
        Box((0.62, 0.05, 0.05)),
        origin=Origin(xyz=(0.31, 0.0, 1.88), rpy=(0.0, -0.74, 0.0)),
        material=dark_steel,
        name="diagonal_brace",
    )
    for index, bracket_x in enumerate((0.45, 1.85, 3.10), start=1):
        support_frame.visual(
            Box((0.12, 0.04, 0.05)),
            origin=Origin(xyz=(bracket_x, 0.065, 2.125)),
            material=dark_steel,
            name=f"track_bracket_{index}",
        )
    support_frame.visual(
        Box((3.70, 0.12, 0.012)),
        origin=Origin(xyz=(1.85, 0.13, 2.10)),
        material=dark_steel,
        name="track_top",
    )
    support_frame.visual(
        Box((3.70, 0.012, 0.078)),
        origin=Origin(xyz=(1.85, 0.076, 2.055)),
        material=dark_steel,
        name="track_back_wall",
    )
    support_frame.visual(
        Box((3.70, 0.012, 0.078)),
        origin=Origin(xyz=(1.85, 0.184, 2.055)),
        material=dark_steel,
        name="track_front_wall",
    )
    support_frame.visual(
        Box((0.012, 0.12, 0.09)),
        origin=Origin(xyz=(3.694, 0.13, 2.055)),
        material=dark_steel,
        name="track_end_stop",
    )
    support_frame.inertial = Inertial.from_geometry(
        Box((3.70, 0.22, 2.40)),
        mass=180.0,
        origin=Origin(xyz=(1.85, 0.04, 1.20)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((2.24, 0.04, 0.06)),
        origin=Origin(xyz=(1.12, 0.0, -0.03)),
        material=galvanized,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((2.24, 0.04, 0.06)),
        origin=Origin(xyz=(1.12, 0.0, -1.77)),
        material=galvanized,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.06, 0.04, 1.80)),
        origin=Origin(xyz=(0.03, 0.0, -0.90)),
        material=galvanized,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.06, 0.04, 1.80)),
        origin=Origin(xyz=(2.21, 0.0, -0.90)),
        material=galvanized,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((2.12, 0.03, 0.04)),
        origin=Origin(xyz=(1.12, 0.0, -0.90)),
        material=galvanized,
        name="mid_rail",
    )
    for index, picket_x in enumerate((0.22, 0.42, 0.62, 0.82, 1.02, 1.22, 1.42, 1.62, 1.82, 2.02), start=1):
        gate_leaf.visual(
            Box((0.022, 0.022, 1.68)),
            origin=Origin(xyz=(picket_x, 0.0, -0.90)),
            material=galvanized,
            name=f"picket_{index}",
        )
    gate_leaf.visual(
        Box((0.016, 0.012, 0.184)),
        origin=Origin(xyz=(0.24, 0.0, 0.088)),
        material=galvanized,
        name="front_hanger",
    )
    gate_leaf.visual(
        Box((0.016, 0.012, 0.184)),
        origin=Origin(xyz=(1.78, 0.0, 0.088)),
        material=galvanized,
        name="rear_hanger",
    )
    gate_leaf.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(0.24, 0.0, 0.174), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="front_roller",
    )
    gate_leaf.visual(
        Cylinder(radius=0.020, length=0.028),
        origin=Origin(xyz=(1.78, 0.0, 0.174), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_black,
        name="rear_roller",
    )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((2.24, 0.05, 1.98)),
        mass=95.0,
        origin=Origin(xyz=(1.12, 0.0, -0.81)),
    )

    model.articulation(
        "support_to_gate",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.14, 0.13, 1.90)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.50,
            lower=0.0,
            upper=1.28,
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

    support_frame = object_model.get_part("support_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("support_to_gate")

    ctx.expect_gap(
        gate_leaf,
        support_frame,
        axis="y",
        positive_elem="top_rail",
        negative_elem="header_beam",
        min_gap=0.05,
        max_gap=0.10,
        name="gate leaf runs in front of the fixed frame",
    )
    ctx.expect_gap(
        support_frame,
        gate_leaf,
        axis="x",
        positive_elem="receiver_post",
        negative_elem="right_stile",
        min_gap=0.02,
        max_gap=0.05,
        name="closed gate nearly meets the receiver post",
    )
    ctx.expect_overlap(
        gate_leaf,
        support_frame,
        axes="x",
        elem_a="front_roller",
        elem_b="track_top",
        min_overlap=0.03,
        name="front roller starts captured under the track",
    )
    ctx.expect_gap(
        support_frame,
        gate_leaf,
        axis="z",
        positive_elem="track_top",
        negative_elem="front_roller",
        min_gap=0.0,
        max_gap=0.001,
        name="front roller bears against the track roof",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 1.28}):
        ctx.expect_overlap(
            gate_leaf,
            support_frame,
            axes="x",
            elem_a="rear_roller",
            elem_b="track_top",
            min_overlap=0.03,
            name="rear roller remains retained in the track when open",
        )
        ctx.expect_gap(
            support_frame,
            gate_leaf,
            axis="z",
            positive_elem="track_top",
            negative_elem="rear_roller",
            min_gap=0.0,
            max_gap=0.001,
            name="rear roller bears against the track roof when open",
        )
        open_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf translates toward the parking side",
        rest_pos is not None and open_pos is not None and open_pos[0] > rest_pos[0] + 1.20,
        details=f"rest={rest_pos}, open={open_pos}",
    )
    ctx.expect_origin_gap(
        gate_leaf,
        support_frame,
        axis="y",
        min_gap=0.12,
        max_gap=0.14,
        name="moving leaf origin is offset toward one side of the frame",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
