from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.20, 0.22, 1.0))
    galvanized = model.material("galvanized", rgba=(0.70, 0.72, 0.74, 1.0))
    concrete = model.material("concrete", rgba=(0.56, 0.56, 0.54, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.18, 0.22, 0.06)),
        origin=Origin(xyz=(0.05, 0.0, 0.03)),
        material=concrete,
        name="left_footing",
    )
    frame.visual(
        Box((0.18, 0.22, 0.06)),
        origin=Origin(xyz=(2.95, 0.0, 0.03)),
        material=concrete,
        name="right_footing",
    )
    frame.visual(
        Box((0.10, 0.10, 1.26)),
        origin=Origin(xyz=(0.05, 0.0, 0.69)),
        material=powder_coat,
        name="support_post",
    )
    frame.visual(
        Box((0.10, 0.10, 1.26)),
        origin=Origin(xyz=(2.95, 0.0, 0.69)),
        material=powder_coat,
        name="receiver_post",
    )
    frame.visual(
        Box((3.60, 0.12, 0.04)),
        origin=Origin(xyz=(1.20, 0.0, 1.28)),
        material=powder_coat,
        name="track_web",
    )
    frame.visual(
        Box((3.60, 0.012, 0.08)),
        origin=Origin(xyz=(1.20, 0.054, 1.22)),
        material=powder_coat,
        name="track_front_lip",
    )
    frame.visual(
        Box((3.60, 0.012, 0.08)),
        origin=Origin(xyz=(1.20, -0.054, 1.22)),
        material=powder_coat,
        name="track_back_lip",
    )
    frame.visual(
        Box((0.18, 0.06, 0.16)),
        origin=Origin(xyz=(2.86, 0.0, 0.62)),
        material=powder_coat,
        name="receiver_keeper",
    )
    frame.inertial = Inertial.from_geometry(
        Box((3.60, 0.22, 1.32)),
        mass=42.0,
        origin=Origin(xyz=(1.20, 0.0, 0.66)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="left_roller",
    )
    gate_leaf.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(1.95, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=galvanized,
        name="right_roller",
    )
    gate_leaf.visual(
        Box((0.03, 0.014, 0.062)),
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
        material=galvanized,
        name="left_hanger",
    )
    gate_leaf.visual(
        Box((0.03, 0.014, 0.062)),
        origin=Origin(xyz=(1.95, 0.0, -0.056)),
        material=galvanized,
        name="right_hanger",
    )
    gate_leaf.visual(
        Box((2.35, 0.04, 0.04)),
        origin=Origin(xyz=(1.175, 0.0, -0.105)),
        material=powder_coat,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((2.35, 0.04, 0.04)),
        origin=Origin(xyz=(1.175, 0.0, -1.045)),
        material=powder_coat,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.04, 0.04, 0.904)),
        origin=Origin(xyz=(0.02, 0.0, -0.575)),
        material=powder_coat,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.04, 0.04, 0.904)),
        origin=Origin(xyz=(2.33, 0.0, -0.575)),
        material=powder_coat,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((2.27, 0.03, 0.03)),
        origin=Origin(xyz=(1.175, 0.0, -0.575)),
        material=powder_coat,
        name="mid_rail",
    )

    picket_x_positions = (0.30, 0.58, 0.86, 1.14, 1.42, 1.70, 1.98)
    for index, x_pos in enumerate(picket_x_positions, start=1):
        gate_leaf.visual(
            Box((0.02, 0.015, 0.904)),
            origin=Origin(xyz=(x_pos, 0.0, -0.575)),
            material=powder_coat,
            name=f"picket_{index}",
        )

    gate_leaf.inertial = Inertial.from_geometry(
        Box((2.35, 0.08, 1.08)),
        mass=22.0,
        origin=Origin(xyz=(1.175, 0.0, -0.54)),
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.32, 0.0, 1.232)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=0.70,
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
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    with ctx.pose({slide: 0.0}):
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="x",
            positive_elem="receiver_post",
            negative_elem="right_stile",
            min_gap=0.18,
            max_gap=0.28,
            name="closed gate leaf sits near the receiver post",
        )
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="z",
            positive_elem="track_web",
            negative_elem="left_roller",
            min_gap=0.0,
            max_gap=0.001,
            name="left roller stays captured under the top track",
        )
        ctx.expect_gap(
            frame,
            gate_leaf,
            axis="z",
            positive_elem="track_web",
            negative_elem="right_roller",
            min_gap=0.0,
            max_gap=0.001,
            name="right roller stays captured under the top track",
        )

    rest_pos = ctx.part_world_position(gate_leaf)
    if upper is not None:
        with ctx.pose({slide: upper}):
            ctx.expect_gap(
                frame,
                gate_leaf,
                axis="x",
                positive_elem="receiver_post",
                negative_elem="right_stile",
                min_gap=0.88,
                name="open pose clears a wider opening at the receiver side",
            )
            ctx.expect_gap(
                frame,
                gate_leaf,
                axis="z",
                positive_elem="track_web",
                negative_elem="left_roller",
                min_gap=0.0,
                max_gap=0.001,
                name="left roller remains under the track at full travel",
            )
            ctx.expect_gap(
                frame,
                gate_leaf,
                axis="z",
                positive_elem="track_web",
                negative_elem="right_roller",
                min_gap=0.0,
                max_gap=0.001,
                name="right roller remains under the track at full travel",
            )
            open_pos = ctx.part_world_position(gate_leaf)

        ctx.check(
            "gate leaf slides left to open",
            rest_pos is not None
            and open_pos is not None
            and open_pos[0] < rest_pos[0] - 0.60,
            details=f"rest={rest_pos}, open={open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
