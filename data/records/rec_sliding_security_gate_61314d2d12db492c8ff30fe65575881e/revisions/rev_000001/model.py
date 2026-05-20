from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_security_gate")

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.19, 0.20, 1.0))
    galvanized = model.material("galvanized", rgba=(0.65, 0.67, 0.70, 1.0))
    nylon = model.material("nylon", rgba=(0.82, 0.84, 0.86, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.46, 0.12, 2.20)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 1.10)),
    )
    frame.visual(
        Box((0.08, 0.10, 2.20)),
        origin=Origin(xyz=(-0.69, 0.0, 1.10)),
        material=powder_coat,
        name="left_post",
    )
    frame.visual(
        Box((0.08, 0.10, 2.20)),
        origin=Origin(xyz=(0.69, 0.0, 1.10)),
        material=powder_coat,
        name="right_post",
    )
    frame.visual(
        Box((1.46, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 2.14)),
        material=powder_coat,
        name="header_beam",
    )
    frame.visual(
        Box((1.28, 0.07, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 2.071)),
        material=galvanized,
        name="track_top_wall",
    )
    frame.visual(
        Box((1.28, 0.006, 0.054)),
        origin=Origin(xyz=(0.0, -0.027, 2.035)),
        material=galvanized,
        name="track_front_lip",
    )
    frame.visual(
        Box((1.28, 0.006, 0.054)),
        origin=Origin(xyz=(0.0, 0.027, 2.035)),
        material=galvanized,
        name="track_back_lip",
    )
    frame.visual(
        Box((0.10, 0.08, 0.30)),
        origin=Origin(xyz=(0.69, 0.0, 1.00)),
        material=galvanized,
        name="strike_receiver",
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.10, 0.06, 1.92)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
    )

    def add_diagonal_bar(
        name: str,
        start: tuple[float, float],
        end: tuple[float, float],
        *,
        depth: float = 0.018,
        thickness: float = 0.018,
        overshoot: float = 0.010,
    ) -> None:
        dx = end[0] - start[0]
        dz = end[1] - start[1]
        length = math.hypot(dx, dz) + overshoot
        angle = math.atan2(dx, dz)
        gate_leaf.visual(
            Box((thickness, depth, length)),
            origin=Origin(
                xyz=((start[0] + end[0]) * 0.5, 0.0, (start[1] + end[1]) * 0.5),
                rpy=(0.0, angle, 0.0),
            ),
            material=powder_coat,
            name=name,
        )

    gate_leaf.visual(
        Box((0.05, 0.028, 1.90)),
        origin=Origin(xyz=(-0.525, 0.0, 0.0)),
        material=powder_coat,
        name="left_stile",
    )
    gate_leaf.visual(
        Box((0.05, 0.028, 1.90)),
        origin=Origin(xyz=(0.525, 0.0, 0.0)),
        material=powder_coat,
        name="right_stile",
    )
    gate_leaf.visual(
        Box((1.10, 0.028, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=powder_coat,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((1.10, 0.028, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, -0.925)),
        material=powder_coat,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((1.00, 0.020, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=powder_coat,
        name="mid_rail",
    )

    for i, x_pos in enumerate((-0.26, 0.0, 0.26), start=1):
        gate_leaf.visual(
            Box((0.02, 0.018, 1.80)),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=powder_coat,
            name=f"picket_{i}",
        )

    add_diagonal_bar("brace_rise", (-0.48, -0.90), (0.48, 0.90))
    add_diagonal_bar("brace_fall", (-0.48, 0.90), (0.48, -0.90))

    for name, x_pos in (("left", -0.28), ("right", 0.28)):
        gate_leaf.visual(
            Box((0.04, 0.016, 0.084)),
            origin=Origin(xyz=(x_pos, 0.0, 0.99)),
            material=galvanized,
            name=f"{name}_hanger",
        )
        gate_leaf.visual(
            Box((0.06, 0.032, 0.03)),
            origin=Origin(xyz=(x_pos, 0.0, 1.047)),
            material=nylon,
            name=f"{name}_guide_shoe",
        )

    gate_leaf.visual(
        Box((0.07, 0.040, 0.18)),
        origin=Origin(xyz=(0.50, 0.0, 0.0)),
        material=galvanized,
        name="lock_box",
    )
    gate_leaf.visual(
        Box((0.025, 0.035, 0.32)),
        origin=Origin(xyz=(0.455, 0.0, 0.0)),
        material=galvanized,
        name="pull_handle",
    )

    model.articulation(
        "frame_to_gate_leaf",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.30,
            lower=0.0,
            upper=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    gate_leaf = object_model.get_part("gate_leaf")
    slide = object_model.get_articulation("frame_to_gate_leaf")

    ctx.expect_origin_distance(
        gate_leaf,
        frame,
        axes="y",
        max_dist=0.001,
        name="gate leaf stays centered on the main body depth axis",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="y",
        inner_elem="left_guide_shoe",
        outer_elem="track_top_wall",
        margin=0.0,
        name="left guide shoe stays within the top track width",
    )
    ctx.expect_within(
        gate_leaf,
        frame,
        axes="y",
        inner_elem="right_guide_shoe",
        outer_elem="track_top_wall",
        margin=0.0,
        name="right guide shoe stays within the top track width",
    )
    ctx.expect_contact(
        frame,
        gate_leaf,
        elem_a="track_top_wall",
        elem_b="left_guide_shoe",
        name="left guide shoe bears on the top track roof",
    )
    ctx.expect_contact(
        frame,
        gate_leaf,
        elem_a="track_top_wall",
        elem_b="right_guide_shoe",
        name="right guide shoe bears on the top track roof",
    )

    rest_pos = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 0.18}):
        ctx.expect_within(
            gate_leaf,
            frame,
            axes="y",
            inner_elem="right_guide_shoe",
            outer_elem="track_top_wall",
            margin=0.0,
            name="guide shoe stays centered in the track at full extension",
        )
        ctx.expect_overlap(
            gate_leaf,
            frame,
            axes="x",
            elem_a="right_guide_shoe",
            elem_b="track_top_wall",
            min_overlap=0.04,
            name="top guide shoe remains captured along the track travel",
        )
        ctx.expect_contact(
            frame,
            gate_leaf,
            elem_a="track_top_wall",
            elem_b="right_guide_shoe",
            name="right guide shoe still bears on the track roof at full extension",
        )
        extended_pos = ctx.part_world_position(gate_leaf)

    ctx.check(
        "gate leaf translates in the positive track direction",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
