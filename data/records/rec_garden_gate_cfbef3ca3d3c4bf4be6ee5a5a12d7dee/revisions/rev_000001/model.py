from __future__ import annotations

from math import pi, sqrt, atan2

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
    model = ArticulatedObject(name="sliding_garden_gate")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_green = model.material("painted_green_steel", rgba=(0.05, 0.23, 0.13, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    concrete = model.material("weathered_concrete", rgba=(0.46, 0.45, 0.40, 1.0))
    brass = model.material("aged_brass_handle", rgba=(0.72, 0.55, 0.20, 1.0))

    track_posts = model.part("track_posts")
    track_posts.visual(
        Box((5.35, 0.38, 0.04)),
        origin=Origin(xyz=(0.42, 0.00, 0.02)),
        material=concrete,
        name="concrete_strip",
    )
    track_posts.visual(
        Box((5.15, 0.055, 0.06)),
        origin=Origin(xyz=(0.42, 0.00, 0.07)),
        material=galvanized,
        name="track_rail",
    )
    track_posts.visual(
        Box((0.14, 0.14, 1.56)),
        origin=Origin(xyz=(-1.70, 0.22, 0.82)),
        material=galvanized,
        name="latch_post",
    )
    track_posts.visual(
        Box((0.13, 0.13, 1.60)),
        origin=Origin(xyz=(1.58, 0.22, 0.84)),
        material=galvanized,
        name="rear_guide_post",
    )
    track_posts.visual(
        Box((0.08, 0.22, 0.08)),
        origin=Origin(xyz=(-1.62, 0.09, 0.96)),
        material=galvanized,
        name="latch_receiver",
    )
    track_posts.visual(
        Box((0.22, 0.075, 0.06)),
        origin=Origin(xyz=(1.49, 0.22, 1.55)),
        material=galvanized,
        name="guide_post_arm",
    )
    track_posts.visual(
        Box((0.08, 0.42, 0.06)),
        origin=Origin(xyz=(1.40, 0.06, 1.55)),
        material=galvanized,
        name="guide_crossbar",
    )
    for y, roller_name, pin_name in (
        (0.078, "guide_roller_outer", "guide_pin_outer"),
        (-0.078, "guide_roller_inner", "guide_pin_inner"),
    ):
        track_posts.visual(
            Cylinder(radius=0.035, length=0.27),
            origin=Origin(xyz=(1.40, y, 1.39)),
            material=black_rubber,
            name=roller_name,
        )
        track_posts.visual(
            Cylinder(radius=0.012, length=0.30),
            origin=Origin(xyz=(1.40, y, 1.40)),
            material=galvanized,
            name=pin_name,
        )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((3.08, 0.07, 0.09)),
        origin=Origin(xyz=(0.00, 0.00, 0.36)),
        material=dark_green,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((3.08, 0.07, 0.09)),
        origin=Origin(xyz=(0.00, 0.00, 1.46)),
        material=dark_green,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.09, 0.07, 1.18)),
        origin=Origin(xyz=(-1.50, 0.00, 0.91)),
        material=dark_green,
        name="front_stile",
    )
    gate_leaf.visual(
        Box((0.09, 0.07, 1.18)),
        origin=Origin(xyz=(1.50, 0.00, 0.91)),
        material=dark_green,
        name="rear_stile",
    )
    gate_leaf.visual(
        Box((2.86, 0.055, 0.065)),
        origin=Origin(xyz=(0.00, 0.00, 0.92)),
        material=dark_green,
        name="middle_rail",
    )
    for idx, x in enumerate((-1.08, -0.72, -0.36, 0.0, 0.36, 0.72, 1.08)):
        gate_leaf.visual(
            Box((0.045, 0.045, 1.02)),
            origin=Origin(xyz=(x, 0.00, 0.91)),
            material=dark_green,
            name=f"picket_{idx}",
        )

    brace_len = sqrt(2.48 * 2.48 + 0.82 * 0.82)
    brace_angle = -atan2(0.82, 2.48)
    gate_leaf.visual(
        Box((brace_len, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.91), rpy=(0.0, brace_angle, 0.0)),
        material=dark_green,
        name="diagonal_brace",
    )
    gate_leaf.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(-1.52, -0.055, 0.96), rpy=(pi / 2, 0.0, 0.0)),
        material=galvanized,
        name="handle_boss",
    )

    for x, prefix in ((-0.92, "front"), (0.92, "rear")):
        gate_leaf.visual(
            Box((0.24, 0.12, 0.045)),
            origin=Origin(xyz=(x, 0.00, 0.295)),
            material=galvanized,
            name=f"{prefix}_carriage_plate",
        )
        gate_leaf.visual(
            Box((0.055, 0.055, 0.17)),
            origin=Origin(xyz=(x - 0.070, 0.00, 0.245)),
            material=galvanized,
            name=f"{prefix}_carriage_strut_0",
        )
        gate_leaf.visual(
            Box((0.055, 0.055, 0.17)),
            origin=Origin(xyz=(x + 0.070, 0.00, 0.245)),
            material=galvanized,
            name=f"{prefix}_carriage_strut_1",
        )
        gate_leaf.visual(
            Cylinder(radius=0.018, length=0.20),
            origin=Origin(xyz=(x, 0.00, 0.190), rpy=(pi / 2, 0.0, 0.0)),
            material=galvanized,
            name=f"{prefix}_wheel_axle",
        )
        gate_leaf.visual(
            Cylinder(radius=0.090, length=0.070),
            origin=Origin(xyz=(x, 0.00, 0.190), rpy=(pi / 2, 0.0, 0.0)),
            material=black_rubber,
            name=f"{prefix}_wheel",
        )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.052, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=brass,
        name="pivot_hub",
    )
    latch_handle.visual(
        Box((0.36, 0.030, 0.045)),
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
        material=brass,
        name="handle_lever",
    )
    latch_handle.visual(
        Cylinder(radius=0.035, length=0.055),
        origin=Origin(xyz=(0.39, 0.0, 0.0), rpy=(pi / 2, 0.0, 0.0)),
        material=brass,
        name="lever_grip",
    )

    model.articulation(
        "track_to_gate",
        ArticulationType.PRISMATIC,
        parent=track_posts,
        child=gate_leaf,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=650.0, velocity=0.45, lower=0.0, upper=1.20),
    )
    model.articulation(
        "gate_to_handle",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=latch_handle,
        origin=Origin(xyz=(-1.52, -0.0925, 0.96)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    track_posts = object_model.get_part("track_posts")
    gate_leaf = object_model.get_part("gate_leaf")
    latch_handle = object_model.get_part("latch_handle")
    slide = object_model.get_articulation("track_to_gate")
    handle_pivot = object_model.get_articulation("gate_to_handle")

    ctx.expect_gap(
        gate_leaf,
        track_posts,
        axis="z",
        positive_elem="front_wheel",
        negative_elem="track_rail",
        min_gap=0.0,
        max_gap=0.004,
        name="front roller rides on ground track",
    )
    ctx.expect_gap(
        gate_leaf,
        track_posts,
        axis="z",
        positive_elem="rear_wheel",
        negative_elem="track_rail",
        min_gap=0.0,
        max_gap=0.004,
        name="rear roller rides on ground track",
    )
    ctx.expect_gap(
        track_posts,
        gate_leaf,
        axis="y",
        positive_elem="guide_roller_outer",
        negative_elem="top_rail",
        min_gap=0.004,
        max_gap=0.020,
        name="outer guide roller clears top rail",
    )
    ctx.expect_gap(
        gate_leaf,
        track_posts,
        axis="y",
        positive_elem="top_rail",
        negative_elem="guide_roller_inner",
        min_gap=0.004,
        max_gap=0.020,
        name="inner guide roller clears top rail",
    )

    rest_position = ctx.part_world_position(gate_leaf)
    with ctx.pose({slide: 1.20}):
        extended_position = ctx.part_world_position(gate_leaf)
        ctx.expect_overlap(
            gate_leaf,
            track_posts,
            axes="x",
            elem_a="top_rail",
            elem_b="guide_crossbar",
            min_overlap=0.04,
            name="open gate remains under rear guide",
        )

    ctx.check(
        "gate leaf translates along fence line",
        rest_position is not None
        and extended_position is not None
        and extended_position[0] > rest_position[0] + 1.15
        and abs(extended_position[1] - rest_position[1]) < 1e-6,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    rest_grip = ctx.part_element_world_aabb(latch_handle, elem="lever_grip")
    with ctx.pose({handle_pivot: 0.65}):
        turned_grip = ctx.part_element_world_aabb(latch_handle, elem="lever_grip")

    def _center_z(aabb):
        return None if aabb is None else 0.5 * (aabb[0][2] + aabb[1][2])

    ctx.check(
        "latch handle rotates downward on pivot",
        _center_z(rest_grip) is not None
        and _center_z(turned_grip) is not None
        and _center_z(turned_grip) < _center_z(rest_grip) - 0.18,
        details=f"rest_grip={rest_grip}, turned_grip={turned_grip}",
    )

    return ctx.report()


object_model = build_object_model()
