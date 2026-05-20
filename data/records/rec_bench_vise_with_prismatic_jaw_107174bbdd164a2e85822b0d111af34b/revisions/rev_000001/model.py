from __future__ import annotations

import math

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


def _aabb_center(
    aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None,
) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bench_vise")

    cast_iron = model.material("cast_iron", rgba=(0.29, 0.33, 0.38, 1.0))
    jaw_steel = model.material("jaw_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    bright_steel = model.material("bright_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))

    screw_z = 0.014

    body = model.part("body")
    body.visual(
        Box((0.050, 0.068, 0.012)),
        origin=Origin(xyz=(-0.020, 0.0, 0.006)),
        material=cast_iron,
        name="base_pad",
    )
    body.visual(
        Box((0.042, 0.056, 0.052)),
        origin=Origin(xyz=(-0.024, 0.0, 0.038)),
        material=cast_iron,
        name="rear_casting",
    )
    body.visual(
        Box((0.090, 0.024, 0.012)),
        origin=Origin(xyz=(0.042, 0.0, 0.030)),
        material=cast_iron,
        name="guide",
    )
    body.visual(
        Box((0.028, 0.009, 0.020)),
        origin=Origin(xyz=(0.006, -0.0105, screw_z)),
        material=cast_iron,
        name="nose_side_0",
    )
    body.visual(
        Box((0.028, 0.009, 0.020)),
        origin=Origin(xyz=(0.006, 0.0105, screw_z)),
        material=cast_iron,
        name="nose_side_1",
    )
    body.visual(
        Box((0.022, 0.042, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0, 0.069)),
        material=cast_iron,
        name="anvil",
    )
    body.visual(
        Box((0.0032, 0.050, 0.026)),
        origin=Origin(xyz=(-0.0016, 0.0, 0.050)),
        material=jaw_steel,
        name="fixed_jaw_face",
    )

    jaw = model.part("jaw")
    jaw.visual(
        Box((0.010, 0.050, 0.028)),
        origin=Origin(xyz=(0.008, 0.0, 0.050)),
        material=cast_iron,
        name="jaw_backing",
    )
    jaw.visual(
        Box((0.046, 0.034, 0.004)),
        origin=Origin(xyz=(0.033, 0.0, 0.038)),
        material=cast_iron,
        name="guide_cap",
    )
    jaw.visual(
        Box((0.046, 0.034, 0.004)),
        origin=Origin(xyz=(0.033, 0.0, 0.022)),
        material=cast_iron,
        name="guide_floor",
    )
    jaw.visual(
        Box((0.046, 0.005, 0.020)),
        origin=Origin(xyz=(0.033, -0.0145, 0.030)),
        material=cast_iron,
        name="guide_wall_0",
    )
    jaw.visual(
        Box((0.046, 0.005, 0.020)),
        origin=Origin(xyz=(0.033, 0.0145, 0.030)),
        material=cast_iron,
        name="guide_wall_1",
    )
    jaw.visual(
        Box((0.036, 0.010, 0.014)),
        origin=Origin(xyz=(0.036, -0.011, screw_z)),
        material=cast_iron,
        name="nut_cheek_0",
    )
    jaw.visual(
        Box((0.036, 0.010, 0.014)),
        origin=Origin(xyz=(0.036, 0.011, screw_z)),
        material=cast_iron,
        name="nut_cheek_1",
    )
    jaw.visual(
        Box((0.015, 0.018, 0.018)),
        origin=Origin(xyz=(0.016, 0.0, 0.045)),
        material=cast_iron,
        name="bridge",
    )
    jaw.visual(
        Box((0.0032, 0.050, 0.026)),
        origin=Origin(xyz=(0.0016, 0.0, 0.050)),
        material=jaw_steel,
        name="jaw_face",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.PRISMATIC,
        parent=body,
        child=jaw,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.05,
            lower=0.0,
            upper=0.055,
        ),
    )

    screw = model.part("screw")
    screw.visual(
        Cylinder(radius=0.0052, length=0.140),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="shaft",
    )
    screw.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(-0.001, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="rear_collar",
    )
    screw.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.134, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bright_steel,
        name="hub",
    )
    screw.visual(
        Cylinder(radius=0.0035, length=0.040),
        origin=Origin(xyz=(0.134, 0.020, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=bright_steel,
        name="handle_arm",
    )
    screw.visual(
        Box((0.010, 0.012, 0.008)),
        origin=Origin(xyz=(0.134, 0.040, 0.0)),
        material=bright_steel,
        name="handle_lug",
    )

    model.articulation(
        "body_to_screw",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=screw,
        origin=Origin(xyz=(0.0, 0.0, screw_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=18.0),
    )

    grip = model.part("grip")
    grip.visual(
        Box((0.008, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=grip_black,
        name="grip_knuckle",
    )
    grip.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=grip_black,
        name="grip_body",
    )

    model.articulation(
        "screw_to_grip",
        ArticulationType.REVOLUTE,
        parent=screw,
        child=grip,
        origin=Origin(xyz=(0.134, 0.040, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    jaw = object_model.get_part("jaw")
    screw = object_model.get_part("screw")
    grip = object_model.get_part("grip")

    jaw_slide = object_model.get_articulation("body_to_jaw")
    screw_spin = object_model.get_articulation("body_to_screw")
    grip_fold = object_model.get_articulation("screw_to_grip")

    jaw_open = jaw_slide.motion_limits.upper if jaw_slide.motion_limits is not None else 0.0

    ctx.expect_gap(
        jaw,
        body,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.0035,
        max_gap=0.0045,
        name="jaw starts with a slim clamping gap",
    )
    ctx.expect_overlap(
        jaw,
        body,
        axes="yz",
        min_overlap=0.020,
        name="front jaw stays aligned with the guide stack",
    )
    ctx.expect_within(
        screw,
        jaw,
        axes="yz",
        inner_elem="shaft",
        margin=0.0,
        name="screw shaft stays centered through the front jaw body",
    )
    ctx.expect_overlap(
        screw,
        jaw,
        axes="x",
        elem_a="shaft",
        min_overlap=0.045,
        name="front jaw starts engaged on the screw",
    )

    rest_jaw_pos = ctx.part_world_position(jaw)
    with ctx.pose({jaw_slide: jaw_open}):
        ctx.expect_gap(
            jaw,
            body,
            axis="x",
            positive_elem="jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.058,
            max_gap=0.060,
            name="jaw opens to a small hobby-vise work gap",
        )
        ctx.expect_overlap(
            jaw,
            body,
            axes="yz",
            min_overlap=0.020,
            name="front jaw stays aligned when opened",
        )
        ctx.expect_within(
            screw,
            jaw,
            axes="yz",
            inner_elem="shaft",
            margin=0.0,
            name="screw shaft stays centered through the front jaw at full opening",
        )
        ctx.expect_overlap(
            screw,
            jaw,
            axes="x",
            elem_a="shaft",
            min_overlap=0.040,
            name="front jaw retains screw engagement when opened",
        )
        open_jaw_pos = ctx.part_world_position(jaw)

    ctx.check(
        "front jaw slides forward to open",
        rest_jaw_pos is not None
        and open_jaw_pos is not None
        and open_jaw_pos[0] > rest_jaw_pos[0] + 0.050,
        details=f"rest={rest_jaw_pos}, open={open_jaw_pos}",
    )

    rest_grip_pivot = ctx.part_world_position(grip)
    with ctx.pose({screw_spin: math.pi / 2.0}):
        spun_grip_pivot = ctx.part_world_position(grip)

    ctx.check(
        "screw rotation carries the handle grip around the screw axis",
        rest_grip_pivot is not None
        and spun_grip_pivot is not None
        and spun_grip_pivot[2] > rest_grip_pivot[2] + 0.030
        and abs(spun_grip_pivot[1]) < 0.010,
        details=f"rest={rest_grip_pivot}, spun={spun_grip_pivot}",
    )

    unfolded_grip = _aabb_center(ctx.part_element_world_aabb(grip, elem="grip_body"))
    with ctx.pose({grip_fold: 1.20}):
        folded_grip = _aabb_center(ctx.part_element_world_aabb(grip, elem="grip_body"))

    ctx.check(
        "folding grip tucks forward around the handle end",
        unfolded_grip is not None
        and folded_grip is not None
        and folded_grip[0] > unfolded_grip[0] + 0.010
        and folded_grip[2] > unfolded_grip[2] + 0.006,
        details=f"unfolded={unfolded_grip}, folded={folded_grip}",
    )

    return ctx.report()


object_model = build_object_model()
