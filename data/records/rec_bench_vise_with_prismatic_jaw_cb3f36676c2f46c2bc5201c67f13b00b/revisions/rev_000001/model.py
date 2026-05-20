from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machinist_bench_vise")

    base_iron = model.material("base_iron", rgba=(0.20, 0.23, 0.26, 1.0))
    body_blue = model.material("body_blue", rgba=(0.14, 0.30, 0.56, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.84, 0.85, 0.86, 1.0))
    black = model.material("black", rgba=(0.10, 0.10, 0.11, 1.0))

    lock_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.028,
            0.016,
            body_style="mushroom",
            top_diameter=0.022,
        ),
        "jaw_lock_knob",
    )

    swivel_base = model.part("swivel_base")
    swivel_base.visual(
        Cylinder(radius=0.120, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_iron,
        name="base_plate",
    )
    swivel_base.visual(
        Box((0.260, 0.090, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=base_iron,
        name="mounting_wings",
    )
    swivel_base.visual(
        Cylinder(radius=0.100, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=base_iron,
        name="swivel_race",
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.098, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=body_blue,
        name="turntable",
    )
    body.visual(
        Box((0.090, 0.082, 0.054)),
        origin=Origin(xyz=(-0.115, 0.0, 0.040)),
        material=body_blue,
        name="pedestal",
    )
    body.visual(
        Box((0.070, 0.115, 0.110)),
        origin=Origin(xyz=(-0.115, 0.0, 0.108)),
        material=body_blue,
        name="fixed_jaw_block",
    )
    body.visual(
        Box((0.006, 0.096, 0.038)),
        origin=Origin(xyz=(-0.078, 0.0, 0.144)),
        material=steel,
        name="fixed_jaw_face",
    )
    body.visual(
        Box((0.072, 0.090, 0.016)),
        origin=Origin(xyz=(-0.115, 0.0, 0.171)),
        material=steel,
        name="anvil",
    )
    body.visual(
        Box((0.040, 0.060, 0.028)),
        origin=Origin(xyz=(-0.095, 0.0, 0.076)),
        material=body_blue,
        name="rear_brace",
    )
    body.visual(
        Box((0.034, 0.038, 0.034)),
        origin=Origin(xyz=(-0.062, 0.0, 0.105)),
        material=body_blue,
        name="jaw_saddle",
    )
    body.visual(
        Box((0.218, 0.038, 0.032)),
        origin=Origin(xyz=(0.059, 0.0, 0.105)),
        material=steel,
        name="guide_beam",
    )
    body.visual(
        Box((0.050, 0.056, 0.030)),
        origin=Origin(xyz=(0.145, 0.0, 0.110)),
        material=body_blue,
        name="front_nose",
    )
    moving_jaw = model.part("moving_jaw")
    moving_jaw.visual(
        Box((0.006, 0.094, 0.038)),
        origin=Origin(xyz=(0.003, 0.0, 0.039)),
        material=steel,
        name="jaw_face",
    )
    moving_jaw.visual(
        Box((0.075, 0.110, 0.050)),
        origin=Origin(xyz=(0.0375, 0.0, 0.046)),
        material=body_blue,
        name="top_bridge",
    )
    moving_jaw.visual(
        Box((0.158, 0.021, 0.092)),
        origin=Origin(xyz=(0.091, -0.0315, 0.006)),
        material=body_blue,
        name="left_cheek",
    )
    moving_jaw.visual(
        Box((0.158, 0.021, 0.092)),
        origin=Origin(xyz=(0.091, 0.0315, 0.006)),
        material=body_blue,
        name="right_cheek",
    )
    moving_jaw.visual(
        Box((0.062, 0.070, 0.038)),
        origin=Origin(xyz=(0.031, 0.0, -0.040)),
        material=body_blue,
        name="lower_chin",
    )
    moving_jaw.visual(
        Box((0.020, 0.070, 0.020)),
        origin=Origin(xyz=(0.165, 0.0, 0.034)),
        material=body_blue,
        name="rear_cap",
    )
    moving_jaw.visual(
        Cylinder(radius=0.020, length=0.130),
        origin=Origin(
            xyz=(0.125, 0.0, -0.040),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="screw_sleeve",
    )
    moving_jaw.visual(
        Cylinder(radius=0.026, length=0.016),
        origin=Origin(
            xyz=(0.198, 0.0, -0.040),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="front_hub",
    )
    moving_jaw.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(
            xyz=(0.115, 0.053, 0.000),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="lock_boss",
    )

    main_handle = model.part("main_handle")
    main_handle.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(
            xyz=(0.026, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=polished_steel,
        name="hub",
    )
    main_handle.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(
            xyz=(0.032, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=polished_steel,
        name="handle_bar",
    )
    main_handle.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.032, -0.098, 0.0)),
        material=polished_steel,
        name="handle_end_0",
    )
    main_handle.visual(
        Sphere(radius=0.013),
        origin=Origin(xyz=(0.032, 0.098, 0.0)),
        material=polished_steel,
        name="handle_end_1",
    )

    jaw_lock = model.part("jaw_lock")
    jaw_lock.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.010, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="lock_shaft",
    )
    jaw_lock.visual(
        lock_knob_mesh,
        origin=Origin(
            xyz=(0.0, 0.026, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black,
        name="knob",
    )
    jaw_lock.visual(
        Cylinder(radius=0.0035, length=0.024),
        origin=Origin(xyz=(0.0, 0.036, 0.0)),
        material=steel,
        name="thumb_bar",
    )

    model.articulation(
        "base_swivel",
        ArticulationType.REVOLUTE,
        parent=swivel_base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=1.2,
            lower=-1.40,
            upper=1.40,
        ),
    )
    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=moving_jaw,
        origin=Origin(xyz=(-0.070, 0.0, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.18,
            lower=0.0,
            upper=0.135,
        ),
    )
    model.articulation(
        "screw_handle_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=main_handle,
        origin=Origin(xyz=(0.206, 0.0, -0.040)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=10.0),
    )
    model.articulation(
        "guide_lock_spin",
        ArticulationType.CONTINUOUS,
        parent=moving_jaw,
        child=jaw_lock,
        origin=Origin(xyz=(0.115, 0.065, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=8.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def _aabb_size(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple(hi[i] - lo[i] for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    swivel_base = object_model.get_part("swivel_base")
    body = object_model.get_part("body")
    moving_jaw = object_model.get_part("moving_jaw")
    main_handle = object_model.get_part("main_handle")
    jaw_slide = object_model.get_articulation("jaw_slide")
    base_swivel = object_model.get_articulation("base_swivel")
    screw_handle_spin = object_model.get_articulation("screw_handle_spin")
    guide_lock_spin = object_model.get_articulation("guide_lock_spin")

    ctx.expect_gap(
        moving_jaw,
        body,
        axis="x",
        positive_elem="jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.001,
        max_gap=0.005,
        name="jaw faces nearly close at rest",
    )
    ctx.expect_gap(
        body,
        swivel_base,
        axis="z",
        positive_elem="turntable",
        negative_elem="swivel_race",
        min_gap=0.0,
        max_gap=0.001,
        name="body turntable seats on swivel race",
    )

    rest_pos = ctx.part_world_position(moving_jaw)
    with ctx.pose({jaw_slide: 0.135}):
        extended_pos = ctx.part_world_position(moving_jaw)
        ctx.expect_overlap(
            moving_jaw,
            body,
            axes="x",
            elem_a="left_cheek",
            elem_b="guide_beam",
            min_overlap=0.060,
            name="sliding jaw keeps guide engagement when opened",
        )

    ctx.check(
        "sliding jaw moves outward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    rest_face = _aabb_center(ctx.part_element_world_aabb(body, elem="fixed_jaw_face"))
    with ctx.pose({base_swivel: 0.90}):
        swiveled_face = _aabb_center(ctx.part_element_world_aabb(body, elem="fixed_jaw_face"))
    ctx.check(
        "body swivels on the base",
        rest_face is not None
        and swiveled_face is not None
        and abs(swiveled_face[1]) > 0.04
        and abs(math.hypot(rest_face[0], rest_face[1]) - math.hypot(swiveled_face[0], swiveled_face[1])) < 0.003,
        details=f"rest_face={rest_face}, swiveled_face={swiveled_face}",
    )

    handle_rest = _aabb_size(ctx.part_element_world_aabb(main_handle, elem="handle_bar"))
    with ctx.pose({screw_handle_spin: math.pi / 2.0}):
        handle_quarter = _aabb_size(ctx.part_element_world_aabb(main_handle, elem="handle_bar"))
    ctx.check(
        "main screw handle rotates about the screw axis",
        handle_rest is not None
        and handle_quarter is not None
        and handle_rest[1] > 0.16
        and handle_rest[2] < 0.03
        and handle_quarter[2] > 0.16
        and handle_quarter[1] < 0.03,
        details=f"rest={handle_rest}, quarter_turn={handle_quarter}",
    )

    lock_rest = _aabb_size(ctx.part_element_world_aabb("jaw_lock", elem="thumb_bar"))
    with ctx.pose({guide_lock_spin: math.pi / 2.0}):
        lock_quarter = _aabb_size(ctx.part_element_world_aabb("jaw_lock", elem="thumb_bar"))
    ctx.check(
        "side lock knob rotates on its short shaft",
        lock_rest is not None
        and lock_quarter is not None
        and lock_rest[2] > 0.020
        and lock_rest[0] < 0.010
        and lock_quarter[0] > 0.020
        and lock_quarter[2] < 0.010,
        details=f"rest={lock_rest}, quarter_turn={lock_quarter}",
    )

    return ctx.report()


object_model = build_object_model()
