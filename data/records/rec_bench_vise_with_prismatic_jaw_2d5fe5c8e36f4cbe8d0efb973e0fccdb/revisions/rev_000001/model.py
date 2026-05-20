from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _build_base_shell():
    plate = cq.Workplane("XY").box(0.24, 0.17, 0.018).translate((0.0, 0.0, 0.009))
    lower_ring = cq.Workplane("XY").cylinder(0.018, 0.095).translate((0.0, 0.0, 0.027))
    front_boss = cq.Workplane("XY").box(0.055, 0.075, 0.026).translate((-0.110, 0.0, 0.031))
    return plate.union(lower_ring).union(front_boss)


def _build_front_jaw_shell():
    jaw_block = cq.Workplane("XY").box(0.038, 0.154, 0.100).translate((-0.019, 0.0, 0.0))
    carriage = cq.Workplane("XY").box(0.118, 0.098, 0.070).translate((0.059, 0.0, -0.021))
    shell = jaw_block.union(carriage)
    shell = shell.cut(
        cq.Workplane("XY").box(0.124, 0.072, 0.046).translate((0.058, 0.0, -0.023))
    )
    shell = shell.union(
        cq.Workplane("YZ").cylinder(0.042, 0.020).translate((-0.024, 0.0, -0.037))
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_base_bench_vise")

    cast_iron = model.material("cast_iron", rgba=(0.21, 0.24, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.15, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.63, 0.66, 0.69, 1.0))
    jaw_plate = model.material("jaw_plate", rgba=(0.15, 0.16, 0.17, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_build_base_shell(), "vise_base_shell"),
        material=cast_iron,
        name="base_shell",
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.090, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron,
        name="swivel_ring",
    )
    body.visual(
        Box((0.165, 0.110, 0.030)),
        origin=Origin(xyz=(0.118, 0.0, 0.113)),
        material=cast_iron,
        name="body_shell",
    )
    body.visual(
        Box((0.070, 0.110, 0.050)),
        origin=Origin(xyz=(0.165, 0.0, 0.073)),
        material=cast_iron,
        name="rear_block",
    )
    body.visual(
        Box((0.050, 0.090, 0.018)),
        origin=Origin(xyz=(0.176, 0.0, 0.137)),
        material=cast_iron,
        name="anvil",
    )
    body.visual(
        Box((0.110, 0.010, 0.080)),
        origin=Origin(xyz=(0.075, -0.057, 0.058)),
        material=cast_iron,
        name="side_rib_0",
    )
    body.visual(
        Box((0.110, 0.010, 0.080)),
        origin=Origin(xyz=(0.075, 0.057, 0.058)),
        material=cast_iron,
        name="side_rib_1",
    )
    body.visual(
        Box((0.160, 0.064, 0.040)),
        origin=Origin(xyz=(0.082, 0.0, 0.056)),
        material=machined_steel,
        name="guide_bar",
    )
    body.visual(
        Box((0.032, 0.154, 0.100)),
        origin=Origin(xyz=(0.016, 0.0, 0.079)),
        material=cast_iron,
        name="fixed_jaw",
    )
    body.visual(
        Box((0.004, 0.148, 0.078)),
        origin=Origin(xyz=(0.002, 0.0, 0.079)),
        material=jaw_plate,
        name="fixed_plate",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        mesh_from_cadquery(_build_front_jaw_shell(), "vise_front_jaw_shell"),
        material=cast_iron,
        name="jaw_shell",
    )
    front_jaw.visual(
        Box((0.004, 0.148, 0.078)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=jaw_plate,
        name="moving_plate",
    )

    screw_handle = model.part("screw_handle")
    screw_handle.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub",
    )
    screw_handle.visual(
        Cylinder(radius=0.0055, length=0.180),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="handle_bar",
    )
    screw_handle.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.020, -0.090, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="handle_end_0",
    )
    screw_handle.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(-0.020, 0.090, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="handle_end_1",
    )

    lock_handle = model.part("lock_handle")
    lock_handle.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(-0.011, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="hub",
    )
    lock_handle.visual(
        Cylinder(radius=0.0048, length=0.122),
        origin=Origin(xyz=(-0.016, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=machined_steel,
        name="handle_bar",
    )
    lock_handle.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.016, -0.061, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="handle_end_0",
    )
    lock_handle.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(-0.016, 0.061, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined_steel,
        name="handle_end_1",
    )

    base_swivel = model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.8),
    )
    jaw_slide = model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_jaw,
        origin=Origin(xyz=(0.0, 0.0, 0.079)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.12, lower=0.0, upper=0.090),
    )
    screw_spin = model.articulation(
        "screw_spin",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=screw_handle,
        origin=Origin(xyz=(-0.044, 0.0, -0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=6.0),
    )
    lock_spin = model.articulation(
        "lock_spin",
        ArticulationType.CONTINUOUS,
        parent=base_plate,
        child=lock_handle,
        origin=Origin(xyz=(-0.1375, 0.0, 0.031)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=5.0),
    )

    base_swivel.meta["qc_samples"] = [0.0, math.pi / 4.0]
    jaw_slide.meta["qc_samples"] = [0.0, 0.045, 0.090]
    screw_spin.meta["qc_samples"] = [0.0, math.pi / 2.0]
    lock_spin.meta["qc_samples"] = [0.0, math.pi / 2.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    front_jaw = object_model.get_part("front_jaw")
    screw_handle = object_model.get_part("screw_handle")
    lock_handle = object_model.get_part("lock_handle")

    base_swivel = object_model.get_articulation("base_swivel")
    jaw_slide = object_model.get_articulation("jaw_slide")
    screw_spin = object_model.get_articulation("screw_spin")
    lock_spin = object_model.get_articulation("lock_spin")

    ctx.allow_overlap(
        body,
        front_jaw,
        elem_a="fixed_jaw",
        elem_b="jaw_shell",
        reason=(
            "The moving jaw shell is simplified as a captured saddle around the fixed "
            "jaw nose, while the actual clamping faces remain separated at the jaw plates."
        ),
    )

    with ctx.pose({jaw_slide: 0.0}):
        ctx.expect_gap(
            body,
            front_jaw,
            axis="x",
            positive_elem="fixed_plate",
            negative_elem="moving_plate",
            max_gap=0.001,
            max_penetration=0.0,
            name="jaws close face to face",
        )
        ctx.expect_overlap(
            front_jaw,
            body,
            axes="x",
            elem_a="jaw_shell",
            elem_b="guide_bar",
            min_overlap=0.115,
            name="closed jaw stays deeply engaged on the guide",
        )

    rest_pos = None
    swivel_pos = None
    with ctx.pose({jaw_slide: 0.060}):
        rest_pos = ctx.part_world_position(front_jaw)
        ctx.expect_gap(
            body,
            front_jaw,
            axis="x",
            positive_elem="fixed_plate",
            negative_elem="moving_plate",
            min_gap=0.059,
            max_gap=0.061,
            name="front jaw opens to a clear work gap",
        )

    with ctx.pose({jaw_slide: 0.090}):
        ctx.expect_overlap(
            front_jaw,
            body,
            axes="x",
            elem_a="jaw_shell",
            elem_b="guide_bar",
            min_overlap=0.025,
            name="front jaw retains insertion at full opening",
        )

    with ctx.pose({jaw_slide: 0.060, base_swivel: math.pi / 4.0}):
        swivel_pos = ctx.part_world_position(front_jaw)

    ctx.check(
        "swivel base turns the vise body around vertical axis",
        rest_pos is not None
        and swivel_pos is not None
        and abs(swivel_pos[1]) > 0.035
        and abs(swivel_pos[0] - rest_pos[0]) > 0.010,
        details=f"rest={rest_pos}, swivel={swivel_pos}",
    )

    screw_rest = None
    screw_rot = None
    with ctx.pose({screw_spin: 0.0}):
        screw_rest = ctx.part_element_world_aabb(screw_handle, elem="handle_bar")
    with ctx.pose({screw_spin: math.pi / 2.0}):
        screw_rot = ctx.part_element_world_aabb(screw_handle, elem="handle_bar")

    def _span(aabb, axis: int) -> float:
        if aabb is None:
            return -1.0
        return aabb[1][axis] - aabb[0][axis]

    ctx.check(
        "screw handle rotates on the lead screw axis",
        screw_rest is not None
        and screw_rot is not None
        and _span(screw_rest, 1) > 0.16
        and _span(screw_rest, 2) < 0.02
        and _span(screw_rot, 2) > 0.16
        and _span(screw_rot, 1) < 0.02,
        details=f"rest={screw_rest}, rotated={screw_rot}",
    )

    lock_rest = None
    lock_rot = None
    with ctx.pose({lock_spin: 0.0}):
        lock_rest = ctx.part_element_world_aabb(lock_handle, elem="handle_bar")
    with ctx.pose({lock_spin: math.pi / 2.0}):
        lock_rot = ctx.part_element_world_aabb(lock_handle, elem="handle_bar")

    ctx.check(
        "base locking handle spins about its clamp screw axis",
        lock_rest is not None
        and lock_rot is not None
        and _span(lock_rest, 1) > 0.10
        and _span(lock_rot, 2) > 0.10,
        details=f"rest={lock_rest}, rotated={lock_rot}",
    )

    return ctx.report()


object_model = build_object_model()
