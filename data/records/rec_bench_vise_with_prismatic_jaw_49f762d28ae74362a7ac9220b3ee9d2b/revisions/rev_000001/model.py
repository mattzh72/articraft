from __future__ import annotations

from math import pi

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


JAW_CENTER_Z = 0.155
SCREW_AXIS_Z = 0.102
PIPE_HINGE_X = -0.035
PIPE_HINGE_Z = 0.048
JAW_WIDTH = 0.126
JAW_HEIGHT = 0.028
SLIDE_TRAVEL = 0.140


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _x_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0] - (length / 2.0), center[1], center[2]))
    )


def _y_cylinder(
    radius: float,
    length: float,
    center: tuple[float, float, float],
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length)
        .translate((center[0], center[1] - (length / 2.0), center[2]))
    )


def _make_body_shape() -> cq.Workplane:
    base = _box((0.240, 0.180, 0.018), (-0.030, 0.0, 0.009)).edges("|Z").fillet(0.020)
    pedestal = _box((0.100, 0.110, 0.055), (-0.075, 0.0, 0.0455))
    rear_web = _box((0.085, 0.095, 0.045), (-0.100, 0.0, 0.094))
    fixed_body = _box((0.170, 0.140, 0.080), (-0.085, 0.0, 0.140))
    front_bridge = _box((0.060, 0.082, 0.044), (-0.040, 0.0, 0.100))
    rail_0 = _box((0.180, 0.034, 0.030), (0.090, 0.053, 0.093))
    rail_1 = _box((0.180, 0.034, 0.030), (0.090, -0.053, 0.093))
    hinge_mount = _box((0.030, 0.080, 0.018), (-0.075, 0.0, 0.071))
    hinge_ear_0 = _box((0.018, 0.018, 0.028), (PIPE_HINGE_X, 0.055, 0.050))
    hinge_ear_1 = _box((0.018, 0.018, 0.028), (PIPE_HINGE_X, -0.055, 0.050))

    shell = (
        base.union(pedestal)
        .union(rear_web)
        .union(fixed_body)
        .union(front_bridge)
        .union(rail_0)
        .union(rail_1)
        .union(hinge_mount)
        .union(hinge_ear_0)
        .union(hinge_ear_1)
    )

    slide_slot = _box((0.220, 0.072, 0.032), (0.090, 0.0, 0.093))
    screw_bore = _x_cylinder(0.0145, 0.280, (-0.040, 0.0, SCREW_AXIS_Z))
    pipe_clearance = _box((0.090, 0.078, 0.050), (0.015, 0.0, 0.045))
    return shell.cut(slide_slot).cut(screw_bore).cut(pipe_clearance)


def _make_front_jaw_shape() -> cq.Workplane:
    jaw_block = _box((0.085, 0.146, 0.080), (0.0505, 0.0, 0.0))
    guide_shoe = _box((0.180, 0.060, 0.024), (0.070, 0.0, -0.066))
    lower_pipe_block = _box((0.090, 0.070, 0.040), (0.055, 0.0, -0.088))
    brace_0 = _box((0.060, 0.014, 0.036), (0.050, 0.022, -0.042))
    brace_1 = _box((0.060, 0.014, 0.036), (0.050, -0.022, -0.042))
    collar = _x_cylinder(0.024, 0.032, (0.082, 0.0, -0.053))

    shell = (
        jaw_block.union(guide_shoe)
        .union(lower_pipe_block)
        .union(brace_0)
        .union(brace_1)
        .union(collar)
    )

    screw_bore = _x_cylinder(0.0155, 0.240, (0.050, 0.0, -0.053))
    pipe_groove = _y_cylinder(0.022, 0.118, (0.038, 0.0, -0.110))
    return shell.cut(screw_bore).cut(pipe_groove)


def _make_pipe_jaw_shape() -> cq.Workplane:
    hinge_barrel = _y_cylinder(0.012, 0.088, (0.000, 0.0, 0.0))
    jaw_core = _box((0.050, 0.060, 0.018), (0.028, 0.0, -0.010))
    jaw_nose = _box((0.022, 0.060, 0.024), (0.062, 0.0, -0.012))
    jaw_cap = _box((0.024, 0.060, 0.010), (0.038, 0.0, 0.004))
    grip_relief = _y_cylinder(0.012, 0.066, (0.040, 0.0, 0.016))
    return hinge_barrel.union(jaw_core).union(jaw_nose).union(jaw_cap).cut(grip_relief)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_vise")

    model.material("cast_blue", rgba=(0.19, 0.33, 0.56, 1.0))
    model.material("jaw_steel", rgba=(0.62, 0.66, 0.70, 1.0))
    model.material("bright_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("dark_steel", rgba=(0.32, 0.34, 0.36, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "body_shell"),
        material="cast_blue",
        name="body_shell",
    )
    body.visual(
        Box((0.008, JAW_WIDTH, JAW_HEIGHT)),
        origin=Origin(xyz=(-0.004, 0.0, JAW_CENTER_Z)),
        material="jaw_steel",
        name="fixed_jaw_face",
    )

    front_jaw = model.part("front_jaw")
    front_jaw.visual(
        Box((0.085, 0.146, 0.080)),
        origin=Origin(xyz=(0.0505, 0.0, 0.0)),
        material="cast_blue",
        name="jaw_shell",
    )
    front_jaw.visual(
        Box((0.090, 0.070, 0.040)),
        origin=Origin(xyz=(0.053, 0.0, -0.088)),
        material="cast_blue",
        name="jaw_lower",
    )
    front_jaw.visual(
        Box((0.060, 0.014, 0.050)),
        origin=Origin(xyz=(0.050, 0.022, -0.049)),
        material="cast_blue",
        name="jaw_brace_0",
    )
    front_jaw.visual(
        Box((0.060, 0.014, 0.050)),
        origin=Origin(xyz=(0.050, -0.022, -0.049)),
        material="cast_blue",
        name="jaw_brace_1",
    )
    front_jaw.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.152, 0.0, -0.053), rpy=(0.0, pi / 2.0, 0.0)),
        material="cast_blue",
        name="jaw_nose",
    )
    front_jaw.visual(
        Box((0.008, JAW_WIDTH, JAW_HEIGHT)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material="jaw_steel",
        name="moving_jaw_face",
    )
    front_jaw.visual(
        Box((0.174, 0.056, 0.012)),
        origin=Origin(xyz=(0.070, 0.0, -0.072)),
        material="cast_blue",
        name="guide_shoe",
    )
    front_jaw.visual(
        Box((0.058, 0.060, 0.010)),
        origin=Origin(xyz=(0.046, 0.0, -0.080)),
        material="jaw_steel",
        name="pipe_saddle",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.018, length=0.040),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="bright_steel",
        name="handle_hub",
    )
    handle.visual(
        Cylinder(radius=0.011, length=0.190),
        origin=Origin(xyz=(-0.090, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="dark_steel",
        name="lead_screw",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.220),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="handle_bar",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.006, 0.112, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="bar_end_0",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.024),
        origin=Origin(xyz=(0.006, -0.112, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material="bright_steel",
        name="bar_end_1",
    )

    pipe_jaw = model.part("pipe_jaw")
    pipe_jaw.visual(
        Box((0.070, 0.060, 0.018)),
        origin=Origin(xyz=(0.040, 0.0, -0.014)),
        material="dark_steel",
        name="pipe_shell",
    )
    pipe_jaw.visual(
        Box((0.032, 0.060, 0.012)),
        origin=Origin(xyz=(0.062, 0.0, -0.002)),
        material="dark_steel",
        name="pipe_cap",
    )
    pipe_jaw.visual(
        Cylinder(radius=0.012, length=0.088),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="pipe_barrel",
    )
    pipe_jaw.visual(
        Box((0.032, 0.096, 0.006)),
        origin=Origin(xyz=(0.059, 0.0, -0.002)),
        material="jaw_steel",
        name="pipe_face",
    )

    model.articulation(
        "jaw_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=front_jaw,
        origin=Origin(xyz=(0.0, 0.0, JAW_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=SLIDE_TRAVEL, effort=1800.0, velocity=0.22),
    )
    model.articulation(
        "handle_spin",
        ArticulationType.CONTINUOUS,
        parent=front_jaw,
        child=handle,
        origin=Origin(xyz=(0.185, 0.0, SCREW_AXIS_Z - JAW_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )
    model.articulation(
        "pipe_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pipe_jaw,
        origin=Origin(xyz=(PIPE_HINGE_X, 0.0, PIPE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=120.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    front_jaw = object_model.get_part("front_jaw")
    handle = object_model.get_part("handle")
    pipe_jaw = object_model.get_part("pipe_jaw")

    jaw_slide = object_model.get_articulation("jaw_slide")
    handle_spin = object_model.get_articulation("handle_spin")
    pipe_hinge = object_model.get_articulation("pipe_hinge")

    slide_upper = jaw_slide.motion_limits.upper or 0.0
    pipe_upper = pipe_hinge.motion_limits.upper or 0.0

    ctx.allow_overlap(
        front_jaw,
        handle,
        elem_a="jaw_nose",
        elem_b="lead_screw",
        reason="The lead screw is intentionally represented as running inside the movable jaw nose bearing housing.",
    )
    ctx.allow_overlap(
        body,
        pipe_jaw,
        elem_a="body_shell",
        elem_b="pipe_barrel",
        reason="The lower pipe jaw hinge barrel is intentionally represented inside the simplified body clevis without explicit ear bores.",
    )
    ctx.allow_overlap(
        body,
        pipe_jaw,
        elem_a="body_shell",
        elem_b="pipe_shell",
        reason="The closed pipe jaw is represented as nesting into a simplified solid underbody pocket rather than a fully hollow cast pipe-jaw cavity.",
    )

    ctx.expect_gap(
        front_jaw,
        body,
        axis="x",
        positive_elem="moving_jaw_face",
        negative_elem="fixed_jaw_face",
        min_gap=0.0,
        max_gap=0.002,
        name="flat jaws nearly close at rest",
    )
    ctx.expect_overlap(
        front_jaw,
        body,
        axes="yz",
        elem_a="moving_jaw_face",
        elem_b="fixed_jaw_face",
        min_overlap=0.024,
        name="flat jaws align across width and height",
    )
    ctx.expect_within(
        front_jaw,
        body,
        axes="yz",
        inner_elem="guide_shoe",
        outer_elem="body_shell",
        margin=0.004,
        name="guide shoe sits inside the body envelope",
    )
    ctx.expect_gap(
        front_jaw,
        pipe_jaw,
        axis="z",
        positive_elem="pipe_saddle",
        negative_elem="pipe_face",
        min_gap=0.002,
        max_gap=0.025,
        name="pipe jaw tucks just below the lower saddle",
    )

    rest_front_pos = ctx.part_world_position(front_jaw)
    with ctx.pose({jaw_slide: slide_upper}):
        ctx.expect_gap(
            front_jaw,
            body,
            axis="x",
            positive_elem="moving_jaw_face",
            negative_elem="fixed_jaw_face",
            min_gap=0.110,
            name="main jaws open for pipe and stock work",
        )
        ctx.expect_overlap(
            front_jaw,
            body,
            axes="x",
            elem_a="guide_shoe",
            elem_b="body_shell",
            min_overlap=0.050,
            name="guide shoe retains insertion at full opening",
        )
        ctx.expect_within(
            front_jaw,
            body,
            axes="yz",
            inner_elem="guide_shoe",
            outer_elem="body_shell",
            margin=0.004,
            name="guide shoe stays captured by the ways at full opening",
        )
        open_front_pos = ctx.part_world_position(front_jaw)

    ctx.check(
        "front jaw slides outward",
        rest_front_pos is not None
        and open_front_pos is not None
        and open_front_pos[0] > rest_front_pos[0] + 0.10,
        details=f"rest={rest_front_pos}, open={open_front_pos}",
    )

    rest_handle_pos = ctx.part_world_position(handle)
    with ctx.pose({handle_spin: pi / 2.0}):
        spun_handle_pos = ctx.part_world_position(handle)
    ctx.check(
        "handle rotates about the lead screw axis",
        rest_handle_pos is not None
        and spun_handle_pos is not None
        and max(abs(a - b) for a, b in zip(rest_handle_pos, spun_handle_pos)) < 1e-6,
        details=f"rest={rest_handle_pos}, spun={spun_handle_pos}",
    )
    ctx.check(
        "handle articulation is continuous",
        handle_spin.articulation_type == ArticulationType.CONTINUOUS
        and handle_spin.motion_limits is not None
        and handle_spin.motion_limits.lower is None
        and handle_spin.motion_limits.upper is None,
        details=f"type={handle_spin.articulation_type}, limits={handle_spin.motion_limits}",
    )

    rest_pipe_aabb = ctx.part_world_aabb(pipe_jaw)
    with ctx.pose({pipe_hinge: pipe_upper}):
        open_pipe_aabb = ctx.part_world_aabb(pipe_jaw)
    ctx.check(
        "pipe jaw swings downward on its hinge",
        rest_pipe_aabb is not None
        and open_pipe_aabb is not None
        and open_pipe_aabb[0][2] < rest_pipe_aabb[0][2] - 0.030,
        details=f"rest={rest_pipe_aabb}, open={open_pipe_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
