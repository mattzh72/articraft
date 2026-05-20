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


def build_barrel_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(0.030)
        .circle(0.023)
        .extrude(0.422)
        .translate((0.0, 0.0, 0.014))
    )


def build_gauge_bezel() -> cq.Workplane:
    outer = (
        cq.Workplane("XZ")
        .center(0.0, 0.090)
        .circle(0.037)
        .extrude(0.022)
        .translate((0.0, 0.052, 0.0))
    )
    cavity = (
        cq.Workplane("XZ")
        .center(0.0, 0.090)
        .circle(0.031)
        .extrude(0.016)
        .translate((0.0, 0.058, 0.0))
    )
    return outer.cut(cavity)


def build_grip_sleeve() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(0.019)
        .circle(0.012)
        .extrude(0.072)
        .translate((-0.036, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_pump")

    model.material("frame_black", rgba=(0.13, 0.13, 0.14, 1.0))
    model.material("barrel_red", rgba=(0.73, 0.10, 0.10, 1.0))
    model.material("steel", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("rubber", rgba=(0.08, 0.08, 0.08, 1.0))
    model.material("gauge_white", rgba=(0.94, 0.94, 0.92, 1.0))
    model.material("needle_yellow", rgba=(0.94, 0.72, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.300, 0.120, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="frame_black",
        name="base",
    )
    frame.visual(
        Box((0.070, 0.040, 0.064)),
        origin=Origin(xyz=(0.0, -0.020, 0.048)),
        material="frame_black",
        name="lower_brace",
    )
    frame.visual(
        Box((0.018, 0.024, 0.438)),
        origin=Origin(xyz=(-0.050, -0.022, 0.235)),
        material="frame_black",
        name="support_0",
    )
    frame.visual(
        Box((0.018, 0.024, 0.438)),
        origin=Origin(xyz=(0.050, -0.022, 0.235)),
        material="frame_black",
        name="support_1",
    )
    frame.visual(
        Box((0.160, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, -0.022, 0.455)),
        material="frame_black",
        name="top_bridge",
    )
    frame.visual(
        mesh_from_cadquery(build_barrel_shell(), "barrel_shell"),
        material="barrel_red",
        name="barrel_shell",
    )
    frame.visual(
        Box((0.018, 0.054, 0.016)),
        origin=Origin(xyz=(0.0, 0.027, 0.090)),
        material="frame_black",
        name="gauge_stem",
    )
    frame.visual(
        Box((0.006, 0.020, 0.024)),
        origin=Origin(xyz=(-0.0115, 0.0, 0.448)),
        material="frame_black",
        name="guide_0",
    )
    frame.visual(
        Box((0.006, 0.020, 0.024)),
        origin=Origin(xyz=(0.0115, 0.0, 0.448)),
        material="frame_black",
        name="guide_1",
    )
    frame.visual(
        Box((0.020, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, -0.0115, 0.448)),
        material="frame_black",
        name="guide_2",
    )
    frame.visual(
        Box((0.020, 0.006, 0.024)),
        origin=Origin(xyz=(0.0, 0.0115, 0.448)),
        material="frame_black",
        name="guide_3",
    )
    frame.visual(
        mesh_from_cadquery(build_gauge_bezel(), "gauge_bezel"),
        material="gauge_white",
        name="gauge_bezel",
    )
    frame.visual(
        Cylinder(radius=0.0025, length=0.010),
        origin=Origin(xyz=(0.0, 0.058, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="gauge_axle",
    )
    frame.visual(
        Box((0.010, 0.036, 0.010)),
        origin=Origin(xyz=(0.061, 0.000, 0.340)),
        material="frame_black",
        name="hook_arm",
    )
    frame.visual(
        Box((0.010, 0.010, 0.020)),
        origin=Origin(xyz=(0.061, 0.018, 0.349)),
        material="frame_black",
        name="hook_tip",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0085, length=0.500),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material="steel",
        name="rod",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber",
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.167)),
        material="frame_black",
        name="handle_boss",
    )

    grip = model.part("grip")
    grip.visual(
        mesh_from_cadquery(build_grip_sleeve(), "grip_sleeve"),
        material="steel",
        name="sleeve",
    )
    grip.visual(
        Box((0.014, 0.022, 0.044)),
        origin=Origin(xyz=(-0.024, 0.014, -0.038)),
        material="frame_black",
        name="link_0",
    )
    grip.visual(
        Box((0.014, 0.022, 0.044)),
        origin=Origin(xyz=(0.024, 0.014, -0.038)),
        material="frame_black",
        name="link_1",
    )
    grip.visual(
        Cylinder(radius=0.013, length=0.110),
        origin=Origin(xyz=(0.0, 0.032, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="rubber",
        name="grip_bar",
    )

    needle = model.part("needle")
    needle.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="needle_yellow",
        name="hub",
    )
    needle.visual(
        Box((0.003, 0.002, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material="needle_yellow",
        name="pointer",
    )
    needle.visual(
        Box((0.006, 0.002, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material="needle_yellow",
        name="counterweight",
    )

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.436)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.180, effort=40.0, velocity=0.6),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.045, 0.0, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0),
    )
    model.articulation(
        "frame_to_needle",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=needle,
        origin=Origin(xyz=(0.0, 0.066, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=0.2, velocity=8.0),
    )

    return model


def aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")

    handle_joint = object_model.get_articulation("frame_to_handle")
    grip_joint = object_model.get_articulation("handle_to_grip")
    needle_joint = object_model.get_articulation("frame_to_needle")

    handle_limits = handle_joint.motion_limits
    needle_limits = needle_joint.motion_limits

    if handle_limits is not None and handle_limits.upper is not None:
        rest_position = ctx.part_world_position(handle)

        with ctx.pose({handle_joint: 0.0}):
            ctx.expect_within(
                handle,
                frame,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.004,
                name="rod stays centered in barrel at rest",
            )
            ctx.expect_overlap(
                handle,
                frame,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.280,
                name="rod remains deeply inserted at rest",
            )

        with ctx.pose({handle_joint: handle_limits.upper}):
            ctx.expect_within(
                handle,
                frame,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.004,
                name="rod stays centered in barrel at full stroke",
            )
            ctx.expect_overlap(
                handle,
                frame,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.100,
                name="rod remains retained in barrel at full stroke",
            )
            extended_position = ctx.part_world_position(handle)

        ctx.check(
            "handle rises along barrel axis",
            rest_position is not None
            and extended_position is not None
            and extended_position[2] > rest_position[2] + 0.15,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    ctx.expect_within(
        handle,
        grip,
        axes="yz",
        inner_elem="crossbar",
        outer_elem="sleeve",
        margin=0.004,
        name="grip sleeve stays centered around crossbar",
    )
    ctx.expect_overlap(
        handle,
        grip,
        axes="x",
        elem_a="crossbar",
        elem_b="sleeve",
        min_overlap=0.060,
        name="grip sleeve keeps axial engagement on crossbar",
    )
    ctx.allow_overlap(
        handle,
        grip,
        elem_a="crossbar",
        elem_b="sleeve",
        reason="The folding hand grip is represented as a rotating sleeve proxy wrapped around the T-handle crossbar.",
    )

    rest_grip_center = ctx.part_element_world_aabb(grip, elem="grip_bar")
    with ctx.pose({grip_joint: math.pi / 2.0}):
        folded_grip_center = ctx.part_element_world_aabb(grip, elem="grip_bar")

    rest_grip_center = aabb_center(rest_grip_center)
    folded_grip_center = aabb_center(folded_grip_center)
    ctx.check(
        "grip folds around crossbar axis",
        rest_grip_center is not None
        and folded_grip_center is not None
        and folded_grip_center[1] > rest_grip_center[1] + 0.020
        and folded_grip_center[2] > rest_grip_center[2] + 0.070,
        details=f"rest={rest_grip_center}, folded={folded_grip_center}",
    )

    ctx.expect_within(
        needle,
        frame,
        axes="xz",
        inner_elem="pointer",
        outer_elem="gauge_bezel",
        margin=0.004,
        name="needle stays within gauge opening",
    )

    if needle_limits is not None and needle_limits.lower is not None and needle_limits.upper is not None:
        with ctx.pose({needle_joint: needle_limits.lower}):
            low_center = aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))
        with ctx.pose({needle_joint: needle_limits.upper}):
            high_center = aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))

        ctx.check(
            "needle sweeps across gauge face",
            low_center is not None
            and high_center is not None
            and low_center[0] < -0.008
            and high_center[0] > 0.008,
            details=f"low={low_center}, high={high_center}",
        )

    return ctx.report()


object_model = build_object_model()
