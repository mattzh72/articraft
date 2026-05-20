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


def _tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length / 2.0, both=True)
    )
    return mesh_from_cadquery(tube, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_training_pump")

    frame_black = model.material("frame_black", rgba=(0.10, 0.11, 0.12, 1.0))
    barrel_black = model.material("barrel_black", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.69, 0.71, 0.74, 1.0))
    grip_gray = model.material("grip_gray", rgba=(0.20, 0.22, 0.24, 1.0))
    dial_white = model.material("dial_white", rgba=(0.95, 0.95, 0.93, 1.0))
    orange = model.material("orange", rgba=(0.88, 0.31, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.20, 0.045, 0.016)),
        origin=Origin(xyz=(0.0, 0.070, 0.008)),
        material=frame_black,
        name="foot_0",
    )
    base.visual(
        Box((0.20, 0.045, 0.016)),
        origin=Origin(xyz=(0.0, -0.070, 0.008)),
        material=frame_black,
        name="foot_1",
    )
    base.visual(
        Box((0.14, 0.160, 0.022)),
        origin=Origin(xyz=(-0.030, 0.0, 0.027)),
        material=frame_black,
        name="rear_bridge",
    )
    base.visual(
        Box((0.080, 0.120, 0.028)),
        origin=Origin(xyz=(0.042, 0.0, 0.030)),
        material=frame_black,
        name="front_bridge",
    )
    base.visual(
        Box((0.032, 0.060, 0.036)),
        origin=Origin(xyz=(0.074, 0.0, 0.046)),
        material=frame_black,
        name="gauge_neck",
    )
    base.visual(
        _tube_mesh(
            "pump_barrel",
            outer_radius=0.019,
            inner_radius=0.0115,
            length=0.270,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.173)),
        material=barrel_black,
        name="barrel",
    )
    base.visual(
        _tube_mesh(
            "pump_barrel_collar",
            outer_radius=0.022,
            inner_radius=0.0075,
            length=0.018,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.299)),
        material=steel,
        name="barrel_collar",
    )
    base.visual(
        _tube_mesh(
            "pump_gauge_ring",
            outer_radius=0.037,
            inner_radius=0.028,
            length=0.018,
        ),
        origin=Origin(xyz=(0.099, 0.0, 0.048), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="gauge_ring",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.005),
        origin=Origin(xyz=(0.0965, 0.0, 0.048), rpy=(0.0, pi / 2.0, 0.0)),
        material=dial_white,
        name="dial",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0065, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, -0.075)),
        material=steel,
        name="rod",
    )
    handle.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="guide_cap",
    )
    handle.visual(
        Box((0.030, 0.040, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=frame_black,
        name="yoke",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.260),
        origin=Origin(xyz=(0.0, 0.0, 0.125), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.026, 0.125), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="stop_0",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.114, 0.125), rpy=(pi / 2.0, 0.0, 0.0)),
        material=frame_black,
        name="stop_1",
    )

    grip = model.part("grip")
    grip.visual(
        _tube_mesh(
            "pump_grip_sleeve",
            outer_radius=0.017,
            inner_radius=0.0086,
            length=0.080,
        ),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_gray,
        name="sleeve",
    )
    grip.visual(
        Box((0.022, 0.074, 0.022)),
        origin=Origin(xyz=(0.014, 0.0, -0.022)),
        material=grip_gray,
        name="pad",
    )

    needle = model.part("needle")
    needle.visual(
        Box((0.002, 0.003, 0.022)),
        origin=Origin(xyz=(0.004, 0.0, 0.011)),
        material=orange,
        name="pointer",
    )
    needle.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.002, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=orange,
        name="hub",
    )

    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=0.50, lower=0.0, upper=0.160),
    )
    model.articulation(
        "grip_roll",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.0, 0.070, 0.125)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "gauge_sweep",
        ArticulationType.REVOLUTE,
        parent=base,
        child=needle,
        origin=Origin(xyz=(0.099, 0.0, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    needle = object_model.get_part("needle")

    handle_slide = object_model.get_articulation("handle_slide")
    grip_roll = object_model.get_articulation("grip_roll")
    gauge_sweep = object_model.get_articulation("gauge_sweep")

    handle_limits = handle_slide.motion_limits
    needle_limits = gauge_sweep.motion_limits

    if handle_limits is not None and handle_limits.upper is not None:
        ctx.expect_within(
            handle,
            base,
            axes="xy",
            inner_elem="rod",
            outer_elem="barrel",
            margin=0.0,
            name="rod stays centered in the barrel at rest",
        )
        ctx.expect_overlap(
            handle,
            base,
            axes="z",
            elem_a="rod",
            elem_b="barrel",
            min_overlap=0.20,
            name="collapsed rod remains deeply inserted in the barrel",
        )

        rest_handle_pos = ctx.part_world_position(handle)
        with ctx.pose({handle_slide: handle_limits.upper}):
            ctx.expect_within(
                handle,
                base,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel",
                margin=0.0,
                name="rod stays centered in the barrel at full stroke",
            )
            ctx.expect_overlap(
                handle,
                base,
                axes="z",
                elem_a="rod",
                elem_b="barrel",
                min_overlap=0.08,
                name="extended rod keeps retained insertion in the barrel",
            )
            extended_handle_pos = ctx.part_world_position(handle)

        ctx.check(
            "handle rises on a positive pump stroke",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > rest_handle_pos[2] + 0.14,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    if needle_limits is not None and needle_limits.lower is not None and needle_limits.upper is not None:
        with ctx.pose({gauge_sweep: needle_limits.lower}):
            ctx.expect_within(
                needle,
                base,
                axes="yz",
                inner_elem="pointer",
                outer_elem="dial",
                margin=0.001,
                name="needle stays inside the dial at low pressure",
            )
            low_pointer_center = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))

        with ctx.pose({gauge_sweep: needle_limits.upper}):
            ctx.expect_within(
                needle,
                base,
                axes="yz",
                inner_elem="pointer",
                outer_elem="dial",
                margin=0.001,
                name="needle stays inside the dial at high pressure",
            )
            high_pointer_center = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))

        ctx.check(
            "gauge needle sweeps across the dial",
            low_pointer_center is not None
            and high_pointer_center is not None
            and abs(high_pointer_center[1] - low_pointer_center[1]) > 0.015,
            details=f"low={low_pointer_center}, high={high_pointer_center}",
        )

    rest_pad_center = _aabb_center(ctx.part_element_world_aabb(grip, elem="pad"))
    with ctx.pose({grip_roll: 1.15}):
        rolled_pad_center = _aabb_center(ctx.part_element_world_aabb(grip, elem="pad"))

    ctx.check(
        "grip rotates about the crossbar axis",
        rest_pad_center is not None
        and rolled_pad_center is not None
        and abs(rolled_pad_center[0] - rest_pad_center[0]) > 0.020
        and abs(rolled_pad_center[2] - rest_pad_center[2]) < 0.010,
        details=f"rest={rest_pad_center}, rolled={rolled_pad_center}",
    )

    return ctx.report()


object_model = build_object_model()
