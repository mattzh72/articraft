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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _barrel_shape():
    main_shell = cq.Workplane("XY").circle(0.022).circle(0.018).extrude(0.410)
    lower_collar = cq.Workplane("XY").circle(0.025).circle(0.018).extrude(0.030)
    upper_guide = (
        cq.Workplane("XY")
        .circle(0.027)
        .circle(0.009)
        .extrude(0.028)
        .translate((0.0, 0.0, 0.410))
    )
    return main_shell.union(lower_collar).union(upper_guide)


def _button_guide_shape():
    return cq.Workplane("XZ").circle(0.0048).circle(0.0033).extrude(0.014)


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return (
        (lo[0] + hi[0]) * 0.5,
        (lo[1] + hi[1]) * 0.5,
        (lo[2] + hi[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="workshop_floor_pump")

    steel_dark = model.material("steel_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    barrel_silver = model.material("barrel_silver", rgba=(0.72, 0.75, 0.78, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.09, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    dial_white = model.material("dial_white", rgba=(0.95, 0.96, 0.94, 1.0))
    glass_clear = model.material("glass_clear", rgba=(0.76, 0.83, 0.88, 0.30))
    safety_red = model.material("safety_red", rgba=(0.80, 0.16, 0.13, 1.0))
    needle_orange = model.material("needle_orange", rgba=(0.93, 0.42, 0.08, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.095, 0.280, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=steel_dark,
        name="base_plate",
    )
    frame.visual(
        Box((0.068, 0.092, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=steel_dark,
        name="base_pedestal",
    )
    frame.visual(
        Box((0.046, 0.084, 0.004)),
        origin=Origin(xyz=(0.0, -0.074, 0.018)),
        material=rubber_dark,
        name="tread_0",
    )
    frame.visual(
        Box((0.046, 0.084, 0.004)),
        origin=Origin(xyz=(0.0, 0.074, 0.018)),
        material=rubber_dark,
        name="tread_1",
    )
    frame.visual(
        mesh_from_cadquery(_barrel_shape(), "pump_barrel"),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=barrel_silver,
        name="barrel_shell",
    )
    frame.visual(
        Box((0.018, 0.028, 0.100)),
        origin=Origin(xyz=(0.038, 0.0, 0.066)),
        material=steel_dark,
        name="gauge_stem",
    )

    rear_support_roots = [(-0.020, -0.058, 0.012), (-0.020, 0.058, 0.012)]
    rear_support_tops = [(-0.018, -0.038, 0.442), (-0.018, 0.038, 0.442)]
    for a, b in zip(rear_support_roots, rear_support_tops):
        _add_member(frame, a, b, radius=0.009, material=steel_dark)

    frame.visual(
        Box((0.030, 0.100, 0.018)),
        origin=Origin(xyz=(-0.018, 0.0, 0.445)),
        material=steel_dark,
        name="top_bridge",
    )
    _add_member(
        frame,
        (-0.018, -0.030, 0.445),
        (-0.008, -0.020, 0.430),
        radius=0.0065,
        material=steel_dark,
    )
    _add_member(
        frame,
        (-0.018, 0.030, 0.445),
        (-0.008, 0.020, 0.430),
        radius=0.0065,
        material=steel_dark,
    )

    frame.visual(
        Box((0.010, 0.024, 0.010)),
        origin=Origin(xyz=(-0.028, 0.060, 0.305)),
        material=steel_dark,
        name="hose_hook_stem",
    )
    frame.visual(
        Box((0.010, 0.010, 0.032)),
        origin=Origin(xyz=(-0.028, 0.072, 0.321)),
        material=steel_dark,
        name="hose_hook_post",
    )
    frame.visual(
        Box((0.010, 0.016, 0.010)),
        origin=Origin(xyz=(-0.028, 0.064, 0.337)),
        material=steel_dark,
        name="hose_hook_tip",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0065, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=barrel_silver,
        name="piston_rod",
    )
    handle.visual(
        Box((0.028, 0.022, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.250)),
        material=steel_dark,
        name="handle_block",
    )
    handle.visual(
        Box((0.036, 0.220, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.278)),
        material=steel_dark,
        name="handle_bar",
    )
    for index, grip_y in enumerate((-0.078, 0.078)):
        handle.visual(
            Cylinder(radius=0.014, length=0.084),
            origin=Origin(xyz=(0.0, grip_y, 0.278), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=grip_black,
            name=f"grip_{index}",
        )

    model.articulation(
        "frame_to_handle",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.468)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.110,
        ),
    )

    gauge = model.part("gauge")
    gauge.visual(
        Cylinder(radius=0.035, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=steel_dark,
        name="gauge_body",
    )
    gauge.visual(
        Cylinder(radius=0.041, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel_dark,
        name="bezel",
    )
    gauge.visual(
        Cylinder(radius=0.036, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=dial_white,
        name="dial_face",
    )
    gauge.visual(
        Cylinder(radius=0.034, length=0.0015),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=glass_clear,
        name="lens",
    )
    gauge.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=steel_dark,
        name="mount_spigot",
    )
    gauge.visual(
        mesh_from_cadquery(_button_guide_shape(), "gauge_button_guide"),
        origin=Origin(xyz=(0.0, 0.052, -0.004)),
        material=steel_dark,
        name="button_guide",
    )

    model.articulation(
        "frame_to_gauge",
        ArticulationType.FIXED,
        parent=frame,
        child=gauge,
        origin=Origin(xyz=(0.095, 0.0, 0.105), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    needle = model.part("needle")
    needle.visual(
        Cylinder(radius=0.0042, length=0.0024),
        origin=Origin(xyz=(0.0, 0.0, 0.0012)),
        material=steel_dark,
        name="hub",
    )
    needle.visual(
        Box((0.003, 0.026, 0.0016)),
        origin=Origin(xyz=(0.0, 0.013, 0.0016)),
        material=needle_orange,
        name="pointer",
    )
    needle.visual(
        Box((0.0024, 0.010, 0.0014)),
        origin=Origin(xyz=(0.0, -0.006, 0.0015)),
        material=needle_orange,
        name="tail",
    )

    model.articulation(
        "gauge_to_needle",
        ArticulationType.REVOLUTE,
        parent=gauge,
        child=needle,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.1,
            velocity=2.5,
            lower=-2.2,
            upper=0.8,
        ),
    )

    button = model.part("button")
    button.visual(
        Cylinder(radius=0.0036, length=0.004),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_red,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.0030, length=0.010),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_red,
        name="button_stem",
    )

    model.articulation(
        "gauge_to_button",
        ArticulationType.PRISMATIC,
        parent=gauge,
        child=button,
        origin=Origin(xyz=(0.0, 0.052, -0.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    handle = object_model.get_part("handle")
    gauge = object_model.get_part("gauge")
    needle = object_model.get_part("needle")
    button = object_model.get_part("button")

    handle_joint = object_model.get_articulation("frame_to_handle")
    needle_joint = object_model.get_articulation("gauge_to_needle")
    button_joint = object_model.get_articulation("gauge_to_button")

    ctx.expect_contact(
        gauge,
        frame,
        elem_a="mount_spigot",
        elem_b="gauge_stem",
        name="gauge is supported by the frame stem",
    )

    handle_limits = handle_joint.motion_limits
    if handle_limits is not None and handle_limits.lower is not None and handle_limits.upper is not None:
        with ctx.pose({handle_joint: handle_limits.lower}):
            ctx.expect_within(
                handle,
                frame,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rest rod stays centered in the barrel",
            )
            ctx.expect_overlap(
                handle,
                frame,
                axes="z",
                elem_a="piston_rod",
                elem_b="barrel_shell",
                min_overlap=0.17,
                name="rest rod remains inserted in the barrel",
            )
            rest_handle_pos = ctx.part_world_position(handle)

        with ctx.pose({handle_joint: handle_limits.upper}):
            ctx.expect_within(
                handle,
                frame,
                axes="xy",
                inner_elem="piston_rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="extended rod stays centered in the barrel",
            )
            ctx.expect_overlap(
                handle,
                frame,
                axes="z",
                elem_a="piston_rod",
                elem_b="barrel_shell",
                min_overlap=0.06,
                name="extended rod still retains insertion",
            )
            extended_handle_pos = ctx.part_world_position(handle)

        ctx.check(
            "handle slides upward on the barrel axis",
            rest_handle_pos is not None
            and extended_handle_pos is not None
            and extended_handle_pos[2] > rest_handle_pos[2] + 0.08,
            details=f"rest={rest_handle_pos}, extended={extended_handle_pos}",
        )

    button_limits = button_joint.motion_limits
    if button_limits is not None and button_limits.lower is not None and button_limits.upper is not None:
        with ctx.pose({button_joint: button_limits.lower}):
            ctx.expect_within(
                button,
                gauge,
                axes="xz",
                inner_elem="button_stem",
                outer_elem="button_guide",
                margin=0.0,
                name="button stem stays aligned with guide at rest",
            )
            ctx.expect_overlap(
                button,
                gauge,
                axes="y",
                elem_a="button_stem",
                elem_b="button_guide",
                min_overlap=0.009,
                name="button stem remains engaged in the guide at rest",
            )
            rest_button_pos = ctx.part_world_position(button)

        with ctx.pose({button_joint: button_limits.upper}):
            ctx.expect_within(
                button,
                gauge,
                axes="xz",
                inner_elem="button_stem",
                outer_elem="button_guide",
                margin=0.0,
                name="button stem stays aligned when pressed",
            )
            ctx.expect_overlap(
                button,
                gauge,
                axes="y",
                elem_a="button_stem",
                elem_b="button_guide",
                min_overlap=0.006,
                name="button stem remains inserted when pressed",
            )
            pressed_button_pos = ctx.part_world_position(button)

        ctx.check(
            "pressure release button presses inward",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[1] < rest_button_pos[1] - 0.002,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    needle_limits = needle_joint.motion_limits
    if needle_limits is not None and needle_limits.lower is not None and needle_limits.upper is not None:
        with ctx.pose({needle_joint: needle_limits.lower}):
            low_pointer_center = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))
        with ctx.pose({needle_joint: needle_limits.upper}):
            high_pointer_center = _aabb_center(ctx.part_element_world_aabb(needle, elem="pointer"))

        ctx.check(
            "needle sweeps across the gauge face",
            low_pointer_center is not None
            and high_pointer_center is not None
            and abs(high_pointer_center[1] - low_pointer_center[1]) > 0.010
            and abs(high_pointer_center[2] - low_pointer_center[2]) > 0.010,
            details=f"low={low_pointer_center}, high={high_pointer_center}",
        )

    return ctx.report()


object_model = build_object_model()
