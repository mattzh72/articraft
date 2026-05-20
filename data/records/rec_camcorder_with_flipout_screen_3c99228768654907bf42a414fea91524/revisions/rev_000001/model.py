from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _body_section(x_pos: float, width: float, height: float, radius: float) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + height * 0.5)
        for y_pos, z_pos in rounded_rect_profile(width, height, radius, corner_segments=8)
    ]


def _elem_center(ctx: TestContext, part, elem: str) -> tuple[float, float, float] | None:
    aabb = ctx.part_element_world_aabb(part, elem=elem)
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_hd_camcorder")

    body_finish = model.material("body_finish", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.06, 0.06, 0.07, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.60, 0.62, 0.66, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.13, 0.18, 0.23, 0.90))
    clear_poly = model.material("clear_poly", rgba=(0.80, 0.92, 1.00, 0.35))

    body = model.part("body")

    body_shell = section_loft(
        [
            _body_section(-0.055, 0.047, 0.054, 0.010),
            _body_section(-0.018, 0.055, 0.063, 0.012),
            _body_section(0.022, 0.058, 0.068, 0.013),
            _body_section(0.055, 0.048, 0.052, 0.011),
        ]
    )
    grip_shell = CapsuleGeometry(radius=0.015, length=0.030)

    body.visual(_mesh("camcorder_body_shell", body_shell), material=body_finish, name="body_shell")
    body.visual(
        _mesh("camcorder_grip_shell", grip_shell),
        origin=Origin(xyz=(0.019, -0.039, 0.036)),
        material=body_finish,
        name="grip_shell",
    )
    body.visual(
        Box((0.058, 0.018, 0.052)),
        origin=Origin(xyz=(0.016, -0.023, 0.036)),
        material=body_finish,
        name="grip_bridge",
    )
    body.visual(
        Box((0.072, 0.007, 0.038)),
        origin=Origin(xyz=(0.013, -0.050, 0.036)),
        material=grip_rubber,
        name="hand_strap",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.054, 0.0, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.064, 0.0, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="barrel_shell",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.074, 0.0, 0.034), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=screen_glass,
        name="lens_glass",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.018),
        origin=Origin(xyz=(-0.004, 0.0, 0.077)),
        material=trim_dark,
        name="top_handle_rear_post",
    )
    body.visual(
        Cylinder(radius=0.0048, length=0.022),
        origin=Origin(xyz=(0.017, 0.0, 0.079)),
        material=trim_dark,
        name="top_handle_front_post",
    )
    body.visual(
        _mesh("camcorder_handle_bar", CapsuleGeometry(radius=0.0052, length=0.028)),
        origin=Origin(xyz=(0.007, 0.0, 0.089), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="top_handle_bar",
    )
    body.visual(
        Box((0.028, 0.004, 0.050)),
        origin=Origin(xyz=(-0.016, 0.030, 0.038)),
        material=trim_dark,
        name="hinge_pad",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(-0.023, -0.010, 0.063)),
        material=satin_metal,
        name="dial_mount",
    )

    monitor = model.part("monitor")
    monitor.visual(
        Cylinder(radius=0.0042, length=0.054),
        material=clear_poly,
        name="hinge_spine",
    )
    monitor.visual(
        Box((0.074, 0.006, 0.050)),
        origin=Origin(xyz=(0.037, 0.004, 0.0)),
        material=body_finish,
        name="monitor_shell",
    )
    monitor.visual(
        Box((0.064, 0.0012, 0.044)),
        origin=Origin(xyz=(0.039, 0.0004, 0.0)),
        material=screen_glass,
        name="monitor_glass",
    )
    monitor.visual(
        Box((0.012, 0.003, 0.050)),
        origin=Origin(xyz=(0.005, 0.002, 0.0)),
        material=clear_poly,
        name="hinge_flange",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        Cylinder(radius=0.025, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="ring_shell",
    )
    lens_ring.visual(
        Box((0.010, 0.004, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=trim_dark,
        name="focus_tab",
    )

    mode_dial = model.part("mode_dial")
    dial_shell = KnobGeometry(
        0.028,
        0.007,
        body_style="cylindrical",
        edge_radius=0.0012,
        grip=KnobGrip(style="fluted", count=24, depth=0.0012),
        center=False,
    )
    mode_dial.visual(
        _mesh("camcorder_mode_dial", dial_shell),
        material=trim_dark,
        name="dial_shell",
    )
    mode_dial.visual(
        Box((0.006, 0.003, 0.0015)),
        origin=Origin(xyz=(0.011, 0.0, 0.0072)),
        material=satin_metal,
        name="selector_tab",
    )

    model.articulation(
        "monitor_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=monitor,
        origin=Origin(xyz=(-0.020, 0.0325, 0.038)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "lens_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.083, 0.0, 0.034)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=8.0,
        ),
    )
    model.articulation(
        "mode_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.023, -0.010, 0.069)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    monitor = object_model.get_part("monitor")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")

    monitor_hinge = object_model.get_articulation("monitor_hinge")
    lens_spin = object_model.get_articulation("lens_spin")
    mode_spin = object_model.get_articulation("mode_spin")

    ctx.expect_gap(
        monitor,
        body,
        axis="y",
        positive_elem="monitor_shell",
        negative_elem="hinge_pad",
        min_gap=0.0005,
        max_gap=0.004,
        name="monitor closes just outside the left-side hinge pad",
    )
    ctx.expect_overlap(
        monitor,
        body,
        axes="xz",
        elem_a="monitor_shell",
        elem_b="hinge_pad",
        min_overlap=0.012,
        name="monitor stays aligned with the body side when closed",
    )
    ctx.expect_gap(
        lens_ring,
        body,
        axis="x",
        positive_elem="ring_shell",
        negative_elem="barrel_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="lens ring sits just ahead of the short barrel housing",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        positive_elem="dial_shell",
        negative_elem="dial_mount",
        max_gap=0.001,
        max_penetration=0.0005,
        name="mode dial seats on its top-shell mount",
    )

    handle_center = _elem_center(ctx, body, "top_handle_bar")
    dial_center = _elem_center(ctx, mode_dial, "dial_shell")
    ctx.check(
        "mode dial sits behind the top handle",
        handle_center is not None
        and dial_center is not None
        and dial_center[0] < handle_center[0] - 0.012,
        details=f"handle_center={handle_center}, dial_center={dial_center}",
    )

    closed_monitor_center = _elem_center(ctx, monitor, "monitor_shell")
    with ctx.pose({monitor_hinge: math.radians(100.0)}):
        open_monitor_center = _elem_center(ctx, monitor, "monitor_shell")
    ctx.check(
        "monitor swings outward from the left side",
        closed_monitor_center is not None
        and open_monitor_center is not None
        and open_monitor_center[1] > closed_monitor_center[1] + 0.025,
        details=f"closed={closed_monitor_center}, open={open_monitor_center}",
    )

    rest_ring_tab = _elem_center(ctx, lens_ring, "focus_tab")
    with ctx.pose({lens_spin: math.pi / 2.0}):
        spun_ring_tab = _elem_center(ctx, lens_ring, "focus_tab")
    ctx.check(
        "lens ring rotates around the barrel axis",
        rest_ring_tab is not None
        and spun_ring_tab is not None
        and abs(spun_ring_tab[1] - rest_ring_tab[1]) > 0.020
        and spun_ring_tab[2] < rest_ring_tab[2] - 0.018,
        details=f"rest={rest_ring_tab}, spun={spun_ring_tab}",
    )

    rest_dial_tab = _elem_center(ctx, mode_dial, "selector_tab")
    with ctx.pose({mode_spin: math.pi / 2.0}):
        spun_dial_tab = _elem_center(ctx, mode_dial, "selector_tab")
    ctx.check(
        "mode dial rotates on its vertical center axis",
        rest_dial_tab is not None
        and spun_dial_tab is not None
        and spun_dial_tab[1] > rest_dial_tab[1] + 0.008
        and spun_dial_tab[0] < rest_dial_tab[0] - 0.008,
        details=f"rest={rest_dial_tab}, spun={spun_dial_tab}",
    )

    return ctx.report()


object_model = build_object_model()
