from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="home_video_camcorder")

    body_dark = model.material("body_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    shell_gray = model.material("shell_gray", rgba=(0.24, 0.25, 0.27, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.06, 0.06, 0.07, 1.0))
    glass_dark = model.material("glass_dark", rgba=(0.05, 0.07, 0.10, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.10, 0.14, 0.18, 1.0))
    dial_gray = model.material("dial_gray", rgba=(0.30, 0.31, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.108, 0.050, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=body_dark,
        name="main_shell",
    )
    body.visual(
        Box((0.056, 0.042, 0.012)),
        origin=Origin(xyz=(-0.012, 0.0, 0.066)),
        material=shell_gray,
        name="top_shell",
    )
    body.visual(
        Box((0.050, 0.010, 0.050)),
        origin=Origin(xyz=(0.005, -0.030, 0.032)),
        material=shell_gray,
        name="grip_side",
    )
    body.visual(
        Box((0.026, 0.036, 0.024)),
        origin=Origin(xyz=(-0.040, 0.0, 0.052)),
        material=body_dark,
        name="rear_pack",
    )
    body.visual(
        Box((0.010, 0.038, 0.038)),
        origin=Origin(xyz=(0.049, 0.0, 0.037)),
        material=trim_dark,
        name="lens_mount",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.016),
        origin=Origin(xyz=(0.046, 0.0, 0.037), rpy=(0.0, pi / 2.0, 0.0)),
        material=shell_gray,
        name="lens_shroud",
    )
    body.visual(
        Box((0.006, 0.003, 0.048)),
        origin=Origin(xyz=(-0.022, 0.0255, 0.036)),
        material=shell_gray,
        name="screen_support",
    )
    body.visual(
        Box((0.020, 0.014, 0.016)),
        origin=Origin(xyz=(-0.049, 0.0, 0.042)),
        material=trim_dark,
        name="eyecup",
    )
    body.visual(
        Box((0.044, 0.044, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.005)),
        material=body_dark,
        name="lower_chin",
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.058, 0.012, 0.008)),
        origin=Origin(xyz=(0.018, 0.006, 0.094)),
        material=trim_dark,
        name="handle_beam",
    )
    handle.visual(
        Box((0.008, 0.010, 0.022)),
        origin=Origin(xyz=(0.036, 0.006, 0.083)),
        material=trim_dark,
        name="front_pillar",
    )
    handle.visual(
        Box((0.008, 0.010, 0.022)),
        origin=Origin(xyz=(0.000, 0.006, 0.083)),
        material=trim_dark,
        name="rear_pillar",
    )

    strap = model.part("strap")
    strap.visual(
        Box((0.014, 0.012, 0.012)),
        origin=Origin(xyz=(0.018, -0.041, 0.045)),
        material=trim_dark,
        name="front_anchor",
    )
    strap.visual(
        Box((0.014, 0.012, 0.012)),
        origin=Origin(xyz=(-0.024, -0.041, 0.043)),
        material=trim_dark,
        name="rear_anchor",
    )
    strap.visual(
        Box((0.052, 0.004, 0.020)),
        origin=Origin(xyz=(-0.003, -0.045, 0.044)),
        material=shell_gray,
        name="strap_band",
    )

    screen = model.part("screen")
    screen.visual(
        Box((0.062, 0.008, 0.046)),
        origin=Origin(xyz=(0.031, 0.004, 0.023)),
        material=shell_gray,
        name="panel_frame",
    )
    screen.visual(
        Box((0.052, 0.0016, 0.036)),
        origin=Origin(xyz=(0.034, 0.0016, 0.023)),
        material=glass_dark,
        name="display",
    )
    screen.visual(
        Box((0.006, 0.010, 0.046)),
        origin=Origin(xyz=(0.003, 0.005, 0.023)),
        material=trim_dark,
        name="hinge_edge",
    )

    lens_ring = model.part("lens_ring")
    lens_ring.visual(
        Cylinder(radius=0.021, length=0.024),
        origin=Origin(xyz=(0.012, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_dark,
        name="ring_barrel",
    )
    lens_ring.visual(
        Cylinder(radius=0.015, length=0.002),
        origin=Origin(xyz=(0.023, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_glass",
    )
    lens_ring.visual(
        Box((0.006, 0.010, 0.004)),
        origin=Origin(xyz=(0.015, 0.0, 0.023)),
        material=shell_gray,
        name="zoom_grip",
    )

    mode_dial = model.part("mode_dial")
    mode_dial.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dial_gray,
        name="dial_body",
    )
    mode_dial.visual(
        Box((0.005, 0.003, 0.002)),
        origin=Origin(xyz=(0.008, 0.0, 0.007)),
        material=shell_gray,
        name="dial_tab",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.FIXED,
        parent=body,
        child=handle,
        origin=Origin(),
    )
    model.articulation(
        "body_to_strap",
        ArticulationType.FIXED,
        parent=body,
        child=strap,
        origin=Origin(),
    )
    model.articulation(
        "screen_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=screen,
        origin=Origin(xyz=(-0.026, 0.027, 0.013)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.5, lower=0.0, upper=1.6),
    )
    model.articulation(
        "lens_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=lens_ring,
        origin=Origin(xyz=(0.054, 0.0, 0.037)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=8.0),
    )
    model.articulation(
        "dial_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=mode_dial,
        origin=Origin(xyz=(-0.030, 0.006, 0.072)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
    )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    screen = object_model.get_part("screen")
    lens_ring = object_model.get_part("lens_ring")
    mode_dial = object_model.get_part("mode_dial")

    screen_hinge = object_model.get_articulation("screen_hinge")
    lens_spin = object_model.get_articulation("lens_spin")
    dial_spin = object_model.get_articulation("dial_spin")

    ctx.expect_gap(
        screen,
        body,
        axis="y",
        max_gap=0.0015,
        max_penetration=0.0,
        name="screen closes flush to the body side",
    )
    ctx.expect_overlap(
        screen,
        body,
        axes="xz",
        min_overlap=0.030,
        name="closed screen covers the side display area",
    )
    ctx.expect_gap(
        mode_dial,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-5,
        name="mode dial sits on the top shell",
    )
    ctx.expect_overlap(
        mode_dial,
        body,
        axes="xy",
        min_overlap=0.020,
        name="mode dial stays centered over the top shell",
    )

    closed_screen_center = None
    open_screen_center = None
    with ctx.pose({screen_hinge: 0.0}):
        closed_screen_center = _aabb_center(ctx.part_element_world_aabb(screen, elem="panel_frame"))
    with ctx.pose({screen_hinge: 1.2}):
        open_screen_center = _aabb_center(ctx.part_element_world_aabb(screen, elem="panel_frame"))
        ctx.expect_overlap(
            screen,
            body,
            axes="z",
            min_overlap=0.035,
            name="opened screen stays vertically aligned with the body",
        )

    ctx.check(
        "screen swings outward from the left side",
        closed_screen_center is not None
        and open_screen_center is not None
        and open_screen_center[1] > closed_screen_center[1] + 0.020,
        details=f"closed_center={closed_screen_center}, open_center={open_screen_center}",
    )

    lens_grip_center_0 = None
    lens_grip_center_90 = None
    with ctx.pose({lens_spin: 0.0}):
        lens_grip_center_0 = _aabb_center(ctx.part_element_world_aabb(lens_ring, elem="zoom_grip"))
    with ctx.pose({lens_spin: pi / 2.0}):
        lens_grip_center_90 = _aabb_center(ctx.part_element_world_aabb(lens_ring, elem="zoom_grip"))

    ctx.check(
        "front zoom ring visibly rotates about the optical axis",
        lens_grip_center_0 is not None
        and lens_grip_center_90 is not None
        and abs(lens_grip_center_90[1] - lens_grip_center_0[1]) > 0.012
        and abs(lens_grip_center_90[2] - lens_grip_center_0[2]) > 0.012,
        details=f"q0={lens_grip_center_0}, q90={lens_grip_center_90}",
    )

    dial_tab_center_0 = None
    dial_tab_center_90 = None
    with ctx.pose({dial_spin: 0.0}):
        dial_tab_center_0 = _aabb_center(ctx.part_element_world_aabb(mode_dial, elem="dial_tab"))
    with ctx.pose({dial_spin: pi / 2.0}):
        dial_tab_center_90 = _aabb_center(ctx.part_element_world_aabb(mode_dial, elem="dial_tab"))

    ctx.check(
        "top mode dial visibly rotates on its center axis",
        dial_tab_center_0 is not None
        and dial_tab_center_90 is not None
        and abs(dial_tab_center_90[0] - dial_tab_center_0[0]) > 0.006
        and abs(dial_tab_center_90[1] - dial_tab_center_0[1]) > 0.006,
        details=f"q0={dial_tab_center_0}, q90={dial_tab_center_90}",
    )

    handle_aabb = ctx.part_world_aabb(handle)
    dial_aabb = ctx.part_world_aabb(mode_dial)
    ctx.check(
        "mode dial sits behind the carry handle",
        handle_aabb is not None
        and dial_aabb is not None
        and dial_aabb[1][0] < handle_aabb[0][0] - 0.005,
        details=f"handle_aabb={handle_aabb}, dial_aabb={dial_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
