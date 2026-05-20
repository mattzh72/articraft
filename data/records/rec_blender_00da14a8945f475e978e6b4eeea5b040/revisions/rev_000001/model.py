from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def _ring(outer_radius: float, inner_radius: float, height: float, z: float = 0.0):
    """CadQuery annular cylinder spanning z..z+height."""
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, z))
    )


def _cup_shell():
    bottom = cq.Workplane("XY").circle(0.085).extrude(0.008)
    wall = _ring(0.085, 0.079, 0.137, z=0.008)
    return bottom.union(wall)


def _socket_ring():
    # A stepped bayonet receiver: a wide shallow recess above a smaller bore.
    rim_deck = _ring(0.085, 0.056, 0.006, z=0.141)
    outer_wall = _ring(0.058, 0.052, 0.042, z=0.143)
    lower_shelf = _ring(0.052, 0.034, 0.010, z=0.143)
    return rim_deck.union(outer_wall).union(lower_shelf)


def _lock_flange():
    return _ring(0.057, 0.020, 0.006, z=0.0)


def _lock_sleeve():
    sleeve = _ring(0.031, 0.020, 0.032, z=-0.032)
    tab_a = cq.Workplane("XY").box(0.020, 0.010, 0.008).translate((0.038, 0.0, -0.014))
    tab_b = cq.Workplane("XY").box(0.020, 0.010, 0.008).translate((-0.038, 0.0, -0.014))
    return sleeve.union(tab_a).union(tab_b)


def _guard_cage():
    bottom_hoop = _ring(0.047, 0.039, 0.010, z=-0.135)
    center_boss = cq.Workplane("XY").circle(0.016).extrude(0.014).translate((0.0, 0.0, -0.122))
    shape = bottom_hoop.union(center_boss)
    for angle in (0.0, 90.0, 180.0, 270.0):
        spoke = (
            cq.Workplane("XY")
            .box(0.060, 0.006, 0.006)
            .translate((0.026, 0.0, -0.114))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        post = (
            cq.Workplane("XY")
            .box(0.006, 0.006, 0.024)
            .translate((0.043, 0.0, -0.123))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        shape = shape.union(spoke).union(post)
    return shape


def _blade_star():
    hub = cq.Workplane("XY").circle(0.008).extrude(0.015).translate((0.0, 0.0, -0.006))
    blade_profile = [(0.004, -0.004), (0.036, -0.011), (0.033, 0.006), (0.004, 0.005)]
    shape = hub
    for angle in (0.0, 120.0, 240.0):
        blade = (
            cq.Workplane("XY")
            .polyline(blade_profile)
            .close()
            .extrude(0.0025)
            .translate((0.0, 0.0, -0.00125))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        shape = shape.union(blade)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smoothie_blender_wand")

    clear_plastic = model.material("clear_plastic", rgba=(0.65, 0.90, 1.0, 0.35))
    white_plastic = model.material("white_plastic", rgba=(0.93, 0.94, 0.91, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.42, 0.45, 0.47, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.08, 0.09, 0.10, 1.0))
    stainless = model.material("stainless", rgba=(0.70, 0.72, 0.72, 1.0))
    blade_metal = model.material("blade_metal", rgba=(0.90, 0.92, 0.95, 1.0))

    base_cup = model.part("base_cup")
    base_cup.visual(
        mesh_from_cadquery(_cup_shell(), "round_cup_shell", tolerance=0.0012),
        material=clear_plastic,
        name="cup_shell",
    )
    base_cup.visual(
        mesh_from_cadquery(_socket_ring(), "twist_socket_ring", tolerance=0.001),
        material=soft_gray,
        name="socket_ring",
    )
    base_cup.visual(
        Cylinder(radius=0.091, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_gray,
        name="foot_ring",
    )

    lock_slide = model.part("lock_slide")
    lock_slide.visual(
        mesh_from_cadquery(_lock_flange(), "press_lock_flange", tolerance=0.0008),
        material=dark_gray,
        name="flange",
    )
    lock_slide.visual(
        mesh_from_cadquery(_lock_sleeve(), "press_lock_sleeve", tolerance=0.0008),
        material=dark_gray,
        name="sleeve_tabs",
    )

    wand_body = model.part("wand_body")
    wand_body.visual(
        Cylinder(radius=0.035, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=white_plastic,
        name="lock_collar",
    )
    wand_body.visual(
        Cylinder(radius=0.030, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=white_plastic,
        name="motor_tube",
    )
    wand_body.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.260)),
        material=white_plastic,
        name="rounded_cap",
    )
    wand_body.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=soft_gray,
        name="lower_grip_band",
    )
    wand_body.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=soft_gray,
        name="upper_grip_band",
    )
    wand_body.visual(
        Cylinder(radius=0.013, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=stainless,
        name="wand_shaft",
    )
    wand_body.visual(
        mesh_from_cadquery(_guard_cage(), "blade_guard_cage", tolerance=0.0008),
        material=stainless,
        name="guard_cage",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_cadquery(_blade_star(), "tri_blade_star", tolerance=0.0005),
        material=blade_metal,
        name="blade_star",
    )

    model.articulation(
        "base_to_lock_slide",
        ArticulationType.PRISMATIC,
        parent=base_cup,
        child=lock_slide,
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.08, lower=0.0, upper=0.018),
    )
    model.articulation(
        "lock_slide_to_wand",
        ArticulationType.REVOLUTE,
        parent=lock_slide,
        child=wand_body,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=0.0, upper=math.radians(75.0)),
    )
    model.articulation(
        "wand_to_blade",
        ArticulationType.CONTINUOUS,
        parent=wand_body,
        child=blade,
        origin=Origin(xyz=(0.0, 0.0, -0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=80.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_cup = object_model.get_part("base_cup")
    lock_slide = object_model.get_part("lock_slide")
    wand_body = object_model.get_part("wand_body")
    blade = object_model.get_part("blade")
    press = object_model.get_articulation("base_to_lock_slide")
    twist = object_model.get_articulation("lock_slide_to_wand")
    spin = object_model.get_articulation("wand_to_blade")

    ctx.allow_overlap(
        base_cup,
        lock_slide,
        elem_a="socket_ring",
        elem_b="sleeve_tabs",
        reason="The press-lock sleeve is intentionally represented as sliding inside the bayonet socket proxy.",
    )
    ctx.allow_overlap(
        wand_body,
        blade,
        elem_a="guard_cage",
        elem_b="blade_star",
        reason="The blade axle is intentionally captured in the fixed bearing boss at the wand tip.",
    )

    ctx.expect_gap(
        lock_slide,
        base_cup,
        axis="z",
        positive_elem="flange",
        negative_elem="socket_ring",
        max_gap=0.001,
        max_penetration=0.0005,
        name="press sleeve rests on socket lip",
    )
    ctx.expect_gap(
        wand_body,
        lock_slide,
        axis="z",
        positive_elem="lock_collar",
        negative_elem="flange",
        max_gap=0.001,
        max_penetration=0.0005,
        name="wand collar sits on twist flange",
    )
    ctx.expect_within(
        lock_slide,
        base_cup,
        axes="xy",
        inner_elem="sleeve_tabs",
        outer_elem="socket_ring",
        margin=0.002,
        name="lock sleeve stays centered in socket",
    )
    ctx.expect_overlap(
        lock_slide,
        base_cup,
        axes="z",
        elem_a="sleeve_tabs",
        elem_b="socket_ring",
        min_overlap=0.025,
        name="lock sleeve remains inserted in socket",
    )
    ctx.expect_within(
        blade,
        wand_body,
        axes="xy",
        inner_elem="blade_star",
        outer_elem="guard_cage",
        margin=0.004,
        name="blade stays inside guard hoop",
    )
    ctx.expect_overlap(
        blade,
        wand_body,
        axes="z",
        elem_a="blade_star",
        elem_b="guard_cage",
        min_overlap=0.0005,
        name="blade axle is retained in bearing",
    )

    rest_pos = ctx.part_world_position(lock_slide)
    with ctx.pose({press: 0.018}):
        ctx.expect_overlap(
            lock_slide,
            base_cup,
            axes="z",
            elem_a="sleeve_tabs",
            elem_b="socket_ring",
            min_overlap=0.010,
            name="pressed sleeve stays retained",
        )
        pressed_pos = ctx.part_world_position(lock_slide)
    ctx.check(
        "press joint moves downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.012,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    ctx.check(
        "twist lock is a limited revolute mount",
        twist.articulation_type == ArticulationType.REVOLUTE
        and twist.motion_limits is not None
        and twist.motion_limits.upper is not None
        and 1.0 < twist.motion_limits.upper < 1.4,
        details=f"type={twist.articulation_type}, limits={twist.motion_limits}",
    )
    ctx.check(
        "blade joint spins continuously",
        spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spin.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
