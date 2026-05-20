from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

MM = 0.001


def _box(size_x: float, size_y: float, size_z: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center)
    )


def _make_base_shape() -> cq.Workplane:
    plate = _box(62.0, 118.0, 2.5, (0.0, 0.0, 1.25))
    deck = _box(26.0, 96.0, 7.5, (-2.0, 0.0, 6.25))
    front_pedestal = _box(10.0, 18.0, 8.4, (24.0, 0.0, 6.7))
    left_post = _box(8.0, 14.0, 12.5, (-26.0, -45.0, 8.75))
    right_post = _box(8.0, 14.0, 12.5, (-26.0, 45.0, 8.75))
    toe_rib = _box(6.0, 90.0, 3.5, (22.0, 0.0, 4.25))

    base = (
        plate.union(deck)
        .union(front_pedestal)
        .union(left_post)
        .union(right_post)
        .union(toe_rib)
    )

    punch_holes = (
        cq.Workplane("XY")
        .pushPoints([(-2.0, -40.0), (-2.0, 40.0)])
        .circle(3.6)
        .extrude(20.0)
        .translate((0.0, 0.0, -2.0))
    )
    paper_slot = _box(20.0, 100.0, 4.0, (11.0, 0.0, 3.5))

    return base.cut(punch_holes).cut(paper_slot)


def _make_handle_shape() -> cq.Workplane:
    top_skin = _box(48.0, 108.0, 1.6, (28.0, 0.0, 6.0))
    left_skirt = _box(42.0, 1.8, 9.0, (28.0, -53.1, 1.0))
    right_skirt = _box(42.0, 1.8, 9.0, (28.0, 53.1, 1.0))
    front_lip = _box(4.0, 100.0, 8.0, (50.0, 0.0, 2.0))
    rear_bridge = _box(8.0, 76.0, 7.0, (6.0, 0.0, 3.0))
    spine = _box(16.0, 18.0, 8.0, (20.0, 0.0, 1.5))
    punch_bar = _box(8.0, 86.0, 4.0, (20.0, 0.0, -2.5))
    latch_tab = _box(5.0, 14.0, 4.0, (47.0, 0.0, -1.5))
    hinge_barrel = _y_cylinder(3.7, 76.0, (0.0, 0.0, 0.0))
    left_punch = cq.Workplane("XY").circle(3.1).extrude(3.4).translate((20.0, -40.0, -7.7))
    right_punch = cq.Workplane("XY").circle(3.1).extrude(3.4).translate((20.0, 40.0, -7.7))

    return (
        hinge_barrel.union(rear_bridge)
        .union(top_skin)
        .union(left_skirt)
        .union(right_skirt)
        .union(front_lip)
        .union(spine)
        .union(punch_bar)
        .union(latch_tab)
        .union(left_punch)
        .union(right_punch)
    )


def _make_clip_shape() -> cq.Workplane:
    pivot_barrel = _y_cylinder(2.3, 14.0, (0.0, 0.0, 0.0))
    arm = _box(8.0, 14.0, 2.2, (-1.5, 0.0, 0.3))
    hook = _box(3.0, 14.0, 3.0, (-4.0, 0.0, -2.0))
    finger_tab = _box(6.0, 14.0, 2.6, (4.0, 0.0, 0.5))

    return pivot_barrel.union(arm).union(hook).union(finger_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_hole_punch")

    model.material("painted_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    model.material("stamped_steel", rgba=(0.73, 0.75, 0.77, 1.0))
    model.material("clip_steel", rgba=(0.64, 0.66, 0.69, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base", unit_scale=MM),
        material="painted_steel",
        name="base_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_make_handle_shape(), "handle", unit_scale=MM),
        material="stamped_steel",
        name="handle_shell",
    )

    clip = model.part("clip")
    clip.visual(
        mesh_from_cadquery(_make_clip_shape(), "clip", unit_scale=MM),
        material="clip_steel",
        name="clip_body",
    )

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.026, 0.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=30.0, velocity=3.0),
    )
    model.articulation(
        "clip_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=clip,
        origin=Origin(xyz=(0.024, 0.0, 0.013)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.05, effort=8.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    clip = object_model.get_part("clip")
    handle_hinge = object_model.get_articulation("handle_hinge")
    clip_pivot = object_model.get_articulation("clip_pivot")

    ctx.allow_overlap(
        base,
        clip,
        reason="The front lock clip uses a simplified solid pivot lug instead of a separate pin and bored clevis.",
    )

    ctx.expect_overlap(
        handle,
        base,
        axes="xy",
        min_overlap=0.04,
        name="stored handle covers the paper throat",
    )
    ctx.expect_overlap(
        clip,
        handle,
        axes="y",
        min_overlap=0.012,
        name="lock clip aligns with the centered handle latch",
    )

    base_aabb = ctx.part_world_aabb(base)
    closed_handle_aabb = ctx.part_world_aabb(handle)
    closed_clip_aabb = ctx.part_world_aabb(clip)

    stored_height_ok = (
        base_aabb is not None
        and closed_handle_aabb is not None
        and closed_handle_aabb[1][2] <= base_aabb[1][2] + 0.014
    )
    ctx.check(
        "stored handle stays compact above the base",
        stored_height_ok,
        details=f"base={base_aabb}, handle={closed_handle_aabb}",
    )

    clip_release = clip_pivot.motion_limits.upper if clip_pivot.motion_limits is not None else 1.0
    handle_open = handle_hinge.motion_limits.upper if handle_hinge.motion_limits is not None else 1.0

    with ctx.pose({clip_pivot: clip_release}):
        released_clip_aabb = ctx.part_world_aabb(clip)

    clip_releases = (
        closed_clip_aabb is not None
        and released_clip_aabb is not None
        and released_clip_aabb[0][2] < closed_clip_aabb[0][2] - 0.002
        and released_clip_aabb[0][0] > closed_clip_aabb[0][0] + 0.0005
    )
    ctx.check(
        "clip rotates downward to release storage lock",
        clip_releases,
        details=f"stored={closed_clip_aabb}, released={released_clip_aabb}",
    )

    with ctx.pose({clip_pivot: clip_release, handle_hinge: handle_open}):
        open_handle_aabb = ctx.part_world_aabb(handle)
        ctx.expect_overlap(
            handle,
            base,
            axes="y",
            min_overlap=0.09,
            name="open handle stays centered across the base width",
        )

    handle_lifts = (
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.025
    )
    ctx.check(
        "handle lifts clearly when the clip is released",
        handle_lifts,
        details=f"stored={closed_handle_aabb}, open={open_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
