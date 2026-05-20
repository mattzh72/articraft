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


def _octagonal_bezel() -> cq.Workplane:
    """Low raised octagonal trim ring around the gimbal well."""
    height = 0.018
    outer = cq.Workplane("XY").polygon(8, 0.142).extrude(height)
    opening = cq.Workplane("XY").circle(0.044).extrude(height + 0.006).translate((0.0, 0.0, -0.003))
    return outer.cut(opening)


def _outer_fork_guard() -> cq.Workplane:
    """Squat rotating outer yoke: annular guard with two inner bearing cheeks."""
    ring = cq.Workplane("XY").circle(0.052).circle(0.036).extrude(0.012).translate((0.0, 0.0, -0.006))
    cheek_0 = cq.Workplane("XY").box(0.026, 0.012, 0.026).translate((0.0, 0.044, 0.0))
    cheek_1 = cq.Workplane("XY").box(0.026, 0.012, 0.026).translate((0.0, -0.044, 0.0))
    return ring.union(cheek_0).union(cheek_1)


def _inner_ring_carrier() -> cq.Workplane:
    """Vertical inner gimbal ring with spokes and a central hub."""
    thickness = 0.012
    annulus = (
        cq.Workplane("XZ")
        .circle(0.030)
        .circle(0.018)
        .extrude(thickness)
        .translate((0.0, -thickness / 2.0, 0.0))
    )
    hub = cq.Workplane("XZ").circle(0.009).extrude(thickness).translate((0.0, -thickness / 2.0, 0.0))
    horizontal_spoke = (
        cq.Workplane("XZ")
        .rect(0.060, 0.006)
        .extrude(thickness)
        .translate((0.0, -thickness / 2.0, 0.0))
    )
    vertical_spoke = (
        cq.Workplane("XZ")
        .rect(0.006, 0.060)
        .extrude(thickness)
        .translate((0.0, -thickness / 2.0, 0.0))
    )
    return annulus.union(hub).union(horizontal_spoke).union(vertical_spoke)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guarded_thumbstick")

    console_mat = model.material("matte_console", rgba=(0.025, 0.028, 0.030, 1.0))
    bezel_mat = model.material("blackened_steel", rgba=(0.070, 0.074, 0.078, 1.0))
    fork_mat = model.material("dark_machined_fork", rgba=(0.12, 0.13, 0.14, 1.0))
    steel_mat = model.material("brushed_pivot_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    ring_mat = model.material("oxide_inner_ring", rgba=(0.025, 0.030, 0.035, 1.0))
    rubber_mat = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))

    plate_thickness = 0.024
    pivot_z = 0.060

    base = model.part("base")
    base.visual(
        Box((0.180, 0.160, plate_thickness)),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness / 2.0)),
        material=console_mat,
        name="console_plate",
    )
    base.visual(
        mesh_from_cadquery(_octagonal_bezel(), "octagonal_bezel"),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness)),
        material=bezel_mat,
        name="octagonal_bezel",
    )
    base.visual(
        Cylinder(radius=0.042, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, plate_thickness + 0.002)),
        material=rubber_mat,
        name="recess_liner",
    )

    for index, x in enumerate((-0.062, 0.062)):
        base.visual(
            Box((0.020, 0.040, 0.040)),
            origin=Origin(xyz=(x, 0.0, 0.046)),
            material=bezel_mat,
            name=f"bearing_pedestal_{index}",
        )
        base.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(x, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel_mat,
            name=f"fork_bearing_{index}",
        )

    outer_fork = model.part("outer_fork")
    outer_fork.visual(
        mesh_from_cadquery(_outer_fork_guard(), "outer_fork_guard"),
        material=fork_mat,
        name="fork_guard",
    )
    for index, y in enumerate((-0.035, 0.035)):
        outer_fork.visual(
            Box((0.018, 0.006, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.0)),
            material=steel_mat,
            name=f"ring_bearing_{index}",
        )
    for index, x in enumerate((-0.043, 0.043)):
        outer_fork.visual(
            Cylinder(radius=0.007, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel_mat,
            name=f"trunnion_{index}",
        )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        mesh_from_cadquery(_inner_ring_carrier(), "inner_ring_carrier"),
        material=ring_mat,
        name="ring_carrier",
    )
    inner_ring.visual(
        Cylinder(radius=0.0055, length=0.064),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel_mat,
        name="ring_pivot_pin",
    )
    inner_ring.visual(
        Cylinder(radius=0.010, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=rubber_mat,
        name="control_post",
    )
    inner_ring.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=rubber_mat,
        name="thumb_cap",
    )

    model.articulation(
        "base_to_fork",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_fork,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "fork_to_ring",
        ArticulationType.REVOLUTE,
        parent=outer_fork,
        child=inner_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0, lower=-0.35, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_fork = object_model.get_part("outer_fork")
    inner_ring = object_model.get_part("inner_ring")
    fork_joint = object_model.get_articulation("base_to_fork")
    ring_joint = object_model.get_articulation("fork_to_ring")

    ctx.expect_within(
        inner_ring,
        outer_fork,
        axes="xy",
        inner_elem="ring_carrier",
        outer_elem="fork_guard",
        margin=0.0,
        name="inner ring nests inside outer guard footprint",
    )
    ctx.expect_within(
        outer_fork,
        base,
        axes="xy",
        inner_elem="fork_guard",
        outer_elem="octagonal_bezel",
        margin=0.004,
        name="outer fork sits inside the octagonal bezel plan",
    )

    ctx.check(
        "gimbal axes are orthogonal horizontal axes",
        tuple(fork_joint.axis) == (1.0, 0.0, 0.0) and tuple(ring_joint.axis) == (0.0, 1.0, 0.0),
        details=f"fork_axis={fork_joint.axis}, ring_axis={ring_joint.axis}",
    )

    def _aabb_center(bounds: object) -> tuple[float, float, float] | None:
        if bounds is None:
            return None
        lo, hi = bounds
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    neutral_cap = _aabb_center(ctx.part_element_world_aabb(inner_ring, elem="thumb_cap"))
    with ctx.pose({fork_joint: 0.30}):
        forked_cap = _aabb_center(ctx.part_element_world_aabb(inner_ring, elem="thumb_cap"))
    with ctx.pose({ring_joint: 0.30}):
        ringed_cap = _aabb_center(ctx.part_element_world_aabb(inner_ring, elem="thumb_cap"))

    ctx.check(
        "outer fork rotation tilts the post about the x axis",
        neutral_cap is not None and forked_cap is not None and abs(forked_cap[1] - neutral_cap[1]) > 0.010,
        details=f"neutral={neutral_cap}, posed={forked_cap}",
    )
    ctx.check(
        "inner ring rotation tilts the post about the y axis",
        neutral_cap is not None and ringed_cap is not None and abs(ringed_cap[0] - neutral_cap[0]) > 0.010,
        details=f"neutral={neutral_cap}, posed={ringed_cap}",
    )

    return ctx.report()


object_model = build_object_model()
