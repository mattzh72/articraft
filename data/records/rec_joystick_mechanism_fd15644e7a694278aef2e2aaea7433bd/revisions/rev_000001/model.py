from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_root_joystick_gimbal")

    painted_steel = model.material("satin_black_steel", rgba=(0.025, 0.026, 0.028, 1.0))
    dark_cast = model.material("dark_cast_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    bare_steel = model.material("brushed_pin_steel", rgba=(0.70, 0.70, 0.66, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.006, 0.006, 0.005, 1.0))
    accent = model.material("red_center_band", rgba=(0.62, 0.035, 0.025, 1.0))

    pivot_z = 0.144

    support_fork = model.part("support_fork")
    support_fork.visual(
        Box((0.300, 0.220, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=painted_steel,
        name="base_plate",
    )
    support_fork.visual(
        Box((0.230, 0.120, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=dark_cast,
        name="fork_foot",
    )
    support_fork.visual(
        Box((0.030, 0.120, 0.165)),
        origin=Origin(xyz=(-0.100, 0.0, 0.0985)),
        material=dark_cast,
        name="fork_cheek_0",
    )
    support_fork.visual(
        Box((0.030, 0.120, 0.165)),
        origin=Origin(xyz=(0.100, 0.0, 0.0985)),
        material=dark_cast,
        name="fork_cheek_1",
    )
    support_fork.visual(
        Box((0.200, 0.016, 0.075)),
        origin=Origin(xyz=(0.0, -0.052, 0.0835)),
        material=dark_cast,
        name="rear_web",
    )
    support_fork.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(-0.081, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="fork_bearing_0",
    )
    support_fork.visual(
        Cylinder(radius=0.027, length=0.008),
        origin=Origin(xyz=(0.081, 0.0, pivot_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="fork_bearing_1",
    )
    support_fork.visual(
        Box((0.150, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.052, 0.025)),
        material=dark_cast,
        name="front_rib",
    )
    for i, (x, y) in enumerate(((-0.115, -0.080), (-0.115, 0.080), (0.115, -0.080), (0.115, 0.080))):
        support_fork.visual(
            Cylinder(radius=0.009, length=0.005),
            origin=Origin(xyz=(x, y, 0.0185)),
            material=bare_steel,
            name=f"mount_bolt_{i}",
        )

    outer_yoke = model.part("outer_yoke")
    # A rigid rectangular rocker frame.  The central splits in the fore/aft bars
    # leave real clearance for the inner-yoke shaft and are bridged by bearing
    # rings, so the member reads as one supported yoke rather than loose bars.
    for x in (-0.058, 0.058):
        outer_yoke.visual(
            Box((0.016, 0.156, 0.024)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=painted_steel,
            name=f"side_bar_{0 if x < 0 else 1}",
        )
    for y in (-0.070, 0.070):
        for x in (-0.043, 0.043):
            outer_yoke.visual(
                Box((0.046, 0.014, 0.024)),
                origin=Origin(xyz=(x, y, 0.0)),
                material=painted_steel,
                name=f"split_bar_{0 if y < 0 else 1}_{0 if x < 0 else 1}",
            )
    outer_yoke.visual(
        mesh_from_geometry(TorusGeometry(radius=0.018, tube=0.0048), "inner_bearing_0"),
        origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="inner_bearing_0",
    )
    outer_yoke.visual(
        mesh_from_geometry(TorusGeometry(radius=0.018, tube=0.0048), "inner_bearing_1"),
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="inner_bearing_1",
    )
    outer_yoke.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(-0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="outer_pin_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.024, length=0.005),
        origin=Origin(xyz=(-0.0745, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="outer_collar_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.011, length=0.018),
        origin=Origin(xyz=(0.068, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="outer_pin_1",
    )
    outer_yoke.visual(
        Cylinder(radius=0.024, length=0.005),
        origin=Origin(xyz=(0.0745, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bare_steel,
        name="outer_collar_1",
    )

    inner_yoke = model.part("inner_yoke")
    inner_yoke.visual(
        Cylinder(radius=0.0075, length=0.166),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="inner_shaft",
    )
    inner_yoke.visual(
        Cylinder(radius=0.019, length=0.005),
        origin=Origin(xyz=(0.0, -0.0627, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="inner_collar_0",
    )
    inner_yoke.visual(
        Cylinder(radius=0.019, length=0.005),
        origin=Origin(xyz=(0.0, 0.0627, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bare_steel,
        name="inner_collar_1",
    )
    inner_yoke.visual(
        Sphere(radius=0.023),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_cast,
        name="center_hub",
    )
    inner_yoke.visual(
        Cylinder(radius=0.012, length=0.235),
        origin=Origin(xyz=(0.0, 0.0, 0.128)),
        material=rubber,
        name="lever_stem",
    )
    inner_yoke.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.272)),
        material=accent,
        name="grip_band",
    )
    inner_yoke.visual(
        Sphere(radius=0.027),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=rubber,
        name="grip_knob",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.REVOLUTE,
        parent=support_fork,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_yoke,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0, lower=-0.50, upper=0.50),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_fork = object_model.get_part("support_fork")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_yoke = object_model.get_part("inner_yoke")
    support_to_outer = object_model.get_articulation("support_to_outer")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

    ctx.check("three supported links", len(object_model.parts) == 3)
    ctx.check("two revolute gimbal axes", len(object_model.articulations) == 2)
    ctx.check(
        "outer and inner axes are perpendicular",
        abs(sum(a * b for a, b in zip(support_to_outer.axis, outer_to_inner.axis))) < 1e-6,
        details=f"outer_axis={support_to_outer.axis}, inner_axis={outer_to_inner.axis}",
    )
    ctx.check(
        "outer fork axis has realistic stops",
        support_to_outer.motion_limits.lower <= -0.40 and support_to_outer.motion_limits.upper >= 0.40,
        details=str(support_to_outer.motion_limits),
    )
    ctx.check(
        "inner yoke axis has realistic stops",
        outer_to_inner.motion_limits.lower <= -0.45 and outer_to_inner.motion_limits.upper >= 0.45,
        details=str(outer_to_inner.motion_limits),
    )

    # The moving axes are not decorative: each is seated against a visible pair
    # of bearings/collars in the rest pose.
    ctx.expect_gap(
        outer_yoke,
        support_fork,
        axis="x",
        positive_elem="outer_collar_0",
        negative_elem="fork_bearing_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="outer yoke seats on first fork bearing",
    )
    ctx.expect_gap(
        support_fork,
        outer_yoke,
        axis="x",
        positive_elem="fork_bearing_1",
        negative_elem="outer_collar_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="outer yoke seats on second fork bearing",
    )
    ctx.expect_overlap(
        outer_yoke,
        support_fork,
        axes="yz",
        elem_a="outer_collar_0",
        elem_b="fork_bearing_0",
        min_overlap=0.030,
        name="outer collar footprint is captured by fork bearing",
    )
    ctx.expect_gap(
        inner_yoke,
        outer_yoke,
        axis="y",
        positive_elem="inner_collar_0",
        negative_elem="inner_bearing_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="inner yoke seats on first outer bearing",
    )
    ctx.expect_gap(
        outer_yoke,
        inner_yoke,
        axis="y",
        positive_elem="inner_bearing_1",
        negative_elem="inner_collar_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="inner yoke seats on second outer bearing",
    )
    ctx.expect_overlap(
        inner_yoke,
        outer_yoke,
        axes="xz",
        elem_a="inner_collar_0",
        elem_b="inner_bearing_0",
        min_overlap=0.030,
        name="inner collar footprint is captured by outer bearing",
    )

    def _element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lo, hi = bounds
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    rest_grip = _element_center(inner_yoke, "grip_knob")
    with ctx.pose({support_to_outer: 0.35}):
        outer_tilt_grip = _element_center(inner_yoke, "grip_knob")
    with ctx.pose({outer_to_inner: 0.40}):
        inner_tilt_grip = _element_center(inner_yoke, "grip_knob")

    ctx.check(
        "outer fork axis tilts lever about x",
        rest_grip is not None
        and outer_tilt_grip is not None
        and outer_tilt_grip[1] < rest_grip[1] - 0.08,
        details=f"rest={rest_grip}, tilted={outer_tilt_grip}",
    )
    ctx.check(
        "inner yoke axis tilts lever about y",
        rest_grip is not None
        and inner_tilt_grip is not None
        and inner_tilt_grip[0] > rest_grip[0] + 0.08,
        details=f"rest={rest_grip}, tilted={inner_tilt_grip}",
    )

    return ctx.report()


object_model = build_object_model()
