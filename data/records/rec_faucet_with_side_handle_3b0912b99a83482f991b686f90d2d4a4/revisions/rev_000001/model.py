from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins = aabb[0]
    maxs = aabb[1]
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_faucet")

    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.82, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.87, 0.89, 0.92, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.20, 0.21, 0.23, 1.0))

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.032, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=stainless,
        name="deck_flange",
    )
    body.visual(
        Cylinder(radius=0.029, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=stainless,
        name="base_shell",
    )
    body.visual(
        Cylinder(radius=0.0275, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=stainless,
        name="transition_collar",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.111)),
        material=polished_steel,
        name="body_column",
    )
    body.visual(
        Cylinder(radius=0.031, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.156)),
        material=stainless,
        name="spout_seat",
    )
    body.visual(
        Box((0.014, 0.018, 0.018)),
        origin=Origin(xyz=(0.026, 0.0, 0.072)),
        material=stainless,
        name="side_boss",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_trim,
        name="aerator_core",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stainless,
        name="swivel_collar",
    )
    spout_tube = tube_from_spline_points(
        [
            (0.0, 0.0, 0.004),
            (0.0, 0.0, 0.095),
            (0.0, 0.036, 0.185),
            (0.0, 0.120, 0.285),
            (0.0, 0.198, 0.270),
            (0.0, 0.222, 0.205),
            (0.0, 0.222, 0.142),
        ],
        radius=0.0125,
        samples_per_segment=18,
        radial_segments=22,
        cap_ends=True,
    )
    spout.visual(
        _mesh("gooseneck_tube", spout_tube),
        material=polished_steel,
        name="gooseneck_tube",
    )
    spout.visual(
        Cylinder(radius=0.0145, length=0.028),
        origin=Origin(xyz=(0.0, 0.222, 0.135)),
        material=stainless,
        name="outlet",
    )
    spout.visual(
        Cylinder(radius=0.0085, length=0.005),
        origin=Origin(xyz=(0.0, 0.222, 0.120)),
        material=dark_trim,
        name="aerator",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="pivot_barrel",
    )
    handle_arm = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.020, 0.0, 0.003),
            (0.058, 0.0, 0.010),
            (0.092, 0.0, 0.017),
        ],
        radius=0.0048,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    handle.visual(
        _mesh("handle_arm", handle_arm),
        material=polished_steel,
        name="lever_arm",
    )
    handle.visual(
        Box((0.028, 0.012, 0.007)),
        origin=Origin(xyz=(0.098, 0.0, 0.018)),
        material=stainless,
        name="grip",
    )

    model.articulation(
        "spout_swivel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.5),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.042, 0.0, 0.072)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=0.82,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    handle = object_model.get_part("handle")
    spout_swivel = object_model.get_articulation("spout_swivel")
    handle_pivot = object_model.get_articulation("handle_pivot")

    ctx.expect_gap(
        spout,
        body,
        axis="z",
        positive_elem="swivel_collar",
        negative_elem="spout_seat",
        min_gap=0.0,
        max_gap=0.001,
        name="spout collar seats on the faucet body",
    )
    ctx.expect_gap(
        handle,
        body,
        axis="x",
        positive_elem="pivot_barrel",
        negative_elem="side_boss",
        min_gap=0.0,
        max_gap=0.001,
        name="handle pivot barrel sits against the side boss",
    )

    rest_outlet = _aabb_center(ctx.part_element_world_aabb(spout, elem="outlet"))
    with ctx.pose({spout_swivel: math.pi / 2.0}):
        turned_outlet = _aabb_center(ctx.part_element_world_aabb(spout, elem="outlet"))

    ctx.check(
        "spout rotates around the vertical axis",
        rest_outlet is not None
        and turned_outlet is not None
        and rest_outlet[1] > 0.18
        and abs(rest_outlet[0]) < 0.02
        and turned_outlet[0] < -0.18
        and abs(turned_outlet[1]) < 0.04
        and abs(turned_outlet[2] - rest_outlet[2]) < 0.002,
        details=f"rest_outlet={rest_outlet}, turned_outlet={turned_outlet}",
    )

    rest_grip = _aabb_center(ctx.part_element_world_aabb(handle, elem="grip"))
    handle_upper = handle_pivot.motion_limits.upper if handle_pivot.motion_limits is not None else 0.82
    with ctx.pose({handle_pivot: handle_upper}):
        raised_grip = _aabb_center(ctx.part_element_world_aabb(handle, elem="grip"))

    ctx.check(
        "side handle lifts upward from the closed position",
        rest_grip is not None
        and raised_grip is not None
        and raised_grip[2] > rest_grip[2] + 0.045
        and raised_grip[0] < rest_grip[0] - 0.015,
        details=f"rest_grip={rest_grip}, raised_grip={raised_grip}",
    )

    return ctx.report()


object_model = build_object_model()
