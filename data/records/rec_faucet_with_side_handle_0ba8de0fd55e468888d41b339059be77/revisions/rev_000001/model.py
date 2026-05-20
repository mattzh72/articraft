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


def _build_spout_mesh():
    return tube_from_spline_points(
        [
            (0.0, 0.0, 0.018),
            (0.0, 0.0, 0.145),
            (0.012, 0.0, 0.255),
            (0.095, 0.0, 0.352),
            (0.198, 0.0, 0.346),
            (0.246, 0.0, 0.294),
        ],
        radius=0.0175,
        samples_per_segment=20,
        radial_segments=24,
        cap_ends=True,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_corner, max_corner) = aabb
    return tuple((low + high) * 0.5 for low, high in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_faucet")

    chrome = model.material("chrome", rgba=(0.78, 0.80, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.69, 0.72, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=chrome,
        name="deck_flange",
    )
    base.visual(
        Cylinder(radius=0.025, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.027)),
        material=steel,
        name="base_column",
    )

    spout = model.part("spout")
    spout.visual(
        Cylinder(radius=0.029, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=chrome,
        name="spout_neck",
    )
    spout.visual(
        mesh_from_geometry(_build_spout_mesh(), "faucet_spout"),
        material=chrome,
        name="spout_arch",
    )
    spout.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(
            xyz=(0.248, 0.0, 0.293),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=dark_trim,
        name="aerator",
    )

    side_handle = model.part("side_handle")
    side_handle.visual(
        Cylinder(radius=0.009, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="pivot_barrel",
    )
    side_handle.visual(
        Box((0.012, 0.038, 0.012)),
        origin=Origin(xyz=(0.0, 0.024, 0.003), rpy=(-0.34, 0.0, 0.0)),
        material=chrome,
        name="handle_arm",
    )
    side_handle.visual(
        Box((0.010, 0.056, 0.008)),
        origin=Origin(xyz=(0.0, 0.056, -0.002), rpy=(-0.24, 0.0, 0.0)),
        material=graphite,
        name="handle_paddle",
    )

    filter_knob = model.part("filter_knob")
    filter_knob.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="knob_shaft",
    )
    filter_knob.visual(
        Cylinder(radius=0.013, length=0.014),
        origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="knob_body",
    )
    filter_knob.visual(
        Box((0.0025, 0.009, 0.004)),
        origin=Origin(xyz=(0.0, -0.017, 0.009)),
        material=dark_trim,
        name="knob_indicator",
    )

    model.articulation(
        "base_to_spout",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.5),
    )
    model.articulation(
        "spout_to_side_handle",
        ArticulationType.REVOLUTE,
        parent=spout,
        child=side_handle,
        origin=Origin(xyz=(0.0, 0.038, 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.2,
            lower=0.0,
            upper=0.95,
        ),
    )
    model.articulation(
        "spout_to_filter_knob",
        ArticulationType.CONTINUOUS,
        parent=spout,
        child=filter_knob,
        origin=Origin(xyz=(0.0, -0.017, 0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spout = object_model.get_part("spout")
    side_handle = object_model.get_part("side_handle")
    filter_knob = object_model.get_part("filter_knob")

    spout_joint = object_model.get_articulation("base_to_spout")
    handle_joint = object_model.get_articulation("spout_to_side_handle")
    knob_joint = object_model.get_articulation("spout_to_filter_knob")

    ctx.check(
        "spout rotates continuously",
        spout_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spout_joint.articulation_type}",
    )
    ctx.check(
        "filter knob rotates continuously",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )
    ctx.expect_origin_distance(
        spout,
        base,
        axes="xy",
        max_dist=0.001,
        name="spout stays centered on the base axis",
    )
    ctx.expect_overlap(
        spout,
        base,
        axes="xy",
        min_overlap=0.040,
        name="spout collar covers the rotating base axis",
    )
    ctx.expect_origin_distance(
        filter_knob,
        spout,
        axes="y",
        min_dist=0.015,
        max_dist=0.019,
        name="filter knob sits on the spout side",
    )

    handle_rest = ctx.part_world_aabb(side_handle)
    with ctx.pose({handle_joint: 0.85}):
        handle_open = ctx.part_world_aabb(side_handle)
    ctx.check(
        "handle lifts upward",
        handle_rest is not None
        and handle_open is not None
        and handle_open[1][2] > handle_rest[1][2] + 0.020,
        details=f"rest={handle_rest}, open={handle_open}",
    )

    handle_rest_pos = ctx.part_world_position(side_handle)
    with ctx.pose({spout_joint: math.pi / 2.0}):
        handle_turned_pos = ctx.part_world_position(side_handle)
        knob_turned_pos = ctx.part_world_position(filter_knob)
    ctx.check(
        "spout swings attached controls around the base axis",
        handle_rest_pos is not None
        and handle_turned_pos is not None
        and abs(handle_turned_pos[0]) > 0.020
        and abs(handle_turned_pos[1]) < abs(handle_rest_pos[1]) * 0.35,
        details=f"rest={handle_rest_pos}, turned={handle_turned_pos}",
    )
    ctx.check(
        "filter knob follows the spout rotation",
        knob_turned_pos is not None and abs(knob_turned_pos[0]) > 0.010,
        details=f"turned={knob_turned_pos}",
    )

    spout_rest_center = _aabb_center(ctx.part_world_aabb(spout))
    with ctx.pose({spout_joint: math.pi}):
        spout_turned_center = _aabb_center(ctx.part_world_aabb(spout))
    ctx.check(
        "spout sweep changes heading when rotated",
        spout_rest_center is not None
        and spout_turned_center is not None
        and spout_rest_center[0] > 0.050
        and spout_turned_center[0] < -0.050,
        details=f"rest={spout_rest_center}, turned={spout_turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
