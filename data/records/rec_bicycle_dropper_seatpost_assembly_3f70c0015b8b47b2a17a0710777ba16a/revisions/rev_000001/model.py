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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_tube(outer_radius: float, inner_radius: float, length: float) -> cq.Workplane:
    """Hollow round tube with open bore along local +Z, authored in meters."""
    return cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wireless_dropper_seatpost")

    dark_anodized = model.material("dark_anodized", rgba=(0.015, 0.016, 0.017, 1.0))
    satin_black = model.material("satin_black", rgba=(0.02, 0.022, 0.025, 1.0))
    frame_gray = model.material("frame_gray", rgba=(0.12, 0.13, 0.14, 1.0))
    polished = model.material("polished_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    stanchion = model.material("black_stanchion", rgba=(0.03, 0.032, 0.034, 1.0))
    blue_led = model.material("blue_status_led", rgba=(0.0, 0.35, 1.0, 1.0))

    outer_tube = model.part("outer_tube")

    # Bike frame references: a hollow seat tube and clamp show the dropper
    # inserted into and clamped by the frame, while also carrying the handlebar
    # remote so the wireless assembly is not floating.
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.023, 0.0168, 0.38), "frame_seat_tube"),
        origin=Origin(xyz=(0.0, 0.0, -0.11)),
        material=frame_gray,
        name="frame_seat_tube",
    )
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.027, 0.0155, 0.044), "seat_clamp_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=satin_black,
        name="seat_clamp_collar",
    )
    outer_tube.visual(
        Box((0.034, 0.014, 0.030)),
        origin=Origin(xyz=(0.0, -0.030, 0.267)),
        material=satin_black,
        name="clamp_split_lug",
    )
    outer_tube.visual(
        Cylinder(radius=0.0035, length=0.052),
        origin=Origin(xyz=(0.0, -0.037, 0.267), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polished,
        name="clamp_bolt",
    )

    # The stationary lower post is a true hollow round sleeve.  Its through bore
    # gives the moving inner stanchion real radial clearance instead of relying
    # on an overlap allowance.
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.0159, 0.0134, 0.34), "outer_sleeve_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=dark_anodized,
        name="outer_sleeve_shell",
    )
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.0190, 0.0128, 0.018), "wiper_seal"),
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        material=satin_black,
        name="wiper_seal",
    )
    outer_tube.visual(
        Box((0.018, 0.010, 0.074)),
        origin=Origin(xyz=(-0.019, 0.0, 0.255)),
        material=satin_black,
        name="electronics_pod",
    )
    outer_tube.visual(
        Sphere(radius=0.0025),
        origin=Origin(xyz=(-0.024, 0.0, 0.277)),
        material=blue_led,
        name="status_led",
    )

    # Simplified bicycle frame/handlebar support for the wireless remote.
    outer_tube.visual(
        Cylinder(radius=0.014, length=0.55),
        origin=Origin(xyz=(0.310, 0.0, 0.220), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_gray,
        name="top_tube",
    )
    outer_tube.visual(
        Box((0.052, 0.028, 0.028)),
        origin=Origin(xyz=(0.045, 0.0, 0.220)),
        material=frame_gray,
        name="top_tube_gusset",
    )
    outer_tube.visual(
        Cylinder(radius=0.018, length=0.18),
        origin=Origin(xyz=(0.55, 0.0, 0.310)),
        material=frame_gray,
        name="head_tube",
    )
    outer_tube.visual(
        Cylinder(radius=0.011, length=0.19),
        origin=Origin(xyz=(0.55, 0.0, 0.475)),
        material=frame_gray,
        name="stem",
    )
    outer_tube.visual(
        Cylinder(radius=0.011, length=0.62),
        origin=Origin(xyz=(0.55, 0.0, 0.570), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=frame_gray,
        name="handlebar",
    )
    outer_tube.visual(
        mesh_from_cadquery(_annular_tube(0.017, 0.0106, 0.026), "remote_bar_clamp"),
        origin=Origin(xyz=(0.55, -0.193, 0.570), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="remote_bar_clamp",
    )
    outer_tube.visual(
        Box((0.042, 0.024, 0.018)),
        origin=Origin(xyz=(0.538, -0.180, 0.548)),
        material=satin_black,
        name="remote_body",
    )
    for i, y in enumerate((-0.1955, -0.1645)):
        outer_tube.visual(
            Box((0.024, 0.012, 0.024)),
            origin=Origin(xyz=(0.510, y, 0.536)),
            material=satin_black,
            name=f"remote_yoke_{i}",
        )
    outer_tube.visual(
        Cylinder(radius=0.0045, length=0.030),
        origin=Origin(xyz=(0.505, -0.180, 0.530), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=polished,
        name="remote_pivot_pin",
    )

    inner_tube = model.part("inner_tube")
    inner_tube.visual(
        Cylinder(radius=0.0121, length=0.510),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=stanchion,
        name="inner_stanchion",
    )
    inner_tube.visual(
        Cylinder(radius=0.017, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark_anodized,
        name="saddle_clamp_head",
    )
    inner_tube.visual(
        Box((0.052, 0.034, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.318)),
        material=dark_anodized,
        name="rail_cradle",
    )
    for y in (-0.014, 0.014):
        inner_tube.visual(
            Cylinder(radius=0.0027, length=0.150),
            origin=Origin(xyz=(0.0, y, 0.327), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=polished,
            name=f"saddle_rail_{0 if y < 0 else 1}",
        )
    inner_tube.visual(
        Box((0.070, 0.050, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=satin_black,
        name="saddle_base_pad",
    )

    remote_lever = model.part("remote_lever")
    remote_lever.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="lever_boss",
    )
    remote_lever.visual(
        Box((0.018, 0.014, 0.016)),
        origin=Origin(xyz=(-0.010, 0.0, -0.006)),
        material=satin_black,
        name="lever_root_web",
    )
    remote_lever.visual(
        Box((0.074, 0.012, 0.006)),
        origin=Origin(xyz=(-0.047, 0.0, -0.014)),
        material=satin_black,
        name="lever_blade",
    )
    remote_lever.visual(
        Box((0.028, 0.018, 0.008)),
        origin=Origin(xyz=(-0.085, 0.0, -0.015)),
        material=satin_black,
        name="thumb_paddle",
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_tube,
        origin=Origin(xyz=(0.0, 0.0, 0.403)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.35, lower=0.0, upper=0.125),
    )
    model.articulation(
        "remote_hinge",
        ArticulationType.REVOLUTE,
        parent=outer_tube,
        child=remote_lever,
        origin=Origin(xyz=(0.505, -0.180, 0.530)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=6.0, lower=0.0, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_tube")
    inner = object_model.get_part("inner_tube")
    lever = object_model.get_part("remote_lever")
    dropper = object_model.get_articulation("outer_to_inner")
    hinge = object_model.get_articulation("remote_hinge")

    ctx.allow_overlap(
        outer,
        lever,
        elem_a="remote_pivot_pin",
        elem_b="lever_boss",
        reason="The lever boss is intentionally captured on the remote hinge pin.",
    )
    ctx.expect_overlap(
        outer,
        lever,
        axes="y",
        elem_a="remote_pivot_pin",
        elem_b="lever_boss",
        min_overlap=0.010,
        name="remote hinge pin spans lever boss",
    )

    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem="inner_stanchion",
        outer_elem="outer_sleeve_shell",
        margin=0.0,
        name="inner stanchion centered inside outer sleeve footprint",
    )
    ctx.expect_overlap(
        inner,
        outer,
        axes="z",
        elem_a="inner_stanchion",
        elem_b="outer_sleeve_shell",
        min_overlap=0.18,
        name="extended post retains insertion",
    )

    rest_pos = ctx.part_world_position(inner)
    with ctx.pose({dropper: 0.125}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem="inner_stanchion",
            outer_elem="outer_sleeve_shell",
            margin=0.0,
            name="dropped stanchion stays centered in sleeve",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="z",
            elem_a="inner_stanchion",
            elem_b="outer_sleeve_shell",
            min_overlap=0.30,
            name="dropped post keeps hidden bushing engagement",
        )
        dropped_pos = ctx.part_world_position(inner)

    ctx.check(
        "inner tube telescopes downward",
        rest_pos is not None and dropped_pos is not None and dropped_pos[2] < rest_pos[2] - 0.10,
        details=f"rest={rest_pos}, dropped={dropped_pos}",
    )

    rest_lever = ctx.part_world_aabb(lever)
    with ctx.pose({hinge: 0.55}):
        pulled_lever = ctx.part_world_aabb(lever)
    ctx.check(
        "remote lever pivots upward toward bar",
        rest_lever is not None
        and pulled_lever is not None
        and pulled_lever[1][2] > rest_lever[1][2] + 0.015,
        details=f"rest={rest_lever}, pulled={pulled_lever}",
    )

    return ctx.report()


object_model = build_object_model()
