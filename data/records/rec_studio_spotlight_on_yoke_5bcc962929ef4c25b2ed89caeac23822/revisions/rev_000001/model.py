from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _make_yoke_frame() -> cq.Workplane:
    outer_width = 0.38
    inner_width = 0.32
    depth = 0.055
    bridge_bottom = 0.03
    bridge_height = 0.035
    frame_top = 0.32
    tilt_axis_z = 0.22
    tilt_bore_radius = 0.018

    outer_profile = (
        cq.Workplane("YZ")
        .center(0.0, bridge_bottom + (frame_top - bridge_bottom) / 2.0)
        .rect(outer_width, frame_top - bridge_bottom)
        .extrude(depth)
        .translate((-depth / 2.0, 0.0, 0.0))
    )
    inner_cut = (
        cq.Workplane("YZ")
        .center(0.0, bridge_bottom + bridge_height + (frame_top - (bridge_bottom + bridge_height)) / 2.0)
        .rect(inner_width, frame_top - (bridge_bottom + bridge_height))
        .extrude(depth + 0.01)
        .translate((-(depth + 0.01) / 2.0, 0.0, 0.0))
    )
    frame = outer_profile.cut(inner_cut)

    trunnion_bore = (
        cq.Workplane("XZ")
        .center(0.0, tilt_axis_z)
        .circle(tilt_bore_radius)
        .extrude(outer_width + 0.02)
        .translate((0.0, -(outer_width + 0.02) / 2.0, 0.0))
    )
    return frame.cut(trunnion_bore)


def _make_lamp_can() -> cq.Workplane:
    body_radius = 0.12
    wall = 0.008
    body_length = 0.32
    body_center_x = 0.02
    rear_cap = 0.03

    outer_shell = (
        cq.Workplane("YZ")
        .circle(body_radius)
        .extrude(body_length)
        .translate((body_center_x - body_length / 2.0, 0.0, 0.0))
    )
    inner_cut = (
        cq.Workplane("YZ")
        .circle(body_radius - wall)
        .extrude(body_length - rear_cap)
        .translate((body_center_x - body_length / 2.0 + rear_cap, 0.0, 0.0))
    )
    rear_service_cap = (
        cq.Workplane("YZ")
        .circle(0.055)
        .extrude(0.018)
        .translate((body_center_x - body_length / 2.0 - 0.018, 0.0, 0.0))
    )
    return outer_shell.cut(inner_cut).union(rear_service_cap)


def _make_front_bezel() -> cq.Workplane:
    outer_radius = 0.128
    inner_radius = 0.106
    bezel_length = 0.028
    bezel_start_x = 0.18

    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(bezel_length)
        .translate((bezel_start_x, 0.0, 0.0))
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_spotlight_yoke_stand")

    stand_black = model.material("stand_black", rgba=(0.08, 0.08, 0.09, 1.0))
    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    hardware_gray = model.material("hardware_gray", rgba=(0.22, 0.22, 0.24, 1.0))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=0.18, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=stand_black,
        name="base_disk",
    )
    stand.visual(
        Cylinder(radius=0.032, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.355)),
        material=stand_black,
        name="column",
    )
    stand.visual(
        Cylinder(radius=0.05, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.6925)),
        material=hardware_gray,
        name="top_collar",
    )
    stand.visual(
        Cylinder(radius=0.07, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.7275)),
        material=hardware_gray,
        name="top_plate",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.06, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=hardware_gray,
        name="pan_hub",
    )
    yoke.visual(
        mesh_from_cadquery(_make_yoke_frame(), "yoke_frame"),
        material=stand_black,
        name="yoke_frame",
    )

    lamp = model.part("lamp")
    lamp.visual(
        mesh_from_cadquery(_make_lamp_can(), "lamp_can"),
        material=body_black,
        name="can_body",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.078),
        origin=Origin(xyz=(0.0, 0.121, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_gray,
        name="trunnion_0",
    )
    lamp.visual(
        Cylinder(radius=0.018, length=0.078),
        origin=Origin(xyz=(0.0, -0.121, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hardware_gray,
        name="trunnion_1",
    )
    lamp.visual(
        mesh_from_cadquery(_make_front_bezel(), "front_bezel"),
        material=hardware_gray,
        name="front_bezel",
    )

    model.articulation(
        "stand_to_yoke",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "yoke_to_lamp",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.0,
            lower=-math.radians(50.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("stand_to_yoke")
    tilt = object_model.get_articulation("yoke_to_lamp")

    ctx.expect_contact(
        yoke,
        stand,
        elem_a="pan_hub",
        elem_b="top_plate",
        name="pan hub seats on stand top plate",
    )
    ctx.allow_overlap(
        lamp,
        yoke,
        elem_a="trunnion_0",
        elem_b="yoke_frame",
        reason="The lamp's tilt trunnion is intentionally seated in the yoke cheek at the pivot axis.",
    )
    ctx.allow_overlap(
        lamp,
        yoke,
        elem_a="trunnion_1",
        elem_b="yoke_frame",
        reason="The opposite tilt trunnion is intentionally seated in the yoke cheek at the pivot axis.",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="y",
        inner_elem="can_body",
        outer_elem="yoke_frame",
        margin=0.003,
        name="lamp can stays between yoke arms",
    )

    rest_front = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_bezel"))
    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    pan_quarter = math.pi / 2.0

    with ctx.pose({tilt: tilt_upper}):
        tilted_front = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_bezel"))

    ctx.check(
        "tilt raises the lamp nose",
        rest_front is not None
        and tilted_front is not None
        and tilt_upper is not None
        and tilted_front[2] > rest_front[2] + 0.12
        and tilted_front[0] < rest_front[0] - 0.03,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}, tilt_upper={tilt_upper}",
    )

    with ctx.pose({pan: pan_quarter}):
        panned_front = _aabb_center(ctx.part_element_world_aabb(lamp, elem="front_bezel"))

    ctx.check(
        "pan swings the lamp around the stand axis",
        rest_front is not None
        and panned_front is not None
        and panned_front[1] > rest_front[1] + 0.14
        and abs(panned_front[0]) < rest_front[0] * 0.4,
        details=f"rest_front={rest_front}, panned_front={panned_front}",
    )

    return ctx.report()


object_model = build_object_model()
