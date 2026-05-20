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


def _centered_cylinder_z(radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(length).translate((0.0, 0.0, -length / 2.0))


def _centered_cylinder_x(radius: float, length: float) -> cq.Workplane:
    return _centered_cylinder_z(radius, length).rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)


def _centered_cylinder_y(radius: float, length: float) -> cq.Workplane:
    return _centered_cylinder_z(radius, length).rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)


def _build_body_shape() -> cq.Workplane:
    flange = _centered_cylinder_z(0.031, 0.008).translate((0.0, 0.0, 0.004))
    collar = _centered_cylinder_z(0.024, 0.056).translate((0.0, 0.0, 0.036))
    neck = _centered_cylinder_z(0.0155, 0.016).translate((0.0, 0.0, 0.072))
    swivel_post = _centered_cylinder_z(0.0142, 0.030).translate((0.0, 0.0, 0.095))
    thrust_ring = _centered_cylinder_z(0.0178, 0.0010).translate((0.0, 0.0, 0.0815))
    lever_pad = _centered_cylinder_y(0.0062, 0.0040).translate((0.0, 0.0270, 0.071))
    lever_bridge = cq.Workplane("XY").box(0.016, 0.013, 0.014).translate((0.0, 0.0215, 0.071))

    body = flange.union(collar).union(neck).union(swivel_post).union(thrust_ring).union(lever_pad).union(lever_bridge)

    rear_flat_cut = cq.Workplane("XY").box(0.034, 0.060, 0.060).translate((-0.035, 0.0, 0.041))
    cover_recess = cq.Workplane("XY").box(0.007, 0.019, 0.030).translate((-0.0155, 0.0, 0.043))

    body = body.cut(rear_flat_cut).cut(cover_recess)

    upper_bridge = cq.Workplane("XY").box(0.004, 0.019, 0.004).translate((-0.0170, 0.0, 0.059))
    ear_a = _centered_cylinder_y(0.0024, 0.0040).translate((-0.0190, -0.0055, 0.0585))
    ear_b = _centered_cylinder_y(0.0024, 0.0040).translate((-0.0190, 0.0055, 0.0585))
    ear_pad_a = cq.Workplane("XY").box(0.004, 0.004, 0.006).translate((-0.0170, -0.0055, 0.0555))
    ear_pad_b = cq.Workplane("XY").box(0.004, 0.004, 0.006).translate((-0.0170, 0.0055, 0.0555))

    return body.union(upper_bridge).union(ear_a).union(ear_b).union(ear_pad_a).union(ear_pad_b)


def _build_spout_shape() -> cq.Workplane:
    sleeve_outer = _centered_cylinder_z(0.0180, 0.0300)
    sleeve_inner = _centered_cylinder_z(0.0142, 0.0306)
    sleeve = sleeve_outer.cut(sleeve_inner)

    arm = (
        cq.Workplane("XY")
        .box(0.142, 0.026, 0.012)
        .edges("|X")
        .fillet(0.0042)
        .translate((0.084, 0.0, 0.003))
    )
    root_block = (
        cq.Workplane("XY")
        .box(0.040, 0.028, 0.016)
        .edges("|X")
        .fillet(0.0045)
        .translate((0.026, 0.0, 0.002))
    )
    outlet_slot = cq.Workplane("XY").box(0.012, 0.007, 0.004).translate((0.149, 0.0, -0.002))
    socket_clearance = _centered_cylinder_z(0.0147, 0.0340)

    return sleeve.union(root_block).union(arm).cut(socket_clearance).cut(outlet_slot)


def _build_lever_shape() -> cq.Workplane:
    cap = _centered_cylinder_y(0.0062, 0.0030).translate((0.0, 0.0015, 0.0))
    blade = (
        cq.Workplane("XY")
        .box(0.010, 0.036, 0.005)
        .edges("|Y")
        .fillet(0.0018)
        .translate((0.0, 0.018, 0.0065))
    )
    stem = blade.rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -20.0)

    return cap.union(stem)


def _build_cover_shape() -> cq.Workplane:
    panel = (
        cq.Workplane("XY")
        .box(0.0028, 0.014, 0.022)
        .edges("|X")
        .fillet(0.0016)
        .translate((0.0014, 0.0, -0.011))
    )
    knuckle = _centered_cylinder_y(0.0021, 0.0060)
    return panel.union(knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_vanity_faucet")

    chrome = model.material("chrome", rgba=(0.79, 0.81, 0.83, 1.0))
    control = model.material("control", rgba=(0.67, 0.69, 0.72, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_build_body_shape(), "body_shell"), material=chrome, name="body_shell")

    spout = model.part("spout")
    spout.visual(mesh_from_cadquery(_build_spout_shape(), "spout_shell"), material=chrome, name="spout_shell")

    lever = model.part("lever")
    lever.visual(mesh_from_cadquery(_build_lever_shape(), "lever_shell"), material=control, name="lever_shell")

    cover = model.part("cover")
    cover.visual(mesh_from_cadquery(_build_cover_shape(), "cover_shell"), material=chrome, name="cover_shell")

    model.articulation(
        "body_to_spout",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=spout,
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0),
    )
    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0290, 0.071)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=0.0, upper=0.95),
    )
    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(-0.0190, 0.0, 0.0585)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=0.0, upper=1.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    spout = object_model.get_part("spout")
    lever = object_model.get_part("lever")
    cover = object_model.get_part("cover")

    spout_joint = object_model.get_articulation("body_to_spout")
    lever_joint = object_model.get_articulation("body_to_lever")
    cover_joint = object_model.get_articulation("body_to_cover")

    ctx.allow_overlap(
        body,
        spout,
        reason="The rotating spout sleeve is intentionally modeled as a nested swivel bearing around the faucet spindle and thrust ring.",
    )

    ctx.expect_origin_distance(
        spout,
        body,
        axes="xy",
        max_dist=0.001,
        name="spout stays centered on the faucet body axis",
    )
    ctx.expect_origin_gap(
        spout,
        body,
        axis="z",
        min_gap=0.085,
        max_gap=0.105,
        name="spout swivel axis sits at vanity faucet height",
    )
    ctx.expect_origin_gap(
        lever,
        body,
        axis="y",
        min_gap=0.026,
        max_gap=0.034,
        name="lever is mounted off the faucet side",
    )
    ctx.expect_origin_gap(
        body,
        cover,
        axis="x",
        min_gap=0.016,
        max_gap=0.024,
        name="maintenance cover is positioned on the rear collar face",
    )
    ctx.expect_within(
        cover,
        body,
        axes="yz",
        margin=0.006,
        name="closed maintenance cover stays within the collar footprint",
    )

    rest_spout_aabb = ctx.part_world_aabb(spout)
    with ctx.pose({spout_joint: math.pi / 2.0}):
        turned_spout_aabb = ctx.part_world_aabb(spout)
    ctx.check(
        "spout rotates toward the side",
        rest_spout_aabb is not None
        and turned_spout_aabb is not None
        and turned_spout_aabb[1][1] > rest_spout_aabb[1][1] + 0.090,
        details=f"rest={rest_spout_aabb}, turned={turned_spout_aabb}",
    )

    rest_lever_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({lever_joint: 0.95}):
        lifted_lever_aabb = ctx.part_world_aabb(lever)
    ctx.check(
        "lever lifts upward on its side pivot",
        rest_lever_aabb is not None
        and lifted_lever_aabb is not None
        and lifted_lever_aabb[1][2] > rest_lever_aabb[1][2] + 0.012,
        details=f"rest={rest_lever_aabb}, lifted={lifted_lever_aabb}",
    )

    rest_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_joint: 1.2}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "rear cover swings outward from the collar",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][0] < rest_cover_aabb[0][0] - 0.008,
        details=f"rest={rest_cover_aabb}, open={open_cover_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
