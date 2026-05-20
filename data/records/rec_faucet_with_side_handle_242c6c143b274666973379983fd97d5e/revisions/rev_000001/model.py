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


def _build_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(0.028).extrude(0.006)
    column = cq.Workplane("XY").circle(0.022).extrude(0.042)

    rear_blend = cq.Workplane("XY").sphere(0.017).translate((0.0, 0.0, 0.034))
    spout_root = (
        cq.Workplane("YZ")
        .circle(0.0108)
        .extrude(0.036)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -28.0)
        .translate((0.004, 0.0, 0.034))
    )
    elbow_blend = cq.Workplane("XY").sphere(0.011).translate((0.036, 0.0, 0.051))
    spout_run = cq.Workplane("YZ").circle(0.0098).extrude(0.056).translate((0.036, 0.0, 0.051))
    tip_neck = cq.Workplane("YZ").circle(0.0085).extrude(0.014).translate((0.092, 0.0, 0.051))

    handle_pad = (
        cq.Workplane("XZ")
        .center(0.0, 0.033)
        .circle(0.0075)
        .extrude(0.005)
        .translate((0.0, 0.019, 0.0))
    )
    handle_knuckle_0 = (
        cq.Workplane("YZ")
        .center(0.0262, 0.033)
        .circle(0.0046)
        .extrude(0.0015, both=True)
        .translate((-0.0032, 0.0, 0.0))
    )
    handle_knuckle_1 = (
        cq.Workplane("YZ")
        .center(0.0262, 0.033)
        .circle(0.0046)
        .extrude(0.0015, both=True)
        .translate((0.0032, 0.0, 0.0))
    )

    tip_fork_0 = (
        cq.Workplane("XZ")
        .center(0.109, 0.051)
        .circle(0.003)
        .extrude(0.00125, both=True)
        .translate((0.0, -0.0042, 0.0))
    )
    tip_fork_1 = (
        cq.Workplane("XZ")
        .center(0.109, 0.051)
        .circle(0.003)
        .extrude(0.00125, both=True)
        .translate((0.0, 0.0042, 0.0))
    )
    tip_strut_0 = cq.Workplane("XY").box(0.0045, 0.0026, 0.0052).translate((0.1054, -0.0042, 0.051))
    tip_strut_1 = cq.Workplane("XY").box(0.0045, 0.0026, 0.0052).translate((0.1054, 0.0042, 0.051))

    body = (
        base.union(column)
        .union(rear_blend)
        .union(spout_root)
        .union(elbow_blend)
        .union(spout_run)
        .union(tip_neck)
        .union(handle_pad)
        .union(handle_knuckle_0)
        .union(handle_knuckle_1)
        .union(tip_strut_0)
        .union(tip_strut_1)
        .union(tip_fork_0)
        .union(tip_fork_1)
    )

    return body


def _build_lever_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("YZ")
        .circle(0.005)
        .extrude(0.00185, both=True)
        .intersect(cq.Workplane("XY").box(0.02, 0.02, 0.02).translate((0.0, 0.01, 0.0)))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.009, 0.015, 0.0056)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 18.0)
        .translate((0.0, 0.0102, 0.0048))
    )
    arm = (
        cq.Workplane("XY")
        .box(0.010, 0.050, 0.005)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 18.0)
        .translate((0.0, 0.036, 0.011))
    )
    paddle = (
        cq.Workplane("XY")
        .box(0.012, 0.024, 0.0038)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 18.0)
        .translate((0.0, 0.059, 0.018))
    )
    return hub.union(neck).union(arm).union(paddle)


def _build_nozzle_shape() -> cq.Workplane:
    barrel = cq.Workplane("XZ").circle(0.0028).extrude(0.00235, both=True)
    neck = (
        cq.Workplane("YZ")
        .circle(0.0046)
        .extrude(0.008)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)
        .translate((0.0018, 0.0, -0.0008))
    )
    body = (
        cq.Workplane("YZ")
        .circle(0.0072)
        .extrude(0.016)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)
        .translate((0.0064, 0.0, -0.0015))
    )
    front_ring = (
        cq.Workplane("YZ")
        .circle(0.0084)
        .extrude(0.0035)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)
        .translate((0.019, 0.0, -0.0042))
    )
    bore = (
        cq.Workplane("YZ")
        .circle(0.0042)
        .extrude(0.013)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -12.0)
        .translate((0.0135, 0.0, -0.0025))
    )
    return barrel.union(neck).union(body).union(front_ring).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bathroom_faucet")

    chrome = model.material("chrome", rgba=(0.77, 0.79, 0.81, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body"),
        material=chrome,
        name="body_shell",
    )

    lever = model.part("lever")
    lever.visual(
        mesh_from_cadquery(_build_lever_shape(), "lever"),
        material=chrome,
        name="lever_shell",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        mesh_from_cadquery(_build_nozzle_shape(), "nozzle"),
        material=chrome,
        name="nozzle_shell",
    )

    model.articulation(
        "body_to_lever",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lever,
        origin=Origin(xyz=(0.0, 0.0262, 0.033)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(50.0),
        ),
    )

    model.articulation(
        "body_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(0.109, 0.0, 0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=math.radians(-25.0),
            upper=math.radians(35.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lever = object_model.get_part("lever")
    nozzle = object_model.get_part("nozzle")
    lever_joint = object_model.get_articulation("body_to_lever")
    nozzle_joint = object_model.get_articulation("body_to_nozzle")

    ctx.allow_overlap(
        body,
        nozzle,
        reason="The swivel aerator is represented as a compact hinge cartridge nested inside the spout-tip clevis.",
    )

    ctx.expect_contact(
        lever,
        body,
        contact_tol=0.0012,
        name="lever pivot meets body mount",
    )
    ctx.expect_contact(
        nozzle,
        body,
        contact_tol=0.0012,
        name="aerator hinge meets spout tip",
    )

    body_aabb = ctx.part_world_aabb(body)
    body_ok = (
        body_aabb is not None
        and body_aabb[0][2] >= -0.001
        and 0.10 <= body_aabb[1][0] <= 0.12
        and 0.045 <= body_aabb[1][2] <= 0.08
    )
    ctx.check(
        "faucet silhouette matches vanity scale",
        body_ok,
        details=f"body_aabb={body_aabb}",
    )

    lever_rest = ctx.part_world_aabb(lever)
    lever_upper = lever_joint.motion_limits.upper if lever_joint.motion_limits is not None else None
    with ctx.pose({lever_joint: lever_upper if lever_upper is not None else 0.0}):
        lever_open = ctx.part_world_aabb(lever)
    lever_ok = (
        lever_rest is not None
        and lever_open is not None
        and lever_open[1][2] > lever_rest[1][2] + 0.018
    )
    ctx.check(
        "side lever lifts upward",
        lever_ok,
        details=f"rest={lever_rest}, open={lever_open}",
    )

    nozzle_rest = ctx.part_world_aabb(nozzle)
    nozzle_upper = nozzle_joint.motion_limits.upper if nozzle_joint.motion_limits is not None else None
    with ctx.pose({nozzle_joint: nozzle_upper if nozzle_upper is not None else 0.0}):
        nozzle_tilted = ctx.part_world_aabb(nozzle)
    nozzle_ok = (
        nozzle_rest is not None
        and nozzle_tilted is not None
        and nozzle_tilted[0][2] < nozzle_rest[0][2] - 0.003
    )
    ctx.check(
        "aerator nozzle tilts downward",
        nozzle_ok,
        details=f"rest={nozzle_rest}, tilted={nozzle_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
