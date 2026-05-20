from __future__ import annotations

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


BASE_PLATE_LENGTH = 0.068
BASE_PLATE_WIDTH = 0.052
BASE_PLATE_THICKNESS = 0.006

BODY_RADIUS = 0.022
BODY_HEIGHT = 0.036
SHOULDER_X = 0.020
SHOULDER_Z = 0.044
SPOUT_RADIUS = 0.014
SPOUT_LENGTH = 0.084
SPOUT_X = 0.028
SPOUT_Z = 0.060

HANDLE_PIVOT_X = 0.002
HANDLE_PIVOT_Y = 0.029
HANDLE_PIVOT_Z = 0.033

NOZZLE_HINGE_X = 0.119
NOZZLE_HINGE_Z = 0.060


def _body_shape() -> cq.Workplane:
    base_plate = (
        cq.Workplane("XY")
        .box(BASE_PLATE_LENGTH, BASE_PLATE_WIDTH, BASE_PLATE_THICKNESS)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS / 2.0))
        .edges("|Z")
        .fillet(0.0038)
    )

    body_core = (
        cq.Workplane("XY")
        .circle(BODY_RADIUS)
        .extrude(BODY_HEIGHT)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS))
    )

    shoulder = (
        cq.Workplane("XY")
        .box(0.040, 0.040, 0.022)
        .translate((SHOULDER_X, 0.0, SHOULDER_Z))
        .edges("|Z")
        .fillet(0.006)
    )

    spout = (
        cq.Workplane("YZ")
        .circle(SPOUT_RADIUS)
        .extrude(SPOUT_LENGTH)
        .translate((SPOUT_X, 0.0, SPOUT_Z))
    )

    handle_mount = (
        cq.Workplane("YZ")
        .circle(0.009)
        .extrude(0.014)
        .translate((HANDLE_PIVOT_X - 0.007, HANDLE_PIVOT_Y - 0.009, HANDLE_PIVOT_Z))
    )

    lug_0 = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.012)
        .translate((NOZZLE_HINGE_X - 0.004, -0.007, NOZZLE_HINGE_Z))
    )
    lug_1 = (
        cq.Workplane("XY")
        .box(0.010, 0.004, 0.012)
        .translate((NOZZLE_HINGE_X - 0.004, 0.007, NOZZLE_HINGE_Z))
    )

    return (
        base_plate.union(body_core)
        .union(shoulder)
        .union(spout)
        .union(handle_mount)
        .union(lug_0)
        .union(lug_1)
    )


def _handle_shape() -> cq.Workplane:
    hub = (
        cq.Workplane("YZ")
        .circle(0.0065)
        .extrude(0.012)
        .translate((-0.006, 0.0065, 0.0))
    )
    root = cq.Workplane("XY").box(0.012, 0.014, 0.010).translate((0.0, 0.007, 0.0))
    lever = cq.Workplane("XY").box(0.012, 0.050, 0.008).translate((0.0, 0.033, 0.004))
    fingertip = cq.Workplane("XY").box(0.014, 0.012, 0.010).translate((0.0, 0.060, 0.006))

    return hub.union(root).union(lever).union(fingertip).rotate((0, 0, 0), (1, 0, 0), 18)


def _nozzle_shape() -> cq.Workplane:
    bridge = cq.Workplane("XY").box(0.010, 0.010, 0.007).translate((0.006, 0.0, -0.004))
    head = (
        cq.Workplane("YZ")
        .circle(0.0085)
        .extrude(0.017)
        .faces(">X")
        .workplane()
        .circle(0.0045)
        .cutBlind(-0.002)
        .translate((0.007, 0.0, -0.005))
    )
    return bridge.union(head)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bidet_faucet")

    chrome = model.material("chrome", rgba=(0.84, 0.85, 0.87, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shape(), "body_shell"),
        material=chrome,
        name="body_shell",
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shape(), "handle_shell"),
        material=chrome,
        name="handle_shell",
    )

    nozzle = model.part("nozzle")
    nozzle.visual(
        mesh_from_cadquery(_nozzle_shape(), "nozzle_shell"),
        material=chrome,
        name="nozzle_shell",
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=8.0, velocity=2.0),
    )
    model.articulation(
        "body_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=nozzle,
        origin=Origin(xyz=(NOZZLE_HINGE_X, 0.0, NOZZLE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.35, upper=0.55, effort=2.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    nozzle = object_model.get_part("nozzle")
    handle_joint = object_model.get_articulation("body_to_handle")
    nozzle_joint = object_model.get_articulation("body_to_nozzle")

    ctx.expect_origin_gap(
        handle,
        body,
        axis="y",
        min_gap=0.020,
        max_gap=0.040,
        name="handle pivot sits on the faucet side",
    )
    ctx.expect_overlap(
        handle,
        body,
        axes="xz",
        min_overlap=0.010,
        name="handle pivot aligns with the body shoulder",
    )
    ctx.expect_origin_gap(
        nozzle,
        body,
        axis="x",
        min_gap=0.110,
        max_gap=0.125,
        name="nozzle hinge sits at the spout tip",
    )
    ctx.expect_overlap(
        nozzle,
        body,
        axes="yz",
        min_overlap=0.010,
        name="nozzle stays aligned to the spout outlet",
    )

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_shell")
    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None
    with ctx.pose({handle_joint: handle_upper}):
        handle_lifted = ctx.part_element_world_aabb(handle, elem="handle_shell")
    ctx.check(
        "side handle lifts upward on its pivot",
        handle_rest is not None
        and handle_lifted is not None
        and handle_lifted[1][2] > handle_rest[1][2] + 0.010,
        details=f"rest={handle_rest}, lifted={handle_lifted}",
    )

    nozzle_rest = ctx.part_element_world_aabb(nozzle, elem="nozzle_shell")
    nozzle_upper = nozzle_joint.motion_limits.upper if nozzle_joint.motion_limits is not None else None
    with ctx.pose({nozzle_joint: nozzle_upper}):
        nozzle_tilted = ctx.part_element_world_aabb(nozzle, elem="nozzle_shell")
    ctx.check(
        "tip nozzle swivels downward around its horizontal hinge",
        nozzle_rest is not None
        and nozzle_tilted is not None
        and nozzle_tilted[0][2] < nozzle_rest[0][2] - 0.004,
        details=f"rest={nozzle_rest}, tilted={nozzle_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
