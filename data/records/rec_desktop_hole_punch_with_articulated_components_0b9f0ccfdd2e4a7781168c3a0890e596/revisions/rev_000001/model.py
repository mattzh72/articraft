from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _body_shape() -> cq.Workplane:
    width = 0.064
    length = 0.118
    front_height = 0.020
    rear_height = 0.032

    side_profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-length * 0.50, 0.000),
                (length * 0.50, 0.000),
                (length * 0.50, 0.015),
                (0.042, front_height),
                (-0.010, 0.023),
                (-0.036, 0.028),
                (-length * 0.50, rear_height),
            ]
        )
        .close()
    )
    body = side_profile.extrude(width * 0.50, both=True).edges("|X").fillet(0.003)

    underside_cavity = cq.Workplane("XY").box(0.054, 0.096, 0.021).translate((0.0, 0.004, 0.0125))
    drawer_cavity = cq.Workplane("XY").box(0.051, 0.064, 0.0115).translate((0.007, 0.004, 0.0085))
    body = body.cut(underside_cavity).cut(drawer_cavity)

    body = body.union(cq.Workplane("XY").box(0.042, 0.015, 0.007).translate((0.0, -0.047, 0.030)))
    for x_pos in (-0.028, 0.028):
        tower = cq.Workplane("XY").box(0.008, 0.015, 0.019).translate((x_pos, -0.047, 0.0335))
        body = body.union(tower)

    body = body.union(cq.Workplane("XY").box(0.0045, 0.021, 0.015).translate((0.03125, -0.013, 0.0195)))
    rocker_stub_0 = cq.Workplane("XZ").circle(0.0015).extrude(0.0025).translate((0.0345, -0.018, 0.024))
    rocker_stub_1 = cq.Workplane("XZ").circle(0.0015).extrude(0.0025).translate((0.0345, -0.010, 0.024))
    body = body.union(rocker_stub_0).union(rocker_stub_1)

    return body


def _handle_shape() -> cq.Workplane:
    handle_width = 0.048

    lever_plate = (
        cq.Workplane("YZ")
        .polyline(
            [
                (-0.010, 0.004),
                (0.010, 0.028),
                (0.036, 0.056),
                (0.058, 0.070),
                (0.060, 0.081),
                (0.047, 0.089),
                (0.018, 0.060),
                (-0.010, 0.016),
            ]
        )
        .close()
        .extrude(handle_width * 0.50, both=True)
        .edges("|X")
        .fillet(0.0025)
    )

    hinge_barrel = cq.Workplane("YZ").circle(0.0055).extrude(handle_width * 0.50, both=True)
    front_grip = cq.Workplane("XY").box(handle_width, 0.018, 0.012).translate((0.0, 0.056, 0.080))
    rear_web = cq.Workplane("XY").box(handle_width, 0.010, 0.014).translate((0.0, 0.004, 0.010))
    center_web = cq.Workplane("XY").box(0.020, 0.014, 0.064).translate((0.0, 0.047, 0.038))
    punch_bridge = cq.Workplane("XY").box(0.030, 0.012, 0.010).translate((0.0, 0.054, 0.008))

    handle = lever_plate.union(hinge_barrel).union(front_grip).union(rear_web).union(center_web).union(punch_bridge)
    for x_pos in (-0.014, 0.014):
        stem = cq.Workplane("XY").circle(0.0028).extrude(0.010).translate((x_pos, 0.054, -0.007))
        handle = handle.union(stem)

    return handle


def _drawer_shape() -> cq.Workplane:
    tray = cq.Workplane("XY").box(0.046, 0.060, 0.008).translate((-0.023, 0.0, 0.004))
    pocket = cq.Workplane("XY").box(0.040, 0.054, 0.006).translate((-0.0235, 0.0, 0.0055))
    return tray.cut(pocket)


def _rocker_shape() -> cq.Workplane:
    paddle = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.0004, 0.0015),
                (0.0028, 0.0010),
                (0.0038, -0.0045),
                (0.0031, -0.0110),
                (0.0010, -0.0128),
                (-0.0008, -0.0050),
            ]
        )
        .close()
        .extrude(0.006, both=True)
    )
    knuckle = cq.Workplane("XZ").circle(0.0014).extrude(0.00225, both=True)
    return paddle.union(knuckle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_punch")

    body_finish = model.material("body_finish", rgba=(0.22, 0.23, 0.25, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.15, 0.16, 0.17, 1.0))
    rocker_finish = model.material("rocker_finish", rgba=(0.60, 0.15, 0.14, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shape(), "body_shell"), material=body_finish, name="body_shell")

    handle = model.part("handle")
    handle.visual(mesh_from_cadquery(_handle_shape(), "handle_shell"), material=handle_finish, name="handle_shell")

    drawer = model.part("drawer")
    drawer.visual(mesh_from_cadquery(_drawer_shape(), "drawer_tray"), material=drawer_finish, name="drawer_tray")
    drawer.visual(
        Box((0.005, 0.062, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, 0.005)),
        material=drawer_finish,
        name="drawer_face",
    )

    rocker = model.part("release_rocker")
    rocker.visual(mesh_from_cadquery(_rocker_shape(), "release_rocker"), material=rocker_finish, name="rocker_paddle")

    model.articulation(
        "handle_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, -0.0485, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=0.0, upper=1.02),
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.032, 0.004, 0.0035)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=0.20, lower=0.0, upper=0.028),
    )
    model.articulation(
        "rocker_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(0.0345, -0.013, 0.024)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    drawer = object_model.get_part("drawer")
    rocker = object_model.get_part("release_rocker")

    handle_hinge = object_model.get_articulation("handle_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    rocker_hinge = object_model.get_articulation("rocker_hinge")

    ctx.allow_isolated_part(
        handle,
        reason="The handle is visibly carried by the rear hinge supports with a small modeled running clearance around the pivot.",
    )
    ctx.expect_within(drawer, body, axes="yz", margin=0.004, name="drawer stays aligned in the body slot")
    ctx.expect_overlap(drawer, body, axes="x", min_overlap=0.020, name="closed drawer remains inserted")
    ctx.expect_origin_gap(
        rocker,
        drawer,
        axis="z",
        min_gap=0.006,
        max_gap=0.030,
        name="release rocker sits above the drawer seam",
    )
    ctx.expect_origin_distance(
        rocker,
        drawer,
        axes="y",
        max_dist=0.030,
        name="release rocker stays beside the drawer opening",
    )

    handle_rest_aabb = ctx.part_world_aabb(handle)
    drawer_rest_pos = ctx.part_world_position(drawer)
    rocker_rest_aabb = ctx.part_world_aabb(rocker)

    handle_open_aabb = None
    drawer_open_pos = None
    rocker_open_aabb = None

    handle_upper = handle_hinge.motion_limits.upper if handle_hinge.motion_limits is not None else 0.9
    drawer_upper = drawer_slide.motion_limits.upper if drawer_slide.motion_limits is not None else 0.02
    rocker_upper = rocker_hinge.motion_limits.upper if rocker_hinge.motion_limits is not None else 0.2

    with ctx.pose({handle_hinge: handle_upper, drawer_slide: drawer_upper, rocker_hinge: rocker_upper}):
        ctx.expect_within(drawer, body, axes="yz", margin=0.004, name="drawer remains guided at full extension")
        ctx.expect_overlap(drawer, body, axes="x", min_overlap=0.015, name="extended drawer keeps retained insertion")
        handle_open_aabb = ctx.part_world_aabb(handle)
        drawer_open_pos = ctx.part_world_position(drawer)
        rocker_open_aabb = ctx.part_world_aabb(rocker)

    ctx.check(
        "handle lifts upward",
        handle_rest_aabb is not None
        and handle_open_aabb is not None
        and handle_open_aabb[1][2] > handle_rest_aabb[1][2] + 0.010,
        details=f"rest={handle_rest_aabb}, open={handle_open_aabb}",
    )
    ctx.check(
        "drawer slides outboard",
        drawer_rest_pos is not None
        and drawer_open_pos is not None
        and drawer_open_pos[0] > drawer_rest_pos[0] + 0.020,
        details=f"rest={drawer_rest_pos}, open={drawer_open_pos}",
    )
    ctx.check(
        "rocker tips outward",
        rocker_rest_aabb is not None
        and rocker_open_aabb is not None
        and rocker_open_aabb[1][0] > rocker_rest_aabb[1][0] + 0.0015,
        details=f"rest={rocker_rest_aabb}, open={rocker_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
