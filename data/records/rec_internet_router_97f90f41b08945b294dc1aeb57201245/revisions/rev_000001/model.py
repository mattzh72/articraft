from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_router")

    body_mat = Material("warm_white_router_plastic", color=(0.86, 0.88, 0.84, 1.0))
    cover_mat = Material("slightly_lighter_service_cover", color=(0.78, 0.80, 0.77, 1.0))
    dark_mat = Material("dark_charcoal_plastic", color=(0.03, 0.035, 0.04, 1.0))
    socket_mat = Material("black_recessed_socket", color=(0.0, 0.0, 0.0, 1.0))
    green_mat = Material("green_status_led", color=(0.0, 0.85, 0.25, 1.0))
    blue_mat = Material("blue_status_led", color=(0.0, 0.32, 1.0, 1.0))
    amber_mat = Material("amber_link_led", color=(1.0, 0.58, 0.08, 1.0))

    # A slim, wall-hugging router body: width (X), depth (Y), height (Z).
    body_width = 0.220
    body_depth = 0.032
    body_height = 0.160
    body_front_y = -body_depth / 2.0
    body_top_z = body_height / 2.0

    body = model.part("body")
    rounded_body = (
        cq.Workplane("XY")
        .box(body_width, body_depth, body_height)
        .edges()
        .fillet(0.004)
    )
    body.visual(
        mesh_from_cadquery(rounded_body, "rounded_router_body", tolerance=0.0008),
        material=body_mat,
        name="body_shell",
    )

    # Back-side wall mounting cues: two dark keyhole marks on the wall-facing face.
    for z in (-0.040, 0.040):
        body.visual(
            Cylinder(radius=0.007, length=0.0015),
            origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.0004, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name=f"wall_keyhole_{0 if z < 0 else 1}",
        )
        body.visual(
            Box((0.006, 0.0015, 0.018)),
            origin=Origin(xyz=(0.0, body_depth / 2.0 + 0.0004, z - 0.010)),
            material=dark_mat,
            name=f"wall_slot_{0 if z < 0 else 1}",
        )

    # Recessed service-port bay behind the hinged front cover.
    body.visual(
        Box((0.192, 0.0020, 0.080)),
        origin=Origin(xyz=(0.0, body_front_y - 0.0011, -0.025)),
        material=dark_mat,
        name="service_backing",
    )
    for i, x in enumerate((-0.058, -0.020, 0.018, 0.056)):
        body.visual(
            Box((0.030, 0.0012, 0.018)),
            origin=Origin(xyz=(x, body_front_y - 0.0017, -0.033)),
            material=socket_mat,
            name=f"ethernet_port_{i}",
        )
        body.visual(
            Box((0.018, 0.0010, 0.003)),
            origin=Origin(xyz=(x, body_front_y - 0.0024, -0.023)),
            material=amber_mat,
            name=f"port_contacts_{i}",
        )
    body.visual(
        Box((0.034, 0.0012, 0.014)),
        origin=Origin(xyz=(0.0, body_front_y - 0.0017, -0.057)),
        material=socket_mat,
        name="service_usb_port",
    )

    # Small status LEDs remain visible above the service cover.
    body.visual(
        Box((0.060, 0.0015, 0.012)),
        origin=Origin(xyz=(-0.021, body_front_y - 0.0004, 0.044)),
        material=dark_mat,
        name="led_window_strip",
    )
    for i, (x, mat) in enumerate(((-0.042, green_mat), (-0.028, green_mat), (-0.014, blue_mat), (0.000, amber_mat))):
        body.visual(
            Cylinder(radius=0.0030, length=0.0016),
            origin=Origin(xyz=(x, body_front_y - 0.0010, 0.044), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=mat,
            name=f"status_led_{i}",
        )

    # Hinge lugs for the lower service cover.  The moving barrel occupies the
    # center span; these fixed barrels sit just outside it.
    cover_hinge_z = -0.064
    cover_hinge_y = body_front_y - 0.0060
    for side, x in enumerate((-0.101, 0.101)):
        body.visual(
            Box((0.014, 0.008, 0.020)),
            origin=Origin(xyz=(x, cover_hinge_y + 0.003, cover_hinge_z + 0.002)),
            material=dark_mat,
            name=f"cover_hinge_tab_{side}",
        )
        body.visual(
            Cylinder(radius=0.0042, length=0.014),
            origin=Origin(xyz=(x, cover_hinge_y, cover_hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name=f"cover_hinge_barrel_{side}",
        )

    # Two small top-edge bases/sockets for the antennas.
    antenna_xs = (-0.066, 0.066)
    antenna_hinge_y = -0.006
    antenna_pivot_z = body_top_z + 0.0162
    for i, (x, socket_name) in enumerate(zip(antenna_xs, ("antenna_socket_0", "antenna_socket_1"))):
        body.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, antenna_hinge_y, body_top_z + 0.005)),
            material=dark_mat,
            name=socket_name,
        )

    service_cover = model.part("service_cover")
    cover_height = 0.075
    cover_width = 0.180
    cover_thickness = 0.006
    service_cover.visual(
        Box((cover_width, cover_thickness, cover_height)),
        origin=Origin(xyz=(0.0, 0.0, cover_height / 2.0)),
        material=cover_mat,
        name="cover_panel",
    )
    service_cover.visual(
        Cylinder(radius=0.0040, length=0.176),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_mat,
        name="moving_hinge_barrel",
    )
    service_cover.visual(
        Box((0.058, 0.0030, 0.010)),
        origin=Origin(xyz=(0.0, -0.0045, 0.060)),
        material=dark_mat,
        name="finger_pull",
    )
    for i, x in enumerate((-cover_width / 2.0 + 0.006, cover_width / 2.0 - 0.006)):
        service_cover.visual(
            Box((0.006, 0.0020, 0.066)),
            origin=Origin(xyz=(x, -0.0040, 0.036)),
            material=dark_mat,
            name=f"cover_side_lip_{i}",
        )

    model.articulation(
        "body_to_service_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_cover,
        origin=Origin(xyz=(0.0, cover_hinge_y, cover_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.65),
    )

    for i, x in enumerate(antenna_xs):
        antenna = model.part(f"antenna_{i}")
        antenna.visual(
            Cylinder(radius=0.0060, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_mat,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0035, length=0.075),
            origin=Origin(xyz=(0.0, 0.0, 0.006 + 0.075 / 2.0)),
            material=dark_mat,
            name="antenna_rod",
        )
        antenna.visual(
            Sphere(radius=0.0040),
            origin=Origin(xyz=(0.0, 0.0, 0.006 + 0.075)),
            material=dark_mat,
            name="rounded_tip",
        )
        model.articulation(
            f"body_to_antenna_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=antenna,
            origin=Origin(xyz=(x, antenna_hinge_y, antenna_pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=-1.05, upper=1.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("service_cover")
    cover_hinge = object_model.get_articulation("body_to_service_cover")
    antenna_0 = object_model.get_part("antenna_0")
    antenna_1 = object_model.get_part("antenna_1")
    antenna_joint_0 = object_model.get_articulation("body_to_antenna_0")
    antenna_joint_1 = object_model.get_articulation("body_to_antenna_1")

    ctx.expect_gap(
        body,
        cover,
        axis="y",
        positive_elem="service_backing",
        negative_elem="cover_panel",
        min_gap=0.0005,
        max_gap=0.006,
        name="closed service cover sits just proud of port bay",
    )
    ctx.expect_overlap(
        cover,
        body,
        axes="xz",
        elem_a="cover_panel",
        elem_b="service_backing",
        min_overlap=0.070,
        name="cover spans the recessed service ports",
    )
    ctx.expect_gap(
        cover,
        cover,
        axis="z",
        positive_elem="cover_panel",
        negative_elem="moving_hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0045,
        name="cover panel is carried by its lower hinge barrel",
    )
    ctx.expect_gap(
        antenna_0,
        body,
        axis="z",
        positive_elem="hinge_barrel",
        negative_elem="antenna_socket_0",
        min_gap=0.0,
        max_gap=0.001,
        name="antenna 0 hinge is seated on its top socket",
    )
    ctx.expect_gap(
        antenna_1,
        body,
        axis="z",
        positive_elem="hinge_barrel",
        negative_elem="antenna_socket_1",
        min_gap=0.0,
        max_gap=0.001,
        name="antenna 1 hinge is seated on its top socket",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    closed_antenna_aabb = ctx.part_world_aabb(antenna_0)
    with ctx.pose({cover_hinge: 1.45, antenna_joint_0: 0.85, antenna_joint_1: -0.85}):
        open_cover_aabb = ctx.part_world_aabb(cover)
        tilted_antenna_aabb = ctx.part_world_aabb(antenna_0)

    ctx.check(
        "service cover rotates downward and outward",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] < closed_cover_aabb[1][2] - 0.020
        and open_cover_aabb[0][1] < closed_cover_aabb[0][1] - 0.035,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )
    ctx.check(
        "antenna hinge tilts the rod off vertical",
        closed_antenna_aabb is not None
        and tilted_antenna_aabb is not None
        and tilted_antenna_aabb[0][1] < closed_antenna_aabb[0][1] - 0.030
        and tilted_antenna_aabb[1][2] < closed_antenna_aabb[1][2] - 0.015,
        details=f"closed={closed_antenna_aabb}, tilted={tilted_antenna_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
