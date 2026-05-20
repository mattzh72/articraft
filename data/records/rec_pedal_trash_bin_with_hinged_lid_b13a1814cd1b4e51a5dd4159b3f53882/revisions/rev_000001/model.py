from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.290
BODY_DEPTH = 0.390
BODY_HEIGHT = 0.600
BODY_WALL = 0.0025
BODY_CORNER = 0.020

LID_OVERHANG = 0.010
LID_HEIGHT = 0.030
LID_WALL = 0.0022
LID_HINGE_Z = BODY_HEIGHT + 0.001

PEDAL_WIDTH = 0.182
PEDAL_HEIGHT = 0.060
PEDAL_DEPTH = 0.010
PEDAL_HINGE_Y = (BODY_DEPTH / 2.0) + 0.004
PEDAL_HINGE_Z = 0.020

DAMPER_WIDTH = 0.078
DAMPER_DEPTH = 0.050
DAMPER_HEIGHT = 0.026
DAMPER_WALL = 0.002
DAMPER_X = 0.074
DAMPER_HINGE_Y = -(BODY_DEPTH / 2.0) - DAMPER_DEPTH
DAMPER_HINGE_Z = BODY_HEIGHT + 0.016


def _body_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(BODY_CORNER)
        .faces(">Z")
        .shell(-BODY_WALL)
    )


def _lid_cap() -> cq.Workplane:
    lid_width = BODY_WIDTH + LID_OVERHANG
    lid_depth = BODY_DEPTH + LID_OVERHANG
    lid_corner = BODY_CORNER + 0.004
    return (
        cq.Workplane("XY")
        .box(lid_width, lid_depth, LID_HEIGHT)
        .translate((0.0, lid_depth / 2.0, LID_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(lid_corner)
        .faces("<Z")
        .shell(-LID_WALL)
    )


def _damper_cover() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(DAMPER_WIDTH, DAMPER_DEPTH, DAMPER_HEIGHT)
        .translate((0.0, DAMPER_DEPTH / 2.0, DAMPER_HEIGHT / 2.0))
        .edges("|Z")
        .fillet(0.008)
        .faces("<Z")
        .shell(-DAMPER_WALL)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_pedal_bin")

    shell_mat = model.material("shell_brushed", rgba=(0.72, 0.74, 0.76, 1.0))
    lid_mat = model.material("lid_metal", rgba=(0.63, 0.65, 0.68, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell"),
        material=shell_mat,
        name="shell",
    )
    body.visual(
        Box((BODY_WIDTH * 0.64, 0.004, 0.008)),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH / 2.0) + 0.002,
                BODY_HEIGHT - 0.003,
            )
        ),
        material=lid_mat,
        name="hinge_spine",
    )
    body.visual(
        Box((0.210, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH / 2.0) - 0.004, PEDAL_HINGE_Z)),
        material=lid_mat,
        name="pedal_housing",
    )
    body.visual(
        Box((0.138, 0.004, 0.030)),
        origin=Origin(xyz=(0.0, (BODY_DEPTH / 2.0) - BODY_WALL - 0.002, BODY_HEIGHT - 0.042)),
        material=lid_mat,
        name="bucket_handle_mount",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.110),
        origin=Origin(
            xyz=(0.0, (BODY_DEPTH / 2.0) - BODY_WALL - 0.009, BODY_HEIGHT - 0.030),
            rpy=(0.0, 1.57079632679, 0.0),
        ),
        material=lid_mat,
        name="bucket_handle",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(-0.050, (BODY_DEPTH / 2.0) - BODY_WALL - 0.007, BODY_HEIGHT - 0.040)),
        material=lid_mat,
        name="bucket_handle_post_0",
    )
    body.visual(
        Cylinder(radius=0.004, length=0.024),
        origin=Origin(xyz=(0.050, (BODY_DEPTH / 2.0) - BODY_WALL - 0.007, BODY_HEIGHT - 0.040)),
        material=lid_mat,
        name="bucket_handle_post_1",
    )
    body.visual(
        Box((0.068, 0.045, 0.018)),
        origin=Origin(
            xyz=(
                DAMPER_X,
                -(BODY_DEPTH / 2.0) - 0.0225,
                BODY_HEIGHT + 0.007,
            )
        ),
        material=lid_mat,
        name="damper_base",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_cap(), "lid_cap"),
        material=lid_mat,
        name="cap",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -(BODY_DEPTH / 2.0), LID_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.35,
        ),
    )

    pedal_mat = model.material("pedal_graphite", rgba=(0.15, 0.15, 0.16, 1.0))
    pedal = model.part("pedal")
    pedal.visual(
        Box((PEDAL_WIDTH, PEDAL_DEPTH, 0.010)),
        origin=Origin(xyz=(0.0, 0.005, 0.005)),
        material=pedal_mat,
        name="pedal_heel",
    )
    pedal.visual(
        Box((PEDAL_WIDTH, PEDAL_DEPTH, PEDAL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.012, 0.034), rpy=(-0.20, 0.0, 0.0)),
        material=pedal_mat,
        name="pedal_plate",
    )
    model.articulation(
        "pedal_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, PEDAL_HINGE_Y, PEDAL_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.2,
            lower=0.0,
            upper=0.42,
        ),
    )

    damper = model.part("damper_cover")
    damper.visual(
        mesh_from_cadquery(_damper_cover(), "damper_cover"),
        material=lid_mat,
        name="cover",
    )
    model.articulation(
        "damper_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=damper,
        origin=Origin(xyz=(DAMPER_X, DAMPER_HINGE_Y, DAMPER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    pedal = object_model.get_part("pedal")
    damper = object_model.get_part("damper_cover")
    lid_hinge = object_model.get_articulation("lid_hinge")
    pedal_hinge = object_model.get_articulation("pedal_hinge")
    damper_hinge = object_model.get_articulation("damper_hinge")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="cap",
            negative_elem="shell",
            max_gap=0.014,
            max_penetration=0.0,
            name="lid sits neatly above the shell rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="cap",
            elem_b="shell",
            min_overlap=0.24,
            name="lid covers the top opening footprint",
        )

    closed_aabb = ctx.part_element_world_aabb(lid, elem="cap")
    with ctx.pose({lid_hinge: 1.2}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="cap")

    ctx.check(
        "lid opens upward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    with ctx.pose({pedal_hinge: 0.0}):
        ctx.expect_gap(
            pedal,
            body,
            axis="y",
            max_gap=0.012,
            max_penetration=0.0,
            name="pedal sits on the front housing without burying into it",
        )
        ctx.expect_overlap(
            pedal,
            body,
            axes="x",
            elem_a="pedal_heel",
            elem_b="pedal_housing",
            min_overlap=0.14,
            name="pedal spans the lower front support width",
        )

    pedal_closed = ctx.part_element_world_aabb(pedal, elem="pedal_plate")
    with ctx.pose({pedal_hinge: 0.42}):
        pedal_pressed = ctx.part_element_world_aabb(pedal, elem="pedal_plate")

    ctx.check(
        "pedal presses inward",
        pedal_closed is not None
        and pedal_pressed is not None
        and pedal_pressed[0][1] < pedal_closed[0][1] - 0.015,
        details=f"closed={pedal_closed}, pressed={pedal_pressed}",
    )

    with ctx.pose({damper_hinge: 0.0}):
        ctx.expect_gap(
            damper,
            body,
            axis="z",
            positive_elem="cover",
            negative_elem="damper_base",
            max_gap=0.001,
            max_penetration=0.0,
            name="damper cover closes against the rear mechanism zone",
        )

    damper_closed = ctx.part_element_world_aabb(damper, elem="cover")
    with ctx.pose({damper_hinge: 0.8}):
        damper_open = ctx.part_element_world_aabb(damper, elem="cover")

    ctx.check(
        "damper cover lifts upward",
        damper_closed is not None
        and damper_open is not None
        and damper_open[1][2] > damper_closed[1][2] + 0.025,
        details=f"closed={damper_closed}, open={damper_open}",
    )

    handle_aabb = ctx.part_element_world_aabb(body, elem="bucket_handle")
    ctx.check(
        "bucket handle stays visible just below the lid line",
        handle_aabb is not None
        and handle_aabb[1][2] < BODY_HEIGHT
        and handle_aabb[1][2] > BODY_HEIGHT - 0.050
        and handle_aabb[1][1] > (BODY_DEPTH / 2.0) - 0.030,
        details=f"handle={handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
