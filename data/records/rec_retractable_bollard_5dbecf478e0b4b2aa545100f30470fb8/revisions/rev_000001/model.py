from __future__ import annotations

from math import pi

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


CURB_LENGTH = 0.74
CURB_WIDTH = 0.46
CURB_HEIGHT = 0.18
CURB_TOP_Z = 0.032
CURB_BOTTOM_Z = CURB_TOP_Z - CURB_HEIGHT

SOCKET_OUTER_RADIUS = 0.16
SLEEVE_OUTER_RADIUS = 0.112
SLEEVE_INNER_RADIUS = 0.092
SLEEVE_DEPTH = 0.52
SLEEVE_HOLE_DEPTH = 0.50
SLEEVE_RECESS_RADIUS = 0.128
SLEEVE_FLANGE_HEIGHT = 0.006

POST_RADIUS = 0.084
POST_INSERTION = 0.50
POST_EXPOSED = 0.98
POST_LENGTH = POST_INSERTION + POST_EXPOSED
POST_TRAVEL = 0.20

PLATE_LENGTH = 0.18
PLATE_WIDTH = 0.094
PLATE_THICKNESS = 0.010
PLATE_CENTER_X = 0.212
PLATE_HINGE_X = PLATE_CENTER_X + (PLATE_LENGTH / 2.0)
PLATE_HINGE_Z = CURB_TOP_Z + 0.007
PLATE_SLOT_LENGTH = 0.102
PLATE_SLOT_WIDTH = 0.018

PLATE_SEAT_LENGTH = 0.182
PLATE_SEAT_WIDTH = 0.096
PLATE_SEAT_BOTTOM_Z = CURB_TOP_Z - PLATE_THICKNESS
LOCK_POCKET_LENGTH = 0.150
LOCK_POCKET_WIDTH = 0.058
LOCK_POCKET_BOTTOM_Z = -0.013
LOCK_CHANNEL_LENGTH = 0.060
LOCK_CHANNEL_WIDTH = 0.022
LOCK_CHANNEL_CENTER_X = 0.138

PLUG_RADIUS = 0.032
PLUG_THICKNESS = 0.016
PLUG_STEM_RADIUS = 0.012
PLUG_STEM_LENGTH = 0.018
PLUG_GRIP_LENGTH = 0.060
PLUG_GRIP_WIDTH = 0.014
PLUG_GRIP_HEIGHT = 0.008
PLUG_TURN_ANGLE = pi / 2.0
POST_SOCKET_RADIUS = 0.018
POST_SOCKET_DEPTH = 0.021


def _vertical_cylinder(radius: float, height: float, z_min: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((0.0, 0.0, z_min))


def _y_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -(length / 2.0)))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
    )


def _make_concrete_body() -> cq.Workplane:
    curb = cq.Workplane("XY").box(CURB_LENGTH, CURB_WIDTH, CURB_HEIGHT).translate(
        (0.0, 0.0, CURB_TOP_Z - (CURB_HEIGHT / 2.0))
    )
    buried_socket = _vertical_cylinder(
        SOCKET_OUTER_RADIUS,
        CURB_BOTTOM_Z - (-SLEEVE_DEPTH),
        -SLEEVE_DEPTH,
    )
    body = curb.union(buried_socket)

    sleeve_recess = _vertical_cylinder(SLEEVE_RECESS_RADIUS, CURB_TOP_Z, 0.0)
    sleeve_clearance = _vertical_cylinder(SLEEVE_INNER_RADIUS, SLEEVE_HOLE_DEPTH, -SLEEVE_HOLE_DEPTH)
    lock_pocket = cq.Workplane("XY").box(
        LOCK_POCKET_LENGTH,
        LOCK_POCKET_WIDTH,
        PLATE_SEAT_BOTTOM_Z - LOCK_POCKET_BOTTOM_Z,
    ).translate(
        (
            PLATE_CENTER_X,
            0.0,
            (PLATE_SEAT_BOTTOM_Z + LOCK_POCKET_BOTTOM_Z) / 2.0,
        )
    )
    lock_channel = cq.Workplane("XY").box(
        LOCK_CHANNEL_LENGTH,
        LOCK_CHANNEL_WIDTH,
        PLATE_SEAT_BOTTOM_Z - LOCK_POCKET_BOTTOM_Z,
    ).translate(
        (
            LOCK_CHANNEL_CENTER_X,
            0.0,
            (PLATE_SEAT_BOTTOM_Z + LOCK_POCKET_BOTTOM_Z) / 2.0,
        )
    )

    return body.cut(sleeve_recess).cut(sleeve_clearance).cut(lock_pocket).cut(lock_channel)


def _make_plate_mount() -> cq.Workplane:
    base_strip = cq.Workplane("XY").box(0.020, 0.104, 0.006).translate(
        (PLATE_HINGE_X + 0.026, 0.0, CURB_TOP_Z + 0.003)
    )
    ears = (
        cq.Workplane("XY")
        .box(0.014, 0.020, 0.018)
        .translate((PLATE_HINGE_X + 0.028, 0.030, CURB_TOP_Z + 0.009))
        .union(
            cq.Workplane("XY")
            .box(0.014, 0.020, 0.018)
            .translate((PLATE_HINGE_X + 0.028, -0.030, CURB_TOP_Z + 0.009))
        )
    )
    return base_strip.union(ears)


def _make_post_shell() -> cq.Workplane:
    post = _vertical_cylinder(POST_RADIUS, POST_LENGTH, -POST_INSERTION)
    plug_socket = _vertical_cylinder(POST_SOCKET_RADIUS, POST_SOCKET_DEPTH, POST_EXPOSED - POST_SOCKET_DEPTH)
    return post.cut(plug_socket)


def _make_plate_panel() -> cq.Workplane:
    panel = cq.Workplane("XY").box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS).translate(
        (-PLATE_LENGTH / 2.0, 0.0, CURB_TOP_Z + (PLATE_THICKNESS / 2.0) - PLATE_HINGE_Z)
    )
    barrel = _y_axis_cylinder(0.008, 0.040)
    slot = cq.Workplane("XY").box(PLATE_SLOT_LENGTH, PLATE_SLOT_WIDTH, 0.020).translate(
        (-0.092, 0.0, CURB_TOP_Z + (PLATE_THICKNESS / 2.0) - PLATE_HINGE_Z)
    )
    return panel.union(barrel).cut(slot)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="removable_access_bollard")

    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.61, 1.0))
    galvanized = model.material("galvanized", rgba=(0.71, 0.73, 0.76, 1.0))
    bollard_paint = model.material("bollard_paint", rgba=(0.85, 0.68, 0.13, 1.0))
    plug_black = model.material("plug_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_concrete_body(), "curb_body"),
        material=concrete,
        name="curb_body",
    )
    base.visual(
        mesh_from_cadquery(_make_plate_mount(), "plate_mount"),
        material=galvanized,
        name="plate_mount",
    )

    post = model.part("post")
    post.visual(
        mesh_from_cadquery(_make_post_shell(), "post_shell"),
        material=bollard_paint,
        name="post_shell",
    )

    locking_plate = model.part("locking_plate")
    locking_plate.visual(
        mesh_from_cadquery(_make_plate_panel(), "locking_plate"),
        material=galvanized,
        name="plate_panel",
    )

    release_plug = model.part("release_plug")
    release_plug.visual(
        Cylinder(radius=PLUG_RADIUS, length=PLUG_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, PLUG_THICKNESS / 2.0)),
        material=plug_black,
        name="plug_body",
    )
    release_plug.visual(
        Cylinder(radius=PLUG_STEM_RADIUS, length=PLUG_STEM_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, -(PLUG_STEM_LENGTH / 2.0))),
        material=plug_black,
        name="plug_stem",
    )
    release_plug.visual(
        Box((PLUG_GRIP_LENGTH, PLUG_GRIP_WIDTH, PLUG_GRIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                PLUG_THICKNESS + (PLUG_GRIP_HEIGHT / 2.0),
            )
        ),
        material=plug_black,
        name="plug_grip",
    )

    model.articulation(
        "post_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=post,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=POST_TRAVEL, effort=180.0, velocity=0.20),
    )
    model.articulation(
        "plate_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=locking_plate,
        origin=Origin(xyz=(PLATE_HINGE_X, 0.0, PLATE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=25.0, velocity=1.2),
    )
    model.articulation(
        "plug_turn",
        ArticulationType.REVOLUTE,
        parent=post,
        child=release_plug,
        origin=Origin(xyz=(0.0, 0.0, POST_EXPOSED)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=PLUG_TURN_ANGLE, effort=6.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    post = object_model.get_part("post")
    locking_plate = object_model.get_part("locking_plate")
    release_plug = object_model.get_part("release_plug")

    post_slide = object_model.get_articulation("post_slide")
    plate_hinge = object_model.get_articulation("plate_hinge")
    plug_turn = object_model.get_articulation("plug_turn")

    ctx.expect_origin_distance(
        post,
        base,
        axes="xy",
        max_dist=0.001,
        name="post axis stays centered over the sleeve",
    )
    ctx.allow_overlap(
        base,
        post,
        elem_a="curb_body",
        elem_b="post_shell",
        reason=(
            "The buried sleeve is represented by the curb-body proxy while the bollard "
            "post is intentionally authored as a sliding member inside that socket."
        ),
    )
    ctx.expect_contact(
        release_plug,
        post,
        elem_a="plug_body",
        elem_b="post_shell",
        contact_tol=0.001,
        name="release plug seats on the post top",
    )

    rest_post_aabb = ctx.part_world_aabb(post)
    with ctx.pose({post_slide: POST_TRAVEL}):
        ctx.expect_origin_distance(
            post,
            base,
            axes="xy",
            max_dist=0.001,
            name="lifted post remains coaxial with the sleeve",
        )
        lifted_post_aabb = ctx.part_world_aabb(post)

    ctx.check(
        "post lifts upward from the sleeve",
        rest_post_aabb is not None
        and lifted_post_aabb is not None
        and lifted_post_aabb[1][2] > rest_post_aabb[1][2] + 0.18,
        details=f"rest={rest_post_aabb}, lifted={lifted_post_aabb}",
    )

    rest_plate_aabb = ctx.part_world_aabb(locking_plate)
    with ctx.pose({plate_hinge: 1.10}):
        open_plate_aabb = ctx.part_world_aabb(locking_plate)

    ctx.check(
        "locking plate swings up to uncover the recess",
        rest_plate_aabb is not None
        and open_plate_aabb is not None
        and open_plate_aabb[1][2] > rest_plate_aabb[1][2] + 0.07,
        details=f"rest={rest_plate_aabb}, open={open_plate_aabb}",
    )

    rest_grip_aabb = ctx.part_element_world_aabb(release_plug, elem="plug_grip")
    with ctx.pose({plug_turn: PLUG_TURN_ANGLE}):
        turned_grip_aabb = ctx.part_element_world_aabb(release_plug, elem="plug_grip")
        ctx.expect_contact(
            release_plug,
            post,
            elem_a="plug_body",
            elem_b="post_shell",
            contact_tol=0.001,
            name="release plug remains seated while turned",
        )

    rest_x_span = None if rest_grip_aabb is None else rest_grip_aabb[1][0] - rest_grip_aabb[0][0]
    rest_y_span = None if rest_grip_aabb is None else rest_grip_aabb[1][1] - rest_grip_aabb[0][1]
    turned_x_span = None if turned_grip_aabb is None else turned_grip_aabb[1][0] - turned_grip_aabb[0][0]
    turned_y_span = None if turned_grip_aabb is None else turned_grip_aabb[1][1] - turned_grip_aabb[0][1]

    ctx.check(
        "release plug grip visibly quarter turns",
        rest_x_span is not None
        and rest_y_span is not None
        and turned_x_span is not None
        and turned_y_span is not None
        and rest_x_span > rest_y_span * 2.0
        and turned_y_span > turned_x_span * 2.0,
        details=(
            f"rest_spans=({rest_x_span}, {rest_y_span}), "
            f"turned_spans=({turned_x_span}, {turned_y_span})"
        ),
    )

    return ctx.report()


object_model = build_object_model()
