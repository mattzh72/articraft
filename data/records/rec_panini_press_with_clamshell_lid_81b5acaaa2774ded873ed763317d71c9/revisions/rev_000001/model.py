from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_LENGTH = 0.300
BODY_WIDTH = 0.230
BODY_HEIGHT = 0.045
BODY_WALL = 0.004

LID_BACK_OFFSET = 0.005
LID_LENGTH = 0.235
LID_WIDTH = 0.220
LID_HEIGHT = 0.038
LID_WALL = 0.0035

HINGE_X = -0.108
HINGE_Z = 0.046


def _body_shell_shape():
    outer = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_LENGTH - 2.0 * BODY_WALL,
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - BODY_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_WALL))
    )
    return outer.cut(inner)


def _lid_shell_shape():
    outer = (
        cq.Workplane("XY")
        .box(LID_LENGTH, LID_WIDTH, LID_HEIGHT, centered=(False, True, False))
        .translate((LID_BACK_OFFSET, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.018)
        .edges(">Z")
        .fillet(0.012)
    )
    inner = (
        cq.Workplane("XY")
        .box(
            LID_LENGTH - 2.0 * LID_WALL,
            LID_WIDTH - 2.0 * LID_WALL,
            LID_HEIGHT - LID_WALL,
            centered=(False, True, False),
        )
        .translate((LID_BACK_OFFSET + LID_WALL, 0.0, LID_WALL))
    )
    shell = outer.cut(inner)
    handle_mount = (
        cq.Workplane("XY")
        .box(0.032, 0.096, 0.022, centered=(True, True, False))
        .translate((0.226, 0.0, 0.010))
    )
    handle_grip = (
        cq.Workplane("XY")
        .box(0.022, 0.094, 0.014, centered=(True, True, False))
        .translate((0.236, 0.0, 0.030))
    )
    return shell.union(handle_mount).union(handle_grip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sandwich_press")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    body_trim = model.material("body_trim", rgba=(0.10, 0.10, 0.11, 1.0))
    plate_finish = model.material("plate_finish", rgba=(0.69, 0.70, 0.72, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.12, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "sandwich_press_body_shell"),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((0.048, 0.094, 0.018)),
        origin=Origin(xyz=(-0.139, 0.0, 0.053)),
        material=body_trim,
        name="hinge_block",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.084),
        origin=Origin(
            xyz=(HINGE_X, 0.0, HINGE_Z),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_trim,
        name="hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(
            xyz=(0.076, BODY_WIDTH * 0.5 + 0.003, 0.028),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_trim,
        name="dial_boss",
    )
    body.visual(
        Box((0.284, 0.221, 0.008)),
        origin=Origin(xyz=(0.008, 0.0, 0.0385)),
        material=plate_finish,
        name="lower_plate",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "sandwich_press_lid_shell"),
        material=body_finish,
        name="lid_shell",
    )
    lid.visual(
        Box((0.224, 0.214, 0.008)),
        origin=Origin(xyz=(0.127, 0.0, 0.0025)),
        material=plate_finish,
        name="upper_plate",
    )
    lid.visual(
        Box((0.228, 0.216, 0.006)),
        origin=Origin(xyz=(0.127, 0.0, 0.0095)),
        material=body_trim,
        name="plate_backer",
    )
    lid.visual(
        Cylinder(radius=0.0065, length=0.072),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=body_trim,
        name="hinge_sleeve",
    )
    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.030,
                0.018,
                body_style="skirted",
                top_diameter=0.024,
                skirt=KnobSkirt(0.034, 0.004, flare=0.08),
                grip=KnobGrip(style="fluted", count=14, depth=0.0010),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
                center=False,
            ),
            "sandwich_press_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_finish,
        name="dial_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.076, BODY_WIDTH * 0.5 + 0.007, 0.028)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    dial = object_model.get_part("dial")
    lid_hinge = object_model.get_articulation("body_to_lid")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.allow_overlap(
        body,
        lid,
        elem_a="hinge_pin",
        elem_b="hinge_sleeve",
        reason="The visible rear hinge is represented as a lid sleeve rotating around a fixed hinge pin.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            max_gap=0.003,
            max_penetration=0.0,
            name="cooking plates nearly meet when closed",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.18,
            name="upper plate stays centered over lower plate",
        )
        ctx.expect_gap(
            dial,
            body,
            axis="y",
            positive_elem="dial_cap",
            negative_elem="dial_boss",
            max_gap=0.001,
            max_penetration=0.0,
            name="dial mounts flush against the right-side boss",
        )
        ctx.expect_overlap(
            dial,
            body,
            axes="xz",
            elem_a="dial_cap",
            elem_b="dial_boss",
            min_overlap=0.020,
            name="dial stays centered on the right-side wall",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    closed_upper_plate_aabb = ctx.part_element_world_aabb(lid, elem="upper_plate")
    lid_shell_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    hinge_block_aabb = ctx.part_element_world_aabb(body, elem="hinge_block")
    body_shell_aabb = ctx.part_element_world_aabb(body, elem="body_shell")

    handle_centered = False
    if closed_lid_aabb is not None and closed_upper_plate_aabb is not None:
        handle_centered = (
            abs(
                (closed_lid_aabb[0][1] + closed_lid_aabb[1][1]) * 0.5
                - (closed_upper_plate_aabb[0][1] + closed_upper_plate_aabb[1][1]) * 0.5
            )
            <= 0.004
            and closed_lid_aabb[1][0] >= closed_upper_plate_aabb[1][0] + 0.006
        )
    ctx.check(
        "front handle is centered over the lid nose",
        handle_centered,
        details=f"lid_shell={closed_lid_aabb!r}, upper_plate={closed_upper_plate_aabb!r}",
    )

    hinge_proud = False
    if hinge_block_aabb is not None and body_shell_aabb is not None:
        hinge_proud = (
            hinge_block_aabb[1][2] >= body_shell_aabb[1][2] + 0.010
            and hinge_block_aabb[0][0] <= body_shell_aabb[0][0] + 0.015
        )
    ctx.check(
        "rear hinge block stays visible above the lower body",
        hinge_proud,
        details=f"hinge_block={hinge_block_aabb!r}, body_shell={body_shell_aabb!r}",
    )

    dial_limits = dial_joint.motion_limits
    ctx.check(
        "dial uses continuous rotation limits",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"limits={dial_limits!r}",
    )

    open_lid_aabb = None
    upper_limit = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if upper_limit is not None:
        with ctx.pose({lid_hinge: upper_limit}):
            open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    lid_opens_upward = False
    if closed_lid_aabb is not None and open_lid_aabb is not None:
        lid_opens_upward = open_lid_aabb[1][2] >= closed_lid_aabb[1][2] + 0.090
    ctx.check(
        "top shell rotates upward on the rear hinge line",
        lid_opens_upward,
        details=f"closed_lid={closed_lid_aabb!r}, open_lid={open_lid_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
