from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.160
BODY_WIDTH = 0.120
BODY_HEIGHT = 0.060
BASE_FOOT_HEIGHT = 0.010
UPPER_BODY_LENGTH = 0.148
UPPER_BODY_WIDTH = 0.108

PEDESTAL_RADIUS = 0.020
PEDESTAL_HEIGHT = 0.004

TURRET_RADIUS = 0.0275
TURRET_HEIGHT = 0.024
PAD_SIZE = 0.062
PAD_THICKNESS = 0.010


def _build_body_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BASE_FOOT_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.006)
    )

    housing = (
        cq.Workplane("XY")
        .workplane(offset=BASE_FOOT_HEIGHT)
        .box(
            UPPER_BODY_LENGTH,
            UPPER_BODY_WIDTH,
            BODY_HEIGHT - BASE_FOOT_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.008)
        .faces(">Z")
        .edges()
        .fillet(0.004)
    )

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=BODY_HEIGHT)
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
    )

    return foot.union(housing).union(pedestal)


def _build_yaw_head_shape() -> cq.Workplane:
    drum = (
        cq.Workplane("XY")
        .circle(TURRET_RADIUS)
        .extrude(TURRET_HEIGHT)
        .faces(">Z")
        .edges()
        .fillet(0.002)
    )

    output_pad = (
        cq.Workplane("XY")
        .workplane(offset=TURRET_HEIGHT)
        .box(PAD_SIZE, PAD_SIZE, PAD_THICKNESS, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .edges()
        .fillet(0.002)
    )

    return drum.union(output_pad)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="turret_style_yaw_head")

    model.material("housing_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("head_gray", rgba=(0.67, 0.70, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "turret_body"),
        material="housing_black",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT + PEDESTAL_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (BODY_HEIGHT + PEDESTAL_HEIGHT) / 2.0)),
    )

    yaw_head = model.part("yaw_head")
    yaw_head.visual(
        mesh_from_cadquery(_build_yaw_head_shape(), "yaw_head"),
        material="head_gray",
        name="head_shell",
    )
    yaw_head.inertial = Inertial.from_geometry(
        Box((PAD_SIZE, PAD_SIZE, TURRET_HEIGHT + PAD_THICKNESS)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (TURRET_HEIGHT + PAD_THICKNESS) / 2.0)),
    )

    model.articulation(
        "body_to_yaw_head",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yaw_head,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-pi,
            upper=pi,
            effort=12.0,
            velocity=2.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    yaw_head = object_model.get_part("yaw_head")
    yaw_joint = object_model.get_articulation("body_to_yaw_head")

    ctx.check("body part present", body.name == "body")
    ctx.check("yaw head part present", yaw_head.name == "yaw_head")
    ctx.check(
        "yaw joint is vertical revolute",
        yaw_joint.joint_type == ArticulationType.REVOLUTE
        and tuple(round(v, 6) for v in yaw_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={yaw_joint.joint_type}, axis={yaw_joint.axis}",
    )
    limits = yaw_joint.motion_limits
    ctx.check(
        "yaw joint has bilateral travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )

    with ctx.pose({yaw_joint: 0.0}):
        ctx.expect_contact(
            yaw_head,
            body,
            contact_tol=0.001,
            name="yaw head is seated on the body at rest",
        )
        ctx.expect_overlap(
            yaw_head,
            body,
            axes="xy",
            min_overlap=0.055,
            name="yaw head stays centered over the body footprint",
        )
        rest_aabb = ctx.part_world_aabb(yaw_head)

    with ctx.pose({yaw_joint: pi / 4.0}):
        ctx.expect_contact(
            yaw_head,
            body,
            contact_tol=0.001,
            name="yaw head remains supported while rotated",
        )
        turned_aabb = ctx.part_world_aabb(yaw_head)

    rest_dx = None if rest_aabb is None else rest_aabb[1][0] - rest_aabb[0][0]
    turned_dx = None if turned_aabb is None else turned_aabb[1][0] - turned_aabb[0][0]
    ctx.check(
        "square output pad visibly rotates in plan",
        rest_dx is not None and turned_dx is not None and turned_dx > rest_dx + 0.015,
        details=f"rest_dx={rest_dx}, turned_dx={turned_dx}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
