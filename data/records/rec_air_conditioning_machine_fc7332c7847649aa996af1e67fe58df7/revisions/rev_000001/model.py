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

UNIT_WIDTH = 0.92
BODY_HEIGHT = 0.29
BODY_DEPTH = 0.205

OUTLET_WIDTH = UNIT_WIDTH - 0.09
OUTLET_DEPTH = 0.074
OUTLET_HEIGHT = 0.056
OUTLET_CENTER_Y = 0.150
OUTLET_CENTER_Z = 0.053

FLAP_WIDTH = UNIT_WIDTH - 0.08
FLAP_HEIGHT = 0.065
FLAP_HINGE_Y = BODY_DEPTH
FLAP_HINGE_Z = OUTLET_CENTER_Z + (OUTLET_HEIGHT / 2.0)

DISPLAY_WIDTH = 0.145
DISPLAY_HEIGHT = 0.054
DISPLAY_X = 0.255
DISPLAY_HINGE_Y = BODY_DEPTH
DISPLAY_HINGE_Z = 0.229


def _build_body_shape() -> cq.Workplane:
    outer_body = (
        cq.Workplane("XY")
        .box(UNIT_WIDTH, BODY_DEPTH, BODY_HEIGHT)
        .edges("|X")
        .fillet(0.036)
        .translate((0.0, BODY_DEPTH / 2.0, BODY_HEIGHT / 2.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            UNIT_WIDTH - 0.046,
            BODY_DEPTH - 0.030,
            BODY_HEIGHT - 0.050,
        )
        .translate(
            (
                0.0,
                (BODY_DEPTH - 0.030) / 2.0 - 0.006,
                (BODY_HEIGHT - 0.050) / 2.0 + 0.012,
            )
        )
    )
    body = outer_body.cut(cavity.val())

    outlet_cut = (
        cq.Workplane("XY")
        .box(OUTLET_WIDTH, OUTLET_DEPTH, OUTLET_HEIGHT)
        .translate((0.0, OUTLET_CENTER_Y, OUTLET_CENTER_Z))
    )
    return body.cut(outlet_cut.val())


def _build_flap_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(FLAP_WIDTH, 0.014, FLAP_HEIGHT)
        .edges("|X")
        .fillet(0.004)
        .translate((0.0, 0.007, -(FLAP_HEIGHT / 2.0)))
    )


def _build_display_cover_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(DISPLAY_WIDTH, 0.010, DISPLAY_HEIGHT)
        .edges("|X")
        .fillet(0.003)
        .translate((0.0, 0.005, -(DISPLAY_HEIGHT / 2.0)))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mini_split_indoor_unit")

    model.material("shell_white", rgba=(0.93, 0.94, 0.93, 1.0))
    model.material("trim_white", rgba=(0.90, 0.91, 0.90, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material="shell_white",
        name="shell",
    )

    flap = model.part("flap")
    flap.visual(
        mesh_from_cadquery(_build_flap_shape(), "lower_flap"),
        material="trim_white",
        name="panel",
    )

    display_cover = model.part("display_cover")
    display_cover.visual(
        mesh_from_cadquery(_build_display_cover_shape(), "display_cover"),
        material="trim_white",
        name="cover",
    )

    model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(0.0, FLAP_HINGE_Y, FLAP_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.2, effort=10.0, velocity=1.5),
    )
    model.articulation(
        "body_to_display_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=display_cover,
        origin=Origin(xyz=(DISPLAY_X, DISPLAY_HINGE_Y, DISPLAY_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.1, effort=3.0, velocity=1.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    display_cover = object_model.get_part("display_cover")
    flap_joint = object_model.get_articulation("body_to_flap")
    display_joint = object_model.get_articulation("body_to_display_cover")

    ctx.expect_overlap(
        flap,
        body,
        axes="x",
        min_overlap=FLAP_WIDTH - 0.03,
        name="lower flap spans nearly the full outlet width",
    )
    ctx.expect_overlap(
        display_cover,
        body,
        axes="xz",
        min_overlap=0.04,
        name="display cover sits within the fascia footprint",
    )

    flap_rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_joint: flap_joint.motion_limits.upper}):
        flap_open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "lower flap opens forward from the outlet edge",
        flap_rest_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[1][1] > flap_rest_aabb[1][1] + 0.03
        and flap_open_aabb[0][2] > flap_rest_aabb[0][2] + 0.01,
        details=f"rest={flap_rest_aabb}, open={flap_open_aabb}",
    )

    cover_rest_aabb = ctx.part_world_aabb(display_cover)
    with ctx.pose({display_joint: display_joint.motion_limits.upper}):
        cover_open_aabb = ctx.part_world_aabb(display_cover)
    ctx.check(
        "display cover lifts upward from the fascia",
        cover_rest_aabb is not None
        and cover_open_aabb is not None
        and cover_open_aabb[1][1] > cover_rest_aabb[1][1] + 0.01
        and cover_open_aabb[0][2] > cover_rest_aabb[0][2] + 0.02,
        details=f"rest={cover_rest_aabb}, open={cover_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
