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


BODY_W = 0.056
BODY_D = 0.034
BODY_H = 0.016

BUTTON_CAP_W = BUTTON_WELL_W = 0.0192
BUTTON_CAP_D = BUTTON_WELL_D = 0.0142
BUTTON_CAP_H = 0.0028
BUTTON_WELL_DEPTH = 0.0068
BUTTON_TRAVEL = 0.0017
BUTTON_CENTER_Y = -0.0065
BUTTON_CENTER_X = 0.0113
BUTTON_BOTTOM_Z = BODY_H - BUTTON_CAP_H + 0.0004

HINGE_RADIUS = 0.0015
HINGE_Y = BODY_D * 0.5 + 0.0005
HINGE_Z = BODY_H + 0.0006
BODY_KNUCKLE_LEN = 0.011
BODY_KNUCKLE_X = 0.0155

COVER_W = 0.048
COVER_D = 0.0155
COVER_T = 0.0022
COVER_CLEARANCE = 0.00025
COVER_REAR_OVERLAP = 0.0010
COVER_BARREL_LEN = 0.018
COVER_PANEL_CENTER_Y = -COVER_D * 0.5 + COVER_REAR_OVERLAP
COVER_PANEL_CENTER_Z = BODY_H + COVER_CLEARANCE + COVER_T * 0.5 - HINGE_Z


def _build_body_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0042)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-BUTTON_CENTER_X, BUTTON_CENTER_Y),
                (BUTTON_CENTER_X, BUTTON_CENTER_Y),
            ]
        )
        .rect(BUTTON_WELL_W, BUTTON_WELL_D)
        .cutBlind(-BUTTON_WELL_DEPTH)
    )


def _build_cover_panel() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(COVER_W, COVER_D, COVER_T, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0020)
        .translate((0.0, COVER_PANEL_CENTER_Y, COVER_PANEL_CENTER_Z))
    )


def _build_button_cap() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BUTTON_CAP_W, BUTTON_CAP_D, BUTTON_CAP_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.0020)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shutter_remote")

    body_dark = model.material("body_dark", rgba=(0.13, 0.14, 0.15, 1.0))
    cover_dark = model.material("cover_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    button_dark = model.material("button_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    body_shell_mesh = mesh_from_cadquery(_build_body_shell(), "remote_body_shell")
    cover_panel_mesh = mesh_from_cadquery(_build_cover_panel(), "remote_cover_panel")
    button_cap_mesh = mesh_from_cadquery(_build_button_cap(), "remote_button_cap")

    body = model.part("body")
    body.visual(body_shell_mesh, material=body_dark, name="body_shell")
    for suffix, x_center in enumerate((-BODY_KNUCKLE_X, BODY_KNUCKLE_X)):
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LEN),
            origin=Origin(
                xyz=(x_center, HINGE_Y, HINGE_Z),
                rpy=(0.0, 1.5707963267948966, 0.0),
            ),
            material=body_dark,
            name=f"hinge_knuckle_{suffix}",
        )

    cover = model.part("cover")
    cover.visual(cover_panel_mesh, material=cover_dark, name="cover_panel")
    cover.visual(
        Cylinder(radius=HINGE_RADIUS, length=COVER_BARREL_LEN),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=cover_dark,
        name="cover_barrel",
    )

    for suffix, x_center in enumerate((-BUTTON_CENTER_X, BUTTON_CENTER_X)):
        button = model.part(f"button_{suffix}")
        button.visual(button_cap_mesh, material=button_dark, name="button_cap")
        model.articulation(
            f"body_to_button_{suffix}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_center, BUTTON_CENTER_Y, BUTTON_BOTTOM_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.08,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    model.articulation(
        "body_to_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=3.5,
            lower=0.0,
            upper=1.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    cover_hinge = object_model.get_articulation("body_to_cover")
    button_0_slide = object_model.get_articulation("body_to_button_0")
    button_1_slide = object_model.get_articulation("body_to_button_1")

    ctx.expect_overlap(
        cover,
        body,
        axes="xy",
        min_overlap=0.012,
        name="cover stays over the remote body",
    )
    ctx.expect_overlap(
        button_0,
        body,
        axes="xy",
        min_overlap=0.012,
        name="button_0 sits on the front control area",
    )
    ctx.expect_overlap(
        button_1,
        body,
        axes="xy",
        min_overlap=0.012,
        name="button_1 sits on the front control area",
    )

    rest_cover_aabb = ctx.part_world_aabb(cover)
    cover_upper = cover_hinge.motion_limits.upper if cover_hinge.motion_limits is not None else None
    with ctx.pose({cover_hinge: cover_upper if cover_upper is not None else 1.0}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "cover opens upward",
        rest_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > rest_cover_aabb[1][2] + 0.010,
        details=f"rest_aabb={rest_cover_aabb}, open_aabb={open_cover_aabb}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_0_slide: BUTTON_TRAVEL}):
        pressed_button_0 = ctx.part_world_position(button_0)
        stationary_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "button_0 depresses independently",
        rest_button_0 is not None
        and pressed_button_0 is not None
        and rest_button_1 is not None
        and stationary_button_1 is not None
        and pressed_button_0[2] < rest_button_0[2] - 0.0010
        and abs(stationary_button_1[2] - rest_button_1[2]) < 1e-6,
        details=(
            f"rest_button_0={rest_button_0}, pressed_button_0={pressed_button_0}, "
            f"rest_button_1={rest_button_1}, stationary_button_1={stationary_button_1}"
        ),
    )

    with ctx.pose({button_1_slide: BUTTON_TRAVEL}):
        stationary_button_0 = ctx.part_world_position(button_0)
        pressed_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "button_1 depresses independently",
        rest_button_0 is not None
        and stationary_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_1 is not None
        and pressed_button_1[2] < rest_button_1[2] - 0.0010
        and abs(stationary_button_0[2] - rest_button_0[2]) < 1e-6,
        details=(
            f"rest_button_1={rest_button_1}, pressed_button_1={pressed_button_1}, "
            f"rest_button_0={rest_button_0}, stationary_button_0={stationary_button_0}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
