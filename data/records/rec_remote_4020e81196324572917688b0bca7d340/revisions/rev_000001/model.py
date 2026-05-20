from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shutter_remote")

    body_cq = (
        cq.Workplane("XY")
        .box(0.040, 0.080, 0.015)
        .edges("|Z")
        .fillet(0.005)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.010, 0.020), (0.010, 0.020)])
        .hole(0.017, 0.008)
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_cq, "body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.002, height=0.010),
        origin=Origin(xyz=(-0.010, -0.040, 0.0075), rpy=(0.0, 1.570796, 0.0)),
        name="body_hinge_left",
    )
    body.visual(
        Cylinder(radius=0.002, height=0.010),
        origin=Origin(xyz=(0.010, -0.040, 0.0075), rpy=(0.0, 1.570796, 0.0)),
        name="body_hinge_right",
    )

    cover_cq = (
        cq.Workplane("XY")
        .box(0.040, 0.080, 0.005)
        .edges("|Z")
        .fillet(0.005)
        .faces("<Z")
        .shell(-0.001)
        .faces("<Y")
        .workplane()
        .pushPoints([(-0.010, 0.0), (0.010, 0.0)])
        .rect(0.010, 0.005)
        .cutBlind(-0.0025)
    )

    cover = model.part("top_cover")
    cover.visual(
        mesh_from_cadquery(cover_cq, "cover_shell"),
        origin=Origin(xyz=(0.0, 0.040, 0.0025)),  # offset from hinge
        name="cover_shell",
    )
    cover.visual(
        Cylinder(radius=0.002, height=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 1.570796, 0.0)),
        name="cover_hinge_center",
    )

    model.articulation(
        "cover_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.0, -0.040, 0.0075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=2.0),
    )

    button_0 = model.part("button_0")
    button_0.visual(
        Cylinder(radius=0.008, height=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)), # Center of 0.008 height cylinder is at 0.004
        name="button_0_cap",
    )
    model.articulation(
        "button_0_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_0,
        origin=Origin(xyz=(-0.010, 0.020, 0.0025)), # Bottom of hole is at Z=0.0025
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003),
    )

    button_1 = model.part("button_1")
    button_1.visual(
        Cylinder(radius=0.008, height=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        name="button_1_cap",
    )
    model.articulation(
        "button_1_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button_1,
        origin=Origin(xyz=(0.010, 0.020, 0.0025)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    cover = object_model.get_part("top_cover")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    # Cover sits flush on the body. We can allow a tiny overlap if they touch, or use expect_contact.
    # We hollowed out the cover, so it doesn't overlap the buttons anymore.
    # The buttons are inside the holes, so we allow overlap for the retained insertion.
    ctx.allow_overlap(button_0, body, reason="Button sits inside the body recess.")
    ctx.allow_overlap(button_1, body, reason="Button sits inside the body recess.")
    ctx.allow_overlap(cover, body, reason="Cover rests flush on the body.")

    ctx.allow_isolated_part(button_0, reason="Button has clearance gap in its guide hole.")
    ctx.allow_isolated_part(button_1, reason="Button has clearance gap in its guide hole.")

    ctx.expect_within(button_0, body, axes="xy")
    ctx.expect_within(button_1, body, axes="xy")
    ctx.expect_overlap(cover, body, axes="xy")

    with ctx.pose(cover_hinge=1.5):
        cover_aabb = ctx.part_world_aabb(cover)
        if cover_aabb is not None:
            ctx.check("cover_opens_upward", cover_aabb[1][2] > 0.030, "Cover AABB max Z should be higher when open.")

    with ctx.pose(button_0_press=0.003):
        # The button part origin doesn't move when we check part_world_position!
        # Wait, prismatic joints move the part frame.
        # Revolute joints rotate the part frame.
        # But let's use AABB to be safe.
        b0_aabb = ctx.part_world_aabb(button_0)
        if b0_aabb is not None:
            ctx.check("button_0_presses_down", b0_aabb[1][2] < 0.008, "Button 0 top should move down when pressed.")

    return ctx.report()


object_model = build_object_model()
