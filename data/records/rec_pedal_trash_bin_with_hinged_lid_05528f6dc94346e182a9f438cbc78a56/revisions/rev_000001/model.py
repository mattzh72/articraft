from __future__ import annotations

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedal_trash_bin")

    body_mat = model.material("body_mat", color=(0.85, 0.85, 0.85))
    lid_mat = model.material("lid_mat", color=(0.2, 0.2, 0.2))
    pedal_mat = model.material("pedal_mat", color=(0.1, 0.1, 0.1))

    # --- Body ---
    base_solid = (
        cq.Workplane("XY")
        .box(0.3, 0.3, 0.1)
        .translate((0.0, 0.0, 0.05))
    )
    upper_hollow = (
        cq.Workplane("XY")
        .box(0.3, 0.3, 0.5)
        .translate((0.0, 0.0, 0.35))
        .faces(">Z")
        .shell(-0.01)
    )
    body_cq = base_solid.union(upper_hollow)

    cutout = (
        cq.Workplane("XY")
        .box(0.16, 0.1, 0.08)
        .translate((0.0, 0.15, 0.04))
    )
    body_cq = body_cq.cut(cutout)

    # body_hinge_support = (
    #     cq.Workplane("XY")
    #     .box(0.3, 0.02, 0.02)
    #     .translate((0.0, -0.14, 0.61))
    # )
    # body_cq = body_cq.union(body_hinge_support)

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(body_cq, "body_mesh"),
        material=body_mat,
        name="body_vis",
    )

    # --- Pedal ---
    pedal_cq = (
        cq.Workplane("XY")
        .box(0.14, 0.14, 0.02)
        .translate((0.0, 0.07, 0.0))
        .union(
            cq.Workplane("YZ")
            .cylinder(0.162, 0.01)
        )
    )
    pedal = model.part("pedal")
    pedal.visual(
        mesh_from_cadquery(pedal_cq, "pedal_mesh"),
        material=pedal_mat,
        name="pedal_vis",
    )

    pedal_joint = model.articulation(
        "pedal_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, 0.11, 0.05)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=0.25),
    )

    # --- Lid ---
    lid_cq = (
        cq.Workplane("XY")
        .box(0.3, 0.3, 0.02)
        .translate((0.0, 0.15, 0.01))
        .union(
            cq.Workplane("YZ")
            .cylinder(0.3, 0.01)
        )
    )
    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(lid_cq, "lid_mesh"),
        material=lid_mat,
        name="lid_vis",
    )

    model.articulation(
        "lid_joint",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, -0.15, 0.60)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0, lower=0.0, upper=1.5),
        mimic=Mimic(joint="pedal_joint", multiplier=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    pedal = object_model.get_part("pedal")
    lid = object_model.get_part("lid")

    ctx.allow_overlap(
        body, pedal,
        reason="Pedal pivot pin is intentionally embedded in the body cutout walls."
    )
    ctx.allow_overlap(
        body, lid,
        reason="Lid hinge cylinder is intentionally embedded in the body back wall."
    )

    ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.1, name="lid_covers_body")

    pedal_joint = object_model.get_articulation("pedal_joint")
    with ctx.pose({pedal_joint: 0.25}):
        lid_aabb = ctx.part_world_aabb(lid)
        if lid_aabb:
            ctx.check("lid_opens_upward", lid_aabb[1][2] > 0.8, "Lid should open to Z > 0.8")
        
        pedal_aabb = ctx.part_world_aabb(pedal)
        if pedal_aabb:
            ctx.check("pedal_moves_down", pedal_aabb[0][2] < 0.02, "Pedal front should move down")

    return ctx.report()


object_model = build_object_model()
