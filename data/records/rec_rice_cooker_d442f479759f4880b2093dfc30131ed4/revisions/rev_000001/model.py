from __future__ import annotations

import math

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


BODY_LENGTH = 0.28
BODY_WIDTH = 0.24
BODY_HEIGHT = 0.17
BODY_WALL = 0.017
BODY_BOTTOM = 0.022
HINGE_X = -0.130
HINGE_Z = 0.170
LID_LENGTH = 0.27
LID_WIDTH = 0.232
DIAL_Z = 0.078
DIAL_X = 0.162


def _body_shell_shape() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .ellipse(BODY_LENGTH / 2.0, BODY_WIDTH / 2.0)
        .extrude(BODY_HEIGHT)
    )

    inner = (
        cq.Workplane("XY")
        .ellipse(BODY_LENGTH / 2.0 - BODY_WALL, BODY_WIDTH / 2.0 - BODY_WALL)
        .extrude(BODY_HEIGHT - BODY_BOTTOM + 0.010)
        .translate((0.0, 0.0, BODY_BOTTOM))
    )
    return outer.cut(inner)


def _lid_shell_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .center(LID_LENGTH / 2.0, 0.0)
        .ellipse(LID_LENGTH / 2.0, LID_WIDTH / 2.0)
        .extrude(0.016)
    )
    crown = (
        cq.Workplane("XY")
        .center(LID_LENGTH * 0.52, 0.0)
        .ellipse(LID_LENGTH * 0.34, LID_WIDTH * 0.30)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.016))
    )
    vent = (
        cq.Workplane("XY")
        .center(0.090, 0.0)
        .rect(0.028, 0.022)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.018))
    )
    return base.union(crown).union(vent)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="domestic_rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.94, 0.94, 0.92, 1.0))
    silver_trim = model.material("silver_trim", rgba=(0.72, 0.74, 0.76, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.60, 0.61, 0.62, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((0.028, 0.110, 0.090)),
        origin=Origin(xyz=(0.146, 0.0, 0.082)),
        material=silver_trim,
        name="control_panel",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=Origin(xyz=(0.159, 0.0, DIAL_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="dial_bezel",
    )
    body.visual(
        Box((0.024, 0.082, 0.018)),
        origin=Origin(xyz=(-0.128, 0.0, 0.161)),
        material=silver_trim,
        name="hinge_mount",
    )
    body.visual(
        Box((0.020, 0.050, 0.010)),
        origin=Origin(xyz=(0.149, 0.0, 0.145)),
        material=warm_gray,
        name="latch_pad",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell_shape(), "lid_shell"),
        material=shell_white,
        name="lid_shell",
    )
    lid.visual(
        Box((0.018, 0.080, 0.014)),
        origin=Origin(xyz=(0.009, 0.0, 0.007)),
        material=silver_trim,
        name="hinge_leaf",
    )
    lid.visual(
        Box((0.022, 0.050, 0.014)),
        origin=Origin(xyz=(LID_LENGTH - 0.011, 0.0, 0.010)),
        material=warm_gray,
        name="latch_lip",
    )

    selector = model.part("selector")
    selector.visual(
        Cylinder(radius=0.017, length=0.016),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="selector_knob",
    )
    selector.visual(
        Cylinder(radius=0.021, length=0.006),
        origin=Origin(xyz=(0.003, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="selector_skirt",
    )
    selector.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_gray,
        name="selector_cap",
    )
    selector.visual(
        Box((0.002, 0.004, 0.008)),
        origin=Origin(xyz=(0.017, 0.0, 0.0115)),
        material=silver_trim,
        name="selector_marker",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "body_to_selector",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector,
        origin=Origin(xyz=(DIAL_X, 0.0, DIAL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    selector = object_model.get_part("selector")
    lid_hinge = object_model.get_articulation("body_to_lid")
    selector_joint = object_model.get_articulation("body_to_selector")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="body_shell",
        max_gap=0.008,
        max_penetration=0.0,
        name="closed lid sits tightly above the cooker body",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_shell",
        elem_b="body_shell",
        min_overlap=0.18,
        name="closed lid covers the body opening footprint",
    )
    ctx.expect_gap(
        selector,
        body,
        axis="x",
        positive_elem="selector_knob",
        negative_elem="dial_bezel",
        max_gap=0.003,
        max_penetration=0.0,
        name="selector knob mounts directly to the front bezel",
    )

    latch_aabb = ctx.part_element_world_aabb(lid, elem="latch_lip")
    selector_aabb = ctx.part_element_world_aabb(selector, elem="selector_knob")
    if latch_aabb is not None and selector_aabb is not None:
        latch_center_z = 0.5 * (float(latch_aabb[0][2]) + float(latch_aabb[1][2]))
        selector_center_z = 0.5 * (float(selector_aabb[0][2]) + float(selector_aabb[1][2]))
        ctx.check(
            "selector sits below latch",
            selector_center_z < latch_center_z - 0.035,
            details=f"selector_z={selector_center_z:.4f}, latch_z={latch_center_z:.4f}",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    if closed_lid_aabb is not None and open_lid_aabb is not None:
        ctx.check(
            "lid opens upward",
            float(open_lid_aabb[1][2]) > float(closed_lid_aabb[1][2]) + 0.10,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )
        ctx.check(
            "lid swings rearward from the front edge",
            float(open_lid_aabb[1][0]) < float(closed_lid_aabb[1][0]) - 0.10,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    marker_aabb_0 = ctx.part_element_world_aabb(selector, elem="selector_marker")
    with ctx.pose({selector_joint: math.pi / 2.0}):
        marker_aabb_90 = ctx.part_element_world_aabb(selector, elem="selector_marker")
    if marker_aabb_0 is not None and marker_aabb_90 is not None:
        marker_y0 = 0.5 * (float(marker_aabb_0[0][1]) + float(marker_aabb_0[1][1]))
        marker_z0 = 0.5 * (float(marker_aabb_0[0][2]) + float(marker_aabb_0[1][2]))
        marker_y90 = 0.5 * (float(marker_aabb_90[0][1]) + float(marker_aabb_90[1][1]))
        marker_z90 = 0.5 * (float(marker_aabb_90[0][2]) + float(marker_aabb_90[1][2]))
        ctx.check(
            "selector dial rotates around its axle",
            abs(marker_y90 - marker_y0) > 0.008 and abs(marker_z90 - marker_z0) > 0.008,
            details=(
                f"q0=(y={marker_y0:.4f}, z={marker_z0:.4f}), "
                f"q90=(y={marker_y90:.4f}, z={marker_z90:.4f})"
            ),
        )

    return ctx.report()


object_model = build_object_model()
