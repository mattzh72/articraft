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

BODY_L = 0.29
BODY_W = 0.24
BODY_H = 0.046

PLATE_L = 0.22
PLATE_W = 0.17
PLATE_T = 0.008

HINGE_X = -0.125
HINGE_Z = 0.066

LID_L = 0.272
LID_W = 0.232

TRAY_L = 0.164
TRAY_W = 0.168
TRAY_H = 0.018


def _body_shell() -> object:
    profile = [
        (-BODY_L / 2.0, 0.0),
        (-BODY_L / 2.0, 0.035),
        (-0.108, 0.045),
        (0.090, 0.045),
        (0.126, 0.036),
        (BODY_L / 2.0, 0.020),
        (BODY_L / 2.0, 0.0),
    ]
    shell = cq.Workplane("XZ").polyline(profile).close().extrude(BODY_W / 2.0, both=True)

    tray_bay = (
        cq.Workplane("XY")
        .box(0.182, 0.182, 0.029, centered=(True, True, False))
        .translate((0.060, 0.0, 0.0))
    )
    hinge_block = (
        cq.Workplane("XY")
        .box(0.040, 0.084, 0.026, centered=(True, True, False))
        .translate((-0.125, 0.0, BODY_H - 0.001))
    )
    rails = (
        cq.Workplane("XY")
        .box(0.112, 0.010, 0.010, centered=(True, True, False))
        .translate((0.042, 0.092, 0.009))
        .union(
            cq.Workplane("XY")
            .box(0.112, 0.010, 0.010, centered=(True, True, False))
            .translate((0.042, -0.092, 0.009))
        )
    )
    return shell.cut(tray_bay).union(hinge_block).union(rails)


def _lid_shell() -> object:
    outer_profile = [
        (0.000, 0.000),
        (0.000, 0.018),
        (0.026, 0.043),
        (0.120, 0.062),
        (0.214, 0.057),
        (0.252, 0.040),
        (LID_L, 0.018),
        (LID_L, 0.000),
    ]
    inner_profile = [
        (0.012, -0.004),
        (0.012, 0.010),
        (0.032, 0.028),
        (0.116, 0.042),
        (0.206, 0.038),
        (0.242, 0.025),
        (0.252, 0.008),
        (0.252, -0.004),
    ]
    outer = cq.Workplane("XZ").polyline(outer_profile).close().extrude(LID_W / 2.0, both=True)
    inner = cq.Workplane("XZ").polyline(inner_profile).close().extrude((LID_W - 0.028) / 2.0, both=True)
    return outer.cut(inner)
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sandwich_press")

    model.material("shell_black", rgba=(0.17, 0.18, 0.19, 1.0))
    model.material("plate_dark", rgba=(0.11, 0.11, 0.12, 1.0))
    model.material("hinge_dark", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("accent_dark", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("tray_steel", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")
    body.visual(mesh_from_cadquery(_body_shell(), "body_shell"), material="shell_black", name="body_shell")
    body.visual(
        Box((PLATE_L, PLATE_W, PLATE_T)),
        origin=Origin(xyz=(0.010, 0.0, 0.049)),
        material="plate_dark",
        name="lower_plate",
    )
    lid = model.part("lid")
    lid.visual(mesh_from_cadquery(_lid_shell(), "lid_shell"), material="shell_black", name="lid_shell")
    lid.visual(
        Box((PLATE_L, PLATE_W, 0.007)),
        origin=Origin(xyz=(0.136, 0.0, -0.0065)),
        material="plate_dark",
        name="upper_plate",
    )
    lid.visual(
        Box((0.192, 0.024, 0.018)),
        origin=Origin(xyz=(0.136, 0.094, 0.0015)),
        material="plate_dark",
        name="plate_rib_0",
    )
    lid.visual(
        Box((0.192, 0.024, 0.018)),
        origin=Origin(xyz=(0.136, -0.094, 0.0015)),
        material="plate_dark",
        name="plate_rib_1",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.036),
        origin=Origin(xyz=(0.002, 0.080, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_dark",
        name="hinge_ear_0",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.036),
        origin=Origin(xyz=(0.002, -0.080, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="hinge_dark",
        name="hinge_ear_1",
    )
    for y_pos, name in ((0.047, "handle_post_0"), (-0.047, "handle_post_1")):
        lid.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(LID_L - 0.002, y_pos, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="accent_dark",
            name=name,
        )
    lid.visual(
        Cylinder(radius=0.007, length=0.118),
        origin=Origin(xyz=(LID_L + 0.008, 0.0, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="accent_dark",
        name="handle_grip",
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.18,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((TRAY_L, TRAY_W, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material="tray_steel",
        name="tray_pan",
    )
    tray.visual(
        Box((TRAY_L - 0.008, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, (TRAY_W / 2.0) - 0.002, 0.006)),
        material="tray_steel",
        name="tray_wall_0",
    )
    tray.visual(
        Box((TRAY_L - 0.008, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, -(TRAY_W / 2.0) + 0.002, 0.006)),
        material="tray_steel",
        name="tray_wall_1",
    )
    tray.visual(
        Box((0.004, TRAY_W - 0.008, 0.012)),
        origin=Origin(xyz=(-(TRAY_L / 2.0) + 0.002, 0.0, 0.006)),
        material="tray_steel",
        name="tray_back",
    )
    tray.visual(
        Box((0.006, TRAY_W - 0.008, 0.018)),
        origin=Origin(xyz=((TRAY_L / 2.0) - 0.003, 0.0, 0.009)),
        material="tray_steel",
        name="tray_front",
    )
    tray.visual(
        Box((0.012, 0.108, 0.010)),
        origin=Origin(xyz=((TRAY_L / 2.0) + 0.006, 0.0, 0.005)),
        material="tray_steel",
        name="tray_pull",
    )

    model.articulation(
        "tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.051, 0.0, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=0.055,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_slide = object_model.get_articulation("tray_slide")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="upper_plate",
            negative_elem="lower_plate",
            min_gap=0.0005,
            max_gap=0.0035,
            name="closed plates stay nearly seated",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="upper_plate",
            elem_b="lower_plate",
            min_overlap=0.16,
            name="upper and lower cooking plates align in plan",
        )

    closed_handle = ctx.part_element_world_aabb(lid, elem="handle_grip")
    with ctx.pose({lid_hinge: 1.18}):
        open_handle = ctx.part_element_world_aabb(lid, elem="handle_grip")

    handle_lifts = (
        closed_handle is not None
        and open_handle is not None
        and open_handle[0][2] > closed_handle[0][2] + 0.07
    )
    ctx.check(
        "handle rises when the lid opens",
        handle_lifts,
        details=f"closed={closed_handle}, open={open_handle}",
    )

    with ctx.pose({tray_slide: 0.0}):
        ctx.expect_gap(
            body,
            tray,
            axis="z",
            positive_elem="lower_plate",
            min_gap=0.020,
            max_gap=0.036,
            name="tray stays below the lower cooking plate",
        )
        ctx.expect_within(
            tray,
            body,
            axes="y",
            margin=0.0,
            name="tray remains centered between the short guide rails",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.15,
            name="tray stays inserted at rest",
        )

    rest_tray = ctx.part_world_aabb(tray)
    with ctx.pose({tray_slide: 0.055}):
        ctx.expect_within(
            tray,
            body,
            axes="y",
            margin=0.0,
            name="extended tray remains laterally guided",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.11,
            name="extended tray keeps retained insertion",
        )
        extended_tray = ctx.part_world_aabb(tray)

    tray_extends = (
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[1][0] > rest_tray[1][0] + 0.04
    )
    ctx.check(
        "tray slides forward",
        tray_extends,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    return ctx.report()


object_model = build_object_model()
