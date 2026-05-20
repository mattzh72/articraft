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


BODY_LENGTH = 0.32
BODY_WIDTH = 0.26
BODY_HEIGHT = 0.16
BODY_WALL = 0.0045

LID_LENGTH = 0.298
LID_WIDTH = 0.238
LID_HEIGHT = 0.042
LID_WALL = 0.004

HINGE_X = -0.145
HINGE_Z = BODY_HEIGHT

VENT_CENTER_X = 0.076
VENT_JOINT_X = 0.055
VENT_JOINT_Z = 0.041


def _body_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse(BODY_LENGTH / 2.0, BODY_WIDTH / 2.0)
        .extrude(BODY_HEIGHT)
        .edges(">Z")
        .fillet(0.016)
        .faces(">Z")
        .shell(-BODY_WALL)
    )


def _base_trim() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .ellipse((BODY_LENGTH - 0.012) / 2.0, (BODY_WIDTH - 0.012) / 2.0)
        .extrude(0.028)
        .edges(">Z")
        .fillet(0.006)
    )


def _lid_shell() -> cq.Workplane:
    lid = (
        cq.Workplane("XY")
        .ellipse(LID_LENGTH / 2.0, LID_WIDTH / 2.0)
        .extrude(LID_HEIGHT)
        .edges(">Z")
        .fillet(0.014)
        .faces("<Z")
        .shell(-LID_WALL)
    )
    vent_cut = (
        cq.Workplane("XY")
        .center(VENT_CENTER_X - 0.135, 0.0)
        .ellipse(0.018, 0.010)
        .extrude(LID_HEIGHT + 0.01)
    )
    return lid.cut(vent_cut)


def _vent_collar() -> cq.Workplane:
    outer = cq.Workplane("XY").ellipse(0.026, 0.016).extrude(0.003)
    inner = cq.Workplane("XY").ellipse(0.019, 0.011).extrude(0.003)
    return outer.cut(inner)


def _front_handle() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.032, 0.100, 0.016, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.005)
        .edges(">Z")
        .fillet(0.004)
    )


def _cap_shell() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.042, 0.030, 0.010, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.004)
        .edges(">Z")
        .fillet(0.003)
    )


def _element_center_z(ctx: TestContext, part_name: str, elem_name: str) -> float | None:
    bounds = ctx.part_element_world_aabb(part_name, elem=elem_name)
    if bounds is None:
        return None
    lower, upper = bounds
    return (lower[2] + upper[2]) / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_rice_cooker")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    lid_silver = model.material("lid_silver", rgba=(0.84, 0.85, 0.87, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    display_black = model.material("display_black", rgba=(0.08, 0.09, 0.10, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "housing_shell"),
        material=body_white,
        name="housing_shell",
    )
    body.visual(
        mesh_from_cadquery(_base_trim(), "base_trim"),
        material=trim_dark,
        name="base_trim",
    )
    body.visual(
        Box((0.014, 0.102, 0.050)),
        origin=Origin(xyz=(0.153, 0.0, 0.055)),
        material=trim_dark,
        name="control_panel",
    )
    body.visual(
        Box((0.003, 0.050, 0.018)),
        origin=Origin(xyz=(0.161, 0.0, 0.067)),
        material=display_black,
        name="display_window",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(HINGE_X, -0.045, HINGE_Z - 0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_knuckle_0",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.028),
        origin=Origin(xyz=(HINGE_X, 0.045, HINGE_Z - 0.001), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_knuckle_1",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shell(), "lid_shell"),
        origin=Origin(xyz=(0.135, 0.0, 0.002)),
        material=lid_silver,
        name="lid_shell",
    )
    lid.visual(
        mesh_from_cadquery(_front_handle(), "front_handle"),
        origin=Origin(xyz=(0.238, 0.0, 0.026)),
        material=trim_dark,
        name="front_handle",
    )
    lid.visual(
        mesh_from_cadquery(_vent_collar(), "vent_collar"),
        origin=Origin(xyz=(VENT_CENTER_X, 0.0, 0.035)),
        material=trim_dark,
        name="vent_collar",
    )
    lid.visual(
        Cylinder(radius=0.007, length=0.052),
        origin=Origin(xyz=(-0.004, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    lid.visual(
        Cylinder(radius=0.0032, length=0.008),
        origin=Origin(xyz=(VENT_JOINT_X, -0.012, VENT_JOINT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="vent_knuckle_0",
    )
    lid.visual(
        Cylinder(radius=0.0032, length=0.008),
        origin=Origin(xyz=(VENT_JOINT_X, 0.012, VENT_JOINT_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="vent_knuckle_1",
    )

    vent_cap = model.part("vent_cap")
    vent_cap.visual(
        mesh_from_cadquery(_cap_shell(), "cap_shell"),
        material=lid_silver,
        name="cap_shell",
    )
    vent_cap.visual(
        Cylinder(radius=0.003, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="cap_barrel",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "lid_to_vent_cap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=vent_cap,
        origin=Origin(xyz=(VENT_JOINT_X, 0.0, VENT_JOINT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    vent_cap = object_model.get_part("vent_cap")
    lid_hinge = object_model.get_articulation("body_to_lid")
    vent_hinge = object_model.get_articulation("lid_to_vent_cap")

    with ctx.pose({lid_hinge: 0.0, vent_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="housing_shell",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed lid sits just above the housing rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="housing_shell",
            min_overlap=0.22,
            name="closed lid covers the housing opening",
        )
        ctx.expect_overlap(
            vent_cap,
            lid,
            axes="xy",
            elem_a="cap_shell",
            elem_b="vent_collar",
            min_overlap=0.020,
            name="closed cap covers the steam vent",
        )
        ctx.expect_gap(
            vent_cap,
            lid,
            axis="z",
            positive_elem="cap_shell",
            negative_elem="vent_collar",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed cap rests close to the vent collar",
        )

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits is not None else None
    if lid_upper is not None:
        handle_rest_z = None
        handle_open_z = None
        with ctx.pose({lid_hinge: 0.0}):
            handle_rest_z = _element_center_z(ctx, "lid", "front_handle")
        with ctx.pose({lid_hinge: lid_upper}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="front_handle",
                negative_elem="housing_shell",
                min_gap=0.09,
                name="opened lid lifts the front handle well above the cooker body",
            )
            handle_open_z = _element_center_z(ctx, "lid", "front_handle")
        ctx.check(
            "lid opens upward",
            handle_rest_z is not None
            and handle_open_z is not None
            and handle_open_z > handle_rest_z + 0.10,
            details=f"rest_z={handle_rest_z}, open_z={handle_open_z}",
        )

    vent_upper = vent_hinge.motion_limits.upper if vent_hinge.motion_limits is not None else None
    if vent_upper is not None:
        cap_rest_z = None
        cap_open_z = None
        with ctx.pose({vent_hinge: 0.0}):
            cap_rest_z = _element_center_z(ctx, "vent_cap", "cap_shell")
        with ctx.pose({vent_hinge: vent_upper}):
            ctx.expect_gap(
                vent_cap,
                lid,
                axis="z",
                positive_elem="cap_shell",
                negative_elem="vent_collar",
                min_gap=0.002,
                name="opened vent cap lifts clear of the vent collar",
            )
            cap_open_z = _element_center_z(ctx, "vent_cap", "cap_shell")
        ctx.check(
            "vent cap opens upward",
            cap_rest_z is not None and cap_open_z is not None and cap_open_z > cap_rest_z + 0.015,
            details=f"rest_z={cap_rest_z}, open_z={cap_open_z}",
        )

    return ctx.report()


object_model = build_object_model()
