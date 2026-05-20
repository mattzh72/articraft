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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_X = 0.34
BODY_Y = 0.28
BODY_Z = 0.128
TOP_RECESS_DEPTH = 0.008
PEDESTAL_RADIUS = 0.060
PEDESTAL_HEIGHT = 0.012

CARTRIDGE_RADIUS = 0.053
CARTRIDGE_HEIGHT = 0.040
PLATE_RADIUS = 0.100
PLATE_THICKNESS = 0.014
PLATE_BOTTOM_Z = 0.036
HUB_RADIUS = 0.036
HUB_HEIGHT = 0.018
PLATE_BOLT_CIRCLE = 0.066

TAB_X = 0.028
TAB_Y = 0.018
TAB_Z = 0.010
TAB_CENTER_X = 0.072
TAB_CENTER_Z = PLATE_BOTTOM_Z + PLATE_THICKNESS + (TAB_Z * 0.5) - 0.002


def _build_body_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(BODY_X, BODY_Y, BODY_Z)
        .translate((0.0, 0.0, BODY_Z * 0.5))
        .edges("|Z")
        .fillet(0.012)
    )
    recess = (
        cq.Workplane("XY")
        .box(BODY_X - 0.060, BODY_Y - 0.060, TOP_RECESS_DEPTH)
        .translate((0.0, 0.0, BODY_Z - (TOP_RECESS_DEPTH * 0.5)))
    )
    recess_keepout = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS + 0.010)
        .extrude(TOP_RECESS_DEPTH + 0.004)
        .translate((0.0, 0.0, BODY_Z - TOP_RECESS_DEPTH - 0.002))
    )
    body = body.cut(recess.cut(recess_keepout))
    body = (
        body.faces(">Z")
        .workplane()
        .polarArray(0.082, 45.0, 360.0, 4)
        .circle(0.0045)
        .cutBlind(-0.006)
    )
    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BODY_Z))
    )
    body = body.union(pedestal)
    body = (
        body.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(0.120, 0.050)
        .cutBlind(-0.004)
    )
    return body


def _build_output_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_BOTTOM_Z)
        .circle(PLATE_RADIUS)
        .extrude(PLATE_THICKNESS)
    )
    hub = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_BOTTOM_Z - 0.004)
        .circle(HUB_RADIUS)
        .extrude(HUB_HEIGHT)
    )
    holes = (
        cq.Workplane("XY")
        .workplane(offset=PLATE_BOTTOM_Z - 0.002)
        .polarArray(PLATE_BOLT_CIRCLE, 0.0, 360.0, 6)
        .circle(0.0045)
        .extrude(PLATE_THICKNESS + 0.006)
    )
    plate = plate.union(hub).cut(holes)
    plate = (
        plate.faces(">Z")
        .workplane()
        .circle(0.018)
        .cutBlind(-0.004)
    )
    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_service_yaw_stage")

    model.material("body_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_alloy", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("tab_black", rgba=(0.11, 0.12, 0.13, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "body_shell"),
        material="body_charcoal",
        name="body_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_X, BODY_Y, BODY_Z + PEDESTAL_HEIGHT)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, (BODY_Z + PEDESTAL_HEIGHT) * 0.5)),
    )

    output_stage = model.part("output_stage")
    output_stage.visual(
        Cylinder(radius=CARTRIDGE_RADIUS, length=CARTRIDGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, CARTRIDGE_HEIGHT * 0.5)),
        material="machined_alloy",
        name="rotary_cartridge",
    )
    output_stage.visual(
        mesh_from_cadquery(_build_output_plate_shape(), "output_plate"),
        material="machined_alloy",
        name="output_plate",
    )
    output_stage.visual(
        Box((TAB_X, TAB_Y, TAB_Z)),
        origin=Origin(xyz=(TAB_CENTER_X, 0.0, TAB_CENTER_Z)),
        material="tab_black",
        name="service_tab",
    )
    output_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=PLATE_RADIUS, length=PLATE_BOTTOM_Z + PLATE_THICKNESS),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, (PLATE_BOTTOM_Z + PLATE_THICKNESS) * 0.5)),
    )

    model.articulation(
        "body_to_output_stage",
        ArticulationType.REVOLUTE,
        parent=body,
        child=output_stage,
        origin=Origin(xyz=(0.0, 0.0, BODY_Z + PEDESTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=2.2,
            lower=-2.8,
            upper=2.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    output_stage = object_model.get_part("output_stage")
    yaw = object_model.get_articulation("body_to_output_stage")

    limits = yaw.motion_limits
    ctx.check(
        "yaw axis is vertical revolute",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and yaw.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"type={yaw.articulation_type}, axis={yaw.axis}, limits={limits}",
    )

    with ctx.pose({yaw: 0.0}):
        ctx.expect_contact(
            output_stage,
            body,
            elem_a="rotary_cartridge",
            elem_b="body_shell",
            name="cartridge seats on the body pedestal",
        )
        ctx.expect_gap(
            output_stage,
            body,
            axis="z",
            positive_elem="output_plate",
            negative_elem="body_shell",
            min_gap=0.020,
            max_gap=0.080,
            name="output plate stays clearly above the boxed body",
        )
        ctx.expect_overlap(
            output_stage,
            body,
            axes="xy",
            elem_a="rotary_cartridge",
            elem_b="body_shell",
            min_overlap=0.090,
            name="cartridge remains centered over the body pedestal",
        )

    rest_tab_aabb = ctx.part_element_world_aabb(output_stage, elem="service_tab")
    with ctx.pose({yaw: pi * 0.5}):
        turned_tab_aabb = ctx.part_element_world_aabb(output_stage, elem="service_tab")
        ctx.expect_gap(
            output_stage,
            body,
            axis="z",
            positive_elem="output_plate",
            negative_elem="body_shell",
            min_gap=0.020,
            max_gap=0.080,
            name="output plate clears the body after a quarter turn",
        )

    if rest_tab_aabb is None or turned_tab_aabb is None:
        ctx.fail("service tab pose observable", "service tab AABB was unavailable")
    else:
        rest_center = tuple(
            0.5 * (rest_tab_aabb[0][i] + rest_tab_aabb[1][i]) for i in range(3)
        )
        turned_center = tuple(
            0.5 * (turned_tab_aabb[0][i] + turned_tab_aabb[1][i]) for i in range(3)
        )
        ctx.check(
            "service tab rotates around the vertical axis",
            rest_center[0] > 0.055
            and abs(rest_center[1]) < 0.015
            and abs(turned_center[0]) < 0.020
            and turned_center[1] > 0.055,
            details=f"rest_center={rest_center}, turned_center={turned_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
