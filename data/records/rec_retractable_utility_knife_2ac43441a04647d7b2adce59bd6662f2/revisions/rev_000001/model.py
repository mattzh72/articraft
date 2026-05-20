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


def build_handle():
    handle_solid = (
        cq.Workplane("XZ")
        .moveTo(-0.075, -0.018)
        .lineTo(0.075, -0.018)
        .lineTo(0.080, -0.014)
        .lineTo(0.080, 0.014)
        .lineTo(0.065, 0.018)
        .lineTo(-0.070, 0.018)
        .lineTo(-0.080, 0.008)
        .close()
        .extrude(0.010, both=True)
        .edges("|Y")
        .fillet(0.002)
    )

    cavity = (
        cq.Workplane("XZ")
        .moveTo(-0.072, -0.015)
        .lineTo(0.073, -0.015)
        .lineTo(0.085, -0.009)
        .lineTo(0.085, 0.009)
        .lineTo(0.063, 0.015)
        .lineTo(-0.068, 0.015)
        .lineTo(-0.076, 0.007)
        .close()
        .extrude(0.007, both=True)
    )

    slot = (
        cq.Workplane("XY")
        .workplane(offset=0.015)
        .center(0.020, 0)
        .rect(0.060, 0.006)
        .extrude(0.020, both=True)
    )

    handle = handle_solid.cut(cavity).cut(slot)
    return handle


def build_carrier():
    carrier_body = (
        cq.Workplane("XZ")
        .moveTo(-0.020, -0.012)
        .lineTo(0.020, -0.012)
        .lineTo(0.020, 0.012)
        .lineTo(-0.020, 0.012)
        .close()
        .extrude(0.006, both=True)
    )

    thumb_button = (
        cq.Workplane("XZ")
        .center(0, 0.014)
        .rect(0.012, 0.010)
        .extrude(0.0025, both=True)
    )
    
    thumb_button_top = (
        cq.Workplane("XZ")
        .center(0, 0.020)
        .rect(0.020, 0.004)
        .extrude(0.006, both=True)
        .edges("|Y")
        .fillet(0.001)
    )

    blade = (
        cq.Workplane("XZ")
        .moveTo(0.010, -0.008)
        .lineTo(0.065, -0.008)
        .lineTo(0.065, -0.002)
        .lineTo(0.045, 0.008)
        .lineTo(0.010, 0.008)
        .close()
        .extrude(0.0005, both=True)
    )

    carrier = carrier_body.union(thumb_button).union(thumb_button_top).union(blade)
    return carrier


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_knife")

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(build_handle(), "handle_shell"),
        name="shell"
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        mesh_from_cadquery(build_carrier(), "carrier_body"),
        name="carrier"
    )

    model.articulation(
        name="blade_slide",
        articulation_type=ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=0.0, upper=0.040)
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle")
    carrier = object_model.get_part("blade_carrier")
    slide = object_model.get_articulation("blade_slide")

    ctx.allow_overlap(
        handle, carrier,
        reason="The blade carrier slides inside the handle cavity and the thumb button rides in the top slot."
    )

    ctx.expect_within(
        carrier, handle,
        axes="y",
        margin=0.001,
        name="carrier stays centered in handle on Y"
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            carrier, handle,
            axes="x",
            margin=0.001,
            name="retracted blade is fully inside the handle"
        )

    with ctx.pose({slide: 0.040}):
        ctx.expect_overlap(
            carrier, handle,
            axes="x",
            min_overlap=0.040,
            name="extended carrier retains insertion in the handle"
        )

    return ctx.report()


object_model = build_object_model()