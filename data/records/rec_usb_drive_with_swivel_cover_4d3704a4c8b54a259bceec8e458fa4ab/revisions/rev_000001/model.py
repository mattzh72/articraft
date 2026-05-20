from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="usb_swivel_drive")

    plastic_black = model.material("plastic_black", rgba=(0.14, 0.14, 0.15, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.77, 0.79, 0.81, 1.0))
    connector_steel = model.material("connector_steel", rgba=(0.70, 0.72, 0.74, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.044, 0.017, 0.008)),
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
        material=plastic_black,
        name="body_shell",
    )
    body.visual(
        Box((0.012, 0.0122, 0.0046)),
        origin=Origin(xyz=(0.054, 0.0, 0.0)),
        material=connector_steel,
        name="usb_plug",
    )
    body.visual(
        Cylinder(radius=0.0035, length=0.0204),
        origin=Origin(xyz=(0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pivot_sleeve",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.060, 0.020, 0.010)),
        mass=0.022,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    cover = model.part("cover")
    cover.visual(
        Box((0.0574, 0.0012, 0.011)),
        origin=Origin(xyz=(0.0287, 0.0108, 0.0)),
        material=satin_steel,
        name="left_arm",
    )
    cover.visual(
        Box((0.0574, 0.0012, 0.011)),
        origin=Origin(xyz=(0.0287, -0.0108, 0.0)),
        material=satin_steel,
        name="right_arm",
    )
    cover.visual(
        Box((0.0012, 0.0228, 0.011)),
        origin=Origin(xyz=(0.0568, 0.0, 0.0)),
        material=satin_steel,
        name="front_bridge",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=0.0012),
        origin=Origin(xyz=(0.0039, 0.0108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="left_pivot_tab",
    )
    cover.visual(
        Cylinder(radius=0.0042, length=0.0012),
        origin=Origin(xyz=(0.0039, -0.0108, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="right_pivot_tab",
    )
    cover.inertial = Inertial.from_geometry(
        Box((0.062, 0.023, 0.011)),
        mass=0.010,
        origin=Origin(xyz=(0.031, 0.0, 0.0)),
    )

    model.articulation(
        "swivel_cover",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cover,
        origin=Origin(xyz=(0.004, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    cover = object_model.get_part("cover")
    swivel = object_model.get_articulation("swivel_cover")

    with ctx.pose({swivel: 0.0}):
        ctx.expect_gap(
            cover,
            body,
            axis="x",
            positive_elem="front_bridge",
            negative_elem="usb_plug",
            min_gap=0.0001,
            max_gap=0.0025,
            name="closed bridge sits just ahead of the connector",
        )
        ctx.expect_overlap(
            cover,
            body,
            axes="x",
            elem_a="left_arm",
            elem_b="body_shell",
            min_overlap=0.040,
            name="left arm runs alongside the body in the closed pose",
        )

    with ctx.pose({swivel: math.pi}):
        ctx.expect_gap(
            body,
            cover,
            axis="x",
            positive_elem="body_shell",
            negative_elem="front_bridge",
            min_gap=0.040,
            name="half turn swings the cover bridge behind the body",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
