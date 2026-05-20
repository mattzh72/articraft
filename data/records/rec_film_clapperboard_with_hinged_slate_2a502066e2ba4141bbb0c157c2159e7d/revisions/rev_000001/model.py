from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clapperboard")

    acrylic_black = model.material("acrylic_black", rgba=(0.05, 0.05, 0.06, 1.0))
    matte_face = model.material("matte_face", rgba=(0.08, 0.08, 0.09, 1.0))
    chalk_white = model.material("chalk_white", rgba=(0.92, 0.92, 0.90, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.79, 0.81, 1.0))
    stripe_white = model.material("stripe_white", rgba=(0.96, 0.96, 0.95, 1.0))

    slate = model.part("slate")

    panel_width = 0.280
    panel_height = 0.200
    panel_thickness = 0.005
    hinge_y = 0.005
    hinge_z = panel_height + 0.006

    slate.visual(
        Box((panel_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(0.0, 0.0, panel_height / 2.0)),
        material=acrylic_black,
        name="panel",
    )
    slate.visual(
        Box((0.252, 0.0012, 0.162)),
        origin=Origin(xyz=(0.0, (panel_thickness / 2.0) + 0.0004, 0.095)),
        material=matte_face,
        name="face",
    )

    for z_pos in (0.125, 0.091, 0.057):
        slate.visual(
            Box((0.220, 0.0006, 0.0022)),
            origin=Origin(xyz=(0.0, (panel_thickness / 2.0) + 0.00055, z_pos)),
            material=chalk_white,
            name=f"line_{int(round(z_pos * 1000.0))}",
        )
    for x_pos in (-0.052, 0.050):
        slate.visual(
            Box((0.0022, 0.0006, 0.070)),
            origin=Origin(xyz=(x_pos, (panel_thickness / 2.0) + 0.00055, 0.071)),
            material=chalk_white,
            name=f"divider_{'l' if x_pos < 0.0 else 'r'}",
        )

    slate.visual(
        Box((0.274, 0.003, 0.008)),
        origin=Origin(xyz=(0.0, -0.0010, panel_height + 0.003)),
        material=aluminum,
        name="rail_strip",
    )

    rail_knuckle_centers = (-0.110, 0.0, 0.110)
    for index, x_pos in enumerate(rail_knuckle_centers):
        slate.visual(
            Box((0.036, 0.004, 0.008)),
            origin=Origin(xyz=(x_pos, 0.0005, panel_height + 0.003)),
            material=aluminum,
            name=f"rail_saddle_{index}",
        )
        slate.visual(
            Cylinder(radius=0.003, length=0.036),
            origin=Origin(xyz=(x_pos, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=f"rail_knuckle_{index}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((0.286, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, 0.0041, -0.014)),
        material=acrylic_black,
        name="bar",
    )
    clapstick.visual(
        Box((0.252, 0.002, 0.004)),
        origin=Origin(xyz=(0.0, 0.0068, -0.026)),
        material=matte_face,
        name="striking_edge",
    )

    stick_knuckle_centers = (-0.055, 0.055)
    for index, x_pos in enumerate(stick_knuckle_centers):
        clapstick.visual(
            Box((0.046, 0.005, 0.006)),
            origin=Origin(xyz=(x_pos, 0.0035, -0.003)),
            material=acrylic_black,
            name=f"cheek_{index}",
        )
        clapstick.visual(
            Cylinder(radius=0.003, length=0.046),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name=f"stick_knuckle_{index}",
        )

    stripe_pitch = 0.056
    for index in range(5):
        clapstick.visual(
            Box((0.036, 0.0016, 0.010)),
            origin=Origin(
                xyz=((-2 + index) * stripe_pitch, 0.0070, -0.014),
                rpy=(0.0, 0.5, 0.0),
            ),
            material=stripe_white,
            name=f"stripe_{index}",
        )

    model.articulation(
        "slate_hinge",
        ArticulationType.REVOLUTE,
        parent=slate,
        child=clapstick,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    slate = object_model.get_part("slate")
    clapstick = object_model.get_part("clapstick")
    hinge = object_model.get_articulation("slate_hinge")
    limits = hinge.motion_limits

    ctx.expect_gap(
        clapstick,
        slate,
        axis="y",
        positive_elem="bar",
        negative_elem="face",
        max_gap=0.0025,
        max_penetration=0.0,
        name="closed clapstick sits just proud of the writing face",
    )
    ctx.expect_overlap(
        clapstick,
        slate,
        axes="x",
        elem_a="bar",
        elem_b="panel",
        min_overlap=0.240,
        name="clapstick spans the handheld slate width",
    )

    if limits is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.upper}):
            ctx.expect_gap(
                clapstick,
                slate,
                axis="y",
                positive_elem="striking_edge",
                negative_elem="face",
                min_gap=0.020,
                name="opened clapstick swings clear of the face",
            )

    return ctx.report()


object_model = build_object_model()
