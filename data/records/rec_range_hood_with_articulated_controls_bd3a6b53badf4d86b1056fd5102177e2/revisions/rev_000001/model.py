from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chimney_range_hood")

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.19, 0.20, 1.0))
    control_black = model.material("control_black", rgba=(0.10, 0.10, 0.11, 1.0))

    canopy_width = 0.90
    canopy_depth = 0.50
    canopy_height = 0.21
    fascia_depth = 0.02
    fascia_height = 0.085
    chimney_width = 0.34
    chimney_depth = 0.25
    chimney_height = 0.58

    hood_body = model.part("hood_body")
    hood_body.visual(
        Box((canopy_width, canopy_depth, canopy_height)),
        origin=Origin(xyz=(0.0, 0.0, canopy_height * 0.5)),
        material=stainless,
        name="canopy_shell",
    )
    hood_body.visual(
        Box((canopy_width, fascia_depth, fascia_height)),
        origin=Origin(
            xyz=(
                0.0,
                canopy_depth * 0.5 + fascia_depth * 0.5,
                0.060 + fascia_height * 0.5,
            )
        ),
        material=dark_trim,
        name="front_fascia",
    )
    hood_body.visual(
        Box((chimney_width, chimney_depth, chimney_height)),
        origin=Origin(
            xyz=(
                0.0,
                -0.075,
                canopy_height + chimney_height * 0.5,
            )
        ),
        material=stainless,
        name="chimney_shell",
    )
    hood_body.visual(
        Box((0.36, 0.27, 0.022)),
        origin=Origin(xyz=(0.0, -0.075, canopy_height + chimney_height + 0.011)),
        material=stainless,
        name="chimney_cap",
    )
    hood_body.visual(
        Box((0.055, 0.010, 0.012)),
        origin=Origin(xyz=(0.305, 0.1775, -0.005)),
        material=dark_trim,
        name="hinge_mount",
    )

    speed_dial = model.part("speed_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.044,
            0.024,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.050, 0.005, flare=0.06),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "range_hood_speed_dial",
    )
    speed_dial.visual(
        dial_mesh,
        origin=Origin(rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="dial_shell",
    )

    service_flap = model.part("service_flap")
    service_flap.visual(
        Box((0.145, 0.098, 0.010)),
        origin=Origin(xyz=(0.0, -0.049, -0.005)),
        material=stainless,
        name="flap_panel",
    )
    service_flap.visual(
        Box((0.072, 0.016, 0.012)),
        origin=Origin(xyz=(0.0, -0.106, -0.006)),
        material=dark_trim,
        name="pull_lip",
    )

    model.articulation(
        "body_to_speed_dial",
        ArticulationType.CONTINUOUS,
        parent=hood_body,
        child=speed_dial,
        origin=Origin(xyz=(-0.170, 0.270, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
    )
    model.articulation(
        "body_to_service_flap",
        ArticulationType.REVOLUTE,
        parent=hood_body,
        child=service_flap,
        origin=Origin(xyz=(0.305, 0.170, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood_body = object_model.get_part("hood_body")
    speed_dial = object_model.get_part("speed_dial")
    service_flap = object_model.get_part("service_flap")
    dial_joint = object_model.get_articulation("body_to_speed_dial")
    flap_joint = object_model.get_articulation("body_to_service_flap")

    body_aabb = ctx.part_world_aabb(hood_body)
    if body_aabb is not None:
        mins, maxs = body_aabb
        size = tuple(float(maxs[i] - mins[i]) for i in range(3))
        ctx.check(
            "domestic_wall_hood_scale",
            0.88 <= size[0] <= 0.92 and 0.50 <= size[1] <= 0.53 and 0.80 <= size[2] <= 0.84,
            details=f"size={size!r}",
        )
    else:
        ctx.fail("domestic_wall_hood_scale", "Expected a body AABB for the range hood.")

    ctx.check(
        "speed_dial_is_continuous_front_axis",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(dial_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={dial_joint.articulation_type!r}, axis={tuple(dial_joint.axis)!r}",
    )

    ctx.expect_gap(
        speed_dial,
        hood_body,
        axis="y",
        positive_elem="dial_shell",
        negative_elem="front_fascia",
        max_gap=0.001,
        max_penetration=0.0,
        name="speed dial seats on front fascia",
    )
    ctx.expect_gap(
        hood_body,
        service_flap,
        axis="z",
        positive_elem="canopy_shell",
        negative_elem="flap_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="service flap closes flush with underside",
    )

    closed_aabb = ctx.part_element_world_aabb(service_flap, elem="pull_lip")
    with ctx.pose({flap_joint: 1.0}):
        ctx.expect_gap(
            hood_body,
            service_flap,
            axis="z",
            positive_elem="canopy_shell",
            negative_elem="pull_lip",
            min_gap=0.030,
            name="service flap hangs below the hood when opened",
        )
        open_aabb = ctx.part_element_world_aabb(service_flap, elem="pull_lip")

    ctx.check(
        "service flap drops downward when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.050,
        details=f"closed={closed_aabb!r}, open={open_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
