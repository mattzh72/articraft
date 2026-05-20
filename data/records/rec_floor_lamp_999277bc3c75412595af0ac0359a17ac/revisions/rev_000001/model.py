from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arc_floor_lamp")

    dark_base = model.material("dark_weighted_base", rgba=(0.025, 0.025, 0.023, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.004, 1.0))
    brushed_metal = model.material("brushed_steel", rgba=(0.58, 0.55, 0.50, 1.0))
    warm_white = model.material("warm_white_shade", rgba=(0.92, 0.88, 0.78, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.83, 0.46, 0.78))

    base = model.part("base")
    base_profile = [
        (0.000, 0.000),
        (0.305, 0.000),
        (0.340, 0.012),
        (0.345, 0.055),
        (0.310, 0.074),
        (0.000, 0.074),
    ]
    base.visual(
        mesh_from_geometry(LatheGeometry(base_profile, segments=96), "beveled_weighted_base"),
        material=dark_base,
        name="weighted_disc",
    )
    base.visual(
        Cylinder(radius=0.034, length=1.185),
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        material=brushed_metal,
        name="upright_column",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.185)),
        material=brushed_metal,
        name="column_collar",
    )
    base.visual(
        Box((0.135, 0.166, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 1.247)),
        material=brushed_metal,
        name="yoke_bridge",
    )
    base.visual(
        Box((0.112, 0.022, 0.160)),
        origin=Origin(xyz=(0.0, -0.071, 1.320)),
        material=brushed_metal,
        name="near_yoke_cheek",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.0, -0.0845, 1.310), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="near_pivot_cap",
    )
    base.visual(
        Box((0.112, 0.022, 0.160)),
        origin=Origin(xyz=(0.0, 0.071, 1.320)),
        material=brushed_metal,
        name="far_yoke_cheek",
    )
    base.visual(
        Cylinder(radius=0.035, length=0.012),
        origin=Origin(xyz=(0.0, 0.0845, 1.310), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="far_pivot_cap",
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.045, length=0.120),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_metal,
        name="pivot_barrel",
    )
    arm_points = [
        (0.000, 0.0, 0.000),
        (0.210, 0.0, 0.270),
        (0.760, 0.0, 0.470),
        (1.330, 0.0, 0.270),
        (1.620, 0.0, -0.110),
    ]
    arm.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                arm_points,
                radius=0.019,
                samples_per_segment=24,
                radial_segments=28,
                cap_ends=True,
            ),
            "swept_arc_arm",
        ),
        material=brushed_metal,
        name="arm_tube",
    )

    shade = model.part("shade")
    shade_profile = [
        (0.000, -0.085),
        (0.030, -0.085),
        (0.030, -0.161),
        (0.055, -0.185),
        (0.235, -0.197),
        (0.235, -0.239),
        (0.205, -0.251),
        (0.000, -0.251),
    ]
    shade.visual(
        Cylinder(radius=0.022, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.0425)),
        material=brushed_metal,
        name="shade_stem",
    )
    shade.visual(
        mesh_from_geometry(LatheGeometry(shade_profile, segments=96), "flat_disc_shade"),
        material=warm_white,
        name="shade_body",
    )
    shade.visual(
        Cylinder(radius=0.182, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.256)),
        material=warm_diffuser,
        name="underside_diffuser",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 1.310)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.28, upper=0.42),
    )
    model.articulation(
        "arm_to_shade",
        ArticulationType.FIXED,
        parent=arm,
        child=shade,
        origin=Origin(xyz=(1.620, 0.0, -0.110)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    shade = object_model.get_part("shade")
    tilt = object_model.get_articulation("base_to_arm")

    ctx.allow_overlap(
        arm,
        shade,
        elem_a="arm_tube",
        elem_b="shade_stem",
        reason="The shade hanger stem is intentionally seated into the capped end of the arc tube.",
    )

    ctx.expect_overlap(
        arm,
        base,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="near_yoke_cheek",
        min_overlap=0.055,
        name="pivot barrel sits inside the base-top yoke",
    )
    ctx.expect_contact(
        shade,
        arm,
        elem_a="shade_stem",
        elem_b="arm_tube",
        contact_tol=0.012,
        name="shade hanger stem is seated at the arc tube end",
    )

    at_rest = ctx.part_world_position(shade)
    with ctx.pose({tilt: 0.30}):
        tilted_up = ctx.part_world_position(shade)
    ctx.check(
        "positive tilt raises the arc end",
        at_rest is not None and tilted_up is not None and tilted_up[2] > at_rest[2] + 0.08,
        details=f"rest={at_rest}, tilted={tilted_up}",
    )

    return ctx.report()


object_model = build_object_model()
