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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


BASE_RADIUS = 0.23
BASE_THICKNESS = 0.055
POST_RADIUS = 0.018
POST_TOP_Z = 1.32
BOOM_PIVOT_Z = POST_TOP_Z
BOOM_REAR = 0.39
BOOM_FRONT = 0.82
SHADE_PIVOT_X = 0.90


def _shade_shell_geometry() -> LatheGeometry:
    """Thin spun-metal conical shade, open at the broad lower mouth."""

    outer_profile = [
        (0.047, -0.120),
        (0.064, -0.155),
        (0.118, -0.270),
        (0.165, -0.365),
    ]
    inner_profile = [
        (0.039, -0.126),
        (0.056, -0.159),
        (0.110, -0.272),
        (0.155, -0.356),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="counterbalance_drafting_floor_lamp")

    model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("brushed_steel", rgba=(0.50, 0.52, 0.54, 1.0))
    model.material("dark_hardware", rgba=(0.07, 0.07, 0.075, 1.0))
    model.material("shade_enamel", rgba=(0.90, 0.88, 0.80, 1.0))
    model.material("warm_glass", rgba=(1.0, 0.84, 0.42, 0.72))

    stand = model.part("stand")
    stand.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="matte_black",
        name="round_base",
    )
    post_upper_z = POST_TOP_Z - 0.075
    stand.visual(
        Cylinder(radius=POST_RADIUS, length=post_upper_z - BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, (post_upper_z + BASE_THICKNESS) / 2.0)),
        material="brushed_steel",
        name="tall_post",
    )
    stand.visual(
        Cylinder(radius=0.034, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + 0.0275)),
        material="dark_hardware",
        name="base_collar",
    )
    stand.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z - 0.085)),
        material="dark_hardware",
        name="top_collar",
    )
    stand.visual(
        Box((0.105, 0.120, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, POST_TOP_Z - 0.060)),
        material="dark_hardware",
        name="yoke_bridge",
    )
    for index, y in enumerate((-0.052, 0.052)):
        stand.visual(
            Box((0.105, 0.012, 0.128)),
            origin=Origin(xyz=(0.0, y, POST_TOP_Z)),
            material="dark_hardware",
            name=f"post_yoke_{index}",
        )
        stand.visual(
            Cylinder(radius=0.038, length=0.007),
            origin=Origin(
                xyz=(0.0, y + (0.009 if y > 0.0 else -0.009), POST_TOP_Z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material="brushed_steel",
            name=f"pivot_boss_{index}",
        )

    boom = model.part("boom")
    boom.visual(
        Cylinder(radius=0.012, length=BOOM_REAR + BOOM_FRONT),
        origin=Origin(
            xyz=((BOOM_FRONT - BOOM_REAR) / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="brushed_steel",
        name="boom_tube",
    )
    boom.visual(
        Cylinder(radius=0.036, length=0.092),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_hardware",
        name="pivot_barrel",
    )
    boom.visual(
        Cylinder(radius=0.058, length=0.165),
        origin=Origin(
            xyz=(-BOOM_REAR - 0.045, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material="matte_black",
        name="counterweight",
    )
    boom.visual(
        Box((0.016, 0.100, 0.026)),
        origin=Origin(xyz=(BOOM_FRONT - 0.015, 0.0, 0.0)),
        material="dark_hardware",
        name="end_bridge",
    )
    for index, y in enumerate((-0.046, 0.046)):
        boom.visual(
            Box((0.110, 0.012, 0.070)),
            origin=Origin(xyz=(SHADE_PIVOT_X - 0.050, y, 0.0)),
            material="dark_hardware",
            name=f"shade_yoke_{index}",
        )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.024, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material="dark_hardware",
        name="shade_barrel",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, -0.0625)),
        material="dark_hardware",
        name="shade_stem",
    )
    shade.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.120)),
        material="dark_hardware",
        name="top_cap",
    )
    shade.visual(
        mesh_from_geometry(_shade_shell_geometry(), "spun_shade"),
        origin=Origin(),
        material="shade_enamel",
        name="shade_shell",
    )
    shade.visual(
        Cylinder(radius=0.026, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
        material="dark_hardware",
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.235)),
        material="warm_glass",
        name="bulb",
    )

    model.articulation(
        "stand_to_boom",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=boom,
        origin=Origin(xyz=(0.0, 0.0, BOOM_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.0, lower=-0.65, upper=0.75),
    )
    model.articulation(
        "boom_to_shade",
        ArticulationType.REVOLUTE,
        parent=boom,
        child=shade,
        origin=Origin(xyz=(SHADE_PIVOT_X, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.8, lower=-0.95, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    stand = object_model.get_part("stand")
    boom = object_model.get_part("boom")
    shade = object_model.get_part("shade")
    boom_joint = object_model.get_articulation("stand_to_boom")
    shade_joint = object_model.get_articulation("boom_to_shade")

    ctx.check("floor lamp has stand boom and shade", all((stand, boom, shade)))
    ctx.check(
        "boom pivots on a horizontal revolute joint",
        boom_joint is not None
        and boom_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(boom_joint.axis[1]) > 0.95,
        details=f"joint={boom_joint}",
    )
    ctx.check(
        "shade rotates on a horizontal revolute joint",
        shade_joint is not None
        and shade_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(shade_joint.axis[1]) > 0.95,
        details=f"joint={shade_joint}",
    )
    if stand is None or boom is None or shade is None or boom_joint is None or shade_joint is None:
        return ctx.report()

    with ctx.pose({boom_joint: 0.0, shade_joint: 0.0}):
        ctx.expect_gap(
            boom,
            stand,
            axis="z",
            max_penetration=0.0,
            positive_elem="pivot_barrel",
            negative_elem="yoke_bridge",
            name="boom pivot clears the post-top bridge",
        )
        ctx.expect_gap(
            shade,
            boom,
            axis="x",
            min_gap=0.002,
            positive_elem="shade_barrel",
            negative_elem="end_bridge",
            name="shade hinge sits just beyond the boom end bridge",
        )
        rest_boom = ctx.part_element_world_aabb(boom, elem="boom_tube")
        rest_shade = ctx.part_element_world_aabb(shade, elem="shade_shell")

    with ctx.pose({boom_joint: boom_joint.motion_limits.upper, shade_joint: 0.0}):
        raised_boom = ctx.part_element_world_aabb(boom, elem="boom_tube")

    with ctx.pose({boom_joint: 0.0, shade_joint: shade_joint.motion_limits.upper}):
        tilted_shade = ctx.part_element_world_aabb(shade, elem="shade_shell")

    ctx.check(
        "upper boom limit lifts the boom end",
        rest_boom is not None
        and raised_boom is not None
        and (raised_boom[1][2] - rest_boom[1][2]) > 0.20,
        details=f"rest={rest_boom}, raised={raised_boom}",
    )
    ctx.check(
        "shade joint visibly tilts the conical shade",
        rest_shade is not None
        and tilted_shade is not None
        and abs(
            ((tilted_shade[0][0] + tilted_shade[1][0]) * 0.5)
            - ((rest_shade[0][0] + rest_shade[1][0]) * 0.5)
        )
        > 0.08,
        details=f"rest={rest_shade}, tilted={tilted_shade}",
    )

    return ctx.report()


object_model = build_object_model()
