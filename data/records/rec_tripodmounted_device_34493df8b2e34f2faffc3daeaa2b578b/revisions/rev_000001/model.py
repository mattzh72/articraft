from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _cylinder_rpy_from_z(direction: tuple[float, float, float]) -> tuple[float, float, float]:
    """Return an rpy that aims a primitive cylinder's local +Z along direction."""
    dx, dy, dz = direction
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        return (0.0, 0.0, 0.0)
    dx, dy, dz = dx / length, dy / length, dz / length
    roll = math.asin(-dy)
    pitch = math.atan2(dx, dz)
    return (roll, pitch, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_binocular_tripod")

    black = model.material("matte_black", rgba=(0.015, 0.017, 0.018, 1.0))
    dark = model.material("dark_graphite", rgba=(0.08, 0.085, 0.09, 1.0))
    olive = model.material("field_olive", rgba=(0.18, 0.22, 0.15, 1.0))
    metal = model.material("brushed_metal", rgba=(0.55, 0.55, 0.50, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    glass = model.material("blue_coated_glass", rgba=(0.12, 0.28, 0.42, 0.72))

    crown = model.part("crown")
    crown.visual(
        Cylinder(radius=0.135, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.045)),
        material=dark,
        name="hub",
    )
    crown.visual(
        Cylinder(radius=0.038, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 1.130)),
        material=metal,
        name="center_post",
    )
    crown.visual(
        Cylinder(radius=0.092, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 1.1825)),
        material=dark,
        name="top_plate",
    )

    hinge_radius = 0.25
    hinge_z = 1.0
    leg_length = 1.10
    leg_out = 0.455
    leg_down = -0.891
    leg_axis_z = _cylinder_rpy_from_z((0.0, 1.0, 0.0))

    legs = []
    for i in range(3):
        theta = 2.0 * math.pi * i / 3.0 + math.pi / 2.0
        radial = (math.cos(theta), math.sin(theta), 0.0)
        tangent = (-math.sin(theta), math.cos(theta), 0.0)
        yaw = theta

        crown.visual(
            Box((hinge_radius + 0.02, 0.060, 0.050)),
            origin=Origin(
                xyz=(radial[0] * hinge_radius * 0.5, radial[1] * hinge_radius * 0.5, 1.055),
                rpy=(0.0, 0.0, yaw),
            ),
            material=dark,
            name=f"hinge_arm_{i}",
        )

        leg = model.part(f"leg_{i}")
        legs.append((leg, radial, tangent))

        leg.visual(
            Cylinder(radius=0.030, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=leg_axis_z),
            material=metal,
            name="hinge_knuckle",
        )

        leg_direction = (
            radial[0] * leg_out,
            radial[1] * leg_out,
            leg_down,
        )
        leg.visual(
            Cylinder(radius=0.024, length=leg_length),
            origin=Origin(
                xyz=(
                    leg_direction[0] * leg_length * 0.5,
                    leg_direction[1] * leg_length * 0.5,
                    leg_direction[2] * leg_length * 0.5,
                ),
                rpy=_cylinder_rpy_from_z(leg_direction),
            ),
            material=olive,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.055, length=0.030),
            origin=Origin(
                xyz=(
                    leg_direction[0] * leg_length,
                    leg_direction[1] * leg_length,
                    leg_direction[2] * leg_length - 0.006,
                )
            ),
            material=rubber,
            name="foot_pad",
        )

        model.articulation(
            f"leg_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=crown,
            child=leg,
            origin=Origin(xyz=(radial[0] * hinge_radius, radial[1] * hinge_radius, hinge_z)),
            axis=tangent,
            motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-0.35, upper=0.45),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.120, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark,
        name="pan_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.043, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=metal,
        name="pan_spindle",
    )
    pan_head.visual(
        Box((0.170, 0.370, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=dark,
        name="bracket_base",
    )
    for y, name in ((0.170, "bracket_cheek_0"), (-0.170, "bracket_cheek_1")):
        pan_head.visual(
            Box((0.080, 0.035, 0.170)),
            origin=Origin(xyz=(0.0, y, 0.275)),
            material=dark,
            name=name,
        )
    for y, name in ((0.170, "hinge_boss_0"), (-0.170, "hinge_boss_1")):
        pan_head.visual(
            Cylinder(radius=0.032, length=0.040),
            origin=Origin(xyz=(0.0, y, 0.340), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=metal,
            name=name,
        )

    model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=crown,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
    )

    viewer = model.part("viewer")
    x_tube_rpy = (0.0, math.pi / 2.0, 0.0)
    y_tube_rpy = (-math.pi / 2.0, 0.0, 0.0)
    for y, suffix in ((0.075, "0"), (-0.075, "1")):
        viewer.visual(
            Cylinder(radius=0.052, length=0.460),
            origin=Origin(xyz=(0.040, y, 0.035), rpy=x_tube_rpy),
            material=black,
            name=f"barrel_{suffix}",
        )
        viewer.visual(
            Cylinder(radius=0.063, length=0.035),
            origin=Origin(xyz=(0.285, y, 0.035), rpy=x_tube_rpy),
            material=black,
            name=f"objective_ring_{suffix}",
        )
        viewer.visual(
            Cylinder(radius=0.048, length=0.006),
            origin=Origin(xyz=(0.301, y, 0.035), rpy=x_tube_rpy),
            material=glass,
            name=f"objective_glass_{suffix}",
        )
        viewer.visual(
            Cylinder(radius=0.038, length=0.055),
            origin=Origin(xyz=(-0.205, y, 0.035), rpy=x_tube_rpy),
            material=rubber,
            name=f"eyepiece_{suffix}",
        )
        viewer.visual(
            Cylinder(radius=0.026, length=0.006),
            origin=Origin(xyz=(-0.235, y, 0.035), rpy=x_tube_rpy),
            material=glass,
            name=f"eye_glass_{suffix}",
        )

    viewer.visual(
        Box((0.270, 0.120, 0.090)),
        origin=Origin(xyz=(0.025, 0.0, 0.035)),
        material=black,
        name="center_bridge",
    )
    viewer.visual(
        Box((0.160, 0.190, 0.055)),
        origin=Origin(xyz=(-0.020, 0.0, 0.095)),
        material=black,
        name="prism_hump",
    )
    viewer.visual(
        Cylinder(radius=0.022, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=y_tube_rpy),
        material=metal,
        name="tilt_trunnion",
    )

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=viewer,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-0.45, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    crown = object_model.get_part("crown")
    pan_head = object_model.get_part("pan_head")
    viewer = object_model.get_part("viewer")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_hinge = object_model.get_articulation("tilt_hinge")

    ctx.expect_gap(
        pan_head,
        crown,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="pan_disk",
        negative_elem="top_plate",
        name="pan head sits on crown bearing",
    )
    ctx.expect_contact(
        viewer,
        pan_head,
        elem_a="tilt_trunnion",
        elem_b="hinge_boss_0",
        contact_tol=0.001,
        name="viewer trunnion meets first hinge boss",
    )
    ctx.expect_contact(
        viewer,
        pan_head,
        elem_a="tilt_trunnion",
        elem_b="hinge_boss_1",
        contact_tol=0.001,
        name="viewer trunnion meets second hinge boss",
    )

    at_rest = ctx.part_element_world_aabb(viewer, elem="objective_glass_0")
    with ctx.pose({tilt_hinge: 0.50}):
        tilted = ctx.part_element_world_aabb(viewer, elem="objective_glass_0")
    ctx.check(
        "tilt hinge raises objective end",
        at_rest is not None
        and tilted is not None
        and ((tilted[0][2] + tilted[1][2]) * 0.5) > ((at_rest[0][2] + at_rest[1][2]) * 0.5) + 0.08,
        details=f"rest={at_rest}, tilted={tilted}",
    )

    with ctx.pose({pan_axis: math.pi / 2.0}):
        spun = ctx.part_element_world_aabb(viewer, elem="objective_glass_0")
    ctx.check(
        "continuous pan swings viewer around vertical axis",
        at_rest is not None
        and spun is not None
        and abs(((spun[0][1] + spun[1][1]) * 0.5) - ((at_rest[0][0] + at_rest[1][0]) * 0.5)) < 0.08,
        details=f"rest={at_rest}, spun={spun}",
    )

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        hinge = object_model.get_articulation(f"leg_hinge_{i}")
        foot_rest = ctx.part_element_world_aabb(leg, elem="foot_pad")
        with ctx.pose({hinge: -0.25}):
            foot_splayed = ctx.part_element_world_aabb(leg, elem="foot_pad")
        ctx.check(
            f"leg {i} rotates on crown hinge",
            foot_rest is not None
            and foot_splayed is not None
            and abs(((foot_splayed[0][2] + foot_splayed[1][2]) * 0.5) - ((foot_rest[0][2] + foot_rest[1][2]) * 0.5)) > 0.05,
            details=f"rest={foot_rest}, splayed={foot_splayed}",
        )

    return ctx.report()


object_model = build_object_model()
