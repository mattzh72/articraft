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
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    black_powder = model.material("black_powder", rgba=(0.10, 0.11, 0.12, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.74, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.12, 0.56, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_graphite,
        name="base_plinth",
    )
    frame.visual(
        Box((0.18, 0.30, 0.24)),
        origin=Origin(xyz=(-0.46, 0.0, 0.17)),
        material=brushed_steel,
        name="left_support_base",
    )
    frame.visual(
        Box((0.18, 0.30, 0.24)),
        origin=Origin(xyz=(0.46, 0.0, 0.17)),
        material=brushed_steel,
        name="right_support_base",
    )
    frame.visual(
        Box((0.10, 0.20, 1.03)),
        origin=Origin(xyz=(-0.46, 0.0, 0.565)),
        material=brushed_steel,
        name="left_support_post",
    )
    frame.visual(
        Box((0.10, 0.20, 1.03)),
        origin=Origin(xyz=(0.46, 0.0, 0.565)),
        material=brushed_steel,
        name="right_support_post",
    )
    frame.visual(
        Box((0.18, 0.26, 0.16)),
        origin=Origin(xyz=(-0.46, 0.0, 1.15)),
        material=brushed_steel,
        name="left_support_head",
    )
    frame.visual(
        Box((0.18, 0.26, 0.16)),
        origin=Origin(xyz=(0.46, 0.0, 1.15)),
        material=brushed_steel,
        name="right_support_head",
    )
    frame.visual(
        Box((0.94, 0.16, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.26)),
        material=brushed_steel,
        name="top_bridge",
    )
    frame.visual(
        Cylinder(radius=0.07, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        material=dark_graphite,
        name="center_column",
    )
    frame.visual(
        Cylinder(radius=0.10, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        material=black_powder,
        name="bearing_collar",
    )
    frame.visual(
        Cylinder(radius=0.13, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=black_powder,
        name="column_skirt",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.12, 0.56, 1.34)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.67)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.085, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=black_powder,
        name="rotor_collar",
    )
    rotor.visual(
        Cylinder(radius=0.065, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=dark_graphite,
        name="hub_cap",
    )

    arm_length = 0.31
    arm_radius = 0.022
    arm_center_x = arm_length * 0.5
    arm_z = 0.045
    for arm_name, endcap_name, angle in (
        ("arm_0_tube", "arm_0_endcap", 0.0),
        ("arm_1_tube", "arm_1_endcap", 2.0 * math.pi / 3.0),
        ("arm_2_tube", "arm_2_endcap", 4.0 * math.pi / 3.0),
    ):
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(arm_center_x * math.cos(angle), arm_center_x * math.sin(angle), arm_z),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brushed_steel,
            name=arm_name,
        )
        rotor.visual(
            Cylinder(radius=0.032, length=0.05),
            origin=Origin(
                xyz=((arm_length + 0.012) * math.cos(angle), (arm_length + 0.012) * math.sin(angle), arm_z),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=safety_yellow,
            name=endcap_name,
        )

    rotor.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("hub_spin")

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="rotor_collar",
        negative_elem="bearing_collar",
        max_gap=0.002,
        max_penetration=0.0,
        name="rotor collar seats on bearing collar",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="x",
        positive_elem="right_support_post",
        negative_elem="arm_0_tube",
        min_gap=0.03,
        name="rest pose arm clears right support",
    )

    rest_arm = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")
    with ctx.pose({spin: math.pi}):
        ctx.expect_gap(
            rotor,
            frame,
            axis="x",
            positive_elem="arm_0_tube",
            negative_elem="left_support_post",
            min_gap=0.03,
            name="half turn arm clears left support",
        )
        spun_arm = ctx.part_element_world_aabb(rotor, elem="arm_0_tube")

    ctx.check(
        "hub spin swings an arm across the gate opening",
        rest_arm is not None
        and spun_arm is not None
        and rest_arm[1][0] > 0.29
        and spun_arm[0][0] < -0.29,
        details=f"rest={rest_arm}, half_turn={spun_arm}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
