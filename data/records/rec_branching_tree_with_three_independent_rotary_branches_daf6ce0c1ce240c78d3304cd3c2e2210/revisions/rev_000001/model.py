from __future__ import annotations

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


PIVOT_Z = 1.014
BRANCH_XS = (-0.42, 0.0, 0.42)
BRANCH_YAWS = (math.radians(-18.0), 0.0, math.radians(18.0))


def _add_static_bridge(bridge, *, steel, dark_steel, fastener) -> None:
    """Build the fixed overhead support as a heavy bridge, not as a moving arm."""

    bridge.visual(
        Box((1.28, 0.16, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 1.200)),
        material=steel,
        name="main_bridge",
    )
    bridge.visual(
        Box((1.18, 0.205, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 1.255)),
        material=dark_steel,
        name="top_flange",
    )
    bridge.visual(
        Box((1.18, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, 0.098, 1.198)),
        material=dark_steel,
        name="side_lip_0",
    )
    bridge.visual(
        Box((1.18, 0.018, 0.090)),
        origin=Origin(xyz=(0.0, -0.098, 1.198)),
        material=dark_steel,
        name="side_lip_1",
    )

    for index, x in enumerate(BRANCH_XS):
        bridge.visual(
            Box((0.190, 0.185, 0.028)),
            # The pad overlaps the main beam by 4 mm and leaves a flat underside
            # for its matching hanger bracket.
            origin=Origin(xyz=(x, 0.0, 1.150)),
            material=dark_steel,
            name=f"bracket_pad_{index}",
        )
        for sx in (-0.060, 0.060):
            for sy in (-0.052, 0.052):
                bridge.visual(
                    Cylinder(radius=0.0075, length=0.010),
                    origin=Origin(xyz=(x + sx, sy, 1.169)),
                    material=fastener,
                    name=f"pad_bolt_{index}_{sx}_{sy}",
                )


def _add_hanger_bracket(bracket, *, paint, dark_steel, fastener) -> None:
    """A compact drop bracket with a flat top plate and thrust-bearing cap."""

    bracket.visual(
        Box((0.180, 0.160, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=paint,
        name="mount_plate",
    )
    bracket.visual(
        Box((0.062, 0.054, 0.082)),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=paint,
        name="drop_web",
    )
    bracket.visual(
        Box((0.110, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, 0.057, 0.057)),
        material=paint,
        name="side_cheek_0",
    )
    bracket.visual(
        Box((0.110, 0.014, 0.088)),
        origin=Origin(xyz=(0.0, -0.057, 0.057)),
        material=paint,
        name="side_cheek_1",
    )
    bracket.visual(
        Cylinder(radius=0.038, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=dark_steel,
        name="bearing_cap",
    )
    bracket.visual(
        Box((0.120, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_steel,
        name="cross_strap",
    )
    for sx in (-0.058, 0.058):
        for sy in (-0.048, 0.048):
            bracket.visual(
                Cylinder(radius=0.006, length=0.006),
                origin=Origin(xyz=(sx, sy, 0.096)),
                material=fastener,
                name=f"bracket_bolt_{sx}_{sy}",
            )


def _add_branch_arm(arm, *, paint, dark_steel, fastener) -> None:
    """One hanging rotary branch: a vertical spindle carrying a T-shaped arm."""

    arm.visual(
        Cylinder(radius=0.016, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=dark_steel,
        name="pivot_shaft",
    )
    arm.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.093)),
        material=dark_steel,
        name="hub_disc",
    )
    arm.visual(
        Box((0.056, 0.340, 0.034)),
        origin=Origin(xyz=(0.0, -0.185, -0.115)),
        material=paint,
        name="branch_spar",
    )
    arm.visual(
        Box((0.190, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, -0.350, -0.115)),
        material=paint,
        name="end_crossbar",
    )
    arm.visual(
        Box((0.136, 0.032, 0.026)),
        origin=Origin(xyz=(0.0, -0.252, -0.086)),
        material=paint,
        name="raised_rib",
    )
    for sx in (-0.082, 0.082):
        arm.visual(
            Cylinder(radius=0.023, length=0.030),
            origin=Origin(xyz=(sx, -0.350, -0.115)),
            material=fastener,
            name=f"end_boss_{sx}",
        )

    arm.visual(
        Cylinder(radius=0.009, length=0.150),
        origin=Origin(
            xyz=(-0.034, -0.174, -0.099),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="side_tie_0",
    )
    arm.visual(
        Cylinder(radius=0.009, length=0.150),
        origin=Origin(
            xyz=(0.034, -0.174, -0.099),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="side_tie_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_three_branch_rotary_frame")

    bridge_steel = model.material("bridge_steel", rgba=(0.46, 0.49, 0.51, 1.0))
    dark_steel = model.material("dark_bearing_steel", rgba=(0.15, 0.16, 0.17, 1.0))
    bracket_yellow = model.material("fixed_bracket_yellow", rgba=(0.92, 0.68, 0.16, 1.0))
    arm_orange = model.material("moving_arm_orange", rgba=(0.90, 0.36, 0.12, 1.0))
    fastener = model.material("black_fasteners", rgba=(0.05, 0.05, 0.055, 1.0))

    bridge = model.part("top_bridge")
    _add_static_bridge(bridge, steel=bridge_steel, dark_steel=dark_steel, fastener=fastener)
    bridge.inertial = Inertial.from_geometry(
        Box((1.36, 0.24, 0.16)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 1.205)),
    )

    for index, (x, yaw) in enumerate(zip(BRANCH_XS, BRANCH_YAWS)):
        bracket = model.part(f"bracket_{index}")
        _add_hanger_bracket(
            bracket,
            paint=bracket_yellow,
            dark_steel=dark_steel,
            fastener=fastener,
        )
        bracket.inertial = Inertial.from_geometry(
            Box((0.19, 0.17, 0.13)),
            mass=0.75,
            origin=Origin(xyz=(0.0, 0.0, 0.064)),
        )

        arm = model.part(f"arm_{index}")
        _add_branch_arm(
            arm,
            paint=arm_orange,
            dark_steel=dark_steel,
            fastener=fastener,
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.24, 0.42, 0.14)),
            mass=0.95,
            origin=Origin(xyz=(0.0, -0.20, -0.085)),
        )

        model.articulation(
            f"bridge_to_bracket_{index}",
            ArticulationType.FIXED,
            parent=bridge,
            child=bracket,
            origin=Origin(xyz=(x, 0.0, PIVOT_Z)),
        )
        model.articulation(
            f"bracket_to_arm_{index}",
            ArticulationType.REVOLUTE,
            parent=bracket,
            child=arm,
            origin=Origin(rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.2,
                lower=math.radians(-95.0),
                upper=math.radians(95.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("top_bridge")

    moving_joints = [object_model.get_articulation(f"bracket_to_arm_{i}") for i in range(3)]
    fixed_joints = [object_model.get_articulation(f"bridge_to_bracket_{i}") for i in range(3)]
    ctx.check(
        "three separate fixed brackets and three independent rotary arms",
        len(fixed_joints) == 3 and len(moving_joints) == 3,
        details=f"fixed={fixed_joints}, moving={moving_joints}",
    )

    for index in range(3):
        bracket = object_model.get_part(f"bracket_{index}")
        arm = object_model.get_part(f"arm_{index}")
        joint = object_model.get_articulation(f"bracket_to_arm_{index}")

        ctx.expect_gap(
            bridge,
            bracket,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0005,
            name=f"bracket_{index} is seated under the bridge pad",
        )
        ctx.expect_gap(
            bridge,
            arm,
            axis="z",
            min_gap=0.075,
            name=f"arm_{index} hangs below the fixed bridge",
        )
        ctx.expect_gap(
            bracket,
            arm,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="bearing_cap",
            negative_elem="pivot_shaft",
            name=f"arm_{index} spindle seats against its own bearing",
        )
        ctx.expect_overlap(
            bracket,
            arm,
            axes="xy",
            min_overlap=0.020,
            elem_a="bearing_cap",
            elem_b="pivot_shaft",
            name=f"arm_{index} spindle is centered under its bracket",
        )

        with ctx.pose({joint: math.radians(70.0)}):
            ctx.expect_gap(
                bridge,
                arm,
                axis="z",
                min_gap=0.075,
                name=f"arm_{index} remains under-slung when rotated",
            )
            ctx.expect_overlap(
                bracket,
                arm,
                axes="xy",
                min_overlap=0.020,
                elem_a="bearing_cap",
                elem_b="pivot_shaft",
                name=f"arm_{index} rotates around its own support",
            )

    return ctx.report()


object_model = build_object_model()
