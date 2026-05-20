from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

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

    frame_coat = model.material("frame_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.11, 0.12, 0.14, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.16, 0.78, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=housing_gray,
        name="base_plinth",
    )
    frame.visual(
        Box((0.18, 0.24, 1.08)),
        origin=Origin(xyz=(-0.52, 0.0, 0.54)),
        material=frame_coat,
        name="left_pedestal",
    )
    frame.visual(
        Box((0.18, 0.24, 1.08)),
        origin=Origin(xyz=(0.52, 0.0, 0.54)),
        material=frame_coat,
        name="right_pedestal",
    )
    frame.visual(
        Box((1.16, 0.20, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=frame_coat,
        name="top_crossbar",
    )
    frame.visual(
        Box((0.10, 0.24, 0.26)),
        origin=Origin(xyz=(-0.52, 0.0, 1.01)),
        material=housing_gray,
        name="left_header_pod",
    )
    frame.visual(
        Box((0.10, 0.24, 0.26)),
        origin=Origin(xyz=(0.52, 0.0, 1.01)),
        material=housing_gray,
        name="right_header_pod",
    )
    frame.visual(
        Cylinder(radius=0.082, length=0.82),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=frame_coat,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.096, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=dark_metal,
        name="bearing_cap",
    )

    rail_levels = (0.54, 0.70)
    for side, sign in (("left", -1.0), ("right", 1.0)):
        for rail_index, rail_z in enumerate(rail_levels, start=1):
            frame.visual(
                Cylinder(radius=0.024, length=0.35),
                origin=Origin(
                    xyz=(sign * 0.255, 0.0, rail_z),
                    rpy=(0.0, pi / 2.0, 0.0),
                ),
                material=stainless,
                name=f"{side}_guide_rail_{rail_index}",
            )

    frame.inertial = Inertial.from_geometry(
        Box((1.16, 0.78, 1.30)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.032, length=0.15),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=stainless,
        name="rotor_spindle",
    )
    rotor.visual(
        Cylinder(radius=0.074, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    rotor.visual(
        Cylinder(radius=0.11, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=stainless,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.078, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.204)),
        material=dark_metal,
        name="hub_top_cap",
    )

    arm_center_radius = 0.22
    arm_length = 0.34
    arm_height = 0.10
    arm_tip_radius = 0.39
    for arm_index, angle in enumerate((0.0, 2.0 * pi / 3.0, 4.0 * pi / 3.0)):
        rotor.visual(
            Cylinder(radius=0.017, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * cos(angle), arm_center_radius * sin(angle), arm_height),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_{arm_index}",
        )
        rotor.visual(
            Cylinder(radius=0.026, length=0.09),
            origin=Origin(
                xyz=(0.105 * cos(angle), 0.105 * sin(angle), arm_height),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=dark_metal,
            name=f"arm_root_{arm_index}",
        )
        rotor.visual(
            Cylinder(radius=0.022, length=0.036),
            origin=Origin(
                xyz=(arm_tip_radius * cos(angle), arm_tip_radius * sin(angle), arm_height),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_tip_{arm_index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((0.84, 0.84, 0.25)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    rotor_spin = object_model.get_articulation("rotor_spin")

    ctx.check(
        "rotor uses continuous spin joint",
        rotor_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={rotor_spin.articulation_type}",
    )
    ctx.check(
        "rotor spins about vertical axis",
        tuple(rotor_spin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={rotor_spin.axis}",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="xy",
        elem_a="hub_barrel",
        elem_b="bearing_cap",
        min_overlap=0.18,
        name="hub remains centered over the support bearing",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="lower_bearing_collar",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.006,
        name="rotating stage stays compact above the support",
    )

    with ctx.pose({rotor_spin: 0.0}):
        ctx.expect_gap(
            frame,
            rotor,
            axis="x",
            positive_elem="right_pedestal",
            min_gap=0.01,
            max_gap=0.05,
            name="rest pose arm clears the right pedestal",
        )

    with ctx.pose({rotor_spin: pi}):
        ctx.expect_gap(
            rotor,
            frame,
            axis="x",
            negative_elem="left_pedestal",
            min_gap=0.01,
            max_gap=0.05,
            name="half-turn pose arm clears the left pedestal",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
