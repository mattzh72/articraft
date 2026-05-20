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

    frame_paint = model.material("frame_paint", rgba=(0.24, 0.27, 0.30, 1.0))
    column_finish = model.material("column_finish", rgba=(0.69, 0.71, 0.73, 1.0))
    stainless = model.material("stainless", rgba=(0.82, 0.83, 0.85, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.11, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.06, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.44, 0.02)),
        material=frame_paint,
        name="base_front_rail",
    )
    frame.visual(
        Box((1.06, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, -0.44, 0.02)),
        material=frame_paint,
        name="base_rear_rail",
    )
    frame.visual(
        Box((0.06, 0.82, 0.04)),
        origin=Origin(xyz=(0.50, 0.0, 0.02)),
        material=frame_paint,
        name="base_right_rail",
    )
    frame.visual(
        Box((0.06, 0.82, 0.04)),
        origin=Origin(xyz=(-0.50, 0.0, 0.02)),
        material=frame_paint,
        name="base_left_rail",
    )
    frame.visual(
        Box((0.94, 0.06, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=frame_paint,
        name="base_cross_x",
    )
    frame.visual(
        Box((0.06, 0.82, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=frame_paint,
        name="base_cross_y",
    )
    frame.visual(
        Cylinder(radius=0.14, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=column_finish,
        name="mounting_disc",
    )

    post_positions = [
        (-0.50, -0.41),
        (-0.50, 0.41),
        (0.50, -0.41),
        (0.50, 0.41),
    ]
    for index, (px, py) in enumerate(post_positions):
        frame.visual(
            Cylinder(radius=0.028, length=1.04),
            origin=Origin(xyz=(px, py, 0.56)),
            material=column_finish,
            name=f"post_{index}",
        )

    frame.visual(
        Box((1.06, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.44, 1.06)),
        material=frame_paint,
        name="top_front_rail",
    )
    frame.visual(
        Box((1.06, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.44, 1.06)),
        material=frame_paint,
        name="top_rear_rail",
    )
    frame.visual(
        Box((0.05, 0.82, 0.04)),
        origin=Origin(xyz=(0.50, 0.0, 1.06)),
        material=frame_paint,
        name="top_right_rail",
    )
    frame.visual(
        Box((0.05, 0.82, 0.04)),
        origin=Origin(xyz=(-0.50, 0.0, 1.06)),
        material=frame_paint,
        name="top_left_rail",
    )
    frame.visual(
        Box((0.04, 0.82, 0.04)),
        origin=Origin(xyz=(0.50, 0.0, 0.56)),
        material=frame_paint,
        name="mid_right_guide",
    )
    frame.visual(
        Box((0.04, 0.82, 0.04)),
        origin=Origin(xyz=(-0.50, 0.0, 0.56)),
        material=frame_paint,
        name="mid_left_guide",
    )
    frame.visual(
        Cylinder(radius=0.055, length=0.67),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=column_finish,
        name="center_column",
    )
    frame.visual(
        Cylinder(radius=0.078, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.80)),
        material=bearing_dark,
        name="bearing_collar",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.06, 0.88, 1.08)),
        mass=72.0,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.10, length=0.10),
        material=stainless,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.076, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, -0.0625)),
        material=bearing_dark,
        name="lower_bearing_race",
    )
    rotor.visual(
        Cylinder(radius=0.062, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=bearing_dark,
        name="hub_cap",
    )

    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        arm_dx = math.cos(angle)
        arm_dy = math.sin(angle)
        rotor.visual(
            Cylinder(radius=0.018, length=0.34),
            origin=Origin(
                xyz=(0.17 * arm_dx, 0.17 * arm_dy, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=stainless,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.023, length=0.055),
            origin=Origin(
                xyz=(0.3675 * arm_dx, 0.3675 * arm_dy, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=grip_black,
            name=f"arm_tip_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((0.82, 0.82, 0.14)),
        mass=16.0,
    )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.915)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=6.0),
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
        positive_elem="lower_bearing_race",
        negative_elem="bearing_collar",
        max_gap=0.0015,
        max_penetration=0.0,
        name="rotor sits on exposed bearing collar",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.0,
        name="rotor stays inside frame footprint at rest",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="top_front_rail",
        negative_elem="hub_cap",
        min_gap=0.04,
        name="top frame clears the rotary stage",
    )

    with ctx.pose({spin: math.tau / 6.0}):
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.0,
            name="rotor stays inside frame footprint when turned",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
