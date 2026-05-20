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

    frame_metal = model.material("frame_metal", rgba=(0.29, 0.31, 0.34, 1.0))
    rotor_metal = model.material("rotor_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    cap_dark = model.material("cap_dark", rgba=(0.10, 0.11, 0.12, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.49, 0.04)),
        material=frame_metal,
        name="base_front_rail",
    )
    frame.visual(
        Box((1.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.49, 0.04)),
        material=frame_metal,
        name="base_rear_rail",
    )
    frame.visual(
        Box((0.08, 0.90, 0.08)),
        origin=Origin(xyz=(-0.49, 0.0, 0.04)),
        material=frame_metal,
        name="base_left_rail",
    )
    frame.visual(
        Box((0.08, 0.90, 0.08)),
        origin=Origin(xyz=(0.49, 0.0, 0.04)),
        material=frame_metal,
        name="base_right_rail",
    )

    post_height = 1.14
    post_center_z = 0.08 + post_height / 2.0
    frame.visual(
        Box((0.08, 0.08, post_height)),
        origin=Origin(xyz=(-0.49, -0.49, post_center_z)),
        material=frame_metal,
        name="post_left_rear",
    )
    frame.visual(
        Box((0.08, 0.08, post_height)),
        origin=Origin(xyz=(-0.49, 0.49, post_center_z)),
        material=frame_metal,
        name="post_left_front",
    )
    frame.visual(
        Box((0.08, 0.08, post_height)),
        origin=Origin(xyz=(0.49, -0.49, post_center_z)),
        material=frame_metal,
        name="post_right_rear",
    )
    frame.visual(
        Box((0.08, 0.08, post_height)),
        origin=Origin(xyz=(0.49, 0.49, post_center_z)),
        material=frame_metal,
        name="post_right_front",
    )

    frame.visual(
        Box((1.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, 0.49, 1.26)),
        material=frame_metal,
        name="top_front_rail",
    )
    frame.visual(
        Box((1.06, 0.08, 0.08)),
        origin=Origin(xyz=(0.0, -0.49, 1.26)),
        material=frame_metal,
        name="top_rear_rail",
    )
    frame.visual(
        Box((0.08, 0.90, 0.08)),
        origin=Origin(xyz=(-0.49, 0.0, 1.26)),
        material=frame_metal,
        name="top_left_rail",
    )
    frame.visual(
        Box((0.08, 0.90, 0.08)),
        origin=Origin(xyz=(0.49, 0.0, 1.26)),
        material=frame_metal,
        name="top_right_rail",
    )

    frame.visual(
        Cylinder(radius=0.09, length=0.74),
        origin=Origin(xyz=(0.0, 0.0, 0.47)),
        material=frame_metal,
        name="central_support",
    )
    frame.visual(
        Cylinder(radius=0.12, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.86)),
        material=cap_dark,
        name="bearing_cap",
    )
    frame.visual(
        Box((0.34, 0.34, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=cap_dark,
        name="support_plinth",
    )
    frame.visual(
        Box((0.90, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=frame_metal,
        name="base_crossmember_x",
    )
    frame.visual(
        Box((0.08, 0.90, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=frame_metal,
        name="base_crossmember_y",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.06, 1.06, 1.30)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.08, length=0.10),
        material=rotor_metal,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=cap_dark,
        name="hub_top_cap",
    )
    rotor.visual(
        Cylinder(radius=0.050, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, -0.09)),
        material=cap_dark,
        name="hub_lower_boss",
    )

    arm_length = 0.36
    arm_radius = 0.02
    arm_center_radius = 0.235
    collar_radius = 0.135
    rotor.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(xyz=(arm_center_radius, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rotor_metal,
        name="arm_1",
    )
    rotor.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(collar_radius, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cap_dark,
        name="arm_collar_1",
    )
    rotor.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(
                arm_center_radius * math.cos(2.0 * math.pi / 3.0),
                arm_center_radius * math.sin(2.0 * math.pi / 3.0),
                0.0,
            ),
            rpy=(0.0, math.pi / 2.0, 2.0 * math.pi / 3.0),
        ),
        material=rotor_metal,
        name="arm_2",
    )
    rotor.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(
            xyz=(
                collar_radius * math.cos(2.0 * math.pi / 3.0),
                collar_radius * math.sin(2.0 * math.pi / 3.0),
                0.0,
            ),
            rpy=(0.0, math.pi / 2.0, 2.0 * math.pi / 3.0),
        ),
        material=cap_dark,
        name="arm_collar_2",
    )
    rotor.visual(
        Cylinder(radius=arm_radius, length=arm_length),
        origin=Origin(
            xyz=(
                arm_center_radius * math.cos(4.0 * math.pi / 3.0),
                arm_center_radius * math.sin(4.0 * math.pi / 3.0),
                0.0,
            ),
            rpy=(0.0, math.pi / 2.0, 4.0 * math.pi / 3.0),
        ),
        material=rotor_metal,
        name="arm_3",
    )
    rotor.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(
            xyz=(
                collar_radius * math.cos(4.0 * math.pi / 3.0),
                collar_radius * math.sin(4.0 * math.pi / 3.0),
                0.0,
            ),
            rpy=(0.0, math.pi / 2.0, 4.0 * math.pi / 3.0),
        ),
        material=cap_dark,
        name="arm_collar_3",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.43, length=0.18),
        mass=18.0,
        origin=Origin(),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.01)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_lower_boss",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.04,
        name="rotor boss clears the fixed bearing cap",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="xy",
        elem_a="hub_drum",
        elem_b="support_plinth",
        min_overlap=0.12,
        name="hub stays centered over the central support",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="top_front_rail",
        negative_elem="hub_top_cap",
        min_gap=0.08,
        name="top frame remains above the rotor stage",
    )

    with ctx.pose({joint: math.pi / 3.0}):
        ctx.expect_gap(
            frame,
            rotor,
            axis="x",
            positive_elem="post_right_front",
            negative_elem="arm_1",
            min_gap=0.03,
            name="rotated arm clears the right-front post",
        )
    with ctx.pose({joint: -math.pi / 3.0}):
        ctx.expect_gap(
            rotor,
            frame,
            axis="x",
            positive_elem="arm_1",
            negative_elem="post_left_front",
            min_gap=0.03,
            name="rotated arm clears the left-front post",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
