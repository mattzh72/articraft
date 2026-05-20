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

    frame_paint = model.material("frame_paint", rgba=(0.19, 0.22, 0.24, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.50, 0.52, 0.54, 1.0))
    steel = model.material("steel", rgba=(0.77, 0.79, 0.82, 1.0))
    brushed_dark = model.material("brushed_dark", rgba=(0.36, 0.39, 0.41, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((2.50, 2.50, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=deck_gray,
        name="base_deck",
    )
    frame.visual(
        Cylinder(radius=0.11, length=0.95),
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
        material=frame_paint,
        name="center_column",
    )
    frame.visual(
        Cylinder(radius=0.17, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=brushed_dark,
        name="bearing_cap",
    )

    post_radius = 0.045
    post_length = 1.55
    post_center_z = 0.08 + post_length / 2.0
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            frame.visual(
                Cylinder(radius=post_radius, length=post_length),
                origin=Origin(xyz=(1.08 * x_sign, 1.08 * y_sign, post_center_z)),
                material=frame_paint,
                name=f"corner_post_{'p' if x_sign > 0.0 else 'n'}x_{'p' if y_sign > 0.0 else 'n'}y",
            )

    frame.visual(
        Box((2.16, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, 1.08, 1.63)),
        material=frame_paint,
        name="top_front_rail",
    )
    frame.visual(
        Box((2.16, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, -1.08, 1.63)),
        material=frame_paint,
        name="top_rear_rail",
    )
    frame.visual(
        Box((0.09, 2.16, 0.08)),
        origin=Origin(xyz=(1.08, 0.0, 1.63)),
        material=frame_paint,
        name="top_right_rail",
    )
    frame.visual(
        Box((0.09, 2.16, 0.08)),
        origin=Origin(xyz=(-1.08, 0.0, 1.63)),
        material=frame_paint,
        name="top_left_rail",
    )
    frame.visual(
        Box((0.07, 2.08, 0.06)),
        origin=Origin(xyz=(1.08, 0.0, 0.72)),
        material=frame_paint,
        name="right_side_guard",
    )
    frame.visual(
        Box((0.07, 2.08, 0.06)),
        origin=Origin(xyz=(-1.08, 0.0, 0.72)),
        material=frame_paint,
        name="left_side_guard",
    )
    frame.visual(
        Box((0.04, 2.00, 0.18)),
        origin=Origin(xyz=(1.08, 0.0, 0.17)),
        material=frame_paint,
        name="right_kick_panel",
    )
    frame.visual(
        Box((0.04, 2.00, 0.18)),
        origin=Origin(xyz=(-1.08, 0.0, 0.17)),
        material=frame_paint,
        name="left_kick_panel",
    )
    frame.inertial = Inertial.from_geometry(
        Box((2.50, 2.50, 1.67)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 0.0, 0.835)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.24, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=steel,
        name="hub_shell",
    )
    rotor.visual(
        Cylinder(radius=0.12, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=brushed_dark,
        name="hub_crown",
    )

    arm_radius = 0.03
    arm_length = 0.72
    arm_mid_radius = 0.20 + arm_length / 2.0
    end_post_radius = 0.028
    end_post_length = 0.94
    end_post_center_z = -end_post_length / 2.0
    end_post_radius_from_axis = 0.20 + arm_length

    for index in range(3):
        angle = index * 2.0 * math.pi / 3.0
        c = math.cos(angle)
        s = math.sin(angle)
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(arm_mid_radius * c, arm_mid_radius * s, 0.0),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=steel,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=end_post_radius, length=end_post_length),
            origin=Origin(
                xyz=(end_post_radius_from_axis * c, end_post_radius_from_axis * s, end_post_center_z),
            ),
            material=steel,
            name=f"end_post_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.045, length=0.10),
            origin=Origin(
                xyz=(end_post_radius_from_axis * c, end_post_radius_from_axis * s, 0.02),
            ),
            material=rubber,
            name=f"arm_bumper_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.95, length=1.12),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, -0.34)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.07)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    ctx.expect_origin_distance(
        frame,
        rotor,
        axes="xy",
        min_dist=0.0,
        max_dist=0.001,
        name="rotor stays centered on the turnstile axis",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_shell",
        negative_elem="bearing_cap",
        max_gap=0.002,
        max_penetration=0.0,
        name="hub shell sits directly on the fixed bearing cap",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        margin=0.0,
        name="rotating cage remains inside the fixed frame at rest",
    )

    rest_center = aabb_center(ctx.part_element_world_aabb(rotor, elem="end_post_0"))
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        turned_center = aabb_center(ctx.part_element_world_aabb(rotor, elem="end_post_0"))
        ctx.expect_within(
            rotor,
            frame,
            axes="xy",
            margin=0.0,
            name="rotating cage remains inside the fixed frame after a third-turn",
        )

    ctx.check(
        "continuous joint turns the radial arm around the vertical axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.85
        and abs(rest_center[1]) < 0.05
        and turned_center[0] < -0.35
        and turned_center[1] > 0.70,
        details=f"rest_center={rest_center}, turned_center={turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
