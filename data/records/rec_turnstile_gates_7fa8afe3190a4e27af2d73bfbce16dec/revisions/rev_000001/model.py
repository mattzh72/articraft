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

    frame_paint = model.material("frame_paint", rgba=(0.34, 0.37, 0.40, 1.0))
    rotor_steel = model.material("rotor_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.15, 0.16, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.30, 1.30, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_trim,
        name="base_plinth",
    )

    post_size = 0.08
    post_height = 1.12
    post_center_z = 0.06 + post_height / 2.0
    post_offset = 0.57
    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            frame.visual(
                Box((post_size, post_size, post_height)),
                origin=Origin(xyz=(x_sign * post_offset, y_sign * post_offset, post_center_z)),
                material=frame_paint,
                name=f"post_{'r' if x_sign > 0 else 'l'}_{'f' if y_sign > 0 else 'b'}",
            )

    beam_span = 2.0 * post_offset - post_size
    beam_z = 1.18
    frame.visual(
        Box((beam_span, post_size, post_size)),
        origin=Origin(xyz=(0.0, post_offset, beam_z)),
        material=frame_paint,
        name="top_beam_front",
    )
    frame.visual(
        Box((beam_span, post_size, post_size)),
        origin=Origin(xyz=(0.0, -post_offset, beam_z)),
        material=frame_paint,
        name="top_beam_back",
    )
    frame.visual(
        Box((post_size, beam_span, post_size)),
        origin=Origin(xyz=(post_offset, 0.0, beam_z)),
        material=frame_paint,
        name="top_beam_right",
    )
    frame.visual(
        Box((post_size, beam_span, post_size)),
        origin=Origin(xyz=(-post_offset, 0.0, beam_z)),
        material=frame_paint,
        name="top_beam_left",
    )

    frame.visual(
        Cylinder(radius=0.06, length=0.89),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=frame_paint,
        name="center_column",
    )
    frame.visual(
        Cylinder(radius=0.10, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.89)),
        material=dark_trim,
        name="bearing_housing",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.30, 1.30, 1.22)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.61)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.12, length=0.10),
        origin=Origin(),
        material=rotor_steel,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.05, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_trim,
        name="top_cap",
    )

    arm_radius = 0.022
    arm_length = 0.40
    arm_mid_radius = 0.305
    for index in range(3):
        angle = index * 2.0 * math.pi / 3.0
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(
                    arm_mid_radius * math.cos(angle),
                    arm_mid_radius * math.sin(angle),
                    0.0,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=rotor_steel,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.08),
            origin=Origin(
                xyz=(
                    0.49 * math.cos(angle),
                    0.49 * math.sin(angle),
                    0.0,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=dark_trim,
            name=f"arm_tip_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.50, length=0.14),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=3.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_drum",
        negative_elem="base_plinth",
        min_gap=0.88,
        max_gap=0.96,
        name="rotating hub sits high above the base",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_drum",
        negative_elem="bearing_housing",
        min_gap=0.0,
        max_gap=0.001,
        name="hub seats directly above the bearing housing",
    )

    rest_arm = ctx.part_element_world_aabb(rotor, elem="arm_0")
    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        rotated_arm = ctx.part_element_world_aabb(rotor, elem="arm_0")

    def _center_x_y(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return (
            0.5 * (lower[0] + upper[0]),
            0.5 * (lower[1] + upper[1]),
        )

    rest_center = _center_x_y(rest_arm)
    rotated_center = _center_x_y(rotated_arm)
    ctx.check(
        "rotor arm advances around the vertical axis",
        rest_center is not None
        and rotated_center is not None
        and rest_center[0] > 0.20
        and abs(rest_center[1]) < 0.03
        and rotated_center[0] < -0.10
        and rotated_center[1] > 0.18,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
