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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    frame_paint = model.material("frame_paint", rgba=(0.25, 0.27, 0.29, 1.0))
    frame_dark = model.material("frame_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    stainless_dark = model.material("stainless_dark", rgba=(0.56, 0.58, 0.60, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.30, 0.72, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=frame_dark,
        name="base_plinth",
    )
    frame.visual(
        Cylinder(radius=0.17, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=frame_dark,
        name="base_collar",
    )
    frame.visual(
        Cylinder(radius=0.10, length=0.64),
        origin=Origin(xyz=(0.0, 0.0, 0.48)),
        material=frame_paint,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.20, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.85)),
        material=frame_dark,
        name="lower_bearing_housing",
    )

    for side, sign in (("left", -1.0), ("right", 1.0)):
        frame.visual(
            Box((0.22, 0.50, 0.84)),
            origin=Origin(xyz=(0.63 * sign, 0.0, 0.48)),
            material=frame_paint,
            name=f"{side}_support",
        )
        frame.visual(
            Box((0.54, 0.18, 0.16)),
            origin=Origin(xyz=(0.36 * sign, 0.0, 0.74)),
            material=frame_dark,
            name=f"{side}_stage_block",
        )
        frame.visual(
            Box((0.30, 0.58, 0.12)),
            origin=Origin(xyz=(0.63 * sign, 0.0, 0.96)),
            material=frame_dark,
            name=f"{side}_stage_cap",
        )
        for end, y_pos in (("front", 0.39), ("rear", -0.39)):
            frame.visual(
                Cylinder(radius=0.038, length=0.28),
                origin=Origin(xyz=(0.63 * sign, y_pos, 0.77), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=frame_paint,
                name=f"{side}_{end}_lower_rail",
            )
            frame.visual(
                Cylinder(radius=0.032, length=0.28),
                origin=Origin(xyz=(0.63 * sign, y_pos, 0.95), rpy=(-math.pi / 2.0, 0.0, 0.0)),
                material=stainless_dark,
                name=f"{side}_{end}_upper_rail",
            )

    frame.visual(
        Box((1.06, 0.18, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.07)),
        material=frame_dark,
        name="overhead_bridge",
    )
    frame.visual(
        Cylinder(radius=0.18, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        material=frame_dark,
        name="upper_bearing_cap",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.30, 0.72, 1.12)),
        mass=120.0,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.05, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=stainless_dark,
        name="hub_shaft",
    )
    rotor.visual(
        Cylinder(radius=0.13, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=stainless,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.09, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, -0.03)),
        material=stainless_dark,
        name="lower_rotor_collar",
    )
    rotor.visual(
        Cylinder(radius=0.09, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=stainless_dark,
        name="upper_rotor_collar",
    )

    arm_length = 0.34
    arm_center_radius = 0.25
    sleeve_length = 0.12
    sleeve_center_radius = 0.37
    tip_center_radius = 0.43
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        c = math.cos(angle)
        s = math.sin(angle)
        arm_rpy = (0.0, math.pi / 2.0, angle)
        rotor.visual(
            Cylinder(radius=0.022, length=arm_length),
            origin=Origin(
                xyz=(arm_center_radius * c, arm_center_radius * s, 0.0),
                rpy=arm_rpy,
            ),
            material=stainless,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=sleeve_length),
            origin=Origin(
                xyz=(sleeve_center_radius * c, sleeve_center_radius * s, 0.0),
                rpy=arm_rpy,
            ),
            material=stainless_dark,
            name=f"arm_{index}_grip",
        )
        rotor.visual(
            Sphere(radius=0.03),
            origin=Origin(xyz=(tip_center_radius * c, tip_center_radius * s, 0.0)),
            material=stainless_dark,
            name=f"arm_{index}_tip",
        )

    rotor.inertial = Inertial.from_geometry(
        Box((0.92, 0.92, 0.16)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )

    model.articulation(
        "hub_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    hub_spin = object_model.get_articulation("hub_spin")

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_drum",
        negative_elem="lower_bearing_housing",
        min_gap=0.008,
        max_gap=0.025,
        name="hub clears the lower bearing housing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="upper_bearing_cap",
        negative_elem="hub_drum",
        min_gap=0.008,
        max_gap=0.025,
        name="hub clears the upper bearing cap",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="x",
        positive_elem="right_stage_cap",
        negative_elem="arm_0_tip",
        min_gap=0.015,
        max_gap=0.050,
        name="right side frame clears the forward arm at rest",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    rest_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="arm_0_tip"))
    with ctx.pose({hub_spin: math.pi / 2.0}):
        turned_tip = _aabb_center(ctx.part_element_world_aabb(rotor, elem="arm_0_tip"))

    ctx.check(
        "arm tip swings around the vertical axis",
        rest_tip is not None
        and turned_tip is not None
        and rest_tip[0] > 0.40
        and abs(rest_tip[1]) < 0.02
        and turned_tip[1] > 0.40
        and abs(turned_tip[0]) < 0.02
        and abs(turned_tip[2] - rest_tip[2]) < 0.002,
        details=f"rest_tip={rest_tip}, turned_tip={turned_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
