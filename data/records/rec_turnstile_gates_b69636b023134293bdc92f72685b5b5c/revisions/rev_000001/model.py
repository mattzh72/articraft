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

    stainless = model.material("stainless", rgba=(0.76, 0.78, 0.80, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.20, 0.24, 0.29, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.12, 0.13, 0.15, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.72, 0.18, 1.0))

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((1.24, 0.72, 1.10)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    frame.visual(
        Box((1.24, 0.72, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=painted_steel,
        name="base_deck",
    )
    frame.visual(
        Box((0.42, 0.26, 0.56)),
        origin=Origin(xyz=(0.00, 0.0, 0.31)),
        material=painted_steel,
        name="drive_pedestal",
    )
    frame.visual(
        Cylinder(radius=0.078, length=0.62),
        origin=Origin(xyz=(0.14, 0.0, 0.59)),
        material=stainless,
        name="support_column",
    )
    frame.visual(
        Box((0.20, 0.18, 0.12)),
        origin=Origin(xyz=(0.08, 0.0, 0.80)),
        material=painted_steel,
        name="upper_gusset",
    )
    frame.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(0.14, 0.0, 0.875)),
        material=dark_polymer,
        name="bearing_cap",
    )
    frame.visual(
        Box((0.16, 0.22, 0.08)),
        origin=Origin(xyz=(-0.12, 0.0, 0.57)),
        material=dark_polymer,
        name="service_access_box",
    )

    post_radius = 0.03
    post_length = 1.08
    post_z = 0.56
    for name, x_pos, y_pos in (
        ("left_front_post", -0.38, 0.29),
        ("left_rear_post", -0.38, -0.29),
        ("right_front_post", 0.66, 0.29),
        ("right_rear_post", 0.66, -0.29),
    ):
        frame.visual(
            Cylinder(radius=post_radius, length=post_length),
            origin=Origin(xyz=(x_pos, y_pos, post_z)),
            material=stainless,
            name=name,
        )

    frame.visual(
        Box((1.04, 0.05, 0.05)),
        origin=Origin(xyz=(0.14, 0.29, 1.085)),
        material=stainless,
        name="front_header",
    )
    frame.visual(
        Box((1.04, 0.05, 0.05)),
        origin=Origin(xyz=(0.14, -0.29, 1.085)),
        material=stainless,
        name="rear_header",
    )
    frame.visual(
        Box((0.05, 0.58, 0.05)),
        origin=Origin(xyz=(-0.38, 0.0, 1.085)),
        material=stainless,
        name="left_roof_rail",
    )
    frame.visual(
        Box((0.05, 0.58, 0.05)),
        origin=Origin(xyz=(0.66, 0.0, 1.085)),
        material=stainless,
        name="right_roof_rail",
    )
    frame.visual(
        Box((0.05, 0.58, 0.05)),
        origin=Origin(xyz=(-0.38, 0.0, 0.44)),
        material=stainless,
        name="left_lower_rail",
    )
    frame.visual(
        Box((0.05, 0.58, 0.05)),
        origin=Origin(xyz=(0.66, 0.0, 0.44)),
        material=stainless,
        name="right_lower_rail",
    )
    frame.visual(
        Box((0.08, 0.08, 0.12)),
        origin=Origin(xyz=(0.66, 0.29, 1.02)),
        material=safety_yellow,
        name="entry_beacon",
    )

    rotor = model.part("rotor")
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.18),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )
    rotor.visual(
        Cylinder(radius=0.06, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_polymer,
        name="lower_collar",
    )
    rotor.visual(
        Cylinder(radius=0.085, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=dark_polymer,
        name="hub_drum",
    )
    rotor.visual(
        Cylinder(radius=0.095, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=painted_steel,
        name="hub_top_cap",
    )

    arm_radius = 0.018
    arm_length = 0.38
    arm_center_radius = 0.145
    tip_center_radius = 0.350
    for arm_name, tip_name, angle in (
        ("arm_0", "arm_tip_0", 0.0),
        ("arm_1", "arm_tip_1", 2.0 * pi / 3.0),
        ("arm_2", "arm_tip_2", 4.0 * pi / 3.0),
    ):
        x_dir = cos(angle)
        y_dir = sin(angle)
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(x_dir * arm_center_radius, y_dir * arm_center_radius, 0.085),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=stainless,
            name=arm_name,
        )
        rotor.visual(
            Cylinder(radius=0.024, length=0.05),
            origin=Origin(
                xyz=(x_dir * tip_center_radius, y_dir * tip_center_radius, 0.085),
                rpy=(0.0, pi / 2.0, angle),
            ),
            material=dark_polymer,
            name=tip_name,
        )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.14, 0.0, 0.90)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("rotor_spin")

    ctx.check(
        "rotor uses a vertical continuous axis",
        spin.articulation_type == ArticulationType.CONTINUOUS and tuple(spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )
    ctx.check(
        "rotor axis is offset from the base center",
        abs(spin.origin.xyz[0]) >= 0.10 or abs(spin.origin.xyz[1]) >= 0.10,
        details=f"joint_origin={spin.origin.xyz}",
    )

    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_drum",
        negative_elem="bearing_cap",
        max_gap=0.002,
        max_penetration=1e-6,
        name="hub sits directly on the fixed bearing cap",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="xy",
        elem_a="hub_drum",
        elem_b="bearing_cap",
        min_overlap=0.16,
        name="hub stays centered over the support bearing",
    )

    with ctx.pose({spin: 2.0 * pi / 3.0}):
        ctx.expect_gap(
            rotor,
            frame,
            axis="z",
            negative_elem="drive_pedestal",
            min_gap=0.30,
            name="rotating member stays well above the drive pedestal while turning",
        )
        ctx.expect_overlap(
            rotor,
            frame,
            axes="xy",
            elem_a="hub_drum",
            elem_b="bearing_cap",
            min_overlap=0.16,
            name="hub remains centered over the support at a turned pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
