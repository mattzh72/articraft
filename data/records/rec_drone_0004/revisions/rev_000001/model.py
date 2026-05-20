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
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


ARM_THICKNESS = 0.006
MOTOR_RADIUS = 0.016
MOTOR_TOP_RADIUS = 0.011
MOTOR_RADIAL_OFFSET = 0.105
PROP_RADIUS = 0.062

ROTOR_LAYOUT = {
    "front_left": math.pi / 4.0,
    "front_right": -math.pi / 4.0,
    "rear_left": 3.0 * math.pi / 4.0,
    "rear_right": -3.0 * math.pi / 4.0,
}


def _polar_xy(radius: float, angle: float) -> tuple[float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle))


def _arm_profile() -> list[tuple[float, float]]:
    return [
        (-0.026, -0.010),
        (-0.020, -0.013),
        (0.006, -0.012),
        (0.028, -0.0095),
        (0.060, -0.0080),
        (0.080, -0.0080),
        (0.092, -0.0120),
        (0.097, -0.0085),
        (0.100, 0.0000),
        (0.097, 0.0085),
        (0.092, 0.0120),
        (0.080, 0.0080),
        (0.060, 0.0080),
        (0.028, 0.0095),
        (0.006, 0.0120),
        (-0.020, 0.0130),
        (-0.026, 0.0100),
    ]


def _make_arm_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(_arm_profile(), ARM_THICKNESS),
        "racing_quad_x_arm",
    )


def _canopy_section(
    x_pos: float,
    width: float,
    height: float,
    corner_radius: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + height * 0.5)
        for y_pos, z_pos in rounded_rect_profile(width, height, corner_radius)
    ]


def _make_canopy_mesh():
    sections = [
        _canopy_section(-0.024, 0.024, 0.0015, 0.0007),
        _canopy_section(-0.008, 0.038, 0.0045, 0.0016),
        _canopy_section(0.014, 0.050, 0.0060, 0.0020),
        _canopy_section(0.034, 0.032, 0.0038, 0.0012),
    ]
    return mesh_from_geometry(section_loft(sections), "racing_quad_canopy")


def _propeller_blade_profile() -> list[tuple[float, float]]:
    return [
        (0.010, -0.0068),
        (0.018, -0.0098),
        (0.034, -0.0090),
        (0.049, -0.0052),
        (0.060, -0.0010),
        (0.062, 0.0024),
        (0.052, 0.0058),
        (0.036, 0.0086),
        (0.018, 0.0095),
        (0.010, 0.0060),
    ]


def _make_propeller_blade_mesh():
    blade = ExtrudeGeometry(_propeller_blade_profile(), 0.002)
    opposite_blade = blade.copy().rotate_z(math.pi)
    blade.merge(opposite_blade)
    return mesh_from_geometry(blade, "racing_quad_two_blade_prop")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="racing_quadrotor")

    carbon = model.material("carbon", rgba=(0.12, 0.12, 0.13, 1.0))
    aluminum = model.material("aluminum", rgba=(0.46, 0.48, 0.51, 1.0))
    shell_black = model.material("shell_black", rgba=(0.09, 0.09, 0.10, 1.0))
    prop_black = model.material("prop_black", rgba=(0.15, 0.15, 0.16, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.18, 0.21, 0.25, 0.92))

    arm_mesh = _make_arm_mesh()
    canopy_mesh = _make_canopy_mesh()
    propeller_blade_mesh = _make_propeller_blade_mesh()

    frame = model.part("frame")
    frame.visual(
        Box((0.090, 0.056, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=carbon,
        name="lower_plate",
    )
    frame.visual(
        Box((0.062, 0.044, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0205)),
        material=carbon,
        name="upper_plate",
    )
    frame.visual(
        Box((0.036, 0.028, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material=shell_black,
        name="stack_cover",
    )

    for index, (post_x, post_y) in enumerate(
        ((-0.020, -0.015), (-0.020, 0.015), (0.020, -0.015), (0.020, 0.015)),
        start=1,
    ):
        frame.visual(
            Cylinder(radius=0.003, length=0.015),
            origin=Origin(xyz=(post_x, post_y, 0.0115)),
            material=aluminum,
            name=f"stack_post_{index}",
        )

    frame.visual(
        canopy_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=shell_black,
        name="canopy",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.034, 0.0, 0.0235), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="camera_lens",
    )

    for rotor_name, angle in ROTOR_LAYOUT.items():
        arm_x, arm_y = _polar_xy(0.028, angle)
        motor_x, motor_y = _polar_xy(MOTOR_RADIAL_OFFSET, angle)
        frame.visual(
            arm_mesh,
            origin=Origin(xyz=(arm_x, arm_y, ARM_THICKNESS * 0.5), rpy=(0.0, 0.0, angle)),
            material=carbon,
            name=f"{rotor_name}_arm",
        )
        frame.visual(
            Cylinder(radius=MOTOR_RADIUS, length=0.018),
            origin=Origin(xyz=(motor_x, motor_y, 0.009)),
            material=aluminum,
            name=f"{rotor_name}_motor_pod",
        )
        frame.visual(
            Cylinder(radius=MOTOR_TOP_RADIUS, length=0.002),
            origin=Origin(xyz=(motor_x, motor_y, 0.019)),
            material=shell_black,
            name=f"{rotor_name}_motor_top",
        )

        prop = model.part(f"{rotor_name}_prop")
        prop.visual(
            Cylinder(radius=0.0024, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=aluminum,
            name="shaft",
        )
        prop.visual(
            Cylinder(radius=0.012, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=shell_black,
            name="hub",
        )
        prop.visual(
            propeller_blade_mesh,
            origin=Origin(
                xyz=(0.0, 0.0, 0.011),
                rpy=(0.0, 0.0, math.pi / 2.0 if rotor_name in {"front_right", "rear_left"} else 0.0),
            ),
            material=prop_black,
            name="blades",
        )
        prop.visual(
            Cylinder(radius=0.0045, length=0.003),
            origin=Origin(xyz=(0.0, 0.0, 0.0115)),
            material=aluminum,
            name="locknut",
        )

        model.articulation(
            f"frame_to_{rotor_name}_prop",
            ArticulationType.CONTINUOUS,
            parent=frame,
            child=prop,
            origin=Origin(xyz=(motor_x, motor_y, 0.020)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.2, velocity=120.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    required_parts = {"frame"} | {f"{name}_prop" for name in ROTOR_LAYOUT}
    required_joints = {f"frame_to_{name}_prop" for name in ROTOR_LAYOUT}

    missing_parts: list[str] = []
    for part_name in sorted(required_parts):
        try:
            object_model.get_part(part_name)
        except Exception:
            missing_parts.append(part_name)

    missing_joints: list[str] = []
    for joint_name in sorted(required_joints):
        try:
            object_model.get_articulation(joint_name)
        except Exception:
            missing_joints.append(joint_name)

    ctx.check(
        "required_parts_present",
        not missing_parts,
        f"Missing parts: {', '.join(missing_parts)}",
    )
    ctx.check(
        "required_joints_present",
        not missing_joints,
        f"Missing articulations: {', '.join(missing_joints)}",
    )

    if missing_parts or missing_joints:
        return ctx.report()

    frame = object_model.get_part("frame")

    for rotor_name in ROTOR_LAYOUT:
        prop = object_model.get_part(f"{rotor_name}_prop")
        joint = object_model.get_articulation(f"frame_to_{rotor_name}_prop")
        limits = joint.motion_limits

        ctx.check(
            f"{rotor_name}_prop_joint_type",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{joint.name} should be CONTINUOUS",
        )
        ctx.check(
            f"{rotor_name}_prop_joint_axis",
            tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0),
            f"{joint.name} axis is {joint.axis}",
        )
        ctx.check(
            f"{rotor_name}_prop_joint_limits",
            limits is not None
            and limits.lower is None
            and limits.upper is None
            and limits.velocity >= 50.0,
            f"{joint.name} motion limits are {limits}",
        )

        ctx.expect_contact(
            prop,
            frame,
            elem_a="shaft",
            elem_b=f"{rotor_name}_motor_top",
            name=f"{rotor_name}_shaft_contacts_motor",
        )
        ctx.expect_overlap(
            prop,
            frame,
            axes="xy",
            elem_a="hub",
            elem_b=f"{rotor_name}_motor_pod",
            min_overlap=0.022,
            name=f"{rotor_name}_hub_centered_on_motor",
        )
        ctx.expect_gap(
            prop,
            frame,
            axis="z",
            positive_elem="blades",
            negative_elem=f"{rotor_name}_motor_top",
            min_gap=0.009,
            name=f"{rotor_name}_blades_clear_motor_top",
        )

    front_left_prop = object_model.get_part("front_left_prop")
    front_left_joint = object_model.get_articulation("frame_to_front_left_prop")
    with ctx.pose({front_left_joint: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="front_left_prop_clear_at_quarter_turn")
        ctx.expect_contact(
            front_left_prop,
            frame,
            elem_a="shaft",
            elem_b="front_left_motor_top",
            name="front_left_prop_stays_seated_when_rotated",
        )
        ctx.expect_gap(
            front_left_prop,
            frame,
            axis="z",
            positive_elem="blades",
            negative_elem="canopy",
            min_gap=0.002,
            name="front_left_prop_clears_canopy_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
