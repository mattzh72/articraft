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
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    wire_from_points,
)


PLATE_RADIUS = 0.19
PLATE_RIM_RADIUS = PLATE_RADIUS * math.cos(math.pi / 6.0)
ARM_LENGTH = 0.39
ARM_FOLD_ANGLE = math.radians(110.0)
PROP_RADIUS = 0.145
CENTER_STACK_HEIGHT = 0.048
ARM_AXLE_HEIGHT = 0.066


def _save_mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _hex_profile(radius: float, *, angle_offset: float = math.pi / 6.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(angle_offset + index * math.tau / 6.0),
            radius * math.sin(angle_offset + index * math.tau / 6.0),
        )
        for index in range(6)
    ]


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    corner: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_center + z_pos)
        for y_pos, z_pos in rounded_rect_profile(width, height, corner, corner_segments=6)
    ]


def _blade_section(
    span_x: float,
    *,
    chord: float,
    thickness: float,
    pitch_deg: float,
    y_offset: float,
    z_center: float,
) -> list[tuple[float, float, float]]:
    pitch = math.radians(pitch_deg)
    half_chord = chord * 0.5
    half_thickness = thickness * 0.5
    profile = [
        (-0.54 * half_chord, 0.00 * half_thickness),
        (-0.20 * half_chord, 0.95 * half_thickness),
        (0.34 * half_chord, 0.55 * half_thickness),
        (0.52 * half_chord, 0.00 * half_thickness),
        (0.18 * half_chord, -0.55 * half_thickness),
        (-0.30 * half_chord, -0.40 * half_thickness),
    ]
    section: list[tuple[float, float, float]] = []
    for y_local, z_local in profile:
        y_rot = y_local * math.cos(pitch) - z_local * math.sin(pitch)
        z_rot = y_local * math.sin(pitch) + z_local * math.cos(pitch)
        section.append((span_x, y_offset + y_rot, z_center + z_rot))
    return section


def _build_arm_boom_mesh() -> MeshGeometry:
    return section_loft(
        [
            _yz_section(-0.006, width=0.040, height=0.020, corner=0.006, z_center=0.010),
            _yz_section(0.090, width=0.035, height=0.018, corner=0.005, z_center=0.010),
            _yz_section(0.250, width=0.028, height=0.015, corner=0.004, z_center=0.013),
            _yz_section(0.355, width=0.024, height=0.013, corner=0.003, z_center=0.018),
        ]
    )


def _build_propeller_blades_mesh() -> MeshGeometry:
    single_blade = section_loft(
        [
            _blade_section(0.026, chord=0.058, thickness=0.009, pitch_deg=24.0, y_offset=0.000, z_center=0.006),
            _blade_section(0.072, chord=0.046, thickness=0.0065, pitch_deg=15.0, y_offset=0.006, z_center=0.0065),
            _blade_section(0.118, chord=0.030, thickness=0.0040, pitch_deg=7.0, y_offset=0.013, z_center=0.0065),
            _blade_section(0.145, chord=0.018, thickness=0.0026, pitch_deg=3.0, y_offset=0.018, z_center=0.0065),
        ]
    )
    blades = single_blade.copy()
    blades.merge(single_blade.copy().rotate_z(math.pi))
    return blades


def _arm_specs() -> list[dict[str, float | int | str]]:
    specs: list[dict[str, float | int | str]] = []
    arm_labels = ("front", "front_right", "rear_right", "rear", "rear_left", "front_left")
    for index, label in enumerate(arm_labels):
        angle = index * math.tau / 6.0
        fold_sign = 1.0 if index % 2 == 0 else -1.0
        specs.append(
            {
                "index": index,
                "label": label,
                "angle": angle,
                "fold_sign": fold_sign,
                "hinge_x": PLATE_RIM_RADIUS * math.cos(angle),
                "hinge_y": PLATE_RIM_RADIUS * math.sin(angle),
            }
        )
    return specs


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_hexarotor")

    carbon = model.material("carbon", rgba=(0.12, 0.13, 0.14, 1.0))
    carbon_plate = model.material("carbon_plate", rgba=(0.09, 0.10, 0.11, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.60, 0.63, 0.67, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.24, 0.26, 0.29, 1.0))
    motor_bell = model.material("motor_bell", rgba=(0.78, 0.40, 0.10, 1.0))
    prop_black = model.material("prop_black", rgba=(0.07, 0.08, 0.09, 1.0))
    skid_black = model.material("skid_black", rgba=(0.08, 0.08, 0.09, 1.0))

    center_frame = model.part("center_frame")
    center_frame.inertial = Inertial.from_geometry(
        Box((0.42, 0.42, 0.16)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    lower_plate_mesh = _save_mesh("lower_center_plate", ExtrudeGeometry(_hex_profile(PLATE_RADIUS), 0.004))
    upper_plate_mesh = _save_mesh("upper_center_plate", ExtrudeGeometry(_hex_profile(PLATE_RADIUS * 0.92), 0.004))
    center_frame.visual(
        lower_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=carbon_plate,
        name="lower_plate",
    )
    center_frame.visual(
        upper_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=carbon_plate,
        name="upper_plate",
    )

    for index in range(6):
        angle = index * math.tau / 6.0 + math.pi / 6.0
        standoff_x = 0.088 * math.cos(angle)
        standoff_y = 0.088 * math.sin(angle)
        center_frame.visual(
            Cylinder(radius=0.008, length=0.024),
            origin=Origin(xyz=(standoff_x, standoff_y, 0.034)),
            material=anodized_aluminum,
            name=f"standoff_{index}",
        )

    center_frame.visual(
        Box((0.150, 0.090, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=dark_aluminum,
        name="avionics_pod",
    )
    center_frame.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.071)),
        material=anodized_aluminum,
        name="gps_puck",
    )

    skid_left = wire_from_points(
        [
            (-0.095, -0.120, 0.020),
            (-0.095, -0.120, -0.075),
            (-0.095, 0.120, -0.075),
            (-0.095, 0.120, 0.020),
        ],
        radius=0.006,
        radial_segments=16,
        corner_mode="fillet",
        corner_radius=0.028,
        corner_segments=10,
        cap_ends=True,
    )
    skid_right = wire_from_points(
        [
            (0.095, -0.120, 0.020),
            (0.095, -0.120, -0.075),
            (0.095, 0.120, -0.075),
            (0.095, 0.120, 0.020),
        ],
        radius=0.006,
        radial_segments=16,
        corner_mode="fillet",
        corner_radius=0.028,
        corner_segments=10,
        cap_ends=True,
    )
    center_frame.visual(_save_mesh("left_landing_skid", skid_left), material=skid_black, name="left_skid")
    center_frame.visual(_save_mesh("right_landing_skid", skid_right), material=skid_black, name="right_skid")

    arm_boom_mesh = _save_mesh("hexarotor_arm_boom", _build_arm_boom_mesh())
    prop_blades_mesh = _save_mesh("hexarotor_propeller_blades", _build_propeller_blades_mesh())

    for arm_spec in _arm_specs():
        label = str(arm_spec["label"])
        angle = float(arm_spec["angle"])
        fold_sign = float(arm_spec["fold_sign"])
        hinge_origin = Origin(
            xyz=(float(arm_spec["hinge_x"]), float(arm_spec["hinge_y"]), 0.022),
            rpy=(0.0, 0.0, angle),
        )

        center_frame.visual(
            Cylinder(radius=0.016, length=0.006),
            origin=Origin(xyz=(float(arm_spec["hinge_x"]), float(arm_spec["hinge_y"]), 0.019)),
            material=anodized_aluminum,
            name=f"{label}_hinge_pedestal",
        )
        for cheek_index, lateral_offset in enumerate((-0.020, 0.020)):
            center_frame.visual(
                Box((0.020, 0.008, 0.018)),
                origin=Origin(
                    xyz=(
                        float(arm_spec["hinge_x"])
                        - 0.019 * math.cos(angle)
                        - lateral_offset * math.sin(angle),
                        float(arm_spec["hinge_y"])
                        - 0.019 * math.sin(angle)
                        + lateral_offset * math.cos(angle),
                        0.027,
                    ),
                    rpy=(0.0, 0.0, angle),
                ),
                material=dark_aluminum,
                name=f"{label}_hinge_cheek_{cheek_index}",
            )

        arm = model.part(f"{label}_arm")
        arm.inertial = Inertial.from_geometry(
            Box((0.44, 0.05, 0.08)),
            mass=0.55,
            origin=Origin(xyz=(0.20, 0.0, 0.03)),
        )
        arm.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
            material=dark_aluminum,
            name="hinge_collar",
        )
        arm.visual(arm_boom_mesh, material=carbon, name="arm_boom")
        arm.visual(
            Box((0.050, 0.050, 0.006)),
            origin=Origin(xyz=(ARM_LENGTH - 0.036, 0.0, 0.026)),
            material=dark_aluminum,
            name="motor_mount_plate",
        )
        arm.visual(
            Cylinder(radius=0.031, length=0.030),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.044)),
            material=motor_bell,
            name="motor_can",
        )
        arm.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(ARM_LENGTH, 0.0, 0.061)),
            material=dark_aluminum,
            name="motor_cap",
        )

        arm_lower = 0.0 if fold_sign > 0.0 else -ARM_FOLD_ANGLE
        arm_upper = ARM_FOLD_ANGLE if fold_sign > 0.0 else 0.0
        arm_joint = model.articulation(
            f"{label}_arm_fold",
            ArticulationType.REVOLUTE,
            parent=center_frame,
            child=arm,
            origin=hinge_origin,
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=1.4,
                lower=arm_lower,
                upper=arm_upper,
            ),
        )

        prop = model.part(f"{label}_propeller")
        prop.inertial = Inertial.from_geometry(
            Cylinder(radius=PROP_RADIUS, length=0.020),
            mass=0.11,
            origin=Origin(xyz=(0.0, 0.0, 0.010)),
        )
        prop.visual(
            Cylinder(radius=0.026, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=dark_aluminum,
            name="prop_hub",
        )
        prop.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.009)),
            material=anodized_aluminum,
            name="prop_cap",
        )
        prop.visual(prop_blades_mesh, material=prop_black, name="prop_blades")

        model.articulation(
            f"{label}_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=arm,
            child=prop,
            origin=Origin(xyz=(ARM_LENGTH, 0.0, ARM_AXLE_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=45.0),
        )

        arm.meta["fold_joint"] = arm_joint.name

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

    required_parts = ["center_frame"]
    required_joints: list[str] = []
    for arm_spec in _arm_specs():
        label = str(arm_spec["label"])
        required_parts.extend([f"{label}_arm", f"{label}_propeller"])
        required_joints.extend([f"{label}_arm_fold", f"{label}_prop_spin"])

    resolved_parts = {}
    resolved_joints = {}

    for part_name in required_parts:
        try:
            resolved_parts[part_name] = object_model.get_part(part_name)
            ctx.check(f"{part_name}_exists", True)
        except Exception as exc:
            ctx.fail(f"{part_name}_exists", str(exc))

    for joint_name in required_joints:
        try:
            resolved_joints[joint_name] = object_model.get_articulation(joint_name)
            ctx.check(f"{joint_name}_exists", True)
        except Exception as exc:
            ctx.fail(f"{joint_name}_exists", str(exc))

    if ctx.report().failures:
        return ctx.report()

    center_frame = resolved_parts["center_frame"]

    for arm_spec in _arm_specs():
        label = str(arm_spec["label"])
        fold_sign = float(arm_spec["fold_sign"])
        arm = resolved_parts[f"{label}_arm"]
        prop = resolved_parts[f"{label}_propeller"]
        arm_joint = resolved_joints[f"{label}_arm_fold"]
        prop_joint = resolved_joints[f"{label}_prop_spin"]

        ctx.expect_contact(arm, center_frame, name=f"{label}_arm_contacts_center_frame")
        ctx.expect_contact(prop, arm, name=f"{label}_propeller_contacts_arm")
        ctx.expect_origin_distance(
            arm,
            center_frame,
            axes="xy",
            min_dist=PLATE_RIM_RADIUS - 0.005,
            max_dist=PLATE_RIM_RADIUS + 0.005,
            name=f"{label}_arm_hinge_at_plate_rim",
        )
        ctx.expect_origin_distance(
            prop,
            center_frame,
            axes="xy",
            min_dist=0.54,
            max_dist=0.57,
            name=f"{label}_propeller_at_arm_tip_radius",
        )

        ctx.check(
            f"{label}_arm_fold_axis_vertical",
            tuple(round(v, 6) for v in arm_joint.axis) == (0.0, 0.0, 1.0),
            f"Unexpected fold axis {arm_joint.axis}",
        )
        ctx.check(
            f"{label}_prop_spin_axis_vertical",
            tuple(round(v, 6) for v in prop_joint.axis) == (0.0, 0.0, 1.0),
            f"Unexpected prop axis {prop_joint.axis}",
        )
        ctx.check(
            f"{label}_propeller_is_continuous",
            prop_joint.articulation_type == ArticulationType.CONTINUOUS,
            f"Expected CONTINUOUS, got {prop_joint.articulation_type}",
        )

        arm_limits = arm_joint.motion_limits
        prop_limits = prop_joint.motion_limits
        assert arm_limits is not None
        assert prop_limits is not None
        expected_lower = 0.0 if fold_sign > 0.0 else -ARM_FOLD_ANGLE
        expected_upper = ARM_FOLD_ANGLE if fold_sign > 0.0 else 0.0
        ctx.check(
            f"{label}_arm_fold_limits_direction",
            abs((arm_limits.lower or 0.0) - expected_lower) < 1e-6
            and abs((arm_limits.upper or 0.0) - expected_upper) < 1e-6,
            (
                f"Expected limits ({expected_lower:.3f}, {expected_upper:.3f}) "
                f"got ({arm_limits.lower}, {arm_limits.upper})"
            ),
        )
        ctx.check(
            f"{label}_propeller_velocity_range",
            prop_limits.velocity >= 30.0,
            f"Velocity too low for prop spin: {prop_limits.velocity}",
        )

    front_arm_joint = resolved_joints["front_arm_fold"]
    front_prop = resolved_parts["front_propeller"]
    front_rest = ctx.part_world_position(front_prop)
    assert front_rest is not None
    front_rest_radius = math.hypot(front_rest[0], front_rest[1])
    with ctx.pose({front_arm_joint: ARM_FOLD_ANGLE * 0.92}):
        front_folded = ctx.part_world_position(front_prop)
        assert front_folded is not None
        front_folded_radius = math.hypot(front_folded[0], front_folded[1])
        ctx.check(
            "front_arm_folds_inward",
            front_folded_radius < front_rest_radius - 0.12,
            f"Folded radius {front_folded_radius:.3f} was not sufficiently inside rest radius {front_rest_radius:.3f}",
        )
        ctx.check(
            "front_arm_preserves_propeller_height_while_folding",
            abs(front_folded[2] - front_rest[2]) < 0.002,
            f"Unexpected height shift from {front_rest[2]:.3f} to {front_folded[2]:.3f}",
        )
        ctx.expect_contact(resolved_parts["front_arm"], center_frame, name="front_arm_stays_mounted_when_folded")
        ctx.expect_contact(front_prop, resolved_parts["front_arm"], name="front_propeller_stays_mounted_when_folded")

    front_right_arm_joint = resolved_joints["front_right_arm_fold"]
    front_right_prop = resolved_parts["front_right_propeller"]
    right_rest = ctx.part_world_position(front_right_prop)
    assert right_rest is not None
    right_rest_radius = math.hypot(right_rest[0], right_rest[1])
    with ctx.pose({front_right_arm_joint: -ARM_FOLD_ANGLE * 0.92}):
        right_folded = ctx.part_world_position(front_right_prop)
        assert right_folded is not None
        right_folded_radius = math.hypot(right_folded[0], right_folded[1])
        ctx.check(
            "front_right_arm_folds_inward",
            right_folded_radius < right_rest_radius - 0.12,
            f"Folded radius {right_folded_radius:.3f} was not sufficiently inside rest radius {right_rest_radius:.3f}",
        )
        ctx.expect_contact(
            resolved_parts["front_right_arm"],
            center_frame,
            name="front_right_arm_stays_mounted_when_folded",
        )
        ctx.expect_contact(
            front_right_prop,
            resolved_parts["front_right_arm"],
            name="front_right_propeller_stays_mounted_when_folded",
        )

    front_prop_spin = resolved_joints["front_prop_spin"]
    front_spin_rest = ctx.part_world_position(front_prop)
    assert front_spin_rest is not None
    with ctx.pose({front_prop_spin: 1.4}):
        front_spin_pose = ctx.part_world_position(front_prop)
        assert front_spin_pose is not None
        ctx.check(
            "front_propeller_spin_keeps_axle_position",
            all(abs(a - b) < 1e-6 for a, b in zip(front_spin_rest, front_spin_pose)),
            f"Propeller moved while spinning: {front_spin_rest} -> {front_spin_pose}",
        )
        ctx.expect_contact(front_prop, resolved_parts["front_arm"], name="front_propeller_contacts_arm_while_spinning")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
