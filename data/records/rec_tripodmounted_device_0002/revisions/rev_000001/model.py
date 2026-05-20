from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


LEG_ANGLES = (
    ("front_leg", 0.0),
    ("rear_left_leg", 2.0 * math.pi / 3.0),
    ("rear_right_leg", 4.0 * math.pi / 3.0),
)


def _yaw_xy(theta: float, x: float, y: float) -> tuple[float, float]:
    return (
        x * math.cos(theta) - y * math.sin(theta),
        x * math.sin(theta) + y * math.cos(theta),
    )


def _place_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    material=None,
    name: str | None = None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _place_cylinder_x(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material=None,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _place_cylinder_y(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    yaw: float = 0.0,
    material=None,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _place_cylinder_z(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material=None,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_bolt_ring(
    part,
    *,
    radius: float,
    bolt_radius: float,
    bolt_height: float,
    count: int,
    z: float,
    material=None,
    prefix: str,
) -> None:
    for index in range(count):
        angle = 2.0 * math.pi * index / count
        part.visual(
            Cylinder(radius=bolt_radius, length=bolt_height),
            origin=Origin(xyz=(radius * math.cos(angle), radius * math.sin(angle), z)),
            material=material,
            name=f"{prefix}_{index + 1}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_tripod_mounted_device", assets=ASSETS)

    steel = model.material("powder_steel", rgba=(0.22, 0.24, 0.26, 1.0))
    polymer = model.material("molded_polymer", rgba=(0.12, 0.14, 0.16, 1.0))
    zinc = model.material("zinc_fastener", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("rubber_boot", rgba=(0.08, 0.08, 0.08, 1.0))
    accent = model.material("utility_accent", rgba=(0.79, 0.47, 0.12, 1.0))
    glass = model.material("optic_window", rgba=(0.30, 0.43, 0.50, 0.92))

    head = model.part("tripod_head")
    pan = model.part("pan_carriage")
    device = model.part("device_unit")
    front_leg = model.part("front_leg")
    rear_left_leg = model.part("rear_left_leg")
    rear_right_leg = model.part("rear_right_leg")

    _place_box(
        head,
        (0.32, 0.24, 0.022),
        (0.0, 0.0, 0.011),
        material=steel,
        name="head_plate",
    )
    _place_box(
        head,
        (0.17, 0.13, 0.048),
        (0.0, 0.0, -0.024),
        material=steel,
        name="head_core",
    )
    _place_cylinder_z(
        head,
        radius=0.062,
        length=0.090,
        xyz=(0.0, 0.0, -0.056),
        material=steel,
        name="center_column",
    )
    _place_cylinder_z(
        head,
        radius=0.080,
        length=0.014,
        xyz=(0.0, 0.0, 0.029),
        material=steel,
        name="turntable_pad",
    )
    _place_box(
        head,
        (0.050, 0.050, 0.018),
        (0.115, 0.0, 0.031),
        material=polymer,
        name="service_cap",
    )
    _place_box(
        head,
        (0.055, 0.095, 0.050),
        (0.0, 0.0, -0.108),
        material=polymer,
        name="center_socket",
    )
    _add_bolt_ring(
        head,
        radius=0.097,
        bolt_radius=0.006,
        bolt_height=0.006,
        count=6,
        z=0.025,
        material=zinc,
        prefix="plate_bolt",
    )

    leg_joint_radius = 0.128
    leg_joint_z = -0.037
    for leg_name, theta in LEG_ANGLES:
        gusset_xy = _yaw_xy(theta, 0.060, 0.0)
        boss_xy = _yaw_xy(theta, 0.112, -0.010)
        ear_xy = _yaw_xy(theta, 0.112, 0.010)
        crown_xy = _yaw_xy(theta, leg_joint_radius, 0.0)
        _place_box(
            head,
            (0.096, 0.074, 0.058),
            (gusset_xy[0], gusset_xy[1], -0.036),
            rpy=(0.0, 0.0, theta),
            material=steel,
            name=f"{leg_name}_gusset",
        )
        _place_box(
            head,
            (0.022, 0.010, 0.040),
            (boss_xy[0], boss_xy[1], leg_joint_z),
            rpy=(0.0, 0.0, theta),
            material=steel,
            name=f"{leg_name}_hinge_boss",
        )
        _place_box(
            head,
            (0.022, 0.010, 0.040),
            (ear_xy[0], ear_xy[1], leg_joint_z),
            rpy=(0.0, 0.0, theta),
            material=steel,
            name=f"{leg_name}_hinge_ear",
        )
        _place_cylinder_z(
            head,
            radius=0.007,
            length=0.010,
            xyz=(crown_xy[0], crown_xy[1], 0.026),
            material=zinc,
            name=f"{leg_name}_crown_bolt",
        )

    _place_cylinder_z(
        pan,
        radius=0.080,
        length=0.010,
        xyz=(0.0, 0.0, 0.005),
        material=steel,
        name="bearing_disk",
    )
    _place_cylinder_z(
        pan,
        radius=0.062,
        length=0.028,
        xyz=(0.0, 0.0, 0.022),
        material=steel,
        name="rotating_collar",
    )
    _place_box(
        pan,
        (0.336, 0.120, 0.020),
        (0.0, 0.0, 0.046),
        material=steel,
        name="yoke_base",
    )
    _place_box(
        pan,
        (0.032, 0.124, 0.290),
        (-0.166, 0.0, 0.201),
        material=steel,
        name="left_yoke_arm",
    )
    _place_box(
        pan,
        (0.032, 0.124, 0.290),
        (0.166, 0.0, 0.201),
        material=steel,
        name="right_yoke_arm",
    )
    _place_box(
        pan,
        (0.022, 0.044, 0.120),
        (-0.183, -0.040, 0.246),
        material=steel,
        name="rear_brace",
    )
    _place_box(
        pan,
        (0.022, 0.044, 0.120),
        (0.183, 0.040, 0.246),
        material=steel,
        name="front_tie",
    )
    _place_cylinder_x(
        pan,
        radius=0.022,
        length=0.024,
        xyz=(-0.151, 0.0, 0.320),
        material=steel,
        name="left_tilt_boss",
    )
    _place_cylinder_x(
        pan,
        radius=0.022,
        length=0.024,
        xyz=(0.151, 0.0, 0.320),
        material=steel,
        name="right_tilt_boss",
    )
    _place_cylinder_x(
        pan,
        radius=0.034,
        length=0.034,
        xyz=(-0.184, 0.0, 0.320),
        material=accent,
        name="left_tilt_lock",
    )
    _place_cylinder_x(
        pan,
        radius=0.034,
        length=0.034,
        xyz=(0.184, 0.0, 0.320),
        material=accent,
        name="right_tilt_lock",
    )
    _place_box(
        pan,
        (0.020, 0.056, 0.020),
        (0.110, 0.052, 0.045),
        rpy=(0.0, 0.0, 0.35),
        material=accent,
        name="pan_lock_handle",
    )
    _add_bolt_ring(
        pan,
        radius=0.048,
        bolt_radius=0.0045,
        bolt_height=0.006,
        count=4,
        z=0.059,
        material=zinc,
        prefix="pan_bolt",
    )

    _place_box(
        device,
        (0.222, 0.070, 0.164),
        (0.0, 0.0, -0.112),
        material=polymer,
        name="device_shell",
    )
    _place_box(
        device,
        (0.198, 0.010, 0.110),
        (0.0, 0.034, -0.112),
        material=glass,
        name="front_window",
    )
    _place_box(
        device,
        (0.198, 0.030, 0.022),
        (0.0, 0.0, -0.028),
        material=steel,
        name="top_rail",
    )
    _place_box(
        device,
        (0.176, 0.018, 0.094),
        (0.0, -0.030, -0.122),
        material=steel,
        name="rear_service_pack",
    )
    _place_box(
        device,
        (0.012, 0.082, 0.178),
        (-0.117, 0.0, -0.113),
        material=rubber,
        name="left_guard",
    )
    _place_box(
        device,
        (0.012, 0.082, 0.178),
        (0.117, 0.0, -0.113),
        material=rubber,
        name="right_guard",
    )
    _place_box(
        device,
        (0.070, 0.012, 0.036),
        (0.088, 0.036, -0.146),
        material=polymer,
        name="control_panel",
    )
    _place_cylinder_x(
        device,
        radius=0.020,
        length=0.036,
        xyz=(-0.121, 0.0, 0.0),
        material=steel,
        name="left_trunnion",
    )
    _place_cylinder_x(
        device,
        radius=0.020,
        length=0.036,
        xyz=(0.121, 0.0, 0.0),
        material=steel,
        name="right_trunnion",
    )
    _place_box(
        device,
        (0.028, 0.018, 0.074),
        (-0.107, 0.0, -0.025),
        material=steel,
        name="left_trunnion_bracket",
    )
    _place_box(
        device,
        (0.028, 0.018, 0.074),
        (0.107, 0.0, -0.025),
        material=steel,
        name="right_trunnion_bracket",
    )
    for sign_x in (-1.0, 1.0):
        for sign_z in (-1.0, 1.0):
            _place_cylinder_y(
                device,
                radius=0.0045,
                length=0.010,
                xyz=(0.088 * sign_x, 0.040, -0.112 + 0.040 * sign_z),
                material=zinc,
                name=f"bezel_fastener_{int((sign_x + 1.0) * 1.5 + (sign_z + 1.0) * 0.5)}",
            )
    _place_cylinder_z(
        device,
        radius=0.007,
        length=0.010,
        xyz=(0.088, 0.042, -0.138),
        material=accent,
        name="control_knob",
    )

    leg_pitch = math.pi - math.radians(23.0)
    leg_dir_x = math.sin(math.radians(23.0))
    leg_dir_z = -math.cos(math.radians(23.0))

    def build_leg(leg_part, foot_name: str) -> None:
        _place_box(
            leg_part,
            (0.024, 0.026, 0.084),
            (leg_dir_x * 0.042, 0.0, leg_dir_z * 0.042),
            rpy=(0.0, leg_pitch, 0.0),
            material=steel,
            name="hinge_collar",
        )
        _place_box(
            leg_part,
            (0.066, 0.042, 0.520),
            (leg_dir_x * 0.302, 0.0, leg_dir_z * 0.302),
            rpy=(0.0, leg_pitch, 0.0),
            material=steel,
            name="upper_strut",
        )
        _place_box(
            leg_part,
            (0.082, 0.056, 0.096),
            (leg_dir_x * 0.592, 0.0, leg_dir_z * 0.592),
            rpy=(0.0, leg_pitch, 0.0),
            material=polymer,
            name="leg_clamp",
        )
        _place_box(
            leg_part,
            (0.058, 0.038, 0.560),
            (leg_dir_x * 0.898, 0.0, leg_dir_z * 0.898),
            rpy=(0.0, leg_pitch, 0.0),
            material=steel,
            name="lower_strut",
        )
        _place_box(
            leg_part,
            (0.066, 0.048, 0.098),
            (leg_dir_x * 1.166, 0.0, leg_dir_z * 1.166),
            rpy=(0.0, leg_pitch, 0.0),
            material=steel,
            name="ankle_block",
        )
        _place_box(
            leg_part,
            (0.094, 0.058, 0.074),
            (leg_dir_x * 1.225, 0.0, leg_dir_z * 1.225),
            rpy=(0.0, leg_pitch, 0.0),
            material=rubber,
            name=foot_name,
        )
        _place_box(
            leg_part,
            (0.020, 0.054, 0.540),
            (leg_dir_x * 0.474 - 0.016, 0.0, leg_dir_z * 0.474),
            rpy=(0.0, leg_pitch, 0.0),
            material=steel,
            name="rear_rib",
        )
        _place_cylinder_y(
            leg_part,
            radius=0.006,
            length=0.018,
            xyz=(leg_dir_x * 0.060, 0.018, leg_dir_z * 0.060),
            material=zinc,
            name="outer_pivot_bolt",
        )
        _place_cylinder_y(
            leg_part,
            radius=0.006,
            length=0.018,
            xyz=(leg_dir_x * 0.060, -0.018, leg_dir_z * 0.060),
            material=zinc,
            name="inner_pivot_bolt",
        )

    build_leg(front_leg, "front_foot")
    build_leg(rear_left_leg, "rear_left_foot")
    build_leg(rear_right_leg, "rear_right_foot")

    model.articulation(
        "head_to_front_leg",
        ArticulationType.REVOLUTE,
        parent=head,
        child=front_leg,
        origin=Origin(xyz=(leg_joint_radius, 0.0, leg_joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.18,
        ),
    )
    model.articulation(
        "head_to_rear_left_leg",
        ArticulationType.REVOLUTE,
        parent=head,
        child=rear_left_leg,
        origin=Origin(xyz=(_yaw_xy(LEG_ANGLES[1][1], leg_joint_radius, 0.0)[0], _yaw_xy(LEG_ANGLES[1][1], leg_joint_radius, 0.0)[1], leg_joint_z), rpy=(0.0, 0.0, LEG_ANGLES[1][1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.18,
        ),
    )
    model.articulation(
        "head_to_rear_right_leg",
        ArticulationType.REVOLUTE,
        parent=head,
        child=rear_right_leg,
        origin=Origin(xyz=(_yaw_xy(LEG_ANGLES[2][1], leg_joint_radius, 0.0)[0], _yaw_xy(LEG_ANGLES[2][1], leg_joint_radius, 0.0)[1], leg_joint_z), rpy=(0.0, 0.0, LEG_ANGLES[2][1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.2,
            lower=-0.10,
            upper=0.18,
        ),
    )
    model.articulation(
        "head_to_pan",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=pan,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5),
    )
    model.articulation(
        "pan_to_device",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=device,
        origin=Origin(xyz=(0.0, 0.0, 0.320)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.0,
            lower=-0.35,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    head = object_model.get_part("tripod_head")
    pan = object_model.get_part("pan_carriage")
    device = object_model.get_part("device_unit")
    front_leg = object_model.get_part("front_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")

    head_to_front_leg = object_model.get_articulation("head_to_front_leg")
    head_to_rear_left_leg = object_model.get_articulation("head_to_rear_left_leg")
    head_to_rear_right_leg = object_model.get_articulation("head_to_rear_right_leg")
    head_to_pan = object_model.get_articulation("head_to_pan")
    pan_to_device = object_model.get_articulation("pan_to_device")

    head_pad = head.get_visual("turntable_pad")
    front_boss = head.get_visual("front_leg_hinge_boss")
    front_ear = head.get_visual("front_leg_hinge_ear")
    left_boss = head.get_visual("rear_left_leg_hinge_boss")
    left_ear = head.get_visual("rear_left_leg_hinge_ear")
    right_boss = head.get_visual("rear_right_leg_hinge_boss")
    right_ear = head.get_visual("rear_right_leg_hinge_ear")

    pan_bearing = pan.get_visual("bearing_disk")
    pan_left_boss = pan.get_visual("left_tilt_boss")
    pan_right_boss = pan.get_visual("right_tilt_boss")

    left_trunnion = device.get_visual("left_trunnion")
    right_trunnion = device.get_visual("right_trunnion")
    front_collar = front_leg.get_visual("hinge_collar")
    left_collar = rear_left_leg.get_visual("hinge_collar")
    right_collar = rear_right_leg.get_visual("hinge_collar")
    front_foot = front_leg.get_visual("front_foot")
    rear_left_foot = rear_left_leg.get_visual("rear_left_foot")
    rear_right_foot = rear_right_leg.get_visual("rear_right_foot")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    ctx.allow_overlap(
        head,
        front_leg,
        elem_a=front_boss,
        elem_b=front_collar,
        reason="Simplified front clevis omits axle bore and bushing clearance, so the hinge boss slightly shares volume with the captured leg root.",
    )
    ctx.allow_overlap(
        head,
        front_leg,
        elem_a=front_ear,
        elem_b=front_collar,
        reason="Simplified front clevis omits axle bore and bushing clearance, so the opposite cheek slightly shares volume with the captured leg root.",
    )
    ctx.allow_overlap(
        head,
        rear_left_leg,
        elem_a=left_boss,
        elem_b=left_collar,
        reason="Simplified rear-left clevis omits axle bore and bushing clearance, so the hinge cheek slightly shares volume with the captured leg root.",
    )
    ctx.allow_overlap(
        head,
        rear_left_leg,
        elem_a=left_ear,
        elem_b=left_collar,
        reason="Simplified rear-left clevis omits axle bore and bushing clearance, so the opposite cheek slightly shares volume with the captured leg root.",
    )
    ctx.allow_overlap(
        head,
        rear_right_leg,
        elem_a=right_boss,
        elem_b=right_collar,
        reason="Simplified rear-right clevis omits axle bore and bushing clearance, so the hinge cheek slightly shares volume with the captured leg root.",
    )
    ctx.allow_overlap(
        head,
        rear_right_leg,
        elem_a=right_ear,
        elem_b=right_collar,
        reason="Simplified rear-right clevis omits axle bore and bushing clearance, so the opposite cheek slightly shares volume with the captured leg root.",
    )

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    for part in (head, pan, device, front_leg, rear_left_leg, rear_right_leg):
        ctx.check(
            f"{part.name}_has_geometry",
            len(part.visuals) > 0,
            details=f"{part.name} should have authored visual geometry",
        )

    ctx.expect_contact(
        head,
        front_leg,
        elem_a=front_boss,
        elem_b=front_collar,
        contact_tol=0.001,
        name="front_leg_hinge_contacts_head",
    )
    ctx.expect_contact(
        head,
        rear_left_leg,
        elem_a=left_boss,
        elem_b=left_collar,
        contact_tol=0.001,
        name="rear_left_leg_hinge_contacts_head",
    )
    ctx.expect_contact(
        head,
        rear_right_leg,
        elem_a=right_boss,
        elem_b=right_collar,
        contact_tol=0.001,
        name="rear_right_leg_hinge_contacts_head",
    )
    ctx.expect_contact(
        head,
        pan,
        elem_a=head_pad,
        elem_b=pan_bearing,
        contact_tol=0.001,
        name="pan_bearing_contacts_head_plate",
    )
    ctx.expect_contact(
        pan,
        device,
        elem_a=pan_left_boss,
        elem_b=left_trunnion,
        contact_tol=0.0015,
        name="left_tilt_boss_contacts_left_trunnion",
    )
    ctx.expect_contact(
        pan,
        device,
        elem_a=pan_right_boss,
        elem_b=right_trunnion,
        contact_tol=0.0015,
        name="right_tilt_boss_contacts_right_trunnion",
    )
    ctx.expect_origin_distance(
        device,
        head,
        axes="xy",
        max_dist=0.02,
        name="device_stays_centered_over_tripod",
    )
    ctx.expect_gap(
        device,
        head,
        axis="z",
        min_gap=0.002,
        name="device_clears_head_plate",
    )
    ctx.expect_overlap(
        head,
        pan,
        axes="xy",
        min_overlap=0.12,
        elem_a=head_pad,
        elem_b=pan_bearing,
        name="pan_bearing_has_broad_support_overlap",
    )

    foot_aabbs = [
        ctx.part_element_world_aabb(front_leg, elem=front_foot),
        ctx.part_element_world_aabb(rear_left_leg, elem=rear_left_foot),
        ctx.part_element_world_aabb(rear_right_leg, elem=rear_right_foot),
    ]
    feet_ok = all(aabb is not None for aabb in foot_aabbs)
    if feet_ok:
        foot_centers = [
            (
                (aabb[0][0] + aabb[1][0]) / 2.0,
                (aabb[0][1] + aabb[1][1]) / 2.0,
                (aabb[0][2] + aabb[1][2]) / 2.0,
            )
            for aabb in foot_aabbs
            if aabb is not None
        ]
        pair_dists = []
        for i in range(len(foot_centers)):
            for j in range(i + 1, len(foot_centers)):
                dx = foot_centers[i][0] - foot_centers[j][0]
                dy = foot_centers[i][1] - foot_centers[j][1]
                pair_dists.append(math.hypot(dx, dy))
        ctx.check(
            "tripod_feet_form_wide_triangle",
            min(pair_dists) >= 0.78,
            details=f"foot spacing too tight: {pair_dists}",
        )
    else:
        ctx.fail("tripod_feet_form_wide_triangle", "could not resolve foot AABBs")

    part_bounds = [
        ctx.part_world_aabb(part)
        for part in (head, pan, device, front_leg, rear_left_leg, rear_right_leg)
    ]
    if all(bounds is not None for bounds in part_bounds):
        mins = [min(bounds[0][axis] for bounds in part_bounds if bounds is not None) for axis in range(3)]
        maxs = [max(bounds[1][axis] for bounds in part_bounds if bounds is not None) for axis in range(3)]
        total_size = [maxs[axis] - mins[axis] for axis in range(3)]
        ctx.check(
            "overall_height_is_realistic",
            1.25 <= total_size[2] <= 1.70,
            details=f"overall height {total_size[2]:.3f} m outside realistic tripod range",
        )
        ctx.check(
            "overall_footprint_is_realistic",
            total_size[0] >= 0.95 and total_size[1] >= 0.95,
            details=f"footprint too small: {total_size[0]:.3f} x {total_size[1]:.3f} m",
        )
    else:
        ctx.fail("overall_bounds_resolve", "one or more part AABBs were unavailable")

    for joint in (head_to_front_leg, head_to_rear_left_leg, head_to_rear_right_leg, pan_to_device):
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_pose_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_pose_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_pose_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_pose_no_floating")

    with ctx.pose({head_to_pan: math.pi / 2.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_quarter_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="pan_quarter_turn_no_floating")

    with ctx.pose({head_to_pan: math.pi, pan_to_device: 0.65}):
        ctx.fail_if_parts_overlap_in_current_pose(name="pan_half_turn_and_tilt_no_overlap")
        ctx.fail_if_isolated_parts(name="pan_half_turn_and_tilt_no_floating")

    ctx.check(
        "leg_joint_ranges_are_practical",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower >= -0.30
            and joint.motion_limits.upper <= 0.35
            for joint in (head_to_front_leg, head_to_rear_left_leg, head_to_rear_right_leg)
        ),
        details="tripod leg spread should remain within practical field tripod limits",
    )
    ctx.check(
        "device_tilt_range_is_practical",
        pan_to_device.motion_limits is not None
        and pan_to_device.motion_limits.lower is not None
        and pan_to_device.motion_limits.upper is not None
        and pan_to_device.motion_limits.lower >= -0.70
        and pan_to_device.motion_limits.upper <= 1.20,
        details="device tilt should look like a real yoke-mounted utility bracket",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
