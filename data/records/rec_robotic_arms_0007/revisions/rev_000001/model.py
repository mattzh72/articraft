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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _yz_section(
    *,
    width: float,
    height: float,
    radius: float,
    x: float,
    z_center: float = 0.0,
    y_center: float = 0.0,
):
    return [
        (x, y + y_center, z + z_center)
        for z, y in rounded_rect_profile(height, width, radius, corner_segments=8)
    ]


def _add_bolt(
    part,
    *,
    xyz: tuple[float, float, float],
    material,
    axis: str = "z",
    radius: float = 0.005,
    length: float = 0.008,
    name: str | None = None,
) -> None:
    if axis == "x":
        rpy = (0.0, math.pi / 2.0, 0.0)
    elif axis == "y":
        rpy = (math.pi / 2.0, 0.0, 0.0)
    else:
        rpy = (0.0, 0.0, 0.0)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_robotic_arm", assets=ASSETS)

    base_coat = model.material("base_coat", rgba=(0.18, 0.20, 0.22, 1.0))
    arm_paint = model.material("arm_paint", rgba=(0.84, 0.44, 0.10, 1.0))
    housing_gray = model.material("housing_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    molded_black = model.material("molded_black", rgba=(0.09, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.12, 1.0))

    upper_link_mesh = _save_mesh(
        "robotic_arm_upper_link.obj",
        section_loft(
            [
                _yz_section(width=0.150, height=0.128, radius=0.028, x=0.000, z_center=0.020),
                _yz_section(width=0.138, height=0.118, radius=0.025, x=0.155, z_center=0.016),
                _yz_section(width=0.118, height=0.102, radius=0.020, x=0.270, z_center=0.006),
            ]
        ),
    )
    forearm_link_mesh = _save_mesh(
        "robotic_arm_forearm_link.obj",
        section_loft(
            [
                _yz_section(width=0.116, height=0.096, radius=0.020, x=0.000, z_center=0.000),
                _yz_section(width=0.104, height=0.088, radius=0.018, x=0.120, z_center=-0.005),
                _yz_section(width=0.090, height=0.078, radius=0.015, x=0.210, z_center=-0.010),
            ]
        ),
    )

    base = model.part("base")
    base.visual(
        Box((0.62, 0.48, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=base_coat,
        name="base_plinth",
    )
    base.visual(
        Box((0.42, 0.30, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=housing_gray,
        name="upper_deck",
    )
    base.visual(
        Box((0.18, 0.18, 0.12)),
        origin=Origin(xyz=(-0.17, 0.0, 0.16)),
        material=housing_gray,
        name="service_cabinet",
    )
    base.visual(
        Cylinder(radius=0.11, length=0.16),
        origin=Origin(xyz=(0.02, 0.0, 0.22)),
        material=base_coat,
        name="pedestal_core",
    )
    base.visual(
        Cylinder(radius=0.17, length=0.03),
        origin=Origin(xyz=(0.02, 0.0, 0.285)),
        material=molded_black,
        name="slew_ring",
    )
    for sx in (-0.24, 0.24):
        for sy in (-0.17, 0.17):
            base.visual(
                Cylinder(radius=0.048, length=0.018),
                origin=Origin(xyz=(sx, sy, 0.009)),
                material=rubber,
            )
    for bolt_y in (-0.055, 0.055):
        for bolt_z in (0.13, 0.18):
            _add_bolt(
                base,
                xyz=(-0.080, bolt_y, bolt_z),
                axis="x",
                radius=0.005,
                length=0.010,
                material=steel,
            )
    for bolt_x in (-0.03, 0.07):
        for bolt_y in (-0.06, 0.06):
            _add_bolt(
                base,
                xyz=(bolt_x, bolt_y, 0.298),
                axis="z",
                radius=0.005,
                length=0.010,
                material=steel,
            )
    base.inertial = Inertial.from_geometry(
        Box((0.62, 0.48, 0.30)),
        mass=52.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    shoulder_carriage = model.part("shoulder_carriage")
    shoulder_carriage.visual(
        Cylinder(radius=0.16, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=molded_black,
        name="turntable_ring",
    )
    shoulder_carriage.visual(
        Box((0.22, 0.22, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=base_coat,
        name="drive_body",
    )
    shoulder_carriage.visual(
        Box((0.16, 0.10, 0.10)),
        origin=Origin(xyz=(-0.11, 0.0, 0.14)),
        material=housing_gray,
        name="rear_drive_pack",
    )
    shoulder_carriage.visual(
        Box((0.14, 0.22, 0.10)),
        origin=Origin(xyz=(-0.01, 0.0, 0.21)),
        material=base_coat,
        name="yoke_bridge",
    )
    shoulder_carriage.visual(
        Box((0.10, 0.12, 0.10)),
        origin=Origin(xyz=(-0.01, 0.0, 0.16)),
        material=base_coat,
        name="yoke_support_core",
    )
    shoulder_carriage.visual(
        Box((0.18, 0.03, 0.26)),
        origin=Origin(xyz=(0.02, 0.105, 0.31)),
        material=arm_paint,
        name="shoulder_yoke_left",
    )
    shoulder_carriage.visual(
        Box((0.18, 0.03, 0.26)),
        origin=Origin(xyz=(0.02, -0.105, 0.31)),
        material=arm_paint,
        name="shoulder_yoke_right",
    )
    shoulder_carriage.visual(
        Box((0.07, 0.065, 0.12)),
        origin=Origin(xyz=(-0.03, 0.145, 0.29)),
        material=housing_gray,
        name="shoulder_actuator_left",
    )
    shoulder_carriage.visual(
        Box((0.07, 0.065, 0.12)),
        origin=Origin(xyz=(-0.03, -0.145, 0.29)),
        material=housing_gray,
        name="shoulder_actuator_right",
    )
    shoulder_carriage.visual(
        Box((0.10, 0.23, 0.03)),
        origin=Origin(xyz=(-0.01, 0.0, 0.432)),
        material=base_coat,
        name="top_bridge_cap",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.05, length=0.014),
        origin=Origin(xyz=(0.02, 0.123, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="shoulder_outer_collar_left",
    )
    shoulder_carriage.visual(
        Cylinder(radius=0.05, length=0.014),
        origin=Origin(xyz=(0.02, -0.123, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="shoulder_outer_collar_right",
    )
    for bolt_x in (-0.05, 0.03):
        for bolt_z in (0.25, 0.33):
            _add_bolt(
                shoulder_carriage,
                xyz=(-0.05 if bolt_x < 0.0 else 0.0, 0.165, bolt_z),
                axis="y",
                length=0.012,
                material=steel,
            )
            _add_bolt(
                shoulder_carriage,
                xyz=(-0.05 if bolt_x < 0.0 else 0.0, -0.165, bolt_z),
                axis="y",
                length=0.012,
                material=steel,
            )
    shoulder_carriage.inertial = Inertial.from_geometry(
        Box((0.26, 0.34, 0.46)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.045, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="shoulder_hub",
    )
    upper_arm.visual(
        Box((0.10, 0.16, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.03)),
        material=arm_paint,
        name="upper_root_block",
    )
    upper_arm.visual(
        upper_link_mesh,
        origin=Origin(xyz=(0.04, 0.0, 0.0)),
        material=arm_paint,
        name="upper_link_shell",
    )
    upper_arm.visual(
        Box((0.20, 0.05, 0.03)),
        origin=Origin(xyz=(0.18, 0.0, 0.075)),
        material=housing_gray,
        name="upper_spine_rib",
    )
    upper_arm.visual(
        Box((0.20, 0.014, 0.06)),
        origin=Origin(xyz=(0.20, 0.057, 0.02)),
        material=housing_gray,
        name="upper_side_plate_left",
    )
    upper_arm.visual(
        Box((0.20, 0.014, 0.06)),
        origin=Origin(xyz=(0.20, -0.057, 0.02)),
        material=housing_gray,
        name="upper_side_plate_right",
    )
    upper_arm.visual(
        Box((0.13, 0.17, 0.08)),
        origin=Origin(xyz=(0.27, 0.0, 0.045)),
        material=housing_gray,
        name="elbow_actuator_housing",
    )
    upper_arm.visual(
        Box((0.065, 0.030, 0.10)),
        origin=Origin(xyz=(0.355, 0.080, 0.0)),
        material=housing_gray,
        name="elbow_knuckle_web_left",
    )
    upper_arm.visual(
        Box((0.065, 0.030, 0.10)),
        origin=Origin(xyz=(0.355, -0.080, 0.0)),
        material=housing_gray,
        name="elbow_knuckle_web_right",
    )
    upper_arm.visual(
        Box((0.07, 0.025, 0.14)),
        origin=Origin(xyz=(0.39, 0.0775, 0.0)),
        material=arm_paint,
        name="elbow_clevis_left",
    )
    upper_arm.visual(
        Box((0.07, 0.025, 0.14)),
        origin=Origin(xyz=(0.39, -0.0775, 0.0)),
        material=arm_paint,
        name="elbow_clevis_right",
    )
    upper_arm.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(xyz=(0.39, 0.097, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="elbow_outer_collar_left",
    )
    upper_arm.visual(
        Cylinder(radius=0.038, length=0.014),
        origin=Origin(xyz=(0.39, -0.097, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="elbow_outer_collar_right",
    )
    for bolt_x in (0.13, 0.23):
        for bolt_z in (0.00, 0.05):
            _add_bolt(
                upper_arm,
                xyz=(bolt_x, 0.068, bolt_z),
                axis="y",
                material=steel,
            )
            _add_bolt(
                upper_arm,
                xyz=(bolt_x, -0.068, bolt_z),
                axis="y",
                material=steel,
            )
    for bolt_x in (0.24, 0.32):
        for bolt_y in (-0.05, 0.05):
            _add_bolt(
                upper_arm,
                xyz=(bolt_x, bolt_y, 0.087),
                axis="z",
                material=steel,
            )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.46, 0.20, 0.18)),
        mass=12.0,
        origin=Origin(xyz=(0.21, 0.0, 0.03)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.04, length=0.13),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="elbow_hub",
    )
    forearm.visual(
        Box((0.09, 0.10, 0.08)),
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        material=arm_paint,
        name="forearm_root_block",
    )
    forearm.visual(
        forearm_link_mesh,
        origin=Origin(xyz=(0.03, 0.0, 0.0)),
        material=arm_paint,
        name="forearm_link_shell",
    )
    forearm.visual(
        Box((0.14, 0.14, 0.06)),
        origin=Origin(xyz=(0.18, 0.0, 0.045)),
        material=housing_gray,
        name="wrist_actuator_housing",
    )
    forearm.visual(
        Box((0.050, 0.024, 0.08)),
        origin=Origin(xyz=(0.275, 0.062, -0.005)),
        material=housing_gray,
        name="wrist_knuckle_web_left",
    )
    forearm.visual(
        Box((0.050, 0.024, 0.08)),
        origin=Origin(xyz=(0.275, -0.062, -0.005)),
        material=housing_gray,
        name="wrist_knuckle_web_right",
    )
    forearm.visual(
        Box((0.16, 0.05, 0.028)),
        origin=Origin(xyz=(0.16, 0.0, -0.050)),
        material=housing_gray,
        name="underside_stiffener",
    )
    forearm.visual(
        Box((0.08, 0.11, 0.016)),
        origin=Origin(xyz=(0.17, 0.0, 0.078)),
        material=base_coat,
        name="service_cover",
    )
    forearm.visual(
        Box((0.055, 0.022, 0.11)),
        origin=Origin(xyz=(0.305, 0.061, -0.005)),
        material=arm_paint,
        name="wrist_clevis_left",
    )
    forearm.visual(
        Box((0.055, 0.022, 0.11)),
        origin=Origin(xyz=(0.305, -0.061, -0.005)),
        material=arm_paint,
        name="wrist_clevis_right",
    )
    forearm.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.305, 0.078, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="wrist_outer_collar_left",
    )
    forearm.visual(
        Cylinder(radius=0.032, length=0.012),
        origin=Origin(xyz=(0.305, -0.078, -0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="wrist_outer_collar_right",
    )
    for bolt_x in (0.14, 0.20):
        for bolt_y in (-0.035, 0.035):
            _add_bolt(
                forearm,
                xyz=(bolt_x, bolt_y, 0.090),
                axis="z",
                material=steel,
                radius=0.0045,
            )
    for bolt_x in (0.11, 0.21):
        for bolt_z in (0.015, 0.045):
            _add_bolt(
                forearm,
                xyz=(bolt_x, 0.056, bolt_z),
                axis="y",
                material=steel,
                radius=0.0045,
            )
            _add_bolt(
                forearm,
                xyz=(bolt_x, -0.056, bolt_z),
                axis="y",
                material=steel,
                radius=0.0045,
            )
    forearm.inertial = Inertial.from_geometry(
        Box((0.35, 0.16, 0.14)),
        mass=8.5,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.032, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=molded_black,
        name="wrist_hub",
    )
    wrist.visual(
        Box((0.09, 0.08, 0.08)),
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        material=arm_paint,
        name="wrist_body",
    )
    wrist.visual(
        Cylinder(radius=0.042, length=0.055),
        origin=Origin(xyz=(0.110, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=housing_gray,
        name="wrist_motor_cap",
    )
    wrist.visual(
        Cylinder(radius=0.036, length=0.030),
        origin=Origin(xyz=(0.150, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.03, 0.07, 0.07)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=housing_gray,
        name="quick_attach_block",
    )
    wrist.visual(
        Box((0.05, 0.012, 0.028)),
        origin=Origin(xyz=(0.215, 0.024, 0.0)),
        material=steel,
        name="tool_finger_upper",
    )
    wrist.visual(
        Box((0.05, 0.012, 0.028)),
        origin=Origin(xyz=(0.215, -0.024, 0.0)),
        material=steel,
        name="tool_finger_lower",
    )
    wrist.visual(
        Box((0.012, 0.010, 0.020)),
        origin=Origin(xyz=(0.238, 0.018, 0.0)),
        material=molded_black,
        name="finger_pad_upper",
    )
    wrist.visual(
        Box((0.012, 0.010, 0.020)),
        origin=Origin(xyz=(0.238, -0.018, 0.0)),
        material=molded_black,
        name="finger_pad_lower",
    )
    wrist.visual(
        Box((0.03, 0.04, 0.018)),
        origin=Origin(xyz=(0.096, 0.0, 0.050)),
        material=base_coat,
        name="sensor_pod",
    )
    for bolt_y in (-0.020, 0.020):
        _add_bolt(
            wrist,
            xyz=(0.134, bolt_y, 0.030),
            axis="x",
            material=steel,
            radius=0.004,
        )
        _add_bolt(
            wrist,
            xyz=(0.182, bolt_y, -0.024),
            axis="x",
            material=steel,
            radius=0.004,
        )
    wrist.inertial = Inertial.from_geometry(
        Box((0.26, 0.10, 0.12)),
        mass=4.0,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shoulder_carriage,
        origin=Origin(xyz=(0.02, 0.0, 0.30)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=240.0, velocity=1.2),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent=shoulder_carriage,
        child=upper_arm,
        origin=Origin(xyz=(0.02, 0.0, 0.34)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.1, lower=-1.10, upper=1.00),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.39, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.4, lower=-1.35, upper=1.20),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=wrist,
        origin=Origin(xyz=(0.305, 0.0, -0.005)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.8, lower=-1.40, upper=1.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    shoulder_carriage = object_model.get_part("shoulder_carriage")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    wrist = object_model.get_part("wrist")

    base_yaw = object_model.get_articulation("base_yaw")
    shoulder_pitch = object_model.get_articulation("shoulder_pitch")
    elbow_pitch = object_model.get_articulation("elbow_pitch")
    wrist_pitch = object_model.get_articulation("wrist_pitch")

    slew_ring = base.get_visual("slew_ring")
    turntable_ring = shoulder_carriage.get_visual("turntable_ring")
    shoulder_yoke_left = shoulder_carriage.get_visual("shoulder_yoke_left")
    shoulder_yoke_right = shoulder_carriage.get_visual("shoulder_yoke_right")
    shoulder_outer_collar_left = shoulder_carriage.get_visual("shoulder_outer_collar_left")
    shoulder_outer_collar_right = shoulder_carriage.get_visual("shoulder_outer_collar_right")
    shoulder_hub = upper_arm.get_visual("shoulder_hub")
    elbow_clevis_left = upper_arm.get_visual("elbow_clevis_left")
    elbow_clevis_right = upper_arm.get_visual("elbow_clevis_right")
    elbow_outer_collar_left = upper_arm.get_visual("elbow_outer_collar_left")
    elbow_outer_collar_right = upper_arm.get_visual("elbow_outer_collar_right")
    elbow_hub = forearm.get_visual("elbow_hub")
    wrist_clevis_left = forearm.get_visual("wrist_clevis_left")
    wrist_clevis_right = forearm.get_visual("wrist_clevis_right")
    wrist_outer_collar_left = forearm.get_visual("wrist_outer_collar_left")
    wrist_outer_collar_right = forearm.get_visual("wrist_outer_collar_right")
    wrist_hub = wrist.get_visual("wrist_hub")
    tool_finger_upper = wrist.get_visual("tool_finger_upper")
    tool_finger_lower = wrist.get_visual("tool_finger_lower")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

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

    ctx.expect_contact(shoulder_carriage, base, elem_a=turntable_ring, elem_b=slew_ring)
    ctx.expect_overlap(
        shoulder_carriage,
        base,
        axes="xy",
        min_overlap=0.28,
        elem_a=turntable_ring,
        elem_b=slew_ring,
    )
    ctx.expect_gap(
        shoulder_carriage,
        base,
        axis="z",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=turntable_ring,
        negative_elem=slew_ring,
    )

    ctx.expect_contact(upper_arm, shoulder_carriage, elem_a=shoulder_hub, elem_b=shoulder_yoke_left)
    ctx.expect_contact(upper_arm, shoulder_carriage, elem_a=shoulder_hub, elem_b=shoulder_yoke_right)
    ctx.expect_gap(
        shoulder_carriage,
        upper_arm,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=shoulder_yoke_left,
        negative_elem=shoulder_hub,
    )
    ctx.expect_gap(
        upper_arm,
        shoulder_carriage,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=shoulder_hub,
        negative_elem=shoulder_yoke_right,
    )
    ctx.expect_gap(
        shoulder_carriage,
        upper_arm,
        axis="y",
        min_gap=0.025,
        positive_elem=shoulder_outer_collar_left,
        negative_elem=shoulder_hub,
    )
    ctx.expect_gap(
        upper_arm,
        shoulder_carriage,
        axis="y",
        min_gap=0.025,
        positive_elem=shoulder_hub,
        negative_elem=shoulder_outer_collar_right,
    )

    ctx.expect_contact(forearm, upper_arm, elem_a=elbow_hub, elem_b=elbow_clevis_left)
    ctx.expect_contact(forearm, upper_arm, elem_a=elbow_hub, elem_b=elbow_clevis_right)
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=elbow_clevis_left,
        negative_elem=elbow_hub,
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=elbow_hub,
        negative_elem=elbow_clevis_right,
    )
    ctx.expect_gap(
        upper_arm,
        forearm,
        axis="y",
        min_gap=0.024,
        positive_elem=elbow_outer_collar_left,
        negative_elem=elbow_hub,
    )
    ctx.expect_gap(
        forearm,
        upper_arm,
        axis="y",
        min_gap=0.024,
        positive_elem=elbow_hub,
        negative_elem=elbow_outer_collar_right,
    )

    ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=wrist_clevis_left)
    ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=wrist_clevis_right)
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=wrist_clevis_left,
        negative_elem=wrist_hub,
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        max_gap=0.0,
        max_penetration=0.0,
        positive_elem=wrist_hub,
        negative_elem=wrist_clevis_right,
    )
    ctx.expect_gap(
        forearm,
        wrist,
        axis="y",
        min_gap=0.020,
        positive_elem=wrist_outer_collar_left,
        negative_elem=wrist_hub,
    )
    ctx.expect_gap(
        wrist,
        forearm,
        axis="y",
        min_gap=0.020,
        positive_elem=wrist_hub,
        negative_elem=wrist_outer_collar_right,
    )

    base_aabb = ctx.part_world_aabb(base)
    wrist_aabb = ctx.part_world_aabb(wrist)
    rest_wrist_pos = ctx.part_world_position(wrist)
    upper_finger_rest = ctx.part_element_world_aabb(wrist, elem=tool_finger_upper)
    lower_finger_rest = ctx.part_element_world_aabb(wrist, elem=tool_finger_lower)

    if base_aabb is not None:
        base_dx = base_aabb[1][0] - base_aabb[0][0]
        base_dy = base_aabb[1][1] - base_aabb[0][1]
        ctx.check(
            "stable_base_footprint",
            base_dx >= 0.60 and base_dy >= 0.45 and base_aabb[1][2] >= 0.29,
            f"Base footprint is {base_dx:.3f} x {base_dy:.3f} m with top z={base_aabb[1][2]:.3f}.",
        )
    else:
        ctx.fail("stable_base_footprint", "Base world AABB was unavailable.")

    if wrist_aabb is not None and base_aabb is not None:
        reach_x = wrist_aabb[1][0] - base_aabb[0][0]
        ctx.check(
            "reach_is_practical",
            reach_x >= 0.95 and wrist_aabb[1][2] >= 0.65,
            f"Reach span to wrist is {reach_x:.3f} m with wrist top z={wrist_aabb[1][2]:.3f}.",
        )
    else:
        ctx.fail("reach_is_practical", "Could not measure base or wrist bounds.")

    if upper_finger_rest is not None and lower_finger_rest is not None:
        jaw_gap = upper_finger_rest[0][1] - lower_finger_rest[1][1]
        ctx.check(
            "serviceable_tool_gap",
            jaw_gap >= 0.030,
            f"Tool jaw gap is {jaw_gap:.3f} m.",
        )
    else:
        ctx.fail("serviceable_tool_gap", "Could not measure tool finger bounds.")

    if rest_wrist_pos is None:
        ctx.fail("rest_wrist_pose_available", "Wrist world position was unavailable in the rest pose.")
    else:
        with ctx.pose({base_yaw: 1.00}):
            yaw_wrist_pos = ctx.part_world_position(wrist)
            if yaw_wrist_pos is None:
                ctx.fail("base_yaw_moves_arm", "Wrist world position was unavailable in yaw pose.")
            else:
                ctx.check(
                    "base_yaw_moves_arm",
                    abs(yaw_wrist_pos[1]) >= 0.55 and yaw_wrist_pos[0] < rest_wrist_pos[0] - 0.20,
                    (
                        f"Rest wrist position {rest_wrist_pos}, yaw wrist position {yaw_wrist_pos}; "
                        "expected a large lateral sweep."
                    ),
                )
            ctx.expect_contact(shoulder_carriage, base, elem_a=turntable_ring, elem_b=slew_ring)

        with ctx.pose({shoulder_pitch: 0.65}):
            shoulder_wrist_pos = ctx.part_world_position(wrist)
            if shoulder_wrist_pos is None:
                ctx.fail("shoulder_pitch_lifts_chain", "Wrist world position was unavailable in shoulder pose.")
            else:
                ctx.check(
                    "shoulder_pitch_lifts_chain",
                    shoulder_wrist_pos[2] >= rest_wrist_pos[2] + 0.35
                    and shoulder_wrist_pos[0] <= rest_wrist_pos[0] - 0.12,
                    (
                        f"Rest wrist position {rest_wrist_pos}, shoulder-pitched wrist position "
                        f"{shoulder_wrist_pos}."
                    ),
                )
            ctx.expect_contact(upper_arm, shoulder_carriage, elem_a=shoulder_hub, elem_b=shoulder_yoke_left)
            ctx.expect_contact(upper_arm, shoulder_carriage, elem_a=shoulder_hub, elem_b=shoulder_yoke_right)

        with ctx.pose({elbow_pitch: 0.95}):
            elbow_wrist_pos = ctx.part_world_position(wrist)
            if elbow_wrist_pos is None:
                ctx.fail("elbow_pitch_folds_chain", "Wrist world position was unavailable in elbow pose.")
            else:
                ctx.check(
                    "elbow_pitch_folds_chain",
                    elbow_wrist_pos[2] >= rest_wrist_pos[2] + 0.12
                    and elbow_wrist_pos[0] <= rest_wrist_pos[0] - 0.10,
                    f"Rest wrist position {rest_wrist_pos}, elbow-folded wrist position {elbow_wrist_pos}.",
                )
            ctx.expect_contact(forearm, upper_arm, elem_a=elbow_hub, elem_b=elbow_clevis_left)
            ctx.expect_contact(forearm, upper_arm, elem_a=elbow_hub, elem_b=elbow_clevis_right)

        with ctx.pose({wrist_pitch: 0.85}):
            upper_finger_pose = ctx.part_element_world_aabb(wrist, elem=tool_finger_upper)
            if upper_finger_pose is None or upper_finger_rest is None:
                ctx.fail("wrist_pitch_rotates_tool", "Could not measure tool finger bounds in wrist pose.")
            else:
                ctx.check(
                    "wrist_pitch_rotates_tool",
                    upper_finger_pose[1][2] >= upper_finger_rest[1][2] + 0.04
                    and upper_finger_pose[0][0] <= upper_finger_rest[0][0] - 0.01,
                    (
                        f"Rest finger bounds {upper_finger_rest}, posed finger bounds "
                        f"{upper_finger_pose}."
                    ),
                )
            ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=wrist_clevis_left)
            ctx.expect_contact(wrist, forearm, elem_a=wrist_hub, elem_b=wrist_clevis_right)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
