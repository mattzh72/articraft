from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_remote_weapon_station")

    pedestal_gray = model.material("pedestal_gray", rgba=(0.53, 0.57, 0.60, 1.0))
    armor_gray = model.material("armor_gray", rgba=(0.46, 0.50, 0.53, 1.0))
    gun_gray = model.material("gun_gray", rgba=(0.22, 0.24, 0.27, 1.0))
    optics_black = model.material("optics_black", rgba=(0.08, 0.09, 0.11, 1.0))
    sensor_glass = model.material("sensor_glass", rgba=(0.10, 0.18, 0.22, 1.0))

    def add_box(part, size, xyz, *, rpy=(0.0, 0.0, 0.0), material=None, name=None):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_cylinder(part, radius, length, xyz, *, rpy=(0.0, 0.0, 0.0), material=None, name=None):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    pedestal = model.part("pedestal")
    add_cylinder(
        pedestal,
        0.50,
        0.10,
        (0.0, 0.0, 0.05),
        material=pedestal_gray,
        name="deck_skirt",
    )
    add_cylinder(
        pedestal,
        0.22,
        0.58,
        (0.0, 0.0, 0.39),
        material=pedestal_gray,
        name="pedestal_column",
    )
    add_cylinder(
        pedestal,
        0.30,
        0.08,
        (0.0, 0.0, 0.72),
        material=pedestal_gray,
        name="azimuth_shoulder",
    )
    add_cylinder(
        pedestal,
        0.36,
        0.04,
        (0.0, 0.0, 0.74),
        material=armor_gray,
        name="azimuth_bearing_housing",
    )
    add_box(
        pedestal,
        (0.72, 0.16, 0.05),
        (0.0, 0.0, 0.055),
        material=pedestal_gray,
        name="service_access_band",
    )

    upper_frame = model.part("upper_frame")
    add_cylinder(
        upper_frame,
        0.34,
        0.04,
        (0.0, 0.0, 0.02),
        material=armor_gray,
        name="azimuth_ring",
    )
    add_box(
        upper_frame,
        (0.36, 0.44, 0.08),
        (-0.03, 0.0, 0.08),
        material=armor_gray,
        name="frame_waist",
    )
    add_box(
        upper_frame,
        (0.70, 0.05, 0.50),
        (0.06, 0.245, 0.31),
        material=armor_gray,
        name="left_cheek_plate",
    )
    add_box(
        upper_frame,
        (0.70, 0.05, 0.50),
        (0.06, -0.245, 0.31),
        material=armor_gray,
        name="right_cheek_plate",
    )
    add_box(
        upper_frame,
        (0.24, 0.44, 0.06),
        (-0.10, 0.0, 0.59),
        material=armor_gray,
        name="upper_rear_bridge",
    )
    add_box(
        upper_frame,
        (0.08, 0.44, 0.14),
        (-0.30, 0.0, 0.36),
        material=armor_gray,
        name="rear_crosshead",
    )
    add_box(
        upper_frame,
        (0.22, 0.44, 0.10),
        (-0.10, 0.0, 0.14),
        material=armor_gray,
        name="lower_crossmember",
    )
    add_box(
        upper_frame,
        (0.18, 0.05, 0.16),
        (0.34, 0.245, 0.41),
        rpy=(0.0, -0.42, 0.0),
        material=armor_gray,
        name="left_front_armor",
    )
    add_box(
        upper_frame,
        (0.18, 0.05, 0.16),
        (0.34, -0.245, 0.41),
        rpy=(0.0, -0.42, 0.0),
        material=armor_gray,
        name="right_front_armor",
    )
    add_box(
        upper_frame,
        (0.10, 0.07, 0.16),
        (0.00, 0.305, 0.52),
        material=armor_gray,
        name="sensor_hinge_bracket",
    )
    add_box(
        upper_frame,
        (0.12, 0.03, 0.05),
        (0.00, 0.325, 0.58),
        material=armor_gray,
        name="sensor_hinge_saddle",
    )
    add_cylinder(
        upper_frame,
        0.016,
        0.028,
        (-0.044, 0.355, 0.58),
        rpy=(0.0, pi / 2.0, 0.0),
        material=gun_gray,
        name="sensor_hinge_knuckle_a",
    )
    add_cylinder(
        upper_frame,
        0.016,
        0.028,
        (0.044, 0.355, 0.58),
        rpy=(0.0, pi / 2.0, 0.0),
        material=gun_gray,
        name="sensor_hinge_knuckle_b",
    )

    weapon_cradle = model.part("weapon_cradle")
    add_box(
        weapon_cradle,
        (0.46, 0.28, 0.24),
        (0.09, 0.0, -0.02),
        material=armor_gray,
        name="cradle_body",
    )
    add_box(
        weapon_cradle,
        (0.28, 0.20, 0.10),
        (0.00, 0.0, 0.12),
        material=armor_gray,
        name="top_armor",
    )
    add_box(
        weapon_cradle,
        (0.16, 0.32, 0.18),
        (-0.18, 0.0, -0.02),
        material=armor_gray,
        name="rear_counterweight",
    )
    add_box(
        weapon_cradle,
        (0.20, 0.28, 0.12),
        (0.36, 0.0, 0.02),
        rpy=(0.0, -0.46, 0.0),
        material=armor_gray,
        name="front_glacis",
    )
    add_cylinder(
        weapon_cradle,
        0.055,
        0.08,
        (0.0, 0.18, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=gun_gray,
        name="left_trunnion",
    )
    add_cylinder(
        weapon_cradle,
        0.055,
        0.08,
        (0.0, -0.18, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=gun_gray,
        name="right_trunnion",
    )
    add_cylinder(
        weapon_cradle,
        0.065,
        0.18,
        (0.40, 0.0, -0.02),
        rpy=(0.0, pi / 2.0, 0.0),
        material=gun_gray,
        name="barrel_sleeve",
    )
    add_cylinder(
        weapon_cradle,
        0.038,
        0.78,
        (0.82, 0.0, -0.02),
        rpy=(0.0, pi / 2.0, 0.0),
        material=gun_gray,
        name="gun_barrel",
    )
    add_cylinder(
        weapon_cradle,
        0.026,
        0.24,
        (1.33, 0.0, -0.02),
        rpy=(0.0, pi / 2.0, 0.0),
        material=gun_gray,
        name="muzzle_section",
    )
    add_box(
        weapon_cradle,
        (0.22, 0.16, 0.08),
        (0.18, 0.0, -0.16),
        material=gun_gray,
        name="recoil_support",
    )

    sensor_mast = model.part("sensor_mast")
    add_cylinder(
        sensor_mast,
        0.015,
        0.06,
        (0.0, 0.0, 0.0),
        rpy=(0.0, pi / 2.0, 0.0),
        material=gun_gray,
        name="mast_hinge_barrel",
    )
    add_box(
        sensor_mast,
        (0.05, 0.04, 0.06),
        (0.0, 0.03, 0.03),
        material=armor_gray,
        name="mast_hinge_leaf",
    )
    add_box(
        sensor_mast,
        (0.12, 0.10, 0.46),
        (0.0, 0.05, 0.29),
        material=armor_gray,
        name="mast_column",
    )
    add_box(
        sensor_mast,
        (0.20, 0.14, 0.12),
        (0.02, 0.05, 0.58),
        material=armor_gray,
        name="sensor_head",
    )
    add_box(
        sensor_mast,
        (0.08, 0.10, 0.08),
        (0.14, 0.05, 0.58),
        material=optics_black,
        name="camera_fairing",
    )
    add_cylinder(
        sensor_mast,
        0.032,
        0.07,
        (0.20, 0.05, 0.58),
        rpy=(0.0, pi / 2.0, 0.0),
        material=sensor_glass,
        name="electro_optic_window",
    )
    add_cylinder(
        sensor_mast,
        0.012,
        0.14,
        (-0.02, 0.05, 0.71),
        material=gun_gray,
        name="antenna_post",
    )

    model.articulation(
        "pedestal_to_upper_frame",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=upper_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2),
    )
    model.articulation(
        "upper_frame_to_weapon_cradle",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=weapon_cradle,
        origin=Origin(xyz=(0.03, 0.0, 0.34)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.0,
            lower=-0.35,
            upper=1.05,
        ),
    )
    model.articulation(
        "upper_frame_to_sensor_mast",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=sensor_mast,
        origin=Origin(xyz=(0.0, 0.355, 0.58)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-1.10,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    upper_frame = object_model.get_part("upper_frame")
    weapon_cradle = object_model.get_part("weapon_cradle")
    sensor_mast = object_model.get_part("sensor_mast")

    azimuth = object_model.get_articulation("pedestal_to_upper_frame")
    elevation = object_model.get_articulation("upper_frame_to_weapon_cradle")
    mast_hinge = object_model.get_articulation("upper_frame_to_sensor_mast")

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

    ctx.check(
        "azimuth axis is vertical",
        tuple(azimuth.axis) == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {azimuth.axis}",
    )
    ctx.check(
        "elevation axis is horizontal",
        tuple(elevation.axis) == (0.0, 1.0, 0.0),
        f"expected horizontal elevation axis, got {elevation.axis}",
    )
    ctx.check(
        "sensor mast hinge axis is transverse",
        tuple(mast_hinge.axis) == (1.0, 0.0, 0.0),
        f"expected transverse mast hinge axis, got {mast_hinge.axis}",
    )

    ctx.expect_contact(
        upper_frame,
        pedestal,
        contact_tol=1e-5,
        name="upper frame sits on azimuth bearing",
    )
    ctx.expect_overlap(
        upper_frame,
        pedestal,
        axes="xy",
        min_overlap=0.50,
        name="upper frame remains centered on pedestal",
    )
    ctx.expect_contact(
        weapon_cradle,
        upper_frame,
        contact_tol=1e-5,
        name="cradle trunnions seat between cheek plates",
    )
    ctx.expect_contact(
        sensor_mast,
        upper_frame,
        contact_tol=1e-5,
        name="sensor mast hinge is attached to upper frame",
    )
    ctx.expect_origin_gap(
        sensor_mast,
        upper_frame,
        axis="y",
        min_gap=0.30,
        name="sensor mast is side mounted outboard of the frame",
    )

    with ctx.pose({azimuth: 1.1}):
        ctx.expect_contact(
            upper_frame,
            pedestal,
            contact_tol=1e-5,
            name="azimuth bearing stays seated while rotating",
        )

    with ctx.pose({elevation: 0.70}):
        ctx.expect_contact(
            weapon_cradle,
            upper_frame,
            contact_tol=1e-5,
            name="elevation cradle stays captured in the cheek plates",
        )

    with ctx.pose({mast_hinge: -0.80}):
        ctx.expect_contact(
            sensor_mast,
            upper_frame,
            contact_tol=1e-5,
            name="sensor mast stays clipped to the folding hinge when deployed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
