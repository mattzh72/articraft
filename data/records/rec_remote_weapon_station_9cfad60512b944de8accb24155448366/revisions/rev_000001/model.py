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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 24) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def _plate_mesh(name: str, profile: list[tuple[float, float]], *, thickness: float):
    return _save_mesh(
        name,
        ExtrudeGeometry(profile, height=thickness, center=True).rotate_x(math.pi / 2.0),
    )


def _plate_with_hole_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    hole_profile: list[tuple[float, float]],
    *,
    thickness: float,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer_profile,
            [hole_profile],
            height=thickness,
            center=True,
        ).rotate_x(math.pi / 2.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shielded_remote_weapon_station")

    desert_tan = model.material("desert_tan", rgba=(0.68, 0.62, 0.50, 1.0))
    armor_tan = model.material("armor_tan", rgba=(0.62, 0.57, 0.46, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.20, 0.21, 0.23, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    optics_black = model.material("optics_black", rgba=(0.09, 0.10, 0.11, 1.0))
    glass = model.material("glass", rgba=(0.16, 0.30, 0.34, 0.55))

    armor_plate_profile = [
        (-0.06, -0.16),
        (0.08, -0.16),
        (0.18, -0.06),
        (0.34, -0.06),
        (0.34, 0.22),
        (0.10, 0.22),
        (-0.02, 0.10),
        (-0.06, 0.02),
    ]
    armor_plate_mesh = _plate_mesh("cradle_armor_plate", armor_plate_profile, thickness=0.015)

    yoke_outer_profile = [
        (-0.09, -0.065),
        (0.03, -0.065),
        (0.03, 0.065),
        (-0.09, 0.065),
    ]
    yoke_hole_profile = _circle_profile(0.0105, segments=20)
    yoke_cheek_mesh = _plate_with_hole_mesh(
        "optic_yoke_cheek",
        yoke_outer_profile,
        yoke_hole_profile,
        thickness=0.008,
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.78, 0.78, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_gray,
        name="base_plinth",
    )
    pedestal.visual(
        Cylinder(radius=0.17, length=0.88),
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
        material=desert_tan,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.21, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        material=gunmetal,
        name="pedestal_cap",
    )
    pedestal.visual(
        Box((0.18, 0.14, 0.30)),
        origin=Origin(xyz=(-0.13, 0.0, 0.33)),
        material=desert_tan,
        name="service_box",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 1.12)),
        mass=780.0,
        origin=Origin(xyz=(0.0, 0.0, 0.56)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(
        Cylinder(radius=0.085, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=gunmetal,
        name="slew_ring",
    )
    azimuth_stage.visual(
        Box((0.22, 0.14, 0.02)),
        origin=Origin(xyz=(-0.03, 0.0, 0.05)),
        material=gunmetal,
        name="turret_deck",
    )
    azimuth_stage.visual(
        Box((0.18, 0.22, 0.16)),
        origin=Origin(xyz=(-0.18, 0.0, 0.14)),
        material=desert_tan,
        name="rear_drive_housing",
    )
    azimuth_stage.visual(
        Box((0.13, 0.05, 0.06)),
        origin=Origin(xyz=(-0.075, -0.145, 0.13)),
        material=armor_tan,
        name="left_support_bridge",
    )
    azimuth_stage.visual(
        Box((0.13, 0.05, 0.06)),
        origin=Origin(xyz=(-0.075, 0.145, 0.13)),
        material=armor_tan,
        name="right_support_bridge",
    )
    azimuth_stage.visual(
        Box((0.030, 0.290, 0.060)),
        origin=Origin(xyz=(-0.075, 0.0, 0.13)),
        material=armor_tan,
        name="trunnion_bulkhead",
    )
    azimuth_stage.visual(
        Box((0.09, 0.05, 0.18)),
        origin=Origin(xyz=(0.02, -0.145, 0.13)),
        material=armor_tan,
        name="left_trunnion_support",
    )
    azimuth_stage.visual(
        Box((0.09, 0.05, 0.18)),
        origin=Origin(xyz=(0.02, 0.145, 0.13)),
        material=armor_tan,
        name="right_trunnion_support",
    )
    azimuth_stage.visual(
        Box((0.08, 0.05, 0.16)),
        origin=Origin(xyz=(-0.18, -0.135, 0.24)),
        material=desert_tan,
        name="sensor_pedestal",
    )
    azimuth_stage.visual(
        Box((0.12, 0.09, 0.06)),
        origin=Origin(xyz=(-0.18, -0.155, 0.35)),
        material=desert_tan,
        name="sensor_head_base",
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((0.52, 0.40, 0.34)),
        mass=210.0,
        origin=Origin(xyz=(-0.02, 0.0, 0.17)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.028, length=0.24),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="trunnion_shaft",
    )
    cradle.visual(
        armor_plate_mesh,
        origin=Origin(xyz=(0.0, -0.095, 0.0)),
        material=armor_tan,
        name="left_armor_plate",
    )
    cradle.visual(
        armor_plate_mesh,
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        material=armor_tan,
        name="right_armor_plate",
    )
    cradle.visual(
        Box((0.16, 0.19, 0.05)),
        origin=Origin(xyz=(0.02, 0.0, -0.10)),
        material=gunmetal,
        name="lower_crosshead",
    )
    cradle.visual(
        Box((0.08, 0.19, 0.05)),
        origin=Origin(xyz=(-0.02, 0.0, 0.14)),
        material=gunmetal,
        name="upper_rear_bridge",
    )
    cradle.visual(
        Box((0.32, 0.12, 0.16)),
        origin=Origin(xyz=(0.12, 0.0, 0.00)),
        material=gunmetal,
        name="weapon_receiver",
    )
    cradle.visual(
        Box((0.20, 0.12, 0.04)),
        origin=Origin(xyz=(0.10, 0.0, 0.10)),
        material=dark_gray,
        name="feed_cover",
    )
    cradle.visual(
        Box((0.16, 0.05, 0.06)),
        origin=Origin(xyz=(-0.02, 0.0, 0.02)),
        material=dark_gray,
        name="recoil_breech",
    )
    cradle.visual(
        Cylinder(radius=0.030, length=0.88),
        origin=Origin(xyz=(0.66, 0.0, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gunmetal,
        name="barrel",
    )
    cradle.visual(
        Cylinder(radius=0.042, length=0.16),
        origin=Origin(xyz=(1.06, 0.0, 0.00), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="muzzle_device",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((1.22, 0.30, 0.42)),
        mass=165.0,
        origin=Origin(xyz=(0.40, 0.0, 0.01)),
    )

    ammo_box = model.part("ammo_box")
    ammo_box.visual(
        Box((0.30, 0.14, 0.26)),
        material=desert_tan,
        name="ammo_box_shell",
    )
    ammo_box.visual(
        Box((0.30, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=armor_tan,
        name="ammo_box_lid",
    )
    ammo_box.visual(
        Box((0.05, 0.02, 0.07)),
        origin=Origin(xyz=(0.11, 0.08, 0.00)),
        material=gunmetal,
        name="ammo_box_latch",
    )
    ammo_box.visual(
        Box((0.14, 0.035, 0.14)),
        origin=Origin(xyz=(-0.090, 0.0875, 0.010)),
        material=gunmetal,
        name="ammo_mount_bracket",
    )
    ammo_box.inertial = Inertial.from_geometry(
        Box((0.30, 0.14, 0.28)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    side_yoke = model.part("side_yoke")
    side_yoke.visual(
        yoke_cheek_mesh,
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=armor_tan,
        name="inner_yoke_cheek",
    )
    side_yoke.visual(
        yoke_cheek_mesh,
        origin=Origin(xyz=(0.0, 0.024, 0.0)),
        material=armor_tan,
        name="outer_yoke_cheek",
    )
    side_yoke.visual(
        Box((0.040, 0.056, 0.018)),
        origin=Origin(xyz=(-0.068, 0.0, 0.050)),
        material=gunmetal,
        name="yoke_top_bridge",
    )
    side_yoke.visual(
        Box((0.040, 0.056, 0.018)),
        origin=Origin(xyz=(-0.068, 0.0, -0.050)),
        material=gunmetal,
        name="yoke_bottom_bridge",
    )
    side_yoke.visual(
        Box((0.026, 0.056, 0.064)),
        origin=Origin(xyz=(-0.093, 0.0, 0.0)),
        material=gunmetal,
        name="yoke_rear_bridge",
    )
    side_yoke.visual(
        Box((0.060, 0.040, 0.080)),
        origin=Origin(xyz=(-0.110, -0.0375, 0.0)),
        material=desert_tan,
        name="yoke_mount_block",
    )
    side_yoke.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.15)),
        mass=12.0,
        origin=Origin(xyz=(-0.055, -0.015, 0.0)),
    )

    optic_head = model.part("optic_head")
    optic_head.visual(
        Cylinder(radius=0.0085, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="pivot_axle",
    )
    optic_head.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, -0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="inner_retainer",
    )
    optic_head.visual(
        Cylinder(radius=0.015, length=0.010),
        origin=Origin(xyz=(0.0, 0.033, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="outer_retainer",
    )
    optic_head.visual(
        Box((0.105, 0.036, 0.075)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=optics_black,
        name="main_housing",
    )
    optic_head.visual(
        Box((0.032, 0.032, 0.032)),
        origin=Origin(xyz=(0.078, 0.0, 0.0)),
        material=dark_gray,
        name="lens_hood",
    )
    optic_head.visual(
        Cylinder(radius=0.013, length=0.018),
        origin=Origin(xyz=(0.092, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    optic_head.visual(
        Box((0.055, 0.034, 0.028)),
        origin=Origin(xyz=(-0.004, 0.0, -0.045)),
        material=dark_gray,
        name="lower_electronics_box",
    )
    optic_head.inertial = Inertial.from_geometry(
        Box((0.12, 0.06, 0.11)),
        mass=7.0,
        origin=Origin(xyz=(0.02, 0.0, -0.005)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=azimuth_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8500.0, velocity=1.2),
    )
    model.articulation(
        "elevation_rotation",
        ArticulationType.REVOLUTE,
        parent=azimuth_stage,
        child=cradle,
        origin=Origin(xyz=(0.02, 0.0, 0.19)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4200.0,
            velocity=1.1,
            lower=-0.20,
            upper=0.85,
        ),
    )
    model.articulation(
        "ammo_box_mount",
        ArticulationType.FIXED,
        parent=cradle,
        child=ammo_box,
        origin=Origin(xyz=(0.46, -0.205, -0.01)),
    )
    model.articulation(
        "side_yoke_mount",
        ArticulationType.FIXED,
        parent=cradle,
        child=side_yoke,
        origin=Origin(xyz=(0.24, 0.160, 0.08)),
    )
    model.articulation(
        "optic_tilt",
        ArticulationType.REVOLUTE,
        parent=side_yoke,
        child=optic_head,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.6,
            lower=-0.45,
            upper=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    azimuth_stage = object_model.get_part("azimuth_stage")
    cradle = object_model.get_part("cradle")
    ammo_box = object_model.get_part("ammo_box")
    side_yoke = object_model.get_part("side_yoke")
    optic_head = object_model.get_part("optic_head")

    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    elevation_rotation = object_model.get_articulation("elevation_rotation")
    optic_tilt = object_model.get_articulation("optic_tilt")

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
        "azimuth_axis_vertical",
        azimuth_rotation.axis == (0.0, 0.0, 1.0),
        f"expected vertical azimuth axis, got {azimuth_rotation.axis}",
    )
    ctx.check(
        "elevation_axis_horizontal",
        elevation_rotation.axis == (0.0, 1.0, 0.0),
        f"expected horizontal elevation axis, got {elevation_rotation.axis}",
    )
    ctx.check(
        "optic_tilt_axis_horizontal",
        optic_tilt.axis == (0.0, 1.0, 0.0),
        f"expected optic tilt axis to run through the side yoke, got {optic_tilt.axis}",
    )

    ctx.expect_contact(
        azimuth_stage,
        pedestal,
        elem_a="slew_ring",
        elem_b="pedestal_cap",
        name="slew_ring_seats_on_pedestal_cap",
    )
    ctx.expect_contact(
        cradle,
        azimuth_stage,
        elem_a="trunnion_shaft",
        elem_b="left_trunnion_support",
        name="left_trunnion_support_contacts_shaft",
    )
    ctx.expect_contact(
        cradle,
        azimuth_stage,
        elem_a="trunnion_shaft",
        elem_b="right_trunnion_support",
        name="right_trunnion_support_contacts_shaft",
    )
    ctx.expect_contact(
        ammo_box,
        cradle,
        elem_a="ammo_mount_bracket",
        elem_b="left_armor_plate",
        name="ammo_box_bolted_to_left_armor_plate",
    )
    ctx.expect_contact(
        side_yoke,
        cradle,
        elem_a="yoke_mount_block",
        elem_b="right_armor_plate",
        name="side_yoke_mounts_to_right_armor_plate",
    )

    with ctx.pose({elevation_rotation: 0.35}):
        ctx.expect_gap(
            cradle,
            azimuth_stage,
            axis="x",
            max_penetration=0.0,
            positive_elem="weapon_receiver",
            negative_elem="rear_drive_housing",
            name="receiver_clears_rear_drive_housing_when_elevated",
        )

    for pose_name, tilt_value in (("depressed", -0.35), ("raised", 0.55)):
        with ctx.pose({optic_tilt: tilt_value}):
            ctx.expect_contact(
                optic_head,
                side_yoke,
                elem_a="inner_retainer",
                elem_b="inner_yoke_cheek",
                name=f"optic_head_inner_retainer_stays_captured_in_{pose_name}_tilt_pose",
            )
            ctx.expect_contact(
                optic_head,
                side_yoke,
                elem_a="outer_retainer",
                elem_b="outer_yoke_cheek",
                name=f"optic_head_outer_retainer_stays_captured_in_{pose_name}_tilt_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
