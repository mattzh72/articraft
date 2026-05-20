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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    width: float,
    depth: float,
    z: float,
    *,
    radius: float,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=6)
    ]


def _yz_section(
    width_y: float,
    height_z: float,
    x: float,
    *,
    radius: float,
    z_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_shift, z + z_shift)
        for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=6)
    ]


def _lofted_xy_mesh(name: str, sections: list[tuple[float, float, float, float, float]]) -> object:
    loops = [
        _xy_section(width, depth, z, radius=radius, x_shift=x_shift)
        for z, width, depth, radius, x_shift in sections
    ]
    return mesh_from_geometry(section_loft(loops), name)


def _lofted_foot_mesh(name: str, sections: list[tuple[float, float, float, float, float]]) -> object:
    loops = [
        _yz_section(width_y, height_z, x, radius=radius, z_shift=z_shift)
        for x, width_y, height_z, radius, z_shift in sections
    ]
    return mesh_from_geometry(section_loft(loops), name)


def _add_service_panel(
    part,
    *,
    panel_prefix: str,
    side_sign: float,
    x_center: float,
    z_center: float,
    panel_width: float,
    panel_height: float,
    shell_half_depth: float,
    shell_material,
    seal_material,
    hardware_material,
) -> None:
    gasket_y = side_sign * (shell_half_depth + 0.002)
    panel_y = side_sign * (shell_half_depth + 0.007)
    drip_y = side_sign * (shell_half_depth + 0.009)
    bolt_y = side_sign * (shell_half_depth + 0.013)

    part.visual(
        Box((panel_width + 0.014, 0.004, panel_height + 0.014)),
        origin=Origin(xyz=(x_center, gasket_y, z_center)),
        material=seal_material,
        name=f"{panel_prefix}_gasket_{'left' if side_sign > 0.0 else 'right'}",
    )
    part.visual(
        Box((panel_width, 0.010, panel_height)),
        origin=Origin(xyz=(x_center, panel_y, z_center)),
        material=shell_material,
        name=f"{panel_prefix}_panel_{'left' if side_sign > 0.0 else 'right'}",
    )
    part.visual(
        Box((panel_width + 0.030, 0.014, 0.016)),
        origin=Origin(xyz=(x_center - 0.004, drip_y, z_center + panel_height * 0.52)),
        material=shell_material,
        name=f"{panel_prefix}_drip_edge_{'left' if side_sign > 0.0 else 'right'}",
    )
    for bolt_index, bolt_x in enumerate((x_center - panel_width * 0.24, x_center + panel_width * 0.24)):
        part.visual(
            Cylinder(radius=0.008, length=0.010),
            origin=Origin(
                xyz=(bolt_x, bolt_y, z_center + panel_height * (0.22 if bolt_index == 0 else -0.22)),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_material,
            name=f"{panel_prefix}_bolt_{'left' if side_sign > 0.0 else 'right'}_{bolt_index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_robotic_leg")

    shell_green = model.material("shell_green", rgba=(0.31, 0.39, 0.35, 1.0))
    shell_graphite = model.material("shell_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    seal_black = model.material("seal_black", rgba=(0.07, 0.08, 0.08, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.75, 0.78, 1.0))
    tread_rubber = model.material("tread_rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    hip_mount = model.part("hip_mount")
    hip_mount.inertial = Inertial.from_geometry(
        Box((0.30, 0.22, 0.26)),
        mass=16.0,
        origin=Origin(xyz=(0.03, 0.0, 0.13)),
    )
    hip_mount.visual(
        _lofted_xy_mesh(
            "hip_mount_shell",
            [
                (0.080, 0.195, 0.126, 0.026, 0.012),
                (0.150, 0.238, 0.160, 0.032, 0.022),
                (0.220, 0.265, 0.184, 0.036, 0.030),
            ],
        ),
        material=shell_green,
        name="hip_mount_shell",
    )
    hip_mount.visual(
        Box((0.286, 0.206, 0.018)),
        origin=Origin(xyz=(0.03, 0.0, 0.229)),
        material=shell_graphite,
        name="mount_flange",
    )
    hip_mount.visual(
        Box((0.154, 0.118, 0.050)),
        origin=Origin(xyz=(-0.030, 0.0, 0.150)),
        material=shell_graphite,
        name="electronics_bay_cover",
    )
    hip_mount.visual(
        Box((0.084, 0.030, 0.160)),
        origin=Origin(xyz=(0.0, 0.075, 0.000)),
        material=stainless,
        name="hip_lug_left",
    )
    hip_mount.visual(
        Box((0.084, 0.030, 0.160)),
        origin=Origin(xyz=(0.0, -0.075, 0.000)),
        material=stainless,
        name="hip_lug_right",
    )
    hip_mount.visual(
        Box((0.118, 0.124, 0.040)),
        origin=Origin(xyz=(0.036, 0.0, 0.102)),
        material=shell_graphite,
        name="hip_rain_hood",
    )

    upper_leg = model.part("upper_leg")
    upper_leg.inertial = Inertial.from_geometry(
        Box((0.22, 0.15, 0.62)),
        mass=24.0,
        origin=Origin(xyz=(0.02, 0.0, -0.29)),
    )
    upper_leg.visual(
        Cylinder(radius=0.055, length=0.120),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="hip_hub",
    )
    upper_leg.visual(
        Cylinder(radius=0.046, length=0.122),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=seal_black,
        name="hip_seal",
    )
    upper_leg.visual(
        Box((0.154, 0.104, 0.094)),
        origin=Origin(xyz=(0.016, 0.0, -0.072)),
        material=shell_graphite,
        name="upper_shoulder_block",
    )
    upper_leg.visual(
        _lofted_xy_mesh(
            "upper_leg_shell",
            [
                (-0.102, 0.176, 0.118, 0.026, 0.018),
                (-0.210, 0.155, 0.104, 0.022, 0.026),
                (-0.370, 0.138, 0.096, 0.020, 0.030),
                (-0.500, 0.176, 0.132, 0.026, 0.018),
            ],
        ),
        material=shell_green,
        name="upper_leg_shell",
    )
    upper_leg.visual(
        Box((0.118, 0.084, 0.048)),
        origin=Origin(xyz=(0.050, 0.0, -0.492)),
        material=shell_graphite,
        name="knee_weather_shroud",
    )
    upper_leg.visual(
        Box((0.072, 0.030, 0.144)),
        origin=Origin(xyz=(0.010, 0.072, -0.570)),
        material=stainless,
        name="knee_lug_left",
    )
    upper_leg.visual(
        Box((0.072, 0.030, 0.144)),
        origin=Origin(xyz=(0.010, -0.072, -0.570)),
        material=stainless,
        name="knee_lug_right",
    )
    _add_service_panel(
        upper_leg,
        panel_prefix="upper_bay",
        side_sign=1.0,
        x_center=0.030,
        z_center=-0.255,
        panel_width=0.128,
        panel_height=0.214,
        shell_half_depth=0.050,
        shell_material=shell_graphite,
        seal_material=seal_black,
        hardware_material=stainless,
    )
    _add_service_panel(
        upper_leg,
        panel_prefix="upper_bay",
        side_sign=-1.0,
        x_center=0.030,
        z_center=-0.255,
        panel_width=0.128,
        panel_height=0.214,
        shell_half_depth=0.050,
        shell_material=shell_graphite,
        seal_material=seal_black,
        hardware_material=stainless,
    )

    lower_leg = model.part("lower_leg")
    lower_leg.inertial = Inertial.from_geometry(
        Box((0.20, 0.14, 0.60)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.28)),
    )
    lower_leg.visual(
        Cylinder(radius=0.050, length=0.114),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knee_hub",
    )
    lower_leg.visual(
        Cylinder(radius=0.041, length=0.116),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=seal_black,
        name="knee_seal",
    )
    lower_leg.visual(
        Box((0.134, 0.098, 0.090)),
        origin=Origin(xyz=(-0.006, 0.0, -0.070)),
        material=shell_graphite,
        name="lower_shoulder_block",
    )
    lower_leg.visual(
        _lofted_xy_mesh(
            "lower_leg_shell",
            [
                (-0.084, 0.162, 0.106, 0.024, -0.008),
                (-0.198, 0.138, 0.096, 0.020, 0.000),
                (-0.360, 0.126, 0.090, 0.018, 0.010),
                (-0.488, 0.154, 0.124, 0.024, 0.020),
            ],
        ),
        material=shell_green,
        name="lower_leg_shell",
    )
    lower_leg.visual(
        Box((0.104, 0.074, 0.060)),
        origin=Origin(xyz=(0.040, 0.0, -0.488)),
        material=shell_graphite,
        name="ankle_weather_shroud",
    )
    lower_leg.visual(
        Box((0.088, 0.016, 0.126)),
        origin=Origin(xyz=(0.000, 0.067, -0.515)),
        material=stainless,
        name="ankle_fork_left",
    )
    lower_leg.visual(
        Box((0.088, 0.016, 0.126)),
        origin=Origin(xyz=(0.000, -0.067, -0.515)),
        material=stainless,
        name="ankle_fork_right",
    )
    lower_leg.visual(
        Box((0.062, 0.022, 0.112)),
        origin=Origin(xyz=(0.000, 0.082, -0.565)),
        material=stainless,
        name="ankle_lug_left",
    )
    lower_leg.visual(
        Box((0.062, 0.022, 0.112)),
        origin=Origin(xyz=(0.000, -0.082, -0.565)),
        material=stainless,
        name="ankle_lug_right",
    )
    _add_service_panel(
        lower_leg,
        panel_prefix="lower_bay",
        side_sign=1.0,
        x_center=0.018,
        z_center=-0.248,
        panel_width=0.112,
        panel_height=0.200,
        shell_half_depth=0.046,
        shell_material=shell_graphite,
        seal_material=seal_black,
        hardware_material=stainless,
    )
    _add_service_panel(
        lower_leg,
        panel_prefix="lower_bay",
        side_sign=-1.0,
        x_center=0.018,
        z_center=-0.248,
        panel_width=0.112,
        panel_height=0.200,
        shell_half_depth=0.046,
        shell_material=shell_graphite,
        seal_material=seal_black,
        hardware_material=stainless,
    )

    foot = model.part("foot")
    foot.inertial = Inertial.from_geometry(
        Box((0.36, 0.16, 0.15)),
        mass=8.0,
        origin=Origin(xyz=(0.09, 0.0, -0.07)),
    )
    foot.visual(
        Cylinder(radius=0.042, length=0.100),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="ankle_hub",
    )
    foot.visual(
        Cylinder(radius=0.034, length=0.102),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=seal_black,
        name="ankle_seal",
    )
    foot.visual(
        Box((0.120, 0.092, 0.084)),
        origin=Origin(xyz=(-0.012, 0.0, -0.052)),
        material=shell_graphite,
        name="heel_block",
    )
    foot.visual(
        _lofted_foot_mesh(
            "foot_shell",
            [
                (-0.062, 0.098, 0.050, 0.016, -0.082),
                (0.022, 0.118, 0.072, 0.020, -0.094),
                (0.148, 0.144, 0.082, 0.022, -0.090),
                (0.274, 0.084, 0.036, 0.012, -0.092),
            ],
        ),
        material=shell_green,
        name="foot_shell",
    )
    foot.visual(
        Box((0.292, 0.142, 0.016)),
        origin=Origin(xyz=(0.090, 0.0, -0.118)),
        material=tread_rubber,
        name="sole_pad",
    )
    foot.visual(
        Box((0.058, 0.086, 0.018)),
        origin=Origin(xyz=(0.252, 0.0, -0.112)),
        material=tread_rubber,
        name="toe_pad",
    )
    foot.visual(
        Box((0.050, 0.082, 0.018)),
        origin=Origin(xyz=(-0.044, 0.0, -0.108)),
        material=tread_rubber,
        name="heel_pad",
    )
    foot.visual(
        Box((0.038, 0.080, 0.028)),
        origin=Origin(xyz=(0.276, 0.0, -0.090)),
        material=shell_graphite,
        name="toe_bumper",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=upper_leg,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=1.8,
            lower=-0.60,
            upper=1.00,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg,
        child=lower_leg,
        origin=Origin(xyz=(0.0, 0.0, -0.580)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=520.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_leg,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.560)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.4,
            lower=-0.55,
            upper=0.50,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_mount = object_model.get_part("hip_mount")
    upper_leg = object_model.get_part("upper_leg")
    lower_leg = object_model.get_part("lower_leg")
    foot = object_model.get_part("foot")
    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

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

    ctx.expect_contact(hip_mount, upper_leg, contact_tol=0.0005, name="hip_joint_has_bearing_contact")
    ctx.expect_contact(upper_leg, lower_leg, contact_tol=0.0005, name="knee_joint_has_bearing_contact")
    ctx.expect_contact(lower_leg, foot, contact_tol=0.0005, name="ankle_joint_has_bearing_contact")
    ctx.expect_origin_gap(hip_mount, foot, axis="z", min_gap=1.00, max_gap=1.20, name="neutral_leg_drop_is_full_scale")

    with ctx.pose({hip_pitch: 0.95, knee_pitch: 0.0, ankle_pitch: 0.0}):
        foot_forward = ctx.part_world_position(foot)
    ctx.check(
        "hip_pitch_swings_leg_forward",
        foot_forward is not None and foot_forward[0] > 0.75,
        f"expected foot origin to move forward at positive hip flexion, got {foot_forward}",
    )

    with ctx.pose({hip_pitch: 0.0, knee_pitch: 1.35, ankle_pitch: 0.0}):
        foot_folded = ctx.part_world_position(foot)
    ctx.check(
        "knee_pitch_folds_shank_back",
        foot_folded is not None and foot_folded[0] < -0.30,
        f"expected foot origin to move behind the knee during flexion, got {foot_folded}",
    )

    neutral_toe = ctx.part_element_world_aabb(foot, elem="toe_pad")
    with ctx.pose({ankle_pitch: 0.40}):
        dorsiflexed_toe = ctx.part_element_world_aabb(foot, elem="toe_pad")
    ctx.check(
        "ankle_dorsiflexion_lifts_toe",
        neutral_toe is not None
        and dorsiflexed_toe is not None
        and dorsiflexed_toe[1][2] > neutral_toe[1][2] + 0.02,
        f"expected toe pad to lift in positive ankle pitch, neutral={neutral_toe}, dorsiflexed={dorsiflexed_toe}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
