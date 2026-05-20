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


def _xy_section(
    width: float,
    depth: float,
    z: float,
    radius: float,
    *,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_offset, y + y_offset, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=8)
    ]


def _yz_section(
    width_y: float,
    height_z: float,
    x: float,
    z_center: float,
    radius: float,
    *,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_offset, z_center + z)
        for y, z in rounded_rect_profile(width_y, height_z, radius, corner_segments=8)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_leg", assets=ASSETS)

    satin_titanium = model.material("satin_titanium", rgba=(0.71, 0.73, 0.76, 1.0))
    matte_graphite = model.material("matte_graphite", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.30, 0.32, 0.35, 1.0))
    soft_black = model.material("soft_black", rgba=(0.08, 0.09, 0.10, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.83, 0.85, 0.88, 1.0))

    hip_module = model.part("hip_module")
    hip_shell = _save_mesh(
        "hip_module_shell.obj",
        section_loft(
            [
                _xy_section(0.098, 0.086, 0.040, 0.016),
                _xy_section(0.112, 0.090, 0.084, 0.020),
                _xy_section(0.122, 0.096, 0.124, 0.022),
            ]
        ),
    )
    hip_module.visual(hip_shell, material=satin_titanium, name="hip_shell")
    hip_module.visual(
        Box((0.160, 0.110, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.132)),
        material=matte_graphite,
        name="mount_plate",
    )
    hip_module.visual(
        Box((0.052, 0.020, 0.124)),
        origin=Origin(xyz=(0.0, 0.055, 0.062)),
        material=dark_anodized,
        name="left_hip_cheek",
    )
    hip_module.visual(
        Box((0.052, 0.020, 0.124)),
        origin=Origin(xyz=(0.0, -0.055, 0.062)),
        material=dark_anodized,
        name="right_hip_cheek",
    )
    hip_module.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, 0.067, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_hip_axis_cap",
    )
    hip_module.visual(
        Cylinder(radius=0.024, length=0.004),
        origin=Origin(xyz=(0.0, -0.067, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_hip_axis_cap",
    )
    hip_module.visual(
        Box((0.004, 0.050, 0.060)),
        origin=Origin(xyz=(0.061, 0.0, 0.085)),
        material=matte_graphite,
        name="outer_service_panel",
    )
    hip_module.visual(
        Box((0.004, 0.044, 0.054)),
        origin=Origin(xyz=(-0.061, 0.0, 0.082)),
        material=soft_black,
        name="inner_service_panel",
    )
    for index, z_pos in enumerate((0.062, 0.106), start=1):
        hip_module.visual(
            Cylinder(radius=0.0035, length=0.003),
            origin=Origin(xyz=(0.0625, 0.016, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_steel,
            name=f"outer_fastener_{index}",
        )
        hip_module.visual(
            Cylinder(radius=0.0035, length=0.003),
            origin=Origin(xyz=(-0.0625, -0.014, z_pos), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=axle_steel,
            name=f"inner_fastener_{index}",
        )
    hip_module.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.150)),
        mass=6.4,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    upper_leg = model.part("upper_leg")
    upper_shell = _save_mesh(
        "upper_leg_shell.obj",
        section_loft(
            [
                _xy_section(0.094, 0.056, -0.053, 0.014, x_offset=0.010),
                _xy_section(0.084, 0.050, -0.170, 0.012, x_offset=0.008),
                _xy_section(0.104, 0.064, -0.284, 0.016, x_offset=0.002),
            ]
        ),
    )
    upper_leg.visual(
        Cylinder(radius=0.031, length=0.090),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="hip_barrel",
    )
    upper_leg.visual(
        Box((0.070, 0.052, 0.022)),
        origin=Origin(xyz=(0.014, 0.0, -0.042)),
        material=dark_anodized,
        name="upper_bridge",
    )
    upper_leg.visual(upper_shell, material=satin_titanium, name="upper_shell")
    upper_leg.visual(
        Box((0.020, 0.026, 0.230)),
        origin=Origin(xyz=(-0.020, 0.0, -0.169)),
        material=matte_graphite,
        name="rear_spine",
    )
    upper_leg.visual(
        Box((0.050, 0.006, 0.122)),
        origin=Origin(xyz=(0.017, 0.029, -0.165)),
        material=matte_graphite,
        name="outer_actuator_bay",
    )
    upper_leg.visual(
        Box((0.046, 0.006, 0.114)),
        origin=Origin(xyz=(0.013, -0.028, -0.162)),
        material=soft_black,
        name="inner_actuator_bay",
    )
    upper_leg.visual(
        Box((0.076, 0.090, 0.024)),
        origin=Origin(xyz=(0.000, 0.0, -0.296)),
        material=dark_anodized,
        name="knee_bridge",
    )
    upper_leg.visual(
        Box((0.052, 0.020, 0.070)),
        origin=Origin(xyz=(0.000, 0.055, -0.340)),
        material=dark_anodized,
        name="left_knee_cheek",
    )
    upper_leg.visual(
        Box((0.052, 0.020, 0.070)),
        origin=Origin(xyz=(0.000, -0.055, -0.340)),
        material=dark_anodized,
        name="right_knee_cheek",
    )
    upper_leg.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, 0.067, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_knee_axis_cap",
    )
    upper_leg.visual(
        Cylinder(radius=0.022, length=0.004),
        origin=Origin(xyz=(0.0, -0.067, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_knee_axis_cap",
    )
    upper_leg.visual(
        Box((0.032, 0.010, 0.074)),
        origin=Origin(xyz=(0.020, 0.029, -0.226)),
        material=matte_graphite,
        name="mid_seam_break",
    )
    upper_leg.inertial = Inertial.from_geometry(
        Box((0.120, 0.120, 0.360)),
        mass=4.3,
        origin=Origin(xyz=(0.0, 0.0, -0.170)),
    )

    lower_leg = model.part("lower_leg")
    lower_shell = _save_mesh(
        "lower_leg_shell.obj",
        section_loft(
            [
                _xy_section(0.060, 0.044, -0.014, 0.010, x_offset=-0.018),
                _xy_section(0.072, 0.048, -0.156, 0.010, x_offset=-0.014),
                _xy_section(0.088, 0.058, -0.252, 0.014, x_offset=-0.010),
            ]
        ),
    )
    lower_leg.visual(
        Cylinder(radius=0.029, length=0.090),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="knee_barrel",
    )
    lower_leg.visual(lower_shell, material=satin_titanium, name="lower_shell")
    lower_leg.visual(
        Box((0.042, 0.018, 0.182)),
        origin=Origin(xyz=(0.004, 0.023, -0.166)),
        material=matte_graphite,
        name="outer_compact_bay",
    )
    lower_leg.visual(
        Box((0.038, 0.018, 0.176)),
        origin=Origin(xyz=(0.002, -0.023, -0.166)),
        material=soft_black,
        name="inner_compact_bay",
    )
    lower_leg.visual(
        Box((0.022, 0.028, 0.232)),
        origin=Origin(xyz=(-0.024, 0.0, -0.158)),
        material=matte_graphite,
        name="rear_link_web",
    )
    lower_leg.visual(
        Box((0.056, 0.102, 0.028)),
        origin=Origin(xyz=(-0.004, 0.0, -0.248)),
        material=dark_anodized,
        name="ankle_bridge",
    )
    lower_leg.visual(
        Box((0.040, 0.022, 0.100)),
        origin=Origin(xyz=(0.000, 0.051, -0.294)),
        material=dark_anodized,
        name="left_ankle_cheek",
    )
    lower_leg.visual(
        Box((0.040, 0.022, 0.100)),
        origin=Origin(xyz=(0.000, -0.051, -0.294)),
        material=dark_anodized,
        name="right_ankle_cheek",
    )
    lower_leg.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, 0.064, -0.310), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="left_ankle_axis_cap",
    )
    lower_leg.visual(
        Cylinder(radius=0.020, length=0.004),
        origin=Origin(xyz=(0.0, -0.064, -0.310), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=axle_steel,
        name="right_ankle_axis_cap",
    )
    lower_leg.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.330)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
    )

    foot = model.part("foot")
    foot_shell = _save_mesh(
        "foot_shell.obj",
        section_loft(
            [
                _yz_section(0.080, 0.046, -0.065, -0.060, 0.010),
                _yz_section(0.092, 0.034, 0.030, -0.076, 0.009),
                _yz_section(0.104, 0.020, 0.190, -0.082, 0.006),
            ]
        ),
    )
    foot.visual(
        Cylinder(radius=0.027, length=0.080),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_anodized,
        name="ankle_barrel",
    )
    foot.visual(
        Box((0.068, 0.062, 0.040)),
        origin=Origin(xyz=(-0.006, 0.0, -0.047)),
        material=dark_anodized,
        name="ankle_block",
    )
    foot.visual(foot_shell, material=satin_titanium, name="foot_shell")
    foot.visual(
        Box((0.120, 0.078, 0.008)),
        origin=Origin(xyz=(0.112, 0.0, -0.069)),
        material=matte_graphite,
        name="toe_interface_plate",
    )
    foot.visual(
        Box((0.036, 0.090, 0.018)),
        origin=Origin(xyz=(0.206, 0.0, -0.069)),
        material=dark_anodized,
        name="toe_bumper",
    )
    foot.visual(
        Box((0.224, 0.090, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, -0.094)),
        material=soft_black,
        name="sole_pad",
    )
    foot.visual(
        Box((0.056, 0.078, 0.018)),
        origin=Origin(xyz=(-0.060, 0.0, -0.086)),
        material=soft_black,
        name="heel_pad",
    )
    foot.visual(
        Box((0.038, 0.040, 0.020)),
        origin=Origin(xyz=(0.022, 0.0, -0.038)),
        material=matte_graphite,
        name="front_ankle_cover",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.280, 0.100, 0.110)),
        mass=2.2,
        origin=Origin(xyz=(0.060, 0.0, -0.060)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_module,
        child=upper_leg,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=2.4,
            lower=math.radians(-55.0),
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg,
        child=lower_leg,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.8,
            lower=0.0,
            upper=math.radians(90.0),
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_leg,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.310)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.6,
            lower=math.radians(-35.0),
            upper=math.radians(40.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hip_module = object_model.get_part("hip_module")
    upper_leg = object_model.get_part("upper_leg")
    lower_leg = object_model.get_part("lower_leg")
    foot = object_model.get_part("foot")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    hip_shell = hip_module.get_visual("hip_shell")
    left_hip_cheek = hip_module.get_visual("left_hip_cheek")
    right_hip_cheek = hip_module.get_visual("right_hip_cheek")

    hip_barrel = upper_leg.get_visual("hip_barrel")
    knee_bridge = upper_leg.get_visual("knee_bridge")
    left_knee_cheek = upper_leg.get_visual("left_knee_cheek")
    right_knee_cheek = upper_leg.get_visual("right_knee_cheek")

    knee_barrel = lower_leg.get_visual("knee_barrel")
    ankle_bridge = lower_leg.get_visual("ankle_bridge")
    left_ankle_cheek = lower_leg.get_visual("left_ankle_cheek")
    right_ankle_cheek = lower_leg.get_visual("right_ankle_cheek")

    ankle_barrel = foot.get_visual("ankle_barrel")
    sole_pad = foot.get_visual("sole_pad")

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

    ctx.fail_if_articulation_overlaps(max_pose_samples=18)

    ctx.expect_origin_distance(lower_leg, upper_leg, axes="z", min_dist=0.32, max_dist=0.36)
    ctx.expect_origin_distance(foot, lower_leg, axes="z", min_dist=0.29, max_dist=0.33)
    ctx.expect_origin_distance(foot, hip_module, axes="z", min_dist=0.62, max_dist=0.67)
    ctx.expect_origin_distance(foot, hip_module, axes="x", max_dist=0.03)

    ctx.expect_contact(upper_leg, hip_module, elem_a=hip_barrel, elem_b=left_hip_cheek)
    ctx.expect_contact(upper_leg, hip_module, elem_a=hip_barrel, elem_b=right_hip_cheek)
    ctx.expect_contact(lower_leg, upper_leg, elem_a=knee_barrel, elem_b=left_knee_cheek)
    ctx.expect_contact(lower_leg, upper_leg, elem_a=knee_barrel, elem_b=right_knee_cheek)
    ctx.expect_contact(foot, lower_leg, elem_a=ankle_barrel, elem_b=left_ankle_cheek)
    ctx.expect_contact(foot, lower_leg, elem_a=ankle_barrel, elem_b=right_ankle_cheek)

    ctx.expect_gap(
        hip_module,
        upper_leg,
        axis="z",
        positive_elem=hip_shell,
        negative_elem=hip_barrel,
        min_gap=0.006,
        max_gap=0.020,
        name="hip_shell_clears_barrel",
    )
    ctx.expect_gap(
        upper_leg,
        lower_leg,
        axis="z",
        positive_elem=knee_bridge,
        negative_elem=knee_barrel,
        min_gap=0.002,
        max_gap=0.012,
        name="knee_bridge_clears_barrel",
    )
    ctx.expect_gap(
        lower_leg,
        foot,
        axis="z",
        positive_elem=ankle_bridge,
        negative_elem=ankle_barrel,
        min_gap=0.0,
        max_gap=0.025,
        name="ankle_bridge_clears_barrel",
    )
    ctx.expect_contact(
        foot,
        lower_leg,
        elem_a=ankle_barrel,
        elem_b=left_ankle_cheek,
        name="left_ankle_cheek_seats_against_barrel",
    )
    ctx.expect_contact(
        foot,
        lower_leg,
        elem_a=ankle_barrel,
        elem_b=right_ankle_cheek,
        name="right_ankle_cheek_seats_against_barrel",
    )
    ctx.expect_gap(
        lower_leg,
        foot,
        axis="z",
        positive_elem=lower_leg.get_visual("ankle_bridge"),
        negative_elem=sole_pad,
        min_gap=0.11,
        name="sole_sits_well_below_ankle_bridge",
    )

    rest_foot_position = ctx.part_world_position(foot)
    rest_sole_aabb = ctx.part_element_world_aabb(foot, elem=sole_pad)
    assert rest_foot_position is not None
    assert rest_sole_aabb is not None

    with ctx.pose({hip_pitch: math.radians(26.0), knee_pitch: math.radians(68.0), ankle_pitch: math.radians(14.0)}):
        flexed_foot_position = ctx.part_world_position(foot)
        flexed_sole_aabb = ctx.part_element_world_aabb(foot, elem=sole_pad)
        assert flexed_foot_position is not None
        assert flexed_sole_aabb is not None
        ctx.check(
            "flexed_pose_moves_foot_forward",
            flexed_foot_position[0] > rest_foot_position[0] + 0.16,
            details=f"rest_x={rest_foot_position[0]:.4f}, flexed_x={flexed_foot_position[0]:.4f}",
        )
        ctx.check(
            "flexed_pose_lifts_sole",
            flexed_sole_aabb[1][2] > rest_sole_aabb[1][2] + 0.18,
            details=f"rest_sole_top={rest_sole_aabb[1][2]:.4f}, flexed_sole_top={flexed_sole_aabb[1][2]:.4f}",
        )
        ctx.expect_contact(upper_leg, hip_module, elem_a=hip_barrel, elem_b=left_hip_cheek)
        ctx.expect_contact(lower_leg, upper_leg, elem_a=knee_barrel, elem_b=left_knee_cheek)
        ctx.expect_contact(foot, lower_leg, elem_a=ankle_barrel, elem_b=left_ankle_cheek)

    with ctx.pose({hip_pitch: math.radians(-18.0), knee_pitch: math.radians(24.0), ankle_pitch: math.radians(-18.0)}):
        ctx.expect_contact(upper_leg, hip_module, elem_a=hip_barrel, elem_b=right_hip_cheek)
        ctx.expect_contact(lower_leg, upper_leg, elem_a=knee_barrel, elem_b=right_knee_cheek)
        ctx.expect_contact(foot, lower_leg, elem_a=ankle_barrel, elem_b=right_ankle_cheek)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
