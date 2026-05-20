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
    repair_loft,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xy_section(
    z_pos: float,
    size_x: float,
    size_y: float,
    radius: float,
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + dx, y + dy, z_pos)
        for x, y in rounded_rect_profile(size_x, size_y, radius, corner_segments=8)
    ]


def _yz_section(
    x_pos: float,
    size_y: float,
    size_z: float,
    radius: float,
    *,
    dy: float = 0.0,
    dz: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y + dy, z + dz)
        for y, z in rounded_rect_profile(size_y, size_z, radius, corner_segments=8)
    ]


def _joint_y_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def _joint_x_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _add_x_bolts(
    part,
    *,
    prefix: str,
    x_pos: float,
    y_positions: tuple[float, ...],
    z_positions: tuple[float, ...],
    material,
    radius: float = 0.0045,
    length: float = 0.006,
) -> None:
    for iy, y_pos in enumerate(y_positions):
        for iz, z_pos in enumerate(z_positions):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=_joint_x_origin((x_pos, y_pos, z_pos)),
                material=material,
                name=f"{prefix}_{iy}_{iz}",
            )


def _build_object_geometry():
    hip_shell = repair_loft(
        section_loft(
            [
                _xy_section(0.000, 0.118, 0.094, 0.016),
                _xy_section(0.070, 0.166, 0.128, 0.022),
                _xy_section(0.146, 0.142, 0.108, 0.020),
            ]
        )
    )
    thigh_shell = repair_loft(
        section_loft(
            [
                _xy_section(-0.030, 0.090, 0.060, 0.012, dx=0.000),
                _xy_section(-0.160, 0.096, 0.062, 0.014, dx=0.005),
                _xy_section(-0.286, 0.076, 0.054, 0.012, dx=0.010),
            ]
        )
    )
    shank_shell = repair_loft(
        section_loft(
            [
                _xy_section(-0.028, 0.082, 0.056, 0.012, dx=0.000),
                _xy_section(-0.145, 0.080, 0.055, 0.013, dx=0.006),
                _xy_section(-0.255, 0.066, 0.048, 0.011, dx=0.012),
            ]
        )
    )
    foot_shell = repair_loft(
        section_loft(
            [
                _yz_section(0.050, 0.082, 0.050, 0.012, dz=-0.046),
                _yz_section(0.108, 0.112, 0.060, 0.016, dz=-0.050),
                _yz_section(0.176, 0.098, 0.038, 0.012, dz=-0.041),
                _yz_section(0.214, 0.074, 0.022, 0.008, dz=-0.031),
            ]
        )
    )
    toe_guard = tube_from_spline_points(
        [
            (0.050, 0.020, -0.050),
            (0.086, 0.038, -0.032),
            (0.124, 0.038, -0.026),
            (0.164, 0.000, -0.030),
            (0.124, -0.038, -0.026),
            (0.086, -0.038, -0.032),
            (0.050, -0.020, -0.050),
        ],
        radius=0.006,
        samples_per_segment=14,
        radial_segments=16,
        cap_ends=True,
    )
    return (
        _save_mesh("hip_shell.obj", hip_shell),
        _save_mesh("thigh_shell.obj", thigh_shell),
        _save_mesh("shank_shell.obj", shank_shell),
        _save_mesh("foot_shell.obj", foot_shell),
        _save_mesh("toe_guard.obj", toe_guard),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_utility_leg", assets=ASSETS)

    utility_paint = model.material("utility_paint", rgba=(0.60, 0.58, 0.18, 1.0))
    cover_polymer = model.material("cover_polymer", rgba=(0.17, 0.18, 0.19, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.34, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.76, 0.78, 0.80, 1.0))
    service_orange = model.material("service_orange", rgba=(0.86, 0.46, 0.10, 1.0))
    sole_rubber = model.material("sole_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    hip_shell_mesh, thigh_shell_mesh, shank_shell_mesh, foot_shell_mesh, toe_guard_mesh = (
        _build_object_geometry()
    )

    hip = model.part("hip_module")
    hip.visual(
        hip_shell_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
        material=utility_paint,
        name="hip_shell",
    )
    hip.visual(
        Box((0.060, 0.100, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, 0.181)),
        material=cover_polymer,
        name="mount_plate",
    )
    hip.visual(
        Box((0.020, 0.092, 0.086)),
        origin=Origin(xyz=(0.058, 0.000, 0.050)),
        material=cover_polymer,
        name="front_guard",
    )
    hip.visual(
        Box((0.020, 0.092, 0.086)),
        origin=Origin(xyz=(-0.058, 0.000, 0.050)),
        material=cover_polymer,
        name="rear_guard",
    )
    hip.visual(
        Box((0.056, 0.016, 0.084)),
        origin=Origin(xyz=(0.000, 0.045, -0.020)),
        material=dark_steel,
        name="left_hip_ear",
    )
    hip.visual(
        Box((0.056, 0.016, 0.084)),
        origin=Origin(xyz=(0.000, -0.045, -0.020)),
        material=dark_steel,
        name="right_hip_ear",
    )
    hip.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=_joint_y_origin((0.000, 0.041, 0.000)),
        material=service_orange,
        name="left_hip_cap",
    )
    hip.visual(
        Cylinder(radius=0.029, length=0.010),
        origin=_joint_y_origin((0.000, -0.041, 0.000)),
        material=service_orange,
        name="right_hip_cap",
    )
    hip.visual(
        Box((0.058, 0.014, 0.020)),
        origin=Origin(xyz=(0.029, 0.045, 0.026)),
        material=dark_steel,
        name="left_front_ear_bridge",
    )
    hip.visual(
        Box((0.058, 0.014, 0.020)),
        origin=Origin(xyz=(-0.029, 0.045, 0.026)),
        material=dark_steel,
        name="left_rear_ear_bridge",
    )
    hip.visual(
        Box((0.058, 0.014, 0.020)),
        origin=Origin(xyz=(0.029, -0.045, 0.026)),
        material=dark_steel,
        name="right_front_ear_bridge",
    )
    hip.visual(
        Box((0.058, 0.014, 0.020)),
        origin=Origin(xyz=(-0.029, -0.045, 0.026)),
        material=dark_steel,
        name="right_rear_ear_bridge",
    )
    _add_x_bolts(
        hip,
        prefix="front_guard_bolt",
        x_pos=0.066,
        y_positions=(-0.030, 0.030),
        z_positions=(0.020, 0.078),
        material=bolt_steel,
    )
    _add_x_bolts(
        hip,
        prefix="rear_guard_bolt",
        x_pos=-0.066,
        y_positions=(-0.030, 0.030),
        z_positions=(0.020, 0.078),
        material=bolt_steel,
    )
    hip.inertial = Inertial.from_geometry(
        Box((0.180, 0.140, 0.160)),
        mass=9.5,
        origin=Origin(xyz=(0.000, 0.000, 0.050)),
    )

    thigh = model.part("thigh_link")
    thigh.visual(
        Cylinder(radius=0.023, length=0.068),
        origin=_joint_y_origin((0.000, 0.000, 0.000)),
        material=dark_steel,
        name="hip_barrel",
    )
    thigh.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=_joint_y_origin((0.000, 0.035, 0.000)),
        material=bolt_steel,
        name="left_hip_thrust_washer",
    )
    thigh.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=_joint_y_origin((0.000, -0.035, 0.000)),
        material=bolt_steel,
        name="right_hip_thrust_washer",
    )
    thigh.visual(
        Box((0.046, 0.052, 0.034)),
        origin=Origin(xyz=(0.000, 0.000, -0.016)),
        material=dark_steel,
        name="hip_hub",
    )
    thigh.visual(thigh_shell_mesh, material=utility_paint, name="thigh_shell")
    thigh.visual(
        Box((0.036, 0.048, 0.224)),
        origin=Origin(xyz=(0.014, 0.000, -0.142)),
        material=dark_steel,
        name="front_rib",
    )
    thigh.visual(
        Box((0.022, 0.048, 0.176)),
        origin=Origin(xyz=(-0.026, 0.000, -0.166)),
        material=dark_steel,
        name="rear_rib",
    )
    thigh.visual(
        Box((0.012, 0.050, 0.120)),
        origin=Origin(xyz=(0.050, 0.000, -0.165)),
        material=cover_polymer,
        name="left_bay_cover",
    )
    thigh.visual(
        Box((0.012, 0.050, 0.120)),
        origin=Origin(xyz=(-0.050, 0.000, -0.165)),
        material=cover_polymer,
        name="right_bay_cover",
    )
    thigh.visual(
        Box((0.050, 0.014, 0.090)),
        origin=Origin(xyz=(0.010, 0.031, -0.319)),
        material=dark_steel,
        name="left_knee_ear",
    )
    thigh.visual(
        Box((0.050, 0.014, 0.090)),
        origin=Origin(xyz=(0.010, -0.031, -0.319)),
        material=dark_steel,
        name="right_knee_ear",
    )
    thigh.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=_joint_y_origin((0.010, 0.039, -0.330)),
        material=service_orange,
        name="left_knee_cap",
    )
    thigh.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=_joint_y_origin((0.010, -0.039, -0.330)),
        material=service_orange,
        name="right_knee_cap",
    )
    _add_x_bolts(
        thigh,
        prefix="thigh_cover_left_bolt",
        x_pos=0.055,
        y_positions=(-0.020, 0.020),
        z_positions=(-0.215, -0.115),
        material=bolt_steel,
    )
    _add_x_bolts(
        thigh,
        prefix="thigh_cover_right_bolt",
        x_pos=-0.055,
        y_positions=(-0.020, 0.020),
        z_positions=(-0.215, -0.115),
        material=bolt_steel,
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.120, 0.080, 0.360)),
        mass=8.0,
        origin=Origin(xyz=(0.006, 0.000, -0.170)),
    )

    shank = model.part("shank_link")
    shank.visual(
        Cylinder(radius=0.021, length=0.044),
        origin=_joint_y_origin((0.000, 0.000, 0.000)),
        material=dark_steel,
        name="knee_barrel",
    )
    shank.visual(
        Box((0.042, 0.048, 0.032)),
        origin=Origin(xyz=(0.000, 0.000, -0.014)),
        material=dark_steel,
        name="knee_hub",
    )
    shank.visual(shank_shell_mesh, material=utility_paint, name="shank_shell")
    shank.visual(
        Box((0.012, 0.046, 0.116)),
        origin=Origin(xyz=(0.044, 0.000, -0.140)),
        material=cover_polymer,
        name="left_shank_cover",
    )
    shank.visual(
        Box((0.012, 0.046, 0.116)),
        origin=Origin(xyz=(-0.044, 0.000, -0.140)),
        material=cover_polymer,
        name="right_shank_cover",
    )
    shank.visual(
        Box((0.026, 0.040, 0.188)),
        origin=Origin(xyz=(0.020, 0.000, -0.164)),
        material=dark_steel,
        name="front_spine",
    )
    shank.visual(
        Box((0.046, 0.014, 0.082)),
        origin=Origin(xyz=(0.012, 0.032, -0.285)),
        material=dark_steel,
        name="left_ankle_ear",
    )
    shank.visual(
        Box((0.046, 0.014, 0.082)),
        origin=Origin(xyz=(0.012, -0.032, -0.285)),
        material=dark_steel,
        name="right_ankle_ear",
    )
    shank.visual(
        Box((0.040, 0.022, 0.040)),
        origin=Origin(xyz=(0.012, 0.032, -0.252)),
        material=dark_steel,
        name="left_ankle_brace",
    )
    shank.visual(
        Box((0.040, 0.022, 0.040)),
        origin=Origin(xyz=(0.012, -0.032, -0.252)),
        material=dark_steel,
        name="right_ankle_brace",
    )
    shank.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=_joint_y_origin((0.012, 0.040, -0.290)),
        material=service_orange,
        name="left_ankle_cap",
    )
    shank.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=_joint_y_origin((0.012, -0.040, -0.290)),
        material=service_orange,
        name="right_ankle_cap",
    )
    _add_x_bolts(
        shank,
        prefix="shank_cover_left_bolt",
        x_pos=0.049,
        y_positions=(-0.018, 0.018),
        z_positions=(-0.182, -0.098),
        material=bolt_steel,
    )
    _add_x_bolts(
        shank,
        prefix="shank_cover_right_bolt",
        x_pos=-0.049,
        y_positions=(-0.018, 0.018),
        z_positions=(-0.182, -0.098),
        material=bolt_steel,
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.106, 0.074, 0.320)),
        mass=6.4,
        origin=Origin(xyz=(0.007, 0.000, -0.150)),
    )

    foot = model.part("foot_link")
    foot.visual(
        Cylinder(radius=0.019, length=0.046),
        origin=_joint_y_origin((0.000, 0.000, 0.000)),
        material=dark_steel,
        name="ankle_barrel",
    )
    foot.visual(
        Box((0.050, 0.056, 0.028)),
        origin=Origin(xyz=(0.000, 0.000, -0.014)),
        material=dark_steel,
        name="ankle_hub",
    )
    foot.visual(foot_shell_mesh, material=utility_paint, name="foot_shell")
    foot.visual(
        Box((0.060, 0.052, 0.040)),
        origin=Origin(xyz=(0.010, 0.000, -0.036)),
        material=dark_steel,
        name="ankle_spine",
    )
    foot.visual(
        Box((0.060, 0.048, 0.040)),
        origin=Origin(xyz=(-0.025, 0.000, -0.052)),
        material=dark_steel,
        name="heel_block",
    )
    foot.visual(
        Box((0.190, 0.094, 0.010)),
        origin=Origin(xyz=(0.048, 0.000, -0.080)),
        material=sole_rubber,
        name="sole_pad",
    )
    foot.visual(
        Box((0.040, 0.060, 0.016)),
        origin=Origin(xyz=(-0.042, 0.000, -0.070)),
        material=sole_rubber,
        name="heel_pad",
    )
    foot.visual(
        Box((0.048, 0.028, 0.010)),
        origin=Origin(xyz=(0.094, 0.028, -0.073)),
        material=sole_rubber,
        name="left_traction_pad",
    )
    foot.visual(
        Box((0.048, 0.028, 0.010)),
        origin=Origin(xyz=(0.094, -0.028, -0.073)),
        material=sole_rubber,
        name="right_traction_pad",
    )
    foot.visual(toe_guard_mesh, material=dark_steel, name="toe_guard")
    _add_x_bolts(
        foot,
        prefix="foot_plate_bolt",
        x_pos=0.010,
        y_positions=(-0.030, 0.030),
        z_positions=(-0.074, -0.072),
        material=bolt_steel,
        radius=0.0035,
        length=0.004,
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.240, 0.120, 0.100)),
        mass=3.5,
        origin=Origin(xyz=(0.050, 0.000, -0.045)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip,
        child=thigh,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=2.5,
            lower=-0.75,
            upper=0.90,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shank,
        origin=Origin(xyz=(0.010, 0.000, -0.330)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=2.8,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shank,
        child=foot,
        origin=Origin(xyz=(0.012, 0.000, -0.290)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=3.0,
            lower=-0.45,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hip = object_model.get_part("hip_module")
    thigh = object_model.get_part("thigh_link")
    shank = object_model.get_part("shank_link")
    foot = object_model.get_part("foot_link")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    hip_shell = hip.get_visual("hip_shell")
    left_hip_ear = hip.get_visual("left_hip_ear")
    right_hip_ear = hip.get_visual("right_hip_ear")
    thigh_shell = thigh.get_visual("thigh_shell")
    hip_barrel = thigh.get_visual("hip_barrel")
    left_hip_thrust_washer = thigh.get_visual("left_hip_thrust_washer")
    right_hip_thrust_washer = thigh.get_visual("right_hip_thrust_washer")
    left_knee_ear = thigh.get_visual("left_knee_ear")
    right_knee_ear = thigh.get_visual("right_knee_ear")
    shank_shell = shank.get_visual("shank_shell")
    knee_barrel = shank.get_visual("knee_barrel")
    left_ankle_ear = shank.get_visual("left_ankle_ear")
    right_ankle_ear = shank.get_visual("right_ankle_ear")
    foot_shell = foot.get_visual("foot_shell")
    ankle_barrel = foot.get_visual("ankle_barrel")
    toe_guard = foot.get_visual("toe_guard")

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

    ctx.check(
        "joint_axes_are_pitch_axes",
        hip_pitch.axis == (0.0, 1.0, 0.0)
        and knee_pitch.axis == (0.0, 1.0, 0.0)
        and ankle_pitch.axis == (0.0, 1.0, 0.0),
        (
            "Expected explicit +Y pitch axes, got "
            f"{hip_pitch.axis}, {knee_pitch.axis}, {ankle_pitch.axis}"
        ),
    )

    ctx.expect_origin_distance(thigh, hip, axes="xy", max_dist=0.001, name="hip_origin_aligned")
    ctx.expect_origin_gap(hip, shank, axis="z", min_gap=0.30, max_gap=0.36, name="knee_drop_realistic")
    ctx.expect_origin_gap(hip, foot, axis="z", min_gap=0.58, max_gap=0.70, name="overall_leg_height")
    ctx.expect_origin_distance(foot, hip, axes="x", max_dist=0.03, name="foot_stays_under_hip_at_rest")

    ctx.expect_overlap(
        thigh,
        hip,
        axes="xz",
        min_overlap=0.040,
        elem_a=hip_barrel,
        elem_b=left_hip_ear,
        name="hip_barrel_seated_in_left_clevis_plane",
    )
    ctx.expect_gap(
        hip,
        thigh,
        axis="y",
        positive_elem=left_hip_ear,
        negative_elem=hip_barrel,
        min_gap=0.002,
        max_gap=0.008,
        name="left_hip_side_clearance",
    )
    ctx.expect_gap(
        thigh,
        hip,
        axis="y",
        positive_elem=hip_barrel,
        negative_elem=right_hip_ear,
        min_gap=0.002,
        max_gap=0.008,
        name="right_hip_side_clearance",
    )
    ctx.expect_contact(
        hip,
        thigh,
        elem_a=left_hip_ear,
        elem_b=left_hip_thrust_washer,
        name="left_hip_thrust_contact",
    )
    ctx.expect_contact(
        hip,
        thigh,
        elem_a=right_hip_ear,
        elem_b=right_hip_thrust_washer,
        name="right_hip_thrust_contact",
    )
    ctx.expect_gap(
        hip,
        thigh,
        axis="z",
        positive_elem=hip_shell,
        negative_elem=thigh_shell,
        min_gap=0.020,
        max_gap=0.080,
        name="hip_to_thigh_joint_spacing",
    )

    ctx.expect_overlap(
        shank,
        thigh,
        axes="xz",
        min_overlap=0.036,
        elem_a=knee_barrel,
        elem_b=left_knee_ear,
        name="knee_barrel_seated_in_left_clevis_plane",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="y",
        positive_elem=left_knee_ear,
        negative_elem=knee_barrel,
        min_gap=0.001,
        max_gap=0.007,
        name="left_knee_side_clearance",
    )
    ctx.expect_gap(
        shank,
        thigh,
        axis="y",
        positive_elem=knee_barrel,
        negative_elem=right_knee_ear,
        min_gap=0.001,
        max_gap=0.007,
        name="right_knee_side_clearance",
    )
    ctx.expect_gap(
        thigh,
        shank,
        axis="z",
        positive_elem=thigh_shell,
        negative_elem=shank_shell,
        min_gap=0.020,
        max_gap=0.080,
        name="knee_joint_spacing",
    )

    ctx.expect_overlap(
        foot,
        shank,
        axes="xz",
        min_overlap=0.030,
        elem_a=ankle_barrel,
        elem_b=left_ankle_ear,
        name="ankle_barrel_seated_in_left_clevis_plane",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="y",
        positive_elem=left_ankle_ear,
        negative_elem=ankle_barrel,
        min_gap=0.001,
        max_gap=0.007,
        name="left_ankle_side_clearance",
    )
    ctx.expect_gap(
        foot,
        shank,
        axis="y",
        positive_elem=ankle_barrel,
        negative_elem=right_ankle_ear,
        min_gap=0.001,
        max_gap=0.007,
        name="right_ankle_side_clearance",
    )
    ctx.expect_gap(
        shank,
        foot,
        axis="z",
        positive_elem=shank_shell,
        negative_elem=foot_shell,
        min_gap=0.014,
        max_gap=0.080,
        name="ankle_joint_spacing",
    )

    foot_rest = ctx.part_world_position(foot)
    toe_rest = ctx.part_element_world_aabb(foot, elem=toe_guard)
    ctx.check("foot_position_query_available", foot_rest is not None, "Foot world position unavailable")
    ctx.check("toe_guard_query_available", toe_rest is not None, "Toe guard AABB unavailable")

    if foot_rest is not None and toe_rest is not None:
        with ctx.pose({hip_pitch: 0.45}):
            foot_hip_flex = ctx.part_world_position(foot)
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_hip_flex_pose")
            ctx.check(
                "hip_pose_moves_distal_chain",
                foot_hip_flex is not None and abs(foot_hip_flex[0] - foot_rest[0]) > 0.18,
                f"Hip articulation did not move foot enough: rest={foot_rest}, flex={foot_hip_flex}",
            )

        with ctx.pose({knee_pitch: 1.10}):
            foot_knee_flex = ctx.part_world_position(foot)
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_knee_flex_pose")
            ctx.check(
                "knee_pose_lifts_foot",
                foot_knee_flex is not None and foot_knee_flex[2] > foot_rest[2] + 0.12,
                f"Knee articulation did not lift foot enough: rest={foot_rest}, flex={foot_knee_flex}",
            )

        with ctx.pose({ankle_pitch: 0.30}):
            toe_plantar = ctx.part_element_world_aabb(foot, elem=toe_guard)
            ctx.check(
                "ankle_pose_rotates_toe_guard",
                toe_plantar is not None and toe_plantar[0][2] < toe_rest[0][2] - 0.015,
                f"Ankle articulation did not swing toe down enough: rest={toe_rest}, pose={toe_plantar}",
            )

        with ctx.pose({hip_pitch: 0.30, knee_pitch: 1.00, ankle_pitch: -0.18}):
            ctx.fail_if_parts_overlap_in_current_pose(name="no_overlaps_in_crouch_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
