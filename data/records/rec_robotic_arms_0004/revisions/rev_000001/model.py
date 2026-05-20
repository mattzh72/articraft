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
    CapsuleGeometry,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    SectionLoftSpec,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _section_at_x(
    x: float,
    width_y: float,
    height_z: float,
    *,
    z_center: float = 0.0,
    exponent: float = 2.9,
    segments: int = 36,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for y, z in superellipse_profile(
            width_y,
            height_z,
            exponent=exponent,
            segments=segments,
        )
    ]


def _loft_mesh(
    filename: str,
    sections: list[list[tuple[float, float, float]]],
):
    geom = repair_loft(
        section_loft(
            SectionLoftSpec(
                sections=sections,
                cap=True,
                solid=True,
                repair="auto",
                symmetry=None,
            )
        )
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def _capsule_mesh(filename: str, *, overall_length: float, radius: float):
    geom = CapsuleGeometry(
        radius=radius,
        length=max(0.001, overall_length - (2.0 * radius)),
        radial_segments=28,
        height_segments=10,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_dir / filename)


def _y_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def _x_axis_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_arm", assets=ASSETS)

    matte_graphite = model.material("matte_graphite", rgba=(0.15, 0.16, 0.18, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    dark_polymer = model.material("dark_polymer", rgba=(0.07, 0.08, 0.09, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.06, 1.0))

    base_housing_mesh = _loft_mesh(
        "base_shoulder_housing.obj",
        [
            _section_at_x(-0.145, 0.150, 0.110, z_center=0.255, exponent=2.7),
            _section_at_x(-0.080, 0.138, 0.150, z_center=0.310, exponent=2.7),
            _section_at_x(-0.094, 0.104, 0.126, z_center=0.344, exponent=2.8),
            _section_at_x(-0.074, 0.066, 0.074, z_center=0.360, exponent=3.0),
        ],
    )
    upper_shell_mesh = _loft_mesh(
        "upper_arm_shell.obj",
        [
            _section_at_x(0.120, 0.080, 0.054, z_center=0.030, exponent=2.8),
            _section_at_x(0.175, 0.082, 0.088, z_center=0.028, exponent=2.9),
            _section_at_x(0.225, 0.074, 0.086, z_center=0.020, exponent=2.9),
            _section_at_x(0.248, 0.060, 0.056, z_center=0.016, exponent=3.0),
        ],
    )
    forearm_shell_mesh = _loft_mesh(
        "forearm_shell.obj",
        [
            _section_at_x(0.105, 0.060, 0.046, z_center=0.024, exponent=2.8),
            _section_at_x(0.145, 0.064, 0.068, z_center=0.018, exponent=2.9),
            _section_at_x(0.182, 0.058, 0.066, z_center=0.012, exponent=2.9),
            _section_at_x(0.205, 0.046, 0.048, z_center=0.010, exponent=3.0),
        ],
    )
    wrist_shell_mesh = _loft_mesh(
        "wrist_shell.obj",
        [
            _section_at_x(0.070, 0.040, 0.034, z_center=0.020, exponent=2.9),
            _section_at_x(0.090, 0.044, 0.046, z_center=0.018, exponent=2.9),
            _section_at_x(0.108, 0.040, 0.042, z_center=0.012, exponent=3.0),
            _section_at_x(0.118, 0.032, 0.032, z_center=0.010, exponent=3.0),
        ],
    )
    upper_actuator_mesh = _capsule_mesh(
        "upper_actuator_housing.obj",
        overall_length=0.125,
        radius=0.028,
    )
    forearm_actuator_mesh = _capsule_mesh(
        "forearm_actuator_housing.obj",
        overall_length=0.112,
        radius=0.022,
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.180, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=matte_graphite,
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.150, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=satin_aluminum,
        name="plinth_trim",
    )
    base.visual(
        Cylinder(radius=0.086, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=matte_graphite,
        name="pedestal_core",
    )
    base.visual(
        Cylinder(radius=0.096, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.275)),
        material=satin_aluminum,
        name="pedestal_collar",
    )
    base.visual(
        base_housing_mesh,
        origin=Origin(),
        material=matte_graphite,
        name="shoulder_housing",
    )
    base.visual(
        Cylinder(radius=0.068, length=0.064),
        origin=_y_axis_origin((0.0, -0.032, 0.370)),
        material=satin_aluminum,
        name="shoulder_mount",
    )
    base.visual(
        Box((0.048, 0.036, 0.060)),
        origin=Origin(xyz=(-0.056, -0.018, 0.360)),
        material=matte_graphite,
        name="shoulder_support_web",
    )
    base.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=_y_axis_origin((0.0, -0.070, 0.370)),
        material=dark_polymer,
        name="shoulder_break",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.006),
        origin=_y_axis_origin((0.0, -0.067, 0.370)),
        material=brushed_steel,
        name="shoulder_axis_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.360, 0.360, 0.420)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.068, length=0.064),
        origin=_y_axis_origin((0.0, 0.032, 0.0)),
        material=satin_aluminum,
        name="shoulder_collar",
    )
    upper_arm.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=_y_axis_origin((0.0, 0.070, 0.0)),
        material=dark_polymer,
        name="shoulder_seam_band",
    )
    upper_arm.visual(
        Box((0.070, 0.042, 0.020)),
        origin=Origin(xyz=(0.105, 0.020, 0.040)),
        material=matte_graphite,
        name="upper_root_bridge",
    )
    upper_arm.visual(
        upper_shell_mesh,
        origin=Origin(),
        material=matte_graphite,
        name="upper_shell",
    )
    upper_arm.visual(
        upper_actuator_mesh,
        origin=Origin(xyz=(0.195, 0.0, 0.058), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="upper_actuator_housing",
    )
    upper_arm.visual(
        Box((0.085, 0.038, 0.022)),
        origin=Origin(xyz=(0.195, 0.0, 0.034)),
        material=dark_polymer,
        name="upper_actuator_saddle",
    )
    upper_arm.visual(
        Box((0.040, 0.036, 0.018)),
        origin=Origin(xyz=(0.222, -0.008, 0.032)),
        material=matte_graphite,
        name="elbow_bridge",
    )
    upper_arm.visual(
        Cylinder(radius=0.056, length=0.052),
        origin=_y_axis_origin((0.315, -0.026, 0.0)),
        material=satin_aluminum,
        name="elbow_mount",
    )
    upper_arm.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=_y_axis_origin((0.315, -0.057, 0.0)),
        material=dark_polymer,
        name="elbow_break",
    )
    upper_arm.visual(
        Cylinder(radius=0.024, length=0.006),
        origin=_y_axis_origin((0.315, -0.055, 0.0)),
        material=brushed_steel,
        name="elbow_axis_cap",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.330, 0.120, 0.120)),
        mass=4.2,
        origin=Origin(xyz=(0.170, 0.0, 0.010)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.056, length=0.052),
        origin=_y_axis_origin((0.0, 0.026, 0.0)),
        material=satin_aluminum,
        name="elbow_collar",
    )
    forearm.visual(
        Cylinder(radius=0.043, length=0.010),
        origin=_y_axis_origin((0.0, 0.057, 0.0)),
        material=dark_polymer,
        name="elbow_seam_band",
    )
    forearm.visual(
        Box((0.070, 0.040, 0.018)),
        origin=Origin(xyz=(0.090, 0.018, 0.033)),
        material=matte_graphite,
        name="forearm_root_bridge",
    )
    forearm.visual(
        forearm_shell_mesh,
        origin=Origin(),
        material=matte_graphite,
        name="forearm_shell",
    )
    forearm.visual(
        forearm_actuator_mesh,
        origin=Origin(xyz=(0.155, 0.0, -0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_polymer,
        name="forearm_actuator_housing",
    )
    forearm.visual(
        Box((0.080, 0.034, 0.022)),
        origin=Origin(xyz=(0.155, 0.0, -0.032)),
        material=satin_aluminum,
        name="forearm_actuator_saddle",
    )
    forearm.visual(
        Box((0.026, 0.030, 0.016)),
        origin=Origin(xyz=(0.190, -0.006, 0.028)),
        material=matte_graphite,
        name="wrist_bridge",
    )
    forearm.visual(
        Cylinder(radius=0.045, length=0.044),
        origin=_y_axis_origin((0.258, -0.022, 0.0)),
        material=satin_aluminum,
        name="wrist_mount",
    )
    forearm.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=_y_axis_origin((0.258, -0.049, 0.0)),
        material=dark_polymer,
        name="wrist_break",
    )
    forearm.visual(
        Cylinder(radius=0.020, length=0.006),
        origin=_y_axis_origin((0.258, -0.047, 0.0)),
        material=brushed_steel,
        name="wrist_axis_cap",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.275, 0.095, 0.100)),
        mass=2.7,
        origin=Origin(xyz=(0.145, 0.0, -0.004)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.045, length=0.044),
        origin=_y_axis_origin((0.0, 0.022, 0.0)),
        material=satin_aluminum,
        name="wrist_collar",
    )
    wrist.visual(
        Cylinder(radius=0.034, length=0.010),
        origin=_y_axis_origin((0.0, 0.049, 0.0)),
        material=dark_polymer,
        name="wrist_seam_band",
    )
    wrist.visual(
        Box((0.040, 0.032, 0.018)),
        origin=Origin(xyz=(0.070, 0.014, 0.028)),
        material=matte_graphite,
        name="wrist_root_bridge",
    )
    wrist.visual(
        wrist_shell_mesh,
        origin=Origin(),
        material=matte_graphite,
        name="wrist_shell",
    )
    wrist.visual(
        Box((0.060, 0.034, 0.036)),
        origin=Origin(xyz=(0.075, 0.0, 0.026)),
        material=dark_polymer,
        name="wrist_actuator_housing",
    )
    wrist.visual(
        Box((0.032, 0.028, 0.014)),
        origin=Origin(xyz=(0.075, 0.0, 0.005)),
        material=satin_aluminum,
        name="wrist_actuator_band",
    )
    wrist.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=_x_axis_origin((0.132, 0.0, 0.0)),
        material=brushed_steel,
        name="tool_flange",
    )
    wrist.visual(
        Box((0.016, 0.040, 0.020)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=rubber_black,
        name="tool_pad",
    )
    wrist.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=_y_axis_origin((0.0, 0.047, 0.0)),
        material=brushed_steel,
        name="wrist_axis_cap",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.160, 0.080, 0.080)),
        mass=1.3,
        origin=Origin(xyz=(0.072, 0.0, 0.004)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent="base",
        child="upper_arm",
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.6,
            lower=-0.35,
            upper=1.15,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=1.8,
            lower=0.0,
            upper=1.05,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent="forearm",
        child="wrist",
        origin=Origin(xyz=(0.258, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.2,
            lower=-0.70,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_aabb_overlap("upper_arm", "base", axes="xz", min_overlap=0.10)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xz", min_overlap=0.08)
    ctx.expect_aabb_overlap("wrist", "forearm", axes="xz", min_overlap=0.06)

    ctx.expect_aabb_gap(
        "upper_arm",
        "base",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="shoulder_collar",
        negative_elem="shoulder_mount",
        name="shoulder_collar_seats_flush",
    )
    ctx.expect_aabb_gap(
        "forearm",
        "upper_arm",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="elbow_collar",
        negative_elem="elbow_mount",
        name="elbow_collar_seats_flush",
    )
    ctx.expect_aabb_gap(
        "wrist",
        "forearm",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="wrist_collar",
        negative_elem="wrist_mount",
        name="wrist_collar_seats_flush",
    )

    ctx.expect_aabb_gap(
        "forearm",
        "base",
        axis="x",
        min_gap=0.075,
        name="forearm_projects_forward_of_base",
    )
    ctx.expect_aabb_gap(
        "wrist",
        "base",
        axis="x",
        min_gap=0.18,
        name="wrist_projects_well_forward_of_base",
    )

    ctx.expect_joint_motion_axis(
        "shoulder_pitch",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "elbow_pitch",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "wrist",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(shoulder_pitch=0.95, elbow_pitch=0.80, wrist_pitch=0.25):
        ctx.expect_aabb_gap(
            "wrist",
            "base",
            axis="z",
            min_gap=0.18,
            name="work_pose_wrist_rises_clearly_above_base",
        )
        ctx.expect_aabb_overlap(
            "wrist",
            "base",
            axes="y",
            min_overlap=0.05,
            name="work_pose_wrist_stays_centered_over_base",
        )

    with ctx.pose(shoulder_pitch=0.95):
        ctx.expect_aabb_gap(
            "upper_arm",
            "base",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="shoulder_collar",
            negative_elem="shoulder_mount",
            name="raised_shoulder_collar_stays_seated",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
