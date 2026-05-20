from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
from math import pi

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_side_loft,
    tube_from_spline_points,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _link_shell_mesh(name: str, sections):
    geom = superellipse_side_loft(
        sections,
        exponents=2.7,
        segments=52,
        cap=True,
        closed=True,
    )
    geom.rotate_z(-pi / 2.0)
    return _save_mesh(name, geom)


def _cable_mesh(name: str, points, radius: float):
    geom = tube_from_spline_points(
        points,
        radius=radius,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    return _save_mesh(name, geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_arm", assets=ASSETS)

    orange_paint = model.material("orange_paint", rgba=(0.92, 0.47, 0.14, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.22, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    steel = model.material("steel", rgba=(0.64, 0.66, 0.70, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    pedestal_mesh = _save_mesh(
        "pedestal_body.obj",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.150, 0.0),
                (0.150, 0.012),
                (0.136, 0.028),
                (0.110, 0.060),
                (0.086, 0.104),
                (0.086, 0.125),
                (0.0, 0.125),
            ],
            segments=56,
        ),
    )
    upper_arm_mesh = _link_shell_mesh(
        "upper_arm_shell.obj",
        [
            (0.000, -0.050, 0.050, 0.102),
            (0.090, -0.053, 0.058, 0.108),
            (0.220, -0.041, 0.048, 0.090),
            (0.340, -0.060, 0.060, 0.116),
        ],
    )
    forearm_mesh = _link_shell_mesh(
        "forearm_shell.obj",
        [
            (0.000, -0.045, 0.045, 0.094),
            (0.100, -0.043, 0.048, 0.088),
            (0.220, -0.034, 0.040, 0.074),
            (0.300, -0.050, 0.050, 0.094),
        ],
    )
    wrist_mesh = _link_shell_mesh(
        "wrist_shell.obj",
        [
            (0.000, -0.038, 0.038, 0.082),
            (0.050, -0.033, 0.035, 0.070),
            (0.100, -0.030, 0.032, 0.060),
        ],
    )
    upper_cable_mesh = _cable_mesh(
        "upper_arm_cable.obj",
        [
            (0.030, -0.016, 0.043),
            (0.120, -0.024, 0.052),
            (0.240, -0.022, 0.047),
            (0.330, -0.016, 0.050),
        ],
        radius=0.007,
    )
    forearm_cable_mesh = _cable_mesh(
        "forearm_cable.obj",
        [
            (0.020, -0.012, 0.036),
            (0.100, -0.018, 0.042),
            (0.210, -0.017, 0.039),
            (0.285, -0.012, 0.040),
        ],
        radius=0.006,
    )

    pedestal = model.part("pedestal")
    pedestal.visual(
        pedestal_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=graphite,
        name="pedestal_shell",
    )
    pedestal.visual(
        Cylinder(radius=0.086, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=dark_metal,
        name="turntable_flange",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.125),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
    )

    turret = model.part("turret")
    turret.visual(
        Cylinder(radius=0.082, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=dark_metal,
        name="base_collar",
    )
    turret.visual(
        Cylinder(radius=0.072, length=0.142),
        origin=Origin(xyz=(0.0, 0.0, 0.103)),
        material=orange_paint,
        name="rotating_column",
    )
    turret.visual(
        Box((0.090, 0.120, 0.062)),
        origin=Origin(xyz=(0.020, 0.0, 0.176)),
        material=orange_paint,
        name="shoulder_bridge",
    )
    turret.visual(
        Box((0.055, 0.020, 0.150)),
        origin=Origin(xyz=(0.015, 0.052, 0.160)),
        material=orange_paint,
        name="left_yoke",
    )
    turret.visual(
        Box((0.055, 0.020, 0.150)),
        origin=Origin(xyz=(0.015, -0.052, 0.160)),
        material=orange_paint,
        name="right_yoke",
    )
    turret.visual(
        Cylinder(radius=0.054, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.214), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shoulder_motor",
    )
    turret.inertial = Inertial.from_geometry(
        Box((0.180, 0.180, 0.220)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        upper_arm_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=orange_paint,
        name="upper_arm_shell",
    )
    upper_arm.visual(
        Cylinder(radius=0.049, length=0.126),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="shoulder_collar",
    )
    upper_arm.visual(
        Cylinder(radius=0.050, length=0.112),
        origin=Origin(xyz=(0.340, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_hub",
    )
    upper_arm.visual(
        upper_cable_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_rubber,
        name="upper_arm_cable",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.360, 0.110, 0.130)),
        mass=11.0,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        forearm_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=orange_paint,
        name="forearm_shell",
    )
    forearm.visual(
        Cylinder(radius=0.046, length=0.106),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elbow_collar",
    )
    forearm.visual(
        Cylinder(radius=0.041, length=0.096),
        origin=Origin(xyz=(0.300, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wrist_hub",
    )
    forearm.visual(
        forearm_cable_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=black_rubber,
        name="forearm_cable",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.310, 0.095, 0.110)),
        mass=7.5,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        wrist_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=orange_paint,
        name="wrist_body",
    )
    wrist.visual(
        Cylinder(radius=0.040, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="wrist_collar",
    )
    wrist.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(0.066, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="roll_nose",
    )
    wrist.visual(
        Box((0.028, 0.070, 0.050)),
        origin=Origin(xyz=(0.072, 0.0, 0.0)),
        material=orange_paint,
        name="roll_motor_cover",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.120, 0.090, 0.090)),
        mass=3.0,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    tool_head = model.part("tool_head")
    tool_head.visual(
        Cylinder(radius=0.028, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="roll_flange",
    )
    tool_head.visual(
        Box((0.060, 0.064, 0.050)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=steel,
        name="gripper_palm",
    )
    tool_head.visual(
        Box((0.024, 0.052, 0.016)),
        origin=Origin(xyz=(0.056, 0.0, -0.028)),
        material=graphite,
        name="tool_mount",
    )
    tool_head.inertial = Inertial.from_geometry(
        Box((0.090, 0.090, 0.085)),
        mass=1.4,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    left_finger = model.part("left_finger")
    left_finger.visual(
        Box((0.022, 0.018, 0.020)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=steel,
        name="left_carriage",
    )
    left_finger.visual(
        Box((0.042, 0.010, 0.052)),
        origin=Origin(xyz=(0.039, -0.010, 0.0)),
        material=graphite,
        name="left_jaw",
    )
    left_finger.visual(
        Box((0.026, 0.004, 0.034)),
        origin=Origin(xyz=(0.045, -0.016, 0.0)),
        material=black_rubber,
        name="left_pad",
    )
    left_finger.inertial = Inertial.from_geometry(
        Box((0.065, 0.020, 0.055)),
        mass=0.45,
        origin=Origin(xyz=(0.032, -0.004, 0.0)),
    )

    right_finger = model.part("right_finger")
    right_finger.visual(
        Box((0.022, 0.018, 0.020)),
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
        material=steel,
        name="right_carriage",
    )
    right_finger.visual(
        Box((0.042, 0.010, 0.052)),
        origin=Origin(xyz=(0.039, 0.010, 0.0)),
        material=graphite,
        name="right_jaw",
    )
    right_finger.visual(
        Box((0.026, 0.004, 0.034)),
        origin=Origin(xyz=(0.045, 0.016, 0.0)),
        material=black_rubber,
        name="right_pad",
    )
    right_finger.inertial = Inertial.from_geometry(
        Box((0.065, 0.020, 0.055)),
        mass=0.45,
        origin=Origin(xyz=(0.032, 0.004, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent="pedestal",
        child="turret",
        origin=Origin(xyz=(0.0, 0.0, 0.127)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.6,
            lower=-2.9,
            upper=2.9,
        ),
    )
    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent="turret",
        child="upper_arm",
        origin=Origin(xyz=(0.0, 0.0, 0.214)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=1.8,
            lower=-0.75,
            upper=1.55,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="forearm",
        origin=Origin(xyz=(0.340, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=70.0,
            velocity=2.0,
            lower=-0.25,
            upper=2.35,
        ),
    )
    model.articulation(
        "wrist_pitch",
        ArticulationType.REVOLUTE,
        parent="forearm",
        child="wrist",
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.6,
            lower=-1.45,
            upper=1.55,
        ),
    )
    model.articulation(
        "wrist_roll",
        ArticulationType.CONTINUOUS,
        parent="wrist",
        child="tool_head",
        origin=Origin(xyz=(0.094, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=3.4,
        ),
    )
    model.articulation(
        "left_finger_slide",
        ArticulationType.PRISMATIC,
        parent="tool_head",
        child="left_finger",
        origin=Origin(xyz=(0.028, 0.028, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )
    model.articulation(
        "right_finger_slide",
        ArticulationType.PRISMATIC,
        parent="tool_head",
        child="right_finger",
        origin=Origin(xyz=(0.028, -0.028, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.08,
            lower=0.0,
            upper=0.018,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("pedestal", "turret", axes="xy", max_dist=0.001)
    ctx.expect_aabb_overlap("pedestal", "turret", axes="xy", min_overlap=0.150)
    ctx.expect_aabb_contact("pedestal", "turret")
    ctx.expect_aabb_gap("turret", "pedestal", axis="z", max_gap=0.001, max_penetration=0.004)

    ctx.expect_aabb_contact("turret", "upper_arm")
    ctx.expect_aabb_overlap("turret", "upper_arm", axes="yz", min_overlap=0.090)
    ctx.expect_joint_motion_axis(
        "base_yaw",
        "upper_arm",
        world_axis="y",
        direction="positive",
        min_delta=0.050,
    )
    ctx.expect_joint_motion_axis(
        "shoulder_pitch",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.040,
    )

    ctx.expect_aabb_contact("upper_arm", "forearm")
    ctx.expect_aabb_overlap("upper_arm", "forearm", axes="yz", min_overlap=0.070)
    ctx.expect_joint_motion_axis(
        "elbow_pitch",
        "forearm",
        world_axis="z",
        direction="positive",
        min_delta=0.040,
    )

    ctx.expect_aabb_contact("forearm", "wrist")
    ctx.expect_aabb_overlap("forearm", "wrist", axes="yz", min_overlap=0.060)
    ctx.expect_joint_motion_axis(
        "wrist_pitch",
        "wrist",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_aabb_contact("wrist", "tool_head")
    ctx.expect_aabb_overlap("wrist", "tool_head", axes="yz", min_overlap=0.040)

    ctx.expect_aabb_contact("tool_head", "left_finger")
    ctx.expect_aabb_contact("tool_head", "right_finger")
    ctx.expect_aabb_overlap("left_finger", "right_finger", axes="xz", min_overlap=0.030)
    ctx.expect_aabb_gap("left_finger", "right_finger", axis="y", max_gap=0.035, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "left_finger_slide",
        "left_finger",
        world_axis="y",
        direction="positive",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "right_finger_slide",
        "right_finger",
        world_axis="y",
        direction="negative",
        min_delta=0.010,
    )

    with ctx.pose(shoulder_pitch=1.05, elbow_pitch=1.25, wrist_pitch=-0.55):
        ctx.expect_aabb_contact("turret", "upper_arm")
        ctx.expect_aabb_contact("upper_arm", "forearm")
        ctx.expect_aabb_contact("forearm", "wrist")
        ctx.expect_aabb_contact("wrist", "tool_head")
        ctx.expect_aabb_contact("tool_head", "left_finger")
        ctx.expect_aabb_contact("tool_head", "right_finger")
        ctx.expect_aabb_overlap("left_finger", "right_finger", axes="xz", min_overlap=0.030)

    with ctx.pose(base_yaw=1.20, shoulder_pitch=0.70, elbow_pitch=0.95):
        ctx.expect_aabb_contact("upper_arm", "forearm")
        ctx.expect_aabb_contact("forearm", "wrist")
        ctx.expect_aabb_contact("wrist", "tool_head")

    with ctx.pose(shoulder_pitch=-0.70, elbow_pitch=0.10, wrist_pitch=0.0):
        ctx.expect_aabb_contact("turret", "upper_arm")
        ctx.expect_aabb_contact("upper_arm", "forearm")
        ctx.expect_aabb_contact("forearm", "wrist")
        ctx.expect_aabb_contact("wrist", "tool_head")

    with ctx.pose(wrist_roll=1.57):
        ctx.expect_aabb_contact("wrist", "tool_head")
        ctx.expect_aabb_contact("tool_head", "left_finger")
        ctx.expect_aabb_contact("tool_head", "right_finger")

    with ctx.pose(wrist_roll=0.78, left_finger_slide=0.018, right_finger_slide=0.018):
        ctx.expect_aabb_contact("wrist", "tool_head")
        ctx.expect_aabb_contact("tool_head", "left_finger")
        ctx.expect_aabb_contact("tool_head", "right_finger")

    with ctx.pose(left_finger_slide=0.018, right_finger_slide=0.018):
        ctx.expect_aabb_contact("tool_head", "left_finger")
        ctx.expect_aabb_contact("tool_head", "right_finger")
        ctx.expect_aabb_overlap("left_finger", "right_finger", axes="xz", min_overlap=0.030)
        ctx.expect_aabb_gap("left_finger", "right_finger", axis="y", max_gap=0.070, max_penetration=0.0)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
