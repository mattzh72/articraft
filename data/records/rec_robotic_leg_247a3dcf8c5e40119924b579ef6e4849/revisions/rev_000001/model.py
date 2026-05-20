from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _superellipse_loop(
    depth: float,
    width: float,
    z: float,
    *,
    x_offset: float = 0.0,
    exponent: float = 3.0,
    segments: int = 48,
) -> list[tuple[float, float, float]]:
    """Closed XY loop at constant z, with depth along X and width along Y."""

    pts: list[tuple[float, float, float]] = []
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = math.copysign(abs(c) ** (2.0 / exponent), c) * depth * 0.5 + x_offset
        y = math.copysign(abs(s) ** (2.0 / exponent), s) * width * 0.5
        pts.append((x, y, z))
    return pts


def _lofted_shell(
    sections: list[tuple[float, float, float, float]],
    name: str,
    *,
    exponent: float = 3.2,
) -> object:
    """Create a smooth capped fairing from (z, depth, width, x_offset) sections."""

    profiles = [
        _superellipse_loop(depth, width, z, x_offset=x_offset, exponent=exponent)
        for z, depth, width, x_offset in sections
    ]
    return mesh_from_geometry(LoftGeometry(profiles, cap=True, closed=True), name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_robotic_leg")

    painted = model.material("ceramic_painted_metal", rgba=(0.86, 0.86, 0.82, 1.0))
    graphite = model.material("satin_graphite", rgba=(0.07, 0.075, 0.08, 1.0))
    polymer = model.material("smoked_polymer", rgba=(0.015, 0.017, 0.020, 1.0))
    elastomer = model.material("black_elastomer", rgba=(0.006, 0.006, 0.005, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.62, 0.64, 0.62, 1.0))
    copper = model.material("muted_copper", rgba=(0.72, 0.43, 0.24, 1.0))

    # Root hip cassette: a compact pelvis-side load path with an exposed yoke.
    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        _lofted_shell(
            [
                (0.040, 0.200, 0.260, -0.180),
                (0.105, 0.230, 0.225, -0.180),
                (0.175, 0.165, 0.170, -0.165),
            ],
            "hip_mount_cassette",
        ),
        material=painted,
        name="pelvis_cassette",
    )
    hip_mount.visual(
        Box((0.045, 0.245, 0.050)),
        origin=Origin(xyz=(-0.082, 0.0, 0.0)),
        material=graphite,
        name="hip_yoke_bridge",
    )
    hip_mount.visual(
        Box((0.040, 0.100, 0.070)),
        origin=Origin(xyz=(-0.095, 0.0, 0.050)),
        material=graphite,
        name="hip_upper_web",
    )
    hip_mount.visual(
        Cylinder(radius=0.065, length=0.032),
        origin=Origin(xyz=(0.0, -0.066, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hip_fork_lug_a",
    )
    hip_mount.visual(
        Cylinder(radius=0.065, length=0.032),
        origin=Origin(xyz=(0.0, 0.066, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hip_fork_lug_b",
    )
    hip_mount.visual(
        Cylinder(radius=0.078, length=0.022),
        origin=Origin(xyz=(0.0, 0.093, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="hip_actuator_cap",
    )
    hip_mount.visual(
        Box((0.012, 0.118, 0.010)),
        origin=Origin(xyz=(-0.080, 0.0, 0.126)),
        material=polymer,
        name="hip_service_seam",
    )

    # Thigh link: structural hub + tapered load-bearing shell + recessed actuator bay.
    thigh = model.part("thigh")
    thigh.visual(
        Cylinder(radius=0.052, length=0.100),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hip_hub",
    )
    thigh.visual(
        Cylinder(radius=0.034, length=0.145),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=graphite,
        name="hip_neck",
    )
    thigh.visual(
        _lofted_shell(
            [
                (-0.085, 0.142, 0.132, 0.004),
                (-0.175, 0.170, 0.148, 0.016),
                (-0.320, 0.135, 0.126, 0.010),
                (-0.455, 0.152, 0.136, -0.004),
            ],
            "thigh_load_shell",
        ),
        material=painted,
        name="thigh_shell",
    )
    thigh.visual(
        Box((0.030, 0.050, 0.450)),
        origin=Origin(xyz=(-0.075, 0.0, -0.287)),
        material=graphite,
        name="thigh_rear_spine",
    )
    thigh.visual(
        Box((0.016, 0.105, 0.310)),
        origin=Origin(xyz=(0.078, 0.0, -0.275)),
        material=polymer,
        name="thigh_actuator_bay",
    )
    thigh.visual(
        Cylinder(radius=0.014, length=0.235),
        origin=Origin(xyz=(0.092, -0.024, -0.282)),
        material=aluminum,
        name="thigh_linear_actuator",
    )
    thigh.visual(
        Cylinder(radius=0.010, length=0.225),
        origin=Origin(xyz=(0.095, 0.026, -0.286)),
        material=copper,
        name="thigh_actuator_rod",
    )
    thigh.visual(
        Box((0.008, 0.018, 0.355)),
        origin=Origin(xyz=(0.085, 0.0, -0.272)),
        material=graphite,
        name="thigh_front_seam",
    )
    thigh.visual(
        Box((0.040, 0.225, 0.034)),
        origin=Origin(xyz=(-0.068, 0.0, -0.520)),
        material=graphite,
        name="knee_yoke_bridge",
    )
    thigh.visual(
        Cylinder(radius=0.058, length=0.035),
        origin=Origin(xyz=(0.0, -0.0635, -0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="knee_fork_lug_a",
    )
    thigh.visual(
        Cylinder(radius=0.058, length=0.035),
        origin=Origin(xyz=(0.0, 0.0635, -0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="knee_fork_lug_b",
    )
    thigh.visual(
        Cylinder(radius=0.068, length=0.020),
        origin=Origin(xyz=(0.0, 0.091, -0.520), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="knee_actuator_cap",
    )

    # Shin link: slimmer carbon-metal surfacing with a rear service bay and ankle fork.
    shin = model.part("shin")
    shin.visual(
        Cylinder(radius=0.048, length=0.092),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="knee_hub",
    )
    shin.visual(
        Cylinder(radius=0.030, length=0.120),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=graphite,
        name="knee_neck",
    )
    shin.visual(
        _lofted_shell(
            [
                (-0.075, 0.126, 0.118, 0.000),
                (-0.175, 0.144, 0.126, -0.006),
                (-0.345, 0.102, 0.104, -0.010),
                (-0.450, 0.116, 0.112, 0.000),
            ],
            "shin_load_shell",
        ),
        material=painted,
        name="shin_shell",
    )
    shin.visual(
        Box((0.030, 0.044, 0.360)),
        origin=Origin(xyz=(0.058, 0.0, -0.265)),
        material=graphite,
        name="shin_front_spar",
    )
    shin.visual(
        Box((0.014, 0.092, 0.260)),
        origin=Origin(xyz=(-0.066, 0.0, -0.270)),
        material=polymer,
        name="shin_actuator_bay",
    )
    shin.visual(
        Cylinder(radius=0.012, length=0.205),
        origin=Origin(xyz=(-0.078, -0.022, -0.272)),
        material=aluminum,
        name="shin_linear_actuator",
    )
    shin.visual(
        Cylinder(radius=0.008, length=0.210),
        origin=Origin(xyz=(-0.080, 0.023, -0.272)),
        material=copper,
        name="shin_actuator_rod",
    )
    shin.visual(
        Box((0.012, 0.014, 0.440)),
        origin=Origin(xyz=(-0.067, 0.0, -0.276)),
        material=graphite,
        name="shin_rear_seam",
    )
    shin.visual(
        Box((0.034, 0.190, 0.030)),
        origin=Origin(xyz=(-0.054, 0.0, -0.500)),
        material=graphite,
        name="ankle_yoke_bridge",
    )
    shin.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, -0.056, -0.500), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="ankle_fork_lug_a",
    )
    shin.visual(
        Cylinder(radius=0.046, length=0.030),
        origin=Origin(xyz=(0.0, 0.056, -0.500), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="ankle_fork_lug_b",
    )
    shin.visual(
        Cylinder(radius=0.054, length=0.018),
        origin=Origin(xyz=(0.0, 0.080, -0.500), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="ankle_actuator_cap",
    )

    # Foot: compact ankle hub, painted upper bridge, and compliant elastomer sole.
    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.041, length=0.082),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="ankle_hub",
    )
    foot.visual(
        Box((0.060, 0.065, 0.118)),
        origin=Origin(xyz=(0.010, 0.0, -0.060)),
        material=graphite,
        name="ankle_to_sole_strut",
    )
    foot.visual(
        _lofted_shell(
            [
                (-0.145, 0.320, 0.118, 0.065),
                (-0.112, 0.292, 0.100, 0.070),
                (-0.076, 0.220, 0.080, 0.060),
            ],
            "foot_upper_shell",
            exponent=3.6,
        ),
        material=painted,
        name="foot_upper",
    )
    foot.visual(
        Box((0.340, 0.128, 0.034)),
        origin=Origin(xyz=(0.075, 0.0, -0.166)),
        material=elastomer,
        name="foot_sole",
    )
    foot.visual(
        Box((0.090, 0.122, 0.024)),
        origin=Origin(xyz=(0.235, 0.0, -0.142)),
        material=elastomer,
        name="toe_bumper",
    )
    foot.visual(
        Box((0.060, 0.090, 0.012)),
        origin=Origin(xyz=(-0.070, 0.0, -0.142)),
        material=polymer,
        name="heel_sensor_pad",
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=thigh,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=3.0, lower=-0.45, upper=1.10),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shin,
        origin=Origin(xyz=(0.0, 0.0, -0.520)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=3.2, lower=0.0, upper=2.10),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shin,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.500)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=4.0, lower=-0.65, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hip = object_model.get_articulation("hip_pitch")
    knee = object_model.get_articulation("knee_pitch")
    ankle = object_model.get_articulation("ankle_pitch")
    thigh = object_model.get_part("thigh")
    shin = object_model.get_part("shin")
    foot = object_model.get_part("foot")
    hip_mount = object_model.get_part("hip_mount")

    ctx.check(
        "serial hip knee ankle chain",
        getattr(hip.parent, "name", hip.parent) == "hip_mount"
        and getattr(hip.child, "name", hip.child) == "thigh"
        and getattr(knee.parent, "name", knee.parent) == "thigh"
        and getattr(knee.child, "name", knee.child) == "shin"
        and getattr(ankle.parent, "name", ankle.parent) == "shin"
        and getattr(ankle.child, "name", ankle.child) == "foot",
        details="The leg should be a single serial hip-knee-ankle kinematic chain.",
    )
    ctx.check(
        "plausible pitch ranges",
        hip.motion_limits.lower < 0.0
        and hip.motion_limits.upper > 1.0
        and knee.motion_limits.lower == 0.0
        and knee.motion_limits.upper > 2.0
        and ankle.motion_limits.lower < -0.5
        and ankle.motion_limits.upper > 0.3,
        details="Joint limits should cover consumer robotic leg pitch motion without impossible rotation.",
    )

    ctx.expect_overlap(
        hip_mount,
        thigh,
        axes="xz",
        min_overlap=0.035,
        elem_a="hip_fork_lug_b",
        elem_b="hip_hub",
        name="hip yoke surrounds thigh hub projection",
    )
    ctx.expect_overlap(
        thigh,
        shin,
        axes="xz",
        min_overlap=0.030,
        elem_a="knee_fork_lug_b",
        elem_b="knee_hub",
        name="knee fork surrounds shin hub projection",
    )
    ctx.expect_overlap(
        shin,
        foot,
        axes="xz",
        min_overlap=0.025,
        elem_a="ankle_fork_lug_b",
        elem_b="ankle_hub",
        name="ankle fork surrounds foot hub projection",
    )

    rest_foot = ctx.part_world_position(foot)
    with ctx.pose({hip: 0.85}):
        hip_flex_foot = ctx.part_world_position(foot)
    ctx.check(
        "hip flexion moves distal chain forward",
        rest_foot is not None and hip_flex_foot is not None and hip_flex_foot[0] > rest_foot[0] + 0.45,
        details=f"rest={rest_foot}, hip_flexed={hip_flex_foot}",
    )

    with ctx.pose({knee: 1.20}):
        knee_flex_foot = ctx.part_world_position(foot)
    ctx.check(
        "knee flexion folds the ankle back",
        rest_foot is not None and knee_flex_foot is not None and knee_flex_foot[0] < rest_foot[0] - 0.40,
        details=f"rest={rest_foot}, knee_flexed={knee_flex_foot}",
    )

    rest_foot_aabb = ctx.part_world_aabb(foot)
    with ctx.pose({ankle: -0.45}):
        toe_up_aabb = ctx.part_world_aabb(foot)
    ctx.check(
        "ankle pitch rotates foot shell",
        rest_foot_aabb is not None
        and toe_up_aabb is not None
        and toe_up_aabb[1][0] > rest_foot_aabb[1][0] + 0.025,
        details=f"rest_aabb={rest_foot_aabb}, ankle_pitch_aabb={toe_up_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
