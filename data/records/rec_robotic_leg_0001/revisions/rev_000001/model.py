from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LoftGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _loft_z(
    sections: list[tuple[float, float, float, float]],
    *,
    corner_scale: float = 0.22,
    corner_segments: int = 10,
):
    profiles = []
    for z, size_x, size_y, x_shift in sections:
        outline = rounded_rect_profile(
            size_x,
            size_y,
            radius=min(size_x, size_y) * corner_scale,
            corner_segments=corner_segments,
        )
        profiles.append([(x + x_shift, y, z) for x, y in outline])
    return LoftGeometry(profiles, cap=True, closed=True)


def _foot_loft(
    sections: list[tuple[float, float, float, float]],
    *,
    exponent: float,
):
    return superellipse_side_loft(
        sections,
        exponents=exponent,
        segments=56,
        cap=True,
        closed=True,
    ).rotate_z(-pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="humanoid_robotic_leg", assets=ASSETS)

    materials = {
        "graphite": _make_material("graphite_shell", (0.22, 0.23, 0.25, 1.0)),
        "steel": _make_material("brushed_steel", (0.67, 0.70, 0.73, 1.0)),
        "polymer": _make_material("matte_polymer", (0.12, 0.13, 0.14, 1.0)),
        "rubber": _make_material("rubber_tread", (0.08, 0.08, 0.09, 1.0)),
    }
    model.materials.extend(materials.values())

    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        _save_mesh(
            "hip_mount_body.obj",
            _loft_z(
                [
                    (-0.002, 0.084, 0.090, 0.000),
                    (0.010, 0.122, 0.100, 0.000),
                    (0.030, 0.116, 0.094, 0.010),
                    (0.056, 0.092, 0.078, 0.012),
                ],
                corner_scale=0.24,
            ),
        ),
        origin=Origin(),
        material=materials["graphite"],
        name="hip_mount_shell",
    )
    hip_mount.visual(
        Cylinder(radius=0.017, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
        name="hip_bearing_barrel",
    )
    hip_mount.visual(
        Cylinder(radius=0.015, length=0.028),
        origin=Origin(xyz=(0.038, 0.0, 0.022), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["polymer"],
        name="hip_front_pod",
    )
    hip_mount.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.032, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=materials["polymer"],
        name="hip_rear_pod",
    )
    hip_mount.inertial = Inertial.from_geometry(
        Box((0.126, 0.100, 0.060)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
    )

    thigh = model.part("thigh")
    thigh.visual(
        _save_mesh(
            "thigh_shell.obj",
            _loft_z(
                [
                    (0.006, 0.092, 0.118, 0.000),
                    (-0.038, 0.108, 0.124, 0.010),
                    (-0.092, 0.116, 0.118, 0.014),
                    (-0.154, 0.112, 0.104, 0.012),
                    (-0.220, 0.099, 0.095, 0.005),
                    (-0.272, 0.086, 0.086, -0.002),
                    (-0.302, 0.080, 0.090, -0.004),
                ],
            ),
        ),
        origin=Origin(),
        material=materials["graphite"],
        name="thigh_shell",
    )
    thigh.visual(
        _save_mesh(
            "thigh_blister.obj",
            _loft_z(
                [
                    (-0.028, 0.032, 0.060, 0.030),
                    (-0.088, 0.040, 0.066, 0.034),
                    (-0.168, 0.038, 0.060, 0.028),
                    (-0.236, 0.028, 0.046, 0.018),
                ],
                corner_scale=0.26,
            ),
        ),
        origin=Origin(),
        material=materials["polymer"],
        name="thigh_actuator_fairing",
    )
    thigh.visual(
        Cylinder(radius=0.024, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, -0.008), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
        name="hip_joint_cover",
    )
    thigh.visual(
        Cylinder(radius=0.022, length=0.102),
        origin=Origin(xyz=(-0.004, 0.0, -0.292), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
        name="knee_joint_cover_upper",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.115, 0.125, 0.308)),
        mass=3.4,
        origin=Origin(xyz=(0.010, 0.0, -0.150)),
    )

    shin = model.part("shin")
    shin.visual(
        _save_mesh(
            "shin_shell.obj",
            _loft_z(
                [
                    (0.008, 0.082, 0.096, -0.004),
                    (-0.050, 0.084, 0.092, 0.000),
                    (-0.120, 0.078, 0.084, 0.010),
                    (-0.200, 0.070, 0.078, 0.014),
                    (-0.270, 0.064, 0.074, 0.010),
                    (-0.304, 0.060, 0.076, 0.006),
                ],
            ),
        ),
        origin=Origin(),
        material=materials["graphite"],
        name="shin_shell",
    )
    shin.visual(
        _save_mesh(
            "shin_blister.obj",
            _loft_z(
                [
                    (-0.018, 0.028, 0.050, 0.018),
                    (-0.090, 0.034, 0.054, 0.022),
                    (-0.170, 0.032, 0.050, 0.024),
                    (-0.245, 0.024, 0.040, 0.018),
                ],
                corner_scale=0.24,
            ),
        ),
        origin=Origin(),
        material=materials["polymer"],
        name="shin_actuator_fairing",
    )
    shin.visual(
        _save_mesh(
            "shin_tendon_rail.obj",
            tube_from_spline_points(
                [
                    (-0.030, -0.020, -0.008),
                    (-0.035, -0.022, -0.098),
                    (-0.029, -0.020, -0.204),
                    (-0.016, -0.018, -0.296),
                ],
                radius=0.0055,
                samples_per_segment=14,
                radial_segments=16,
                cap_ends=True,
            ),
        ),
        origin=Origin(),
        material=materials["steel"],
        name="shin_tendon_rail",
    )
    shin.visual(
        Cylinder(radius=0.024, length=0.094),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
        name="knee_joint_cover_lower",
    )
    shin.visual(
        Cylinder(radius=0.021, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, -0.298), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
        name="ankle_joint_cover_upper",
    )
    shin.inertial = Inertial.from_geometry(
        Box((0.088, 0.098, 0.308)),
        mass=2.7,
        origin=Origin(xyz=(0.006, 0.0, -0.152)),
    )

    foot = model.part("foot")
    foot.visual(
        _save_mesh(
            "foot_shell.obj",
            _foot_loft(
                [
                    (-0.045, -0.084, -0.030, 0.060),
                    (-0.006, -0.076, 0.016, 0.084),
                    (0.040, -0.068, 0.004, 0.094),
                    (0.094, -0.056, -0.004, 0.090),
                    (0.146, -0.042, -0.010, 0.072),
                ],
                exponent=2.9,
            ),
        ),
        origin=Origin(),
        material=materials["graphite"],
        name="foot_shell",
    )
    foot.visual(
        _save_mesh(
            "foot_top_cap.obj",
            _foot_loft(
                [
                    (-0.008, -0.028, 0.018, 0.060),
                    (0.034, -0.018, 0.010, 0.070),
                    (0.070, -0.014, 0.002, 0.060),
                ],
                exponent=2.6,
            ),
        ),
        origin=Origin(),
        material=materials["polymer"],
        name="foot_top_cap",
    )
    foot.visual(
        _save_mesh(
            "foot_sole.obj",
            _foot_loft(
                [
                    (-0.048, -0.090, -0.064, 0.066),
                    (0.020, -0.086, -0.062, 0.098),
                    (0.092, -0.072, -0.056, 0.092),
                    (0.148, -0.058, -0.046, 0.076),
                ],
                exponent=2.4,
            ),
        ),
        origin=Origin(),
        material=materials["rubber"],
        name="foot_sole",
    )
    foot.visual(
        Cylinder(radius=0.022, length=0.084),
        origin=Origin(xyz=(-0.004, 0.0, -0.002), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["steel"],
        name="ankle_joint_cover_lower",
    )
    foot.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(xyz=(0.148, 0.0, -0.040), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["rubber"],
        name="toe_bumper",
    )
    foot.visual(
        Cylinder(radius=0.010, length=0.058),
        origin=Origin(xyz=(-0.046, 0.0, -0.066), rpy=(pi / 2.0, 0.0, 0.0)),
        material=materials["rubber"],
        name="heel_bumper",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.200, 0.100, 0.070)),
        mass=1.2,
        origin=Origin(xyz=(0.052, 0.0, -0.046)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent="hip_mount",
        child="thigh",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=2.2,
            lower=-0.85,
            upper=0.95,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent="thigh",
        child="shin",
        origin=Origin(xyz=(-0.004, 0.0, -0.300)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=2.8,
            lower=0.0,
            upper=1.60,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent="shin",
        child="foot",
        origin=Origin(xyz=(0.0, 0.0, -0.304)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=3.0,
            lower=-0.60,
            upper=0.60,
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
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("thigh", "hip_mount", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap("hip_mount", "thigh", axis="z", max_gap=0.008, max_penetration=0.025)
    ctx.expect_origin_distance("thigh", "hip_mount", axes="xy", max_dist=0.05)

    ctx.expect_aabb_overlap("shin", "thigh", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap("thigh", "shin", axis="z", max_gap=0.010, max_penetration=0.045)
    ctx.expect_origin_distance("shin", "thigh", axes="xy", max_dist=0.03)

    ctx.expect_aabb_overlap("foot", "shin", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_gap("shin", "foot", axis="z", max_gap=0.012, max_penetration=0.038)
    ctx.expect_origin_distance("foot", "shin", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("foot", "hip_mount", axes="xy", max_dist=0.07)

    ctx.expect_joint_motion_axis(
        "hip_pitch",
        "thigh",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "knee_pitch",
        "shin",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "ankle_pitch",
        "foot",
        world_axis="z",
        direction="positive",
        min_delta=0.015,
    )

    with ctx.pose(hip_pitch=0.90):
        ctx.expect_aabb_overlap("thigh", "hip_mount", axes="xy", min_overlap=0.012)
        ctx.expect_origin_distance("thigh", "hip_mount", axes="xy", max_dist=0.16)

    with ctx.pose(hip_pitch=-0.75):
        ctx.expect_aabb_overlap("thigh", "hip_mount", axes="xy", min_overlap=0.012)
        ctx.expect_origin_distance("thigh", "hip_mount", axes="xy", max_dist=0.14)

    with ctx.pose(knee_pitch=1.45):
        ctx.expect_aabb_overlap("shin", "thigh", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_gap("thigh", "shin", axis="z", max_gap=0.018, max_penetration=0.065)
        ctx.expect_origin_distance("foot", "hip_mount", axes="xy", max_dist=0.32)

    with ctx.pose(ankle_pitch=0.55):
        ctx.expect_aabb_overlap("foot", "shin", axes="xy", min_overlap=0.018)
        ctx.expect_origin_distance("foot", "shin", axes="xy", max_dist=0.10)

    with ctx.pose(ankle_pitch=-0.55):
        ctx.expect_aabb_overlap("foot", "shin", axes="xy", min_overlap=0.018)
        ctx.expect_origin_distance("foot", "shin", axes="xy", max_dist=0.10)

    with ctx.pose({"hip_pitch": 0.75, "knee_pitch": 1.20, "ankle_pitch": -0.20}):
        ctx.expect_aabb_overlap("shin", "thigh", axes="xy", min_overlap=0.015)
        ctx.expect_aabb_overlap("foot", "shin", axes="xy", min_overlap=0.016)
        ctx.expect_origin_distance("foot", "hip_mount", axes="xy", max_dist=0.34)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
