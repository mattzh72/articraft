from __future__ import annotations

import math

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.mesh_dir
MESH_DIR.mkdir(parents=True, exist_ok=True)


def _merge_geometries(*geometries):
    merged = None
    for geom in geometries:
        if geom is None:
            continue
        if merged is None:
            merged = geom
        else:
            merged.merge(geom)
    return merged


def _radial_spoke(y: float, r0: float, r1: float, angle: float, radius: float):
    length = max(r1 - r0, 1e-4)
    mid_r = 0.5 * (r0 + r1)
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=16)
    geom.rotate_y((math.pi / 2.0) - angle)
    geom.translate(mid_r * math.cos(angle), y, mid_r * math.sin(angle))
    return geom


def _axial_connector(radius_pos: float, y0: float, y1: float, angle: float, rod_radius: float):
    length = max(abs(y1 - y0), 1e-4)
    geom = CylinderGeometry(radius=rod_radius, height=length, radial_segments=16)
    geom.rotate_x(math.pi / 2.0)
    geom.translate(radius_pos * math.cos(angle), 0.5 * (y0 + y1), radius_pos * math.sin(angle))
    return geom


def _build_base_shell_mesh():
    profile = [
        (0.0, 0.000),
        (0.048, 0.001),
        (0.132, 0.005),
        (0.176, 0.013),
        (0.178, 0.023),
        (0.142, 0.031),
        (0.0, 0.031),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72),
        MESH_DIR / "tilting_fan_base_shell.obj",
    )


def _build_housing_mesh():
    profile = [
        (0.0, -0.108),
        (0.024, -0.108),
        (0.052, -0.098),
        (0.072, -0.074),
        (0.082, -0.036),
        (0.078, 0.000),
        (0.060, 0.022),
        (0.0, 0.030),
    ]
    geom = LatheGeometry(profile, segments=72)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, MESH_DIR / "tilting_fan_motor_housing.obj")


def _build_front_grille_mesh():
    y = 0.088
    pieces = [
        TorusGeometry(0.145, 0.0048, radial_segments=18, tubular_segments=60)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
        TorusGeometry(0.115, 0.0032, radial_segments=14, tubular_segments=48)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
        TorusGeometry(0.084, 0.0029, radial_segments=14, tubular_segments=44)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
        TorusGeometry(0.054, 0.0026, radial_segments=12, tubular_segments=40)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
    ]
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        pieces.append(_radial_spoke(y, 0.020, 0.140, angle, 0.0019))
    return mesh_from_geometry(
        _merge_geometries(*pieces),
        MESH_DIR / "tilting_fan_front_grille.obj",
    )


def _build_rear_grille_mesh():
    y = 0.030
    pieces = [
        TorusGeometry(0.145, 0.0048, radial_segments=18, tubular_segments=60)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
        TorusGeometry(0.098, 0.0031, radial_segments=14, tubular_segments=48)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
        TorusGeometry(0.042, 0.0038, radial_segments=14, tubular_segments=40)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, y, 0.0),
    ]
    for index in range(10):
        angle = (2.0 * math.pi * index) / 10.0
        pieces.append(_radial_spoke(y, 0.042, 0.140, angle, 0.0022))
    return mesh_from_geometry(
        _merge_geometries(*pieces),
        MESH_DIR / "tilting_fan_rear_grille.obj",
    )


def _blade_section(z_pos: float, chord: float, camber_y: float, thickness: float, sweep_x: float):
    x_lead = sweep_x - (0.52 * chord)
    x_trail = sweep_x + (0.48 * chord)
    return [
        (x_lead, camber_y + (0.60 * thickness), z_pos),
        (x_trail, camber_y + (0.18 * thickness), z_pos),
        (x_trail - (0.08 * chord), camber_y - (0.72 * thickness), z_pos),
        (x_lead + (0.12 * chord), camber_y - (0.32 * thickness), z_pos),
    ]


def _build_blade_mesh():
    profiles = [
        _blade_section(0.024, 0.050, 0.010, 0.0090, -0.004),
        _blade_section(0.054, 0.046, 0.008, 0.0075, 0.004),
        _blade_section(0.084, 0.040, 0.005, 0.0060, 0.012),
        _blade_section(0.110, 0.031, 0.002, 0.0048, 0.017),
        _blade_section(0.124, 0.018, 0.000, 0.0036, 0.020),
    ]
    geom = LoftGeometry(profiles, cap=True, closed=True)
    return mesh_from_geometry(geom, MESH_DIR / "tilting_fan_blade.obj")


def _build_spinner_mesh():
    geom = ConeGeometry(radius=0.022, height=0.030, radial_segments=36, closed=True)
    geom.rotate_x(-math.pi / 2.0)
    geom.translate(0.0, 0.019, 0.0)
    return mesh_from_geometry(geom, MESH_DIR / "tilting_fan_spinner.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_household_fan", assets=ASSETS)

    base_shell_mesh = _build_base_shell_mesh()
    housing_mesh = _build_housing_mesh()
    front_grille_mesh = _build_front_grille_mesh()
    rear_grille_mesh = _build_rear_grille_mesh()
    blade_mesh = _build_blade_mesh()
    spinner_mesh = _build_spinner_mesh()

    base = model.part("base")
    base.visual(base_shell_mesh)
    base.visual(Cylinder(radius=0.144, length=0.004), origin=Origin(xyz=(0.0, 0.0, 0.002)))
    base.visual(Cylinder(radius=0.029, length=0.028), origin=Origin(xyz=(0.0, 0.0, 0.045)))
    base.visual(Cylinder(radius=0.018, length=0.220), origin=Origin(xyz=(0.0, 0.0, 0.141)))
    base.visual(Box((0.160, 0.050, 0.022)), origin=Origin(xyz=(0.0, 0.0, 0.206)))
    base.visual(Box((0.022, 0.050, 0.185)), origin=Origin(xyz=(-0.090, 0.0, 0.288)))
    base.visual(Box((0.022, 0.050, 0.185)), origin=Origin(xyz=(0.090, 0.0, 0.288)))
    base.visual(
        Cylinder(radius=0.008, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.370), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    base.visual(Box((0.120, 0.075, 0.012)), origin=Origin(xyz=(0.0, 0.045, 0.036)))
    for x_pos in (-0.020, -0.003, 0.014, 0.031):
        base.visual(Box((0.012, 0.016, 0.007)), origin=Origin(xyz=(x_pos, 0.052, 0.0455)))
    base.visual(Cylinder(radius=0.014, length=0.010), origin=Origin(xyz=(-0.048, 0.045, 0.047)))
    base.visual(Box((0.020, 0.010, 0.004)), origin=Origin(xyz=(0.053, 0.046, 0.044)))
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.178, length=0.031),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0155)),
    )

    head = model.part("head")
    head.visual(housing_mesh, origin=Origin(xyz=(0.0, 0.070, 0.0)))
    head.visual(
        Cylinder(radius=0.032, length=0.104),
        origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    head.visual(Sphere(radius=0.024), origin=Origin(xyz=(0.0, -0.020, 0.0)))
    head.visual(
        Cylinder(radius=0.040, length=0.046),
        origin=Origin(xyz=(0.0, 0.084, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    head.visual(
        Cylinder(radius=0.048, length=0.008),
        origin=Origin(xyz=(0.0, 0.101, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    head.visual(
        Cylinder(radius=0.007, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    for x_pos in (-0.097, 0.097):
        head.visual(
            Cylinder(radius=0.017, length=0.014),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        )
        head.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(
                xyz=(x_pos + (0.011 if x_pos > 0.0 else -0.011), 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
        )
    head.visual(Cylinder(radius=0.005, length=0.022), origin=Origin(xyz=(0.0, -0.020, 0.030)))
    head.visual(Cylinder(radius=0.010, length=0.012), origin=Origin(xyz=(0.0, -0.020, 0.047)))
    head.visual(rear_grille_mesh, origin=Origin(xyz=(0.0, 0.070, 0.0)))
    head.visual(front_grille_mesh, origin=Origin(xyz=(0.0, 0.070, 0.0)))
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        head.visual(
            Cylinder(radius=0.003, length=0.058),
            origin=Origin(
                xyz=(0.145 * math.cos(angle), 0.129, 0.145 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
        )
    head.visual(
        Cylinder(radius=0.019, length=0.006),
        origin=Origin(xyz=(0.0, 0.161, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.180),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.030, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    rotor.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )
    for index in range(3):
        rotor.visual(blade_mesh, origin=Origin(rpy=(0.0, (2.0 * math.pi * index) / 3.0, 0.0)))
    rotor.visual(spinner_mesh)
    rotor.visual(Sphere(radius=0.009), origin=Origin(xyz=(0.0, 0.034, 0.0)))
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.110, length=0.028),
        mass=0.24,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent="base",
        child="head",
        origin=Origin(xyz=(0.0, 0.0, 0.370)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-0.45,
            upper=0.60,
        ),
    )
    model.articulation(
        "fan_spin",
        ArticulationType.CONTINUOUS,
        parent="head",
        child="rotor",
        origin=Origin(xyz=(0.0, 0.095, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=18.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "base",
        "head",
        reason="tilt hinge pin and bearing sleeves intentionally share the pivot envelope",
    )
    ctx.allow_overlap(
        "head",
        "rotor",
        reason="rotor hub sits concentrically inside the motor's front bearing collar",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.05)
    ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.03)
    ctx.expect_aabb_overlap("rotor", "head", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("rotor", "head", axes="xy", max_dist=0.11)
    ctx.expect_origin_distance("rotor", "base", axes="xy", max_dist=0.11)
    ctx.expect_joint_motion_axis(
        "head_tilt", "head", world_axis="z", direction="positive", min_delta=0.02
    )

    with ctx.pose(head_tilt=0.60):
        ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.08)
        ctx.expect_aabb_overlap("rotor", "head", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("rotor", "head", axes="xy", max_dist=0.11)
        ctx.expect_origin_distance("rotor", "base", axes="xy", max_dist=0.14)

    with ctx.pose(head_tilt=-0.45):
        ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.05)
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.08)
        ctx.expect_aabb_overlap("rotor", "head", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("rotor", "head", axes="xy", max_dist=0.11)
        ctx.expect_origin_distance("rotor", "base", axes="xy", max_dist=0.10)

    with ctx.pose(fan_spin=1.25):
        ctx.expect_aabb_overlap("rotor", "head", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("rotor", "head", axes="xy", max_dist=0.11)

    with ctx.pose(fan_spin=2.40):
        ctx.expect_aabb_overlap("rotor", "head", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("rotor", "head", axes="xy", max_dist=0.11)

    with ctx.pose(head_tilt=0.32, fan_spin=1.90):
        ctx.expect_aabb_overlap("rotor", "head", axes="xy", min_overlap=0.03)
        ctx.expect_origin_distance("rotor", "head", axes="xy", max_dist=0.11)
        ctx.expect_origin_distance("rotor", "base", axes="xy", max_dist=0.13)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
