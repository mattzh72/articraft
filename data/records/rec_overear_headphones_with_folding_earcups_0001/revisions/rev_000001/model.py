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
    ExtrudeWithHolesGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    sweep_profile_along_spline,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
FOLD_ANGLE = 0.55


def _material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    builders = (
        lambda: Material(name=name, color=rgba),
        lambda: Material(name=name, rgba=rgba),
        lambda: Material(name, rgba),
        lambda: Material(name=name),
        lambda: Material(name),
    )
    last_error: Exception | None = None
    for build in builders:
        try:
            return build()
        except TypeError as exc:
            last_error = exc
    raise RuntimeError(f"Unable to construct material {name!r}") from last_error


def _band_mesh(
    filename: str,
    points: list[tuple[float, float, float]],
    profile_size: tuple[float, float],
    radius: float,
):
    geom = sweep_profile_along_spline(
        points,
        profile=rounded_rect_profile(
            profile_size[0],
            profile_size[1],
            radius=radius,
            corner_segments=10,
        ),
        samples_per_segment=18,
        cap_profile=True,
        up_hint=(0.0, 1.0, 0.0),
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _ring_mesh(
    filename: str,
    outer_height_z: float,
    outer_width_y: float,
    inner_height_z: float,
    inner_width_y: float,
    thickness_x: float,
    outer_radius: float,
    inner_radius: float,
):
    geom = ExtrudeWithHolesGeometry(
        outer_profile=rounded_rect_profile(
            outer_height_z,
            outer_width_y,
            radius=outer_radius,
            corner_segments=10,
        ),
        hole_profiles=[
            rounded_rect_profile(
                inner_height_z,
                inner_width_y,
                radius=inner_radius,
                corner_segments=10,
            )
        ],
        height=thickness_x,
        cap=True,
        center=True,
        closed=True,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _earcup_shell_mesh(filename: str):
    geom = superellipse_side_loft(
        [
            (-0.037, -0.102, -0.012, 0.038),
            (-0.023, -0.108, -0.008, 0.046),
            (0.000, -0.110, -0.004, 0.052),
            (0.023, -0.108, -0.008, 0.046),
            (0.037, -0.102, -0.012, 0.038),
        ],
        exponents=[2.5, 2.8, 3.1, 2.8, 2.5],
        segments=64,
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, ASSETS.mesh_path(filename))


def _add_earcup(
    part,
    *,
    shell_mesh,
    trim_ring_mesh,
    cushion_ring_mesh,
    side_sign: float,
    shell_material: Material,
    leather_material: Material,
    metal_material: Material,
    fabric_material: Material,
):
    part.visual(shell_mesh, material=shell_material)
    part.visual(
        Box((0.020, 0.070, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=metal_material,
    )
    part.visual(
        Cylinder(radius=0.008, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, -0.002), rpy=(pi / 2.0, 0.0, 0.0)),
        material=metal_material,
    )
    part.visual(
        Box((0.0025, 0.050, 0.070)),
        origin=Origin(xyz=(side_sign * 0.0175, 0.0, -0.056)),
        material=fabric_material,
    )
    part.visual(
        trim_ring_mesh,
        origin=Origin(xyz=(side_sign * 0.022, 0.0, -0.056)),
        material=metal_material,
    )
    part.visual(
        cushion_ring_mesh,
        origin=Origin(xyz=(side_sign * 0.033, 0.0, -0.056)),
        material=leather_material,
    )
    part.visual(
        Box((0.004, 0.054, 0.078)),
        origin=Origin(xyz=(-side_sign * 0.026, 0.0, -0.055)),
        material=shell_material,
    )
    part.visual(
        Box((0.005, 0.020, 0.034)),
        origin=Origin(xyz=(-side_sign * 0.028, 0.0, -0.028)),
        material=metal_material,
    )
    part.inertial = Inertial.from_geometry(
        Box((0.055, 0.082, 0.112)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, -0.056)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_over_ear_headphones", assets=ASSETS)

    matte_graphite = _material("matte_graphite", (0.15, 0.16, 0.18, 1.0))
    black_leather = _material("black_leather", (0.09, 0.09, 0.10, 1.0))
    satin_aluminum = _material("satin_aluminum", (0.70, 0.72, 0.75, 1.0))
    acoustic_fabric = _material("acoustic_fabric", (0.18, 0.19, 0.21, 1.0))

    outer_band_points = [
        (-0.108, 0.0, 0.112),
        (-0.093, 0.0, 0.146),
        (-0.056, 0.0, 0.173),
        (0.000, 0.0, 0.186),
        (0.056, 0.0, 0.173),
        (0.093, 0.0, 0.146),
        (0.108, 0.0, 0.112),
    ]
    inner_pad_points = [
        (-0.099, 0.0, 0.107),
        (-0.078, 0.0, 0.134),
        (-0.045, 0.0, 0.152),
        (0.000, 0.0, 0.160),
        (0.045, 0.0, 0.152),
        (0.078, 0.0, 0.134),
        (0.099, 0.0, 0.107),
    ]

    outer_band_mesh = _band_mesh("headband_outer.obj", outer_band_points, (0.012, 0.036), 0.003)
    top_trim_mesh = _band_mesh("headband_trim.obj", outer_band_points, (0.004, 0.017), 0.0015)
    inner_pad_mesh = _band_mesh("headband_pad.obj", inner_pad_points, (0.015, 0.028), 0.004)
    earcup_shell_mesh = _earcup_shell_mesh("earcup_shell.obj")
    trim_ring_mesh = _ring_mesh(
        "earcup_trim_ring.obj",
        outer_height_z=0.104,
        outer_width_y=0.084,
        inner_height_z=0.074,
        inner_width_y=0.052,
        thickness_x=0.006,
        outer_radius=0.020,
        inner_radius=0.016,
    )
    cushion_ring_mesh = _ring_mesh(
        "earcup_cushion_ring.obj",
        outer_height_z=0.112,
        outer_width_y=0.092,
        inner_height_z=0.080,
        inner_width_y=0.056,
        thickness_x=0.020,
        outer_radius=0.022,
        inner_radius=0.018,
    )

    headband = model.part("headband")
    headband.visual(outer_band_mesh, material=matte_graphite)
    headband.visual(top_trim_mesh, material=satin_aluminum)
    headband.visual(inner_pad_mesh, material=black_leather)
    for side_sign in (-1.0, 1.0):
        x = side_sign * 0.108
        headband.visual(
            Box((0.022, 0.042, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.112)),
            material=matte_graphite,
        )
        headband.visual(
            Box((0.010, 0.070, 0.010)),
            origin=Origin(xyz=(x, 0.0, 0.103)),
            material=matte_graphite,
        )
        headband.visual(
            Box((0.008, 0.012, 0.022)),
            origin=Origin(xyz=(x, 0.026, 0.098)),
            material=satin_aluminum,
        )
        headband.visual(
            Box((0.008, 0.012, 0.022)),
            origin=Origin(xyz=(x, -0.026, 0.098)),
            material=satin_aluminum,
        )
        headband.visual(
            Cylinder(radius=0.0055, length=0.064),
            origin=Origin(xyz=(x, 0.0, 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
            material=satin_aluminum,
        )
    headband.inertial = Inertial.from_geometry(
        Box((0.250, 0.100, 0.120)),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
    )

    left_earcup = model.part("left_earcup")
    _add_earcup(
        left_earcup,
        shell_mesh=earcup_shell_mesh,
        trim_ring_mesh=trim_ring_mesh,
        cushion_ring_mesh=cushion_ring_mesh,
        side_sign=1.0,
        shell_material=matte_graphite,
        leather_material=black_leather,
        metal_material=satin_aluminum,
        fabric_material=acoustic_fabric,
    )

    right_earcup = model.part("right_earcup")
    _add_earcup(
        right_earcup,
        shell_mesh=earcup_shell_mesh,
        trim_ring_mesh=trim_ring_mesh,
        cushion_ring_mesh=cushion_ring_mesh,
        side_sign=-1.0,
        shell_material=matte_graphite,
        leather_material=black_leather,
        metal_material=satin_aluminum,
        fabric_material=acoustic_fabric,
    )

    model.articulation(
        "left_cup_fold",
        ArticulationType.REVOLUTE,
        parent="headband",
        child="left_earcup",
        origin=Origin(xyz=(-0.108, 0.0, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=FOLD_ANGLE,
        ),
    )
    model.articulation(
        "right_cup_fold",
        ArticulationType.REVOLUTE,
        parent="headband",
        child="right_earcup",
        origin=Origin(xyz=(0.108, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=FOLD_ANGLE,
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
        max_pose_samples=192,
        overlap_tol=0.005,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("headband", "left_earcup", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_overlap("headband", "right_earcup", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_gap("headband", "left_earcup", axis="z", max_gap=0.004, max_penetration=0.014)
    ctx.expect_aabb_gap("headband", "right_earcup", axis="z", max_gap=0.004, max_penetration=0.014)
    ctx.expect_joint_motion_axis(
        "left_cup_fold",
        "left_earcup",
        world_axis="x",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "left_cup_fold",
        "left_earcup",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "right_cup_fold",
        "right_earcup",
        world_axis="x",
        direction="negative",
        min_delta=0.01,
    )
    ctx.expect_joint_motion_axis(
        "right_cup_fold",
        "right_earcup",
        world_axis="z",
        direction="positive",
        min_delta=0.01,
    )

    with ctx.pose(left_cup_fold=FOLD_ANGLE):
        ctx.expect_aabb_overlap("headband", "left_earcup", axes="xy", min_overlap=0.02)

    with ctx.pose(right_cup_fold=FOLD_ANGLE):
        ctx.expect_aabb_overlap("headband", "right_earcup", axes="xy", min_overlap=0.02)

    with ctx.pose(left_cup_fold=FOLD_ANGLE, right_cup_fold=FOLD_ANGLE):
        ctx.expect_aabb_overlap("headband", "left_earcup", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_overlap("headband", "right_earcup", axes="xy", min_overlap=0.02)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
