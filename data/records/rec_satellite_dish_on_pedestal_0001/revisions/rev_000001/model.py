from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    LatheGeometry,
    Mesh,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _merge_meshes(*geometries):
    merged = None
    for geometry in geometries:
        merged = geometry.copy() if merged is None else merged.merge(geometry)
    if merged is None:
        raise ValueError("expected at least one geometry to merge")
    return merged


def _save_mesh(geometry, name: str) -> Mesh:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _reflector_mesh(diameter: float, depth: float, shell_thickness: float) -> Mesh:
    radius = diameter / 2.0
    outer_profile = []
    for step in range(1, 19):
        r = radius * step / 18.0
        z_inner = depth * (r / radius) ** 2
        thickness = shell_thickness * (1.18 - 0.33 * (r / radius))
        outer_profile.append((r, z_inner - thickness))

    inner_profile = []
    for step in range(18, -1, -1):
        r = radius * step / 18.0
        z_inner = depth * (r / radius) ** 2 if radius > 0.0 else 0.0
        inner_profile.append((r, z_inner))

    profile = [(0.0, -1.55 * shell_thickness), *outer_profile, (radius, depth), *inner_profile]
    reflector = LatheGeometry(profile, segments=64)
    return _save_mesh(reflector, "reflector_shell.obj")


def _yoke_arms_mesh() -> Mesh:
    left_arm = tube_from_spline_points(
        [
            (0.008, 0.108, 0.102),
            (0.040, 0.145, 0.210),
            (0.080, 0.172, 0.320),
            (0.118, 0.176, 0.420),
        ],
        radius=0.015,
        samples_per_segment=16,
        radial_segments=18,
    )
    right_arm = tube_from_spline_points(
        [
            (0.008, -0.108, 0.102),
            (0.040, -0.145, 0.210),
            (0.080, -0.172, 0.320),
            (0.118, -0.176, 0.420),
        ],
        radius=0.015,
        samples_per_segment=16,
        radial_segments=18,
    )
    return _save_mesh(_merge_meshes(left_arm, right_arm), "yoke_arms.obj")


def _dish_support_mesh() -> Mesh:
    rib_1 = tube_from_spline_points(
        [(0.012, 0.000, 0.000), (0.085, 0.110, 0.055), (0.205, 0.238, 0.105)],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=16,
    )
    rib_2 = tube_from_spline_points(
        [(0.012, 0.000, 0.000), (0.085, -0.110, 0.055), (0.205, -0.238, 0.105)],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=16,
    )
    rib_3 = tube_from_spline_points(
        [(0.012, 0.000, 0.000), (0.085, 0.110, 0.145), (0.205, 0.238, 0.235)],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=16,
    )
    rib_4 = tube_from_spline_points(
        [(0.012, 0.000, 0.000), (0.085, -0.110, 0.145), (0.205, -0.238, 0.235)],
        radius=0.008,
        samples_per_segment=14,
        radial_segments=16,
    )
    main_boom = tube_from_spline_points(
        [
            (0.018, 0.000, 0.020),
            (0.120, 0.000, 0.090),
            (0.300, 0.000, 0.145),
            (0.560, 0.000, 0.160),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
    )
    upper_left = tube_from_spline_points(
        [
            (0.205, 0.238, 0.235),
            (0.330, 0.125, 0.195),
            (0.465, 0.040, 0.172),
            (0.560, 0.010, 0.160),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=16,
    )
    upper_right = tube_from_spline_points(
        [
            (0.205, -0.238, 0.235),
            (0.330, -0.125, 0.195),
            (0.465, -0.040, 0.172),
            (0.560, -0.010, 0.160),
        ],
        radius=0.006,
        samples_per_segment=18,
        radial_segments=16,
    )
    return _save_mesh(
        _merge_meshes(rib_1, rib_2, rib_3, rib_4, main_boom, upper_left, upper_right),
        "dish_supports.obj",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="satellite_dish_terminal", assets=ASSETS)

    reflector_mesh = _reflector_mesh(diameter=0.88, depth=0.13, shell_thickness=0.009)
    yoke_arms_mesh = _yoke_arms_mesh()
    dish_support_mesh = _dish_support_mesh()

    pedestal = model.part("pedestal")
    pedestal.visual(Cylinder(radius=0.36, length=0.06), origin=Origin(xyz=(0.0, 0.0, 0.03)))
    pedestal.visual(Cylinder(radius=0.22, length=0.06), origin=Origin(xyz=(0.0, 0.0, 0.09)))
    pedestal.visual(Cylinder(radius=0.12, length=0.72), origin=Origin(xyz=(0.0, 0.0, 0.48)))
    pedestal.visual(Cylinder(radius=0.16, length=0.10), origin=Origin(xyz=(0.0, 0.0, 0.89)))
    pedestal.visual(Cylinder(radius=0.19, length=0.07), origin=Origin(xyz=(0.0, 0.0, 0.945)))
    pedestal.visual(Box((0.22, 0.16, 0.28)), origin=Origin(xyz=(-0.18, 0.0, 0.25)))
    pedestal.visual(Cylinder(radius=0.025, length=0.52), origin=Origin(xyz=(-0.125, 0.0, 0.59)))
    pedestal.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 1.02)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
    )

    azimuth_stage = model.part("azimuth_stage")
    azimuth_stage.visual(Cylinder(radius=0.17, length=0.08), origin=Origin(xyz=(0.0, 0.0, 0.04)))
    azimuth_stage.visual(Box((0.24, 0.24, 0.12)), origin=Origin(xyz=(-0.02, 0.0, 0.12)))
    azimuth_stage.visual(
        Cylinder(radius=0.018, length=0.36),
        origin=Origin(xyz=(0.12, 0.0, 0.42), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    azimuth_stage.visual(
        Cylinder(radius=0.022, length=0.06),
        origin=Origin(xyz=(0.12, 0.21, 0.42), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    azimuth_stage.visual(
        Cylinder(radius=0.022, length=0.06),
        origin=Origin(xyz=(0.12, -0.21, 0.42), rpy=(-math.pi / 2.0, 0.0, 0.0)),
    )
    azimuth_stage.visual(yoke_arms_mesh)
    azimuth_stage.visual(Box((0.11, 0.14, 0.09)), origin=Origin(xyz=(0.028, 0.0, 0.28)))
    azimuth_stage.visual(
        Cylinder(radius=0.026, length=0.22),
        origin=Origin(xyz=(-0.055, 0.0, 0.23), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    azimuth_stage.inertial = Inertial.from_geometry(
        Box((0.44, 0.46, 0.54)),
        mass=18.0,
        origin=Origin(xyz=(0.02, 0.0, 0.24)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        reflector_mesh,
        origin=Origin(xyz=(0.28, 0.0, 0.16), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    dish_assembly.visual(Box((0.016, 0.10, 0.10)), origin=Origin(xyz=(0.008, 0.0, 0.0)))
    dish_assembly.visual(Box((0.080, 0.17, 0.12)), origin=Origin(xyz=(0.055, 0.0, 0.04)))
    dish_assembly.visual(Cylinder(radius=0.035, length=0.10), origin=Origin(xyz=(0.026, 0.0, 0.0)))
    dish_assembly.visual(dish_support_mesh)
    dish_assembly.visual(Box((0.032, 0.048, 0.060)), origin=Origin(xyz=(0.575, 0.0, 0.160)))
    dish_assembly.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(0.645, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    dish_assembly.visual(
        Cylinder(radius=0.040, length=0.065),
        origin=Origin(xyz=(0.725, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    dish_assembly.visual(Box((0.030, 0.040, 0.055)), origin=Origin(xyz=(0.615, 0.042, 0.182)))
    dish_assembly.visual(Box((0.090, 0.074, 0.090)), origin=Origin(xyz=(0.120, -0.112, 0.080)))
    dish_assembly.inertial = Inertial.from_geometry(
        Box((1.00, 0.92, 0.62)),
        mass=13.0,
        origin=Origin(xyz=(0.30, 0.0, 0.16)),
    )

    model.articulation(
        "azimuth_slew",
        ArticulationType.REVOLUTE,
        parent="pedestal",
        child="azimuth_stage",
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.4,
            lower=-2.35,
            upper=2.35,
        ),
    )
    model.articulation(
        "elevation_tilt",
        ArticulationType.REVOLUTE,
        parent="azimuth_stage",
        child="dish_assembly",
        origin=Origin(xyz=(0.12, 0.0, 0.42)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.3,
            lower=-0.15,
            upper=0.95,
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
        "azimuth_stage",
        "dish_assembly",
        reason="elevation trunnion hardware is intentionally nested and the generated collision hulls around the fork tubes and support boom overlap conservatively",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.005, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("azimuth_stage", "pedestal", axes="xy", min_overlap=0.12)
    ctx.expect_origin_distance("azimuth_stage", "pedestal", axes="xy", max_dist=0.03)
    ctx.expect_aabb_gap("azimuth_stage", "pedestal", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "elevation_tilt",
        "dish_assembly",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )

    base_pos = ctx.part_world_position("pedestal")
    stage_pos = ctx.part_world_position("azimuth_stage")
    dish_origin = ctx.part_world_position("dish_assembly")
    assert 0.97 <= stage_pos[2] - base_pos[2] <= 0.99, (
        "azimuth stage should sit on the pedestal cap"
    )
    assert 0.11 <= dish_origin[0] - stage_pos[0] <= 0.13, (
        "elevation pivot should sit forward of the turntable"
    )
    assert 0.41 <= dish_origin[2] - stage_pos[2] <= 0.43, (
        "elevation pivot should sit well above the azimuth drive"
    )

    with ctx.pose(azimuth_slew=1.0):
        dish_left = ctx.part_world_position("dish_assembly")
        assert dish_left[1] > 0.07, "positive azimuth should swing the dish to camera-left"
        assert dish_left[0] < dish_origin[0] - 0.03, (
            "azimuth rotation should wrap the dish origin around the pedestal"
        )
        assert abs(dish_left[2] - dish_origin[2]) < 0.01, (
            "azimuth should not materially change dish height"
        )

    with ctx.pose(azimuth_slew=-1.0):
        dish_right = ctx.part_world_position("dish_assembly")
        assert dish_right[1] < -0.07, "negative azimuth should swing the dish to camera-right"
        assert dish_right[0] < dish_origin[0] - 0.03, (
            "dish origin should arc around the azimuth axis"
        )
        assert abs(dish_right[2] - dish_origin[2]) < 0.01, (
            "azimuth rotation should preserve elevation-axis height"
        )

    with ctx.pose(elevation_tilt=0.95):
        ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.04)

    with ctx.pose(elevation_tilt=0.90):
        ctx.expect_aabb_overlap("dish_assembly", "pedestal", axes="xy", min_overlap=0.04)

    with ctx.pose(azimuth_slew=-0.8, elevation_tilt=0.90):
        dish_combo = ctx.part_world_position("dish_assembly")
        assert dish_combo[1] < -0.06, "combined aiming pose should retain the azimuth offset"
        assert dish_combo[2] > stage_pos[2] + 0.40, (
            "combined pose should keep the elevation axis well clear of the drive"
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
