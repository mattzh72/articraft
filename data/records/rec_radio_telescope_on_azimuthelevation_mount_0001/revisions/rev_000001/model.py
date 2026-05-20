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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_radio_telescope", assets=ASSETS)

    concrete = model.material("concrete", rgba=(0.69, 0.70, 0.72, 1.0))
    white_paint = model.material("white_paint", rgba=(0.91, 0.93, 0.95, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.38, 0.40, 0.43, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.17, 0.18, 0.20, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    instrument_black = model.material("instrument_black", rgba=(0.07, 0.08, 0.09, 1.0))

    dish_shell_profile = [
        (0.0, -1.55),
        (0.40, -1.52),
        (1.00, -1.45),
        (1.90, -1.24),
        (2.85, -0.86),
        (3.55, -0.48),
        (3.90, -0.24),
        (4.02, -0.06),
        (4.08, 0.03),
        (3.96, 0.11),
        (3.45, -0.03),
        (2.60, -0.34),
        (1.65, -0.74),
        (0.82, -1.05),
        (0.28, -1.18),
        (0.0, -1.22),
    ]
    dish_shell_mesh = _save_mesh(
        "dish_shell.obj",
        LatheGeometry(dish_shell_profile, segments=88),
    )
    dish_rim_mesh = _save_mesh(
        "dish_rim.obj",
        TorusGeometry(radius=4.03, tube=0.085, radial_segments=20, tubular_segments=96),
    )
    back_ring_outer_mesh = _save_mesh(
        "back_ring_outer.obj",
        TorusGeometry(radius=3.02, tube=0.055, radial_segments=18, tubular_segments=84),
    )
    back_ring_inner_mesh = _save_mesh(
        "back_ring_inner.obj",
        TorusGeometry(radius=1.72, tube=0.050, radial_segments=18, tubular_segments=72),
    )
    service_ring_mesh = _save_mesh(
        "service_ring.obj",
        TorusGeometry(radius=1.55, tube=0.08, radial_segments=16, tubular_segments=64),
    )

    arm_profile = rounded_rect_profile(0.44, 0.64, radius=0.08, corner_segments=8)
    left_arm_mesh = _save_mesh(
        "left_yoke_arm.obj",
        sweep_profile_along_spline(
            [
                (0.12, 1.72, 0.64),
                (0.26, 1.72, 1.95),
                (0.46, 1.66, 3.55),
                (0.68, 1.56, 4.72),
                (0.85, 1.46, 5.20),
            ],
            profile=arm_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    right_arm_mesh = _save_mesh(
        "right_yoke_arm.obj",
        sweep_profile_along_spline(
            [
                (0.12, -1.72, 0.64),
                (0.26, -1.72, 1.95),
                (0.46, -1.66, 3.55),
                (0.68, -1.56, 4.72),
                (0.85, -1.46, 5.20),
            ],
            profile=arm_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    spine_mesh = _save_mesh(
        "azimuth_spine.obj",
        sweep_profile_along_spline(
            [
                (-0.85, 0.0, 0.90),
                (-0.55, 0.0, 2.50),
                (0.10, 0.0, 4.15),
                (0.62, 0.0, 5.02),
                (0.85, 0.0, 5.20),
            ],
            profile=rounded_rect_profile(0.38, 0.48, radius=0.07, corner_segments=8),
            samples_per_segment=16,
            cap_profile=True,
        ),
    )

    left_base_brace_mesh = _save_mesh(
        "left_base_brace.obj",
        tube_from_spline_points(
            [
                (2.15, 0.0, 0.70),
                (1.58, 0.0, 1.58),
                (0.82, 0.0, 2.72),
            ],
            radius=0.14,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    front_base_brace_mesh = _save_mesh(
        "front_base_brace.obj",
        tube_from_spline_points(
            [
                (0.0, 2.15, 0.70),
                (0.0, 1.58, 1.58),
                (0.0, 0.82, 2.72),
            ],
            radius=0.14,
            samples_per_segment=14,
            radial_segments=18,
            cap_ends=True,
        ),
    )
    rib_mesh = _save_mesh(
        "dish_back_rib.obj",
        tube_from_spline_points(
            [
                (0.30, 0.0, 0.00),
                (0.92, 0.0, 0.15),
                (1.62, 0.0, 0.80),
                (2.36, 0.0, 3.00),
                (2.56, 0.0, 3.83),
            ],
            radius=0.055,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )
    feed_strut_mesh = _save_mesh(
        "feed_support_strut.obj",
        tube_from_spline_points(
            [
                (2.68, 0.0, 3.76),
                (3.25, 0.0, 2.65),
                (4.02, 0.0, 1.00),
                (4.60, 0.0, 0.02),
            ],
            radius=0.050,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    pedestal_base = model.part("pedestal_base")
    pedestal_base.visual(
        Box((6.0, 6.0, 0.70)),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=concrete,
        name="foundation_plinth",
    )
    pedestal_base.visual(
        Box((1.9, 2.6, 2.2)),
        origin=Origin(xyz=(-1.45, 0.0, 1.80)),
        material=machinery_gray,
        name="equipment_annex",
    )
    pedestal_base.visual(
        Cylinder(radius=0.95, length=4.20),
        origin=Origin(xyz=(0.0, 0.0, 2.80)),
        material=white_paint,
        name="pedestal_tower",
    )
    pedestal_base.visual(
        Cylinder(radius=1.30, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 5.15)),
        material=machinery_gray,
        name="azimuth_bearing_housing",
    )
    pedestal_base.visual(
        service_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 5.46)),
        material=dark_metal,
        name="service_walkway_ring",
    )
    pedestal_base.visual(left_base_brace_mesh, material=machinery_gray, name="brace_pos_x")
    pedestal_base.visual(
        left_base_brace_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=machinery_gray,
        name="brace_neg_x",
    )
    pedestal_base.visual(front_base_brace_mesh, material=machinery_gray, name="brace_pos_y")
    pedestal_base.visual(
        front_base_brace_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=machinery_gray,
        name="brace_neg_y",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Box((6.0, 6.0, 5.65)),
        mass=22000.0,
        origin=Origin(xyz=(0.0, 0.0, 2.825)),
    )

    azimuth_structure = model.part("azimuth_structure")
    azimuth_structure.visual(
        Cylinder(radius=1.40, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=dark_metal,
        name="turntable_drum",
    )
    azimuth_structure.visual(
        Cylinder(radius=2.10, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=machinery_gray,
        name="rotating_deck",
    )
    azimuth_structure.visual(
        Cylinder(radius=0.78, length=2.60),
        origin=Origin(xyz=(0.0, 0.0, 1.80)),
        material=white_paint,
        name="central_turret",
    )
    azimuth_structure.visual(left_arm_mesh, material=white_paint, name="left_yoke_arm")
    azimuth_structure.visual(right_arm_mesh, material=white_paint, name="right_yoke_arm")
    azimuth_structure.visual(spine_mesh, material=machinery_gray, name="rear_spine")
    azimuth_structure.visual(
        Cylinder(radius=0.28, length=3.25),
        origin=Origin(xyz=(0.85, 0.0, 5.20), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="elevation_crosshead",
    )
    azimuth_structure.visual(
        Box((1.15, 0.90, 1.00)),
        origin=Origin(xyz=(-1.05, 1.12, 1.25)),
        material=machinery_gray,
        name="drive_cabinet_left",
    )
    azimuth_structure.visual(
        Box((1.15, 0.90, 1.00)),
        origin=Origin(xyz=(-1.05, -1.12, 1.25)),
        material=machinery_gray,
        name="drive_cabinet_right",
    )
    azimuth_structure.visual(
        Box((1.30, 1.05, 0.72)),
        origin=Origin(xyz=(0.05, 0.0, 1.11)),
        material=dark_metal,
        name="azimuth_drive_pack",
    )
    azimuth_structure.inertial = Inertial.from_geometry(
        Cylinder(radius=1.95, length=6.00),
        mass=6400.0,
        origin=Origin(xyz=(0.15, 0.0, 2.40)),
    )

    dish_assembly = model.part("dish_assembly")
    dish_assembly.visual(
        Cylinder(radius=0.25, length=3.30),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trunnion_shaft",
    )
    dish_assembly.visual(
        Cylinder(radius=0.55, length=0.90),
        origin=Origin(xyz=(0.45, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machinery_gray,
        name="hub_barrel",
    )
    dish_assembly.visual(
        Box((0.90, 1.10, 1.10)),
        origin=Origin(xyz=(0.45, 0.0, 0.0)),
        material=machinery_gray,
        name="hub_block",
    )
    dish_assembly.visual(
        dish_shell_mesh,
        origin=Origin(xyz=(2.75, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_paint,
        name="main_reflector",
    )
    dish_assembly.visual(
        dish_rim_mesh,
        origin=Origin(xyz=(2.66, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="rim_stiffener",
    )
    dish_assembly.visual(
        back_ring_outer_mesh,
        origin=Origin(xyz=(2.35, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_ring_outer",
    )
    dish_assembly.visual(
        back_ring_inner_mesh,
        origin=Origin(xyz=(1.82, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_ring_inner",
    )
    for index in range(12):
        angle = (2.0 * math.pi * index) / 12.0
        dish_assembly.visual(
            rib_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=dark_metal,
            name=f"back_rib_{index:02d}",
        )
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0 + (math.pi / 6.0)
        dish_assembly.visual(
            feed_strut_mesh,
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=aluminum,
            name=f"feed_strut_{index:02d}",
        )
    dish_assembly.visual(
        Box((0.82, 0.58, 0.58)),
        origin=Origin(xyz=(4.78, 0.0, 0.0)),
        material=instrument_black,
        name="feed_cabin",
    )
    dish_assembly.visual(
        Cylinder(radius=0.17, length=0.68),
        origin=Origin(xyz=(5.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="feed_horn",
    )
    dish_assembly.visual(
        Box((1.60, 0.74, 1.00)),
        origin=Origin(xyz=(-1.10, 0.0, 0.0)),
        material=machinery_gray,
        name="counterweight_box",
    )
    dish_assembly.visual(
        Box((1.40, 0.22, 0.22)),
        origin=Origin(xyz=(-0.65, 0.34, 0.0)),
        material=dark_metal,
        name="counterweight_beam_upper",
    )
    dish_assembly.visual(
        Box((1.40, 0.22, 0.22)),
        origin=Origin(xyz=(-0.65, -0.34, 0.0)),
        material=dark_metal,
        name="counterweight_beam_lower",
    )
    dish_assembly.visual(
        Cylinder(radius=0.22, length=1.20),
        origin=Origin(xyz=(-1.72, 0.0, 0.36), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trim_weight_upper",
    )
    dish_assembly.visual(
        Cylinder(radius=0.22, length=1.20),
        origin=Origin(xyz=(-1.72, 0.0, -0.36), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="trim_weight_lower",
    )
    dish_assembly.inertial = Inertial.from_geometry(
        Box((7.20, 8.40, 8.40)),
        mass=5200.0,
        origin=Origin(xyz=(1.85, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent="pedestal_base",
        child="azimuth_structure",
        origin=Origin(xyz=(0.0, 0.0, 5.15)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120000.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "elevation_axis",
        ArticulationType.REVOLUTE,
        parent="azimuth_structure",
        child="dish_assembly",
        origin=Origin(xyz=(0.85, 0.0, 5.20)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80000.0,
            velocity=0.45,
            lower=0.0,
            upper=1.55,
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
        "pedestal_base",
        "azimuth_structure",
        reason="turntable drum nests into the azimuth bearing housing for a seated observatory mount",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.01,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("azimuth_structure", "pedestal_base", axes="xy", max_dist=0.01)
    ctx.expect_aabb_contact("azimuth_structure", "pedestal_base")
    ctx.expect_aabb_overlap("azimuth_structure", "pedestal_base", axes="xy", min_overlap=2.2)

    ctx.expect_aabb_contact("dish_assembly", "azimuth_structure")
    ctx.expect_aabb_overlap("dish_assembly", "azimuth_structure", axes="yz", min_overlap=2.5)
    ctx.expect_joint_motion_axis(
        "elevation_axis",
        "dish_assembly",
        world_axis="z",
        direction="positive",
        min_delta=1.0,
    )

    with ctx.pose(elevation_axis=0.0):
        ctx.expect_aabb_contact("dish_assembly", "azimuth_structure")
        ctx.expect_aabb_overlap("dish_assembly", "pedestal_base", axes="xy", min_overlap=1.4)
        ctx.expect_aabb_gap(
            "dish_assembly", "pedestal_base", axis="z", max_gap=2.0, max_penetration=0.0
        )

    with ctx.pose(elevation_axis=1.55):
        ctx.expect_aabb_contact("dish_assembly", "azimuth_structure")
        ctx.expect_aabb_overlap("dish_assembly", "pedestal_base", axes="xy", min_overlap=0.9)
        ctx.expect_aabb_gap(
            "dish_assembly", "pedestal_base", axis="z", max_gap=10.0, max_penetration=0.0
        )

    with ctx.pose(azimuth_rotation=math.pi / 2.0, elevation_axis=0.55):
        ctx.expect_aabb_contact("azimuth_structure", "pedestal_base")
        ctx.expect_aabb_overlap("azimuth_structure", "pedestal_base", axes="xy", min_overlap=2.2)
        ctx.expect_aabb_contact("dish_assembly", "azimuth_structure")
        ctx.expect_aabb_overlap("dish_assembly", "pedestal_base", axes="xy", min_overlap=1.2)

    with ctx.pose(azimuth_rotation=math.pi, elevation_axis=1.10):
        ctx.expect_aabb_contact("azimuth_structure", "pedestal_base")
        ctx.expect_aabb_overlap("azimuth_structure", "pedestal_base", axes="xy", min_overlap=2.2)
        ctx.expect_aabb_contact("dish_assembly", "azimuth_structure")
        ctx.expect_aabb_overlap("dish_assembly", "pedestal_base", axes="xy", min_overlap=1.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
