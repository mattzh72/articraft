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
MESH_DIR = ASSETS.mesh_dir


def _mirror_x(points: list[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    return [(-x, y, z) for x, y, z in points]


def _build_saddle_meshes():
    MESH_DIR.mkdir(parents=True, exist_ok=True)

    shell_sections = [
        (-0.130, 0.008, 0.026, 0.148),
        (-0.090, 0.006, 0.031, 0.156),
        (-0.040, 0.007, 0.034, 0.146),
        (0.020, 0.006, 0.031, 0.112),
        (0.070, 0.005, 0.025, 0.078),
        (0.110, 0.004, 0.018, 0.046),
        (0.136, 0.004, 0.011, 0.020),
    ]
    pad_sections = [
        (-0.126, 0.013, 0.033, 0.154),
        (-0.082, 0.011, 0.041, 0.162),
        (-0.034, 0.011, 0.044, 0.152),
        (0.022, 0.010, 0.040, 0.116),
        (0.072, 0.009, 0.031, 0.081),
        (0.112, 0.008, 0.021, 0.049),
        (0.136, 0.007, 0.013, 0.021),
    ]

    shell_geom = superellipse_side_loft(shell_sections, exponents=2.8, segments=60)
    pad_geom = superellipse_side_loft(pad_sections, exponents=2.35, segments=60)

    rail_points = [
        (0.032, 0.088, 0.004),
        (0.032, 0.048, 0.001),
        (0.032, 0.008, 0.000),
        (0.032, -0.048, 0.002),
        (0.031, -0.098, 0.006),
        (0.028, -0.122, 0.015),
    ]
    left_rail = tube_from_spline_points(
        rail_points,
        radius=0.0034,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    right_rail = tube_from_spline_points(
        _mirror_x(rail_points),
        radius=0.0034,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    rail_geom = left_rail.merge(right_rail)

    shell_mesh = mesh_from_geometry(shell_geom, MESH_DIR / "dropper_saddle_shell.obj")
    pad_mesh = mesh_from_geometry(pad_geom, MESH_DIR / "dropper_saddle_pad.obj")
    rail_mesh = mesh_from_geometry(rail_geom, MESH_DIR / "dropper_saddle_rails.obj")
    return shell_mesh, pad_mesh, rail_mesh


def build_object_model() -> ArticulatedObject:
    shell_mesh, pad_mesh, rail_mesh = _build_saddle_meshes()

    model = ArticulatedObject(name="bicycle_dropper_seatpost", assets=ASSETS)

    frame_black = model.material("frame_black", rgba=(0.11, 0.11, 0.12, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.08, 0.08, 0.09, 1.0))
    satin_black = model.material("satin_black", rgba=(0.16, 0.16, 0.17, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    black_chrome = model.material("black_chrome", rgba=(0.14, 0.14, 0.15, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.08, 0.08, 0.08, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    dull_aluminum = model.material("dull_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    bronze = model.material("bronze", rgba=(0.62, 0.52, 0.31, 1.0))

    seat_tube_stub = model.part("seat_tube_stub")
    seat_tube_stub.visual(
        Cylinder(radius=0.0185, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=frame_black,
    )
    seat_tube_stub.visual(
        Cylinder(radius=0.0198, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=frame_black,
    )
    seat_tube_stub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0198, length=0.090),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    outer_body = model.part("outer_body")
    outer_body.visual(
        Cylinder(radius=0.0158, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=anodized_black,
    )
    outer_body.visual(
        Cylinder(radius=0.0174, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=dark_graphite,
    )
    outer_body.visual(
        Cylinder(radius=0.0168, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=anodized_black,
    )
    outer_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0174, length=0.255),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.0625)),
    )

    clamp_collar = model.part("clamp_collar")
    clamp_collar.visual(
        Cylinder(radius=0.0190, length=0.018),
        origin=Origin(),
        material=satin_black,
    )
    clamp_collar.visual(
        Cylinder(radius=0.0178, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0065)),
        material=dark_graphite,
    )
    clamp_collar.visual(
        Box((0.013, 0.011, 0.008)),
        origin=Origin(xyz=(0.0185, -0.004, -0.002)),
        material=satin_black,
    )
    clamp_collar.visual(
        Cylinder(radius=0.0035, length=0.014),
        origin=Origin(xyz=(0.025, -0.004, -0.002), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
    )
    clamp_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0190, length=0.018),
        mass=0.08,
        origin=Origin(),
    )

    stanchion = model.part("stanchion")
    stanchion.visual(
        Cylinder(radius=0.0146, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=black_chrome,
    )
    stanchion.visual(
        Cylinder(radius=0.0151, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=bronze,
    )
    stanchion.visual(
        Cylinder(radius=0.0138, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.206)),
        material=anodized_black,
    )
    stanchion.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0146, length=0.240),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=anodized_black,
    )
    saddle_clamp.visual(
        Box((0.028, 0.038, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=anodized_black,
    )
    for side in (-1.0, 1.0):
        saddle_clamp.visual(
            Box((0.022, 0.028, 0.012)),
            origin=Origin(xyz=(0.020 * side, 0.0, 0.004)),
            material=anodized_black,
        )
        saddle_clamp.visual(
            Box((0.022, 0.048, 0.006)),
            origin=Origin(xyz=(0.032 * side, 0.0, 0.005)),
            material=brushed_steel,
        )
        saddle_clamp.visual(
            Box((0.006, 0.056, 0.024)),
            origin=Origin(xyz=(0.040 * side, 0.0, 0.014)),
            material=brushed_steel,
        )
        saddle_clamp.visual(
            Box((0.022, 0.044, 0.006)),
            origin=Origin(xyz=(0.032 * side, 0.0, 0.0175)),
            material=brushed_steel,
        )
    for y_pos in (-0.018, 0.018):
        saddle_clamp.visual(
            Cylinder(radius=0.0024, length=0.086),
            origin=Origin(xyz=(0.0, y_pos, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=brushed_steel,
        )
        saddle_clamp.visual(
            Cylinder(radius=0.0042, length=0.006),
            origin=Origin(xyz=(-0.046, y_pos, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dull_aluminum,
        )
        saddle_clamp.visual(
            Cylinder(radius=0.0042, length=0.006),
            origin=Origin(xyz=(0.046, y_pos, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dull_aluminum,
        )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.100, 0.060, 0.035)),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    saddle = model.part("saddle")
    saddle.visual(shell_mesh, material=dark_graphite)
    saddle.visual(pad_mesh, material=rubber_black)
    saddle.visual(rail_mesh, material=brushed_steel)
    for x_pos, y_pos, length in (
        (-0.032, 0.072, 0.024),
        (0.032, 0.072, 0.024),
        (-0.029, -0.096, 0.026),
        (0.029, -0.096, 0.026),
    ):
        saddle.visual(
            Cylinder(radius=0.0032, length=length),
            origin=Origin(xyz=(x_pos, y_pos, 0.014)),
            material=brushed_steel,
        )
    saddle.inertial = Inertial.from_geometry(
        Box((0.162, 0.272, 0.050)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    model.articulation(
        "seat_tube_to_outer_body",
        ArticulationType.FIXED,
        parent="seat_tube_stub",
        child="outer_body",
        origin=Origin(),
    )
    model.articulation(
        "outer_body_to_collar",
        ArticulationType.FIXED,
        parent="outer_body",
        child="clamp_collar",
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
    )
    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent="outer_body",
        child="stanchion",
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=0.35,
            lower=0.0,
            upper=0.170,
        ),
    )
    model.articulation(
        "stanchion_to_saddle_clamp",
        ArticulationType.FIXED,
        parent="stanchion",
        child="saddle_clamp",
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )
    model.articulation(
        "clamp_to_saddle",
        ArticulationType.FIXED,
        parent="saddle_clamp",
        child="saddle",
        origin=Origin(xyz=(0.0, 0.0, 0.011), rpy=(math.radians(-2.0), 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "clamp_collar",
        "stanchion",
        reason="The collar intentionally sleeves around the telescoping stanchion at the post wiper/seal.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("seat_tube_stub", "outer_body")
    ctx.expect_aabb_contact("clamp_collar", "outer_body")
    ctx.expect_aabb_contact("saddle_clamp", "stanchion")
    ctx.expect_aabb_contact("saddle", "saddle_clamp")
    ctx.expect_aabb_overlap("seat_tube_stub", "outer_body", axes="xy", min_overlap=0.028)
    ctx.expect_aabb_overlap("outer_body", "stanchion", axes="xy", min_overlap=0.026)
    ctx.expect_aabb_overlap("saddle", "saddle_clamp", axes="xy", min_overlap=0.040)
    ctx.expect_origin_distance("outer_body", "stanchion", axes="xy", max_dist=0.002)
    ctx.expect_origin_distance("saddle_clamp", "stanchion", axes="xy", max_dist=0.003)
    ctx.expect_origin_distance("saddle", "stanchion", axes="xy", max_dist=0.006)
    ctx.expect_joint_motion_axis(
        "dropper_travel",
        "saddle",
        world_axis="z",
        direction="positive",
        min_delta=0.12,
    )

    dropped_saddle_z = ctx.part_world_position("saddle")[2]
    dropped_collar_z = ctx.part_world_position("clamp_collar")[2]
    assert dropped_saddle_z > dropped_collar_z + 0.18

    for travel in (0.0, 0.170):
        with ctx.pose(dropper_travel=travel):
            ctx.expect_aabb_contact("saddle", "saddle_clamp")
            ctx.expect_aabb_contact("saddle_clamp", "stanchion")
            ctx.expect_aabb_overlap("outer_body", "stanchion", axes="xy", min_overlap=0.026)
            ctx.expect_origin_distance("outer_body", "stanchion", axes="xy", max_dist=0.002)
            ctx.expect_origin_distance("saddle_clamp", "stanchion", axes="xy", max_dist=0.003)
            ctx.expect_origin_distance("saddle", "stanchion", axes="xy", max_dist=0.006)

    with ctx.pose(dropper_travel=0.170):
        raised_saddle_z = ctx.part_world_position("saddle")[2]
        raised_clamp_z = ctx.part_world_position("saddle_clamp")[2]
        assert raised_saddle_z > dropped_saddle_z + 0.15
        assert raised_saddle_z > raised_clamp_z

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
