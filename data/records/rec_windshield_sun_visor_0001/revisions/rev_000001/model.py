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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
    sweep_profile_along_spline,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _visor_panel_mesh():
    panel_geom = superellipse_side_loft(
        [
            (0.058, -0.0268, -0.0115, 0.110),
            (0.110, -0.0278, -0.0108, 0.136),
            (0.200, -0.0283, -0.0105, 0.146),
            (0.300, -0.0278, -0.0108, 0.140),
            (0.352, -0.0260, -0.0112, 0.120),
        ],
        exponents=(3.0, 3.0, 2.9, 3.0, 3.1),
        segments=56,
        cap=True,
        closed=True,
    )
    panel_geom.translate(0.078, 0.0, 0.0105)
    return mesh_from_geometry(panel_geom, ASSETS.mesh_path("visor_panel.obj"))


def _support_rod_mesh():
    rod_geom = tube_from_spline_points(
        [
            (0.000, 0.000, -0.0015),
            (0.000, 0.020, -0.0015),
            (0.003, 0.038, -0.0024),
            (0.007, 0.051, -0.0036),
            (0.011, 0.060, -0.0044),
        ],
        radius=0.0034,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(rod_geom, ASSETS.mesh_path("visor_support_rod.obj"))


def _clip_hook_mesh():
    hook_geom = sweep_profile_along_spline(
        [
            (0.000, 0.000, -0.004),
            (0.004, 0.000, -0.006),
            (0.006, 0.000, -0.010),
            (0.005, 0.000, -0.014),
            (0.002, 0.000, -0.017),
        ],
        profile=rounded_rect_profile(0.008, 0.004, radius=0.001),
        samples_per_segment=18,
        cap_profile=True,
    )
    return mesh_from_geometry(hook_geom, ASSETS.mesh_path("visor_retaining_clip.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_sun_visor_assembly", assets=ASSETS)

    headliner = model.material("headliner_fabric", rgba=(0.72, 0.72, 0.70, 1.0))
    visor_skin = model.material("visor_vinyl", rgba=(0.63, 0.64, 0.66, 1.0))
    trim = model.material("textured_trim", rgba=(0.26, 0.27, 0.29, 1.0))
    clip_plastic = model.material("clip_plastic", rgba=(0.18, 0.18, 0.19, 1.0))
    label = model.material("warning_label", rgba=(0.88, 0.86, 0.80, 1.0))
    steel = model.material("brushed_steel", rgba=(0.66, 0.68, 0.71, 1.0))

    roof_mount = model.part("roof_mount")
    roof_mount.visual(
        Box((0.070, 0.370, 0.006)),
        origin=Origin(xyz=(0.028, 0.185, -0.003)),
        material=headliner,
    )
    roof_mount.visual(
        Box((0.084, 0.070, 0.012)),
        origin=Origin(xyz=(0.020, 0.028, -0.006)),
        material=trim,
    )
    roof_mount.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(xyz=(0.018, 0.024, -0.013)),
        material=trim,
    )
    roof_mount.visual(
        Cylinder(radius=0.011, length=0.005),
        origin=Origin(xyz=(0.018, 0.024, -0.0215)),
        material=trim,
    )
    roof_mount.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.036, 0.110, -0.0055)),
        material=headliner,
    )
    roof_mount.visual(
        Cylinder(radius=0.006, length=0.002),
        origin=Origin(xyz=(0.036, 0.286, -0.0055)),
        material=headliner,
    )
    roof_mount.inertial = Inertial.from_geometry(
        Box((0.090, 0.370, 0.024)),
        mass=0.55,
        origin=Origin(xyz=(0.028, 0.185, -0.010)),
    )

    swivel_hub = model.part("swivel_hub")
    swivel_hub.visual(
        Cylinder(radius=0.007, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        material=steel,
    )
    swivel_hub.visual(
        Cylinder(radius=0.012, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=trim,
    )
    swivel_hub.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.0, 0.014, -0.008), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=steel,
    )
    swivel_hub.visual(
        Box((0.014, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.010, -0.008)),
        material=trim,
    )
    swivel_hub.inertial = Inertial.from_geometry(
        Box((0.020, 0.035, 0.020)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.010, -0.008)),
    )

    visor = model.part("visor")
    visor.visual(_support_rod_mesh(), material=steel)
    visor.visual(_visor_panel_mesh(), material=visor_skin)
    visor.visual(
        Box((0.024, 0.038, 0.0028)),
        origin=Origin(xyz=(0.018, 0.082, -0.0006)),
        material=trim,
    )
    visor.visual(
        Box((0.050, 0.095, 0.0012)),
        origin=Origin(xyz=(0.082, 0.235, -0.0179)),
        material=label,
    )
    visor.visual(
        Box((0.014, 0.020, 0.004)),
        origin=Origin(xyz=(0.030, 0.349, -0.0008)),
        material=trim,
    )
    visor.inertial = Inertial.from_geometry(
        Box((0.145, 0.315, 0.020)),
        mass=0.72,
        origin=Origin(xyz=(0.082, 0.210, -0.010)),
    )

    retaining_clip = model.part("retaining_clip")
    retaining_clip.visual(
        Box((0.026, 0.022, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=clip_plastic,
    )
    retaining_clip.visual(_clip_hook_mesh(), material=clip_plastic)
    retaining_clip.inertial = Inertial.from_geometry(
        Box((0.028, 0.024, 0.020)),
        mass=0.03,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    model.articulation(
        "visor_swivel",
        ArticulationType.REVOLUTE,
        parent="roof_mount",
        child="swivel_hub",
        origin=Origin(xyz=(0.018, 0.024, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.65,
        ),
    )
    model.articulation(
        "visor_tilt",
        ArticulationType.REVOLUTE,
        parent="swivel_hub",
        child="visor",
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=3.0,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "clip_mount",
        ArticulationType.FIXED,
        parent="roof_mount",
        child="retaining_clip",
        origin=Origin(xyz=(0.030, 0.347, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "retaining_clip",
        "visor",
        reason="the spring clip lightly envelopes the visor tip in the stowed position",
    )
    ctx.allow_overlap(
        "roof_mount",
        "visor",
        reason="the visor's steel support nests closely beneath the roof pivot escutcheon at full drop",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("swivel_hub", "roof_mount")
    ctx.expect_aabb_contact("swivel_hub", "visor")
    ctx.expect_aabb_contact("retaining_clip", "roof_mount")
    ctx.expect_aabb_contact("retaining_clip", "visor")
    ctx.expect_aabb_overlap("visor", "roof_mount", axes="xy", min_overlap=0.015)
    ctx.expect_joint_motion_axis(
        "visor_tilt",
        "visor",
        world_axis="z",
        direction="negative",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "visor_swivel",
        "visor",
        world_axis="x",
        direction="negative",
        min_delta=0.08,
    )

    with ctx.pose(visor_tilt=1.35):
        ctx.expect_aabb_contact("swivel_hub", "visor")

    with ctx.pose(visor_swivel=1.55):
        ctx.expect_aabb_contact("swivel_hub", "visor")
        ctx.expect_aabb_gap("retaining_clip", "visor", axis="y", max_gap=0.25, max_penetration=0.0)

    with ctx.pose(visor_swivel=1.55, visor_tilt=1.10):
        ctx.expect_aabb_contact("swivel_hub", "visor")
        ctx.expect_aabb_gap("retaining_clip", "visor", axis="y", max_gap=0.35, max_penetration=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
