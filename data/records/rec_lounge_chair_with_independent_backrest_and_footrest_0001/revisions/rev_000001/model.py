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
    ExtrudeGeometry,
    Inertial,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_side_loft,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _save_mesh(name, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _rounded_block_mesh(name, width, depth, height, radius):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=8)
    geom = ExtrudeGeometry(profile, height=height, center=True, cap=True, closed=True)
    return _save_mesh(name, geom)


def _seat_cushion_mesh():
    sections = [
        (-0.30, -0.055, 0.040, 0.60),
        (-0.16, -0.068, 0.050, 0.63),
        (0.02, -0.074, 0.056, 0.66),
        (0.19, -0.068, 0.048, 0.64),
        (0.31, -0.050, 0.036, 0.60),
    ]
    geom = superellipse_side_loft(sections, exponents=3.2, segments=56, cap=True, closed=True)
    return _save_mesh("seat_cushion.obj", geom)


def _backrest_mesh():
    sections = [
        (0.00, -0.015, 0.085, 0.60),
        (0.18, -0.020, 0.102, 0.62),
        (0.40, -0.025, 0.122, 0.64),
        (0.62, -0.020, 0.104, 0.60),
        (0.78, -0.015, 0.080, 0.56),
    ]
    geom = superellipse_side_loft(sections, exponents=3.0, segments=56, cap=True, closed=True)
    geom.rotate_x(math.pi / 2.0)
    return _save_mesh("backrest_cushion.obj", geom)


def _footrest_mesh():
    sections = [
        (0.00, -0.045, 0.000, 0.52),
        (0.18, -0.055, 0.012, 0.56),
        (0.40, -0.045, 0.008, 0.54),
    ]
    geom = superellipse_side_loft(sections, exponents=3.0, segments=48, cap=True, closed=True)
    return _save_mesh("footrest_cushion.obj", geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_lounge_chair", assets=ASSETS)

    upholstery = Material("upholstery_taupe", (0.66, 0.61, 0.57, 1.0))
    walnut = Material("walnut_trim", (0.41, 0.27, 0.16, 1.0))
    black_metal = Material("black_metal", (0.13, 0.13, 0.14, 1.0))
    charcoal = Material("charcoal_fabric", (0.22, 0.22, 0.24, 1.0))
    model.materials.extend([upholstery, walnut, black_metal, charcoal])

    seat_shell_mesh = _rounded_block_mesh("seat_shell.obj", 0.78, 0.68, 0.14, 0.085)
    side_cheek_mesh = _rounded_block_mesh("side_cheek.obj", 0.04, 0.76, 0.48, 0.012)
    arm_pad_mesh = _rounded_block_mesh("arm_pad.obj", 0.11, 0.64, 0.09, 0.025)
    seat_cushion_mesh = _seat_cushion_mesh()
    head_pillow_mesh = _rounded_block_mesh("head_pillow.obj", 0.46, 0.12, 0.10, 0.030)
    backrest_cushion_mesh = _backrest_mesh()
    footrest_cushion_mesh = _footrest_mesh()

    base = model.part("base")
    base.visual(
        side_cheek_mesh,
        origin=Origin(xyz=(-0.39, 0.0, 0.25)),
        material=walnut,
        name="left_side_cheek",
    )
    base.visual(
        side_cheek_mesh,
        origin=Origin(xyz=(0.39, 0.0, 0.25)),
        material=walnut,
        name="right_side_cheek",
    )
    base.visual(
        seat_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=upholstery,
        name="seat_shell",
    )
    base.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.01, 0.445), rpy=(-0.05, 0.0, 0.0)),
        material=upholstery,
        name="seat_cushion",
    )
    base.visual(
        arm_pad_mesh,
        origin=Origin(xyz=(-0.33, 0.0, 0.525)),
        material=upholstery,
        name="left_arm_pad",
    )
    base.visual(
        arm_pad_mesh,
        origin=Origin(xyz=(0.33, 0.0, 0.525)),
        material=upholstery,
        name="right_arm_pad",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.78),
        origin=Origin(xyz=(0.0, 0.255, 0.11), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="front_stretcher",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.78),
        origin=Origin(xyz=(0.0, -0.255, 0.11), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="rear_stretcher",
    )
    base.visual(
        Box((0.60, 0.06, 0.07)),
        origin=Origin(xyz=(0.0, 0.315, 0.2525)),
        material=black_metal,
        name="footrest_hinge_beam",
    )
    base.visual(
        Box((0.60, 0.06, 0.06)),
        origin=Origin(xyz=(0.0, -0.315, 0.42)),
        material=black_metal,
        name="backrest_mount_beam",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.88, 0.76, 0.58)),
        mass=34.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.018, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="backrest_hinge_tube",
    )
    backrest.visual(
        Box((0.05, 0.045, 0.11)),
        origin=Origin(xyz=(-0.23, -0.025, 0.07)),
        material=black_metal,
        name="left_backrest_bracket",
    )
    backrest.visual(
        Box((0.05, 0.045, 0.11)),
        origin=Origin(xyz=(0.23, -0.025, 0.07)),
        material=black_metal,
        name="right_backrest_bracket",
    )
    backrest.visual(
        backrest_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.06), rpy=(0.09, 0.0, 0.0)),
        material=upholstery,
        name="backrest_cushion",
    )
    backrest.visual(
        Box((0.54, 0.025, 0.56)),
        origin=Origin(xyz=(0.0, -0.078, 0.345), rpy=(0.09, 0.0, 0.0)),
        material=charcoal,
        name="back_panel",
    )
    backrest.visual(
        head_pillow_mesh,
        origin=Origin(xyz=(0.0, -0.01, 0.72), rpy=(0.09, 0.0, 0.0)),
        material=upholstery,
        name="head_pillow",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.66, 0.18, 0.82)),
        mass=10.0,
        origin=Origin(xyz=(0.0, -0.055, 0.40)),
    )

    footrest = model.part("footrest")
    footrest.visual(
        Cylinder(radius=0.014, length=0.44),
        origin=Origin(xyz=(0.0, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_metal,
        name="footrest_hinge_tube",
    )
    footrest.visual(
        footrest_cushion_mesh,
        origin=Origin(xyz=(0.0, 0.11, -0.005)),
        material=upholstery,
        name="footrest_cushion",
    )
    footrest.visual(
        Box((0.48, 0.32, 0.018)),
        origin=Origin(xyz=(0.0, 0.21, -0.038)),
        material=charcoal,
        name="footrest_support_panel",
    )
    footrest.visual(
        Box((0.025, 0.18, 0.06)),
        origin=Origin(xyz=(-0.22, 0.09, -0.015)),
        material=black_metal,
        name="left_footrest_link",
    )
    footrest.visual(
        Box((0.025, 0.18, 0.06)),
        origin=Origin(xyz=(0.22, 0.09, -0.015)),
        material=black_metal,
        name="right_footrest_link",
    )
    footrest.inertial = Inertial.from_geometry(
        Box((0.56, 0.42, 0.11)),
        mass=5.0,
        origin=Origin(xyz=(0.0, 0.22, -0.05)),
    )

    model.articulation(
        "backrest_hinge",
        ArticulationType.REVOLUTE,
        parent="base",
        child="backrest",
        origin=Origin(xyz=(0.0, -0.325, 0.455)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=0.58,
        ),
    )
    model.articulation(
        "footrest_hinge",
        ArticulationType.REVOLUTE,
        parent="base",
        child="footrest",
        origin=Origin(xyz=(0.0, 0.315, 0.245)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
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
        "backrest",
        "base",
        reason="the upholstered back nests tightly into the rear recline cradle, and generated convex hulls are conservative around the pivot seam",
    )
    ctx.allow_overlap(
        "base",
        "footrest",
        reason="the raised ottoman panel folds tightly beneath the front apron, so collision hulls around the upholstered pad and hinge rails are intentionally conservative near the nesting seam",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.004, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("backrest", "base", axes="xy", min_overlap=0.05)
    ctx.expect_origin_distance("backrest", "base", axes="xy", max_dist=0.42)
    ctx.expect_aabb_overlap("footrest", "base", axes="xy", min_overlap=0.02)
    ctx.expect_origin_distance("footrest", "base", axes="xy", max_dist=0.38)
    ctx.expect_joint_motion_axis(
        "backrest_hinge",
        "backrest",
        world_axis="y",
        direction="negative",
        min_delta=0.10,
    )
    ctx.expect_joint_motion_axis(
        "footrest_hinge",
        "footrest",
        world_axis="z",
        direction="positive",
        min_delta=0.12,
    )

    with ctx.pose(backrest_hinge=0.58):
        ctx.expect_aabb_overlap("backrest", "base", axes="xy", min_overlap=0.04)
        ctx.expect_origin_distance("backrest", "base", axes="xy", max_dist=0.56)

    with ctx.pose(footrest_hinge=1.05):
        ctx.expect_aabb_overlap("footrest", "base", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("footrest", "base", axes="xy", max_dist=0.56)

    with ctx.pose(backrest_hinge=0.58, footrest_hinge=1.05):
        ctx.expect_aabb_overlap("backrest", "base", axes="xy", min_overlap=0.03)
        ctx.expect_aabb_overlap("footrest", "base", axes="xy", min_overlap=0.015)
        ctx.expect_origin_distance("backrest", "footrest", axes="xy", max_dist=0.90)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
