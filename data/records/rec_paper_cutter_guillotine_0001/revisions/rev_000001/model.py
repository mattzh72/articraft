from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.

# >>> USER_CODE_START
import math

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

BASE_WIDTH = 0.42
BASE_DEPTH = 0.31
FOOT_HEIGHT = 0.003
BASE_THICKNESS = 0.018
BASE_TOP_Z = FOOT_HEIGHT + BASE_THICKNESS
SURFACE_THICKNESS = 0.0006
MARKING_THICKNESS = 0.0003
FIXED_BLADE_THICKNESS = 0.0012
ARM_YAW = -0.67
ARM_LENGTH = 0.445
HINGE_POS = (-0.172, 0.112, 0.028)
FENCE_Y = 0.147


def _build_base_panel_mesh():
    profile = rounded_rect_profile(
        BASE_WIDTH,
        BASE_DEPTH,
        radius=0.018,
        corner_segments=10,
    )
    geom = ExtrudeGeometry.from_z0(profile, BASE_THICKNESS).translate(0.0, 0.0, FOOT_HEIGHT)
    return mesh_from_geometry(geom, ASSETS.mesh_dir / "paper_cutter_base_panel.obj")


def _build_arm_beam_mesh():
    profile = [
        (0.000, -0.022),
        (0.040, -0.024),
        (0.155, -0.019),
        (0.285, -0.016),
        (0.360, -0.014),
        (0.390, -0.024),
        (0.432, -0.024),
        (0.447, 0.000),
        (0.432, 0.024),
        (0.390, 0.024),
        (0.360, 0.014),
        (0.285, 0.016),
        (0.155, 0.019),
        (0.040, 0.024),
        (0.000, 0.022),
    ]
    geom = ExtrudeGeometry.centered(profile, 0.012).translate(0.0, 0.0, 0.0025)
    return mesh_from_geometry(geom, ASSETS.mesh_dir / "paper_cutter_arm_beam.obj")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="paper_cutter_guillotine", assets=ASSETS)

    laminate = model.material("laminate_board", rgba=(0.24, 0.25, 0.28, 1.0))
    cutting_mat = model.material("cutting_mat", rgba=(0.15, 0.18, 0.20, 1.0))
    measurement = model.material("measurement_marking", rgba=(0.86, 0.87, 0.88, 1.0))
    anodized = model.material("anodized_aluminum", rgba=(0.67, 0.69, 0.72, 1.0))
    steel = model.material("brushed_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    clear_guard = model.material("clear_guard", rgba=(0.78, 0.88, 0.94, 0.30))
    accent_red = model.material("accent_red", rgba=(0.76, 0.11, 0.11, 1.0))

    base = model.part("base")
    base.visual(_build_base_panel_mesh(), material=laminate, name="base_panel")
    base.visual(
        Box((0.376, 0.262, SURFACE_THICKNESS)),
        origin=Origin(xyz=(0.010, -0.004, BASE_TOP_Z + SURFACE_THICKNESS / 2.0)),
        material=cutting_mat,
        name="cutting_mat_surface",
    )
    base.visual(
        Box((0.372, 0.018, MARKING_THICKNESS)),
        origin=Origin(xyz=(0.008, 0.121, BASE_TOP_Z + MARKING_THICKNESS / 2.0)),
        material=measurement,
        name="top_ruler_strip",
    )
    base.visual(
        Box((0.018, 0.234, MARKING_THICKNESS)),
        origin=Origin(xyz=(-0.160, 0.002, BASE_TOP_Z + MARKING_THICKNESS / 2.0)),
        material=measurement,
        name="side_ruler_strip",
    )
    base.visual(
        Box((0.318, 0.012, FIXED_BLADE_THICKNESS)),
        origin=Origin(
            xyz=(-0.020, -0.011, BASE_TOP_Z + FIXED_BLADE_THICKNESS / 2.0),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=steel,
        name="cutting_edge_bar",
    )
    base.visual(
        Box((0.318, 0.0022, MARKING_THICKNESS)),
        origin=Origin(
            xyz=(-0.020, -0.011, BASE_TOP_Z + MARKING_THICKNESS / 2.0),
            rpy=(0.0, 0.0, ARM_YAW),
        ),
        material=accent_red,
        name="cut_line_indicator",
    )
    base.visual(
        Box((0.026, 0.036, 0.020)),
        origin=Origin(xyz=(-0.172, 0.119, BASE_TOP_Z + 0.010)),
        material=steel,
        name="hinge_mount_block",
    )
    for sx in (-0.158, 0.158):
        for sy in (-0.108, 0.108):
            base.visual(
                Box((0.024, 0.024, FOOT_HEIGHT)),
                origin=Origin(xyz=(sx, sy, FOOT_HEIGHT / 2.0)),
                material=rubber,
                name=f"foot_{'r' if sx > 0 else 'l'}_{'f' if sy < 0 else 'b'}",
            )
    base.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, BASE_THICKNESS + FOOT_HEIGHT)),
        mass=3.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + FOOT_HEIGHT) / 2.0)),
    )

    guide_fence = model.part("guide_fence")
    guide_fence.visual(
        Box((0.340, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=anodized,
        name="rear_fence_bar",
    )
    guide_fence.visual(
        Box((0.340, 0.005, 0.028)),
        origin=Origin(xyz=(0.0, 0.005, 0.014)),
        material=steel,
        name="rear_fence_lip",
    )
    guide_fence.visual(
        Box((0.016, 0.052, 0.020)),
        origin=Origin(xyz=(0.154, -0.023, 0.010)),
        material=anodized,
        name="squaring_stop",
    )
    guide_fence.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.120, 0.003, 0.016), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="fence_adjustment_knob",
    )
    guide_fence.inertial = Inertial.from_geometry(
        Box((0.340, 0.060, 0.028)),
        mass=0.35,
        origin=Origin(xyz=(0.0, -0.010, 0.014)),
    )

    cutting_arm = model.part("cutting_arm")
    cutting_arm.visual(_build_arm_beam_mesh(), material=anodized, name="arm_beam")
    cutting_arm.visual(
        Cylinder(radius=0.015, length=0.038),
        origin=Origin(xyz=(0.018, 0.0, 0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_hub",
    )
    cutting_arm.visual(
        Box((0.308, 0.008, 0.003)),
        origin=Origin(xyz=(0.205, -0.011, -0.0045)),
        material=steel,
        name="blade_backing_strip",
    )
    cutting_arm.visual(
        Box((0.230, 0.003, 0.032)),
        origin=Origin(xyz=(0.190, 0.014, 0.010)),
        material=clear_guard,
        name="finger_guard",
    )
    cutting_arm.visual(
        Box((0.008, 0.008, 0.020)),
        origin=Origin(xyz=(0.105, 0.010, 0.004)),
        material=steel,
        name="guard_post_rear",
    )
    cutting_arm.visual(
        Box((0.008, 0.008, 0.020)),
        origin=Origin(xyz=(0.255, 0.010, 0.004)),
        material=steel,
        name="guard_post_front",
    )
    cutting_arm.visual(
        Cylinder(radius=0.014, length=0.085),
        origin=Origin(xyz=(0.392, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle_grip",
    )
    cutting_arm.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.434, 0.0, 0.012)),
        material=black_plastic,
        name="handle_end_cap",
    )
    cutting_arm.inertial = Inertial.from_geometry(
        Box((ARM_LENGTH, 0.052, 0.040)),
        mass=0.85,
        origin=Origin(xyz=(0.222, 0.0, 0.012)),
    )

    model.articulation(
        "base_to_guide_fence",
        ArticulationType.FIXED,
        parent="base",
        child="guide_fence",
        origin=Origin(xyz=(0.0, FENCE_Y, BASE_TOP_Z + 0.001)),
    )
    model.articulation(
        "base_to_cutting_arm",
        ArticulationType.REVOLUTE,
        parent="base",
        child="cutting_arm",
        origin=Origin(xyz=HINGE_POS, rpy=(0.0, 0.0, ARM_YAW)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.6,
            lower=0.0,
            upper=1.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.015)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.warn_if_coplanar_surfaces(use="visual")
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_contact("guide_fence", "base")
    ctx.expect_aabb_overlap("guide_fence", "base", axes="x", min_overlap=0.300)
    ctx.expect_aabb_overlap("guide_fence", "base", axes="y", min_overlap=0.018)

    ctx.expect_aabb_contact("cutting_arm", "base")
    ctx.expect_aabb_overlap("cutting_arm", "base", axes="xy", min_overlap=0.130)
    ctx.expect_joint_motion_axis(
        "base_to_cutting_arm",
        "cutting_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.050,
    )

    with ctx.pose(base_to_cutting_arm=1.15):
        ctx.expect_aabb_overlap("cutting_arm", "base", axes="x", min_overlap=0.020)
        ctx.expect_aabb_overlap("cutting_arm", "base", axes="y", min_overlap=0.020)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
