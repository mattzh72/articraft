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
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

CEDAR = Material(name="cedar", rgba=(0.60, 0.43, 0.27, 1.0))
AGED_WOOD = Material(name="aged_wood", rgba=(0.53, 0.37, 0.24, 1.0))
BLACK_STEEL = Material(name="black_steel", rgba=(0.12, 0.12, 0.13, 1.0))
GALVANIZED = Material(name="galvanized", rgba=(0.63, 0.65, 0.67, 1.0))
CONCRETE = Material(name="concrete", rgba=(0.53, 0.54, 0.52, 1.0))


def _handle_mesh():
    handle_geom = tube_from_spline_points(
        [
            (0.0, 0.0, -0.035),
            (0.0, 0.018, -0.046),
            (0.0, 0.040, -0.024),
            (0.0, 0.048, 0.0),
            (0.0, 0.040, 0.024),
            (0.0, 0.018, 0.046),
            (0.0, 0.0, 0.035),
        ],
        radius=0.004,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    return mesh_from_geometry(handle_geom, ASSETS.mesh_path("gate_latch_handle.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="garden_gate", assets=ASSETS)

    handle_mesh = _handle_mesh()

    hinge_assembly = model.part("hinge_assembly")
    hinge_assembly.visual(
        Box((0.12, 0.12, 1.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=AGED_WOOD,
    )
    hinge_assembly.visual(
        Box((0.14, 0.14, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 1.565)),
        material=AGED_WOOD,
    )
    hinge_assembly.visual(
        Box((0.10, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 1.595)),
        material=AGED_WOOD,
    )
    hinge_assembly.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.628)),
        material=GALVANIZED,
    )
    for rail_z in (0.30, 0.80):
        hinge_assembly.visual(
            Box((0.48, 0.028, 0.03)),
            origin=Origin(xyz=(-0.30, 0.0, rail_z)),
            material=AGED_WOOD,
        )
    for picket_x in (-0.14, -0.26, -0.38, -0.50):
        hinge_assembly.visual(
            Box((0.08, 0.022, 0.92)),
            origin=Origin(xyz=(picket_x, 0.0, 0.56)),
            material=AGED_WOOD,
        )
        hinge_assembly.visual(
            Box((0.05, 0.022, 0.05)),
            origin=Origin(xyz=(picket_x, 0.0, 1.045)),
            material=AGED_WOOD,
        )
    for hinge_z in (0.20, 0.80):
        hinge_assembly.visual(
            Box((0.072, 0.020, 0.12)),
            origin=Origin(xyz=(0.090, 0.0, hinge_z)),
            material=BLACK_STEEL,
        )
        hinge_assembly.visual(
            Cylinder(radius=0.004, length=0.072),
            origin=Origin(xyz=(0.094, 0.0, hinge_z)),
            material=GALVANIZED,
        )
    hinge_assembly.inertial = Inertial.from_geometry(
        Box((0.62, 0.14, 1.60)),
        mass=18.0,
        origin=Origin(xyz=(-0.25, 0.0, 0.80)),
    )

    threshold = model.part("threshold")
    threshold.visual(
        Box((1.12, 0.16, 0.08)),
        origin=Origin(xyz=(0.56, 0.0, -0.04)),
        material=CONCRETE,
    )
    threshold.visual(
        Box((1.06, 0.11, 0.025)),
        origin=Origin(xyz=(0.56, 0.0, -0.0125)),
        material=GALVANIZED,
    )
    threshold.inertial = Inertial.from_geometry(
        Box((1.12, 0.16, 0.08)),
        mass=12.0,
        origin=Origin(xyz=(0.56, 0.0, -0.04)),
    )

    latch_post = model.part("latch_post")
    latch_post.visual(
        Box((0.12, 0.12, 1.55)),
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
        material=AGED_WOOD,
    )
    latch_post.visual(
        Box((0.14, 0.14, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 1.565)),
        material=AGED_WOOD,
    )
    latch_post.visual(
        Box((0.10, 0.10, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 1.595)),
        material=AGED_WOOD,
    )
    latch_post.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.628)),
        material=GALVANIZED,
    )
    latch_post.visual(
        Box((0.012, 0.05, 0.16)),
        origin=Origin(xyz=(-0.054, 0.0, 0.55)),
        material=BLACK_STEEL,
    )
    latch_post.visual(
        Box((0.03, 0.03, 0.08)),
        origin=Origin(xyz=(-0.044, 0.0, 0.55)),
        material=GALVANIZED,
    )
    latch_post.visual(
        Cylinder(radius=0.005, length=0.07),
        origin=Origin(xyz=(-0.03, 0.0, 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GALVANIZED,
    )
    latch_post.inertial = Inertial.from_geometry(
        Box((0.12, 0.12, 1.55)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.775)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.05, 0.036, 0.98)),
        origin=Origin(xyz=(0.030, 0.0, 0.35)),
        material=CEDAR,
    )
    gate_leaf.visual(
        Box((0.05, 0.036, 0.98)),
        origin=Origin(xyz=(0.900, 0.0, 0.35)),
        material=CEDAR,
    )
    gate_leaf.visual(
        Box((0.82, 0.032, 0.11)),
        origin=Origin(xyz=(0.465, 0.0, -0.085)),
        material=CEDAR,
    )
    gate_leaf.visual(
        Box((0.82, 0.032, 0.08)),
        origin=Origin(xyz=(0.465, 0.0, 0.28)),
        material=CEDAR,
    )
    gate_leaf.visual(
        Box((0.82, 0.032, 0.11)),
        origin=Origin(xyz=(0.465, 0.0, 0.58)),
        material=CEDAR,
    )
    brace_dx = 0.68
    brace_dz = 0.55
    gate_leaf.visual(
        Box((math.hypot(brace_dx, brace_dz), 0.024, 0.05)),
        origin=Origin(
            xyz=(0.46, 0.0, 0.205),
            rpy=(0.0, -math.atan2(brace_dz, brace_dx), 0.0),
        ),
        material=AGED_WOOD,
    )
    picket_xs = (0.15, 0.27, 0.39, 0.51, 0.63, 0.75, 0.87)
    picket_heights = (0.94, 1.00, 1.06, 1.10, 1.06, 1.00, 0.94)
    for picket_x, height in zip(picket_xs, picket_heights):
        gate_leaf.visual(
            Box((0.08, 0.022, height)),
            origin=Origin(xyz=(picket_x, 0.0, -0.12 + (height / 2.0))),
            material=CEDAR,
        )
        gate_leaf.visual(
            Box((0.044, 0.022, 0.05)),
            origin=Origin(xyz=(picket_x, 0.0, height - 0.095)),
            material=CEDAR,
        )
    for hinge_z in (0.0, 0.60):
        gate_leaf.visual(
            Box((0.26, 0.008, 0.10)),
            origin=Origin(xyz=(0.13, -0.016, hinge_z)),
            material=BLACK_STEEL,
        )
        gate_leaf.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(
                xyz=(0.070, -0.016, hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=GALVANIZED,
        )
        gate_leaf.visual(
            Cylinder(radius=0.006, length=0.010),
            origin=Origin(
                xyz=(0.180, -0.016, hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=GALVANIZED,
        )
    gate_leaf.visual(
        Box((0.11, 0.006, 0.12)),
        origin=Origin(xyz=(0.855, 0.022, 0.34)),
        material=BLACK_STEEL,
    )
    gate_leaf.visual(
        Box((0.10, 0.028, 0.07)),
        origin=Origin(xyz=(0.865, 0.0, 0.34)),
        material=BLACK_STEEL,
    )
    gate_leaf.visual(
        Cylinder(radius=0.008, length=0.09),
        origin=Origin(xyz=(0.905, 0.0, 0.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=GALVANIZED,
    )
    gate_leaf.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(xyz=(0.845, 0.016, 0.34), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GALVANIZED,
    )
    gate_leaf.visual(
        handle_mesh,
        origin=Origin(xyz=(0.845, 0.016, 0.34)),
        material=BLACK_STEEL,
    )
    for bolt_x in (0.065, 0.155, 0.815, 0.895):
        for bolt_z in (0.0, 0.60, 0.31):
            if bolt_z == 0.31 and bolt_x < 0.80:
                continue
            gate_leaf.visual(
                Cylinder(radius=0.005, length=0.042),
                origin=Origin(
                    xyz=(bolt_x, 0.0, bolt_z),
                    rpy=(math.pi / 2.0, 0.0, 0.0),
                ),
                material=GALVANIZED,
            )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((0.94, 0.05, 1.12)),
        mass=14.0,
        origin=Origin(xyz=(0.47, 0.0, 0.40)),
    )

    model.articulation(
        "hinge_to_threshold",
        ArticulationType.FIXED,
        parent="hinge_assembly",
        child="threshold",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "threshold_to_latch_post",
        ArticulationType.FIXED,
        parent="threshold",
        child="latch_post",
        origin=Origin(xyz=(1.125, 0.0, 0.0)),
    )
    model.articulation(
        "gate_swing",
        ArticulationType.REVOLUTE,
        parent="hinge_assembly",
        child="gate_leaf",
        origin=Origin(xyz=(0.094, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.0,
            lower=0.0,
            upper=1.45,
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
        "gate_leaf",
        "hinge_assembly",
        reason="strap-hinge pivot hardware sits tightly around the swing axis and can trip conservative collision overlap checks near the hinge barrels",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("hinge_assembly", "threshold", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_gap("hinge_assembly", "threshold", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_overlap("latch_post", "threshold", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_gap("latch_post", "threshold", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_overlap("gate_leaf", "threshold", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_gap("gate_leaf", "threshold", axis="z", max_gap=0.07, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "gate_swing",
        "gate_leaf",
        world_axis="y",
        direction="positive",
        min_delta=0.20,
    )

    with ctx.pose(gate_swing=0.75):
        ctx.expect_aabb_gap("gate_leaf", "threshold", axis="z", max_gap=0.07, max_penetration=0.0)
        ctx.expect_aabb_overlap("gate_leaf", "threshold", axes="xy", min_overlap=0.02)

    with ctx.pose(gate_swing=1.45):
        ctx.expect_aabb_gap("gate_leaf", "threshold", axis="z", max_gap=0.07, max_penetration=0.0)
        ctx.expect_aabb_overlap("gate_leaf", "threshold", axes="xy", min_overlap=0.02)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
