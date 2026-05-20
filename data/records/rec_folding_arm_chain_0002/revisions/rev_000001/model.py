from __future__ import annotations

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
BAR_THICKNESS = 0.012
BAR_WEB_WIDTH = 0.028
EYELET_RADIUS = 0.024
HOLE_RADIUS = 0.010
COLLISION_EYELET_RADIUS = 0.016
ARM_SPANS = (0.24, 0.20, 0.17, 0.14)

BASE_LAYER_Z = -0.008
ARM_LAYER_Z = (0.008, -0.008, 0.008, -0.008)
TOOL_LAYER_Z = 0.008


def _make_base_mount_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").center(-0.070, 0.0).box(0.130, 0.080, 0.016)
    neck = cq.Workplane("XY").center(-0.024, 0.0).rect(0.052, 0.048).extrude(0.018)
    boss = cq.Workplane("XY").circle(0.028).extrude(0.018)
    shape = foot.union(neck).union(boss)
    shape = shape.faces(">Z").workplane().pushPoints([(0.0, 0.0)]).hole(2.0 * HOLE_RADIUS)
    shape = (
        shape.faces(">Z")
        .workplane()
        .pushPoints([(-0.102, -0.022), (-0.102, 0.022), (-0.046, -0.022), (-0.046, 0.022)])
        .hole(0.008)
    )
    return shape


def _make_flat_bar_shape(span: float) -> cq.Workplane:
    eyelets = (
        cq.Workplane("XY")
        .pushPoints([(0.0, 0.0), (span, 0.0)])
        .circle(EYELET_RADIUS)
        .extrude(BAR_THICKNESS)
    )
    web = (
        cq.Workplane("XY").center(span / 2.0, 0.0).rect(span, BAR_WEB_WIDTH).extrude(BAR_THICKNESS)
    )
    shape = eyelets.union(web)

    cutout_len = span - 0.11
    if cutout_len > 0.040:
        shape = (
            shape.faces(">Z")
            .workplane()
            .center(span / 2.0, 0.0)
            .slot2D(cutout_len, BAR_WEB_WIDTH * 0.54)
            .cutThruAll()
        )

    shape = (
        shape.faces(">Z").workplane().pushPoints([(0.0, 0.0), (span, 0.0)]).hole(2.0 * HOLE_RADIUS)
    )
    return shape


def _make_tool_plate_shape() -> cq.Workplane:
    lug = cq.Workplane("XY").circle(EYELET_RADIUS).extrude(BAR_THICKNESS)
    plate = cq.Workplane("XY").center(0.052, 0.0).rect(0.084, 0.056).extrude(BAR_THICKNESS)
    nose = cq.Workplane("XY").center(0.094, 0.0).circle(0.018).extrude(BAR_THICKNESS)
    shape = lug.union(plate).union(nose)
    shape = shape.faces(">Z").workplane().pushPoints([(0.0, 0.0)]).hole(2.0 * HOLE_RADIUS)
    shape = shape.faces(">Z").workplane().center(0.056, 0.0).slot2D(0.032, 0.012).cutThruAll()
    shape = shape.faces(">Z").workplane().pushPoints([(0.084, -0.017), (0.084, 0.017)]).hole(0.006)
    return shape


def _add_flat_bar_part(
    model: ArticulatedObject,
    name: str,
    span: float,
    z_layer: float,
    material_name: str,
):
    part = model.part(name)
    mesh = mesh_from_cadquery(_make_flat_bar_shape(span), f"{name}.obj", assets=ASSETS)
    part.visual(mesh, origin=Origin(xyz=(0.0, 0.0, z_layer)), material=material_name)

    body_len = max(0.040, span - (2.0 * EYELET_RADIUS))



    part.inertial = Inertial.from_geometry(
        Box((span, 2.0 * EYELET_RADIUS, BAR_THICKNESS)),
        mass=0.95 * span,
        origin=Origin(xyz=(span / 2.0, 0.0, z_layer)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain", assets=ASSETS)

    model.material("gunmetal", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("safety_orange", rgba=(0.91, 0.45, 0.12, 1.0))
    model.material("machine_black", rgba=(0.12, 0.13, 0.14, 1.0))

    base = model.part("base")
    base_mesh = mesh_from_cadquery(_make_base_mount_shape(), "base_mount.obj", assets=ASSETS)
    base.visual(base_mesh, origin=Origin(xyz=(0.0, 0.0, BASE_LAYER_Z)), material="gunmetal")



    base.inertial = Inertial.from_geometry(
        Box((0.130, 0.080, 0.018)),
        mass=2.4,
        origin=Origin(xyz=(-0.062, 0.0, BASE_LAYER_Z)),
    )

    arm_1 = _add_flat_bar_part(model, "arm_1", ARM_SPANS[0], ARM_LAYER_Z[0], "safety_orange")
    arm_2 = _add_flat_bar_part(model, "arm_2", ARM_SPANS[1], ARM_LAYER_Z[1], "gunmetal")
    arm_3 = _add_flat_bar_part(model, "arm_3", ARM_SPANS[2], ARM_LAYER_Z[2], "safety_orange")
    arm_4 = _add_flat_bar_part(model, "arm_4", ARM_SPANS[3], ARM_LAYER_Z[3], "gunmetal")

    tool_plate = model.part("tool_plate")
    tool_mesh = mesh_from_cadquery(_make_tool_plate_shape(), "tool_plate.obj", assets=ASSETS)
    tool_plate.visual(
        tool_mesh,
        origin=Origin(xyz=(0.0, 0.0, TOOL_LAYER_Z)),
        material="machine_black",
    )


    tool_plate.inertial = Inertial.from_geometry(
        Box((0.102, 0.056, BAR_THICKNESS)),
        mass=0.22,
        origin=Origin(xyz=(0.052, 0.0, TOOL_LAYER_Z)),
    )

    model.articulation(
        "base_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.70, effort=40.0, velocity=1.5),
    )
    model.articulation(
        "arm_1_to_arm_2",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=arm_2,
        origin=Origin(xyz=(ARM_SPANS[0], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=32.0, velocity=1.7),
    )
    model.articulation(
        "arm_2_to_arm_3",
        ArticulationType.REVOLUTE,
        parent=arm_2,
        child=arm_3,
        origin=Origin(xyz=(ARM_SPANS[1], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.85, effort=24.0, velocity=1.9),
    )
    model.articulation(
        "arm_3_to_arm_4",
        ArticulationType.REVOLUTE,
        parent=arm_3,
        child=arm_4,
        origin=Origin(xyz=(ARM_SPANS[2], 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.55, effort=18.0, velocity=2.1),
    )
    model.articulation(
        "arm_4_to_tool_plate",
        ArticulationType.FIXED,
        parent=arm_4,
        child=tool_plate,
        origin=Origin(xyz=(ARM_SPANS[3], 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.003, overlap_volume_tol=0.0)

    ctx.expect_aabb_overlap("base", "arm_1", axes="xy", min_overlap=0.014)
    ctx.expect_aabb_overlap("arm_1", "arm_2", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("arm_2", "arm_3", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("arm_3", "arm_4", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_overlap("arm_4", "tool_plate", axes="xy", min_overlap=0.012)

    ctx.expect_aabb_gap("arm_1", "base", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_aabb_gap("arm_1", "arm_2", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_aabb_gap("arm_3", "arm_2", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_aabb_gap("arm_3", "arm_4", axis="z", max_gap=0.006, max_penetration=0.0)
    ctx.expect_aabb_gap("tool_plate", "arm_4", axis="z", max_gap=0.006, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_arm_1", "arm_1", world_axis="y", direction="positive", min_delta=0.03
    )
    ctx.expect_joint_motion_axis(
        "arm_1_to_arm_2", "arm_2", world_axis="y", direction="positive", min_delta=0.03
    )
    ctx.expect_joint_motion_axis(
        "arm_2_to_arm_3", "arm_3", world_axis="y", direction="positive", min_delta=0.03
    )
    ctx.expect_joint_motion_axis(
        "arm_3_to_arm_4", "arm_4", world_axis="y", direction="positive", min_delta=0.02
    )

    ctx.expect_origin_distance("arm_1", "arm_2", axes="xy", max_dist=0.25)
    ctx.expect_origin_distance("arm_2", "arm_3", axes="xy", max_dist=0.21)
    ctx.expect_origin_distance("arm_3", "arm_4", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("arm_4", "tool_plate", axes="xy", max_dist=0.15)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
