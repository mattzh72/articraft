from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
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
PALM_SIZE = (0.082, 0.032, 0.024)
PROXIMAL_SIZE = (0.010, 0.036, 0.010)
DISTAL_SIZE = (0.008, 0.022, 0.009)
ROOT_X_OFFSET = 0.034
ROOT_Y_OFFSET = 0.006


def _make_palm_shape() -> cq.Workplane:
    palm_x, palm_y, palm_z = PALM_SIZE
    return cq.Workplane("XY").box(palm_x, palm_y, palm_z).edges("|Z").fillet(0.003)


def _make_link_shape(size: tuple[float, float, float]) -> cq.Workplane:
    width, length, height = size
    return (
        cq.Workplane("XY")
        .box(width, length, height)
        .translate((0.0, length / 2.0, 0.0))
        .edges("|Y")
        .fillet(min(width, height) * 0.2)
    )


def _add_mesh_link(
    model: ArticulatedObject,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    collision_size: tuple[float, float, float],
    collision_origin: Origin,
    mass: float,
    material: str,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, mesh_name, assets=ASSETS), material=material)

    part.inertial = Inertial.from_geometry(
        Box(collision_size),
        mass=mass,
        origin=collision_origin,
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_finger_gripper", assets=ASSETS)

    model.material("palm_gray", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("finger_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))

    palm = _add_mesh_link(
        model=model,
        name="palm",
        shape=_make_palm_shape(),
        mesh_name="palm.obj",
        collision_size=PALM_SIZE,
        collision_origin=Origin(),
        mass=0.45,
        material="palm_gray",
    )

    proximal_origin = Origin(xyz=(0.0, PROXIMAL_SIZE[1] / 2.0, 0.0))
    distal_origin = Origin(xyz=(0.0, DISTAL_SIZE[1] / 2.0, 0.0))

    left_proximal = _add_mesh_link(
        model=model,
        name="left_proximal",
        shape=_make_link_shape(PROXIMAL_SIZE),
        mesh_name="left_proximal.obj",
        collision_size=PROXIMAL_SIZE,
        collision_origin=proximal_origin,
        mass=0.08,
        material="finger_aluminum",
    )
    left_distal = _add_mesh_link(
        model=model,
        name="left_distal",
        shape=_make_link_shape(DISTAL_SIZE),
        mesh_name="left_distal.obj",
        collision_size=DISTAL_SIZE,
        collision_origin=distal_origin,
        mass=0.05,
        material="finger_aluminum",
    )
    right_proximal = _add_mesh_link(
        model=model,
        name="right_proximal",
        shape=_make_link_shape(PROXIMAL_SIZE),
        mesh_name="right_proximal.obj",
        collision_size=PROXIMAL_SIZE,
        collision_origin=proximal_origin,
        mass=0.08,
        material="finger_aluminum",
    )
    right_distal = _add_mesh_link(
        model=model,
        name="right_distal",
        shape=_make_link_shape(DISTAL_SIZE),
        mesh_name="right_distal.obj",
        collision_size=DISTAL_SIZE,
        collision_origin=distal_origin,
        mass=0.05,
        material="finger_aluminum",
    )

    palm_x, palm_y, palm_z = PALM_SIZE
    root_z = palm_z / 2.0 + PROXIMAL_SIZE[2] / 2.0

    model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(-ROOT_X_OFFSET, ROOT_Y_OFFSET, root_z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.42,
            effort=3.0,
            velocity=3.0,
        ),
    )
    model.articulation(
        "left_proximal_to_left_distal",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_distal,
        origin=Origin(xyz=(0.0, PROXIMAL_SIZE[1], 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.34,
            effort=2.0,
            velocity=3.0,
        ),
    )
    model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(ROOT_X_OFFSET, ROOT_Y_OFFSET, root_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.42,
            effort=3.0,
            velocity=3.0,
        ),
    )
    model.articulation(
        "right_proximal_to_right_distal",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_distal,
        origin=Origin(xyz=(0.0, PROXIMAL_SIZE[1], 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.34,
            effort=2.0,
            velocity=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_gap("left_proximal", "palm", axis="z", min_gap=0.0)
    ctx.expect_origin_gap("right_proximal", "palm", axis="z", min_gap=0.0)
    ctx.expect_aabb_gap("left_proximal", "palm", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_gap("right_proximal", "palm", axis="z", max_gap=0.001, max_penetration=0.0)
    ctx.expect_aabb_overlap("left_proximal", "palm", axes="xy", min_overlap=0.008)
    ctx.expect_aabb_overlap("right_proximal", "palm", axes="xy", min_overlap=0.008)
    ctx.expect_origin_distance("left_distal", "left_proximal", axes="xy", max_dist=0.05)
    ctx.expect_origin_distance("right_distal", "right_proximal", axes="xy", max_dist=0.05)

    ctx.expect_joint_motion_axis(
        "palm_to_left_proximal",
        "left_proximal",
        world_axis="x",
        direction="positive",
        min_delta=0.003,
    )
    ctx.expect_joint_motion_axis(
        "left_proximal_to_left_distal",
        "left_distal",
        world_axis="x",
        direction="positive",
        min_delta=0.002,
    )
    ctx.expect_joint_motion_axis(
        "palm_to_right_proximal",
        "right_proximal",
        world_axis="x",
        direction="negative",
        min_delta=0.003,
    )
    ctx.expect_joint_motion_axis(
        "right_proximal_to_right_distal",
        "right_distal",
        world_axis="x",
        direction="negative",
        min_delta=0.002,
    )

    with ctx.pose(
        palm_to_left_proximal=0.42,
        palm_to_right_proximal=0.42,
    ):
        ctx.expect_origin_gap("left_proximal", "palm", axis="z", min_gap=0.0)
        ctx.expect_origin_gap("right_proximal", "palm", axis="z", min_gap=0.0)
        ctx.expect_origin_distance("left_proximal", "right_proximal", axes="xy", max_dist=0.085)

    with ctx.pose(
        palm_to_left_proximal=0.42,
        left_proximal_to_left_distal=0.34,
        palm_to_right_proximal=0.42,
        right_proximal_to_right_distal=0.34,
    ):
        ctx.expect_origin_gap("left_distal", "palm", axis="z", min_gap=0.0)
        ctx.expect_origin_gap("right_distal", "palm", axis="z", min_gap=0.0)
        ctx.expect_origin_distance("left_distal", "right_distal", axes="xy", max_dist=0.05)
        ctx.expect_origin_distance("left_distal", "left_proximal", axes="xy", max_dist=0.04)
        ctx.expect_origin_distance("right_distal", "right_proximal", axes="xy", max_dist=0.04)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
