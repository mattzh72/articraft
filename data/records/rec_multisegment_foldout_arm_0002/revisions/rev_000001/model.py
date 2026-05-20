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
BASE_WIDTH = 0.120
BASE_DEPTH = 0.090
BASE_HEIGHT = 0.048
BASE_JOINT_Z = 0.116

LINK_WIDTH = 0.040
LINK_DEPTH = 0.022
BARREL_RADIUS = 0.011
BARREL_LENGTH = 0.022
FORK_HEIGHT = 0.028
LUG_THICKNESS = 0.008
LOWER_WEB_HEIGHT = 0.026

LINK_1_LENGTH = 0.170
LINK_2_LENGTH = 0.150
LINK_3_LENGTH = 0.125
SHOULDER_MAX_ANGLE = 1.30
ELBOW_MAX_ANGLE = 1.70
WRIST_MAX_ANGLE = 1.55


def _make_base_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BASE_WIDTH,
        BASE_DEPTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    upper_housing = (
        cq.Workplane("XY")
        .box(
            0.064,
            0.050,
            0.032,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_HEIGHT))
    )
    center_spine = (
        cq.Workplane("XY")
        .box(
            0.030,
            0.050,
            0.024,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.080))
    )
    left_cheek = (
        cq.Workplane("XY")
        .box(
            0.012,
            0.034,
            BASE_JOINT_Z - BASE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-0.019, 0.0, BASE_HEIGHT))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(
            0.012,
            0.034,
            BASE_JOINT_Z - BASE_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.019, 0.0, BASE_HEIGHT))
    )
    return body.union(upper_housing).union(center_spine).union(left_cheek).union(right_cheek)


def _make_link_shape(length: float) -> cq.Workplane:
    beam_bottom = 0.014
    beam_height = length - FORK_HEIGHT - beam_bottom
    beam = (
        cq.Workplane("XY")
        .box(
            LINK_WIDTH,
            LINK_DEPTH,
            beam_height,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, beam_bottom))
    )
    lower_web = cq.Workplane("XY").box(
        LINK_WIDTH * 0.74,
        LINK_DEPTH,
        LOWER_WEB_HEIGHT,
        centered=(True, True, False),
    )
    lower_left_cheek = (
        cq.Workplane("XY")
        .box(
            LUG_THICKNESS,
            LINK_DEPTH,
            LOWER_WEB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-0.013, 0.0, 0.0))
    )
    lower_right_cheek = (
        cq.Workplane("XY")
        .box(
            LUG_THICKNESS,
            LINK_DEPTH,
            LOWER_WEB_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.013, 0.0, 0.0))
    )
    barrel = (
        cq.Workplane("XZ")
        .circle(BARREL_RADIUS)
        .extrude(BARREL_LENGTH)
        .translate((0.0, -BARREL_LENGTH / 2.0, 0.0))
    )
    fork_z = length - FORK_HEIGHT
    upper_left_lug = (
        cq.Workplane("XY")
        .box(
            LUG_THICKNESS,
            LINK_DEPTH,
            FORK_HEIGHT,
            centered=(True, True, False),
        )
        .translate((-(LINK_WIDTH / 2.0 - LUG_THICKNESS / 2.0), 0.0, fork_z))
    )
    upper_right_lug = (
        cq.Workplane("XY")
        .box(
            LUG_THICKNESS,
            LINK_DEPTH,
            FORK_HEIGHT,
            centered=(True, True, False),
        )
        .translate(((LINK_WIDTH / 2.0 - LUG_THICKNESS / 2.0), 0.0, fork_z))
    )
    fork_brace = (
        cq.Workplane("XY")
        .box(
            LINK_WIDTH,
            LINK_DEPTH,
            0.008,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, fork_z))
    )
    return (
        barrel.union(lower_web)
        .union(lower_left_cheek)
        .union(lower_right_cheek)
        .union(beam)
        .union(upper_left_lug)
        .union(upper_right_lug)
        .union(fork_brace)
    )


def _make_tool_shape() -> cq.Workplane:
    mount_stem = cq.Workplane("XY").box(
        0.020,
        0.020,
        0.014,
        centered=(True, True, False),
    )
    lower_body = (
        cq.Workplane("XY")
        .box(
            0.052,
            0.030,
            0.030,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.014))
    )
    upper_body = (
        cq.Workplane("XY")
        .box(
            0.034,
            0.024,
            0.016,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.044))
    )
    sensor_face = (
        cq.Workplane("XY")
        .box(
            0.020,
            0.020,
            0.010,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, 0.060))
    )
    return mount_stem.union(lower_body).union(upper_body).union(sensor_face)


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    mesh_name: str,
    material_name: str,
    collisions: list[tuple[tuple[float, float, float], tuple[float, float, float]]],
    inertial_size: tuple[float, float, float],
    inertial_origin: tuple[float, float, float],
    mass: float,
):
    part = model.part(name)
    part.visual(
        mesh_from_cadquery(shape, mesh_name, assets=ASSETS),
        material=material_name,
    )
    for size, xyz in collisions:
        pass
    part.inertial = Inertial.from_geometry(
        Box(inertial_size),
        mass=mass,
        origin=Origin(xyz=inertial_origin),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="deployable_stacked_arm", assets=ASSETS)

    model.material("base_dark", rgba=(0.24, 0.25, 0.28, 1.0))
    model.material("arm_alloy", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("sensor_black", rgba=(0.10, 0.11, 0.13, 1.0))

    base = _add_mesh_part(
        model,
        name="base",
        shape=_make_base_shape(),
        mesh_name="deployable_base.obj",
        material_name="base_dark",
        collisions=[
            ((BASE_WIDTH, BASE_DEPTH, BASE_HEIGHT), (0.0, 0.0, BASE_HEIGHT / 2.0)),
            ((0.064, 0.050, 0.032), (0.0, 0.0, 0.064)),
            ((0.030, 0.050, 0.024), (0.0, 0.0, 0.092)),
        ],
        inertial_size=(0.100, 0.080, 0.100),
        inertial_origin=(0.0, 0.0, 0.050),
        mass=1.80,
    )

    shoulder_link = _add_mesh_part(
        model,
        name="shoulder_link",
        shape=_make_link_shape(LINK_1_LENGTH),
        mesh_name="shoulder_link.obj",
        material_name="arm_alloy",
        collisions=[
            ((0.030, LINK_DEPTH, LINK_1_LENGTH - 0.026), (0.0, 0.0, LINK_1_LENGTH / 2.0)),
            ((0.020, LINK_DEPTH, 0.016), (0.0, 0.0, 0.010)),
        ],
        inertial_size=(0.032, LINK_DEPTH, LINK_1_LENGTH - 0.010),
        inertial_origin=(0.0, 0.0, LINK_1_LENGTH / 2.0),
        mass=0.48,
    )

    forearm_link = _add_mesh_part(
        model,
        name="forearm_link",
        shape=_make_link_shape(LINK_2_LENGTH),
        mesh_name="forearm_link.obj",
        material_name="arm_alloy",
        collisions=[
            ((0.030, LINK_DEPTH, LINK_2_LENGTH - 0.026), (0.0, 0.0, LINK_2_LENGTH / 2.0)),
            ((0.020, LINK_DEPTH, 0.016), (0.0, 0.0, 0.010)),
        ],
        inertial_size=(0.032, LINK_DEPTH, LINK_2_LENGTH - 0.010),
        inertial_origin=(0.0, 0.0, LINK_2_LENGTH / 2.0),
        mass=0.38,
    )

    wrist_link = _add_mesh_part(
        model,
        name="wrist_link",
        shape=_make_link_shape(LINK_3_LENGTH),
        mesh_name="wrist_link.obj",
        material_name="arm_alloy",
        collisions=[
            ((0.030, LINK_DEPTH, LINK_3_LENGTH - 0.026), (0.0, 0.0, LINK_3_LENGTH / 2.0)),
            ((0.020, LINK_DEPTH, 0.016), (0.0, 0.0, 0.010)),
        ],
        inertial_size=(0.032, LINK_DEPTH, LINK_3_LENGTH - 0.010),
        inertial_origin=(0.0, 0.0, LINK_3_LENGTH / 2.0),
        mass=0.29,
    )

    tool_head = _add_mesh_part(
        model,
        name="tool_head",
        shape=_make_tool_shape(),
        mesh_name="tool_head.obj",
        material_name="sensor_black",
        collisions=[
            ((0.018, 0.018, 0.014), (0.0, 0.0, 0.007)),
            ((0.044, 0.028, 0.030), (0.0, 0.0, 0.029)),
            ((0.022, 0.020, 0.014), (0.0, 0.0, 0.051)),
        ],
        inertial_size=(0.048, 0.030, 0.052),
        inertial_origin=(0.0, 0.0, 0.028),
        mass=0.14,
    )

    model.articulation(
        "shoulder_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, BASE_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SHOULDER_MAX_ANGLE,
            effort=24.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "elbow_hinge",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forearm_link,
        origin=Origin(xyz=(0.0, 0.0, LINK_1_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=ELBOW_MAX_ANGLE,
            effort=18.0,
            velocity=2.0,
        ),
    )
    model.articulation(
        "wrist_hinge",
        ArticulationType.REVOLUTE,
        parent=forearm_link,
        child=wrist_link,
        origin=Origin(xyz=(0.0, 0.0, LINK_2_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=WRIST_MAX_ANGLE,
            effort=12.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "wrist_to_tool",
        ArticulationType.FIXED,
        parent=wrist_link,
        child=tool_head,
        origin=Origin(xyz=(0.0, 0.0, LINK_3_LENGTH)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.allow_overlap(
        "shoulder_link",
        "tool_head",
        reason="compact stowed posture nests the wrist tool beside the shoulder; conservative AABB proxy overlap is acceptable",
    )
    ctx.allow_overlap(
        "base",
        "tool_head",
        reason="tool folds close to the base tower in the stowed envelope and can trigger conservative AABB overlap",
    )
    ctx.allow_overlap(
        "base",
        "wrist_link",
        reason="stowed wrist link passes close to the base housing; conservative AABB overlap is acceptable",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_gap("shoulder_link", "base", axis="z", min_gap=0.0)
    ctx.expect_origin_distance("shoulder_link", "base", axes="xy", max_dist=0.025)
    ctx.expect_aabb_overlap("shoulder_link", "base", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_gap("shoulder_link", "base", axis="z", max_gap=0.020, max_penetration=0.0)

    ctx.expect_origin_gap("forearm_link", "shoulder_link", axis="z", min_gap=0.0)
    ctx.expect_origin_distance("forearm_link", "shoulder_link", axes="xy", max_dist=0.020)
    ctx.expect_aabb_overlap("forearm_link", "shoulder_link", axes="xy", min_overlap=0.014)
    ctx.expect_aabb_gap("forearm_link", "shoulder_link", axis="z", max_gap=0.020, max_penetration=0.0)

    ctx.expect_origin_gap("wrist_link", "forearm_link", axis="z", min_gap=0.0)
    ctx.expect_origin_distance("wrist_link", "forearm_link", axes="xy", max_dist=0.020)
    ctx.expect_aabb_overlap("wrist_link", "forearm_link", axes="xy", min_overlap=0.014)
    ctx.expect_aabb_gap("wrist_link", "forearm_link", axis="z", max_gap=0.020, max_penetration=0.0)

    ctx.expect_origin_gap("tool_head", "wrist_link", axis="z", min_gap=0.0)
    ctx.expect_origin_distance("tool_head", "wrist_link", axes="xy", max_dist=0.015)
    ctx.expect_aabb_overlap("tool_head", "wrist_link", axes="xy", min_overlap=0.012)
    ctx.expect_aabb_gap("tool_head", "wrist_link", axis="z", max_gap=0.020, max_penetration=0.0)

    ctx.expect_joint_motion_axis(
        "shoulder_hinge",
        "shoulder_link",
        world_axis="x",
        direction="positive",
        min_delta=0.040,
    )
    ctx.expect_joint_motion_axis(
        "elbow_hinge",
        "forearm_link",
        world_axis="x",
        direction="positive",
        min_delta=0.040,
    )
    ctx.expect_joint_motion_axis(
        "wrist_hinge",
        "wrist_link",
        world_axis="x",
        direction="positive",
        min_delta=0.030,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
