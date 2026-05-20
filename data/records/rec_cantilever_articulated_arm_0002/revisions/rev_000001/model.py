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
PEDESTAL_TOP_Z = 0.17
SHOULDER_OFFSET = (0.12, 0.0, 0.08)
ELBOW_X = 0.40
WRIST_X = 0.34

YAW_LIMIT = 1.6
SHOULDER_LOWER = -0.25
SHOULDER_UPPER = 1.05
ELBOW_LOWER = -0.15
ELBOW_UPPER = 1.55
WRIST_YAW_LIMIT = 1.15


def _filleted_box(
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    radius: float,
) -> cq.Workplane:
    shape = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        shape = shape.edges("|Z").fillet(radius)
    return shape.translate(xyz)


def _mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _box_collision(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
) -> None:
    pass


def _pedestal_shape() -> cq.Workplane:
    foot = _filleted_box((0.30, 0.26, 0.05), (0.0, 0.0, -0.145), 0.010)
    column = _filleted_box((0.22, 0.18, 0.25), (0.0, 0.0, -0.005), 0.012)
    top = _filleted_box((0.16, 0.16, 0.05), (0.0, 0.0, 0.145), 0.008)
    return foot.union(column).union(top)


def _turret_shape() -> cq.Workplane:
    base = _filleted_box((0.18, 0.20, 0.08), (0.0, 0.0, 0.04), 0.008)
    top = _filleted_box((0.12, 0.12, 0.04), (0.0, 0.0, 0.10), 0.006)
    shoulder_cheek = _filleted_box((0.06, 0.14, 0.12), (0.10, 0.0, 0.08), 0.006)
    return base.union(top).union(shoulder_cheek)


def _upper_arm_shape() -> cq.Workplane:
    shoulder_housing = _filleted_box((0.10, 0.16, 0.16), (0.035, 0.0, 0.0), 0.007)
    beam = _filleted_box((0.34, 0.06, 0.06), (0.22, 0.0, 0.0), 0.005)
    top_rib = _filleted_box((0.26, 0.03, 0.03), (0.20, 0.0, 0.045), 0.003)
    elbow_housing = _filleted_box((0.10, 0.12, 0.12), (ELBOW_X, 0.0, 0.0), 0.006)
    return shoulder_housing.union(beam).union(top_rib).union(elbow_housing)


def _forearm_shape() -> cq.Workplane:
    elbow_housing = _filleted_box((0.09, 0.13, 0.13), (0.035, 0.0, 0.0), 0.006)
    beam = _filleted_box((0.28, 0.05, 0.05), (0.18, 0.0, 0.0), 0.004)
    lower_strake = _filleted_box((0.20, 0.03, 0.03), (0.17, 0.0, -0.040), 0.003)
    wrist_housing = _filleted_box((0.08, 0.10, 0.10), (0.31, 0.0, 0.0), 0.005)
    return elbow_housing.union(beam).union(lower_strake).union(wrist_housing)


def _tool_head_shape() -> cq.Workplane:
    wrist_housing = _filleted_box((0.08, 0.10, 0.10), (0.03, 0.0, 0.0), 0.005)
    beam = _filleted_box((0.14, 0.04, 0.04), (0.10, 0.0, 0.0), 0.003)
    jaw_upper = _filleted_box((0.05, 0.02, 0.08), (0.17, 0.035, 0.0), 0.002)
    jaw_lower = _filleted_box((0.05, 0.02, 0.08), (0.17, -0.035, 0.0), 0.002)
    return wrist_housing.union(beam).union(jaw_upper).union(jaw_lower)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overhung_mechanical_arm", assets=ASSETS)

    model.material("pedestal_gray", rgba=(0.32, 0.34, 0.37, 1.0))
    model.material("actuator_dark", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("beam_blue", rgba=(0.18, 0.34, 0.68, 1.0))
    model.material("tool_orange", rgba=(0.92, 0.48, 0.14, 1.0))

    pedestal = model.part("pedestal")
    _mesh_visual(pedestal, _pedestal_shape(), "pedestal.obj", "pedestal_gray")
    _box_collision(pedestal, (0.30, 0.26, 0.05), (0.0, 0.0, -0.145))
    _box_collision(pedestal, (0.22, 0.18, 0.25), (0.0, 0.0, -0.005))
    _box_collision(pedestal, (0.16, 0.16, 0.05), (0.0, 0.0, 0.145))
    pedestal.inertial = Inertial.from_geometry(
        Box((0.26, 0.22, 0.30)),
        mass=18.0,
    )

    yaw_housing = model.part("yaw_housing")
    _mesh_visual(yaw_housing, _turret_shape(), "yaw_housing.obj", "actuator_dark")
    _box_collision(yaw_housing, (0.18, 0.20, 0.08), (0.0, 0.0, 0.04))
    _box_collision(yaw_housing, (0.12, 0.12, 0.04), (0.0, 0.0, 0.10))
    _box_collision(yaw_housing, (0.06, 0.12, 0.10), (0.10, 0.0, 0.08))
    yaw_housing.inertial = Inertial.from_geometry(
        Box((0.18, 0.20, 0.13)),
        mass=4.2,
        origin=Origin(xyz=(0.01, 0.0, 0.065)),
    )

    upper_arm = model.part("upper_arm")
    _mesh_visual(upper_arm, _upper_arm_shape(), "upper_arm.obj", "beam_blue")
    _box_collision(upper_arm, (0.12, 0.14, 0.14), (0.04, 0.0, 0.0))
    _box_collision(upper_arm, (0.30, 0.05, 0.05), (0.22, 0.0, 0.0))
    _box_collision(upper_arm, (0.10, 0.10, 0.10), (0.36, 0.0, 0.0))
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.42, 0.12, 0.12)),
        mass=4.8,
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    _mesh_visual(forearm, _forearm_shape(), "forearm.obj", "beam_blue")
    _box_collision(forearm, (0.11, 0.11, 0.11), (0.045, 0.0, 0.0))
    _box_collision(forearm, (0.24, 0.05, 0.05), (0.17, 0.0, 0.0))
    _box_collision(forearm, (0.07, 0.09, 0.09), (0.315, 0.0, 0.0))
    forearm.inertial = Inertial.from_geometry(
        Box((0.34, 0.11, 0.10)),
        mass=3.2,
        origin=Origin(xyz=(0.17, 0.0, -0.005)),
    )

    tool_head = model.part("tool_head")
    _mesh_visual(tool_head, _tool_head_shape(), "tool_head.obj", "tool_orange")
    _box_collision(tool_head, (0.10, 0.08, 0.08), (0.04, 0.0, 0.0))
    _box_collision(tool_head, (0.12, 0.04, 0.04), (0.10, 0.0, 0.0))
    _box_collision(tool_head, (0.05, 0.10, 0.08), (0.17, 0.0, 0.0))
    tool_head.inertial = Inertial.from_geometry(
        Box((0.22, 0.10, 0.08)),
        mass=1.1,
        origin=Origin(xyz=(0.11, 0.0, 0.0)),
    )

    model.articulation(
        "pedestal_to_turret",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=yaw_housing,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
            effort=60.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "turret_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=yaw_housing,
        child=upper_arm,
        origin=Origin(xyz=SHOULDER_OFFSET),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=SHOULDER_LOWER,
            upper=SHOULDER_UPPER,
            effort=45.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "upper_arm_to_forearm",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(ELBOW_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ELBOW_LOWER,
            upper=ELBOW_UPPER,
            effort=35.0,
            velocity=1.8,
        ),
    )
    model.articulation(
        "forearm_to_tool",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=tool_head,
        origin=Origin(xyz=(WRIST_X, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-WRIST_YAW_LIMIT,
            upper=WRIST_YAW_LIMIT,
            effort=15.0,
            velocity=2.5,
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
        max_pose_samples=96,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("yaw_housing", "pedestal", axes="xy", min_overlap=0.12)
    ctx.expect_aabb_gap("yaw_housing", "pedestal", axis="z", max_gap=0.005, max_penetration=0.0)
    ctx.expect_aabb_overlap("upper_arm", "yaw_housing", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("forearm", "upper_arm", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("tool_head", "forearm", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_gap("upper_arm", "pedestal", axis="z", max_gap=0.03, max_penetration=0.0)
    ctx.expect_origin_gap("forearm", "pedestal", axis="z", min_gap=0.02)
    ctx.expect_origin_gap("tool_head", "pedestal", axis="z", min_gap=0.02)

    ctx.expect_joint_motion_axis(
        "pedestal_to_turret",
        "tool_head",
        world_axis="y",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "turret_to_upper_arm",
        "tool_head",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "upper_arm_to_forearm",
        "tool_head",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "forearm_to_tool",
        "tool_head",
        world_axis="y",
        direction="positive",
        min_delta=0.01,
    )

    with ctx.pose(turret_to_upper_arm=SHOULDER_LOWER):
        ctx.expect_origin_gap("tool_head", "pedestal", axis="z", min_gap=0.005)
        ctx.expect_origin_gap("upper_arm", "pedestal", axis="z", min_gap=0.0)

    with ctx.pose(turret_to_upper_arm=0.85, upper_arm_to_forearm=1.10):
        ctx.expect_origin_gap("tool_head", "yaw_housing", axis="z", min_gap=0.16)
        ctx.expect_origin_gap("tool_head", "pedestal", axis="z", min_gap=0.20)

    with ctx.pose(upper_arm_to_forearm=ELBOW_UPPER):
        ctx.expect_origin_gap("tool_head", "upper_arm", axis="z", min_gap=0.02)
        ctx.expect_origin_gap("tool_head", "pedestal", axis="z", min_gap=0.08)

    with ctx.pose(pedestal_to_turret=YAW_LIMIT, forearm_to_tool=WRIST_YAW_LIMIT):
        ctx.expect_origin_gap("tool_head", "pedestal", axis="z", min_gap=0.02)
        ctx.expect_aabb_overlap("yaw_housing", "pedestal", axes="xy", min_overlap=0.12)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
