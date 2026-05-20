from __future__ import annotations

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
import cadquery as cq

BASE_PLATE = (0.24, 0.18, 0.02)
BASE_PEDESTAL = (0.10, 0.12, 0.10)
HINGE_Z = 0.14

BOOM_SLEEVE_LEN = 0.34
BOOM_OUTER_W = 0.06
BOOM_OUTER_H = 0.05
BOOM_INNER_W = 0.04
BOOM_INNER_H = 0.034
BOOM_SLEEVE_CENTER_X = 0.20
BOOM_PIVOT_LUG = (0.05, 0.03, 0.03)

STAGE_JOINT_X = 0.03
STAGE_CARRIAGE = (0.06, 0.038, 0.032)
STAGE_BEAM = (0.36, 0.034, 0.028)
STAGE_BEAM_CENTER_X = 0.22
HEAD_JOINT_X = 0.39

HEAD_HUB = (0.024, 0.03, 0.024)
HEAD_BODY = (0.09, 0.038, 0.05)
HEAD_BODY_CENTER = (0.07, 0.0, -0.012)

BOOM_UPPER = 1.05
STAGE_UPPER = 0.32
HEAD_LOWER = -0.9
HEAD_UPPER = 0.65


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _box_collision(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
) -> None:
    pass


def _ring_block(
    length: float,
    outer_y: float,
    outer_z: float,
    inner_y: float,
    inner_z: float,
    center_x: float,
) -> cq.Workplane:
    outer = _cq_box((length, outer_y, outer_z), (center_x, 0.0, 0.0))
    inner = _cq_box((length + 0.004, inner_y, inner_z), (center_x, 0.0, 0.0))
    return outer.cut(inner)


def _make_base_shape() -> cq.Workplane:
    plate = _cq_box(BASE_PLATE, (0.0, 0.0, BASE_PLATE[2] / 2.0))
    pedestal = _cq_box(
        BASE_PEDESTAL,
        (0.0, 0.0, BASE_PLATE[2] + BASE_PEDESTAL[2] / 2.0),
    )
    ear_size = (0.06, 0.014, 0.07)
    ear_z = 0.145
    ear_y = 0.024
    left_ear = _cq_box(ear_size, (0.0, ear_y, ear_z))
    right_ear = _cq_box(ear_size, (0.0, -ear_y, ear_z))
    stop_block = _cq_box((0.05, 0.05, 0.018), (0.055, 0.0, 0.100))
    rear_rib = _cq_box((0.06, 0.06, 0.03), (-0.015, 0.0, 0.115))
    return plate.union(pedestal).union(left_ear).union(right_ear).union(stop_block).union(rear_rib)


def _make_boom_shape() -> cq.Workplane:
    sleeve_outer = _cq_box(
        (BOOM_SLEEVE_LEN, BOOM_OUTER_W, BOOM_OUTER_H),
        (BOOM_SLEEVE_CENTER_X, 0.0, 0.0),
    )
    sleeve_inner = _cq_box(
        (BOOM_SLEEVE_LEN + 0.01, BOOM_INNER_W, BOOM_INNER_H),
        (BOOM_SLEEVE_CENTER_X, 0.0, 0.0),
    )
    sleeve = sleeve_outer.cut(sleeve_inner)
    pivot_lug = _cq_box(BOOM_PIVOT_LUG, (0.0, 0.0, 0.0))
    rear_guide = _ring_block(
        0.02, BOOM_OUTER_W, BOOM_OUTER_H, BOOM_INNER_W, BOOM_INNER_H, STAGE_JOINT_X
    )
    front_stop = _ring_block(0.014, BOOM_OUTER_W, BOOM_OUTER_H, BOOM_INNER_W, BOOM_INNER_H, 0.372)
    lower_stop_tab = _cq_box((0.03, 0.045, 0.014), (0.05, 0.0, -0.024))
    return sleeve.union(pivot_lug).union(rear_guide).union(front_stop).union(lower_stop_tab)


def _make_stage_shape() -> cq.Workplane:
    carriage = _cq_box(STAGE_CARRIAGE, (0.03, 0.0, 0.0))
    beam = _cq_box(STAGE_BEAM, (STAGE_BEAM_CENTER_X, 0.0, 0.0))
    fork_y = 0.023
    fork_ear = (0.03, 0.012, 0.05)
    upper_stop = _cq_box((0.018, 0.028, 0.012), (HEAD_JOINT_X - 0.008, 0.0, 0.027))
    lower_stop = _cq_box((0.018, 0.028, 0.012), (HEAD_JOINT_X - 0.008, 0.0, -0.027))
    left_ear = _cq_box(fork_ear, (HEAD_JOINT_X, fork_y, 0.0))
    right_ear = _cq_box(fork_ear, (HEAD_JOINT_X, -fork_y, 0.0))
    return carriage.union(beam).union(left_ear).union(right_ear).union(upper_stop).union(lower_stop)


def _make_head_shape() -> cq.Workplane:
    hub = _cq_box(HEAD_HUB, (0.0, 0.0, 0.0))
    body = _cq_box(HEAD_BODY, HEAD_BODY_CENTER)
    nose = cq.Workplane("YZ").circle(0.019).extrude(0.024).translate((0.103, 0.0, -0.012))
    stop_tab = _cq_box((0.022, 0.026, 0.012), (-0.004, 0.0, 0.026))
    rear_cheek = _cq_box((0.02, 0.026, 0.04), (-0.012, 0.0, -0.008))
    return hub.union(body).union(nose).union(stop_tab).union(rear_cheek)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_inspection_boom", assets=ASSETS)

    model.material("graphite", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("safety_orange", rgba=(0.95, 0.48, 0.10, 1.0))
    model.material("blackened_steel", rgba=(0.18, 0.18, 0.20, 1.0))
    model.material("sensor_glass", rgba=(0.20, 0.42, 0.56, 0.45))

    base_mount = model.part("base_mount")
    base_mount.visual(
        mesh_from_cadquery(_make_base_shape(), "base_mount.obj", assets=ASSETS),
        material="graphite",
    )
    _box_collision(base_mount, BASE_PLATE, (0.0, 0.0, BASE_PLATE[2] / 2.0))
    _box_collision(base_mount, BASE_PEDESTAL, (0.0, 0.0, BASE_PLATE[2] + BASE_PEDESTAL[2] / 2.0))
    _box_collision(base_mount, (0.06, 0.014, 0.07), (0.0, 0.024, 0.145))
    _box_collision(base_mount, (0.06, 0.014, 0.07), (0.0, -0.024, 0.145))
    _box_collision(base_mount, (0.05, 0.05, 0.018), (0.055, 0.0, 0.100))
    base_mount.inertial = Inertial.from_geometry(
        Box((0.24, 0.18, 0.18)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    primary_boom = model.part("primary_boom")
    primary_boom.visual(
        mesh_from_cadquery(_make_boom_shape(), "primary_boom.obj", assets=ASSETS),
        material="safety_orange",
    )
    _box_collision(primary_boom, BOOM_PIVOT_LUG, (0.0, 0.0, 0.0))
    _box_collision(primary_boom, (0.34, BOOM_OUTER_W, 0.008), (0.20, 0.0, 0.021))
    _box_collision(primary_boom, (0.34, BOOM_OUTER_W, 0.008), (0.20, 0.0, -0.021))
    _box_collision(primary_boom, (0.34, 0.01, BOOM_OUTER_H), (0.20, 0.025, 0.0))
    _box_collision(primary_boom, (0.34, 0.01, BOOM_OUTER_H), (0.20, -0.025, 0.0))
    _box_collision(primary_boom, (0.014, BOOM_OUTER_W, BOOM_OUTER_H), (0.372, 0.0, 0.0))
    _box_collision(primary_boom, (0.03, 0.045, 0.014), (0.05, 0.0, -0.024))
    primary_boom.inertial = Inertial.from_geometry(
        Box((0.40, 0.06, 0.05)),
        mass=2.3,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
    )

    telescoping_stage = model.part("telescoping_stage")
    telescoping_stage.visual(
        mesh_from_cadquery(_make_stage_shape(), "telescoping_stage.obj", assets=ASSETS),
        material="blackened_steel",
    )
    _box_collision(telescoping_stage, STAGE_CARRIAGE, (0.03, 0.0, 0.0))
    _box_collision(telescoping_stage, STAGE_BEAM, (STAGE_BEAM_CENTER_X, 0.0, 0.0))
    _box_collision(telescoping_stage, (0.03, 0.012, 0.05), (HEAD_JOINT_X, 0.023, 0.0))
    _box_collision(telescoping_stage, (0.03, 0.012, 0.05), (HEAD_JOINT_X, -0.023, 0.0))
    _box_collision(telescoping_stage, (0.018, 0.028, 0.012), (HEAD_JOINT_X - 0.008, 0.0, 0.027))
    _box_collision(telescoping_stage, (0.018, 0.028, 0.012), (HEAD_JOINT_X - 0.008, 0.0, -0.027))
    telescoping_stage.inertial = Inertial.from_geometry(
        Box((0.42, 0.04, 0.04)),
        mass=1.4,
        origin=Origin(xyz=(0.21, 0.0, 0.0)),
    )

    inspection_head = model.part("inspection_head")
    inspection_head.visual(
        mesh_from_cadquery(_make_head_shape(), "inspection_head.obj", assets=ASSETS),
        material="blackened_steel",
    )
    inspection_head.visual(
        Box((0.01, 0.022, 0.022)),
        origin=Origin(xyz=(0.126, 0.0, -0.012)),
        material="sensor_glass",
    )
    _box_collision(inspection_head, HEAD_HUB, (0.0, 0.0, 0.0))
    _box_collision(inspection_head, HEAD_BODY, HEAD_BODY_CENTER)
    _box_collision(inspection_head, (0.024, 0.028, 0.038), (0.103, 0.0, -0.012))
    inspection_head.inertial = Inertial.from_geometry(
        Box((0.13, 0.04, 0.06)),
        mass=0.7,
        origin=Origin(xyz=(0.06, 0.0, -0.01)),
    )

    model.articulation(
        "base_to_boom",
        ArticulationType.REVOLUTE,
        parent=base_mount,
        child=primary_boom,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=BOOM_UPPER, effort=45.0, velocity=1.2),
    )
    model.articulation(
        "boom_to_stage",
        ArticulationType.PRISMATIC,
        parent=primary_boom,
        child=telescoping_stage,
        origin=Origin(xyz=(STAGE_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=STAGE_UPPER, effort=35.0, velocity=0.3),
    )
    model.articulation(
        "stage_to_head",
        ArticulationType.REVOLUTE,
        parent=telescoping_stage,
        child=inspection_head,
        origin=Origin(xyz=(HEAD_JOINT_X, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=HEAD_LOWER, upper=HEAD_UPPER, effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)

    ctx.allow_overlap(
        "base_mount",
        "primary_boom",
        reason="The boom pivot lug sits inside the base clevis, so conservative AABB overlap at the hinge is intentional.",
    )
    ctx.allow_overlap(
        "primary_boom",
        "telescoping_stage",
        reason="The extension stage nests inside the boom sleeve and may trigger conservative overlap warnings despite wall clearances.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_joint_motion_axis(
        "base_to_boom",
        "primary_boom",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "boom_to_stage",
        "telescoping_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    ctx.expect_joint_motion_axis(
        "stage_to_head",
        "inspection_head",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    ctx.expect_aabb_overlap("base_mount", "primary_boom", axes="xy", min_overlap=0.01)
    ctx.expect_aabb_overlap("primary_boom", "telescoping_stage", axes="xy", min_overlap=0.02)
    ctx.expect_aabb_overlap("inspection_head", "telescoping_stage", axes="xy", min_overlap=0.006)
    ctx.expect_origin_distance("primary_boom", "telescoping_stage", axes="xy", max_dist=0.08)

    with ctx.pose(base_to_boom=BOOM_UPPER):
        ctx.expect_origin_gap("inspection_head", "base_mount", axis="z", min_gap=0.16)
        ctx.expect_aabb_overlap("base_mount", "primary_boom", axes="xy", min_overlap=0.01)

    with ctx.pose(boom_to_stage=STAGE_UPPER):
        ctx.expect_aabb_overlap("primary_boom", "telescoping_stage", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_overlap("inspection_head", "telescoping_stage", axes="xy", min_overlap=0.006)

    with ctx.pose(stage_to_head=HEAD_LOWER):
        ctx.expect_aabb_overlap("inspection_head", "telescoping_stage", axes="xy", min_overlap=0.004)

    with ctx.pose(stage_to_head=HEAD_UPPER):
        ctx.expect_aabb_overlap("inspection_head", "telescoping_stage", axes="xy", min_overlap=0.004)

    with ctx.pose(base_to_boom=BOOM_UPPER, boom_to_stage=STAGE_UPPER):
        ctx.expect_origin_gap("inspection_head", "base_mount", axis="z", min_gap=0.28)
        ctx.expect_aabb_overlap("primary_boom", "telescoping_stage", axes="xy", min_overlap=0.008)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
