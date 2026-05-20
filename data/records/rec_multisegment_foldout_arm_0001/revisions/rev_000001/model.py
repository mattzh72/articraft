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
LINK_BODY_X = 0.026
LINK_BODY_Y = 0.020
ROOT_LUG_X = 0.032
ROOT_LUG_Y = 0.010
ROOT_LUG_Z = 0.018
FORK_EAR_X = 0.034
FORK_EAR_Z = 0.020
EAR_THICKNESS = 0.006
FORK_GAP = 0.012


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _make_link_shape(length: float) -> cq.Workplane:
    beam = _box((LINK_BODY_X, LINK_BODY_Y, length - 0.03), (0.0, 0.0, length / 2.0))
    root_lug = _box((ROOT_LUG_X, ROOT_LUG_Y, ROOT_LUG_Z), (0.0, 0.0, 0.0))
    root_shoulder = _box((0.032, 0.020, 0.030), (0.0, 0.0, 0.020))
    fork_bridge = _box((0.032, 0.020, 0.028), (0.0, 0.0, length - 0.016))
    ear_offset_y = (FORK_GAP + EAR_THICKNESS) / 2.0
    outer_ear_pos = _box((FORK_EAR_X, EAR_THICKNESS, FORK_EAR_Z), (0.0, ear_offset_y, length))
    outer_ear_neg = _box((FORK_EAR_X, EAR_THICKNESS, FORK_EAR_Z), (0.0, -ear_offset_y, length))

    return (
        beam.union(root_lug)
        .union(root_shoulder)
        .union(fork_bridge)
        .union(outer_ear_pos)
        .union(outer_ear_neg)
    )


def _make_base_shape() -> cq.Workplane:
    plate = _box((0.140, 0.100, 0.016), (0.0, 0.0, -0.055))
    rear_block = _box((0.072, 0.060, 0.030), (0.0, 0.0, -0.036))
    mast = _box((0.050, 0.040, 0.074), (0.0, 0.0, -0.022))
    clevis_pad = _box((0.040, 0.022, 0.024), (0.0, 0.0, -0.004))
    ear_offset_y = (FORK_GAP + EAR_THICKNESS) / 2.0
    ear_pos = _box((0.040, EAR_THICKNESS, 0.022), (0.0, ear_offset_y, 0.0))
    ear_neg = _box((0.040, EAR_THICKNESS, 0.022), (0.0, -ear_offset_y, 0.0))

    return plate.union(rear_block).union(mast).union(clevis_pad).union(ear_pos).union(ear_neg)


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_link_part(
    model: ArticulatedObject,
    name: str,
    *,
    length: float,
    mass: float,
    material: str,
) -> object:
    part = model.part(name)
    _add_mesh_visual(part, _make_link_shape(length), f"{name}.obj", material)

    ear_offset_y = (FORK_GAP + EAR_THICKNESS) / 2.0




    part.inertial = Inertial.from_geometry(
        Box((LINK_BODY_X * 1.05, LINK_BODY_Y, length)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, length / 2.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_boom", assets=ASSETS)

    model.material("boom_orange", rgba=(0.92, 0.58, 0.16, 1.0))
    model.material("boom_dark", rgba=(0.24, 0.27, 0.31, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _make_base_shape(), "base.obj", "boom_dark")





    base.inertial = Inertial.from_geometry(
        Box((0.140, 0.100, 0.090)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
    )

    lengths = (0.300, 0.260, 0.220, 0.180)
    masses = (2.2, 1.7, 1.3, 0.9)
    boom_1 = _build_link_part(
        model, "boom_1", length=lengths[0], mass=masses[0], material="boom_orange"
    )
    boom_2 = _build_link_part(
        model, "boom_2", length=lengths[1], mass=masses[1], material="boom_orange"
    )
    boom_3 = _build_link_part(
        model, "boom_3", length=lengths[2], mass=masses[2], material="boom_orange"
    )
    boom_4 = _build_link_part(
        model, "boom_4", length=lengths[3], mass=masses[3], material="boom_orange"
    )

    model.articulation(
        "base_to_boom_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=boom_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=18.0, velocity=1.2),
    )
    model.articulation(
        "boom_1_to_boom_2",
        ArticulationType.REVOLUTE,
        parent=boom_1,
        child=boom_2,
        origin=Origin(xyz=(0.0, 0.0, lengths[0])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.25, effort=14.0, velocity=1.5),
    )
    model.articulation(
        "boom_2_to_boom_3",
        ArticulationType.REVOLUTE,
        parent=boom_2,
        child=boom_3,
        origin=Origin(xyz=(0.0, 0.0, lengths[1])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=11.0, velocity=1.7),
    )
    model.articulation(
        "boom_3_to_boom_4",
        ArticulationType.REVOLUTE,
        parent=boom_3,
        child=boom_4,
        origin=Origin(xyz=(0.0, 0.0, lengths[2])),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.35, effort=8.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=192, overlap_tol=0.002, overlap_volume_tol=0.0)

    ctx.expect_origin_distance("boom_1", "base", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("boom_2", "boom_1", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("boom_3", "boom_2", axes="xy", max_dist=0.005)
    ctx.expect_origin_distance("boom_4", "boom_3", axes="xy", max_dist=0.005)

    ctx.expect_aabb_overlap("boom_1", "base", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("boom_2", "boom_1", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("boom_3", "boom_2", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("boom_4", "boom_3", axes="xy", min_overlap=0.020)

    ctx.expect_aabb_gap("boom_1", "base", axis="z", max_gap=0.010, max_penetration=0.025)
    ctx.expect_aabb_gap("boom_2", "boom_1", axis="z", max_gap=0.010, max_penetration=0.022)
    ctx.expect_aabb_gap("boom_3", "boom_2", axis="z", max_gap=0.010, max_penetration=0.022)
    ctx.expect_aabb_gap("boom_4", "boom_3", axis="z", max_gap=0.010, max_penetration=0.022)

    ctx.expect_origin_gap("boom_2", "base", axis="z", min_gap=0.240)
    ctx.expect_origin_gap("boom_3", "boom_1", axis="z", min_gap=0.220)
    ctx.expect_origin_gap("boom_4", "boom_2", axis="z", min_gap=0.180)
    ctx.expect_origin_gap("boom_4", "base", axis="z", min_gap=0.740)

    ctx.expect_joint_motion_axis(
        "base_to_boom_1",
        "boom_1",
        world_axis="x",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "boom_1_to_boom_2",
        "boom_2",
        world_axis="x",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "boom_2_to_boom_3",
        "boom_3",
        world_axis="x",
        direction="positive",
        min_delta=0.015,
    )
    ctx.expect_joint_motion_axis(
        "boom_3_to_boom_4",
        "boom_4",
        world_axis="x",
        direction="positive",
        min_delta=0.015,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
