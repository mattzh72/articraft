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
from math import pi

PALM_SIZE = (0.22, 0.05, 0.022)
JOINT_X = 0.09
JOINT_Z = 0.020
TOWER_TOP_Z = JOINT_Z - 0.004
TOWER_SIZE = (0.026, 0.022, TOWER_TOP_Z - PALM_SIZE[2] / 2.0)

LINK_DEPTH = 0.018
BASE_LUG_WIDTH = 0.010
BASE_LUG_LENGTH = 0.008
BEAM_WIDTH = 0.014
DISTAL_CLEARANCE = 0.006
PAD_WIDTH = 0.006
PAD_LENGTH = 0.014

PROXIMAL_LENGTH = 0.055
MIDDLE_LENGTH = 0.045
DISTAL_LENGTH = 0.032


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _base_box(size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size, centered=(True, True, True))


def make_palm_shape() -> cq.Workplane:
    base = _base_box(PALM_SIZE)
    left_tower = (
        cq.Workplane("XY")
        .box(*TOWER_SIZE, centered=(True, True, False))
        .translate((JOINT_X, 0.0, PALM_SIZE[2] / 2.0))
    )
    right_tower = (
        cq.Workplane("XY")
        .box(*TOWER_SIZE, centered=(True, True, False))
        .translate((-JOINT_X, 0.0, PALM_SIZE[2] / 2.0))
    )
    relief = (
        cq.Workplane("XY")
        .box(0.11, PALM_SIZE[1] + 0.002, 0.012, centered=(True, True, False))
        .translate((0.0, 0.0, PALM_SIZE[2] / 2.0 + 0.002))
    )
    return base.union(left_tower).union(right_tower).cut(relief)


def make_segment_shape(length: float, *, tip: bool) -> cq.Workplane:
    beam_end = length if tip else length - DISTAL_CLEARANCE
    beam_length = beam_end - BASE_LUG_LENGTH

    base_lug = (
        cq.Workplane("XY")
        .box(BASE_LUG_WIDTH, LINK_DEPTH, BASE_LUG_LENGTH, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.001)
    )
    beam = (
        cq.Workplane("XY")
        .box(BEAM_WIDTH, LINK_DEPTH, beam_length, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_LUG_LENGTH))
        .edges("|Z")
        .fillet(0.0012)
    )
    shape = base_lug.union(beam)

    pocket_length = max(beam_length - 0.012, 0.008)
    pocket = (
        cq.Workplane("XY")
        .box(BEAM_WIDTH * 0.56, LINK_DEPTH * 0.46, pocket_length, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_LUG_LENGTH + 0.006))
    )
    shape = shape.cut(pocket)

    if tip:
        pad = (
            cq.Workplane("XY")
            .box(PAD_WIDTH, LINK_DEPTH, PAD_LENGTH, centered=(True, True, False))
            .translate(
                (
                    BEAM_WIDTH / 2.0 + PAD_WIDTH / 2.0 - 0.001,
                    0.0,
                    length - PAD_LENGTH,
                )
            )
            .edges("|Z")
            .fillet(0.0008)
        )
        shape = shape.union(pad)
    else:
        nose = (
            cq.Workplane("XY")
            .box(BEAM_WIDTH * 0.82, LINK_DEPTH * 0.86, 0.004, centered=(True, True, False))
            .translate((0.0, 0.0, length - DISTAL_CLEARANCE - 0.004))
        )
        shape = shape.union(nose)

    return shape


def add_segment_part(
    model: ArticulatedObject,
    name: str,
    *,
    length: float,
    mesh_name: str,
    material_name: str,
    mass: float,
    tip: bool,
):
    part = model.part(name)
    part.visual(_mesh(make_segment_shape(length, tip=tip), mesh_name), material=material_name)

    beam_end = length if tip else length - DISTAL_CLEARANCE
    beam_length = beam_end - BASE_LUG_LENGTH


    if tip:
        pass
    part.inertial = Inertial.from_geometry(
        Box((BEAM_WIDTH, LINK_DEPTH, length)),
        mass=mass,
        origin=Origin(xyz=(0.0, 0.0, length / 2.0)),
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cad_parallel_finger_gripper", assets=ASSETS)

    model.material("frame_gray", rgba=(0.53, 0.56, 0.60, 1.0))
    model.material("finger_gray", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("jaw_pad", rgba=(0.12, 0.12, 0.13, 1.0))

    palm = model.part("palm")
    palm.visual(_mesh(make_palm_shape(), "palm.obj"), material="frame_gray")



    palm.inertial = Inertial.from_geometry(
        Box((PALM_SIZE[0], PALM_SIZE[1], PALM_SIZE[2] + TOWER_SIZE[2])),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, TOWER_SIZE[2] / 2.0)),
    )

    left_proximal = add_segment_part(
        model,
        "left_proximal",
        length=PROXIMAL_LENGTH,
        mesh_name="proximal_segment.obj",
        material_name="finger_gray",
        mass=0.14,
        tip=False,
    )
    left_middle = add_segment_part(
        model,
        "left_middle",
        length=MIDDLE_LENGTH,
        mesh_name="middle_segment.obj",
        material_name="finger_gray",
        mass=0.11,
        tip=False,
    )
    left_distal = add_segment_part(
        model,
        "left_distal",
        length=DISTAL_LENGTH,
        mesh_name="distal_segment.obj",
        material_name="jaw_pad",
        mass=0.08,
        tip=True,
    )

    right_proximal = add_segment_part(
        model,
        "right_proximal",
        length=PROXIMAL_LENGTH,
        mesh_name="proximal_segment.obj",
        material_name="finger_gray",
        mass=0.14,
        tip=False,
    )
    right_middle = add_segment_part(
        model,
        "right_middle",
        length=MIDDLE_LENGTH,
        mesh_name="middle_segment.obj",
        material_name="finger_gray",
        mass=0.11,
        tip=False,
    )
    right_distal = add_segment_part(
        model,
        "right_distal",
        length=DISTAL_LENGTH,
        mesh_name="distal_segment.obj",
        material_name="jaw_pad",
        mass=0.08,
        tip=True,
    )

    hinge_limits = MotionLimits(lower=0.0, upper=0.30, effort=8.0, velocity=2.5)
    mid_limits = MotionLimits(lower=0.0, upper=0.45, effort=6.0, velocity=2.5)
    distal_limits = MotionLimits(lower=0.0, upper=0.35, effort=4.0, velocity=2.5)

    model.articulation(
        "left_palm_to_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(-JOINT_X, 0.0, JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "left_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, 0.0, PROXIMAL_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=mid_limits,
    )
    model.articulation(
        "left_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=distal_limits,
    )

    model.articulation(
        "right_palm_to_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(JOINT_X, 0.0, JOINT_Z), rpy=(0.0, 0.0, pi)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=hinge_limits,
    )
    model.articulation(
        "right_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, 0.0, PROXIMAL_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=mid_limits,
    )
    model.articulation(
        "right_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_LENGTH)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=distal_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.002, overlap_volume_tol=0.0)

    for side in ("left", "right"):
        ctx.expect_origin_gap(f"{side}_proximal", "palm", axis="z", min_gap=0.0)
        ctx.expect_aabb_overlap(f"{side}_proximal", "palm", axes="xy", min_overlap=0.008)
        ctx.expect_aabb_gap(f"{side}_proximal", "palm", axis="z", max_gap=0.005, max_penetration=0.001)

        ctx.expect_origin_gap(f"{side}_middle", f"{side}_proximal", axis="z", min_gap=0.0)
        ctx.expect_aabb_overlap(f"{side}_middle", f"{side}_proximal", axes="xy", min_overlap=0.008)
        ctx.expect_aabb_gap(f"{side}_middle", f"{side}_proximal", axis="z", max_gap=0.008, max_penetration=0.001)

        ctx.expect_origin_gap(f"{side}_distal", f"{side}_middle", axis="z", min_gap=0.0)
        ctx.expect_aabb_overlap(f"{side}_distal", f"{side}_middle", axes="xy", min_overlap=0.008)
        ctx.expect_aabb_gap(f"{side}_distal", f"{side}_middle", axis="z", max_gap=0.008, max_penetration=0.001)

    ctx.expect_joint_motion_axis(
        "left_palm_to_proximal",
        "left_proximal",
        world_axis="x",
        direction="positive",
        min_delta=0.006,
    )
    ctx.expect_joint_motion_axis(
        "left_proximal_to_middle",
        "left_middle",
        world_axis="x",
        direction="positive",
        min_delta=0.006,
    )
    ctx.expect_joint_motion_axis(
        "left_middle_to_distal",
        "left_distal",
        world_axis="x",
        direction="positive",
        min_delta=0.004,
    )
    ctx.expect_joint_motion_axis(
        "right_palm_to_proximal",
        "right_proximal",
        world_axis="x",
        direction="negative",
        min_delta=0.006,
    )
    ctx.expect_joint_motion_axis(
        "right_proximal_to_middle",
        "right_middle",
        world_axis="x",
        direction="negative",
        min_delta=0.006,
    )
    ctx.expect_joint_motion_axis(
        "right_middle_to_distal",
        "right_distal",
        world_axis="x",
        direction="negative",
        min_delta=0.004,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
