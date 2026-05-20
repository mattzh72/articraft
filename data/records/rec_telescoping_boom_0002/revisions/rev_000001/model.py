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

BASE_LEN = 0.42
BASE_OUTER_W = 0.12
BASE_OUTER_H = 0.08
BASE_WALL = 0.006

MID_LEN = 0.33
MID_OUTER_W = 0.093
MID_OUTER_H = 0.059
MID_WALL = 0.005

INNER_LEN = 0.23
INNER_OUTER_W = 0.066
INNER_OUTER_H = 0.042
INNER_WALL = 0.004

MID_TRAVEL = 0.19
INNER_TRAVEL = 0.16

MID_JOINT_X = 0.075
MID_JOINT_Z = -(BASE_WALL / 2.0 + 0.002 + MID_WALL / 2.0)
INNER_JOINT_X = 0.065
INNER_JOINT_Z = -(MID_WALL / 2.0 + 0.002 + INNER_WALL / 2.0)

BEAM_X_START = 0.222
BEAM_LEN = 0.064
BEAM_X_CENTER = BEAM_X_START + BEAM_LEN / 2.0
BEAM_W = 0.026
BEAM_H = 0.012
BEAM_CENTER_Z = -0.022

HEAD_X_START = BEAM_X_START + BEAM_LEN
HEAD_LEN = 0.048
HEAD_CENTER_X = HEAD_X_START + HEAD_LEN / 2.0
HEAD_W = 0.052
HEAD_H = 0.028
HEAD_CENTER_Z = -0.055

ROLLER_RADIUS = 0.0075
ROLLER_LEN = 0.028
ROLLER_CENTER_X = HEAD_X_START + 0.032
ROLLER_CENTER_Z = -0.071


def _section_center_z(outer_h: float, wall: float) -> float:
    return -(outer_h / 2.0 - wall / 2.0)


def _bottom_surface_z(outer_h: float, wall: float) -> float:
    return _section_center_z(outer_h, wall) - outer_h / 2.0


def _bottom_wall_center_z(outer_h: float, wall: float) -> float:
    return _bottom_surface_z(outer_h, wall) + wall / 2.0


def _box_tube(length: float, outer_w: float, outer_h: float, wall: float) -> cq.Workplane:
    center_z = _section_center_z(outer_h, wall)
    outer = cq.Workplane("YZ").center(0.0, center_z).rect(outer_w, outer_h).extrude(length)
    inner = (
        cq.Workplane("YZ")
        .center(0.0, center_z)
        .rect(outer_w - 2.0 * wall, outer_h - 2.0 * wall)
        .extrude(length + 0.004)
        .translate((-0.002, 0.0, 0.0))
    )
    return outer.cut(inner)


def _stage_with_guide_pads(
    length: float,
    outer_w: float,
    outer_h: float,
    wall: float,
    *,
    side_pad: float,
    top_pad: float,
    pad_len: float,
) -> cq.Workplane:
    center_z = _section_center_z(outer_h, wall)
    top_surface_z = wall / 2.0
    bottom_surface_z = _bottom_surface_z(outer_h, wall)
    side_pad_h = max(outer_h * 0.34, 0.014)
    top_pad_w = max(outer_w * 0.44, 0.028)

    shape = _box_tube(length, outer_w, outer_h, wall)
    for x_center in (0.048, length - 0.048):
        shape = shape.union(
            cq.Workplane("XY")
            .box(pad_len, side_pad, side_pad_h)
            .translate((x_center, outer_w / 2.0 + side_pad / 2.0, center_z))
        )
        shape = shape.union(
            cq.Workplane("XY")
            .box(pad_len, side_pad, side_pad_h)
            .translate((x_center, -(outer_w / 2.0 + side_pad / 2.0), center_z))
        )
        shape = shape.union(
            cq.Workplane("XY")
            .box(pad_len, top_pad_w, top_pad)
            .translate((x_center, 0.0, top_surface_z + top_pad / 2.0))
        )
        shape = shape.union(
            cq.Workplane("XY")
            .box(pad_len, top_pad_w, top_pad)
            .translate((x_center, 0.0, bottom_surface_z - top_pad / 2.0))
        )
    return shape


def _make_base_shape() -> cq.Workplane:
    shoe = (
        cq.Workplane("XY")
        .box(0.17, 0.092, 0.018)
        .translate((0.11, 0.0, _bottom_surface_z(BASE_OUTER_H, BASE_WALL) - 0.009))
    )
    rear_saddle = (
        cq.Workplane("XY")
        .box(0.05, 0.082, 0.03)
        .translate((0.028, 0.0, _section_center_z(BASE_OUTER_H, BASE_WALL) - 0.015))
    )
    return _box_tube(BASE_LEN, BASE_OUTER_W, BASE_OUTER_H, BASE_WALL).union(shoe).union(rear_saddle)


def _make_inner_stage_shape() -> cq.Workplane:
    beam = (
        cq.Workplane("XY")
        .box(BEAM_LEN, BEAM_W, BEAM_H)
        .translate((BEAM_X_CENTER, 0.0, BEAM_CENTER_Z))
    )
    head = (
        cq.Workplane("XY")
        .box(HEAD_LEN, HEAD_W, HEAD_H)
        .translate((HEAD_CENTER_X, 0.0, HEAD_CENTER_Z))
    )
    roller = (
        cq.Workplane("XZ")
        .center(ROLLER_CENTER_X, ROLLER_CENTER_Z)
        .circle(ROLLER_RADIUS)
        .extrude(ROLLER_LEN)
        .translate((0.0, -ROLLER_LEN / 2.0, 0.0))
    )
    return (
        _stage_with_guide_pads(
            INNER_LEN,
            INNER_OUTER_W,
            INNER_OUTER_H,
            INNER_WALL,
            side_pad=0.0014,
            top_pad=0.0012,
            pad_len=0.03,
        )
        .union(beam)
        .union(head)
        .union(roller)
    )


def _add_tube_collisions(part, length: float, outer_w: float, outer_h: float, wall: float) -> None:
    center_z = _section_center_z(outer_h, wall)
    inner_h = outer_h - 2.0 * wall
    x_center = length / 2.0






def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_extension_boom", assets=ASSETS)
    model.material("boom_steel", rgba=(0.60, 0.61, 0.64, 1.0))
    model.material("tool_dark", rgba=(0.17, 0.18, 0.20, 1.0))

    base = model.part("base")
    base_shape = _make_base_shape()
    base.visual(
        mesh_from_cadquery(base_shape, "base_section.obj", assets=ASSETS), material="boom_steel"
    )
    _add_tube_collisions(base, BASE_LEN, BASE_OUTER_W, BASE_OUTER_H, BASE_WALL)

    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_OUTER_W, BASE_OUTER_H + 0.02)),
        mass=4.6,
        origin=Origin(
            xyz=(
                BASE_LEN / 2.0,
                0.0,
                _section_center_z(BASE_OUTER_H, BASE_WALL) - 0.01,
            )
        ),
    )

    mid_stage = model.part("mid_stage")
    mid_shape = _stage_with_guide_pads(
        MID_LEN,
        MID_OUTER_W,
        MID_OUTER_H,
        MID_WALL,
        side_pad=0.0015,
        top_pad=0.0012,
        pad_len=0.034,
    )
    mid_stage.visual(
        mesh_from_cadquery(mid_shape, "mid_stage.obj", assets=ASSETS),
        material="boom_steel",
    )
    _add_tube_collisions(mid_stage, MID_LEN, MID_OUTER_W, MID_OUTER_H, MID_WALL)
    mid_stage.inertial = Inertial.from_geometry(
        Box((MID_LEN, MID_OUTER_W, MID_OUTER_H)),
        mass=1.9,
        origin=Origin(xyz=(MID_LEN / 2.0, 0.0, _section_center_z(MID_OUTER_H, MID_WALL))),
    )

    inner_stage = model.part("inner_stage")
    inner_shape = _make_inner_stage_shape()
    inner_stage.visual(
        mesh_from_cadquery(inner_shape, "inner_stage.obj", assets=ASSETS),
        material="tool_dark",
    )
    _add_tube_collisions(inner_stage, INNER_LEN, INNER_OUTER_W, INNER_OUTER_H, INNER_WALL)



    inner_stage.inertial = Inertial.from_geometry(
        Box((HEAD_X_START + HEAD_LEN, INNER_OUTER_W, 0.075)),
        mass=1.15,
        origin=Origin(xyz=((HEAD_X_START + HEAD_LEN) / 2.0, 0.0, -0.037)),
    )

    model.articulation(
        "base_to_mid_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mid_stage,
        origin=Origin(xyz=(MID_JOINT_X, 0.0, MID_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MID_TRAVEL,
            effort=1400.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "mid_stage_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=mid_stage,
        child=inner_stage,
        origin=Origin(xyz=(INNER_JOINT_X, 0.0, INNER_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=INNER_TRAVEL,
            effort=900.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=128, overlap_tol=0.001, overlap_volume_tol=0.0)
    ctx.expect_aabb_overlap("mid_stage", "base", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("inner_stage", "mid_stage", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("inner_stage", "base", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("mid_stage", "base", axes="xy", max_dist=0.08)
    ctx.expect_origin_distance("inner_stage", "mid_stage", axes="xy", max_dist=0.08)
    ctx.expect_joint_motion_axis(
        "base_to_mid_stage",
        "mid_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.06,
    )
    ctx.expect_joint_motion_axis(
        "mid_stage_to_inner_stage",
        "inner_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.05,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
