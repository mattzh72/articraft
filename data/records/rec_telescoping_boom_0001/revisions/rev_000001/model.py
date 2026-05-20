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
def _stage_shape(
    *,
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    front_collar_len: float,
    rear_collar_len: float,
    collar_growth: float,
    nose_len: float = 0.0,
) -> cq.Workplane:
    shell = cq.Workplane("XY").box(length, outer_width, outer_height)

    if front_collar_len > 0.0:
        shell = shell.union(
            cq.Workplane("XY")
            .box(
                front_collar_len,
                outer_width + collar_growth,
                outer_height + collar_growth,
            )
            .translate((length / 2.0 - front_collar_len / 2.0, 0.0, 0.0))
        )

    if rear_collar_len > 0.0:
        shell = shell.union(
            cq.Workplane("XY")
            .box(
                rear_collar_len,
                outer_width + 0.5 * collar_growth,
                outer_height + 0.5 * collar_growth,
            )
            .translate((-length / 2.0 + rear_collar_len / 2.0, 0.0, 0.0))
        )

    shell = shell.cut(
        cq.Workplane("XY").box(
            length + 0.006,
            outer_width - 2.0 * wall,
            outer_height - 2.0 * wall,
        )
    )

    if nose_len > 0.0:
        shell = shell.union(
            cq.Workplane("XY")
            .box(
                nose_len,
                outer_width + collar_growth + 0.014,
                outer_height + collar_growth + 0.014,
            )
            .translate((length / 2.0 + nose_len / 2.0 - 0.006, 0.0, 0.0))
        )

    return shell.translate((length / 2.0, 0.0, outer_height / 2.0))


def _base_bracket_shape() -> cq.Workplane:
    base_plate = cq.Workplane("XY").box(0.46, 0.34, 0.03).translate((0.0, 0.0, 0.015))
    rear_spine = cq.Workplane("XY").box(0.08, 0.18, 0.17).translate((-0.18, 0.0, 0.115))
    front_web = cq.Workplane("XY").box(0.08, 0.14, 0.10).translate((0.07, 0.0, 0.08))
    saddle = cq.Workplane("XY").box(0.26, 0.16, 0.02).translate((-0.02, 0.0, 0.20))

    left_cheek = (
        cq.Workplane("XY")
        .box(0.24, 0.03, 0.18)
        .translate((-0.06, 0.11, 0.12))
        .cut(cq.Workplane("XY").box(0.11, 0.04, 0.08).translate((-0.05, 0.11, 0.12)))
    )
    right_cheek = (
        cq.Workplane("XY")
        .box(0.24, 0.03, 0.18)
        .translate((-0.06, -0.11, 0.12))
        .cut(cq.Workplane("XY").box(0.11, 0.04, 0.08).translate((-0.05, -0.11, 0.12)))
    )

    return (
        base_plate.union(rear_spine)
        .union(front_web)
        .union(saddle)
        .union(left_cheek)
        .union(right_cheek)
    )


def _add_mesh_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(
        mesh_from_cadquery(shape, filename, assets=ASSETS),
        origin=Origin(),
        material=material,
    )


def _add_tube_wall_collisions(
    part,
    *,
    prefix: str,
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
) -> None:
    cavity_width = outer_width - 2.0 * wall







def build_object_model() -> ArticulatedObject:
    outer_len = 1.35
    outer_width = 0.18
    outer_height = 0.14
    outer_wall = 0.012

    mid_len = 1.02
    mid_width = 0.148
    mid_height = 0.108
    mid_wall = 0.010

    inner_len = 0.82
    inner_width = 0.118
    inner_height = 0.078
    inner_wall = 0.008

    outer_to_mid_z = outer_wall + 0.004
    mid_to_inner_z = mid_wall + 0.005

    model = ArticulatedObject(name="telescoping_boom", assets=ASSETS)
    model.material("boom_yellow", rgba=(0.88, 0.72, 0.10, 1.0))
    model.material("industrial_steel", rgba=(0.28, 0.30, 0.33, 1.0))

    base_bracket = model.part("base_bracket")
    _add_mesh_visual(base_bracket, _base_bracket_shape(), "base_bracket.obj", "industrial_steel")





    base_bracket.inertial = Inertial.from_geometry(
        Box((0.46, 0.34, 0.22)),
        mass=48.0,
        origin=Origin(xyz=(-0.03, 0.0, 0.11)),
    )

    outer_stage = model.part("outer_stage")
    _add_mesh_visual(
        outer_stage,
        _stage_shape(
            length=outer_len,
            outer_width=outer_width,
            outer_height=outer_height,
            wall=outer_wall,
            front_collar_len=0.12,
            rear_collar_len=0.10,
            collar_growth=0.018,
        ),
        "outer_stage.obj",
        "boom_yellow",
    )
    _add_tube_wall_collisions(
        outer_stage,
        prefix="outer",
        length=outer_len,
        outer_width=outer_width,
        outer_height=outer_height,
        wall=outer_wall,
    )
    outer_stage.inertial = Inertial.from_geometry(
        Box((outer_len, outer_width, outer_height)),
        mass=18.0,
        origin=Origin(xyz=(outer_len / 2.0, 0.0, outer_height / 2.0)),
    )

    mid_stage = model.part("mid_stage")
    _add_mesh_visual(
        mid_stage,
        _stage_shape(
            length=mid_len,
            outer_width=mid_width,
            outer_height=mid_height,
            wall=mid_wall,
            front_collar_len=0.10,
            rear_collar_len=0.08,
            collar_growth=0.014,
        ),
        "mid_stage.obj",
        "boom_yellow",
    )
    _add_tube_wall_collisions(
        mid_stage,
        prefix="mid",
        length=mid_len,
        outer_width=mid_width,
        outer_height=mid_height,
        wall=mid_wall,
    )
    mid_stage.inertial = Inertial.from_geometry(
        Box((mid_len, mid_width, mid_height)),
        mass=12.0,
        origin=Origin(xyz=(mid_len / 2.0, 0.0, mid_height / 2.0)),
    )

    inner_stage = model.part("inner_stage")
    _add_mesh_visual(
        inner_stage,
        _stage_shape(
            length=inner_len,
            outer_width=inner_width,
            outer_height=inner_height,
            wall=inner_wall,
            front_collar_len=0.08,
            rear_collar_len=0.06,
            collar_growth=0.012,
            nose_len=0.10,
        ),
        "inner_stage.obj",
        "boom_yellow",
    )
    _add_tube_wall_collisions(
        inner_stage,
        prefix="inner",
        length=inner_len,
        outer_width=inner_width,
        outer_height=inner_height,
        wall=inner_wall,
    )

    inner_stage.inertial = Inertial.from_geometry(
        Box((inner_len + 0.08, inner_width + 0.02, inner_height + 0.02)),
        mass=7.0,
        origin=Origin(xyz=((inner_len + 0.08) / 2.0, 0.0, (inner_height + 0.02) / 2.0)),
    )

    model.articulation(
        "base_to_outer",
        ArticulationType.FIXED,
        parent=base_bracket,
        child=outer_stage,
        origin=Origin(xyz=(-0.06, 0.0, 0.211)),
    )
    model.articulation(
        "outer_to_mid",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=mid_stage,
        origin=Origin(xyz=(0.0, 0.0, outer_to_mid_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.62,
            effort=1200.0,
            velocity=0.30,
        ),
    )
    model.articulation(
        "mid_to_inner",
        ArticulationType.PRISMATIC,
        parent=mid_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, mid_to_inner_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=0.55,
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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=128,
        overlap_tol=0.002,
        overlap_volume_tol=0.0,
    )
    ctx.expect_aabb_gap("outer_stage", "base_bracket", axis="z", max_gap=0.01, max_penetration=0.0)
    ctx.expect_aabb_overlap("outer_stage", "base_bracket", axes="xy", min_overlap=0.08)
    ctx.expect_origin_gap("outer_stage", "base_bracket", axis="z", min_gap=0.05)
    ctx.expect_origin_gap("mid_stage", "base_bracket", axis="z", min_gap=0.05)
    ctx.expect_origin_gap("inner_stage", "base_bracket", axis="z", min_gap=0.05)
    ctx.expect_origin_distance("mid_stage", "outer_stage", axes="xy", max_dist=0.02)
    ctx.expect_origin_distance("inner_stage", "mid_stage", axes="xy", max_dist=0.02)
    ctx.expect_aabb_overlap("mid_stage", "outer_stage", axes="xy", min_overlap=0.08)
    ctx.expect_aabb_overlap("inner_stage", "mid_stage", axes="xy", min_overlap=0.05)
    ctx.expect_aabb_overlap("inner_stage", "outer_stage", axes="xy", min_overlap=0.05)
    ctx.expect_joint_motion_axis(
        "outer_to_mid",
        "mid_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.04,
    )
    ctx.expect_joint_motion_axis(
        "mid_to_inner",
        "inner_stage",
        world_axis="x",
        direction="positive",
        min_delta=0.04,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
