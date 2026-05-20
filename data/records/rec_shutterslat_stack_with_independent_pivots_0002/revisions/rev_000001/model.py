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
FRAME_OUTER_WIDTH = 0.90
FRAME_OUTER_HEIGHT = 1.10
FRAME_DEPTH = 0.12
FRAME_SIDE_RAIL = 0.045
FRAME_TOP_RAIL = 0.055

SLAT_COUNT = 7
SLAT_PITCH = 0.13
SLAT_HEIGHT = 0.105
SLAT_DEPTH = 0.016
SLAT_SIDE_CLEARANCE = 0.003
SLAT_ROTATION_LIMIT = 1.05
SLAT_AXIS_OFFSET_Y = 0.007


def _frame_inner_width() -> float:
    return FRAME_OUTER_WIDTH - 2.0 * FRAME_SIDE_RAIL


def _frame_inner_height() -> float:
    return FRAME_OUTER_HEIGHT - 2.0 * FRAME_TOP_RAIL


def _slat_levels() -> list[float]:
    mid = (SLAT_COUNT - 1) / 2.0
    return [SLAT_PITCH * (index - mid) for index in range(SLAT_COUNT)]


def _make_frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)
    opening = cq.Workplane("XY").box(
        _frame_inner_width(),
        FRAME_DEPTH + 0.02,
        _frame_inner_height(),
    )
    frame = frame.cut(opening)

    boss_depth = 0.010
    boss_radius = 0.015
    left_inner_x = -_frame_inner_width() / 2.0
    right_inner_x = _frame_inner_width() / 2.0

    for z_level in _slat_levels():
        left_boss = (
            cq.Workplane("YZ")
            .center(0.0, z_level)
            .circle(boss_radius)
            .extrude(boss_depth)
            .translate((left_inner_x - boss_depth, 0.0, 0.0))
        )
        right_boss = (
            cq.Workplane("YZ")
            .center(0.0, z_level)
            .circle(boss_radius)
            .extrude(boss_depth)
            .translate((right_inner_x, 0.0, 0.0))
        )
        frame = frame.union(left_boss).union(right_boss)

    return frame


def _make_slat_shape() -> cq.Workplane:
    slat_span = _frame_inner_width() - 2.0 * SLAT_SIDE_CLEARANCE
    return cq.Workplane("YZ").ellipse(SLAT_DEPTH / 2.0, SLAT_HEIGHT / 2.0).extrude(slat_span)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_louver_shutter", assets=ASSETS)
    model.material("frame_finish", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("slat_finish", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "frame.obj", assets=ASSETS), material="frame_finish"
    )




    frame.inertial = Inertial.from_geometry(
        Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)),
        mass=7.5,
    )

    slat_mesh = mesh_from_cadquery(_make_slat_shape(), "slat.obj", assets=ASSETS)
    slat_span = _frame_inner_width() - 2.0 * SLAT_SIDE_CLEARANCE
    joint_x = -_frame_inner_width() / 2.0 + SLAT_SIDE_CLEARANCE

    for index, z_level in enumerate(_slat_levels(), start=1):
        slat_name = f"slat_{index:02d}"
        joint_name = f"frame_to_slat_{index:02d}"

        slat = model.part(slat_name)
        slat.visual(
            slat_mesh,
            origin=Origin(xyz=(0.0, SLAT_AXIS_OFFSET_Y, 0.0)),
            material="slat_finish",
        )

        slat.inertial = Inertial.from_geometry(
            Box((slat_span, SLAT_DEPTH, 0.090)),
            mass=0.32,
            origin=Origin(xyz=(slat_span / 2.0, SLAT_AXIS_OFFSET_Y, 0.0)),
        )

        model.articulation(
            joint_name,
            ArticulationType.REVOLUTE,
            parent=frame,
            child=slat,
            origin=Origin(xyz=(joint_x, 0.0, z_level)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                lower=-SLAT_ROTATION_LIMIT,
                upper=SLAT_ROTATION_LIMIT,
                effort=2.5,
                velocity=2.0,
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

    slat_names = [f"slat_{index:02d}" for index in range(1, SLAT_COUNT + 1)]
    joint_names = [f"frame_to_slat_{index:02d}" for index in range(1, SLAT_COUNT + 1)]

    for slat_name in slat_names:
        ctx.expect_aabb_overlap(slat_name, "frame", axes="xy", min_overlap=0.01)

    for upper_name, lower_name in zip(slat_names[1:], slat_names[:-1]):
        ctx.expect_aabb_gap(upper_name, lower_name, axis="z", max_gap=0.05, max_penetration=0.0)

    for joint_name, slat_name in zip(joint_names, slat_names):
        ctx.expect_joint_motion_axis(
            joint_name,
            slat_name,
            world_axis="z",
            direction="positive",
            min_delta=0.01,
        )

    ctx.expect_origin_gap(slat_names[-1], slat_names[0], axis="z", min_gap=0.55)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
