from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.

# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_FOOT_X = 0.18
BASE_FOOT_Y = 0.18
BASE_FOOT_H = 0.04
PEDESTAL_X = 0.11
PEDESTAL_Y = 0.11
PEDESTAL_H = 0.035
MAST_BASE_Z = BASE_FOOT_H + PEDESTAL_H

BASE_OUTER_X = 0.09
BASE_OUTER_Y = 0.09
BASE_WALL = 0.006
BASE_LENGTH = 0.46

STAGE1_OUTER_X = 0.072
STAGE1_OUTER_Y = 0.072
STAGE1_WALL = 0.005
STAGE1_LENGTH = 0.48

STAGE2_OUTER_X = 0.056
STAGE2_OUTER_Y = 0.056
STAGE2_WALL = 0.0045
STAGE2_LENGTH = 0.42

STAGE3_OUTER_X = 0.042
STAGE3_OUTER_Y = 0.042
STAGE3_WALL = 0.004
STAGE3_LENGTH = 0.36

BASE_TO_STAGE1_Z = MAST_BASE_Z + 0.013
STAGE1_TO_STAGE2_Z = 0.012
STAGE2_TO_STAGE3_Z = 0.011

STAGE1_TRAVEL = 0.22
STAGE2_TRAVEL = 0.18
STAGE3_TRAVEL = 0.16


def _add_frame_band(
    part,
    *,
    outer_x: float,
    outer_y: float,
    wall: float,
    z_center: float,
    height: float,
    material,
) -> None:
    inner_y = max(outer_y - 2.0 * wall, wall)
    part.visual(
        Box((outer_x, wall, height)),
        origin=Origin(xyz=(0.0, outer_y / 2.0 - wall / 2.0, z_center)),
        material=material,
    )
    part.visual(
        Box((outer_x, wall, height)),
        origin=Origin(xyz=(0.0, -outer_y / 2.0 + wall / 2.0, z_center)),
        material=material,
    )
    part.visual(
        Box((wall, inner_y, height)),
        origin=Origin(xyz=(outer_x / 2.0 - wall / 2.0, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((wall, inner_y, height)),
        origin=Origin(xyz=(-outer_x / 2.0 + wall / 2.0, 0.0, z_center)),
        material=material,
    )


def _add_guide_pads(
    part,
    *,
    parent_outer_x: float,
    parent_outer_y: float,
    parent_wall: float,
    child_outer_x: float,
    child_outer_y: float,
    z_center: float,
    height: float,
    material,
) -> None:
    parent_inner_x = parent_outer_x - 2.0 * parent_wall
    parent_inner_y = parent_outer_y - 2.0 * parent_wall
    pad_x = max((parent_inner_x - child_outer_x) / 2.0 + 0.0005, 0.0015)
    pad_y = max((parent_inner_y - child_outer_y) / 2.0 + 0.0005, 0.0015)
    pad_span_y = max(child_outer_y - 0.008, child_outer_y * 0.72)
    pad_span_x = max(child_outer_x - 0.008, child_outer_x * 0.72)

    part.visual(
        Box((pad_x, pad_span_y, height)),
        origin=Origin(xyz=(child_outer_x / 2.0 + pad_x / 2.0, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((pad_x, pad_span_y, height)),
        origin=Origin(xyz=(-child_outer_x / 2.0 - pad_x / 2.0, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((pad_span_x, pad_y, height)),
        origin=Origin(xyz=(0.0, child_outer_y / 2.0 + pad_y / 2.0, z_center)),
        material=material,
    )
    part.visual(
        Box((pad_span_x, pad_y, height)),
        origin=Origin(xyz=(0.0, -child_outer_y / 2.0 - pad_y / 2.0, z_center)),
        material=material,
    )


def _add_cross_brace(
    part,
    *,
    span_x: float,
    span_y: float,
    brace_width: float,
    thickness: float,
    z_center: float,
    material,
) -> None:
    part.visual(
        Box((span_x, brace_width, thickness)),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=material,
    )
    part.visual(
        Box((brace_width, span_y, thickness)),
        origin=Origin(xyz=(0.0, 0.0, z_center)),
        material=material,
    )


def _add_telescoping_stage(
    part,
    *,
    outer_x: float,
    outer_y: float,
    wall: float,
    length: float,
    rail_material,
    collar_material,
    guide_material,
    collar_proud: float,
    collar_height: float,
    child_outer_x: float | None = None,
    child_outer_y: float | None = None,
    z_offset: float = 0.0,
    add_lower_plate: bool = False,
    add_upper_plate: bool = False,
) -> None:
    _add_frame_band(
        part,
        outer_x=outer_x,
        outer_y=outer_y,
        wall=wall,
        z_center=z_offset + length / 2.0,
        height=length,
        material=rail_material,
    )
    for z_center in (z_offset + collar_height / 2.0, z_offset + length - collar_height / 2.0):
        _add_frame_band(
            part,
            outer_x=outer_x + 2.0 * collar_proud,
            outer_y=outer_y + 2.0 * collar_proud,
            wall=wall + collar_proud,
            z_center=z_center,
            height=collar_height,
            material=collar_material,
        )
    if child_outer_x is not None and child_outer_y is not None:
        pad_height = collar_height * 0.78
        _add_guide_pads(
            part,
            parent_outer_x=outer_x,
            parent_outer_y=outer_y,
            parent_wall=wall,
            child_outer_x=child_outer_x,
            child_outer_y=child_outer_y,
            z_center=z_offset + collar_height / 2.0,
            height=pad_height,
            material=guide_material,
        )
        _add_guide_pads(
            part,
            parent_outer_x=outer_x,
            parent_outer_y=outer_y,
            parent_wall=wall,
            child_outer_x=child_outer_x,
            child_outer_y=child_outer_y,
            z_center=z_offset + length - collar_height / 2.0,
            height=pad_height,
            material=guide_material,
        )
    inner_x = max(outer_x - 2.0 * wall, wall)
    inner_y = max(outer_y - 2.0 * wall, wall)
    if add_lower_plate:
        _add_cross_brace(
            part,
            span_x=inner_x,
            span_y=inner_y,
            brace_width=max(min(inner_x, inner_y) * 0.34, 0.01),
            thickness=max(collar_height * 0.46, 0.008),
            z_center=z_offset + collar_height / 2.0,
            material=guide_material,
        )
    if add_upper_plate:
        _add_cross_brace(
            part,
            span_x=inner_x * 0.78,
            span_y=inner_y * 0.78,
            brace_width=max(min(inner_x, inner_y) * 0.28, 0.008),
            thickness=max(collar_height * 0.65, 0.012),
            z_center=z_offset + length - max(collar_height * 0.65, 0.012) / 2.0,
            material=collar_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_antenna_mast")

    base_steel = model.material("base_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    aluminum = model.material("painted_aluminum", rgba=(0.78, 0.81, 0.85, 1.0))
    collar_gray = model.material("collar_gray", rgba=(0.58, 0.62, 0.67, 1.0))
    guide_dark = model.material("guide_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    head_white = model.material("head_white", rgba=(0.90, 0.92, 0.93, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_FOOT_X, BASE_FOOT_Y, BASE_FOOT_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H / 2.0)),
        material=base_steel,
    )
    base.visual(
        Box((PEDESTAL_X, PEDESTAL_Y, PEDESTAL_H)),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_H + PEDESTAL_H / 2.0)),
        material=base_steel,
    )
    _add_telescoping_stage(
        base,
        outer_x=BASE_OUTER_X,
        outer_y=BASE_OUTER_Y,
        wall=BASE_WALL,
        length=BASE_LENGTH,
        rail_material=base_steel,
        collar_material=collar_gray,
        guide_material=guide_dark,
        collar_proud=0.004,
        collar_height=0.026,
        child_outer_x=STAGE1_OUTER_X,
        child_outer_y=STAGE1_OUTER_Y,
        z_offset=MAST_BASE_Z,
    )
    _add_frame_band(
        base,
        outer_x=BASE_OUTER_X + 0.024,
        outer_y=BASE_OUTER_Y + 0.024,
        wall=0.014,
        z_center=MAST_BASE_Z + 0.009,
        height=0.018,
        material=base_steel,
    )
    for z_offset in (MAST_BASE_Z + 0.035, MAST_BASE_Z + 0.07):
        _add_frame_band(
            base,
            outer_x=BASE_OUTER_X + 0.012,
            outer_y=BASE_OUTER_Y + 0.012,
            wall=0.012,
            z_center=z_offset,
            height=0.01,
            material=collar_gray,
        )
    base.inertial = Inertial.from_geometry(
        Box((BASE_FOOT_X, BASE_FOOT_Y, MAST_BASE_Z + BASE_LENGTH)),
        mass=8.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (MAST_BASE_Z + BASE_LENGTH) / 2.0,
            )
        ),
    )

    stage1 = model.part("stage1")
    _add_telescoping_stage(
        stage1,
        outer_x=STAGE1_OUTER_X,
        outer_y=STAGE1_OUTER_Y,
        wall=STAGE1_WALL,
        length=STAGE1_LENGTH,
        rail_material=aluminum,
        collar_material=collar_gray,
        guide_material=guide_dark,
        collar_proud=0.0035,
        collar_height=0.024,
        child_outer_x=STAGE2_OUTER_X,
        child_outer_y=STAGE2_OUTER_Y,
    )
    stage1.inertial = Inertial.from_geometry(
        Box((STAGE1_OUTER_X, STAGE1_OUTER_Y, STAGE1_LENGTH)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, STAGE1_LENGTH / 2.0)),
    )

    stage2 = model.part("stage2")
    _add_telescoping_stage(
        stage2,
        outer_x=STAGE2_OUTER_X,
        outer_y=STAGE2_OUTER_Y,
        wall=STAGE2_WALL,
        length=STAGE2_LENGTH,
        rail_material=aluminum,
        collar_material=collar_gray,
        guide_material=guide_dark,
        collar_proud=0.003,
        collar_height=0.022,
        child_outer_x=STAGE3_OUTER_X,
        child_outer_y=STAGE3_OUTER_Y,
    )
    stage2.inertial = Inertial.from_geometry(
        Box((STAGE2_OUTER_X, STAGE2_OUTER_Y, STAGE2_LENGTH)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, STAGE2_LENGTH / 2.0)),
    )

    stage3 = model.part("stage3")
    _add_telescoping_stage(
        stage3,
        outer_x=STAGE3_OUTER_X,
        outer_y=STAGE3_OUTER_Y,
        wall=STAGE3_WALL,
        length=STAGE3_LENGTH,
        rail_material=aluminum,
        collar_material=collar_gray,
        guide_material=guide_dark,
        collar_proud=0.0025,
        collar_height=0.02,
        add_lower_plate=True,
    )
    stage3.inertial = Inertial.from_geometry(
        Box((STAGE3_OUTER_X, STAGE3_OUTER_Y, STAGE3_LENGTH)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, STAGE3_LENGTH / 2.0)),
    )

    head = model.part("antenna_head")
    head.visual(
        Box((0.036, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=head_white,
    )
    head.visual(
        Box((0.028, 0.02, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=collar_gray,
    )
    head.visual(
        Cylinder(radius=0.0035, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=antenna_black,
    )
    head.inertial = Inertial.from_geometry(
        Box((0.036, 0.028, 0.12)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.PRISMATIC,
        parent="base",
        child="stage1",
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_STAGE1_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.PRISMATIC,
        parent="stage1",
        child="stage2",
        origin=Origin(xyz=(0.0, 0.0, STAGE1_TO_STAGE2_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.28,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.PRISMATIC,
        parent="stage2",
        child="stage3",
        origin=Origin(xyz=(0.0, 0.0, STAGE2_TO_STAGE3_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.32,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )
    model.articulation(
        "stage3_to_head",
        ArticulationType.FIXED,
        parent="stage3",
        child="antenna_head",
        origin=Origin(xyz=(0.0, 0.0, STAGE3_LENGTH)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.allow_overlap(
        "base",
        "stage1",
        reason="nested guide pads intentionally preload the first sliding section",
    )
    ctx.allow_overlap(
        "stage1",
        "stage2",
        reason="compact collar liners intentionally seat the second stage in the first",
    )
    ctx.allow_overlap(
        "stage2",
        "stage3",
        reason="compact collar liners intentionally seat the third stage in the second",
    )
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_far_from_geometry(tol=0.03)
    ctx.warn_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=96,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=96,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance("stage1", "base", axes="xy", max_dist=0.0015)
    ctx.expect_origin_distance("stage2", "stage1", axes="xy", max_dist=0.0015)
    ctx.expect_origin_distance("stage3", "stage2", axes="xy", max_dist=0.0015)
    ctx.expect_origin_distance("antenna_head", "stage3", axes="xy", max_dist=0.0015)

    ctx.expect_aabb_overlap("stage1", "base", axes="xy", min_overlap=0.06)
    ctx.expect_aabb_overlap("stage2", "stage1", axes="xy", min_overlap=0.045)
    ctx.expect_aabb_overlap("stage3", "stage2", axes="xy", min_overlap=0.033)
    ctx.expect_aabb_overlap("antenna_head", "stage3", axes="xy", min_overlap=0.02)

    ctx.expect_aabb_contact("stage1", "base")
    ctx.expect_aabb_contact("stage2", "stage1")
    ctx.expect_aabb_contact("stage3", "stage2")
    ctx.expect_aabb_gap(
        "antenna_head",
        "stage3",
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
    )

    ctx.expect_joint_motion_axis(
        "base_to_stage1",
        "stage1",
        world_axis="z",
        direction="positive",
        min_delta=0.08,
    )
    ctx.expect_joint_motion_axis(
        "stage1_to_stage2",
        "stage2",
        world_axis="z",
        direction="positive",
        min_delta=0.06,
    )
    ctx.expect_joint_motion_axis(
        "stage2_to_stage3",
        "stage3",
        world_axis="z",
        direction="positive",
        min_delta=0.05,
    )

    with ctx.pose(
        base_to_stage1=STAGE1_TRAVEL,
        stage1_to_stage2=STAGE2_TRAVEL,
        stage2_to_stage3=STAGE3_TRAVEL,
    ):
        ctx.expect_origin_distance("stage1", "base", axes="xy", max_dist=0.0015)
        ctx.expect_origin_distance("stage2", "stage1", axes="xy", max_dist=0.0015)
        ctx.expect_origin_distance("stage3", "stage2", axes="xy", max_dist=0.0015)
        ctx.expect_origin_distance("antenna_head", "stage3", axes="xy", max_dist=0.0015)
        ctx.expect_aabb_overlap("stage1", "base", axes="xy", min_overlap=0.06)
        ctx.expect_aabb_overlap("stage2", "stage1", axes="xy", min_overlap=0.045)
        ctx.expect_aabb_overlap("stage3", "stage2", axes="xy", min_overlap=0.033)
        ctx.expect_aabb_overlap("antenna_head", "stage3", axes="xy", min_overlap=0.02)
        ctx.expect_aabb_contact("stage1", "base")
        ctx.expect_aabb_contact("stage2", "stage1")
        ctx.expect_aabb_contact("stage3", "stage2")
        ctx.expect_aabb_gap(
            "antenna_head",
            "stage3",
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
        )
        ctx.expect_origin_gap("antenna_head", "base", axis="z", min_gap=0.95)

    with ctx.pose(base_to_stage1=STAGE1_TRAVEL * 0.5, stage1_to_stage2=STAGE2_TRAVEL * 0.5):
        ctx.expect_origin_distance("stage2", "stage1", axes="xy", max_dist=0.0015)
        ctx.expect_origin_distance("stage3", "stage2", axes="xy", max_dist=0.0015)
        ctx.expect_aabb_contact("stage2", "stage1")
        ctx.expect_aabb_contact("stage3", "stage2")
        ctx.expect_origin_gap("stage3", "base", axis="z", min_gap=0.3)

    return ctx.report()
# >>> USER_CODE_END

object_model = build_object_model()
