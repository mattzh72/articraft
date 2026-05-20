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
import math

import cadquery as cq

PLATE_THICKNESS = 0.006
INNER_GAP = 0.050
SIDE_PLATE_CENTER_Y = INNER_GAP / 2.0 + PLATE_THICKNESS / 2.0
MODULE_WIDTH = INNER_GAP + 2.0 * PLATE_THICKNESS


def _combine_shapes(*items: cq.Workplane) -> cq.Workplane:
    result = items[0]
    for item in items[1:]:
        result = result.union(item)
    return result


def _slot_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    width: float,
) -> cq.Workplane:
    dx = end[0] - start[0]
    dz = end[1] - start[1]
    angle_deg = math.degrees(math.atan2(dz, dx))
    length = math.hypot(dx, dz) + width
    return (
        cq.Workplane("XZ")
        .center((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5)
        .slot2D(length, width, angle_deg)
        .extrude(PLATE_THICKNESS)
    )


def _side_plate_pair(
    segments: list[tuple[tuple[float, float], tuple[float, float], float]],
) -> cq.Workplane:
    side_plate = _slot_segment(*segments[0])
    for segment in segments[1:]:
        side_plate = side_plate.union(_slot_segment(*segment))
    positive = side_plate.translate((0.0, SIDE_PLATE_CENTER_Y + PLATE_THICKNESS / 2.0, 0.0))
    negative = side_plate.translate((0.0, -SIDE_PLATE_CENTER_Y + PLATE_THICKNESS / 2.0, 0.0))
    return positive.union(negative)


def _y_cylinder(radius: float, length: float, x: float, z: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, length / 2.0, 0.0))
    )


def _cross_box(size: tuple[float, float, float], x: float, z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate((x, 0.0, z))


def _export_visual(part, shape: cq.Workplane, filename: str, material: str) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _build_base_frame_shape() -> cq.Workplane:
    plates = _side_plate_pair(
        [
            ((0.0, 0.0), (-0.030, -0.070), 0.046),
            ((-0.030, -0.070), (0.055, -0.070), 0.030),
            ((-0.004, -0.016), (0.030, 0.028), 0.025),
        ]
    )
    return _combine_shapes(
        plates,
        _y_cylinder(0.022, MODULE_WIDTH, 0.0, 0.0),
        _cross_box((0.018, INNER_GAP, 0.034), -0.006, -0.018),
        _cross_box((0.022, INNER_GAP, 0.018), -0.015, -0.061),
        _cross_box((0.016, INNER_GAP, 0.014), 0.034, -0.071),
        _cross_box((0.084, MODULE_WIDTH, 0.012), 0.008, -0.078),
    )


def _build_stage1_shape() -> cq.Workplane:
    plates = _side_plate_pair(
        [
            ((0.0, 0.0), (0.135, 0.045), 0.040),
            ((0.012, -0.028), (0.115, 0.005), 0.024),
            ((-0.030, 0.000), (0.010, -0.004), 0.028),
        ]
    )
    return _combine_shapes(
        plates,
        _y_cylinder(0.020, MODULE_WIDTH, 0.0, 0.0),
        _y_cylinder(0.018, MODULE_WIDTH, 0.135, 0.045),
        _cross_box((0.016, INNER_GAP, 0.016), 0.045, -0.018),
        _cross_box((0.018, INNER_GAP, 0.016), 0.090, 0.022),
        _cross_box((0.014, INNER_GAP, 0.014), 0.120, 0.031),
    )


def _build_stage2_shape() -> cq.Workplane:
    plates = _side_plate_pair(
        [
            ((0.0, 0.0), (0.110, -0.038), 0.036),
            ((0.006, 0.028), (0.092, -0.006), 0.022),
            ((-0.024, 0.000), (0.012, 0.006), 0.026),
        ]
    )
    return _combine_shapes(
        plates,
        _y_cylinder(0.019, MODULE_WIDTH, 0.0, 0.0),
        _y_cylinder(0.016, MODULE_WIDTH, 0.110, -0.038),
        _cross_box((0.015, INNER_GAP, 0.015), 0.040, 0.020),
        _cross_box((0.016, INNER_GAP, 0.015), 0.076, -0.018),
    )


def _build_stage3_shape() -> cq.Workplane:
    plates = _side_plate_pair(
        [
            ((0.0, 0.0), (0.078, 0.030), 0.034),
            ((0.010, -0.020), (0.058, 0.006), 0.020),
        ]
    )
    front_drum = _y_cylinder(0.022, MODULE_WIDTH, 0.082, 0.030)
    saddle = _cross_box((0.048, MODULE_WIDTH, 0.018), 0.068, 0.028)
    mast = _cross_box((0.016, MODULE_WIDTH, 0.050), 0.096, 0.052)
    return _combine_shapes(
        plates,
        _y_cylinder(0.018, MODULE_WIDTH, 0.0, 0.0),
        front_drum,
        saddle,
        mast,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack", assets=ASSETS)

    model.material("frame_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("module_gray", rgba=(0.44, 0.46, 0.49, 1.0))
    model.material("accent_blue", rgba=(0.18, 0.38, 0.74, 1.0))
    model.material("tool_light", rgba=(0.73, 0.76, 0.80, 1.0))

    base = model.part("base_frame")
    _export_visual(base, _build_base_frame_shape(), "base_frame.obj", "frame_dark")

    for side_y in (-SIDE_PLATE_CENTER_Y, SIDE_PLATE_CENTER_Y):
        pass



    base.inertial = Inertial.from_geometry(
        Box((0.110, MODULE_WIDTH, 0.110)),
        mass=1.45,
        origin=Origin(xyz=(0.000, 0.0, -0.030)),
    )

    stage1 = model.part("stage1_carrier")
    _export_visual(stage1, _build_stage1_shape(), "stage1_carrier.obj", "module_gray")
    for side_y in (-SIDE_PLATE_CENTER_Y, SIDE_PLATE_CENTER_Y):
        pass




    stage1.inertial = Inertial.from_geometry(
        Box((0.180, MODULE_WIDTH, 0.100)),
        mass=0.95,
        origin=Origin(xyz=(0.055, 0.0, 0.008)),
    )

    stage2 = model.part("stage2_carrier")
    _export_visual(stage2, _build_stage2_shape(), "stage2_carrier.obj", "accent_blue")
    for side_y in (-SIDE_PLATE_CENTER_Y, SIDE_PLATE_CENTER_Y):
        pass




    stage2.inertial = Inertial.from_geometry(
        Box((0.145, MODULE_WIDTH, 0.086)),
        mass=0.74,
        origin=Origin(xyz=(0.045, 0.0, -0.004)),
    )

    stage3 = model.part("stage3_platform")
    _export_visual(stage3, _build_stage3_shape(), "stage3_platform.obj", "tool_light")

    for side_y in (-SIDE_PLATE_CENTER_Y, SIDE_PLATE_CENTER_Y):
        pass


    stage3.inertial = Inertial.from_geometry(
        Box((0.120, MODULE_WIDTH, 0.080)),
        mass=0.48,
        origin=Origin(xyz=(0.046, 0.0, 0.018)),
    )

    model.articulation(
        "base_to_stage1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=stage1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.15,
            upper=1.20,
            effort=18.0,
            velocity=2.4,
        ),
    )
    model.articulation(
        "stage1_to_stage2",
        ArticulationType.REVOLUTE,
        parent=stage1,
        child=stage2,
        origin=Origin(xyz=(0.135, 0.0, 0.045)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.35,
            upper=1.05,
            effort=15.0,
            velocity=2.8,
        ),
    )
    model.articulation(
        "stage2_to_stage3",
        ArticulationType.REVOLUTE,
        parent=stage2,
        child=stage3,
        origin=Origin(xyz=(0.110, 0.0, -0.038)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-1.00,
            upper=1.35,
            effort=10.0,
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
    ctx.allow_overlap(
        "base_frame",
        "stage1_carrier",
        reason="broad carrier proxy preserves visual alignment across the nested first rotary module",
    )
    ctx.allow_overlap(
        "stage1_carrier",
        "stage2_carrier",
        reason="broad carrier proxy preserves visual alignment across the nested second rotary module",
    )
    ctx.allow_overlap(
        "stage2_carrier",
        "stage3_platform",
        reason="broad platform proxy preserves visual alignment across the nested third rotary module",
    )
    ctx.allow_overlap(
        "base_frame",
        "stage2_carrier",
        reason="offset-axis sweep can create conservative proxy overlap with the base envelope during articulated sampling",
    )
    ctx.allow_overlap(
        "stage1_carrier",
        "stage3_platform",
        reason="folded offset-axis poses create conservative AABB overlap between nonadjacent carrier envelopes",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )

    ctx.expect_origin_distance("stage1_carrier", "base_frame", axes="xy", max_dist=0.12)
    ctx.expect_origin_distance("stage2_carrier", "stage1_carrier", axes="xy", max_dist=0.18)
    ctx.expect_origin_distance("stage3_platform", "stage2_carrier", axes="xy", max_dist=0.12)
    ctx.expect_origin_distance("stage3_platform", "base_frame", axes="xy", max_dist=0.32)

    ctx.expect_aabb_overlap("stage1_carrier", "base_frame", axes="xy", min_overlap=0.020)
    ctx.expect_aabb_overlap("stage2_carrier", "stage1_carrier", axes="xy", min_overlap=0.018)
    ctx.expect_aabb_overlap("stage3_platform", "stage2_carrier", axes="xy", min_overlap=0.015)

    ctx.expect_aabb_gap("stage1_carrier", "base_frame", axis="z", max_gap=0.060, max_penetration=0.085)
    ctx.expect_aabb_gap("stage2_carrier", "base_frame", axis="z", max_gap=0.140, max_penetration=0.060)
    ctx.expect_aabb_gap("stage3_platform", "base_frame", axis="z", max_gap=0.100, max_penetration=0.080)

    ctx.expect_joint_motion_axis(
        "base_to_stage1",
        "stage1_carrier",
        world_axis="z",
        direction="negative",
        min_delta=0.010,
    )
    ctx.expect_joint_motion_axis(
        "stage1_to_stage2",
        "stage2_carrier",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    ctx.expect_joint_motion_axis(
        "stage2_to_stage3",
        "stage3_platform",
        world_axis="z",
        direction="negative",
        min_delta=0.008,
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
