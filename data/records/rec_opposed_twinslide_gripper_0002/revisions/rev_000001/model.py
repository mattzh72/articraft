from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

FRAME_X = 0.314
FRAME_Y = 0.120
BASE_T = 0.018

OUTER_BLOCK_X = 0.024
OUTER_BLOCK_Y = 0.090
OUTER_BLOCK_H = 0.032
OUTER_BLOCK_CX = 0.157

INNER_BLOCK_X = 0.022
INNER_BLOCK_Y = 0.090
INNER_BLOCK_H = 0.026
INNER_BLOCK_CX = 0.024

ROD_R = 0.006
ROD_LEN = 0.110
FRAME_TOP = BASE_T + OUTER_BLOCK_H
ROD_Z = FRAME_TOP + ROD_R
ROD_FRONT_Y = 0.028
ROD_REAR_Y = -0.028
LEFT_RAIL_X = -0.090
RIGHT_RAIL_X = 0.090

SLIDE_X = 0.070
SLIDE_Y = 0.082
SLIDE_Z = 0.030
LEFT_SLIDE_REST_X = -0.095
RIGHT_SLIDE_REST_X = 0.095
SLIDE_TRAVEL = 0.022

STOP_X = 0.024
STOP_Y = 0.040
STOP_Z = 0.020
LEFT_STOP_X = -0.142
RIGHT_STOP_X = 0.142

HOUSING_X = 0.056
HOUSING_Y = 0.024
HOUSING_Z = 0.018
HOUSING_FRONT_Y = 0.055
HOUSING_REAR_Y = -0.055
COVER_X = 0.060
COVER_Y = 0.028
COVER_Z = 0.004

JAW_BASE_X = 0.010
JAW_PLATE_X = 0.006
JAW_PAD_X = 0.004
JAW_Y = 0.058
JAW_PAD_Y = 0.040
JAW_Z = 0.024
JAW_PAD_Z = 0.018
JAW_MOUNT_Z = 0.025


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _fixed_joint(model: ArticulatedObject, name: str, parent, child, xyz, rpy=(0.0, 0.0, 0.0)) -> None:
    model.articulation(
        name,
        ArticulationType.FIXED,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz, rpy=rpy),
    )


def _frame_shape() -> cq.Workplane:
    frame = cq.Workplane("XY").box(FRAME_X, FRAME_Y, BASE_T, centered=(True, True, False))

    for x_sign in (-1.0, 1.0):
        frame = frame.union(
            cq.Workplane("XY")
            .box(OUTER_BLOCK_X, OUTER_BLOCK_Y, OUTER_BLOCK_H, centered=(True, True, False))
            .translate((x_sign * OUTER_BLOCK_CX, 0.0, BASE_T))
        )
        frame = frame.union(
            cq.Workplane("XY")
            .box(INNER_BLOCK_X, INNER_BLOCK_Y, INNER_BLOCK_H, centered=(True, True, False))
            .translate((x_sign * INNER_BLOCK_CX, 0.0, BASE_T))
        )

    for y_pos in (HOUSING_FRONT_Y, HOUSING_REAR_Y):
        frame = frame.union(
            cq.Workplane("XY")
            .box(HOUSING_X + 0.010, HOUSING_Y + 0.012, 0.006, centered=(True, True, False))
            .translate((0.0, y_pos, BASE_T))
        )

    frame = frame.cut(
        cq.Workplane("XY")
        .box(0.092, 0.050, 0.010, centered=(True, True, False))
        .translate((0.0, 0.0, 0.008))
    )
    frame = frame.cut(
        cq.Workplane("XY")
        .box(0.070, 0.024, 0.024, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_T))
    )

    for x_sign in (-1.0, 1.0):
        frame = frame.cut(
            cq.Workplane("XY")
            .box(0.070, 0.022, 0.008, centered=(True, True, False))
            .translate((x_sign * 0.090, 0.0, BASE_T + 0.006))
        )

    return frame


def _guide_rail_shape() -> cq.Workplane:
    return cq.Workplane("YZ").circle(ROD_R).extrude(ROD_LEN / 2.0, both=True)


def _slide_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(SLIDE_X, SLIDE_Y, SLIDE_Z)
    body = body.cut(cq.Workplane("XY").box(0.040, 0.046, 0.010).translate((0.0, 0.0, 0.010)))
    body = body.cut(cq.Workplane("XY").box(0.030, 0.032, 0.012).translate((0.0, 0.0, -0.011)))

    for y_pos in (-0.034, 0.034):
        body = body.cut(cq.Workplane("XY").box(0.040, 0.008, 0.014).translate((0.0, y_pos, -0.002)))

    rod_clear_r = ROD_R + 0.0004
    for y_pos in (ROD_REAR_Y, ROD_FRONT_Y):
        body = body.cut(
            cq.Workplane("YZ")
            .circle(rod_clear_r)
            .extrude((SLIDE_X + 0.008) / 2.0, both=True)
            .translate((0.0, y_pos, 0.0))
        )

    return body


def _slide_bushing_shape() -> cq.Workplane:
    bushings = None
    bushing_outer_r = ROD_R + 0.0004
    bushing_inner_r = ROD_R + 0.00015
    bushing_len = 0.014

    for x_pos in (-0.021, 0.021):
        for y_pos in (ROD_REAR_Y, ROD_FRONT_Y):
            bushing = (
                cq.Workplane("YZ")
                .circle(bushing_outer_r)
                .circle(bushing_inner_r)
                .extrude(bushing_len / 2.0, both=True)
                .translate((x_pos, y_pos, 0.0))
            )
            bushings = bushing if bushings is None else bushings.union(bushing)

    return bushings


def _jaw_bracket_shape() -> cq.Workplane:
    bracket = cq.Workplane("XY").box(JAW_BASE_X, 0.046, 0.020, centered=(False, True, True))
    bracket = bracket.union(
        cq.Workplane("XY")
        .box(JAW_PLATE_X, JAW_Y, JAW_Z, centered=(False, True, True))
        .translate((JAW_BASE_X, 0.0, 0.0))
    )
    bracket = bracket.union(
        cq.Workplane("XY")
        .box(0.008, 0.032, 0.010, centered=(False, True, True))
        .translate((0.008, 0.0, -0.008))
    )
    bracket = bracket.union(
        cq.Workplane("XY")
        .box(0.008, 0.032, 0.008, centered=(False, True, True))
        .translate((0.008, 0.0, 0.008))
    )
    return bracket


def _jaw_pad_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(JAW_PAD_X, JAW_PAD_Y, JAW_PAD_Z, centered=(False, True, True))
        .translate((JAW_BASE_X + JAW_PLATE_X, 0.0, 0.0))
    )


def _housing_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").box(HOUSING_X, HOUSING_Y, HOUSING_Z)
    housing = housing.cut(
        cq.Workplane("XY")
        .box(HOUSING_X - 0.014, HOUSING_Y - 0.008, 0.008)
        .translate((0.0, 0.0, 0.005))
    )
    return housing


def _cover_plate_shape() -> cq.Workplane:
    cover = cq.Workplane("XY").box(COVER_X, COVER_Y, COVER_Z)
    cover = cover.cut(
        cq.Workplane("XY")
        .box(COVER_X - 0.018, 0.010, 0.003)
        .translate((0.0, 0.0, 0.001))
    )
    return cover


def _cover_hardware_shape() -> cq.Workplane:
    hardware = None
    for x_pos in (-0.020, 0.020):
        for y_pos in (-0.008, 0.008):
            screw = (
                cq.Workplane("XY")
                .circle(0.0025)
                .extrude(0.002, both=False)
                .translate((x_pos, y_pos, COVER_Z / 2.0))
            )
            hardware = screw if hardware is None else hardware.union(screw)
    return hardware


def _stop_block_shape() -> cq.Workplane:
    stop = cq.Workplane("XY").box(STOP_X, STOP_Y, STOP_Z)
    stop = stop.cut(
        cq.Workplane("XY")
        .box(STOP_X - 0.010, STOP_Y - 0.014, 0.008)
        .translate((0.0, 0.0, 0.004))
    )
    return stop


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="opposed_twin_slide_gripper", assets=ASSETS)

    frame_gray = model.material("frame_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    carriage_steel = model.material("carriage_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.14, 0.15, 0.17, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.48, 0.28, 1.0))
    jaw_pad_mat = model.material("jaw_pad", rgba=(0.17, 0.18, 0.19, 1.0))

    frame = model.part("frame")
    frame.visual(_mesh(_frame_shape(), "frame.obj"), material=frame_gray, name="body")

    left_front_rail = model.part("left_front_rail")
    left_front_rail.visual(_mesh(_guide_rail_shape(), "guide_rail.obj"), material=rail_steel, name="rod")
    left_rear_rail = model.part("left_rear_rail")
    left_rear_rail.visual(_mesh(_guide_rail_shape(), "guide_rail.obj"), material=rail_steel, name="rod")
    right_front_rail = model.part("right_front_rail")
    right_front_rail.visual(_mesh(_guide_rail_shape(), "guide_rail.obj"), material=rail_steel, name="rod")
    right_rear_rail = model.part("right_rear_rail")
    right_rear_rail.visual(_mesh(_guide_rail_shape(), "guide_rail.obj"), material=rail_steel, name="rod")

    front_sync_housing = model.part("front_sync_housing")
    front_sync_housing.visual(_mesh(_housing_shape(), "sync_housing.obj"), material=black_oxide, name="body")
    rear_sync_housing = model.part("rear_sync_housing")
    rear_sync_housing.visual(_mesh(_housing_shape(), "sync_housing.obj"), material=black_oxide, name="body")

    front_access_cover = model.part("front_access_cover")
    front_access_cover.visual(_mesh(_cover_plate_shape(), "access_cover.obj"), material=carriage_steel, name="plate")
    front_access_cover.visual(_mesh(_cover_hardware_shape(), "cover_hardware.obj"), material=black_oxide, name="hardware")
    rear_access_cover = model.part("rear_access_cover")
    rear_access_cover.visual(_mesh(_cover_plate_shape(), "access_cover.obj"), material=carriage_steel, name="plate")
    rear_access_cover.visual(_mesh(_cover_hardware_shape(), "cover_hardware.obj"), material=black_oxide, name="hardware")

    left_stop_block = model.part("left_stop_block")
    left_stop_block.visual(_mesh(_stop_block_shape(), "stop_block.obj"), material=carriage_steel, name="body")
    right_stop_block = model.part("right_stop_block")
    right_stop_block.visual(_mesh(_stop_block_shape(), "stop_block.obj"), material=carriage_steel, name="body")

    left_slide = model.part("left_slide")
    left_slide.visual(_mesh(_slide_body_shape(), "slide_body.obj"), material=carriage_steel, name="body")
    left_slide.visual(_mesh(_slide_bushing_shape(), "slide_bushings.obj"), material=bronze, name="bushings")
    right_slide = model.part("right_slide")
    right_slide.visual(_mesh(_slide_body_shape(), "slide_body.obj"), material=carriage_steel, name="body")
    right_slide.visual(_mesh(_slide_bushing_shape(), "slide_bushings.obj"), material=bronze, name="bushings")

    left_jaw = model.part("left_jaw")
    left_jaw.visual(_mesh(_jaw_bracket_shape(), "jaw_bracket.obj"), material=carriage_steel, name="bracket")
    left_jaw.visual(_mesh(_jaw_pad_shape(), "jaw_pad.obj"), material=jaw_pad_mat, name="pad")
    right_jaw = model.part("right_jaw")
    right_jaw.visual(_mesh(_jaw_bracket_shape(), "jaw_bracket.obj"), material=carriage_steel, name="bracket")
    right_jaw.visual(_mesh(_jaw_pad_shape(), "jaw_pad.obj"), material=jaw_pad_mat, name="pad")

    _fixed_joint(model, "frame_to_left_front_rail", frame, left_front_rail, (LEFT_RAIL_X, ROD_FRONT_Y, ROD_Z))
    _fixed_joint(model, "frame_to_left_rear_rail", frame, left_rear_rail, (LEFT_RAIL_X, ROD_REAR_Y, ROD_Z))
    _fixed_joint(model, "frame_to_right_front_rail", frame, right_front_rail, (RIGHT_RAIL_X, ROD_FRONT_Y, ROD_Z))
    _fixed_joint(model, "frame_to_right_rear_rail", frame, right_rear_rail, (RIGHT_RAIL_X, ROD_REAR_Y, ROD_Z))

    _fixed_joint(
        model,
        "frame_to_front_sync_housing",
        frame,
        front_sync_housing,
        (0.0, HOUSING_FRONT_Y, FRAME_TOP + HOUSING_Z / 2.0),
    )
    _fixed_joint(
        model,
        "frame_to_rear_sync_housing",
        frame,
        rear_sync_housing,
        (0.0, HOUSING_REAR_Y, FRAME_TOP + HOUSING_Z / 2.0),
    )
    _fixed_joint(
        model,
        "front_sync_housing_to_front_access_cover",
        front_sync_housing,
        front_access_cover,
        (0.0, 0.0, HOUSING_Z / 2.0 + COVER_Z / 2.0),
    )
    _fixed_joint(
        model,
        "rear_sync_housing_to_rear_access_cover",
        rear_sync_housing,
        rear_access_cover,
        (0.0, 0.0, HOUSING_Z / 2.0 + COVER_Z / 2.0),
    )

    _fixed_joint(model, "frame_to_left_stop_block", frame, left_stop_block, (LEFT_STOP_X, 0.0, FRAME_TOP + STOP_Z / 2.0))
    _fixed_joint(model, "frame_to_right_stop_block", frame, right_stop_block, (RIGHT_STOP_X, 0.0, FRAME_TOP + STOP_Z / 2.0))

    model.articulation(
        "frame_to_left_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=left_slide,
        origin=Origin(xyz=(LEFT_SLIDE_REST_X, 0.0, ROD_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.12, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "frame_to_right_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=right_slide,
        origin=Origin(xyz=(RIGHT_SLIDE_REST_X, 0.0, ROD_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.12, lower=0.0, upper=SLIDE_TRAVEL),
    )

    _fixed_joint(model, "left_slide_to_left_jaw", left_slide, left_jaw, (SLIDE_X / 2.0, 0.0, JAW_MOUNT_Z))
    _fixed_joint(
        model,
        "right_slide_to_right_jaw",
        right_slide,
        right_jaw,
        (-SLIDE_X / 2.0, 0.0, JAW_MOUNT_Z),
        rpy=(0.0, 0.0, math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    left_front_rail = object_model.get_part("left_front_rail")
    left_rear_rail = object_model.get_part("left_rear_rail")
    right_front_rail = object_model.get_part("right_front_rail")
    right_rear_rail = object_model.get_part("right_rear_rail")
    front_sync_housing = object_model.get_part("front_sync_housing")
    rear_sync_housing = object_model.get_part("rear_sync_housing")
    front_access_cover = object_model.get_part("front_access_cover")
    rear_access_cover = object_model.get_part("rear_access_cover")
    left_stop_block = object_model.get_part("left_stop_block")
    right_stop_block = object_model.get_part("right_stop_block")
    left_slide = object_model.get_part("left_slide")
    right_slide = object_model.get_part("right_slide")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")

    left_slide_joint = object_model.get_articulation("frame_to_left_slide")
    right_slide_joint = object_model.get_articulation("frame_to_right_slide")

    left_pad = left_jaw.get_visual("pad")
    right_pad = right_jaw.get_visual("pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(left_front_rail, frame, name="left_front_rail_is_seated_to_frame")
    ctx.expect_contact(left_rear_rail, frame, name="left_rear_rail_is_seated_to_frame")
    ctx.expect_contact(right_front_rail, frame, name="right_front_rail_is_seated_to_frame")
    ctx.expect_contact(right_rear_rail, frame, name="right_rear_rail_is_seated_to_frame")

    ctx.expect_gap(front_sync_housing, frame, axis="z", max_gap=0.0005, max_penetration=1e-6, name="front_housing_mounts_flush")
    ctx.expect_gap(rear_sync_housing, frame, axis="z", max_gap=0.0005, max_penetration=1e-6, name="rear_housing_mounts_flush")
    ctx.expect_gap(front_access_cover, front_sync_housing, axis="z", max_gap=0.0005, max_penetration=1e-6, name="front_cover_mounts_flush")
    ctx.expect_gap(rear_access_cover, rear_sync_housing, axis="z", max_gap=0.0005, max_penetration=1e-6, name="rear_cover_mounts_flush")

    ctx.expect_gap(left_slide, left_stop_block, axis="x", max_gap=0.0005, max_penetration=1e-6, name="left_slide_rests_against_outer_stop")
    ctx.expect_gap(right_stop_block, right_slide, axis="x", max_gap=0.0005, max_penetration=1e-6, name="right_slide_rests_against_outer_stop")

    ctx.expect_contact(left_jaw, left_slide, name="left_jaw_is_bolted_to_slide")
    ctx.expect_contact(right_jaw, right_slide, name="right_jaw_is_bolted_to_slide")

    ctx.expect_overlap(left_slide, left_front_rail, axes="yz", min_overlap=0.010, name="left_slide_tracks_front_rail_in_yz")
    ctx.expect_overlap(left_slide, left_rear_rail, axes="yz", min_overlap=0.010, name="left_slide_tracks_rear_rail_in_yz")
    ctx.expect_overlap(right_slide, right_front_rail, axes="yz", min_overlap=0.010, name="right_slide_tracks_front_rail_in_yz")
    ctx.expect_overlap(right_slide, right_rear_rail, axes="yz", min_overlap=0.010, name="right_slide_tracks_rear_rail_in_yz")

    ctx.expect_gap(
        right_jaw,
        left_jaw,
        axis="x",
        positive_elem=right_pad,
        negative_elem=left_pad,
        min_gap=0.079,
        max_gap=0.081,
        name="jaw_pad_gap_at_rest",
    )

    with ctx.pose({left_slide_joint: SLIDE_TRAVEL, right_slide_joint: SLIDE_TRAVEL}):
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            positive_elem=right_pad,
            negative_elem=left_pad,
            min_gap=0.035,
            max_gap=0.037,
            name="jaw_pad_gap_at_closed_pose",
        )
        ctx.expect_overlap(left_slide, left_front_rail, axes="yz", min_overlap=0.010, name="left_slide_remains_guided_when_closed")
        ctx.expect_overlap(right_slide, right_front_rail, axes="yz", min_overlap=0.010, name="right_slide_remains_guided_when_closed")

        left_pos = ctx.part_world_position(left_slide)
        right_pos = ctx.part_world_position(right_slide)
        left_ok = left_pos is not None and abs(left_pos[0] - (LEFT_SLIDE_REST_X + SLIDE_TRAVEL)) < 1e-6
        right_ok = right_pos is not None and abs(right_pos[0] - (RIGHT_SLIDE_REST_X - SLIDE_TRAVEL)) < 1e-6
        symmetry_ok = left_pos is not None and right_pos is not None and abs(left_pos[0] + right_pos[0]) < 1e-6
        ctx.check(
            "symmetric_slide_travel_at_closed_pose",
            left_ok and right_ok and symmetry_ok,
            details=f"left_pos={left_pos}, right_pos={right_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
