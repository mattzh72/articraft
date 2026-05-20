from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.30
BASE_HEIGHT = 0.055
PEDESTAL_RADIUS = 0.10
PEDESTAL_HEIGHT = 0.022

STAGE_LENGTH = 0.32
STAGE_WIDTH = 0.26
STAGE_HEIGHT = 0.040
STAGE_MOUNT_PAD_LENGTH = 0.17
STAGE_MOUNT_PAD_WIDTH = 0.10
STAGE_MOUNT_PAD_HEIGHT = 0.006
STAGE_MOUNT_PAD_Y = 0.082
STAGE_HUB_RADIUS = 0.060
STAGE_HUB_HEIGHT = 0.008

SUPPORT_AXIS_OFFSET_Y = 0.090
SUPPORT_AXIS_OFFSET_Z = STAGE_HEIGHT + STAGE_MOUNT_PAD_HEIGHT + 0.130
SUPPORT_FOOT_BOTTOM_Z = -0.130
SUPPORT_FOOT_THICKNESS = 0.014

LOWER_STAGE_ORIGIN_Z = BASE_HEIGHT + PEDESTAL_HEIGHT


def _base_shape() -> cq.Workplane:
    plinth = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT)
        .edges("|Z")
        .fillet(0.022)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
    )

    pedestal = (
        cq.Workplane("XY")
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_HEIGHT)
        .translate((0.0, 0.0, BASE_HEIGHT))
    )

    top_break = (
        cq.Workplane("XY")
        .box(0.24, 0.16, 0.006)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BASE_HEIGHT - 0.003))
    )

    return plinth.cut(top_break).union(pedestal)


def _lower_stage_shape() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(STAGE_LENGTH, STAGE_WIDTH, STAGE_HEIGHT)
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, 0.0, STAGE_HEIGHT / 2.0))
    )

    central_recess = (
        cq.Workplane("XY")
        .box(0.19, 0.12, 0.006)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, -0.015, STAGE_HEIGHT - 0.003))
    )

    center_hub = (
        cq.Workplane("XY")
        .circle(STAGE_HUB_RADIUS)
        .extrude(STAGE_HUB_HEIGHT)
        .translate((0.0, 0.0, STAGE_HEIGHT))
    )

    support_pad = (
        cq.Workplane("XY")
        .box(STAGE_MOUNT_PAD_LENGTH, STAGE_MOUNT_PAD_WIDTH, STAGE_MOUNT_PAD_HEIGHT)
        .edges("|Z")
        .fillet(0.014)
        .translate(
            (
                0.0,
                STAGE_MOUNT_PAD_Y,
                STAGE_HEIGHT + (STAGE_MOUNT_PAD_HEIGHT / 2.0),
            )
        )
    )

    return body.cut(central_recess).union(center_hub).union(support_pad)


def _support_cheek_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(0.18, 0.10, SUPPORT_FOOT_THICKNESS)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, -0.025, SUPPORT_FOOT_BOTTOM_Z + (SUPPORT_FOOT_THICKNESS / 2.0)))
    )

    cheek_plate = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.082, -0.116),
                (0.082, -0.116),
                (0.070, -0.066),
                (0.048, -0.022),
                (0.030, -0.006),
                (-0.030, -0.006),
                (-0.048, -0.022),
                (-0.070, -0.066),
            ]
        )
        .close()
        .extrude(0.024)
        .translate((0.0, -0.036, 0.0))
    )

    housing_outer = (
        cq.Workplane("XY")
        .circle(0.052)
        .extrude(0.046)
        .translate((0.0, 0.0, -0.046))
    )
    housing_inner = (
        cq.Workplane("XY")
        .circle(0.028)
        .extrude(0.046)
        .translate((0.0, 0.0, -0.046))
    )
    housing = housing_outer.cut(housing_inner)

    axis_relief = (
        cq.Workplane("XY")
        .circle(0.032)
        .extrude(0.090)
        .translate((0.0, 0.0, -0.080))
    )

    return foot.union(cheek_plate).union(housing).cut(axis_relief)


def _upper_output_shape() -> cq.Workplane:
    pilot = (
        cq.Workplane("XY")
        .circle(0.020)
        .extrude(0.018)
    )
    flange = cq.Workplane("XY").circle(0.048).extrude(0.008)
    drum = (
        cq.Workplane("XY")
        .circle(0.040)
        .extrude(0.040)
        .translate((0.0, 0.0, 0.006))
    )
    faceplate = (
        cq.Workplane("XY")
        .circle(0.056)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.044))
    )
    nose = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.016)
        .translate((0.0, 0.0, 0.052))
    )

    body = pilot.union(flange).union(drum).union(faceplate).union(nose)

    bolt_recesses = (
        cq.Workplane("XY")
        .polarArray(0.036, 0.0, 360.0, 4)
        .circle(0.0035)
        .extrude(0.002)
        .translate((0.0, 0.0, 0.052))
    )

    return body.cut(bolt_recesses)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_body_offset_stack")

    model.material("body_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("stage_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("cheek_gray", rgba=(0.58, 0.61, 0.65, 1.0))
    model.material("output_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("index_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_body"),
        material="body_graphite",
        name="base_body",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_lower_stage_shape(), "lower_stage_body"),
        material="stage_aluminum",
        name="lower_stage_body",
    )

    support_cheek = model.part("support_cheek")
    support_cheek.visual(
        mesh_from_cadquery(_support_cheek_shape(), "support_cheek_body"),
        material="cheek_gray",
        name="support_cheek_body",
    )

    upper_output = model.part("upper_output")
    upper_output.visual(
        mesh_from_cadquery(_upper_output_shape(), "upper_output_body"),
        material="output_silver",
        name="upper_output_body",
    )
    upper_output.visual(
        Box((0.018, 0.010, 0.006)),
        origin=Origin(xyz=(0.038, 0.0, 0.055)),
        material="index_black",
        name="index_lug",
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.6,
            lower=-2.1,
            upper=2.1,
        ),
    )
    model.articulation(
        "lower_stage_to_support_cheek",
        ArticulationType.FIXED,
        parent=lower_stage,
        child=support_cheek,
        origin=Origin(xyz=(0.0, SUPPORT_AXIS_OFFSET_Y, SUPPORT_AXIS_OFFSET_Z)),
    )
    model.articulation(
        "support_cheek_to_upper_output",
        ArticulationType.CONTINUOUS,
        parent=support_cheek,
        child=upper_output,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    support_cheek = object_model.get_part("support_cheek")
    upper_output = object_model.get_part("upper_output")

    base_to_lower = object_model.get_articulation("base_to_lower_stage")
    output_rotary = object_model.get_articulation("support_cheek_to_upper_output")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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

    ctx.check(
        "rotary_axes_are_parallel_vertical",
        tuple(base_to_lower.axis) == (0.0, 0.0, 1.0)
        and tuple(output_rotary.axis) == (0.0, 0.0, 1.0),
        details=(
            f"expected both rotary axes to be vertical and parallel; "
            f"got lower={base_to_lower.axis}, upper={output_rotary.axis}"
        ),
    )

    ctx.expect_contact(
        lower_stage,
        base,
        contact_tol=0.002,
        name="lower_stage_is_supported_by_base_pedestal",
    )
    ctx.expect_contact(
        support_cheek,
        lower_stage,
        contact_tol=0.002,
        name="support_cheek_is_mounted_on_stage_pad",
    )
    ctx.expect_contact(
        upper_output,
        support_cheek,
        contact_tol=0.002,
        name="upper_output_is_carried_by_support_cheek",
    )
    ctx.expect_origin_distance(
        support_cheek,
        lower_stage,
        axes="xy",
        min_dist=0.075,
        max_dist=0.105,
        name="upper_axis_is_laterally_offset_from_lower_axis",
    )

    with ctx.pose({base_to_lower: math.pi / 2.0}):
        cheek_pos = ctx.part_world_position(support_cheek)
        ctx.check(
            "lower_stage_positive_rotation_swings_cheek_counterclockwise",
            cheek_pos is not None and cheek_pos[0] < -0.065 and abs(cheek_pos[1]) < 0.03,
            details=f"support cheek position at +90 deg was {cheek_pos}",
        )

    with ctx.pose({output_rotary: math.pi / 2.0}):
        lug_aabb = ctx.part_element_world_aabb(upper_output, elem="index_lug")
        lug_center = _aabb_center(lug_aabb)
        output_origin = ctx.part_world_position(upper_output)
        ctx.check(
            "secondary_output_positive_rotation_carries_index_lug_counterclockwise",
            lug_center is not None
            and output_origin is not None
            and abs(lug_center[0] - output_origin[0]) < 0.015
            and (lug_center[1] - output_origin[1]) > 0.028,
            details=(
                f"upper output origin={output_origin}, "
                f"index lug center after +90 deg={lug_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
