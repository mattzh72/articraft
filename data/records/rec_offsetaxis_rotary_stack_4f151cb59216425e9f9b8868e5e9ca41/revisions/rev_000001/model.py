from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.018
LOWER_AXIS_Z = 0.055
PEDESTAL_SHOULDER_TOP = 0.044
PEDESTAL_RADIUS = 0.062
LOWER_SPINDLE_RADIUS = 0.028

LOWER_STAGE_RADIUS = 0.135
LOWER_STAGE_BOTTOM = 0.000
LOWER_STAGE_THICKNESS = 0.018
LOWER_HUB_OUTER_RADIUS = 0.056
LOWER_HUB_INNER_RADIUS = 0.032
LOWER_HUB_BOTTOM = 0.000
LOWER_HUB_THICKNESS = 0.026

ARM_AXIS_OFFSET = 0.24
ARM_BEAM_START_X = 0.055
ARM_BEAM_END_X = 0.222
ARM_BEAM_WIDTH = 0.070
ARM_BEAM_BOTTOM = 0.014
ARM_BEAM_HEIGHT = 0.032
UPPER_SUPPORT_RADIUS = 0.036
UPPER_SUPPORT_TOP = 0.102
UPPER_SPINDLE_RADIUS = 0.018
UPPER_SPINDLE_TOP = 0.112
UPPER_SUPPORT_FLANGE_RADIUS = 0.046
UPPER_SUPPORT_FLANGE_THICKNESS = 0.010

UPPER_STAGE_RADIUS = 0.090
UPPER_STAGE_BOTTOM = 0.000
UPPER_STAGE_THICKNESS = 0.015
UPPER_HUB_OUTER_RADIUS = 0.036
UPPER_HUB_INNER_RADIUS = 0.022
UPPER_HUB_BOTTOM = 0.000
UPPER_HUB_THICKNESS = 0.021


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .rect(BASE_LENGTH, BASE_WIDTH)
        .extrude(BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.018)
    )

    pedestal = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS)
        .circle(PEDESTAL_RADIUS)
        .extrude(PEDESTAL_SHOULDER_TOP - BASE_THICKNESS)
        .faces(">Z")
        .workplane()
        .circle(LOWER_SPINDLE_RADIUS)
        .extrude(LOWER_AXIS_Z - PEDESTAL_SHOULDER_TOP)
    )

    top_pocket = (
        cq.Workplane("XY")
        .workplane(offset=BASE_THICKNESS - 0.004)
        .rect(0.220, 0.110)
        .extrude(0.004)
    )

    return plate.union(pedestal).cut(top_pocket)


def _lower_platform_shape() -> cq.Workplane:
    lower_disk = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_STAGE_BOTTOM)
        .circle(LOWER_STAGE_RADIUS)
        .extrude(LOWER_STAGE_THICKNESS)
    )

    lower_hub_outer = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_HUB_BOTTOM)
        .circle(LOWER_HUB_OUTER_RADIUS)
        .extrude(LOWER_HUB_THICKNESS)
    )
    lower_hub_cutter = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_HUB_BOTTOM - 0.001)
        .circle(LOWER_HUB_INNER_RADIUS)
        .extrude(LOWER_HUB_THICKNESS + 0.002)
    )
    lower_hub = lower_hub_outer.cut(lower_hub_cutter)

    beam = (
        cq.Workplane("XY")
        .workplane(offset=ARM_BEAM_BOTTOM)
        .center(0.5 * (ARM_BEAM_START_X + ARM_BEAM_END_X), 0.0)
        .rect(ARM_BEAM_END_X - ARM_BEAM_START_X, ARM_BEAM_WIDTH)
        .extrude(ARM_BEAM_HEIGHT)
    )

    lower_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.040, 0.010),
                (ARM_AXIS_OFFSET - 0.020, 0.010),
                (ARM_AXIS_OFFSET - 0.020, 0.062),
                (0.156, 0.050),
                (0.086, 0.028),
            ]
        )
        .close()
        .extrude(0.018, both=True)
    )

    upper_support = (
        cq.Workplane("XY")
        .center(ARM_AXIS_OFFSET, 0.0)
        .circle(UPPER_SUPPORT_RADIUS)
        .extrude(UPPER_SUPPORT_TOP)
    )

    upper_support_flange = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_SUPPORT_TOP - UPPER_SUPPORT_FLANGE_THICKNESS)
        .center(ARM_AXIS_OFFSET, 0.0)
        .circle(UPPER_SUPPORT_FLANGE_RADIUS)
        .extrude(UPPER_SUPPORT_FLANGE_THICKNESS)
    )

    upper_spindle = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_SUPPORT_TOP)
        .center(ARM_AXIS_OFFSET, 0.0)
        .circle(UPPER_SPINDLE_RADIUS)
        .extrude(UPPER_SPINDLE_TOP - UPPER_SUPPORT_TOP)
    )

    arm_saddle = (
        cq.Workplane("XY")
        .workplane(offset=0.040)
        .center(ARM_AXIS_OFFSET - 0.010, 0.0)
        .rect(0.070, 0.088)
        .extrude(0.020)
    )

    return (
        lower_disk.union(lower_hub)
        .union(beam)
        .union(lower_web)
        .union(upper_support)
        .union(upper_support_flange)
        .union(upper_spindle)
        .union(arm_saddle)
    )


def _upper_stage_shape() -> cq.Workplane:
    upper_disk = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_BOTTOM)
        .circle(UPPER_STAGE_RADIUS)
        .extrude(UPPER_STAGE_THICKNESS)
    )

    upper_hub_outer = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_HUB_BOTTOM)
        .circle(UPPER_HUB_OUTER_RADIUS)
        .extrude(UPPER_HUB_THICKNESS)
    )
    upper_hub_cutter = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_HUB_BOTTOM - 0.001)
        .circle(UPPER_HUB_INNER_RADIUS)
        .extrude(UPPER_HUB_THICKNESS + 0.002)
    )
    upper_hub = upper_hub_outer.cut(upper_hub_cutter)

    tooling_pad = (
        cq.Workplane("XY")
        .workplane(offset=0.013)
        .center(0.054, 0.0)
        .rect(0.058, 0.044)
        .extrude(0.013)
    )

    pad_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.018, 0.013),
                (0.082, 0.013),
                (0.066, 0.026),
                (0.028, 0.026),
            ]
        )
        .close()
        .extrude(0.024, both=True)
    )

    center_recess_cutter = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_STAGE_BOTTOM + UPPER_STAGE_THICKNESS - 0.0035)
        .circle(0.050)
        .extrude(0.0035)
    )

    return upper_disk.union(upper_hub).union(tooling_pad).union(pad_web).cut(center_recess_cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="edge_arm_offset_rotary_stack")

    model.material("base_dark", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("carrier_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    model.material("stage_silver", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("index_orange", rgba=(0.85, 0.48, 0.14, 1.0))

    ground_base = model.part("ground_base")
    ground_base.visual(
        mesh_from_cadquery(_base_shape(), "ground_base"),
        material="base_dark",
        name="base_shell",
    )

    lower_platform = model.part("lower_platform")
    lower_platform.visual(
        mesh_from_cadquery(_lower_platform_shape(), "lower_platform"),
        material="carrier_gray",
        name="lower_shell",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_shape(), "upper_stage"),
        material="stage_silver",
        name="upper_shell",
    )

    model.articulation(
        "base_to_lower_platform",
        ArticulationType.REVOLUTE,
        parent=ground_base,
        child=lower_platform,
        origin=Origin(xyz=(0.0, 0.0, LOWER_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.20, upper=2.20, effort=60.0, velocity=1.4),
    )
    model.articulation(
        "lower_platform_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=lower_platform,
        child=upper_stage,
        origin=Origin(xyz=(ARM_AXIS_OFFSET, 0.0, UPPER_SPINDLE_TOP)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.60, upper=2.60, effort=24.0, velocity=2.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ground_base = object_model.get_part("ground_base")
    lower_platform = object_model.get_part("lower_platform")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("base_to_lower_platform")
    upper_joint = object_model.get_articulation("lower_platform_to_upper_stage")

    ctx.check(
        "both revolute axes are parallel vertical shafts",
        lower_joint.axis == (0.0, 0.0, 1.0) and upper_joint.axis == (0.0, 0.0, 1.0),
        details=f"lower_axis={lower_joint.axis}, upper_axis={upper_joint.axis}",
    )

    ctx.expect_origin_distance(
        upper_stage,
        lower_platform,
        axes="xy",
        min_dist=ARM_AXIS_OFFSET - 0.005,
        max_dist=ARM_AXIS_OFFSET + 0.005,
        name="upper axis stays laterally offset from the lower axis",
    )
    ctx.expect_origin_gap(
        upper_stage,
        lower_platform,
        axis="z",
        min_gap=UPPER_SPINDLE_TOP - 0.002,
        max_gap=UPPER_SPINDLE_TOP + 0.002,
        name="upper rotary axis sits above the lower platform on its support arm",
    )
    ctx.expect_overlap(
        lower_platform,
        ground_base,
        axes="xy",
        min_overlap=0.11,
        name="lower rotary carrier remains centered over the grounded base support",
    )
    ctx.expect_overlap(
        upper_stage,
        lower_platform,
        axes="xy",
        min_overlap=0.04,
        name="upper stage overlaps its arm support footprint",
    )

    rest_upper_pos = ctx.part_world_position(upper_stage)
    with ctx.pose({lower_joint: 0.85}):
        swung_upper_pos = ctx.part_world_position(upper_stage)

    ctx.check(
        "lower rotary platform swings the offset upper axis around the base axis",
        rest_upper_pos is not None
        and swung_upper_pos is not None
        and swung_upper_pos[1] > 0.15
        and swung_upper_pos[0] < rest_upper_pos[0] - 0.04,
        details=f"rest_upper={rest_upper_pos}, swung_upper={swung_upper_pos}",
    )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    def _aabb_span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
        if aabb is None:
            return None
        mins, maxs = aabb
        return maxs[axis] - mins[axis]

    rest_stage_origin = ctx.part_world_position(upper_stage)
    rest_stage_aabb = ctx.part_world_aabb(upper_stage)
    rest_stage_center = _aabb_center(rest_stage_aabb)
    rest_stage_y_span = _aabb_span(rest_stage_aabb, 1)
    with ctx.pose({upper_joint: 1.10}):
        turned_stage_origin = ctx.part_world_position(upper_stage)
        turned_stage_aabb = ctx.part_world_aabb(upper_stage)
        turned_stage_center = _aabb_center(turned_stage_aabb)
        turned_stage_y_span = _aabb_span(turned_stage_aabb, 1)

    ctx.check(
        "upper stage rotates independently about its own offset axis",
        rest_stage_origin is not None
        and turned_stage_origin is not None
        and rest_stage_center is not None
        and turned_stage_center is not None
        and rest_stage_y_span is not None
        and turned_stage_y_span is not None
        and max(abs(turned_stage_origin[i] - rest_stage_origin[i]) for i in range(3)) < 1e-6
        and turned_stage_y_span > rest_stage_y_span + 0.04,
        details=(
            f"rest_stage_origin={rest_stage_origin}, turned_stage_origin={turned_stage_origin}, "
            f"rest_stage_center={rest_stage_center}, turned_stage_center={turned_stage_center}, "
            f"rest_stage_y_span={rest_stage_y_span}, turned_stage_y_span={turned_stage_y_span}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
