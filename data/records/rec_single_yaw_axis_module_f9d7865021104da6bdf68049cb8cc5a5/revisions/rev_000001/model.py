from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


BEAM_LENGTH = 0.34
BEAM_WIDTH = 0.085
BEAM_THICKNESS = 0.032
TOP_PAD_LENGTH = 0.120
TOP_PAD_WIDTH = 0.090
TOP_PAD_HEIGHT = 0.020
HOUSING_RADIUS = 0.052
HOUSING_HEIGHT = 0.050
HOUSING_BORE_RADIUS = 0.026
RIB_LENGTH = 0.110
RIB_THICKNESS = 0.012
RIB_HEIGHT = 0.036

JOINT_Z = -(BEAM_THICKNESS / 2.0 + HOUSING_HEIGHT)

SHAFT_RADIUS = 0.022
SHAFT_LENGTH = 0.020
HUB_RADIUS = 0.030
HUB_LENGTH = 0.012
PLATE_LENGTH = 0.110
PLATE_WIDTH = 0.080
PLATE_THICKNESS = 0.010
PLATE_CENTER_Z = -(SHAFT_LENGTH + HUB_LENGTH + PLATE_THICKNESS / 2.0)
PLATE_HOLE_RADIUS = 0.004


def _build_support_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_WIDTH, BEAM_THICKNESS)

    beam_slot_cutters = (
        cq.Workplane("XY")
        .pushPoints([(-0.110, 0.0), (0.110, 0.0)])
        .slot2D(0.026, 0.009, angle=90)
        .extrude(BEAM_THICKNESS + 0.004)
        .translate((0.0, 0.0, -(BEAM_THICKNESS + 0.004) / 2.0))
    )
    beam = beam.cut(beam_slot_cutters)

    top_pad = cq.Workplane("XY").box(TOP_PAD_LENGTH, TOP_PAD_WIDTH, TOP_PAD_HEIGHT).translate(
        (0.0, 0.0, BEAM_THICKNESS / 2.0 + TOP_PAD_HEIGHT / 2.0)
    )

    housing = (
        cq.Workplane("XY")
        .circle(HOUSING_RADIUS)
        .extrude(HOUSING_HEIGHT)
        .translate((0.0, 0.0, -BEAM_THICKNESS / 2.0 - HOUSING_HEIGHT))
    )
    housing_bore = (
        cq.Workplane("XY")
        .circle(HOUSING_BORE_RADIUS)
        .extrude(HOUSING_HEIGHT + 0.004)
        .translate((0.0, 0.0, -BEAM_THICKNESS / 2.0 - HOUSING_HEIGHT - 0.002))
    )
    housing = housing.cut(housing_bore)

    rib_z = -BEAM_THICKNESS / 2.0 - RIB_HEIGHT / 2.0 + 0.005
    rib_y = HOUSING_RADIUS * 0.55
    rib_left = cq.Workplane("XY").box(RIB_LENGTH, RIB_THICKNESS, RIB_HEIGHT).translate(
        (0.0, rib_y, rib_z)
    )
    rib_right = cq.Workplane("XY").box(RIB_LENGTH, RIB_THICKNESS, RIB_HEIGHT).translate(
        (0.0, -rib_y, rib_z)
    )

    return beam.union(top_pad).union(housing).union(rib_left).union(rib_right)


def _build_spindle_shape() -> cq.Workplane:
    top_flange = (
        cq.Workplane("XY")
        .circle(0.036)
        .extrude(0.004)
        .translate((0.0, 0.0, -0.004))
    )
    shaft = (
        cq.Workplane("XY")
        .circle(SHAFT_RADIUS)
        .extrude(SHAFT_LENGTH)
        .translate((0.0, 0.0, -SHAFT_LENGTH))
    )
    hub = (
        cq.Workplane("XY")
        .circle(HUB_RADIUS)
        .extrude(HUB_LENGTH)
        .translate((0.0, 0.0, -(SHAFT_LENGTH + HUB_LENGTH)))
    )
    return top_flange.union(shaft).union(hub)


def _build_tooling_plate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS).translate(
        (0.0, 0.0, PLATE_CENTER_Z)
    )

    hole_cutters = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.032, -0.020),
                (-0.032, 0.020),
                (0.032, -0.020),
                (0.032, 0.020),
            ]
        )
        .circle(PLATE_HOLE_RADIUS)
        .extrude(PLATE_THICKNESS + 0.004)
        .translate((0.0, 0.0, PLATE_CENTER_Z - (PLATE_THICKNESS + 0.004) / 2.0))
    )
    center_bore = (
        cq.Workplane("XY")
        .circle(0.010)
        .extrude(PLATE_THICKNESS + 0.004)
        .translate((0.0, 0.0, PLATE_CENTER_Z - (PLATE_THICKNESS + 0.004) / 2.0))
    )

    return plate.cut(hole_cutters).cut(center_bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_pan_stage")

    model.material("painted_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("machined_aluminum", rgba=(0.74, 0.76, 0.79, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        mesh_from_cadquery(_build_support_shape(), "support_frame"),
        material="painted_steel",
        name="support_structure",
    )

    rotary_stage = model.part("rotary_stage")
    rotary_stage.visual(
        mesh_from_cadquery(_build_spindle_shape(), "rotary_spindle"),
        material="machined_aluminum",
        name="rotary_spindle",
    )
    rotary_stage.visual(
        mesh_from_cadquery(_build_tooling_plate_shape(), "tooling_plate"),
        material="machined_aluminum",
        name="tooling_plate",
    )

    model.articulation(
        "support_to_rotary_stage",
        ArticulationType.REVOLUTE,
        parent=support_frame,
        child=rotary_stage,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
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

    support_frame = object_model.get_part("support_frame")
    rotary_stage = object_model.get_part("rotary_stage")
    pan_joint = object_model.get_articulation("support_to_rotary_stage")
    support_structure = support_frame.get_visual("support_structure")
    tooling_plate = rotary_stage.get_visual("tooling_plate")

    limits = pan_joint.motion_limits
    axis = tuple(round(value, 6) for value in pan_joint.axis)
    ctx.check(
        "pan joint uses the vertical support axis",
        axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -1.57
        and limits.upper >= 1.57,
        details=f"axis={pan_joint.axis}, limits={limits}",
    )

    ctx.expect_origin_gap(
        support_frame,
        rotary_stage,
        axis="z",
        min_gap=0.05,
        name="fixed support reads above the rotating member",
    )
    ctx.expect_gap(
        support_frame,
        rotary_stage,
        axis="z",
        min_gap=0.020,
        max_gap=0.050,
        positive_elem=support_structure,
        negative_elem=tooling_plate,
        name="tooling plate hangs below the fixed head",
    )
    ctx.expect_overlap(
        support_frame,
        rotary_stage,
        axes="xy",
        min_overlap=0.070,
        elem_a=support_structure,
        elem_b=tooling_plate,
        name="tooling plate stays centered beneath the support head",
    )

    rest_aabb = ctx.part_element_world_aabb(rotary_stage, elem=tooling_plate)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        quarter_turn_aabb = ctx.part_element_world_aabb(rotary_stage, elem=tooling_plate)

    rest_dx = rest_aabb[1][0] - rest_aabb[0][0] if rest_aabb is not None else None
    rest_dy = rest_aabb[1][1] - rest_aabb[0][1] if rest_aabb is not None else None
    quarter_dx = (
        quarter_turn_aabb[1][0] - quarter_turn_aabb[0][0]
        if quarter_turn_aabb is not None
        else None
    )
    quarter_dy = (
        quarter_turn_aabb[1][1] - quarter_turn_aabb[0][1]
        if quarter_turn_aabb is not None
        else None
    )
    ctx.check(
        "tooling plate visibly pans about the vertical axis",
        rest_dx is not None
        and rest_dy is not None
        and quarter_dx is not None
        and quarter_dy is not None
        and rest_dx > rest_dy + 0.015
        and quarter_dy > quarter_dx + 0.015,
        details=(
            f"rest_dx={rest_dx}, rest_dy={rest_dy}, "
            f"quarter_dx={quarter_dx}, quarter_dy={quarter_dy}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
