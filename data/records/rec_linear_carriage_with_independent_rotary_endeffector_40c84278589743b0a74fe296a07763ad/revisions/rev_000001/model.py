from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BEAM_LENGTH = 0.82
BEAM_WIDTH = 0.18
BEAM_HEIGHT = 0.10
RAIL_LENGTH = 0.76
RAIL_WIDTH = 0.035
RAIL_HEIGHT = 0.012
RAIL_OFFSET_Y = 0.055
RAIL_TOP_Z = BEAM_HEIGHT / 2.0 + RAIL_HEIGHT

SLIDE_LENGTH = 0.16
SLIDE_WIDTH = 0.21
SLIDE_DECK_THICKNESS = 0.045
SLIDE_SHOE_LENGTH = 0.065
SLIDE_SHOE_WIDTH = 0.045
SLIDE_SHOE_HEIGHT = 0.030
SLIDE_DECK_CENTER_Z = RAIL_TOP_Z + SLIDE_SHOE_HEIGHT + SLIDE_DECK_THICKNESS / 2.0

RETRACTED_SLIDE_X = 0.14
SLIDE_TRAVEL = 0.48

HEAD_HOUSING_CENTER_X = 0.030
HEAD_HOUSING_LENGTH = 0.11
HEAD_HOUSING_WIDTH = 0.12
HEAD_HOUSING_HEIGHT = 0.070
HEAD_HOUSING_CENTER_Z = (
    SLIDE_DECK_CENTER_Z + SLIDE_DECK_THICKNESS / 2.0 + HEAD_HOUSING_HEIGHT / 2.0
)

NOSE_RADIUS = 0.044
NOSE_LENGTH = 0.050
NOSE_CENTER_X = 0.110
NOSE_CENTER_Z = HEAD_HOUSING_CENTER_Z
ROTARY_JOINT_X = NOSE_CENTER_X + NOSE_LENGTH / 2.0
ROTARY_JOINT_Z = NOSE_CENTER_Z

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_transfer_axis")

    frame_material = model.material("frame_steel", color=(0.20, 0.22, 0.25))
    slide_material = model.material("machined_alloy", color=(0.77, 0.78, 0.80))
    head_material = model.material("head_finish", color=(0.16, 0.18, 0.20))

    frame = model.part("frame")
    frame.visual(
        Box((0.045, 0.28, 0.58)),
        origin=Origin(xyz=(-0.0225, 0.0, -0.10)),
        material=frame_material,
        name="rear_column",
    )
    frame.visual(
        Box((0.075, 0.22, 0.08)),
        origin=Origin(xyz=(-0.0075, 0.0, 0.145)),
        material=frame_material,
        name="top_bridge",
    )
    frame.visual(
        Box((0.10, 0.12, 0.07)),
        origin=Origin(xyz=(-0.01, 0.0, -0.18)),
        material=frame_material,
        name="lower_bridge",
    )
    frame.visual(
        Box((0.19, 0.085, 0.05)),
        origin=Origin(xyz=(-0.025, 0.075, -0.415)),
        material=frame_material,
        name="left_foot",
    )
    frame.visual(
        Box((0.19, 0.085, 0.05)),
        origin=Origin(xyz=(-0.025, -0.075, -0.415)),
        material=frame_material,
        name="right_foot",
    )
    frame.visual(
        Box((0.060, 0.075, 0.09)),
        origin=Origin(xyz=(0.02, RAIL_OFFSET_Y, 0.045)),
        material=frame_material,
        name="left_rear_saddle",
    )
    frame.visual(
        Box((0.060, 0.075, 0.09)),
        origin=Origin(xyz=(0.02, -RAIL_OFFSET_Y, 0.045)),
        material=frame_material,
        name="right_rear_saddle",
    )
    frame.visual(
        Box((BEAM_LENGTH, 0.05, 0.05)),
        origin=Origin(xyz=(BEAM_LENGTH / 2.0, 0.065, 0.025)),
        material=frame_material,
        name="left_way_beam",
    )
    frame.visual(
        Box((BEAM_LENGTH, 0.05, 0.05)),
        origin=Origin(xyz=(BEAM_LENGTH / 2.0, -0.065, 0.025)),
        material=frame_material,
        name="right_way_beam",
    )
    frame.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.03 + RAIL_LENGTH / 2.0, RAIL_OFFSET_Y, 0.056)),
        material=frame_material,
        name="left_rail",
    )
    frame.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.03 + RAIL_LENGTH / 2.0, -RAIL_OFFSET_Y, 0.056)),
        material=frame_material,
        name="right_rail",
    )
    frame.visual(
        Box((0.035, 0.17, 0.05)),
        origin=Origin(xyz=(0.8225, 0.0, 0.025)),
        material=frame_material,
        name="front_tie",
    )

    slide = model.part("slide")
    slide.visual(
        Box((SLIDE_LENGTH, SLIDE_WIDTH, SLIDE_DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, SLIDE_DECK_CENTER_Z)),
        material=slide_material,
        name="deck",
    )
    slide.visual(
        Box((SLIDE_SHOE_LENGTH, SLIDE_SHOE_WIDTH, SLIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(-0.035, RAIL_OFFSET_Y, RAIL_TOP_Z + SLIDE_SHOE_HEIGHT / 2.0)),
        material=slide_material,
        name="left_shoe",
    )
    slide.visual(
        Box((SLIDE_SHOE_LENGTH, SLIDE_SHOE_WIDTH, SLIDE_SHOE_HEIGHT)),
        origin=Origin(xyz=(-0.035, -RAIL_OFFSET_Y, RAIL_TOP_Z + SLIDE_SHOE_HEIGHT / 2.0)),
        material=slide_material,
        name="right_shoe",
    )
    slide.visual(
        Box((0.05, 0.11, 0.05)),
        origin=Origin(xyz=(-0.048, 0.0, SLIDE_DECK_CENTER_Z + 0.010)),
        material=slide_material,
        name="rear_carriage_block",
    )
    slide.visual(
        Box((HEAD_HOUSING_LENGTH, HEAD_HOUSING_WIDTH, HEAD_HOUSING_HEIGHT)),
        origin=Origin(xyz=(HEAD_HOUSING_CENTER_X, 0.0, HEAD_HOUSING_CENTER_Z)),
        material=slide_material,
        name="head_housing",
    )
    slide.visual(
        Box((0.065, 0.08, 0.03)),
        origin=Origin(xyz=(-0.01, 0.0, SLIDE_DECK_CENTER_Z + 0.055)),
        material=slide_material,
        name="cable_hump",
    )
    slide.visual(
        Cylinder(radius=NOSE_RADIUS, length=NOSE_LENGTH),
        origin=Origin(
            xyz=(NOSE_CENTER_X, 0.0, NOSE_CENTER_Z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=slide_material,
        name="nose_barrel",
    )

    rotary_head = model.part("rotary_head")
    rotary_head.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=head_material,
        name="flange",
    )
    rotary_head.visual(
        Cylinder(radius=0.035, length=0.050),
        origin=Origin(xyz=(0.037, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=head_material,
        name="spindle",
    )
    rotary_head.visual(
        Cylinder(radius=0.055, length=0.010),
        origin=Origin(xyz=(0.067, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=head_material,
        name="faceplate",
    )
    rotary_head.visual(
        Box((0.020, 0.020, 0.040)),
        origin=Origin(xyz=(0.068, 0.034, 0.0)),
        material=head_material,
        name="tool_pad",
    )
    rotary_head.visual(
        Box((0.016, 0.028, 0.016)),
        origin=Origin(xyz=(0.070, 0.0, 0.038)),
        material=head_material,
        name="key_block",
    )

    model.articulation(
        "frame_to_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide,
        origin=Origin(xyz=(RETRACTED_SLIDE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.60,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "slide_to_rotary_head",
        ArticulationType.REVOLUTE,
        parent=slide,
        child=rotary_head,
        origin=Origin(xyz=(ROTARY_JOINT_X, 0.0, ROTARY_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=4.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slide = object_model.get_part("slide")
    rotary_head = object_model.get_part("rotary_head")
    slide_joint = object_model.get_articulation("frame_to_slide")
    rotary_joint = object_model.get_articulation("slide_to_rotary_head")

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

    ctx.expect_contact(
        slide,
        frame,
        contact_tol=5e-4,
        name="slide_stage_is_supported_by_frame",
    )
    ctx.expect_contact(
        rotary_head,
        slide,
        contact_tol=5e-4,
        name="rotary_head_is_bearing_mounted_to_slide",
    )

    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        ctx.expect_contact(
            slide,
            frame,
            contact_tol=5e-4,
            name="slide_remains_supported_when_extended",
        )

    closed_slide_x = ctx.part_world_position(slide)[0]
    with ctx.pose({slide_joint: slide_joint.motion_limits.upper}):
        extended_slide_x = ctx.part_world_position(slide)[0]

    ctx.check(
        "slide_joint_moves_forward_along_beam",
        extended_slide_x > closed_slide_x + 0.45,
        f"expected forward slide travel > 0.45 m, got {extended_slide_x - closed_slide_x:.4f} m",
    )

    slide_axis_ok = tuple(round(value, 6) for value in slide_joint.axis) == (1.0, 0.0, 0.0)
    ctx.check(
        "slide_joint_is_prismatic_x_axis",
        slide_joint.articulation_type == ArticulationType.PRISMATIC and slide_axis_ok,
        f"expected prismatic +X slide joint, got type={slide_joint.articulation_type} axis={slide_joint.axis}",
    )

    rotary_span = rotary_joint.motion_limits.upper - rotary_joint.motion_limits.lower
    rotary_axis_ok = tuple(round(value, 6) for value in rotary_joint.axis) == (1.0, 0.0, 0.0)
    ctx.check(
        "rotary_head_joint_is_carried_revolute_x_axis",
        rotary_joint.articulation_type == ArticulationType.REVOLUTE
        and rotary_axis_ok
        and rotary_span >= 2.0 * pi - 1e-3,
        (
            "expected a carried rotary head with a wide X-axis revolute output; "
            f"got type={rotary_joint.articulation_type} axis={rotary_joint.axis} span={rotary_span:.4f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
