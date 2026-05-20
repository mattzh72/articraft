from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


TOP_LENGTH = 2.40
TOP_DEPTH = 1.20
TOP_THICKNESS = 0.04
TOP_CORNER_RADIUS = 0.05
TOP_CENTER_Z = 0.72

FRAME_LENGTH = 2.02
FRAME_DEPTH = 0.82
FRAME_HEIGHT = 0.082
FRAME_BEAM_WIDTH = 0.09
FRAME_CENTER_Z = 0.661
FRAME_GUSSET_X = 0.18
FRAME_GUSSET_Y = 0.15
FRAME_GUSSET_H = 0.05

SLEEVE_OUTER_X = 0.11
SLEEVE_OUTER_Y = 0.075
SLEEVE_WALL = 0.005
SLEEVE_HEIGHT = 0.56
SLEEVE_BOTTOM_Z = 0.08
SLEEVE_CENTER_Z = SLEEVE_BOTTOM_Z + SLEEVE_HEIGHT / 2.0

STAGE_MEMBER_X = 0.082
STAGE_MEMBER_Y = 0.052
STAGE_MEMBER_HEIGHT = 0.52
STAGE_UP_FROM_JOINT = 0.46
STAGE_DOWN_FROM_JOINT = STAGE_MEMBER_HEIGHT - STAGE_UP_FROM_JOINT
STAGE_MEMBER_CENTER_Z = (STAGE_UP_FROM_JOINT - STAGE_DOWN_FROM_JOINT) / 2.0
LEG_TRAVEL = 0.28

FOOT_PAD_X = 0.115
FOOT_PAD_Y = 0.078
FOOT_PAD_THICKNESS = 0.02
FOOT_PAD_CENTER_Z = -STAGE_DOWN_FROM_JOINT - FOOT_PAD_THICKNESS / 2.0 + 0.001
WIPER_COLLAR_X = 0.105
WIPER_COLLAR_Y = 0.07
WIPER_COLLAR_THICKNESS = 0.01
WIPER_COLLAR_CENTER_Z = -WIPER_COLLAR_THICKNESS / 2.0

COLUMN_X = 0.88
COLUMN_Y = 0.30

POD_LENGTH = 0.18
POD_DEPTH = 0.06
POD_HEIGHT = 0.045
POD_X = 0.88
POD_Y = FRAME_DEPTH / 2.0 + POD_DEPTH / 2.0 - 0.015
POD_Z = 0.644

KNOB_RADIUS = 0.026
KNOB_THICKNESS = 0.016
KNOB_SHAFT_RADIUS = 0.008
KNOB_SHAFT_LENGTH = 0.014
POINTER_WIDTH = 0.014
POINTER_DEPTH = 0.0035
POINTER_HEIGHT = 0.006

LEG_SPECS = (
    ("front_right_stage", "desk_to_front_right_stage", "front_right_outer_sleeve", COLUMN_X, COLUMN_Y),
    ("rear_right_stage", "desk_to_rear_right_stage", "rear_right_outer_sleeve", COLUMN_X, -COLUMN_Y),
    ("front_left_stage", "desk_to_front_left_stage", "front_left_outer_sleeve", -COLUMN_X, COLUMN_Y),
    ("rear_left_stage", "desk_to_rear_left_stage", "rear_left_outer_sleeve", -COLUMN_X, -COLUMN_Y),
)


def _box(length: float, depth: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, depth, height)


def _translated_box(
    length: float,
    depth: float,
    height: float,
    center_xyz: tuple[float, float, float],
) -> cq.Workplane:
    return _box(length, depth, height).translate(center_xyz)


def _top_shape() -> cq.Workplane:
    return (
        _box(TOP_LENGTH, TOP_DEPTH, TOP_THICKNESS)
        .edges("|Z")
        .fillet(TOP_CORNER_RADIUS)
    )


def _underframe_shape() -> cq.Workplane:
    frame = _translated_box(FRAME_LENGTH, FRAME_BEAM_WIDTH, FRAME_HEIGHT, (0.0, FRAME_DEPTH / 2.0 - FRAME_BEAM_WIDTH / 2.0, 0.0))
    frame = frame.union(
        _translated_box(FRAME_LENGTH, FRAME_BEAM_WIDTH, FRAME_HEIGHT, (0.0, -FRAME_DEPTH / 2.0 + FRAME_BEAM_WIDTH / 2.0, 0.0))
    )
    frame = frame.union(
        _translated_box(FRAME_BEAM_WIDTH, FRAME_DEPTH - 2.0 * FRAME_BEAM_WIDTH, FRAME_HEIGHT, (FRAME_LENGTH / 2.0 - FRAME_BEAM_WIDTH / 2.0, 0.0, 0.0))
    )
    frame = frame.union(
        _translated_box(FRAME_BEAM_WIDTH, FRAME_DEPTH - 2.0 * FRAME_BEAM_WIDTH, FRAME_HEIGHT, (-FRAME_LENGTH / 2.0 + FRAME_BEAM_WIDTH / 2.0, 0.0, 0.0))
    )
    frame = frame.union(_translated_box(FRAME_LENGTH - 0.36, 0.12, 0.06, (0.0, 0.0, -0.011)))
    frame = frame.union(_translated_box(0.10, FRAME_DEPTH - 2.0 * FRAME_BEAM_WIDTH + 0.02, 0.06, (0.46, 0.0, -0.011)))
    frame = frame.union(_translated_box(0.10, FRAME_DEPTH - 2.0 * FRAME_BEAM_WIDTH + 0.02, 0.06, (-0.46, 0.0, -0.011)))

    for x_pos, y_pos in ((COLUMN_X, COLUMN_Y), (COLUMN_X, -COLUMN_Y), (-COLUMN_X, COLUMN_Y), (-COLUMN_X, -COLUMN_Y)):
        frame = frame.union(
            _translated_box(FRAME_GUSSET_X, FRAME_GUSSET_Y, FRAME_GUSSET_H, (x_pos, y_pos, -FRAME_HEIGHT / 2.0 + FRAME_GUSSET_H / 2.0 - 0.004))
        )

    return frame


def _column_sleeve_shape() -> cq.Workplane:
    outer = _box(SLEEVE_OUTER_X, SLEEVE_OUTER_Y, SLEEVE_HEIGHT)
    inner = _box(
        SLEEVE_OUTER_X - 2.0 * SLEEVE_WALL,
        SLEEVE_OUTER_Y - 2.0 * SLEEVE_WALL,
        SLEEVE_HEIGHT + 0.02,
    )
    return outer.cut(inner).edges("|Z").fillet(0.004)


def _pod_shape() -> cq.Workplane:
    return _box(POD_LENGTH, POD_DEPTH, POD_HEIGHT).edges("|Z").fillet(0.008)


def _stage_member_shape() -> cq.Workplane:
    return _box(STAGE_MEMBER_X, STAGE_MEMBER_Y, STAGE_MEMBER_HEIGHT).edges("|Z").fillet(0.004)


def _foot_pad_shape() -> cq.Workplane:
    return _box(FOOT_PAD_X, FOOT_PAD_Y, FOOT_PAD_THICKNESS).edges("|Z").fillet(0.008)


def _wiper_collar_shape() -> cq.Workplane:
    return _box(WIPER_COLLAR_X, WIPER_COLLAR_Y, WIPER_COLLAR_THICKNESS).edges("|Z").fillet(0.004)


def _knob_body_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(KNOB_RADIUS).extrude(KNOB_THICKNESS)


def _knob_shaft_shape() -> cq.Workplane:
    return cq.Workplane("XZ").circle(KNOB_SHAFT_RADIUS).extrude(KNOB_SHAFT_LENGTH)


def _knob_pointer_shape() -> cq.Workplane:
    return _box(POINTER_WIDTH, POINTER_DEPTH, POINTER_HEIGHT)


def _articulation_type_name(articulation: object) -> str:
    joint_type = getattr(articulation, "joint_type", getattr(articulation, "articulation_type", None))
    if hasattr(joint_type, "name"):
        return str(joint_type.name).lower()
    return str(joint_type).split(".")[-1].lower()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="conference_standing_desk")

    model.material("top_laminate", rgba=(0.63, 0.54, 0.43, 1.0))
    model.material("frame_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("stage_steel", rgba=(0.46, 0.49, 0.53, 1.0))
    model.material("pod_plastic", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("knob_black", rgba=(0.06, 0.06, 0.07, 1.0))

    desk_body = model.part("desk_body")
    desk_body.visual(
        mesh_from_cadquery(_top_shape(), "desk_top_panel"),
        origin=Origin(xyz=(0.0, 0.0, TOP_CENTER_Z)),
        material="top_laminate",
        name="top_panel",
    )
    desk_body.visual(
        mesh_from_cadquery(_underframe_shape(), "desk_underframe"),
        origin=Origin(xyz=(0.0, 0.0, FRAME_CENTER_Z)),
        material="frame_steel",
        name="underframe",
    )
    for part_name, _, visual_name, x_pos, y_pos in LEG_SPECS:
        desk_body.visual(
            mesh_from_cadquery(_column_sleeve_shape(), f"{part_name}_sleeve_mesh"),
            origin=Origin(xyz=(x_pos, y_pos, SLEEVE_CENTER_Z)),
            material="frame_steel",
            name=visual_name,
        )
    desk_body.visual(
        mesh_from_cadquery(_pod_shape(), "control_pod_mesh"),
        origin=Origin(xyz=(POD_X, POD_Y, POD_Z)),
        material="pod_plastic",
        name="control_pod",
    )

    for part_name, joint_name, _, x_pos, y_pos in LEG_SPECS:
        stage = model.part(part_name)
        stage.visual(
            mesh_from_cadquery(_stage_member_shape(), f"{part_name}_member_mesh"),
            origin=Origin(xyz=(0.0, 0.0, STAGE_MEMBER_CENTER_Z)),
            material="stage_steel",
            name="stage_member",
        )
        stage.visual(
            mesh_from_cadquery(_foot_pad_shape(), f"{part_name}_foot_mesh"),
            origin=Origin(xyz=(0.0, 0.0, FOOT_PAD_CENTER_Z)),
            material="frame_steel",
            name="foot_pad",
        )
        stage.visual(
            mesh_from_cadquery(_wiper_collar_shape(), f"{part_name}_wiper_mesh"),
            origin=Origin(xyz=(0.0, 0.0, WIPER_COLLAR_CENTER_Z)),
            material="frame_steel",
            name="wiper_collar",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=desk_body,
            child=stage,
            origin=Origin(xyz=(x_pos, y_pos, SLEEVE_BOTTOM_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=900.0,
                velocity=0.08,
                lower=0.0,
                upper=LEG_TRAVEL,
            ),
        )

    control_knob = model.part("control_knob")
    control_knob.visual(
        mesh_from_cadquery(_knob_shaft_shape(), "control_knob_shaft_mesh"),
        origin=Origin(xyz=(0.0, KNOB_SHAFT_LENGTH, 0.0)),
        material="knob_black",
        name="knob_shaft",
    )
    control_knob.visual(
        mesh_from_cadquery(_knob_body_shape(), "control_knob_body_mesh"),
        origin=Origin(xyz=(0.0, KNOB_SHAFT_LENGTH + KNOB_THICKNESS - 0.001, 0.0)),
        material="knob_black",
        name="knob_body",
    )
    control_knob.visual(
        mesh_from_cadquery(_knob_pointer_shape(), "control_knob_pointer_mesh"),
        origin=Origin(
            xyz=(
                0.0,
                KNOB_SHAFT_LENGTH + KNOB_THICKNESS - 0.0015,
                KNOB_RADIUS * 0.58,
            )
        ),
        material="top_laminate",
        name="knob_pointer",
    )
    model.articulation(
        "desk_to_control_knob",
        ArticulationType.CONTINUOUS,
        parent=desk_body,
        child=control_knob,
        origin=Origin(xyz=(POD_X, POD_Y + POD_DEPTH / 2.0, POD_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=6.0),
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

    desk_body = object_model.get_part("desk_body")
    control_knob = object_model.get_part("control_knob")
    knob_joint = object_model.get_articulation("desk_to_control_knob")

    ctx.check(
        "control knob uses a continuous rotary articulation",
        _articulation_type_name(knob_joint) == "continuous",
        details=f"joint_type={getattr(knob_joint, 'joint_type', None)}",
    )
    ctx.check(
        "control knob spins about the pod outboard axis",
        tuple(knob_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis={knob_joint.axis}",
    )
    ctx.expect_contact(
        control_knob,
        desk_body,
        elem_a="knob_shaft",
        elem_b="control_pod",
        contact_tol=0.001,
        name="control knob shaft seats against the side pod",
    )

    for part_name, joint_name, sleeve_name, _, _ in LEG_SPECS:
        stage = object_model.get_part(part_name)
        leg_joint = object_model.get_articulation(joint_name)

        ctx.check(
            f"{part_name} uses a vertical prismatic joint",
            _articulation_type_name(leg_joint) == "prismatic" and tuple(leg_joint.axis) == (0.0, 0.0, -1.0),
            details=f"type={getattr(leg_joint, 'joint_type', None)}, axis={leg_joint.axis}",
        )
        ctx.expect_within(
            stage,
            desk_body,
            axes="xy",
            inner_elem="stage_member",
            outer_elem=sleeve_name,
            margin=0.0,
            name=f"{part_name} stays centered inside its outer sleeve at rest",
        )
        ctx.expect_overlap(
            stage,
            desk_body,
            axes="z",
            elem_a="stage_member",
            elem_b=sleeve_name,
            min_overlap=0.20,
            name=f"{part_name} remains inserted in its sleeve at rest",
        )

        rest_pos = ctx.part_world_position(stage)
        with ctx.pose({leg_joint: LEG_TRAVEL}):
            ctx.expect_within(
                stage,
                desk_body,
                axes="xy",
                inner_elem="stage_member",
                outer_elem=sleeve_name,
                margin=0.0,
                name=f"{part_name} stays centered inside its outer sleeve at full extension",
            )
            ctx.expect_overlap(
                stage,
                desk_body,
                axes="z",
                elem_a="stage_member",
                elem_b=sleeve_name,
                min_overlap=0.10,
                name=f"{part_name} still retains insertion at full extension",
            )
            extended_pos = ctx.part_world_position(stage)

        ctx.check(
            f"{part_name} extends downward when actuated",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[2] < rest_pos[2] - 0.05
            and isclose(extended_pos[0], rest_pos[0], abs_tol=1e-6)
            and isclose(extended_pos[1], rest_pos[1], abs_tol=1e-6),
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
