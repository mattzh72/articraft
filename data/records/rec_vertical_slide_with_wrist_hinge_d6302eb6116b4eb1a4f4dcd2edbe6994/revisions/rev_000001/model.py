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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_W = 0.220
FRAME_D = 0.110
FRAME_H = 0.520
BASE_T = 0.045
BACK_TH = 0.018
SIDE_W = 0.034
SIDE_D = 0.090
SIDE_X = 0.093
TOP_T = 0.028
TOP_D = 0.060
TOP_Y = -0.010

GUIDE_W = 0.012
GUIDE_D = 0.060
GUIDE_H = 0.340
GUIDE_X = 0.070
GUIDE_Y = 0.004
GUIDE_Z = 0.260

SLIDE_HOME_Z = 0.105
SLIDE_TRAVEL = 0.190

CARRIAGE_BODY_W = 0.088
CARRIAGE_BODY_D = 0.048
CARRIAGE_BODY_H = 0.188
SHOE_W = 0.020
SHOE_D = GUIDE_D
SHOE_H = 0.180
SHOE_X = 0.054
SHOE_Y = GUIDE_Y
SHOE_Z = 0.090

NOSE_W = 0.058
NOSE_D = 0.034
NOSE_H = 0.074
NOSE_Y = 0.028
NOSE_Z = 0.108

NECK_W = 0.064
NECK_D = 0.022
NECK_H = 0.032
NECK_Y = 0.038
NECK_Z = 0.145

EAR_W = 0.018
EAR_D = 0.026
EAR_H = 0.042
EAR_X = 0.040
EAR_Y = 0.045
EAR_Z = 0.167

WRIST_ORIGIN_Y = 0.054
WRIST_ORIGIN_Z = 0.165
WRIST_STOP_W = 0.060
WRIST_STOP_D = 0.010
WRIST_STOP_H = 0.036
WRIST_STOP_Y = 0.049
WRIST_STOP_Z = 0.128

BARREL_R = 0.010
BARREL_L = 0.062
FACE_PANEL_W = 0.084
FACE_PANEL_T = 0.014
FACE_PANEL_H = 0.074
FACE_PANEL_Y = 0.007
FACE_PANEL_Z = -0.048
FACE_BRACE_W = 0.052
FACE_BRACE_D = 0.016
FACE_BRACE_H = 0.022
FACE_BRACE_Y = 0.008
FACE_BRACE_Z = -0.021


def _center_z(height: float) -> float:
    return height / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_backed_slide_wrist_unit")

    model.material("painted_steel", rgba=(0.25, 0.28, 0.31, 1.0))
    model.material("ground_steel", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("safety_orange", rgba=(0.84, 0.45, 0.12, 1.0))
    model.material("tool_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    support_frame = model.part("support_frame")
    support_frame.visual(
        Box((FRAME_W, FRAME_D, BASE_T)),
        origin=Origin(xyz=(0.0, 0.0, _center_z(BASE_T))),
        material="painted_steel",
        name="base_block",
    )
    support_frame.visual(
        Box((FRAME_W, BACK_TH, FRAME_H)),
        origin=Origin(
            xyz=(0.0, -(FRAME_D / 2.0) + (BACK_TH / 2.0), _center_z(FRAME_H))
        ),
        material="painted_steel",
        name="back_plate",
    )
    for side_name, x_pos in (("left_cheek", -SIDE_X), ("right_cheek", SIDE_X)):
        support_frame.visual(
            Box((SIDE_W, SIDE_D, FRAME_H - BASE_T)),
            origin=Origin(xyz=(x_pos, 0.0, BASE_T + _center_z(FRAME_H - BASE_T))),
            material="painted_steel",
            name=side_name,
        )
    support_frame.visual(
        Box((FRAME_W - 0.032, TOP_D, TOP_T)),
        origin=Origin(
            xyz=(0.0, TOP_Y, FRAME_H - _center_z(TOP_T))
        ),
        material="painted_steel",
        name="top_bridge",
    )
    support_frame.visual(
        Box((FRAME_W - 0.064, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, -0.034, 0.115)),
        material="painted_steel",
        name="mid_brace",
    )
    for rail_name, x_pos in (("left_guide", -GUIDE_X), ("right_guide", GUIDE_X)):
        support_frame.visual(
            Box((GUIDE_W, GUIDE_D, GUIDE_H)),
            origin=Origin(xyz=(x_pos, GUIDE_Y, GUIDE_Z)),
            material="ground_steel",
            name=rail_name,
        )
    support_frame.inertial = Inertial.from_geometry(
        Box((FRAME_W, FRAME_D, FRAME_H)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_H / 2.0)),
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.visual(
        Box((CARRIAGE_BODY_W, CARRIAGE_BODY_D, CARRIAGE_BODY_H)),
        origin=Origin(
            xyz=(0.0, 0.0, _center_z(CARRIAGE_BODY_H)),
        ),
        material="safety_orange",
        name="carriage_body",
    )
    for shoe_name, x_pos in (("left_shoe", -SHOE_X), ("right_shoe", SHOE_X)):
        slide_carriage.visual(
            Box((SHOE_W, SHOE_D, SHOE_H)),
            origin=Origin(xyz=(x_pos, SHOE_Y, SHOE_Z)),
            material="ground_steel",
            name=shoe_name,
        )
    slide_carriage.visual(
        Box((NOSE_W, NOSE_D, NOSE_H)),
        origin=Origin(xyz=(0.0, NOSE_Y, NOSE_Z)),
        material="safety_orange",
        name="nose_block",
    )
    slide_carriage.visual(
        Box((NECK_W, NECK_D, NECK_H)),
        origin=Origin(xyz=(0.0, NECK_Y, NECK_Z)),
        material="safety_orange",
        name="wrist_neck",
    )
    for ear_name, x_pos in (("left_ear", -EAR_X), ("right_ear", EAR_X)):
        slide_carriage.visual(
            Box((EAR_W, EAR_D, EAR_H)),
            origin=Origin(xyz=(x_pos, EAR_Y, EAR_Z)),
            material="painted_steel",
            name=ear_name,
        )
    slide_carriage.visual(
        Box((WRIST_STOP_W, WRIST_STOP_D, WRIST_STOP_H)),
        origin=Origin(xyz=(0.0, WRIST_STOP_Y, WRIST_STOP_Z)),
        material="painted_steel",
        name="wrist_stop",
    )
    slide_carriage.inertial = Inertial.from_geometry(
        Box((0.132, 0.060, 0.205)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.010, 0.102)),
    )

    wrist_face = model.part("wrist_face")
    wrist_face.visual(
        Cylinder(radius=BARREL_R, length=BARREL_L),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="tool_silver",
        name="barrel_proxy",
    )
    wrist_face.visual(
        Box((FACE_BRACE_W, FACE_BRACE_D, FACE_BRACE_H)),
        origin=Origin(xyz=(0.0, FACE_BRACE_Y, FACE_BRACE_Z)),
        material="tool_silver",
        name="face_brace",
    )
    wrist_face.visual(
        Box((FACE_PANEL_W, FACE_PANEL_T, FACE_PANEL_H)),
        origin=Origin(xyz=(0.0, FACE_PANEL_Y, FACE_PANEL_Z)),
        material="tool_silver",
        name="face_panel",
    )
    wrist_face.inertial = Inertial.from_geometry(
        Box((FACE_PANEL_W, 0.022, 0.090)),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.008, -0.040)),
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support_frame,
        child=slide_carriage,
        origin=Origin(xyz=(0.0, GUIDE_Y, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_face",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=wrist_face,
        origin=Origin(xyz=(0.0, WRIST_ORIGIN_Y, WRIST_ORIGIN_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.0,
            lower=-0.70,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_frame = object_model.get_part("support_frame")
    slide_carriage = object_model.get_part("slide_carriage")
    wrist_face = object_model.get_part("wrist_face")
    frame_to_carriage = object_model.get_articulation("frame_to_carriage")
    carriage_to_face = object_model.get_articulation("carriage_to_face")

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
        support_frame,
        slide_carriage,
        elem_a="left_guide",
        elem_b="left_shoe",
        name="left carriage shoe contacts left guide rail",
    )
    ctx.expect_contact(
        support_frame,
        slide_carriage,
        elem_a="right_guide",
        elem_b="right_shoe",
        name="right carriage shoe contacts right guide rail",
    )
    ctx.expect_within(
        slide_carriage,
        support_frame,
        axes="xy",
        inner_elem="carriage_body",
        margin=0.0,
        name="carriage body remains within the support frame footprint",
    )
    ctx.expect_overlap(
        slide_carriage,
        support_frame,
        axes="z",
        min_overlap=0.180,
        name="carriage has deep guided engagement at home",
    )
    ctx.expect_contact(
        slide_carriage,
        wrist_face,
        elem_a="left_ear",
        elem_b="barrel_proxy",
        name="left ear supports the wrist barrel",
    )
    ctx.expect_contact(
        slide_carriage,
        wrist_face,
        elem_a="right_ear",
        elem_b="barrel_proxy",
        name="right ear supports the wrist barrel",
    )
    ctx.expect_contact(
        slide_carriage,
        wrist_face,
        elem_a="wrist_stop",
        elem_b="face_panel",
        name="wrist face rests against the carriage stop at home",
    )

    rest_carriage_pos = ctx.part_world_position(slide_carriage)
    upper_slide = (
        frame_to_carriage.motion_limits.upper
        if frame_to_carriage.motion_limits is not None
        and frame_to_carriage.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({frame_to_carriage: upper_slide}):
        ctx.expect_contact(
            support_frame,
            slide_carriage,
            elem_a="left_guide",
            elem_b="left_shoe",
            name="left shoe stays on the guide at full lift",
        )
        ctx.expect_contact(
            support_frame,
            slide_carriage,
            elem_a="right_guide",
            elem_b="right_shoe",
            name="right shoe stays on the guide at full lift",
        )
        ctx.expect_overlap(
            slide_carriage,
            support_frame,
            axes="z",
            min_overlap=0.120,
            name="carriage retains insertion at full lift",
        )
        lifted_carriage_pos = ctx.part_world_position(slide_carriage)

    ctx.check(
        "positive slide motion lifts the carriage upward",
        rest_carriage_pos is not None
        and lifted_carriage_pos is not None
        and lifted_carriage_pos[2] > rest_carriage_pos[2] + 0.150,
        details=f"rest={rest_carriage_pos}, lifted={lifted_carriage_pos}",
    )

    face_rest_aabb = ctx.part_element_world_aabb(wrist_face, elem="face_panel")
    upper_wrist = (
        carriage_to_face.motion_limits.upper
        if carriage_to_face.motion_limits is not None
        and carriage_to_face.motion_limits.upper is not None
        else 0.0
    )
    with ctx.pose({carriage_to_face: upper_wrist}):
        ctx.expect_contact(
            slide_carriage,
            wrist_face,
            elem_a="left_ear",
            elem_b="barrel_proxy",
            name="left ear stays engaged through wrist travel",
        )
        ctx.expect_contact(
            slide_carriage,
            wrist_face,
            elem_a="right_ear",
            elem_b="barrel_proxy",
            name="right ear stays engaged through wrist travel",
        )
        face_open_aabb = ctx.part_element_world_aabb(wrist_face, elem="face_panel")

    def _aabb_center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_face_z = _aabb_center_z(face_rest_aabb)
    open_face_z = _aabb_center_z(face_open_aabb)
    ctx.check(
        "positive wrist motion pitches the face upward",
        rest_face_z is not None
        and open_face_z is not None
        and open_face_z > rest_face_z + 0.020,
        details=f"rest_center_z={rest_face_z}, open_center_z={open_face_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
