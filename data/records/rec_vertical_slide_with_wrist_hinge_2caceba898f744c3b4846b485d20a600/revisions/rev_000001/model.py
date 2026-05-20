from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_W = 0.22
BASE_D = 0.08
BASE_T = 0.014

PEDESTAL_W = 0.09
PEDESTAL_D = 0.034
PEDESTAL_H = 0.024

GUIDE_W = 0.024
GUIDE_D = 0.012
GUIDE_H = 0.29

CARRIAGE_RUNNER_W = 0.03
CARRIAGE_RUNNER_D = 0.006
CARRIAGE_RUNNER_H = 0.10
CARRIAGE_RUNNER_Y = 0.008

CARRIAGE_BODY_W = 0.05
CARRIAGE_BODY_D = 0.02
CARRIAGE_BODY_H = 0.06
CARRIAGE_BODY_Y = 0.02

HINGE_BOSS_W = 0.016
HINGE_BOSS_D = 0.006
HINGE_BOSS_H = 0.016
HINGE_BOSS_Y = 0.027
HINGE_BOSS_Z = 0.014

HINGE_Y = 0.03
HINGE_Z = 0.014

BRACKET_ROOT_W = 0.014
BRACKET_ROOT_D = 0.006
BRACKET_ROOT_H = 0.012
BRACKET_ROOT_Y = 0.003

BRACKET_WEB_W = 0.012
BRACKET_WEB_D = 0.022
BRACKET_WEB_H = 0.018
BRACKET_WEB_Y = 0.017
BRACKET_WEB_Z = -0.008

BRACKET_FACE_W = 0.028
BRACKET_FACE_D = 0.008
BRACKET_FACE_H = 0.032
BRACKET_FACE_Y = 0.03
BRACKET_FACE_Z = -0.016

SLIDE_REST_Z = BASE_T + PEDESTAL_H + 0.10
SLIDE_TRAVEL = 0.15


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_hinged_bracket")

    frame_material = model.material("frame_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    carriage_material = model.material("carriage_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    bracket_material = model.material("bracket_steel", rgba=(0.56, 0.58, 0.6, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((BASE_W, BASE_D, BASE_T)),
        material=frame_material,
        origin=Origin(xyz=(0.0, 0.0, BASE_T / 2)),
        name="frame_base",
    )
    frame.visual(
        Box((PEDESTAL_W, PEDESTAL_D, PEDESTAL_H)),
        material=frame_material,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + PEDESTAL_H / 2)),
        name="frame_mount",
    )
    frame.visual(
        Box((0.05, 0.018, 0.07)),
        material=frame_material,
        origin=Origin(xyz=(0.0, -0.002, 0.034 + 0.035)),
        name="frame_lower_stage",
    )
    frame.visual(
        Box((GUIDE_W, GUIDE_D, GUIDE_H)),
        material=frame_material,
        origin=Origin(xyz=(0.0, -0.001, 0.034 + GUIDE_H / 2)),
        name="frame_guide",
    )
    frame.visual(
        Box((0.04, 0.014, 0.012)),
        material=frame_material,
        origin=Origin(xyz=(0.0, -0.001, 0.318)),
        name="frame_cap",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_RUNNER_W, CARRIAGE_RUNNER_D, CARRIAGE_RUNNER_H)),
        material=carriage_material,
        origin=Origin(xyz=(0.0, CARRIAGE_RUNNER_Y, 0.0)),
        name="carriage_runner",
    )
    carriage.visual(
        Box((CARRIAGE_BODY_W, CARRIAGE_BODY_D, CARRIAGE_BODY_H)),
        material=carriage_material,
        origin=Origin(xyz=(0.0, CARRIAGE_BODY_Y, 0.0)),
        name="carriage_body",
    )
    carriage.visual(
        Box((HINGE_BOSS_W, HINGE_BOSS_D, HINGE_BOSS_H)),
        material=carriage_material,
        origin=Origin(xyz=(0.0, HINGE_BOSS_Y, HINGE_BOSS_Z)),
        name="carriage_hinge_boss",
    )

    bracket = model.part("bracket")
    bracket.visual(
        Box((BRACKET_ROOT_W, BRACKET_ROOT_D, BRACKET_ROOT_H)),
        material=bracket_material,
        origin=Origin(xyz=(0.0, BRACKET_ROOT_Y, 0.0)),
        name="bracket_root",
    )
    bracket.visual(
        Box((BRACKET_WEB_W, BRACKET_WEB_D, BRACKET_WEB_H)),
        material=bracket_material,
        origin=Origin(xyz=(0.0, BRACKET_WEB_Y, BRACKET_WEB_Z)),
        name="bracket_web",
    )
    bracket.visual(
        Box((BRACKET_FACE_W, BRACKET_FACE_D, BRACKET_FACE_H)),
        material=bracket_material,
        origin=Origin(xyz=(0.0, BRACKET_FACE_Y, BRACKET_FACE_Z)),
        name="bracket_body",
    )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    model.articulation(
        "carriage_to_bracket",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=bracket,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    bracket = object_model.get_part("bracket")
    slide = object_model.get_articulation("frame_to_carriage")
    hinge = object_model.get_articulation("carriage_to_bracket")

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
        "parts_present",
        frame is not None and carriage is not None and bracket is not None,
        "frame, carriage, and bracket must all be present",
    )

    slide_limits = slide.motion_limits
    ctx.check(
        "vertical_prismatic_slide",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide.axis) == (0.0, 0.0, 1.0)
        and slide_limits is not None
        and slide_limits.lower == 0.0
        and slide_limits.upper is not None
        and slide_limits.upper >= 0.12,
        "carriage must ride on a vertical prismatic joint with meaningful upward travel",
    )

    hinge_limits = hinge.motion_limits
    ctx.check(
        "carried_revolute_bracket",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(hinge.axis) == (1.0, 0.0, 0.0)
        and hinge_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and hinge_limits.upper >= 1.0,
        "bracket must be carried by the moving block on a horizontal revolute hinge",
    )

    ctx.expect_overlap(
        carriage,
        frame,
        axes="x",
        min_overlap=0.05,
        name="guide_width_covers_carriage",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_origin_distance(
            frame,
            carriage,
            axes="xy",
            max_dist=0.001,
            name="carriage_centered_on_guide",
        )

        carriage_aabb = ctx.part_world_aabb(carriage)
        bracket_aabb = ctx.part_element_world_aabb(bracket, elem="bracket_body")
        forward_projection_ok = (
            carriage_aabb is not None
            and bracket_aabb is not None
            and bracket_aabb[1][1] > carriage_aabb[1][1] + 0.012
        )
        ctx.check(
            "bracket_projects_forward_of_carriage",
            forward_projection_ok,
            "the hinged bracket should project forward of the moving block in the closed pose",
        )

    with ctx.pose({slide: 0.0}):
        lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        upper_pos = ctx.part_world_position(carriage)

    translation_ok = (
        lower_pos is not None
        and upper_pos is not None
        and abs(upper_pos[0] - lower_pos[0]) <= 1e-6
        and abs(upper_pos[1] - lower_pos[1]) <= 1e-6
        and upper_pos[2] > lower_pos[2] + 0.12
    )
    ctx.check(
        "carriage_moves_only_upward",
        translation_ok,
        "upper slide pose should raise the carriage substantially without lateral drift",
    )

    with ctx.pose({hinge: 0.0}):
        closed_bracket = ctx.part_element_world_aabb(bracket, elem="bracket_body")
    with ctx.pose({hinge: 1.0}):
        open_bracket = ctx.part_element_world_aabb(bracket, elem="bracket_body")

    hinge_motion_ok = (
        closed_bracket is not None
        and open_bracket is not None
        and open_bracket[1][2] > closed_bracket[1][2] + 0.015
    )
    ctx.check(
        "bracket_opens_upward",
        hinge_motion_ok,
        "positive hinge rotation should lift the bracket upward from the carriage face",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
