from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


PLATFORM_WIDTH = 0.96
PLATFORM_DEPTH = 0.90
DECK_THICKNESS = 0.04
PLATFORM_CENTER_Y = 0.04
PLATFORM_REST_Z = 0.095
LIFT_TRAVEL = 0.78

POST_X = 0.50
POST_Y = -0.56
POST_WIDTH = 0.08
POST_DEPTH = 0.10
POST_HEIGHT = 1.25

SIDE_PANEL_THICKNESS = 0.02
SIDE_PANEL_DEPTH = 0.76
SIDE_PANEL_HEIGHT = 0.34

RAMP_WIDTH = 0.82
RAMP_LENGTH = 0.22
RAMP_THICKNESS = 0.018


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wheelchair_platform_lift")

    frame_steel = model.material("frame_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.52, 0.55, 0.58, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.93, 0.78, 0.14, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    alloy = model.material("alloy", rgba=(0.80, 0.82, 0.84, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((1.18, 0.08, 0.14)),
        origin=Origin(xyz=(0.0, POST_Y - 0.09, 0.07)),
        material=frame_steel,
        name="lower_beam",
    )
    base_frame.visual(
        Box((0.32, 0.16, 0.05)),
        origin=Origin(xyz=(-0.34, -0.57, 0.025)),
        material=dark_trim,
        name="left_base_foot",
    )
    base_frame.visual(
        Box((0.32, 0.16, 0.05)),
        origin=Origin(xyz=(0.34, -0.57, 0.025)),
        material=dark_trim,
        name="right_base_foot",
    )
    base_frame.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(-POST_X, POST_Y, POST_HEIGHT / 2.0)),
        material=frame_steel,
        name="left_post",
    )
    base_frame.visual(
        Box((POST_WIDTH, POST_DEPTH, POST_HEIGHT)),
        origin=Origin(xyz=(POST_X, POST_Y, POST_HEIGHT / 2.0)),
        material=frame_steel,
        name="right_post",
    )
    base_frame.visual(
        Box((1.08, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, POST_Y, 1.16)),
        material=frame_steel,
        name="top_beam",
    )
    base_frame.visual(
        Cylinder(radius=0.055, length=0.93),
        origin=Origin(xyz=(0.0, POST_Y, 0.655)),
        material=dark_trim,
        name="lift_actuator_cover",
    )
    base_frame.visual(
        Box((0.05, 0.10, 0.24)),
        origin=Origin(xyz=(0.565, POST_Y, 0.84)),
        material=alloy,
        name="controller_box",
    )

    platform = model.part("platform")
    platform.visual(
        Box((PLATFORM_WIDTH, PLATFORM_DEPTH, DECK_THICKNESS)),
        material=deck_steel,
        name="deck_plate",
    )
    platform.visual(
        Box((0.04, 0.76, 0.08)),
        origin=Origin(xyz=(-0.46, 0.0, 0.02)),
        material=safety_yellow,
        name="left_curb",
    )
    platform.visual(
        Box((0.04, 0.76, 0.08)),
        origin=Origin(xyz=(0.46, 0.0, 0.02)),
        material=safety_yellow,
        name="right_curb",
    )
    platform.visual(
        Box((0.88, 0.04, 0.08)),
        origin=Origin(xyz=(0.0, -0.43, 0.02)),
        material=safety_yellow,
        name="rear_curb",
    )
    platform.visual(
        Box((0.88, 0.09, 0.08)),
        origin=Origin(xyz=(0.0, -0.495, 0.02)),
        material=frame_steel,
        name="rear_underframe",
    )
    platform.visual(
        Box((0.04, 0.10, 0.24)),
        origin=Origin(xyz=(-0.44, -0.55, 0.10)),
        material=dark_trim,
        name="left_carriage",
    )
    platform.visual(
        Box((0.04, 0.10, 0.24)),
        origin=Origin(xyz=(0.44, -0.55, 0.10)),
        material=dark_trim,
        name="right_carriage",
    )

    left_rail_panel = model.part("left_rail_panel")
    left_rail_panel.visual(
        Box((SIDE_PANEL_THICKNESS, SIDE_PANEL_DEPTH, SIDE_PANEL_HEIGHT)),
        origin=Origin(xyz=(-SIDE_PANEL_THICKNESS / 2.0, 0.0, SIDE_PANEL_HEIGHT / 2.0)),
        material=safety_yellow,
        name="left_panel_body",
    )

    right_rail_panel = model.part("right_rail_panel")
    right_rail_panel.visual(
        Box((SIDE_PANEL_THICKNESS, SIDE_PANEL_DEPTH, SIDE_PANEL_HEIGHT)),
        origin=Origin(xyz=(SIDE_PANEL_THICKNESS / 2.0, 0.0, SIDE_PANEL_HEIGHT / 2.0)),
        material=safety_yellow,
        name="right_panel_body",
    )

    approach_ramp = model.part("approach_ramp")
    approach_ramp.visual(
        Box((RAMP_WIDTH, RAMP_LENGTH, RAMP_THICKNESS)),
        origin=Origin(xyz=(0.0, RAMP_LENGTH / 2.0, -RAMP_THICKNESS / 2.0)),
        material=alloy,
        name="ramp_plate",
    )
    approach_ramp.visual(
        Box((RAMP_WIDTH, 0.02, 0.04)),
        origin=Origin(xyz=(0.0, RAMP_LENGTH - 0.01, 0.02)),
        material=safety_yellow,
        name="ramp_tip_lip",
    )

    model.articulation(
        "lift_stage",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=platform,
        origin=Origin(xyz=(0.0, PLATFORM_CENTER_Y, PLATFORM_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.25,
            lower=0.0,
            upper=LIFT_TRAVEL,
        ),
    )
    model.articulation(
        "left_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=left_rail_panel,
        origin=Origin(xyz=(-PLATFORM_WIDTH / 2.0, 0.0, DECK_THICKNESS / 2.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-math.radians(95.0),
            upper=0.05,
        ),
    )
    model.articulation(
        "right_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=right_rail_panel,
        origin=Origin(xyz=(PLATFORM_WIDTH / 2.0, 0.0, DECK_THICKNESS / 2.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-math.radians(95.0),
            upper=0.05,
        ),
    )
    model.articulation(
        "front_ramp_hinge",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=approach_ramp,
        origin=Origin(xyz=(0.0, PLATFORM_DEPTH / 2.0, DECK_THICKNESS / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-math.radians(18.0),
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    platform = object_model.get_part("platform")
    left_rail_panel = object_model.get_part("left_rail_panel")
    right_rail_panel = object_model.get_part("right_rail_panel")
    approach_ramp = object_model.get_part("approach_ramp")

    lift_stage = object_model.get_articulation("lift_stage")
    left_panel_hinge = object_model.get_articulation("left_panel_hinge")
    right_panel_hinge = object_model.get_articulation("right_panel_hinge")
    front_ramp_hinge = object_model.get_articulation("front_ramp_hinge")

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

    ctx.expect_contact(platform, base_frame, name="platform rides on guide posts")
    ctx.expect_contact(left_rail_panel, platform, name="left panel mounted to platform edge")
    ctx.expect_contact(right_rail_panel, platform, name="right panel mounted to platform edge")
    ctx.expect_contact(approach_ramp, platform, name="approach ramp hinged to deck")

    ctx.check(
        "lift articulation is vertical prismatic",
        lift_stage.joint_type == ArticulationType.PRISMATIC
        and tuple(lift_stage.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift_stage.joint_type}, axis={lift_stage.axis}",
    )
    ctx.check(
        "side panel hinges rotate about platform length",
        tuple(left_panel_hinge.axis) == (0.0, 1.0, 0.0)
        and tuple(right_panel_hinge.axis) == (0.0, -1.0, 0.0),
        details=(
            f"left_axis={left_panel_hinge.axis}, "
            f"right_axis={right_panel_hinge.axis}"
        ),
    )
    ctx.check(
        "front ramp hinge rotates about width axis",
        tuple(front_ramp_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={front_ramp_hinge.axis}",
    )

    platform_rest = ctx.part_world_position(platform)
    if platform_rest is None:
        ctx.fail("platform rest position available", "platform world position returned None")
    else:
        with ctx.pose({lift_stage: 0.60}):
            platform_raised = ctx.part_world_position(platform)
            if platform_raised is None:
                ctx.fail("platform raised position available", "raised platform position returned None")
            else:
                ctx.check(
                    "platform lifts through substantial vertical travel",
                    platform_raised[2] > platform_rest[2] + 0.55,
                    details=f"rest={platform_rest}, raised={platform_raised}",
                )
                ctx.expect_contact(platform, base_frame, name="platform remains guided when raised")

    left_rest_aabb = ctx.part_world_aabb(left_rail_panel)
    right_rest_aabb = ctx.part_world_aabb(right_rail_panel)
    ramp_rest_aabb = ctx.part_world_aabb(approach_ramp)

    if left_rest_aabb is None:
        ctx.fail("left panel rest bounds available", "left panel AABB returned None")
    else:
        with ctx.pose({left_panel_hinge: -1.35}):
            left_folded_aabb = ctx.part_world_aabb(left_rail_panel)
            if left_folded_aabb is None:
                ctx.fail("left panel folded bounds available", "left folded AABB returned None")
            else:
                ctx.check(
                    "left side panel folds outward and down",
                    left_folded_aabb[0][0] < left_rest_aabb[0][0] - 0.10
                    and left_folded_aabb[1][2] < left_rest_aabb[1][2] - 0.14,
                    details=f"rest={left_rest_aabb}, folded={left_folded_aabb}",
                )

    if right_rest_aabb is None:
        ctx.fail("right panel rest bounds available", "right panel AABB returned None")
    else:
        with ctx.pose({right_panel_hinge: -1.35}):
            right_folded_aabb = ctx.part_world_aabb(right_rail_panel)
            if right_folded_aabb is None:
                ctx.fail("right panel folded bounds available", "right folded AABB returned None")
            else:
                ctx.check(
                    "right side panel folds outward and down",
                    right_folded_aabb[1][0] > right_rest_aabb[1][0] + 0.10
                    and right_folded_aabb[1][2] < right_rest_aabb[1][2] - 0.14,
                    details=f"rest={right_rest_aabb}, folded={right_folded_aabb}",
                )

    if ramp_rest_aabb is None:
        ctx.fail("ramp rest bounds available", "ramp AABB returned None")
    else:
        with ctx.pose({front_ramp_hinge: 1.10}):
            ramp_raised_aabb = ctx.part_world_aabb(approach_ramp)
            if ramp_raised_aabb is None:
                ctx.fail("ramp raised bounds available", "raised ramp AABB returned None")
            else:
                ctx.check(
                    "approach ramp raises into a wheel stop",
                    ramp_raised_aabb[1][2] > ramp_rest_aabb[1][2] + 0.14,
                    details=f"rest={ramp_rest_aabb}, raised={ramp_raised_aabb}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
