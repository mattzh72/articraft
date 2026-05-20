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


BODY_LENGTH = 0.34
BODY_WIDTH = 0.09
BODY_HEIGHT = 0.04
BODY_FLOOR_THICKNESS = 0.005
BODY_WALL_THICKNESS = 0.004
BODY_LIP_WIDTH = 0.012
BODY_LIP_THICKNESS = 0.004
BODY_LIP_BOTTOM_Z = 0.030
BODY_REAR_BRIDGE_LENGTH = 0.016
BODY_REAR_BRIDGE_HEIGHT = 0.016

RUNNER_LENGTH = 0.22
RUNNER_BOTTOM_Z = BODY_FLOOR_THICKNESS
RUNNER_STEM_WIDTH = 0.048
RUNNER_STEM_HEIGHT = 0.017
RUNNER_WING_WIDTH = 0.056
RUNNER_WING_THICKNESS = 0.006
RUNNER_CLOSED_CENTER_X = BODY_LENGTH / 2.0 - RUNNER_LENGTH / 2.0
RUNNER_TRAVEL = 0.12


def add_slide_body_visuals(model: ArticulatedObject, body_part) -> None:
    body_material = model.material("body_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    inner_width = BODY_WIDTH - 2.0 * BODY_WALL_THICKNESS
    wall_center_y = BODY_WIDTH / 2.0 - BODY_WALL_THICKNESS / 2.0
    lip_center_y = BODY_WIDTH / 2.0 - BODY_WALL_THICKNESS - BODY_LIP_WIDTH / 2.0

    body_part.visual(
        Box((BODY_LENGTH, BODY_WIDTH, BODY_FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_FLOOR_THICKNESS / 2.0)),
        material=body_material,
        name="floor",
    )
    body_part.visual(
        Box((BODY_LENGTH, BODY_WALL_THICKNESS, BODY_HEIGHT - BODY_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                wall_center_y,
                BODY_FLOOR_THICKNESS + (BODY_HEIGHT - BODY_FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=body_material,
        name="left_wall",
    )
    body_part.visual(
        Box((BODY_LENGTH, BODY_WALL_THICKNESS, BODY_HEIGHT - BODY_FLOOR_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -wall_center_y,
                BODY_FLOOR_THICKNESS + (BODY_HEIGHT - BODY_FLOOR_THICKNESS) / 2.0,
            )
        ),
        material=body_material,
        name="right_wall",
    )
    body_part.visual(
        Box((BODY_LENGTH, BODY_LIP_WIDTH, BODY_LIP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, lip_center_y, BODY_LIP_BOTTOM_Z + BODY_LIP_THICKNESS / 2.0)
        ),
        material=body_material,
        name="left_lip",
    )
    body_part.visual(
        Box((BODY_LENGTH, BODY_LIP_WIDTH, BODY_LIP_THICKNESS)),
        origin=Origin(
            xyz=(0.0, -lip_center_y, BODY_LIP_BOTTOM_Z + BODY_LIP_THICKNESS / 2.0)
        ),
        material=body_material,
        name="right_lip",
    )
    body_part.visual(
        Box((BODY_REAR_BRIDGE_LENGTH, inner_width, BODY_REAR_BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                -BODY_LENGTH / 2.0 + BODY_REAR_BRIDGE_LENGTH / 2.0,
                0.0,
                BODY_FLOOR_THICKNESS + BODY_REAR_BRIDGE_HEIGHT / 2.0,
            )
        ),
        material=body_material,
        name="rear_stop",
    )


def add_runner_visuals(model: ArticulatedObject, runner_part) -> None:
    runner_material = model.material("runner_steel", rgba=(0.56, 0.58, 0.62, 1.0))

    runner_part.visual(
        Box((RUNNER_LENGTH, RUNNER_STEM_WIDTH, RUNNER_STEM_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, RUNNER_BOTTOM_Z + RUNNER_STEM_HEIGHT / 2.0),
        ),
        material=runner_material,
        name="stem",
    )
    runner_part.visual(
        Box((RUNNER_LENGTH, RUNNER_WING_WIDTH, RUNNER_WING_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                RUNNER_BOTTOM_Z
                + RUNNER_STEM_HEIGHT
                + RUNNER_WING_THICKNESS / 2.0,
            )
        ),
        material=runner_material,
        name="head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_telescoping_slide")

    body = model.part("body")
    add_slide_body_visuals(model, body)

    runner = model.part("runner")
    add_runner_visuals(model, runner)

    model.articulation(
        "body_to_runner",
        ArticulationType.PRISMATIC,
        parent=body,
        child=runner,
        origin=Origin(xyz=(RUNNER_CLOSED_CENTER_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.35,
            lower=0.0,
            upper=RUNNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    runner = object_model.get_part("runner")
    slide = object_model.get_articulation("body_to_runner")
    limits = slide.motion_limits

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
        "runner_joint_is_prismatic",
        slide.joint_type == ArticulationType.PRISMATIC,
        details=f"expected PRISMATIC joint, got {slide.joint_type}",
    )
    ctx.check(
        "runner_joint_axis_along_x",
        tuple(round(value, 6) for value in slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected +X slide axis, got {slide.axis}",
    )
    ctx.check(
        "runner_travel_limits_match_slide",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and abs(limits.upper - RUNNER_TRAVEL) < 1e-9,
        details=f"unexpected motion limits: {limits}",
    )

    closed_runner_x = None
    extended_runner_x = None

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            runner,
            body,
            name="runner_supported_when_closed",
        )
        ctx.expect_overlap(
            runner,
            body,
            axes="x",
            min_overlap=0.21,
            name="closed_runner_has_deep_nesting",
        )
        ctx.expect_within(
            runner,
            body,
            axes="yz",
            margin=0.0,
            name="closed_runner_stays_inside_body_profile",
        )
        closed_runner_x = ctx.part_world_position(runner)[0]

    with ctx.pose({slide: RUNNER_TRAVEL}):
        ctx.expect_contact(
            runner,
            body,
            name="runner_supported_when_extended",
        )
        ctx.expect_overlap(
            runner,
            body,
            axes="x",
            min_overlap=0.10,
            name="extended_runner_retains_overlap",
        )
        ctx.expect_within(
            runner,
            body,
            axes="yz",
            margin=0.0,
            name="extended_runner_stays_captured",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlap_when_extended")
        extended_runner_x = ctx.part_world_position(runner)[0]

    ctx.check(
        "runner_travel_matches_prismatic_motion",
        closed_runner_x is not None
        and extended_runner_x is not None
        and abs((extended_runner_x - closed_runner_x) - RUNNER_TRAVEL) < 1e-6,
        details=(
            f"closed x={closed_runner_x}, extended x={extended_runner_x}, "
            f"expected travel={RUNNER_TRAVEL}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
