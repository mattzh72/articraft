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


BEAM_LENGTH = 1.20
BEAM_WIDTH = 0.18
BEAM_HEIGHT = 0.12
BEAM_WALL = 0.012
BEAM_END_CAP = 0.055
SLOT_WIDTH = 0.042

BEAM_INNER_WIDTH = BEAM_WIDTH - (2.0 * BEAM_WALL)
BEAM_INNER_HEIGHT = BEAM_HEIGHT - (2.0 * BEAM_WALL)
BOTTOM_LIP_WIDTH = (BEAM_INNER_WIDTH - SLOT_WIDTH) / 2.0
BOTTOM_LIP_LENGTH = BEAM_LENGTH - (2.0 * BEAM_END_CAP)

TROLLEY_LENGTH = 0.16
TROLLEY_WIDTH = 0.132
TROLLEY_HEIGHT = 0.032
TROLLEY_Z = 0.016

RUNNER_PAD_LENGTH = 0.13
RUNNER_PAD_WIDTH = 0.012
RUNNER_PAD_HEIGHT = 0.024
RUNNER_PAD_Y = (BEAM_INNER_WIDTH / 2.0) - (RUNNER_PAD_WIDTH / 2.0)
RUNNER_PAD_Z = TROLLEY_Z

STEM_LENGTH = 0.07
STEM_WIDTH = 0.028
STEM_HEIGHT = 0.116
STEM_Z = -(STEM_HEIGHT / 2.0)

LOWER_BODY_LENGTH = 0.26
LOWER_BODY_WIDTH = 0.14
LOWER_BODY_HEIGHT = 0.058
LOWER_BODY_Z = -0.145

TRAVEL_HALF_RANGE = 0.40


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_slide_carriage")

    model.material("beam_gray", rgba=(0.72, 0.75, 0.79, 1.0))
    model.material("carriage_orange", rgba=(0.86, 0.44, 0.10, 1.0))
    model.material("runner_black", rgba=(0.12, 0.12, 0.13, 1.0))
    model.material("stem_black", rgba=(0.16, 0.17, 0.19, 1.0))

    support_beam = model.part("support_beam")
    support_beam.visual(
        Box((BEAM_LENGTH, BEAM_WIDTH, BEAM_WALL)),
        material="beam_gray",
        origin=Origin(xyz=(0.0, 0.0, (BEAM_HEIGHT / 2.0) - (BEAM_WALL / 2.0))),
        name="top_wall",
    )
    support_beam.visual(
        Box((BEAM_LENGTH, BEAM_WALL, BEAM_INNER_HEIGHT)),
        material="beam_gray",
        origin=Origin(xyz=(0.0, (BEAM_WIDTH / 2.0) - (BEAM_WALL / 2.0), 0.0)),
        name="guide_wall_pos_y",
    )
    support_beam.visual(
        Box((BEAM_LENGTH, BEAM_WALL, BEAM_INNER_HEIGHT)),
        material="beam_gray",
        origin=Origin(xyz=(0.0, -((BEAM_WIDTH / 2.0) - (BEAM_WALL / 2.0)), 0.0)),
        name="guide_wall_neg_y",
    )
    support_beam.visual(
        Box((BOTTOM_LIP_LENGTH, BOTTOM_LIP_WIDTH, BEAM_WALL)),
        material="beam_gray",
        origin=Origin(
            xyz=(
                0.0,
                (SLOT_WIDTH / 2.0) + (BOTTOM_LIP_WIDTH / 2.0),
                -((BEAM_HEIGHT / 2.0) - (BEAM_WALL / 2.0)),
            )
        ),
        name="bottom_lip_pos_y",
    )
    support_beam.visual(
        Box((BOTTOM_LIP_LENGTH, BOTTOM_LIP_WIDTH, BEAM_WALL)),
        material="beam_gray",
        origin=Origin(
            xyz=(
                0.0,
                -((SLOT_WIDTH / 2.0) + (BOTTOM_LIP_WIDTH / 2.0)),
                -((BEAM_HEIGHT / 2.0) - (BEAM_WALL / 2.0)),
            )
        ),
        name="bottom_lip_neg_y",
    )
    support_beam.visual(
        Box((BEAM_END_CAP, BEAM_WIDTH, BEAM_HEIGHT)),
        material="beam_gray",
        origin=Origin(xyz=(-(BEAM_LENGTH / 2.0) + (BEAM_END_CAP / 2.0), 0.0, 0.0)),
        name="end_cap_neg_x",
    )
    support_beam.visual(
        Box((BEAM_END_CAP, BEAM_WIDTH, BEAM_HEIGHT)),
        material="beam_gray",
        origin=Origin(xyz=((BEAM_LENGTH / 2.0) - (BEAM_END_CAP / 2.0), 0.0, 0.0)),
        name="end_cap_pos_x",
    )
    support_beam.visual(
        Box((0.18, BEAM_WIDTH * 0.70, BEAM_WALL)),
        material="beam_gray",
        origin=Origin(xyz=(-0.30, 0.0, (BEAM_HEIGHT / 2.0) + (BEAM_WALL / 2.0))),
        name="mount_pad_neg_x",
    )
    support_beam.visual(
        Box((0.18, BEAM_WIDTH * 0.70, BEAM_WALL)),
        material="beam_gray",
        origin=Origin(xyz=(0.30, 0.0, (BEAM_HEIGHT / 2.0) + (BEAM_WALL / 2.0))),
        name="mount_pad_pos_x",
    )

    hanging_carriage = model.part("hanging_carriage")
    hanging_carriage.visual(
        Box((TROLLEY_LENGTH, TROLLEY_WIDTH, TROLLEY_HEIGHT)),
        material="carriage_orange",
        origin=Origin(xyz=(0.0, 0.0, TROLLEY_Z)),
        name="upper_trolley",
    )
    hanging_carriage.visual(
        Box((0.10, 0.080, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, TROLLEY_Z + (TROLLEY_HEIGHT / 2.0) + 0.006)),
        material="carriage_orange",
        name="upper_trolley_cap",
    )
    hanging_carriage.visual(
        Box((RUNNER_PAD_LENGTH, RUNNER_PAD_WIDTH, RUNNER_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, RUNNER_PAD_Y, RUNNER_PAD_Z)),
        material="runner_black",
        name="runner_left",
    )
    hanging_carriage.visual(
        Box((RUNNER_PAD_LENGTH, RUNNER_PAD_WIDTH, RUNNER_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -RUNNER_PAD_Y, RUNNER_PAD_Z)),
        material="runner_black",
        name="runner_right",
    )
    hanging_carriage.visual(
        Box((STEM_LENGTH, STEM_WIDTH, STEM_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, STEM_Z)),
        material="stem_black",
        name="hanger_stem",
    )
    hanging_carriage.visual(
        Box((LOWER_BODY_LENGTH, LOWER_BODY_WIDTH, LOWER_BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, LOWER_BODY_Z)),
        material="carriage_orange",
        name="lower_body",
    )
    hanging_carriage.visual(
        Box((0.10, 0.084, 0.018)),
        origin=Origin(
            xyz=(0.0, 0.0, LOWER_BODY_Z + (LOWER_BODY_HEIGHT / 2.0) + 0.009)
        ),
        material="carriage_orange",
        name="lower_body_boss",
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=support_beam,
        child=hanging_carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=450.0,
            velocity=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_beam = object_model.get_part("support_beam")
    hanging_carriage = object_model.get_part("hanging_carriage")
    slide = object_model.get_articulation("beam_to_carriage")

    guide_wall_pos_y = support_beam.get_visual("guide_wall_pos_y")
    guide_wall_neg_y = support_beam.get_visual("guide_wall_neg_y")
    upper_trolley = hanging_carriage.get_visual("upper_trolley")
    upper_trolley_cap = hanging_carriage.get_visual("upper_trolley_cap")
    runner_left = hanging_carriage.get_visual("runner_left")
    runner_right = hanging_carriage.get_visual("runner_right")
    hanger_stem = hanging_carriage.get_visual("hanger_stem")
    lower_body = hanging_carriage.get_visual("lower_body")
    lower_body_boss = hanging_carriage.get_visual("lower_body_boss")

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
        "parts_exist",
        support_beam is not None and hanging_carriage is not None and slide is not None,
        "support beam, hanging carriage, or prismatic joint could not be resolved",
    )
    ctx.check(
        "slide_axis_is_guide_axis",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic guide axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "slide_limits_are_symmetric",
        slide.motion_limits is not None
        and slide.motion_limits.lower == -TRAVEL_HALF_RANGE
        and slide.motion_limits.upper == TRAVEL_HALF_RANGE,
        "carriage travel limits should stay centered on the beam",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            hanging_carriage,
            support_beam,
            axes="yz",
            inner_elem=upper_trolley,
            margin=0.0,
            name="upper_trolley_fits_inside_beam_profile",
        )
        ctx.expect_contact(
            hanging_carriage,
            hanging_carriage,
            elem_a=upper_trolley_cap,
            elem_b=upper_trolley,
            contact_tol=1e-6,
            name="upper_trolley_cap_is_supported",
        )
        ctx.expect_contact(
            hanging_carriage,
            hanging_carriage,
            elem_a=runner_left,
            elem_b=upper_trolley,
            contact_tol=1e-6,
            name="left_runner_pad_is_supported_by_trolley",
        )
        ctx.expect_contact(
            hanging_carriage,
            hanging_carriage,
            elem_a=runner_right,
            elem_b=upper_trolley,
            contact_tol=1e-6,
            name="right_runner_pad_is_supported_by_trolley",
        )
        ctx.expect_contact(
            hanging_carriage,
            hanging_carriage,
            elem_a=hanger_stem,
            elem_b=upper_trolley,
            contact_tol=1e-6,
            name="hanger_stem_is_supported_by_trolley",
        )
        ctx.expect_contact(
            hanging_carriage,
            hanging_carriage,
            elem_a=hanger_stem,
            elem_b=lower_body,
            contact_tol=1e-6,
            name="hanger_stem_supports_lower_body",
        )
        ctx.expect_contact(
            hanging_carriage,
            hanging_carriage,
            elem_a=lower_body_boss,
            elem_b=lower_body,
            contact_tol=1e-6,
            name="lower_body_boss_is_supported",
        )
        ctx.expect_contact(
            hanging_carriage,
            support_beam,
            elem_a=runner_left,
            elem_b=guide_wall_pos_y,
            contact_tol=1e-6,
            name="left_runner_pad_contacts_positive_guide_wall",
        )
        ctx.expect_contact(
            hanging_carriage,
            support_beam,
            elem_a=runner_right,
            elem_b=guide_wall_neg_y,
            contact_tol=1e-6,
            name="right_runner_pad_contacts_negative_guide_wall",
        )
        ctx.expect_gap(
            support_beam,
            hanging_carriage,
            axis="z",
            min_gap=0.015,
            max_gap=0.060,
            negative_elem=lower_body,
            name="lower_body_hangs_below_beam",
        )

    with ctx.pose({slide: slide.motion_limits.lower}):
        ctx.expect_within(
            hanging_carriage,
            support_beam,
            axes="x",
            inner_elem=upper_trolley,
            margin=0.0,
            name="upper_trolley_stays_inside_beam_at_lower_limit",
        )

    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_within(
            hanging_carriage,
            support_beam,
            axes="x",
            inner_elem=upper_trolley,
            margin=0.0,
            name="upper_trolley_stays_inside_beam_at_upper_limit",
        )

    with ctx.pose({slide: -0.30}):
        left_position = ctx.part_world_position(hanging_carriage)
    with ctx.pose({slide: 0.30}):
        right_position = ctx.part_world_position(hanging_carriage)

    if left_position is None or right_position is None:
        ctx.fail("carriage_pose_positions_available", "could not resolve carriage world positions")
    else:
        dx = right_position[0] - left_position[0]
        dy = abs(right_position[1] - left_position[1])
        dz = abs(right_position[2] - left_position[2])
        ctx.check(
            "carriage_moves_only_along_guide_axis",
            abs(dx - 0.60) <= 0.002 and dy <= 1e-6 and dz <= 1e-6,
            (
                "expected 0.60 m translation along +X only; "
                f"observed dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
