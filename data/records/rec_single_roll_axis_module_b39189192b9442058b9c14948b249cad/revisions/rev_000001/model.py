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


TOP_PLATE_LENGTH = 0.240
TOP_PLATE_DEPTH = 0.090
TOP_PLATE_THICK = 0.012
TOP_PLATE_Z = 0.110

SUPPORT_CENTER_X = 0.066
SUPPORT_BLOCK_WIDTH = 0.020
SUPPORT_BLOCK_DEPTH = 0.042
SUPPORT_BLOCK_HEIGHT = 0.036
SUPPORT_BLOCK_Z = 0.036
SUPPORT_HANGER_WIDTH = 0.028
SUPPORT_HANGER_DEPTH = 0.028
SUPPORT_HANGER_HEIGHT = 0.052
SUPPORT_HANGER_Z = 0.076
SUPPORT_STIFFENER_WIDTH = 0.156
SUPPORT_STIFFENER_DEPTH = 0.020
SUPPORT_STIFFENER_HEIGHT = 0.012
SUPPORT_STIFFENER_Z = 0.098

SHAFT_LENGTH = 0.220
SHAFT_RADIUS = 0.0125
ROTOR_DRUM_LENGTH = 0.070
ROTOR_DRUM_RADIUS = 0.025
JOURNAL_RADIUS = 0.018
JOURNAL_LENGTH = 0.014
JOURNAL_CENTER_X = SUPPORT_CENTER_X
FLANGE_RADIUS = 0.040
FLANGE_THICK = 0.016
FLANGE_HUB_RADIUS = 0.031
FLANGE_HUB_LENGTH = 0.026
FLANGE_HUB_CENTER_X = 0.093
FLANGE_CENTER_X = 0.108


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_roll_axis_spindle")

    model.material("powder_coat_charcoal", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("ground_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    bracket = model.part("support_bracket")
    bracket.visual(
        Box((TOP_PLATE_LENGTH, TOP_PLATE_DEPTH, TOP_PLATE_THICK)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_Z)),
        material="powder_coat_charcoal",
        name="top_plate",
    )
    bracket.visual(
        Box((SUPPORT_STIFFENER_WIDTH, SUPPORT_STIFFENER_DEPTH, SUPPORT_STIFFENER_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_STIFFENER_Z)),
        material="powder_coat_charcoal",
        name="stiffener",
    )
    for side_name, x_center in (("left", -SUPPORT_CENTER_X), ("right", SUPPORT_CENTER_X)):
        bracket.visual(
            Box((SUPPORT_HANGER_WIDTH, SUPPORT_HANGER_DEPTH, SUPPORT_HANGER_HEIGHT)),
            origin=Origin(xyz=(x_center, 0.0, SUPPORT_HANGER_Z)),
            material="powder_coat_charcoal",
            name=f"{side_name}_hanger",
        )
        bracket.visual(
            Box((SUPPORT_BLOCK_WIDTH, SUPPORT_BLOCK_DEPTH, SUPPORT_BLOCK_HEIGHT)),
            origin=Origin(xyz=(x_center, 0.0, SUPPORT_BLOCK_Z)),
            material="powder_coat_charcoal",
            name=f"{side_name}_support_block",
        )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=SHAFT_RADIUS, length=SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="shaft",
    )
    spindle.visual(
        Cylinder(radius=ROTOR_DRUM_RADIUS, length=ROTOR_DRUM_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="rotor_drum",
    )
    spindle.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(xyz=(-JOURNAL_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="left_journal",
    )
    spindle.visual(
        Cylinder(radius=JOURNAL_RADIUS, length=JOURNAL_LENGTH),
        origin=Origin(xyz=(JOURNAL_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="right_journal",
    )
    spindle.visual(
        Cylinder(radius=FLANGE_HUB_RADIUS, length=FLANGE_HUB_LENGTH),
        origin=Origin(xyz=(FLANGE_HUB_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="flange_hub",
    )
    spindle.visual(
        Cylinder(radius=FLANGE_RADIUS, length=FLANGE_THICK),
        origin=Origin(xyz=(FLANGE_CENTER_X, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="ground_steel",
        name="output_flange",
    )

    model.articulation(
        "bracket_to_spindle_roll",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=spindle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=25.0, velocity=7.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("support_bracket")
    spindle = object_model.get_part("spindle")
    roll_joint = object_model.get_articulation("bracket_to_spindle_roll")

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
        "module_has_two_parts",
        len(object_model.parts) == 2,
        f"expected 2 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "module_has_single_roll_joint",
        len(object_model.articulations) == 1,
        f"expected 1 articulation, found {len(object_model.articulations)}",
    )
    ctx.check(
        "roll_axis_is_longitudinal_x",
        tuple(round(value, 6) for value in roll_joint.axis) == (1.0, 0.0, 0.0),
        f"expected roll axis (1, 0, 0), got {roll_joint.axis}",
    )
    ctx.check(
        "roll_limits_span_both_directions",
        roll_joint.motion_limits is not None
        and roll_joint.motion_limits.lower is not None
        and roll_joint.motion_limits.upper is not None
        and roll_joint.motion_limits.lower < 0.0 < roll_joint.motion_limits.upper,
        "roll joint should allow bidirectional rotation about the shaft axis",
    )
    ctx.expect_contact(bracket, spindle, name="spindle_is_carried_by_end_supports")
    ctx.expect_origin_distance(
        bracket,
        spindle,
        axes="xyz",
        max_dist=1e-6,
        name="spindle_origin_coincident_with_roll_axis",
    )
    ctx.expect_gap(
        bracket,
        spindle,
        axis="z",
        positive_elem="top_plate",
        negative_elem="output_flange",
        min_gap=0.050,
        name="top_plate_sits_above_rotating_flange",
    )
    ctx.expect_contact(
        bracket,
        spindle,
        elem_a="left_support_block",
        elem_b="left_journal",
        name="left_support_contacts_left_journal",
    )
    ctx.expect_contact(
        bracket,
        spindle,
        elem_a="right_support_block",
        elem_b="right_journal",
        name="right_support_contacts_right_journal",
    )
    ctx.expect_gap(
        spindle,
        bracket,
        axis="x",
        positive_elem="rotor_drum",
        negative_elem="left_support_block",
        min_gap=0.020,
        name="rotor_clears_left_support_block",
    )
    ctx.expect_gap(
        bracket,
        spindle,
        axis="x",
        positive_elem="right_support_block",
        negative_elem="rotor_drum",
        min_gap=0.020,
        name="rotor_clears_right_support_block",
    )

    with ctx.pose({roll_joint: 1.2}):
        ctx.expect_contact(bracket, spindle, name="spindle_remains_carried_when_rotated")
        ctx.expect_gap(
            bracket,
            spindle,
            axis="z",
            positive_elem="top_plate",
            negative_elem="output_flange",
            min_gap=0.050,
            name="top_plate_clearance_persists_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
