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


BODY_DEPTH = 0.34
BODY_WIDTH = 0.22
BODY_HEIGHT = 0.09
FLOOR_THICKNESS = 0.008
WALL_THICKNESS = 0.008

HINGE_RADIUS = 0.006
HINGE_AXIS_X = -(BODY_DEPTH / 2.0) - 0.008
HINGE_AXIS_Z = BODY_HEIGHT + HINGE_RADIUS
BODY_KNUCKLE_LENGTH = 0.048
BODY_KNUCKLE_Y = 0.065
LID_KNUCKLE_LENGTH = 0.074

SKIRT_THICKNESS = 0.007
LID_DEPTH = 0.3558
LID_WIDTH = 0.2356
LID_TOP_THICKNESS = 0.006
LID_PANEL_BOTTOM_Z = 0.016
LID_PANEL_CENTER_Z = LID_PANEL_BOTTOM_Z + (LID_TOP_THICKNESS / 2.0)
SKIRT_BOTTOM_Z = -0.0055
SKIRT_HEIGHT = LID_PANEL_BOTTOM_Z - SKIRT_BOTTOM_Z
SKIRT_CENTER_Z = (LID_PANEL_BOTTOM_Z + SKIRT_BOTTOM_Z) / 2.0
REAR_BEAM_LENGTH = 0.018
REAR_BEAM_HEIGHT = LID_PANEL_BOTTOM_Z
SIDE_SKIRT_LENGTH = LID_DEPTH - SKIRT_THICKNESS - REAR_BEAM_LENGTH
SIDE_SKIRT_CENTER_X = REAR_BEAM_LENGTH + (SIDE_SKIRT_LENGTH / 2.0)

DATUM_PAD_DEPTH = 0.022
DATUM_PAD_WIDTH = 0.018
DATUM_PAD_HEIGHT = 0.0015
DATUM_PAD_X = (BODY_DEPTH / 2.0) - WALL_THICKNESS - (DATUM_PAD_DEPTH / 2.0)
DATUM_PAD_Y = 0.07
DATUM_PAD_TOP_Z = BODY_HEIGHT + DATUM_PAD_HEIGHT
LANDING_PAD_HEIGHT = LID_PANEL_BOTTOM_Z - (DATUM_PAD_TOP_Z - HINGE_AXIS_Z)
LANDING_PAD_CENTER_Z = (LID_PANEL_BOTTOM_Z + (DATUM_PAD_TOP_Z - HINGE_AXIS_Z)) / 2.0
LANDING_PAD_X = DATUM_PAD_X - HINGE_AXIS_X

INDEX_MARK_THICKNESS = 0.0015
INDEX_MARK_WIDTH = 0.022
INDEX_MARK_HEIGHT = 0.018
BODY_INDEX_Z = BODY_HEIGHT + 0.004
LID_INDEX_Z = 0.006

ADJUSTER_BASE_RADIUS = 0.013
ADJUSTER_BASE_HEIGHT = 0.004
ADJUSTER_CAP_RADIUS = 0.008
ADJUSTER_CAP_HEIGHT = 0.006
ADJUSTER_X = 0.292
ADJUSTER_Y = 0.074

REFERENCE_RAIL_LENGTH = 0.16
REFERENCE_RAIL_WIDTH = 0.004
REFERENCE_RAIL_HEIGHT = 0.002
REFERENCE_RAIL_X = 0.205
REFERENCE_RAIL_Y = 0.048


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_sewing_box")

    model.material("body_anthracite", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("lid_graphite", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("datum_ivory", rgba=(0.88, 0.87, 0.82, 1.0))
    model.material("hinge_bronze", rgba=(0.62, 0.52, 0.34, 1.0))
    model.material("signal_red", rgba=(0.75, 0.12, 0.10, 1.0))
    model.material("adjuster_steel", rgba=(0.60, 0.63, 0.68, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_DEPTH, BODY_WIDTH, FLOOR_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_THICKNESS / 2.0)),
        material="body_anthracite",
        name="floor",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=((BODY_DEPTH / 2.0) - (WALL_THICKNESS / 2.0), 0.0, BODY_HEIGHT / 2.0)),
        material="body_anthracite",
        name="front_wall",
    )
    body.visual(
        Box((WALL_THICKNESS, BODY_WIDTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-(BODY_DEPTH / 2.0) + (WALL_THICKNESS / 2.0), 0.0, BODY_HEIGHT / 2.0)),
        material="body_anthracite",
        name="rear_wall",
    )
    body.visual(
        Box((BODY_DEPTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, -((BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0)), BODY_HEIGHT / 2.0)),
        material="body_anthracite",
        name="left_wall",
    )
    body.visual(
        Box((BODY_DEPTH - (2.0 * WALL_THICKNESS), WALL_THICKNESS, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, (BODY_WIDTH / 2.0) - (WALL_THICKNESS / 2.0), BODY_HEIGHT / 2.0)),
        material="body_anthracite",
        name="right_wall",
    )
    body.visual(
        Box((DATUM_PAD_DEPTH, DATUM_PAD_WIDTH, DATUM_PAD_HEIGHT)),
        origin=Origin(xyz=(DATUM_PAD_X, -DATUM_PAD_Y, BODY_HEIGHT + (DATUM_PAD_HEIGHT / 2.0))),
        material="datum_ivory",
        name="left_datum_pad",
    )
    body.visual(
        Box((DATUM_PAD_DEPTH, DATUM_PAD_WIDTH, DATUM_PAD_HEIGHT)),
        origin=Origin(xyz=(DATUM_PAD_X, DATUM_PAD_Y, BODY_HEIGHT + (DATUM_PAD_HEIGHT / 2.0))),
        material="datum_ivory",
        name="right_datum_pad",
    )
    body.visual(
        Box((0.008, BODY_KNUCKLE_LENGTH, 0.012)),
        origin=Origin(xyz=(HINGE_AXIS_X + 0.004, -BODY_KNUCKLE_Y, HINGE_AXIS_Z)),
        material="hinge_bronze",
        name="left_hinge_leaf",
    )
    body.visual(
        Box((0.008, BODY_KNUCKLE_LENGTH, 0.012)),
        origin=Origin(xyz=(HINGE_AXIS_X + 0.004, BODY_KNUCKLE_Y, HINGE_AXIS_Z)),
        material="hinge_bronze",
        name="right_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(HINGE_AXIS_X, -BODY_KNUCKLE_Y, HINGE_AXIS_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="hinge_bronze",
        name="left_hinge_knuckle",
    )
    body.visual(
        Cylinder(radius=HINGE_RADIUS, length=BODY_KNUCKLE_LENGTH),
        origin=Origin(xyz=(HINGE_AXIS_X, BODY_KNUCKLE_Y, HINGE_AXIS_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="hinge_bronze",
        name="right_hinge_knuckle",
    )
    body.visual(
        Box((INDEX_MARK_THICKNESS, INDEX_MARK_WIDTH, INDEX_MARK_HEIGHT)),
        origin=Origin(
            xyz=((BODY_DEPTH / 2.0) + (INDEX_MARK_THICKNESS / 2.0), 0.0, BODY_INDEX_Z),
        ),
        material="signal_red",
        name="body_index_mark",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_DEPTH, LID_WIDTH, LID_TOP_THICKNESS)),
        origin=Origin(xyz=(LID_DEPTH / 2.0, 0.0, LID_PANEL_CENTER_Z)),
        material="lid_graphite",
        name="top_panel",
    )
    lid.visual(
        Box((SKIRT_THICKNESS, LID_WIDTH, SKIRT_HEIGHT)),
        origin=Origin(xyz=(LID_DEPTH - (SKIRT_THICKNESS / 2.0), 0.0, SKIRT_CENTER_Z)),
        material="lid_graphite",
        name="front_skirt",
    )
    lid.visual(
        Box((SIDE_SKIRT_LENGTH, SKIRT_THICKNESS, SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(SIDE_SKIRT_CENTER_X, -((LID_WIDTH / 2.0) - (SKIRT_THICKNESS / 2.0)), SKIRT_CENTER_Z),
        ),
        material="lid_graphite",
        name="left_skirt",
    )
    lid.visual(
        Box((SIDE_SKIRT_LENGTH, SKIRT_THICKNESS, SKIRT_HEIGHT)),
        origin=Origin(
            xyz=(SIDE_SKIRT_CENTER_X, (LID_WIDTH / 2.0) - (SKIRT_THICKNESS / 2.0), SKIRT_CENTER_Z),
        ),
        material="lid_graphite",
        name="right_skirt",
    )
    lid.visual(
        Box((REAR_BEAM_LENGTH, LID_KNUCKLE_LENGTH, REAR_BEAM_HEIGHT)),
        origin=Origin(xyz=(REAR_BEAM_LENGTH / 2.0, 0.0, REAR_BEAM_HEIGHT / 2.0)),
        material="hinge_bronze",
        name="rear_hinge_beam",
    )
    lid.visual(
        Cylinder(radius=HINGE_RADIUS, length=LID_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="hinge_bronze",
        name="center_hinge_knuckle",
    )
    lid.visual(
        Box((DATUM_PAD_DEPTH, DATUM_PAD_WIDTH, LANDING_PAD_HEIGHT)),
        origin=Origin(xyz=(LANDING_PAD_X, -DATUM_PAD_Y, LANDING_PAD_CENTER_Z)),
        material="datum_ivory",
        name="left_landing_pad",
    )
    lid.visual(
        Box((DATUM_PAD_DEPTH, DATUM_PAD_WIDTH, LANDING_PAD_HEIGHT)),
        origin=Origin(xyz=(LANDING_PAD_X, DATUM_PAD_Y, LANDING_PAD_CENTER_Z)),
        material="datum_ivory",
        name="right_landing_pad",
    )
    lid.visual(
        Box((INDEX_MARK_THICKNESS, INDEX_MARK_WIDTH, INDEX_MARK_HEIGHT)),
        origin=Origin(xyz=(LID_DEPTH + (INDEX_MARK_THICKNESS / 2.0), 0.0, LID_INDEX_Z)),
        material="signal_red",
        name="lid_index_mark",
    )
    lid.visual(
        Box((REFERENCE_RAIL_LENGTH, REFERENCE_RAIL_WIDTH, REFERENCE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(REFERENCE_RAIL_X, -REFERENCE_RAIL_Y, LID_PANEL_BOTTOM_Z + LID_TOP_THICKNESS + (REFERENCE_RAIL_HEIGHT / 2.0)),
        ),
        material="datum_ivory",
        name="left_reference_rail",
    )
    lid.visual(
        Box((REFERENCE_RAIL_LENGTH, REFERENCE_RAIL_WIDTH, REFERENCE_RAIL_HEIGHT)),
        origin=Origin(
            xyz=(REFERENCE_RAIL_X, REFERENCE_RAIL_Y, LID_PANEL_BOTTOM_Z + LID_TOP_THICKNESS + (REFERENCE_RAIL_HEIGHT / 2.0)),
        ),
        material="datum_ivory",
        name="right_reference_rail",
    )
    lid.visual(
        Cylinder(radius=ADJUSTER_BASE_RADIUS, length=ADJUSTER_BASE_HEIGHT),
        origin=Origin(
            xyz=(ADJUSTER_X, -ADJUSTER_Y, LID_PANEL_BOTTOM_Z + LID_TOP_THICKNESS + (ADJUSTER_BASE_HEIGHT / 2.0)),
        ),
        material="adjuster_steel",
        name="left_adjuster_base",
    )
    lid.visual(
        Cylinder(radius=ADJUSTER_BASE_RADIUS, length=ADJUSTER_BASE_HEIGHT),
        origin=Origin(
            xyz=(ADJUSTER_X, ADJUSTER_Y, LID_PANEL_BOTTOM_Z + LID_TOP_THICKNESS + (ADJUSTER_BASE_HEIGHT / 2.0)),
        ),
        material="adjuster_steel",
        name="right_adjuster_base",
    )
    lid.visual(
        Cylinder(radius=ADJUSTER_CAP_RADIUS, length=ADJUSTER_CAP_HEIGHT),
        origin=Origin(
            xyz=(
                ADJUSTER_X,
                -ADJUSTER_Y,
                LID_PANEL_BOTTOM_Z + LID_TOP_THICKNESS + ADJUSTER_BASE_HEIGHT + (ADJUSTER_CAP_HEIGHT / 2.0),
            ),
        ),
        material="adjuster_steel",
        name="left_adjuster_cap",
    )
    lid.visual(
        Cylinder(radius=ADJUSTER_CAP_RADIUS, length=ADJUSTER_CAP_HEIGHT),
        origin=Origin(
            xyz=(
                ADJUSTER_X,
                ADJUSTER_Y,
                LID_PANEL_BOTTOM_Z + LID_TOP_THICKNESS + ADJUSTER_BASE_HEIGHT + (ADJUSTER_CAP_HEIGHT / 2.0),
            ),
        ),
        material="adjuster_steel",
        name="right_adjuster_cap",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(HINGE_AXIS_X, 0.0, HINGE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=1.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")
    limits = hinge.motion_limits

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
        "hinge_axis_opens_upward_from_rear",
        tuple(hinge.axis) == (0.0, -1.0, 0.0),
        f"axis={hinge.axis}",
    )
    ctx.check(
        "hinge_limits_match_simple_box_lid_motion",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and 1.8 <= limits.upper <= 2.05,
        f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="left_landing_pad",
            elem_b="left_datum_pad",
            name="left_datum_pad_supports_closed_lid",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="right_landing_pad",
            elem_b="right_datum_pad",
            name="right_datum_pad_supports_closed_lid",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="front_skirt",
            negative_elem="front_wall",
            min_gap=0.0006,
            max_gap=0.0012,
            name="front_seam_gap_is_controlled",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="y",
            positive_elem="left_wall",
            negative_elem="left_skirt",
            min_gap=0.0006,
            max_gap=0.0012,
            name="left_seam_gap_is_controlled",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem="right_skirt",
            negative_elem="right_wall",
            min_gap=0.0006,
            max_gap=0.0012,
            name="right_seam_gap_is_controlled",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            elem_a="lid_index_mark",
            elem_b="body_index_mark",
            min_overlap=0.006,
            name="index_marks_register_across_the_front_seam",
        )

    with ctx.pose({hinge: 1.95}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_wall",
            min_gap=0.14,
            name="front_edge_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
