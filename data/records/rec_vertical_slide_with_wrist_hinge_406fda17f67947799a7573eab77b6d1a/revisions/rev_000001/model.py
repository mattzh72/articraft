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


BODY_WIDTH = 0.62
BODY_DEPTH = 0.52
BASE_HEIGHT = 0.12
MAST_HEIGHT = 0.86
MAST_COLUMN_WIDTH = 0.07
MAST_COLUMN_DEPTH = 0.08
MAST_COLUMN_Y = -0.18
MAST_COLUMN_X = 0.205
BACK_PANEL_DEPTH = 0.025
TOP_CAP_HEIGHT = 0.06

PLATEN_WIDTH = 0.54
PLATEN_DEPTH = 0.34
PLATEN_THICKNESS = 0.045
PLATEN_CENTER_Y = 0.04
PLATEN_LIFT = 0.52

FACEPLATE_WIDTH = 0.52
FACEPLATE_THICKNESS = 0.012
FACEPLATE_HEIGHT = 0.10


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_lift")

    body_gray = model.material("body_gray", rgba=(0.22, 0.23, 0.25, 1.0))
    platen_gray = model.material("platen_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.77, 0.18, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, BODY_DEPTH, BASE_HEIGHT)),
        material=body_gray,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
        name="base_shell",
    )
    body.visual(
        Box((MAST_COLUMN_WIDTH, MAST_COLUMN_DEPTH, MAST_HEIGHT)),
        material=body_gray,
        origin=Origin(xyz=(-MAST_COLUMN_X, MAST_COLUMN_Y, BASE_HEIGHT + MAST_HEIGHT / 2.0)),
        name="left_mast",
    )
    body.visual(
        Box((MAST_COLUMN_WIDTH, MAST_COLUMN_DEPTH, MAST_HEIGHT)),
        material=body_gray,
        origin=Origin(xyz=(MAST_COLUMN_X, MAST_COLUMN_Y, BASE_HEIGHT + MAST_HEIGHT / 2.0)),
        name="right_mast",
    )
    body.visual(
        Box((0.48, BACK_PANEL_DEPTH, 0.84)),
        material=body_gray,
        origin=Origin(xyz=(0.0, -0.225, BASE_HEIGHT + 0.84 / 2.0)),
        name="back_panel",
    )
    body.visual(
        Box((0.50, 0.11, TOP_CAP_HEIGHT)),
        material=body_gray,
        origin=Origin(xyz=(0.0, -0.18, BASE_HEIGHT + MAST_HEIGHT + TOP_CAP_HEIGHT / 2.0)),
        name="top_cap",
    )

    platen = model.part("platen")
    platen.visual(
        Box((PLATEN_WIDTH, PLATEN_DEPTH, PLATEN_THICKNESS)),
        material=platen_gray,
        origin=Origin(xyz=(0.0, 0.055, PLATEN_THICKNESS / 2.0)),
        name="platen_deck",
    )
    platen.visual(
        Box((0.26, 0.03, 0.28)),
        material=platen_gray,
        origin=Origin(xyz=(0.0, -0.095, 0.14)),
        name="carriage_spine",
    )
    platen.visual(
        Box((0.05, 0.09, 0.16)),
        material=platen_gray,
        origin=Origin(xyz=(-0.105, -0.085, 0.08)),
        name="left_guide",
    )
    platen.visual(
        Box((0.05, 0.09, 0.16)),
        material=platen_gray,
        origin=Origin(xyz=(0.105, -0.085, 0.08)),
        name="right_guide",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Box((FACEPLATE_WIDTH, FACEPLATE_THICKNESS, FACEPLATE_HEIGHT)),
        material=safety_yellow,
        origin=Origin(xyz=(0.0, FACEPLATE_THICKNESS / 2.0, FACEPLATE_HEIGHT / 2.0)),
        name="face_panel",
    )
    faceplate.visual(
        Box((0.50, 0.018, 0.016)),
        material=safety_yellow,
        origin=Origin(xyz=(0.0, 0.009, 0.008)),
        name="hinge_leaf",
    )

    model.articulation(
        "body_to_platen",
        ArticulationType.PRISMATIC,
        parent=body,
        child=platen,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.20,
            lower=0.0,
            upper=PLATEN_LIFT,
        ),
    )

    model.articulation(
        "platen_to_faceplate",
        ArticulationType.REVOLUTE,
        parent=platen,
        child=faceplate,
        origin=Origin(xyz=(0.0, 0.225, PLATEN_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    platen = object_model.get_part("platen")
    faceplate = object_model.get_part("faceplate")
    lift = object_model.get_articulation("body_to_platen")
    tilt = object_model.get_articulation("platen_to_faceplate")

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
        "lift_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        f"joint_type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.check(
        "faceplate_joint_is_lateral_revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (1.0, 0.0, 0.0),
        f"joint_type={tilt.articulation_type}, axis={tilt.axis}",
    )

    with ctx.pose({lift: 0.0, tilt: 0.0}):
        ctx.expect_gap(
            platen,
            body,
            axis="z",
            positive_elem="platen_deck",
            negative_elem="base_shell",
            max_gap=0.001,
            max_penetration=0.0,
            name="platen_rests_on_grounded_body",
        )
        ctx.expect_contact(
            platen,
            faceplate,
            elem_a="platen_deck",
            elem_b="hinge_leaf",
            name="faceplate_is_hinged_from_platen_front",
        )
        ctx.expect_overlap(platen, body, axes="x", min_overlap=0.50, name="platen_is_broadly_supported")
        ctx.expect_within(faceplate, platen, axes="x", margin=0.02, name="faceplate_matches_platen_width")

    with ctx.pose({lift: 0.45, tilt: 0.0}):
        ctx.expect_origin_gap(
            platen,
            body,
            axis="z",
            min_gap=0.54,
            name="platen_moves_upward_on_lift_axis",
        )

    with ctx.pose({lift: 0.0, tilt: 0.0}):
        faceplate_closed = ctx.part_element_world_aabb(faceplate, elem="face_panel")
    with ctx.pose({lift: 0.0, tilt: 1.25}):
        faceplate_folded = ctx.part_element_world_aabb(faceplate, elem="face_panel")

    faceplate_motion_ok = (
        faceplate_closed is not None
        and faceplate_folded is not None
        and faceplate_folded[1][2] < faceplate_closed[1][2] - 0.05
        and faceplate_folded[0][1] < faceplate_closed[0][1] - 0.05
    )
    ctx.check(
        "faceplate_folds_down_over_platen",
        faceplate_motion_ok,
        f"closed={faceplate_closed}, folded={faceplate_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
