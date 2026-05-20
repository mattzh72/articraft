from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


MAST_HEIGHT = 2.25
UPRIGHT_WIDTH = 0.14
UPRIGHT_DEPTH = 0.16
UPRIGHT_CENTER_X = 0.355
CHANNEL_CAVITY_WIDTH = 0.082
CHANNEL_CAVITY_DEPTH = 0.115
CHANNEL_CAVITY_CENTER_Y = 0.0225
CHANNEL_SLOT_WIDTH = 0.054
CHANNEL_LIP_DEPTH = 0.024
CHANNEL_LIP_WIDTH = (CHANNEL_CAVITY_WIDTH - CHANNEL_SLOT_WIDTH) / 2.0
CHANNEL_LIP_CENTER_Y = (UPRIGHT_DEPTH / 2.0) - (CHANNEL_LIP_DEPTH / 2.0)

TOP_TIE_WIDTH = (2.0 * UPRIGHT_CENTER_X) + UPRIGHT_WIDTH
TOP_TIE_DEPTH = 0.10
TOP_TIE_HEIGHT = 0.10
TOP_TIE_CENTER_Z = MAST_HEIGHT - (TOP_TIE_HEIGHT / 2.0)

BOTTOM_TIE_WIDTH = 0.74
BOTTOM_TIE_DEPTH = 0.06
BOTTOM_TIE_HEIGHT = 0.12
BOTTOM_TIE_CENTER_Y = -0.10
BOTTOM_TIE_CENTER_Z = BOTTOM_TIE_HEIGHT / 2.0

FRAME_SIDE_X = 0.205
FRAME_SIDE_WIDTH = 0.04
FRAME_DEPTH = 0.07
FRAME_SIDE_HEIGHT = 0.86
TOP_BAR_WIDTH = 0.45
TOP_BAR_DEPTH = 0.05
TOP_BAR_HEIGHT = 0.08
TOP_BAR_CENTER_Z = 0.78
BOTTOM_BAR_WIDTH = 0.50
BOTTOM_BAR_DEPTH = 0.085
BOTTOM_BAR_HEIGHT = 0.14
BOTTOM_BAR_CENTER_Z = 0.18
ROLLER_BRACKET_X = 0.236
ROLLER_BRACKET_WIDTH = 0.05
ROLLER_BRACKET_DEPTH = 0.04
ROLLER_BRACKET_HEIGHT = 0.10
ROLLER_BRACKET_CENTER_Y = 0.005

BACKREST_POST_X = 0.18
BACKREST_POST_WIDTH = 0.035
BACKREST_POST_DEPTH = 0.03
BACKREST_POST_HEIGHT = 0.42
BACKREST_POST_CENTER_Z = 0.82 + (BACKREST_POST_HEIGHT / 2.0)
BACKREST_SLAT_WIDTH = 0.48
BACKREST_SLAT_DEPTH = 0.016
BACKREST_SLAT_HEIGHT = 0.018
BACKREST_SLAT_ZS = (0.90, 1.02, 1.14)

ROLLER_RADIUS = 0.024
ROLLER_LENGTH = 0.06
GUIDE_FACE_X = UPRIGHT_CENTER_X - (UPRIGHT_WIDTH / 2.0)
ROLLER_CENTER_X = GUIDE_FACE_X - ROLLER_RADIUS
ROLLER_CENTER_Y = 0.02
LOWER_ROLLER_Z = 0.25
UPPER_ROLLER_Z = 0.71

FORK_X = 0.185
FORK_WIDTH = 0.11
FORK_THICKNESS = 0.05
FORK_LENGTH = 1.03
FORK_TINE_START_Y = 0.045
FORK_TINE_CENTER_Z = 0.065
FORK_SHANK_DEPTH = 0.04
FORK_SHANK_HEIGHT = 0.30
FORK_SHANK_CENTER_Y = 0.02
FORK_SHANK_CENTER_Z = 0.20
FORK_HEEL_DEPTH = 0.09
FORK_HEEL_HEIGHT = 0.08
FORK_HEEL_CENTER_Y = 0.045
FORK_HEEL_CENTER_Z = 0.08

CARRIAGE_BASE_Z = 0.06
LIFT_TRAVEL = 0.84


def _combine(*solids: cq.Workplane) -> cq.Workplane:
    result = solids[0]
    for solid in solids[1:]:
        result = result.union(solid)
    return result


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _upright_shape(x_center: float) -> cq.Workplane:
    outer = _box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, MAST_HEIGHT), (x_center, 0.0, MAST_HEIGHT / 2.0))
    cavity = _box(
        (CHANNEL_CAVITY_WIDTH, CHANNEL_CAVITY_DEPTH, MAST_HEIGHT + 0.01),
        (x_center, CHANNEL_CAVITY_CENTER_Y, MAST_HEIGHT / 2.0),
    )
    lip_offset_x = (CHANNEL_SLOT_WIDTH / 2.0) + (CHANNEL_LIP_WIDTH / 2.0)
    left_lip = _box(
        (CHANNEL_LIP_WIDTH, CHANNEL_LIP_DEPTH, MAST_HEIGHT),
        (x_center - lip_offset_x, CHANNEL_LIP_CENTER_Y, MAST_HEIGHT / 2.0),
    )
    right_lip = _box(
        (CHANNEL_LIP_WIDTH, CHANNEL_LIP_DEPTH, MAST_HEIGHT),
        (x_center + lip_offset_x, CHANNEL_LIP_CENTER_Y, MAST_HEIGHT / 2.0),
    )
    return outer.cut(cavity).union(left_lip).union(right_lip)


def _frame_shape() -> cq.Workplane:
    left_side = _box(
        (FRAME_SIDE_WIDTH, FRAME_DEPTH, FRAME_SIDE_HEIGHT),
        (-FRAME_SIDE_X, 0.0, FRAME_SIDE_HEIGHT / 2.0),
    )
    right_side = _box(
        (FRAME_SIDE_WIDTH, FRAME_DEPTH, FRAME_SIDE_HEIGHT),
        (FRAME_SIDE_X, 0.0, FRAME_SIDE_HEIGHT / 2.0),
    )
    top_bar = _box((TOP_BAR_WIDTH, TOP_BAR_DEPTH, TOP_BAR_HEIGHT), (0.0, 0.0, TOP_BAR_CENTER_Z))
    bottom_bar = _box((BOTTOM_BAR_WIDTH, BOTTOM_BAR_DEPTH, BOTTOM_BAR_HEIGHT), (0.0, 0.01, BOTTOM_BAR_CENTER_Z))
    left_upper_bracket = _box(
        (ROLLER_BRACKET_WIDTH, ROLLER_BRACKET_DEPTH, ROLLER_BRACKET_HEIGHT),
        (-ROLLER_BRACKET_X, ROLLER_BRACKET_CENTER_Y, UPPER_ROLLER_Z),
    )
    left_lower_bracket = _box(
        (ROLLER_BRACKET_WIDTH, ROLLER_BRACKET_DEPTH, ROLLER_BRACKET_HEIGHT),
        (-ROLLER_BRACKET_X, ROLLER_BRACKET_CENTER_Y, LOWER_ROLLER_Z),
    )
    right_upper_bracket = _box(
        (ROLLER_BRACKET_WIDTH, ROLLER_BRACKET_DEPTH, ROLLER_BRACKET_HEIGHT),
        (ROLLER_BRACKET_X, ROLLER_BRACKET_CENTER_Y, UPPER_ROLLER_Z),
    )
    right_lower_bracket = _box(
        (ROLLER_BRACKET_WIDTH, ROLLER_BRACKET_DEPTH, ROLLER_BRACKET_HEIGHT),
        (ROLLER_BRACKET_X, ROLLER_BRACKET_CENTER_Y, LOWER_ROLLER_Z),
    )
    return _combine(
        left_side,
        right_side,
        top_bar,
        bottom_bar,
        left_upper_bracket,
        left_lower_bracket,
        right_upper_bracket,
        right_lower_bracket,
    )


def _backrest_shape() -> cq.Workplane:
    left_post = _box(
        (BACKREST_POST_WIDTH, BACKREST_POST_DEPTH, BACKREST_POST_HEIGHT),
        (-BACKREST_POST_X, 0.0, BACKREST_POST_CENTER_Z),
    )
    right_post = _box(
        (BACKREST_POST_WIDTH, BACKREST_POST_DEPTH, BACKREST_POST_HEIGHT),
        (BACKREST_POST_X, 0.0, BACKREST_POST_CENTER_Z),
    )
    slats = [
        _box(
            (BACKREST_SLAT_WIDTH, BACKREST_SLAT_DEPTH, BACKREST_SLAT_HEIGHT),
            (0.0, 0.0, z_pos),
        )
        for z_pos in BACKREST_SLAT_ZS
    ]
    return _combine(left_post, right_post, *slats)


def _roller_shape(x_center: float, z_center: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(ROLLER_RADIUS)
        .extrude(ROLLER_LENGTH)
        .translate((x_center, ROLLER_CENTER_Y - (ROLLER_LENGTH / 2.0), z_center))
    )


def _fork_shape(x_center: float) -> cq.Workplane:
    shank = _box(
        (FORK_WIDTH, FORK_SHANK_DEPTH, FORK_SHANK_HEIGHT),
        (x_center, FORK_SHANK_CENTER_Y, FORK_SHANK_CENTER_Z),
    )
    heel = _box(
        (FORK_WIDTH, FORK_HEEL_DEPTH, FORK_HEEL_HEIGHT),
        (x_center, FORK_HEEL_CENTER_Y, FORK_HEEL_CENTER_Z),
    )
    tine = (
        cq.Workplane("XY")
        .box(FORK_WIDTH, FORK_LENGTH, FORK_THICKNESS, centered=(True, False, True))
        .translate((x_center, FORK_TINE_START_Y, FORK_TINE_CENTER_Z))
    )
    tine = tine.faces(">Y").edges("<Z").chamfer(0.03)
    return _combine(shank, heel, tine)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_fork_mast")

    model.material("mast_steel", rgba=(0.23, 0.24, 0.27, 1.0))
    model.material("carriage_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("fork_steel", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("roller_black", rgba=(0.08, 0.08, 0.09, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_upright_shape(-UPRIGHT_CENTER_X), "mast_left_upright"),
        material="mast_steel",
        name="left_upright",
    )
    mast.visual(
        mesh_from_cadquery(_upright_shape(UPRIGHT_CENTER_X), "mast_right_upright"),
        material="mast_steel",
        name="right_upright",
    )
    mast.visual(
        mesh_from_cadquery(
            _box((TOP_TIE_WIDTH, TOP_TIE_DEPTH, TOP_TIE_HEIGHT), (0.0, -0.01, TOP_TIE_CENTER_Z)),
            "mast_top_tie",
        ),
        material="mast_steel",
        name="top_tie",
    )
    mast.visual(
        mesh_from_cadquery(
            _box(
                (BOTTOM_TIE_WIDTH, BOTTOM_TIE_DEPTH, BOTTOM_TIE_HEIGHT),
                (0.0, BOTTOM_TIE_CENTER_Y, BOTTOM_TIE_CENTER_Z),
            ),
            "mast_bottom_tie",
        ),
        material="mast_steel",
        name="bottom_tie",
    )
    mast.inertial = Inertial.from_geometry(
        geometry=Box((0.86, 0.16, MAST_HEIGHT)),
        mass=180.0,
        origin=Origin(xyz=(0.0, 0.0, MAST_HEIGHT / 2.0)),
    )

    carriage = model.part("fork_frame")
    carriage.visual(
        mesh_from_cadquery(_frame_shape(), "fork_frame_structure"),
        material="carriage_steel",
        name="frame",
    )
    carriage.visual(
        mesh_from_cadquery(_backrest_shape(), "fork_frame_backrest"),
        material="carriage_steel",
        name="backrest",
    )
    carriage.visual(
        mesh_from_cadquery(_fork_shape(-FORK_X), "left_fork"),
        material="fork_steel",
        name="left_fork",
    )
    carriage.visual(
        mesh_from_cadquery(_fork_shape(FORK_X), "right_fork"),
        material="fork_steel",
        name="right_fork",
    )
    carriage.visual(
        mesh_from_cadquery(_roller_shape(-ROLLER_CENTER_X, UPPER_ROLLER_Z), "left_upper_roller"),
        material="roller_black",
        name="left_upper_roller",
    )
    carriage.visual(
        mesh_from_cadquery(_roller_shape(-ROLLER_CENTER_X, LOWER_ROLLER_Z), "left_lower_roller"),
        material="roller_black",
        name="left_lower_roller",
    )
    carriage.visual(
        mesh_from_cadquery(_roller_shape(ROLLER_CENTER_X, UPPER_ROLLER_Z), "right_upper_roller"),
        material="roller_black",
        name="right_upper_roller",
    )
    carriage.visual(
        mesh_from_cadquery(_roller_shape(ROLLER_CENTER_X, LOWER_ROLLER_Z), "right_lower_roller"),
        material="roller_black",
        name="right_lower_roller",
    )
    carriage.inertial = Inertial.from_geometry(
        geometry=Box((0.62, 1.07, 1.24)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.535, 0.62)),
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BASE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=LIFT_TRAVEL, effort=22000.0, velocity=0.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("fork_frame")
    lift = object_model.get_articulation("mast_lift")

    left_upright = mast.get_visual("left_upright")
    right_upright = mast.get_visual("right_upright")
    left_upper_roller = carriage.get_visual("left_upper_roller")
    right_upper_roller = carriage.get_visual("right_upper_roller")
    left_lower_roller = carriage.get_visual("left_lower_roller")
    right_lower_roller = carriage.get_visual("right_lower_roller")

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

    lift_kind = getattr(lift.articulation_type, "name", str(lift.articulation_type))
    ctx.check(
        "mast_uses_single_prismatic_lift",
        lift_kind == "PRISMATIC",
        f"Expected a PRISMATIC mast lift, got {lift_kind!r}.",
    )
    ctx.check(
        "mast_lift_axis_is_vertical",
        tuple(round(v, 6) for v in lift.axis) == (0.0, 0.0, 1.0),
        f"Expected vertical lift axis (0, 0, 1), got {lift.axis!r}.",
    )
    ctx.check(
        "mast_lift_limits_are_positive_upstroke",
        lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper is not None
        and lift.motion_limits.upper >= 0.8,
        f"Unexpected lift limits: {lift.motion_limits!r}.",
    )

    ctx.expect_contact(
        carriage,
        mast,
        elem_a=left_upper_roller,
        elem_b=left_upright,
        name="left_upper_guide_contacts_mast",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=right_upper_roller,
        elem_b=right_upright,
        name="right_upper_guide_contacts_mast",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=left_lower_roller,
        elem_b=left_upright,
        name="left_lower_guide_contacts_mast",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=right_lower_roller,
        elem_b=right_upright,
        name="right_lower_guide_contacts_mast",
    )

    rest_z = ctx.part_world_position(carriage)[2]
    with ctx.pose({lift: LIFT_TRAVEL}):
        raised_z = ctx.part_world_position(carriage)[2]
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_raise")
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=left_upper_roller,
            elem_b=left_upright,
            name="left_upper_guide_contacts_mast_at_full_raise",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=right_upper_roller,
            elem_b=right_upright,
            name="right_upper_guide_contacts_mast_at_full_raise",
        )
    ctx.check(
        "fork_frame_moves_upward_with_positive_q",
        raised_z > rest_z + 0.75,
        f"Expected full raise to move the fork frame upward by > 0.75 m, got {raised_z - rest_z:.3f} m.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
