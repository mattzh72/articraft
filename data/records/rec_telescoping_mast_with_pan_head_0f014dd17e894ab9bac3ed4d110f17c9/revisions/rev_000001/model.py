from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BASE_WIDTH = 0.60
BASE_DEPTH = 0.72
FRAME_BEAM = 0.08
FRAME_HEIGHT = 0.10
DECK_SIZE = 0.26
DECK_THICKNESS = 0.02
DECK_TOP_Z = FRAME_HEIGHT + DECK_THICKNESS

OUTER_STAGE_OUTER = 0.18
OUTER_STAGE_WALL = 0.01
OUTER_STAGE_HEIGHT = 0.90
OUTER_STAGE_FLANGE = 0.22
OUTER_STAGE_FLANGE_THICKNESS = 0.02

MIDDLE_STAGE_OUTER = 0.15
MIDDLE_STAGE_WALL = 0.01
MIDDLE_STAGE_HEIGHT = 0.85
MIDDLE_STAGE_BOTTOM_OFFSET = 0.04
MIDDLE_GUIDE_THICKNESS = 0.005
MIDDLE_GUIDE_SPAN = 0.11
MIDDLE_GUIDE_HEIGHT = 0.16
MIDDLE_STAGE_STROKE = 0.44

INNER_STAGE_OUTER = 0.12
INNER_STAGE_WALL = 0.01
INNER_STAGE_HEIGHT = 0.72
INNER_STAGE_BOTTOM_OFFSET = 0.04
INNER_GUIDE_THICKNESS = 0.005
INNER_GUIDE_SPAN = 0.085
INNER_GUIDE_HEIGHT = 0.14
INNER_STAGE_COLLAPSED_OFFSET = 0.18
INNER_STAGE_STROKE = 0.40

TOP_SUPPORT_RADIUS = 0.08
TOP_SUPPORT_THICKNESS = 0.014
HEAD_BASE_THICKNESS = 0.018
HEAD_BODY_HEIGHT = 0.062
HEAD_PLATE_THICKNESS = 0.012
HEAD_STUD_HEIGHT = 0.018


def _add_square_tube(
    part,
    *,
    name_prefix: str,
    outer: float,
    wall: float,
    height: float,
    z0: float,
    material: Material | str,
) -> None:
    half_outer = outer / 2.0
    wall_center = half_outer - wall / 2.0
    center_z = z0 + height / 2.0

    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(wall_center, 0.0, center_z)),
        material=material,
        name=f"{name_prefix}_wall_pos_x",
    )
    part.visual(
        Box((wall, outer, height)),
        origin=Origin(xyz=(-wall_center, 0.0, center_z)),
        material=material,
        name=f"{name_prefix}_wall_neg_x",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, wall_center, center_z)),
        material=material,
        name=f"{name_prefix}_wall_pos_y",
    )
    part.visual(
        Box((outer, wall, height)),
        origin=Origin(xyz=(0.0, -wall_center, center_z)),
        material=material,
        name=f"{name_prefix}_wall_neg_y",
    )


def _add_guide_pads(
    part,
    *,
    name_prefix: str,
    tube_outer: float,
    pad_thickness: float,
    pad_span: float,
    pad_height: float,
    z0: float,
    material: Material | str,
) -> None:
    pad_center = tube_outer / 2.0 + pad_thickness / 2.0
    center_z = z0 + pad_height / 2.0

    part.visual(
        Box((pad_thickness, pad_span, pad_height)),
        origin=Origin(xyz=(pad_center, 0.0, center_z)),
        material=material,
        name=f"{name_prefix}_guide_pos_x",
    )
    part.visual(
        Box((pad_thickness, pad_span, pad_height)),
        origin=Origin(xyz=(-pad_center, 0.0, center_z)),
        material=material,
        name=f"{name_prefix}_guide_neg_x",
    )
    part.visual(
        Box((pad_span, pad_thickness, pad_height)),
        origin=Origin(xyz=(0.0, pad_center, center_z)),
        material=material,
        name=f"{name_prefix}_guide_pos_y",
    )
    part.visual(
        Box((pad_span, pad_thickness, pad_height)),
        origin=Origin(xyz=(0.0, -pad_center, center_z)),
        material=material,
        name=f"{name_prefix}_guide_neg_y",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_inspection_mast")

    frame_steel = model.material("frame_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    mast_steel = model.material("mast_steel", rgba=(0.72, 0.75, 0.78, 1.0))
    wear_pad = model.material("wear_pad", rgba=(0.08, 0.08, 0.09, 1.0))
    head_body = model.material("head_body", rgba=(0.22, 0.22, 0.24, 1.0))
    plate_metal = model.material("plate_metal", rgba=(0.84, 0.86, 0.88, 1.0))

    base_frame = model.part("base_frame")
    runner_x = BASE_WIDTH / 2.0 - FRAME_BEAM / 2.0
    cross_y = BASE_DEPTH / 2.0 - FRAME_BEAM / 2.0

    base_frame.visual(
        Box((FRAME_BEAM, BASE_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(runner_x, 0.0, FRAME_HEIGHT / 2.0)),
        material=frame_steel,
        name="right_runner",
    )
    base_frame.visual(
        Box((FRAME_BEAM, BASE_DEPTH, FRAME_HEIGHT)),
        origin=Origin(xyz=(-runner_x, 0.0, FRAME_HEIGHT / 2.0)),
        material=frame_steel,
        name="left_runner",
    )
    base_frame.visual(
        Box((BASE_WIDTH - 2.0 * FRAME_BEAM, FRAME_BEAM, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, cross_y, FRAME_HEIGHT / 2.0)),
        material=frame_steel,
        name="front_crossmember",
    )
    base_frame.visual(
        Box((BASE_WIDTH - 2.0 * FRAME_BEAM, FRAME_BEAM, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, -cross_y, FRAME_HEIGHT / 2.0)),
        material=frame_steel,
        name="rear_crossmember",
    )
    base_frame.visual(
        Box((BASE_WIDTH - 2.0 * FRAME_BEAM, FRAME_BEAM, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
        material=frame_steel,
        name="center_crossmember_x",
    )
    base_frame.visual(
        Box((FRAME_BEAM, BASE_DEPTH - 2.0 * FRAME_BEAM, FRAME_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
        material=frame_steel,
        name="center_crossmember_y",
    )
    base_frame.visual(
        Box((DECK_SIZE, DECK_SIZE, DECK_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT + DECK_THICKNESS / 2.0)),
        material=plate_metal,
        name="mast_deck",
    )

    outer_stage = model.part("outer_stage")
    outer_stage.visual(
        Box((OUTER_STAGE_FLANGE, OUTER_STAGE_FLANGE, OUTER_STAGE_FLANGE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -OUTER_STAGE_FLANGE_THICKNESS / 2.0)),
        material=plate_metal,
        name="outer_flange",
    )
    _add_square_tube(
        outer_stage,
        name_prefix="outer",
        outer=OUTER_STAGE_OUTER,
        wall=OUTER_STAGE_WALL,
        height=OUTER_STAGE_HEIGHT,
        z0=0.0,
        material=mast_steel,
    )

    middle_stage = model.part("middle_stage")
    _add_square_tube(
        middle_stage,
        name_prefix="middle",
        outer=MIDDLE_STAGE_OUTER,
        wall=MIDDLE_STAGE_WALL,
        height=MIDDLE_STAGE_HEIGHT,
        z0=MIDDLE_STAGE_BOTTOM_OFFSET,
        material=mast_steel,
    )
    _add_guide_pads(
        middle_stage,
        name_prefix="middle",
        tube_outer=MIDDLE_STAGE_OUTER,
        pad_thickness=MIDDLE_GUIDE_THICKNESS,
        pad_span=MIDDLE_GUIDE_SPAN,
        pad_height=MIDDLE_GUIDE_HEIGHT,
        z0=MIDDLE_STAGE_BOTTOM_OFFSET,
        material=wear_pad,
    )

    inner_stage = model.part("inner_stage")
    _add_square_tube(
        inner_stage,
        name_prefix="inner",
        outer=INNER_STAGE_OUTER,
        wall=INNER_STAGE_WALL,
        height=INNER_STAGE_HEIGHT,
        z0=INNER_STAGE_BOTTOM_OFFSET,
        material=mast_steel,
    )
    _add_guide_pads(
        inner_stage,
        name_prefix="inner",
        tube_outer=INNER_STAGE_OUTER,
        pad_thickness=INNER_GUIDE_THICKNESS,
        pad_span=INNER_GUIDE_SPAN,
        pad_height=INNER_GUIDE_HEIGHT,
        z0=INNER_STAGE_BOTTOM_OFFSET,
        material=wear_pad,
    )
    support_z = INNER_STAGE_BOTTOM_OFFSET + INNER_STAGE_HEIGHT + TOP_SUPPORT_THICKNESS / 2.0
    inner_stage.visual(
        Cylinder(radius=TOP_SUPPORT_RADIUS, length=TOP_SUPPORT_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, support_z)),
        material=head_body,
        name="top_support_bearing",
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=TOP_SUPPORT_RADIUS, length=HEAD_BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, HEAD_BASE_THICKNESS / 2.0)),
        material=head_body,
        name="pan_turntable",
    )
    spindle_height = 0.070
    pan_head.visual(
        Cylinder(radius=0.025, length=spindle_height),
        origin=Origin(xyz=(0.0, 0.0, HEAD_BASE_THICKNESS + spindle_height / 2.0)),
        material=head_body,
        name="pan_spindle",
    )
    pan_head.visual(
        Box((0.14, 0.10, HEAD_BODY_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, HEAD_BASE_THICKNESS + HEAD_BODY_HEIGHT / 2.0),
        ),
        material=head_body,
        name="head_housing",
    )
    plate_z = HEAD_BASE_THICKNESS + spindle_height + HEAD_PLATE_THICKNESS / 2.0
    pan_head.visual(
        Box((0.20, 0.14, HEAD_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, plate_z)),
        material=plate_metal,
        name="mounting_plate",
    )
    stud_z = HEAD_BASE_THICKNESS + spindle_height + HEAD_PLATE_THICKNESS + HEAD_STUD_HEIGHT / 2.0
    pan_head.visual(
        Cylinder(radius=0.01, length=HEAD_STUD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, stud_z)),
        material=plate_metal,
        name="mounting_stud",
    )

    model.articulation(
        "base_to_outer_stage",
        ArticulationType.FIXED,
        parent=base_frame,
        child=outer_stage,
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z + OUTER_STAGE_FLANGE_THICKNESS)),
    )
    model.articulation(
        "outer_to_middle_stage",
        ArticulationType.PRISMATIC,
        parent=outer_stage,
        child=middle_stage,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=800.0,
            velocity=0.25,
            lower=0.0,
            upper=MIDDLE_STAGE_STROKE,
        ),
    )
    model.articulation(
        "middle_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(0.0, 0.0, INNER_STAGE_COLLAPSED_OFFSET)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.25,
            lower=0.0,
            upper=INNER_STAGE_STROKE,
        ),
    )
    model.articulation(
        "inner_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=inner_stage,
        child=pan_head,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                INNER_STAGE_BOTTOM_OFFSET + INNER_STAGE_HEIGHT + TOP_SUPPORT_THICKNESS,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    outer_stage = object_model.get_part("outer_stage")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    pan_head = object_model.get_part("pan_head")

    outer_lift = object_model.get_articulation("outer_to_middle_stage")
    inner_lift = object_model.get_articulation("middle_to_inner_stage")
    pan_joint = object_model.get_articulation("inner_to_pan_head")

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
        "single_root_part",
        len(object_model.root_parts()) == 1,
        details=f"expected exactly one root part, got {len(object_model.root_parts())}",
    )
    ctx.check(
        "outer_lift_axis_vertical",
        tuple(outer_lift.axis) == (0.0, 0.0, 1.0),
        details=f"outer lift axis was {outer_lift.axis}",
    )
    ctx.check(
        "inner_lift_axis_vertical",
        tuple(inner_lift.axis) == (0.0, 0.0, 1.0),
        details=f"inner lift axis was {inner_lift.axis}",
    )
    ctx.check(
        "pan_axis_vertical",
        tuple(pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"pan axis was {pan_joint.axis}",
    )

    outer_limits = outer_lift.motion_limits
    inner_limits = inner_lift.motion_limits
    pan_limits = pan_joint.motion_limits
    ctx.check(
        "outer_lift_limits_present",
        outer_limits is not None
        and isclose(outer_limits.lower or 0.0, 0.0)
        and isclose(outer_limits.upper or 0.0, MIDDLE_STAGE_STROKE),
        details=f"outer lift limits were {outer_limits}",
    )
    ctx.check(
        "inner_lift_limits_present",
        inner_limits is not None
        and isclose(inner_limits.lower or 0.0, 0.0)
        and isclose(inner_limits.upper or 0.0, INNER_STAGE_STROKE),
        details=f"inner lift limits were {inner_limits}",
    )
    ctx.check(
        "pan_limits_present",
        pan_limits is not None
        and isclose(pan_limits.lower or 0.0, -pi)
        and isclose(pan_limits.upper or 0.0, pi),
        details=f"pan limits were {pan_limits}",
    )

    ctx.expect_gap(
        outer_stage,
        base_frame,
        axis="z",
        min_gap=0.0,
        max_gap=1e-6,
        max_penetration=0.0,
        name="outer_stage_seated_on_base_deck",
    )
    ctx.expect_contact(
        outer_stage,
        base_frame,
        name="outer_stage_contacts_base_frame",
    )
    ctx.expect_contact(
        middle_stage,
        outer_stage,
        name="middle_stage_guided_in_outer_stage",
    )
    ctx.expect_contact(
        inner_stage,
        middle_stage,
        name="inner_stage_guided_in_middle_stage",
    )
    ctx.expect_contact(
        pan_head,
        inner_stage,
        name="pan_head_supported_on_inner_stage",
    )
    ctx.expect_gap(
        pan_head,
        inner_stage,
        axis="z",
        min_gap=0.0,
        max_gap=1e-6,
        max_penetration=0.0,
        name="pan_head_turntable_seated_on_top_support",
    )
    ctx.expect_overlap(
        middle_stage,
        outer_stage,
        axes="xy",
        min_overlap=MIDDLE_STAGE_OUTER,
        name="middle_stage_nested_inside_outer_stage",
    )
    ctx.expect_overlap(
        inner_stage,
        middle_stage,
        axes="xy",
        min_overlap=INNER_STAGE_OUTER,
        name="inner_stage_nested_inside_middle_stage",
    )

    with ctx.pose({outer_lift: 0.0, inner_lift: 0.0}):
        ctx.expect_origin_gap(
            middle_stage,
            outer_stage,
            axis="z",
            min_gap=0.0,
            max_gap=1e-6,
            name="middle_stage_collapsed_origin",
        )
        ctx.expect_origin_gap(
            inner_stage,
            middle_stage,
            axis="z",
            min_gap=INNER_STAGE_COLLAPSED_OFFSET - 1e-6,
            max_gap=INNER_STAGE_COLLAPSED_OFFSET + 1e-6,
            name="inner_stage_collapsed_origin",
        )

    if outer_limits is not None and inner_limits is not None:
        with ctx.pose({outer_lift: outer_limits.upper, inner_lift: inner_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="mast_fully_extended_no_overlap")
            ctx.fail_if_isolated_parts(name="mast_fully_extended_no_floating")
            ctx.expect_contact(
                middle_stage,
                outer_stage,
                name="middle_stage_contact_at_full_extension",
            )
            ctx.expect_contact(
                inner_stage,
                middle_stage,
                name="inner_stage_contact_at_full_extension",
            )
            ctx.expect_contact(
                pan_head,
                inner_stage,
                name="pan_head_contact_at_full_extension",
            )
            ctx.expect_origin_gap(
                middle_stage,
                outer_stage,
                axis="z",
                min_gap=outer_limits.upper - 1e-6,
                max_gap=outer_limits.upper + 1e-6,
                name="middle_stage_rises_by_full_stroke",
            )
            ctx.expect_origin_gap(
                inner_stage,
                middle_stage,
                axis="z",
                min_gap=INNER_STAGE_COLLAPSED_OFFSET + inner_limits.upper - 1e-6,
                max_gap=INNER_STAGE_COLLAPSED_OFFSET + inner_limits.upper + 1e-6,
                name="inner_stage_rises_by_full_stroke",
            )

    if pan_limits is not None:
        with ctx.pose({pan_joint: pan_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pan_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="pan_lower_pose_no_floating")
            ctx.expect_contact(
                pan_head,
                inner_stage,
                name="pan_lower_pose_support_contact",
            )
        with ctx.pose({pan_joint: pan_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="pan_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="pan_upper_pose_no_floating")
            ctx.expect_contact(
                pan_head,
                inner_stage,
                name="pan_upper_pose_support_contact",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
