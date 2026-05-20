from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BOOM_AXIS = (1.0, 0.0, 0.0)

BASE_STAGE_LENGTH = 2.60
BASE_STAGE_Y = 0.24
BASE_STAGE_Z = 0.18
BASE_STAGE_WALL = 0.012

STAGE_1_LENGTH = 2.20
STAGE_1_Y = 0.20
STAGE_1_Z = 0.14
STAGE_1_WALL = 0.010

STAGE_2_LENGTH = 1.80
STAGE_2_Y = 0.164
STAGE_2_Z = 0.104
STAGE_2_WALL = 0.008

BASE_STAGE_INNER_Y = BASE_STAGE_Y - 2.0 * BASE_STAGE_WALL
BASE_STAGE_INNER_Z = BASE_STAGE_Z - 2.0 * BASE_STAGE_WALL
STAGE_1_INNER_Y = STAGE_1_Y - 2.0 * STAGE_1_WALL
STAGE_1_INNER_Z = STAGE_1_Z - 2.0 * STAGE_1_WALL

STAGE_1_PAD_T = 0.5 * (BASE_STAGE_INNER_Y - STAGE_1_Y)
STAGE_2_PAD_T = 0.5 * (STAGE_1_INNER_Y - STAGE_2_Y)

STAGE_1_RETRACTED_X = 0.35
STAGE_1_TRAVEL = 1.50
STAGE_2_RETRACTED_X = 0.45
STAGE_2_TRAVEL = 0.95

HEAD_BRACKET_PLATE_X = 0.018
HEAD_BRACKET_EAR_X = 0.12

FULL_EXTENSION_TIP_X = (
    STAGE_1_RETRACTED_X
    + STAGE_1_TRAVEL
    + STAGE_2_RETRACTED_X
    + STAGE_2_TRAVEL
    + STAGE_2_LENGTH
)
POSE_TOL = 1e-6


def add_box_section(
    part,
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    material,
) -> None:
    x_center = length / 2.0
    side_z = outer_z - 2.0 * wall

    part.visual(
        Box((length, outer_y, wall)),
        origin=Origin(xyz=(x_center, 0.0, outer_z / 2.0 - wall / 2.0)),
        material=material,
        name="top_wall",
    )
    part.visual(
        Box((length, outer_y, wall)),
        origin=Origin(xyz=(x_center, 0.0, -outer_z / 2.0 + wall / 2.0)),
        material=material,
        name="bottom_wall",
    )
    part.visual(
        Box((length, wall, side_z)),
        origin=Origin(xyz=(x_center, outer_y / 2.0 - wall / 2.0, 0.0)),
        material=material,
        name="left_wall",
    )
    part.visual(
        Box((length, wall, side_z)),
        origin=Origin(xyz=(x_center, -outer_y / 2.0 + wall / 2.0, 0.0)),
        material=material,
        name="right_wall",
    )


def add_rear_guide_pads(
    part,
    *,
    shell_y: float,
    shell_z: float,
    pad_t: float,
    x_start: float,
    pad_length: float,
    side_pad_height: float,
    top_pad_width: float,
    material,
) -> None:
    x_center = x_start + pad_length / 2.0

    part.visual(
        Box((pad_length, top_pad_width, pad_t)),
        origin=Origin(xyz=(x_center, 0.0, shell_z / 2.0 + pad_t / 2.0)),
        material=material,
        name="rear_top_pad",
    )
    part.visual(
        Box((pad_length, top_pad_width, pad_t)),
        origin=Origin(xyz=(x_center, 0.0, -shell_z / 2.0 - pad_t / 2.0)),
        material=material,
        name="rear_bottom_pad",
    )
    part.visual(
        Box((pad_length, pad_t, side_pad_height)),
        origin=Origin(xyz=(x_center, shell_y / 2.0 + pad_t / 2.0, 0.0)),
        material=material,
        name="rear_left_pad",
    )
    part.visual(
        Box((pad_length, pad_t, side_pad_height)),
        origin=Origin(xyz=(x_center, -shell_y / 2.0 - pad_t / 2.0, 0.0)),
        material=material,
        name="rear_right_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rectangular_telescoping_boom")

    mount_gray = model.material("mount_gray", rgba=(0.34, 0.36, 0.39, 1.0))
    steel_gray = model.material("steel_gray", rgba=(0.50, 0.52, 0.56, 1.0))
    boom_yellow = model.material("boom_yellow", rgba=(0.88, 0.72, 0.14, 1.0))
    inner_yellow = model.material("inner_yellow", rgba=(0.94, 0.78, 0.20, 1.0))
    pad_black = model.material("pad_black", rgba=(0.12, 0.12, 0.13, 1.0))

    root_mount = model.part("root_mount")
    root_mount.visual(
        Box((0.95, 0.34, 0.20)),
        origin=Origin(xyz=(0.475, 0.0, -0.31)),
        material=mount_gray,
        name="base_block",
    )
    root_mount.visual(
        Box((0.80, BASE_STAGE_Y, 0.12)),
        origin=Origin(xyz=(0.40, 0.0, -0.15)),
        material=steel_gray,
        name="saddle",
    )
    root_mount.visual(
        Box((0.03, 0.28, 0.24)),
        origin=Origin(xyz=(-0.015, 0.0, -0.03)),
        material=steel_gray,
        name="rear_plate",
    )
    root_mount.inertial = Inertial.from_geometry(
        Box((0.95, 0.34, 0.44)),
        mass=180.0,
        origin=Origin(xyz=(0.475, 0.0, -0.22)),
    )

    base_stage = model.part("base_stage")
    add_box_section(
        base_stage,
        length=BASE_STAGE_LENGTH,
        outer_y=BASE_STAGE_Y,
        outer_z=BASE_STAGE_Z,
        wall=BASE_STAGE_WALL,
        material=boom_yellow,
    )
    base_stage.inertial = Inertial.from_geometry(
        Box((BASE_STAGE_LENGTH, BASE_STAGE_Y, BASE_STAGE_Z)),
        mass=135.0,
        origin=Origin(xyz=(BASE_STAGE_LENGTH / 2.0, 0.0, 0.0)),
    )

    stage_1 = model.part("stage_1")
    add_box_section(
        stage_1,
        length=STAGE_1_LENGTH,
        outer_y=STAGE_1_Y,
        outer_z=STAGE_1_Z,
        wall=STAGE_1_WALL,
        material=inner_yellow,
    )
    add_rear_guide_pads(
        stage_1,
        shell_y=STAGE_1_Y,
        shell_z=STAGE_1_Z,
        pad_t=STAGE_1_PAD_T,
        x_start=0.24,
        pad_length=0.34,
        side_pad_height=0.060,
        top_pad_width=0.100,
        material=pad_black,
    )
    stage_1.inertial = Inertial.from_geometry(
        Box((STAGE_1_LENGTH, BASE_STAGE_INNER_Y, BASE_STAGE_INNER_Z)),
        mass=92.0,
        origin=Origin(xyz=(STAGE_1_LENGTH / 2.0, 0.0, 0.0)),
    )

    stage_2 = model.part("stage_2")
    add_box_section(
        stage_2,
        length=STAGE_2_LENGTH,
        outer_y=STAGE_2_Y,
        outer_z=STAGE_2_Z,
        wall=STAGE_2_WALL,
        material=boom_yellow,
    )
    add_rear_guide_pads(
        stage_2,
        shell_y=STAGE_2_Y,
        shell_z=STAGE_2_Z,
        pad_t=STAGE_2_PAD_T,
        x_start=0.22,
        pad_length=0.28,
        side_pad_height=0.050,
        top_pad_width=0.080,
        material=pad_black,
    )
    stage_2.inertial = Inertial.from_geometry(
        Box((STAGE_2_LENGTH, STAGE_1_INNER_Y, STAGE_1_INNER_Z)),
        mass=58.0,
        origin=Origin(xyz=(STAGE_2_LENGTH / 2.0, 0.0, 0.0)),
    )

    head_bracket = model.part("head_bracket")
    head_bracket.visual(
        Box((HEAD_BRACKET_PLATE_X, 0.12, 0.10)),
        origin=Origin(xyz=(HEAD_BRACKET_PLATE_X / 2.0, 0.0, 0.0)),
        material=steel_gray,
        name="back_plate",
    )
    head_bracket.visual(
        Box((HEAD_BRACKET_EAR_X, 0.024, 0.080)),
        origin=Origin(xyz=(HEAD_BRACKET_PLATE_X + HEAD_BRACKET_EAR_X / 2.0, 0.048, 0.0)),
        material=steel_gray,
        name="left_ear",
    )
    head_bracket.visual(
        Box((HEAD_BRACKET_EAR_X, 0.024, 0.080)),
        origin=Origin(xyz=(HEAD_BRACKET_PLATE_X + HEAD_BRACKET_EAR_X / 2.0, -0.048, 0.0)),
        material=steel_gray,
        name="right_ear",
    )
    head_bracket.inertial = Inertial.from_geometry(
        Box((0.138, 0.12, 0.10)),
        mass=14.0,
        origin=Origin(xyz=(0.069, 0.0, 0.0)),
    )

    model.articulation(
        "mount_to_base_stage",
        ArticulationType.FIXED,
        parent=root_mount,
        child=base_stage,
        origin=Origin(),
    )
    model.articulation(
        "base_stage_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base_stage,
        child=stage_1,
        origin=Origin(xyz=(STAGE_1_RETRACTED_X, 0.0, 0.0)),
        axis=BOOM_AXIS,
        motion_limits=MotionLimits(
            effort=12000.0,
            velocity=0.55,
            lower=0.0,
            upper=STAGE_1_TRAVEL,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE_2_RETRACTED_X, 0.0, 0.0)),
        axis=BOOM_AXIS,
        motion_limits=MotionLimits(
            effort=9000.0,
            velocity=0.60,
            lower=0.0,
            upper=STAGE_2_TRAVEL,
        ),
    )
    model.articulation(
        "stage_2_to_head_bracket",
        ArticulationType.FIXED,
        parent=stage_2,
        child=head_bracket,
        origin=Origin(xyz=(STAGE_2_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root_mount = object_model.get_part("root_mount")
    base_stage = object_model.get_part("base_stage")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    head_bracket = object_model.get_part("head_bracket")

    saddle = root_mount.get_visual("saddle")
    rear_plate = root_mount.get_visual("rear_plate")
    base_bottom_wall = base_stage.get_visual("bottom_wall")
    base_left_wall = base_stage.get_visual("left_wall")
    base_top_wall = base_stage.get_visual("top_wall")
    stage_1_top_wall = stage_1.get_visual("top_wall")
    stage_1_left_wall = stage_1.get_visual("left_wall")
    stage_1_top_pad = stage_1.get_visual("rear_top_pad")
    stage_1_left_pad = stage_1.get_visual("rear_left_pad")
    stage_2_top_wall = stage_2.get_visual("top_wall")
    stage_2_left_wall = stage_2.get_visual("left_wall")
    stage_2_top_pad = stage_2.get_visual("rear_top_pad")
    stage_2_left_pad = stage_2.get_visual("rear_left_pad")
    bracket_back_plate = head_bracket.get_visual("back_plate")

    base_stage_to_stage_1 = object_model.get_articulation("base_stage_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=False,
        ignore_fixed=False,
    )

    ctx.check(
        "single_root_mount",
        sorted(part.name for part in object_model.root_parts()) == ["root_mount"],
        details="The boom should have exactly one grounded root part: root_mount.",
    )
    ctx.check(
        "prismatic_stage_axes",
        base_stage_to_stage_1.axis == BOOM_AXIS and stage_1_to_stage_2.axis == BOOM_AXIS,
        details="Both telescoping stages should slide along the boom x-axis.",
    )

    ctx.expect_contact(
        base_stage,
        root_mount,
        elem_a=base_bottom_wall,
        elem_b=saddle,
        name="base_stage_supported_by_saddle",
    )
    ctx.expect_contact(
        base_stage,
        root_mount,
        elem_a=base_top_wall,
        elem_b=rear_plate,
        name="base_stage_against_root_rear_plate",
    )
    ctx.expect_contact(
        stage_1,
        base_stage,
        elem_a=stage_1_top_pad,
        elem_b=base_top_wall,
        name="stage_1_top_pad_bears_on_base",
    )
    ctx.expect_contact(
        stage_1,
        base_stage,
        elem_a=stage_1_left_pad,
        elem_b=base_left_wall,
        name="stage_1_side_pad_bears_on_base",
    )
    ctx.expect_contact(
        stage_2,
        stage_1,
        elem_a=stage_2_top_pad,
        elem_b=stage_1_top_wall,
        name="stage_2_top_pad_bears_on_stage_1",
    )
    ctx.expect_contact(
        stage_2,
        stage_1,
        elem_a=stage_2_left_pad,
        elem_b=stage_1_left_wall,
        name="stage_2_side_pad_bears_on_stage_1",
    )
    ctx.expect_contact(
        head_bracket,
        stage_2,
        elem_a=bracket_back_plate,
        elem_b=stage_2_top_wall,
        name="head_bracket_attached_to_stage_2_tip",
    )

    ctx.expect_within(
        stage_1,
        base_stage,
        axes="yz",
        margin=0.0,
        name="stage_1_nested_within_base_section",
    )
    ctx.expect_within(
        stage_2,
        stage_1,
        axes="yz",
        margin=0.0,
        name="stage_2_nested_within_stage_1_section",
    )

    ctx.expect_origin_gap(
        stage_1,
        base_stage,
        axis="x",
        min_gap=STAGE_1_RETRACTED_X - POSE_TOL,
        max_gap=STAGE_1_RETRACTED_X + POSE_TOL,
        name="stage_1_retracted_origin_position",
    )
    ctx.expect_origin_gap(
        stage_2,
        stage_1,
        axis="x",
        min_gap=STAGE_2_RETRACTED_X - POSE_TOL,
        max_gap=STAGE_2_RETRACTED_X + POSE_TOL,
        name="stage_2_retracted_origin_position",
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

    for joint_name in ("base_stage_to_stage_1", "stage_1_to_stage_2"):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        if limits is not None and limits.lower is not None and limits.upper is not None:
            with ctx.pose({joint: limits.lower}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_lower_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_lower_no_floating")
            with ctx.pose({joint: limits.upper}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint_name}_upper_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{joint_name}_upper_no_floating")

    with ctx.pose({base_stage_to_stage_1: STAGE_1_TRAVEL}):
        ctx.expect_contact(
            stage_1,
            base_stage,
            elem_a=stage_1_top_pad,
            elem_b=base_top_wall,
            name="stage_1_extended_top_pad_contact",
        )
        ctx.expect_contact(
            stage_1,
            base_stage,
            elem_a=stage_1_left_pad,
            elem_b=base_left_wall,
            name="stage_1_extended_side_pad_contact",
        )
        ctx.expect_within(
            stage_1,
            base_stage,
            axes="yz",
            margin=0.0,
            name="stage_1_extended_stays_centered",
        )
        ctx.expect_origin_gap(
            stage_1,
            base_stage,
            axis="x",
            min_gap=STAGE_1_RETRACTED_X + STAGE_1_TRAVEL - POSE_TOL,
            max_gap=STAGE_1_RETRACTED_X + STAGE_1_TRAVEL + POSE_TOL,
            name="stage_1_extended_origin_position",
        )

    with ctx.pose({stage_1_to_stage_2: STAGE_2_TRAVEL}):
        ctx.expect_contact(
            stage_2,
            stage_1,
            elem_a=stage_2_top_pad,
            elem_b=stage_1_top_wall,
            name="stage_2_extended_top_pad_contact",
        )
        ctx.expect_contact(
            stage_2,
            stage_1,
            elem_a=stage_2_left_pad,
            elem_b=stage_1_left_wall,
            name="stage_2_extended_side_pad_contact",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.0,
            name="stage_2_extended_stays_centered",
        )
        ctx.expect_origin_gap(
            stage_2,
            stage_1,
            axis="x",
            min_gap=STAGE_2_RETRACTED_X + STAGE_2_TRAVEL - POSE_TOL,
            max_gap=STAGE_2_RETRACTED_X + STAGE_2_TRAVEL + POSE_TOL,
            name="stage_2_extended_origin_position",
        )

    with ctx.pose(
        {
            base_stage_to_stage_1: STAGE_1_TRAVEL,
            stage_1_to_stage_2: STAGE_2_TRAVEL,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="fully_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="fully_extended_no_floating")
        ctx.expect_origin_gap(
            head_bracket,
            base_stage,
            axis="x",
            min_gap=FULL_EXTENSION_TIP_X - POSE_TOL,
            max_gap=FULL_EXTENSION_TIP_X + POSE_TOL,
            name="head_bracket_reaches_full_extension_tip_position",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
