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


SUPPORT_PLATE_THICKNESS = 0.020
SUPPORT_PLATE_WIDTH = 0.180
SUPPORT_PLATE_HEIGHT = 0.300

BASE_SLEEVE_LENGTH = 0.420
BASE_SLEEVE_SIZE = 0.120
BASE_SLEEVE_WALL = 0.004
BASE_INNER_SIZE = BASE_SLEEVE_SIZE - (2.0 * BASE_SLEEVE_WALL)

STAGE_1_LENGTH = 0.220
STAGE_1_SIZE = 0.108
STAGE_1_WALL = 0.004
STAGE_1_INNER_SIZE = STAGE_1_SIZE - (2.0 * STAGE_1_WALL)
STAGE_1_GUIDE_LENGTH = 0.270
STAGE_1_GUIDE_CORE = 0.070
STAGE_1_JOINT_X = BASE_SLEEVE_LENGTH
STAGE_1_TRAVEL = 0.220

STAGE_2_LENGTH = 0.180
STAGE_2_SIZE = 0.096
STAGE_2_WALL = 0.0035
STAGE_2_INNER_SIZE = STAGE_2_SIZE - (2.0 * STAGE_2_WALL)
STAGE_2_GUIDE_LENGTH = 0.220
STAGE_2_GUIDE_CORE = 0.062
STAGE_2_JOINT_X = STAGE_1_LENGTH
STAGE_2_TRAVEL = 0.180

STAGE_3_LENGTH = 0.140
STAGE_3_SIZE = 0.084
STAGE_3_WALL = 0.0035
STAGE_3_GUIDE_LENGTH = 0.180
STAGE_3_GUIDE_CORE = 0.054
STAGE_3_JOINT_X = STAGE_2_LENGTH
STAGE_3_TRAVEL = 0.140

FACEPLATE_THICKNESS = 0.008
FACEPLATE_SIZE = 0.110
FACEPLATE_BOSS_SIZE = 0.050
FACEPLATE_BOSS_DEPTH = 0.012

GUIDE_PAD_SPAN_RATIO = 0.72
CAPTURE_PAD_LENGTH = 0.050


def _add_box(part, name: str, size: tuple[float, float, float], center: tuple[float, float, float], material: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_box_tube(
    part,
    *,
    prefix: str,
    length: float,
    outer_size: float,
    wall: float,
    material: str,
    start_x: float = 0.0,
) -> None:
    x_center = start_x + (length / 2.0)
    side_height = outer_size - (2.0 * wall)
    _add_box(
        part,
        f"{prefix}_top",
        (length, outer_size, wall),
        (x_center, 0.0, (outer_size / 2.0) - (wall / 2.0)),
        material,
    )
    _add_box(
        part,
        f"{prefix}_bottom",
        (length, outer_size, wall),
        (x_center, 0.0, -(outer_size / 2.0) + (wall / 2.0)),
        material,
    )
    _add_box(
        part,
        f"{prefix}_left",
        (length, wall, side_height),
        (x_center, -(outer_size / 2.0) + (wall / 2.0), 0.0),
        material,
    )
    _add_box(
        part,
        f"{prefix}_right",
        (length, wall, side_height),
        (x_center, (outer_size / 2.0) - (wall / 2.0), 0.0),
        material,
    )


def _add_capture_guide(
    part,
    *,
    prefix: str,
    guide_length: float,
    core_size: float,
    parent_inner_size: float,
    material: str,
) -> None:
    _add_box(
        part,
        f"{prefix}_core",
        (guide_length, core_size, core_size),
        (-(guide_length / 2.0), 0.0, 0.0),
        material,
    )

    pad_thickness = max(parent_inner_size - core_size, 0.0) / 2.0
    if pad_thickness <= 0.0:
        return

    pad_center_x = -guide_length + (CAPTURE_PAD_LENGTH / 2.0)
    side_span = core_size * GUIDE_PAD_SPAN_RATIO

    _add_box(
        part,
        f"{prefix}_pad_top",
        (CAPTURE_PAD_LENGTH, side_span, pad_thickness),
        (pad_center_x, 0.0, (core_size / 2.0) + (pad_thickness / 2.0)),
        material,
    )
    _add_box(
        part,
        f"{prefix}_pad_bottom",
        (CAPTURE_PAD_LENGTH, side_span, pad_thickness),
        (pad_center_x, 0.0, -(core_size / 2.0) - (pad_thickness / 2.0)),
        material,
    )
    _add_box(
        part,
        f"{prefix}_pad_left",
        (CAPTURE_PAD_LENGTH, pad_thickness, side_span),
        (pad_center_x, -(core_size / 2.0) - (pad_thickness / 2.0), 0.0),
        material,
    )
    _add_box(
        part,
        f"{prefix}_pad_right",
        (CAPTURE_PAD_LENGTH, pad_thickness, side_span),
        (pad_center_x, (core_size / 2.0) + (pad_thickness / 2.0), 0.0),
        material,
    )


def _build_support(part) -> None:
    _add_box(
        part,
        "mount_plate",
        (SUPPORT_PLATE_THICKNESS, SUPPORT_PLATE_WIDTH, SUPPORT_PLATE_HEIGHT),
        (-(SUPPORT_PLATE_THICKNESS / 2.0), 0.0, 0.0),
        "support_steel",
    )
    _add_box_tube(
        part,
        prefix="base_sleeve",
        length=BASE_SLEEVE_LENGTH,
        outer_size=BASE_SLEEVE_SIZE,
        wall=BASE_SLEEVE_WALL,
        material="support_steel",
    )
    _add_box(part, "left_cheek", (0.090, 0.016, 0.160), (0.045, -0.068, 0.0), "support_steel")
    _add_box(part, "right_cheek", (0.090, 0.016, 0.160), (0.045, 0.068, 0.0), "support_steel")
    _add_box(part, "lower_saddle", (0.130, 0.170, 0.028), (0.065, 0.0, -0.074), "support_steel")
    _add_box(part, "upper_rib", (0.090, 0.140, 0.016), (0.045, 0.0, 0.074), "support_steel")


def _build_stage(
    part,
    *,
    prefix: str,
    exposed_length: float,
    outer_size: float,
    wall: float,
    guide_length: float,
    guide_core: float,
    parent_inner_size: float,
    material: str,
) -> None:
    _add_box_tube(
        part,
        prefix=f"{prefix}_tube",
        length=exposed_length,
        outer_size=outer_size,
        wall=wall,
        material=material,
    )
    _add_capture_guide(
        part,
        prefix=f"{prefix}_guide",
        guide_length=guide_length,
        core_size=guide_core,
        parent_inner_size=parent_inner_size,
        material=material,
    )
    shell_inner_size = outer_size - (2.0 * wall)
    bridge_gap = max(shell_inner_size - guide_core, 0.0) / 2.0
    if bridge_gap <= 0.0:
        return

    bridge_length = 0.010
    bridge_center_x = -(bridge_length / 2.0)
    bridge_span = guide_core * GUIDE_PAD_SPAN_RATIO

    _add_box(
        part,
        f"{prefix}_bridge_top",
        (bridge_length, bridge_span, bridge_gap),
        (bridge_center_x, 0.0, (guide_core / 2.0) + (bridge_gap / 2.0)),
        material,
    )
    _add_box(
        part,
        f"{prefix}_bridge_bottom",
        (bridge_length, bridge_span, bridge_gap),
        (bridge_center_x, 0.0, -(guide_core / 2.0) - (bridge_gap / 2.0)),
        material,
    )
    _add_box(
        part,
        f"{prefix}_bridge_left",
        (bridge_length, bridge_gap, bridge_span),
        (bridge_center_x, -(guide_core / 2.0) - (bridge_gap / 2.0), 0.0),
        material,
    )
    _add_box(
        part,
        f"{prefix}_bridge_right",
        (bridge_length, bridge_gap, bridge_span),
        (bridge_center_x, (guide_core / 2.0) + (bridge_gap / 2.0), 0.0),
        material,
    )


def _build_faceplate(part) -> None:
    _add_box(
        part,
        "faceplate_panel",
        (FACEPLATE_THICKNESS, FACEPLATE_SIZE, FACEPLATE_SIZE),
        (FACEPLATE_THICKNESS / 2.0, 0.0, 0.0),
        "plate_aluminum",
    )
    _add_box(
        part,
        "faceplate_boss",
        (FACEPLATE_BOSS_DEPTH, FACEPLATE_BOSS_SIZE, FACEPLATE_BOSS_SIZE),
        (FACEPLATE_THICKNESS + (FACEPLATE_BOSS_DEPTH / 2.0), 0.0, 0.0),
        "plate_aluminum",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_telescoping_mast_arm")

    model.material("support_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("stage_outer", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("stage_mid", rgba=(0.71, 0.73, 0.76, 1.0))
    model.material("stage_tip", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("plate_aluminum", rgba=(0.84, 0.85, 0.87, 1.0))

    base_support = model.part("base_support")
    _build_support(base_support)

    stage_1 = model.part("stage_1")
    _build_stage(
        stage_1,
        prefix="stage_1",
        exposed_length=STAGE_1_LENGTH,
        outer_size=STAGE_1_SIZE,
        wall=STAGE_1_WALL,
        guide_length=STAGE_1_GUIDE_LENGTH,
        guide_core=STAGE_1_GUIDE_CORE,
        parent_inner_size=BASE_INNER_SIZE,
        material="stage_outer",
    )

    stage_2 = model.part("stage_2")
    _build_stage(
        stage_2,
        prefix="stage_2",
        exposed_length=STAGE_2_LENGTH,
        outer_size=STAGE_2_SIZE,
        wall=STAGE_2_WALL,
        guide_length=STAGE_2_GUIDE_LENGTH,
        guide_core=STAGE_2_GUIDE_CORE,
        parent_inner_size=STAGE_1_INNER_SIZE,
        material="stage_mid",
    )

    stage_3 = model.part("stage_3")
    _build_stage(
        stage_3,
        prefix="stage_3",
        exposed_length=STAGE_3_LENGTH,
        outer_size=STAGE_3_SIZE,
        wall=STAGE_3_WALL,
        guide_length=STAGE_3_GUIDE_LENGTH,
        guide_core=STAGE_3_GUIDE_CORE,
        parent_inner_size=STAGE_2_INNER_SIZE,
        material="stage_tip",
    )

    faceplate = model.part("faceplate")
    _build_faceplate(faceplate)

    model.articulation(
        "support_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base_support,
        child=stage_1,
        origin=Origin(xyz=(STAGE_1_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=300.0, velocity=0.35, lower=0.0, upper=STAGE_1_TRAVEL),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE_2_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=0.40, lower=0.0, upper=STAGE_2_TRAVEL),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(STAGE_3_JOINT_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.45, lower=0.0, upper=STAGE_3_TRAVEL),
    )
    model.articulation(
        "stage_3_to_faceplate",
        ArticulationType.FIXED,
        parent=stage_3,
        child=faceplate,
        origin=Origin(xyz=(STAGE_3_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")
    faceplate = object_model.get_part("faceplate")

    support_to_stage_1 = object_model.get_articulation("support_to_stage_1")
    stage_1_to_stage_2 = object_model.get_articulation("stage_1_to_stage_2")
    stage_2_to_stage_3 = object_model.get_articulation("stage_2_to_stage_3")

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

    prismatic_joints = (support_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3)
    same_axis = all(tuple(joint.axis) == (1.0, 0.0, 0.0) for joint in prismatic_joints)
    positive_travel = all(
        joint.motion_limits is not None
        and joint.motion_limits.lower == 0.0
        and joint.motion_limits.upper is not None
        and joint.motion_limits.upper > 0.0
        for joint in prismatic_joints
    )
    ctx.check(
        "all_telescoping_stages_share_longitudinal_prismatic_axis",
        same_axis,
        details="Expected all prismatic joints to use +X as the common telescoping axis.",
    )
    ctx.check(
        "all_telescoping_stages_extend_forward_from_zero",
        positive_travel,
        details="Expected each prismatic stage to start at 0 travel and extend in the positive axis direction.",
    )

    ctx.expect_contact(base_support, stage_1, name="stage_1_is_supported_inside_base_sleeve")
    ctx.expect_contact(stage_1, stage_2, name="stage_2_is_supported_inside_stage_1")
    ctx.expect_contact(stage_2, stage_3, name="stage_3_is_supported_inside_stage_2")
    ctx.expect_contact(stage_3, faceplate, name="faceplate_is_mounted_to_tip_stage")

    ctx.expect_within(stage_1, base_support, axes="yz", margin=0.001, name="stage_1_stays_centered_in_base")
    ctx.expect_within(stage_2, stage_1, axes="yz", margin=0.001, name="stage_2_stays_centered_in_stage_1")
    ctx.expect_within(stage_3, stage_2, axes="yz", margin=0.001, name="stage_3_stays_centered_in_stage_2")

    with ctx.pose(
        {
            support_to_stage_1: STAGE_1_TRAVEL,
            stage_1_to_stage_2: STAGE_2_TRAVEL,
            stage_2_to_stage_3: STAGE_3_TRAVEL,
        }
    ):
        ctx.expect_contact(base_support, stage_1, name="stage_1_remains_guided_when_extended")
        ctx.expect_contact(stage_1, stage_2, name="stage_2_remains_guided_when_extended")
        ctx.expect_contact(stage_2, stage_3, name="stage_3_remains_guided_when_extended")
        ctx.expect_gap(
            faceplate,
            base_support,
            axis="x",
            min_gap=0.55,
            name="faceplate_projects_far_beyond_support_when_deployed",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_part_overlap_when_fully_extended")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
