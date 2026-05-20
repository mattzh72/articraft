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


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.18
PLATE_HEIGHT = 0.24

OUTER_LENGTH = 0.34
OUTER_WIDTH = 0.11
OUTER_HEIGHT = 0.084
OUTER_WALL = 0.004

STAGE_1_LENGTH = 0.30
STAGE_1_WIDTH = 0.099
STAGE_1_HEIGHT = 0.073
STAGE_1_WALL = 0.0035

STAGE_2_LENGTH = 0.26
STAGE_2_WIDTH = 0.088
STAGE_2_HEIGHT = 0.062
STAGE_2_WALL = 0.0032

STAGE_3_LENGTH = 0.22
STAGE_3_WIDTH = 0.074
STAGE_3_HEIGHT = 0.048

STAGE_1_HOME = 0.03
STAGE_2_HOME = 0.03
STAGE_3_HOME = 0.03

STAGE_1_TRAVEL = 0.17
STAGE_2_TRAVEL = 0.14
STAGE_3_TRAVEL = 0.11

STAGE_1_REMAINING_OVERLAP = OUTER_LENGTH - (STAGE_1_HOME + STAGE_1_TRAVEL)
STAGE_2_REMAINING_OVERLAP = STAGE_1_LENGTH - (STAGE_2_HOME + STAGE_2_TRAVEL)
STAGE_3_REMAINING_OVERLAP = STAGE_2_LENGTH - (STAGE_3_HOME + STAGE_3_TRAVEL)

OUTER_INNER_WIDTH = OUTER_WIDTH - 2.0 * OUTER_WALL
OUTER_INNER_HEIGHT = OUTER_HEIGHT - 2.0 * OUTER_WALL
STAGE_1_INNER_WIDTH = STAGE_1_WIDTH - 2.0 * STAGE_1_WALL
STAGE_1_INNER_HEIGHT = STAGE_1_HEIGHT - 2.0 * STAGE_1_WALL
STAGE_2_INNER_WIDTH = STAGE_2_WIDTH - 2.0 * STAGE_2_WALL
STAGE_2_INNER_HEIGHT = STAGE_2_HEIGHT - 2.0 * STAGE_2_WALL

GUIDE_OVERLAP = 0.0003

STAGE_1_SIDE_GUIDE = (OUTER_INNER_WIDTH - STAGE_1_WIDTH) / 2.0
STAGE_1_TOP_GUIDE = (OUTER_INNER_HEIGHT - STAGE_1_HEIGHT) / 2.0
STAGE_2_SIDE_GUIDE = (STAGE_1_INNER_WIDTH - STAGE_2_WIDTH) / 2.0
STAGE_2_TOP_GUIDE = (STAGE_1_INNER_HEIGHT - STAGE_2_HEIGHT) / 2.0
STAGE_3_SIDE_GUIDE = (STAGE_2_INNER_WIDTH - STAGE_3_WIDTH) / 2.0
STAGE_3_TOP_GUIDE = (STAGE_2_INNER_HEIGHT - STAGE_3_HEIGHT) / 2.0


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _add_rect_sleeve_visuals(
    part,
    *,
    prefix: str,
    length: float,
    width: float,
    height: float,
    wall: float,
    material: str,
) -> None:
    _add_box_visual(
        part,
        name=f"{prefix}_top",
        size=(length, width, wall),
        center=(length / 2.0, 0.0, height / 2.0 - wall / 2.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_bottom",
        size=(length, width, wall),
        center=(length / 2.0, 0.0, -(height / 2.0 - wall / 2.0)),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_left",
        size=(length, wall, height),
        center=(length / 2.0, width / 2.0 - wall / 2.0, 0.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_right",
        size=(length, wall, height),
        center=(length / 2.0, -(width / 2.0 - wall / 2.0), 0.0),
        material=material,
    )


def _add_guide_visuals(
    part,
    *,
    prefix: str,
    length: float,
    base_width: float,
    base_height: float,
    side_guide: float,
    top_guide: float,
    material: str,
) -> None:
    if side_guide > 0.0:
        side_size = (length, side_guide + GUIDE_OVERLAP, 0.45 * base_height)
        side_center_y = base_width / 2.0 + side_guide / 2.0 - GUIDE_OVERLAP / 2.0
        _add_box_visual(
            part,
            name=f"{prefix}_guide_left",
            size=side_size,
            center=(length / 2.0, side_center_y, 0.0),
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_guide_right",
            size=side_size,
            center=(length / 2.0, -side_center_y, 0.0),
            material=material,
        )
    if top_guide > 0.0:
        top_size = (length, 0.45 * base_width, top_guide + GUIDE_OVERLAP)
        top_center_z = base_height / 2.0 + top_guide / 2.0 - GUIDE_OVERLAP / 2.0
        _add_box_visual(
            part,
            name=f"{prefix}_guide_top",
            size=top_size,
            center=(length / 2.0, 0.0, top_center_z),
            material=material,
        )
        _add_box_visual(
            part,
            name=f"{prefix}_guide_bottom",
            size=top_size,
            center=(length / 2.0, 0.0, -top_center_z),
            material=material,
        )


def _build_rear_mount(part) -> None:
    _add_box_visual(
        part,
        name="rear_mount_plate",
        size=(PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT),
        center=(-PLATE_THICKNESS / 2.0, 0.0, 0.0),
        material="mount_graphite",
    )
    _add_rect_sleeve_visuals(
        part,
        prefix="outer_sleeve",
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material="mount_graphite",
    )


def _build_stage_1(part) -> None:
    _add_rect_sleeve_visuals(
        part,
        prefix="stage_1_frame",
        length=STAGE_1_LENGTH,
        width=STAGE_1_WIDTH,
        height=STAGE_1_HEIGHT,
        wall=STAGE_1_WALL,
        material="stage_dark",
    )
    _add_guide_visuals(
        part,
        prefix="stage_1",
        length=STAGE_1_LENGTH,
        base_width=STAGE_1_WIDTH,
        base_height=STAGE_1_HEIGHT,
        side_guide=STAGE_1_SIDE_GUIDE,
        top_guide=STAGE_1_TOP_GUIDE,
        material="stage_dark",
    )


def _build_stage_2(part) -> None:
    _add_rect_sleeve_visuals(
        part,
        prefix="stage_2_frame",
        length=STAGE_2_LENGTH,
        width=STAGE_2_WIDTH,
        height=STAGE_2_HEIGHT,
        wall=STAGE_2_WALL,
        material="stage_mid",
    )
    _add_guide_visuals(
        part,
        prefix="stage_2",
        length=STAGE_2_LENGTH,
        base_width=STAGE_2_WIDTH,
        base_height=STAGE_2_HEIGHT,
        side_guide=STAGE_2_SIDE_GUIDE,
        top_guide=STAGE_2_TOP_GUIDE,
        material="stage_mid",
    )


def _build_stage_3(part) -> None:
    _add_box_visual(
        part,
        name="stage_3_core",
        size=(STAGE_3_LENGTH, STAGE_3_WIDTH, STAGE_3_HEIGHT),
        center=(STAGE_3_LENGTH / 2.0, 0.0, 0.0),
        material="stage_light",
    )
    _add_guide_visuals(
        part,
        prefix="stage_3",
        length=STAGE_3_LENGTH,
        base_width=STAGE_3_WIDTH,
        base_height=STAGE_3_HEIGHT,
        side_guide=STAGE_3_SIDE_GUIDE,
        top_guide=STAGE_3_TOP_GUIDE,
        material="stage_light",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_telescoping_reach_module")

    model.material("mount_graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("stage_dark", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("stage_mid", rgba=(0.63, 0.66, 0.69, 1.0))
    model.material("stage_light", rgba=(0.77, 0.79, 0.82, 1.0))

    rear_mount = model.part("rear_mount")
    _build_rear_mount(rear_mount)

    stage_1 = model.part("stage_1")
    _build_stage_1(stage_1)

    stage_2 = model.part("stage_2")
    _build_stage_2(stage_2)

    stage_3 = model.part("stage_3")
    _build_stage_3(stage_3)

    model.articulation(
        "rear_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=rear_mount,
        child=stage_1,
        origin=Origin(xyz=(STAGE_1_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE_1_TRAVEL,
        ),
    )
    model.articulation(
        "stage_1_to_stage_2",
        ArticulationType.PRISMATIC,
        parent=stage_1,
        child=stage_2,
        origin=Origin(xyz=(STAGE_2_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=110.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE_2_TRAVEL,
        ),
    )
    model.articulation(
        "stage_2_to_stage_3",
        ArticulationType.PRISMATIC,
        parent=stage_2,
        child=stage_3,
        origin=Origin(xyz=(STAGE_3_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=STAGE_3_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_mount = object_model.get_part("rear_mount")
    stage_1 = object_model.get_part("stage_1")
    stage_2 = object_model.get_part("stage_2")
    stage_3 = object_model.get_part("stage_3")

    rear_to_stage_1 = object_model.get_articulation("rear_to_stage_1")
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

    articulated_joints = (rear_to_stage_1, stage_1_to_stage_2, stage_2_to_stage_3)
    ctx.check(
        "shared_prismatic_axes",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            and tuple(joint.axis) == (1.0, 0.0, 0.0)
            and joint.motion_limits is not None
            and joint.motion_limits.lower == 0.0
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > 0.0
            for joint in articulated_joints
        ),
        details="Each telescoping section should slide forward on the shared +X axis with positive travel.",
    )

    with ctx.pose(
        {
            rear_to_stage_1: 0.0,
            stage_1_to_stage_2: 0.0,
            stage_2_to_stage_3: 0.0,
        }
    ):
        ctx.expect_origin_distance(
            stage_1,
            rear_mount,
            axes="yz",
            max_dist=1e-6,
            name="stage_1_coaxial_with_outer_sleeve",
        )
        ctx.expect_origin_distance(
            stage_2,
            stage_1,
            axes="yz",
            max_dist=1e-6,
            name="stage_2_coaxial_with_stage_1",
        )
        ctx.expect_origin_distance(
            stage_3,
            stage_2,
            axes="yz",
            max_dist=1e-6,
            name="stage_3_coaxial_with_stage_2",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.0,
            name="stage_2_nested_within_stage_1_profile",
        )
        ctx.expect_within(
            stage_3,
            stage_2,
            axes="yz",
            margin=0.0,
            name="stage_3_nested_within_stage_2_profile",
        )
        ctx.expect_overlap(
            rear_mount,
            stage_1,
            axes="x",
            min_overlap=0.28,
            name="stage_1_closed_overlap_with_outer_sleeve",
        )
        ctx.expect_overlap(
            stage_1,
            stage_2,
            axes="x",
            min_overlap=0.22,
            name="stage_2_closed_overlap_with_stage_1",
        )
        ctx.expect_overlap(
            stage_2,
            stage_3,
            axes="x",
            min_overlap=0.18,
            name="stage_3_closed_overlap_with_stage_2",
        )

    with ctx.pose(
        {
            rear_to_stage_1: STAGE_1_TRAVEL,
            stage_1_to_stage_2: STAGE_2_TRAVEL,
            stage_2_to_stage_3: STAGE_3_TRAVEL,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_extension")
        ctx.expect_origin_gap(
            stage_1,
            rear_mount,
            axis="x",
            min_gap=STAGE_1_HOME + STAGE_1_TRAVEL - 1e-6,
            name="stage_1_extends_forward",
        )
        ctx.expect_origin_gap(
            stage_2,
            stage_1,
            axis="x",
            min_gap=STAGE_2_HOME + STAGE_2_TRAVEL - 1e-6,
            name="stage_2_extends_forward",
        )
        ctx.expect_origin_gap(
            stage_3,
            stage_2,
            axis="x",
            min_gap=STAGE_3_HOME + STAGE_3_TRAVEL - 1e-6,
            name="stage_3_extends_forward",
        )
        ctx.expect_within(
            stage_2,
            stage_1,
            axes="yz",
            margin=0.0,
            name="stage_2_remains_centered_when_extended",
        )
        ctx.expect_within(
            stage_3,
            stage_2,
            axes="yz",
            margin=0.0,
            name="stage_3_remains_centered_when_extended",
        )
        ctx.expect_overlap(
            rear_mount,
            stage_1,
            axes="x",
            min_overlap=STAGE_1_REMAINING_OVERLAP - 0.005,
            name="stage_1_retains_outer_overlap_at_full_extension",
        )
        ctx.expect_overlap(
            stage_1,
            stage_2,
            axes="x",
            min_overlap=STAGE_2_REMAINING_OVERLAP - 0.005,
            name="stage_2_retains_stage_1_overlap_at_full_extension",
        )
        ctx.expect_overlap(
            stage_2,
            stage_3,
            axes="x",
            min_overlap=STAGE_3_REMAINING_OVERLAP - 0.005,
            name="stage_3_retains_stage_2_overlap_at_full_extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
