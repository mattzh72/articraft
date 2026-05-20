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


HOUSING_LENGTH = 0.36
HOUSING_WIDTH = 0.24
HOUSING_BASE_THICKNESS = 0.014
HOUSING_WALL_THICKNESS = 0.012
HOUSING_SIDE_LENGTH = 0.30
HOUSING_WALL_HEIGHT = 0.102
HOUSING_REAR_BULKHEAD_LENGTH = 0.04
HOUSING_FRONT_COLLAR_START = 0.28
HOUSING_FRONT_COLLAR_HEIGHT = 0.100

STAGE1_LENGTH = 0.92
STAGE1_WIDTH = 0.120
STAGE1_BODY_HEIGHT = 0.078
STAGE1_WALL = 0.007
STAGE1_SKID_WIDTH = 0.060
STAGE1_SKID_THICKNESS = 0.006
STAGE1_SKID_START = 0.03
STAGE1_SKID_LENGTH = 0.48

STAGE2_LENGTH = 0.70
STAGE2_WIDTH = 0.096
STAGE2_BODY_HEIGHT = 0.058
STAGE2_WALL = 0.006
STAGE2_SKID_WIDTH = 0.048
STAGE2_SKID_THICKNESS = 0.005
STAGE2_SKID_START = 0.03
STAGE2_SKID_LENGTH = 0.38

STAGE3_LENGTH = 0.54
STAGE3_WIDTH = 0.074
STAGE3_BODY_HEIGHT = 0.040
STAGE3_WALL = 0.005
STAGE3_SKID_WIDTH = 0.036
STAGE3_SKID_THICKNESS = 0.004
STAGE3_SKID_START = 0.02
STAGE3_SKID_LENGTH = 0.30

HOUSING_TO_STAGE1_HOME_X = 0.06
STAGE1_TO_STAGE2_HOME_X = 0.16
STAGE2_TO_STAGE3_HOME_X = 0.14

HOUSING_TO_STAGE1_TRAVEL = 0.26
STAGE1_TO_STAGE2_TRAVEL = 0.22
STAGE2_TO_STAGE3_TRAVEL = 0.18


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


def _add_housing_visuals(part, *, material: str) -> None:
    _add_box_visual(
        part,
        name="housing_base",
        size=(HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_BASE_THICKNESS),
        center=(HOUSING_LENGTH / 2.0, 0.0, HOUSING_BASE_THICKNESS / 2.0),
        material=material,
    )
    side_height = HOUSING_WALL_HEIGHT - HOUSING_BASE_THICKNESS
    side_center_z = HOUSING_BASE_THICKNESS + side_height / 2.0
    side_center_y = HOUSING_WIDTH / 2.0 - HOUSING_WALL_THICKNESS / 2.0
    _add_box_visual(
        part,
        name="housing_left_wall",
        size=(HOUSING_SIDE_LENGTH, HOUSING_WALL_THICKNESS, side_height),
        center=(HOUSING_SIDE_LENGTH / 2.0, side_center_y, side_center_z),
        material=material,
    )
    _add_box_visual(
        part,
        name="housing_right_wall",
        size=(HOUSING_SIDE_LENGTH, HOUSING_WALL_THICKNESS, side_height),
        center=(HOUSING_SIDE_LENGTH / 2.0, -side_center_y, side_center_z),
        material=material,
    )
    _add_box_visual(
        part,
        name="housing_rear_bulkhead",
        size=(HOUSING_REAR_BULKHEAD_LENGTH, HOUSING_WIDTH, side_height),
        center=(HOUSING_REAR_BULKHEAD_LENGTH / 2.0, 0.0, side_center_z),
        material=material,
    )
    front_cheek_length = HOUSING_LENGTH - HOUSING_SIDE_LENGTH
    front_opening_width = STAGE1_WIDTH + 0.016
    cheek_width = (HOUSING_WIDTH - front_opening_width) / 2.0
    cheek_center_y = front_opening_width / 2.0 + cheek_width / 2.0
    cheek_height = HOUSING_FRONT_COLLAR_HEIGHT - HOUSING_BASE_THICKNESS
    cheek_center_z = HOUSING_BASE_THICKNESS + cheek_height / 2.0
    cheek_center_x = HOUSING_SIDE_LENGTH + front_cheek_length / 2.0
    _add_box_visual(
        part,
        name="housing_front_left_cheek",
        size=(front_cheek_length, cheek_width, cheek_height),
        center=(cheek_center_x, cheek_center_y, cheek_center_z),
        material=material,
    )
    _add_box_visual(
        part,
        name="housing_front_right_cheek",
        size=(front_cheek_length, cheek_width, cheek_height),
        center=(cheek_center_x, -cheek_center_y, cheek_center_z),
        material=material,
    )


def _add_tube_visuals(
    part,
    *,
    prefix: str,
    material: str,
    length: float,
    width: float,
    body_height: float,
    wall: float,
    skid_width: float,
    skid_thickness: float,
    skid_start: float,
    skid_length: float,
) -> None:
    side_height = body_height - 2.0 * wall
    side_center_z = skid_thickness + wall + side_height / 2.0
    side_center_y = width / 2.0 - wall / 2.0
    _add_box_visual(
        part,
        name=f"{prefix}_bottom_plate",
        size=(length, width, wall),
        center=(length / 2.0, 0.0, skid_thickness + wall / 2.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_top_plate",
        size=(length, width, wall),
        center=(length / 2.0, 0.0, skid_thickness + body_height - wall / 2.0),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_left_wall",
        size=(length, wall, side_height),
        center=(length / 2.0, side_center_y, side_center_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_right_wall",
        size=(length, wall, side_height),
        center=(length / 2.0, -side_center_y, side_center_z),
        material=material,
    )
    _add_box_visual(
        part,
        name=f"{prefix}_skid",
        size=(skid_length, skid_width, skid_thickness),
        center=(skid_start + skid_length / 2.0, 0.0, skid_thickness / 2.0),
        material=material,
    )


def _tube_proxy_inertial(length: float, width: float, height: float, mass: float) -> Inertial:
    return Inertial.from_geometry(
        Box((length, width, height)),
        mass=mass,
        origin=Origin(xyz=(length / 2.0, 0.0, height / 2.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_telescoping_boom")

    model.material("housing_paint", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("outer_stage_paint", rgba=(0.16, 0.18, 0.20, 1.0))
    model.material("middle_stage_steel", rgba=(0.48, 0.50, 0.54, 1.0))
    model.material("inner_stage_steel", rgba=(0.68, 0.70, 0.74, 1.0))

    housing = model.part("root_housing")
    _add_housing_visuals(housing, material="housing_paint")
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_WALL_HEIGHT)),
        mass=18.0,
        origin=Origin(
            xyz=(
                HOUSING_LENGTH / 2.0,
                0.0,
                HOUSING_WALL_HEIGHT / 2.0,
            )
        ),
    )

    stage1 = model.part("outer_stage")
    _add_tube_visuals(
        stage1,
        prefix="outer_stage",
        material="outer_stage_paint",
        length=STAGE1_LENGTH,
        width=STAGE1_WIDTH,
        body_height=STAGE1_BODY_HEIGHT,
        wall=STAGE1_WALL,
        skid_width=STAGE1_SKID_WIDTH,
        skid_thickness=STAGE1_SKID_THICKNESS,
        skid_start=STAGE1_SKID_START,
        skid_length=STAGE1_SKID_LENGTH,
    )
    stage1.inertial = _tube_proxy_inertial(
        STAGE1_LENGTH,
        STAGE1_WIDTH,
        STAGE1_SKID_THICKNESS + STAGE1_BODY_HEIGHT,
        10.5,
    )

    stage2 = model.part("middle_stage")
    _add_tube_visuals(
        stage2,
        prefix="middle_stage",
        material="middle_stage_steel",
        length=STAGE2_LENGTH,
        width=STAGE2_WIDTH,
        body_height=STAGE2_BODY_HEIGHT,
        wall=STAGE2_WALL,
        skid_width=STAGE2_SKID_WIDTH,
        skid_thickness=STAGE2_SKID_THICKNESS,
        skid_start=STAGE2_SKID_START,
        skid_length=STAGE2_SKID_LENGTH,
    )
    stage2.inertial = _tube_proxy_inertial(
        STAGE2_LENGTH,
        STAGE2_WIDTH,
        STAGE2_SKID_THICKNESS + STAGE2_BODY_HEIGHT,
        6.5,
    )

    stage3 = model.part("inner_stage")
    _add_tube_visuals(
        stage3,
        prefix="inner_stage",
        material="inner_stage_steel",
        length=STAGE3_LENGTH,
        width=STAGE3_WIDTH,
        body_height=STAGE3_BODY_HEIGHT,
        wall=STAGE3_WALL,
        skid_width=STAGE3_SKID_WIDTH,
        skid_thickness=STAGE3_SKID_THICKNESS,
        skid_start=STAGE3_SKID_START,
        skid_length=STAGE3_SKID_LENGTH,
    )
    stage3.inertial = _tube_proxy_inertial(
        STAGE3_LENGTH,
        STAGE3_WIDTH,
        STAGE3_SKID_THICKNESS + STAGE3_BODY_HEIGHT,
        4.0,
    )

    model.articulation(
        "housing_to_outer_stage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=stage1,
        origin=Origin(xyz=(HOUSING_TO_STAGE1_HOME_X, 0.0, HOUSING_BASE_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.35,
            lower=0.0,
            upper=HOUSING_TO_STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "outer_to_middle_stage",
        ArticulationType.PRISMATIC,
        parent=stage1,
        child=stage2,
        origin=Origin(
            xyz=(
                STAGE1_TO_STAGE2_HOME_X,
                0.0,
                STAGE1_SKID_THICKNESS + STAGE1_WALL,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.40,
            lower=0.0,
            upper=STAGE1_TO_STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=stage2,
        child=stage3,
        origin=Origin(
            xyz=(
                STAGE2_TO_STAGE3_HOME_X,
                0.0,
                STAGE2_SKID_THICKNESS + STAGE2_WALL,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=STAGE2_TO_STAGE3_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("root_housing")
    stage1 = object_model.get_part("outer_stage")
    stage2 = object_model.get_part("middle_stage")
    stage3 = object_model.get_part("inner_stage")
    joint1 = object_model.get_articulation("housing_to_outer_stage")
    joint2 = object_model.get_articulation("outer_to_middle_stage")
    joint3 = object_model.get_articulation("middle_to_inner_stage")

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

    ctx.expect_contact(stage1, housing, name="outer_stage_supported_by_housing")
    ctx.expect_contact(stage2, stage1, name="middle_stage_supported_by_outer_stage")
    ctx.expect_contact(stage3, stage2, name="inner_stage_supported_by_middle_stage")

    ctx.expect_within(stage1, housing, axes="yz", margin=0.0, name="outer_stage_kept_in_housing_envelope")
    ctx.expect_within(stage2, stage1, axes="yz", margin=0.0, name="middle_stage_nested_in_outer_stage")
    ctx.expect_within(stage3, stage2, axes="yz", margin=0.0, name="inner_stage_nested_in_middle_stage")

    with ctx.pose({joint1: HOUSING_TO_STAGE1_TRAVEL * 0.85}):
        ctx.expect_origin_gap(
            stage1,
            housing,
            axis="x",
            min_gap=HOUSING_TO_STAGE1_HOME_X + HOUSING_TO_STAGE1_TRAVEL * 0.84,
            max_gap=HOUSING_TO_STAGE1_HOME_X + HOUSING_TO_STAGE1_TRAVEL * 0.86,
            name="outer_stage_translates_forward",
        )
        ctx.expect_within(stage2, stage1, axes="yz", margin=0.0, name="middle_stage_stays_aligned_when_outer_moves")

    with ctx.pose({joint2: STAGE1_TO_STAGE2_TRAVEL * 0.80}):
        ctx.expect_origin_gap(
            stage2,
            stage1,
            axis="x",
            min_gap=STAGE1_TO_STAGE2_HOME_X + STAGE1_TO_STAGE2_TRAVEL * 0.79,
            max_gap=STAGE1_TO_STAGE2_HOME_X + STAGE1_TO_STAGE2_TRAVEL * 0.81,
            name="middle_stage_translates_forward",
        )
        ctx.expect_within(stage3, stage2, axes="yz", margin=0.0, name="inner_stage_stays_aligned_when_middle_moves")

    with ctx.pose({joint3: STAGE2_TO_STAGE3_TRAVEL * 0.75}):
        ctx.expect_origin_gap(
            stage3,
            stage2,
            axis="x",
            min_gap=STAGE2_TO_STAGE3_HOME_X + STAGE2_TO_STAGE3_TRAVEL * 0.74,
            max_gap=STAGE2_TO_STAGE3_HOME_X + STAGE2_TO_STAGE3_TRAVEL * 0.76,
            name="inner_stage_translates_forward",
        )

    with ctx.pose(
        {
            joint1: HOUSING_TO_STAGE1_TRAVEL,
            joint2: STAGE1_TO_STAGE2_TRAVEL,
            joint3: STAGE2_TO_STAGE3_TRAVEL,
        }
    ):
        ctx.expect_within(stage2, stage1, axes="yz", margin=0.0, name="middle_stage_remains_coaxial_at_full_extension")
        ctx.expect_within(stage3, stage2, axes="yz", margin=0.0, name="inner_stage_remains_coaxial_at_full_extension")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
