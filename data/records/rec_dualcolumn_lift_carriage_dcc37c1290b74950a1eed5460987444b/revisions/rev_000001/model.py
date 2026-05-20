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


MAST_OUTER_WIDTH = 0.92
UPRIGHT_WIDTH = 0.10
UPRIGHT_DEPTH = 0.08
UPRIGHT_HEIGHT = 1.72
UPRIGHT_CENTER_X = (MAST_OUTER_WIDTH - UPRIGHT_WIDTH) / 2.0

FOOT_WIDTH = 0.15
FOOT_LENGTH = 0.68
FOOT_THICKNESS = 0.08
FOOT_CENTER_Y = 0.22

HEEL_LENGTH = 0.16
HEEL_CENTER_Y = -0.10

REAR_TIE_WIDTH = MAST_OUTER_WIDTH - 0.10
REAR_TIE_DEPTH = 0.04
REAR_TIE_HEIGHT = 0.10
REAR_TIE_BOTTOM_Z = 0.28
REAR_TIE_CENTER_Y = -0.04

CROSSHEAD_WIDTH = MAST_OUTER_WIDTH
CROSSHEAD_DEPTH = 0.10
CROSSHEAD_HEIGHT = 0.12
CROSSHEAD_BOTTOM_Z = FOOT_THICKNESS + UPRIGHT_HEIGHT - 0.02
CROSSHEAD_CENTER_Y = -0.02

CARRIAGE_WIDTH = 0.58
CARRIAGE_DEPTH = 0.09
CARRIAGE_HEIGHT = 0.52
CARRIAGE_HOME_Z = 0.20
CARRIAGE_CENTER_Y = 0.055
CARRIAGE_TRAVEL = 1.00

SHOE_WIDTH = 0.070
SHOE_DEPTH = 0.030
SHOE_HEIGHT = 0.12
SHOE_CENTER_X = (CARRIAGE_WIDTH / 2.0) + (SHOE_WIDTH / 2.0)
SHOE_Y = 0.0
LOWER_SHOE_Z = 0.11
UPPER_SHOE_Z = 0.36


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_mast_pallet_lift_carriage")

    model.material("mast_steel", rgba=(0.36, 0.39, 0.43, 1.0))
    model.material("carriage_red", rgba=(0.73, 0.18, 0.12, 1.0))
    model.material("shoe_polymer", rgba=(0.09, 0.09, 0.10, 1.0))
    model.material("foot_steel", rgba=(0.30, 0.32, 0.36, 1.0))

    mast_frame = model.part("mast_frame")
    mast_frame.visual(
        Box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT)),
        origin=Origin(
            xyz=(-UPRIGHT_CENTER_X, 0.0, FOOT_THICKNESS + (UPRIGHT_HEIGHT / 2.0))
        ),
        material="mast_steel",
        name="left_upright",
    )
    mast_frame.visual(
        Box((UPRIGHT_WIDTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT)),
        origin=Origin(
            xyz=(UPRIGHT_CENTER_X, 0.0, FOOT_THICKNESS + (UPRIGHT_HEIGHT / 2.0))
        ),
        material="mast_steel",
        name="right_upright",
    )
    mast_frame.visual(
        Box((CROSSHEAD_WIDTH, CROSSHEAD_DEPTH, CROSSHEAD_HEIGHT)),
        origin=Origin(
            xyz=(0.0, CROSSHEAD_CENTER_Y, CROSSHEAD_BOTTOM_Z + (CROSSHEAD_HEIGHT / 2.0))
        ),
        material="mast_steel",
        name="crosshead",
    )
    mast_frame.visual(
        Box((REAR_TIE_WIDTH, REAR_TIE_DEPTH, REAR_TIE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, REAR_TIE_CENTER_Y, REAR_TIE_BOTTOM_Z + (REAR_TIE_HEIGHT / 2.0))
        ),
        material="mast_steel",
        name="rear_tie",
    )
    mast_frame.visual(
        Box((FOOT_WIDTH, FOOT_LENGTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, FOOT_CENTER_Y, FOOT_THICKNESS / 2.0)),
        material="foot_steel",
        name="left_foot",
    )
    mast_frame.visual(
        Box((FOOT_WIDTH, FOOT_LENGTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, FOOT_CENTER_Y, FOOT_THICKNESS / 2.0)),
        material="foot_steel",
        name="right_foot",
    )
    mast_frame.visual(
        Box((FOOT_WIDTH, HEEL_LENGTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(-UPRIGHT_CENTER_X, HEEL_CENTER_Y, FOOT_THICKNESS / 2.0)),
        material="foot_steel",
        name="left_heel",
    )
    mast_frame.visual(
        Box((FOOT_WIDTH, HEEL_LENGTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(UPRIGHT_CENTER_X, HEEL_CENTER_Y, FOOT_THICKNESS / 2.0)),
        material="foot_steel",
        name="right_heel",
    )
    mast_frame.inertial = Inertial.from_geometry(
        Box((MAST_OUTER_WIDTH, FOOT_LENGTH + 0.06, FOOT_THICKNESS + UPRIGHT_HEIGHT)),
        mass=125.0,
        origin=Origin(
            xyz=(0.0, 0.16, (FOOT_THICKNESS + UPRIGHT_HEIGHT + CROSSHEAD_HEIGHT) / 2.0)
        ),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_WIDTH, 0.014, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.026, CARRIAGE_HEIGHT / 2.0)),
        material="carriage_red",
        name="carriage_body",
    )
    carriage.visual(
        Box((CARRIAGE_WIDTH, 0.075, 0.10)),
        origin=Origin(xyz=(0.0, 0.006, 0.05)),
        material="carriage_red",
        name="lower_crossbox",
    )
    carriage.visual(
        Box((CARRIAGE_WIDTH, 0.070, 0.09)),
        origin=Origin(xyz=(0.0, 0.004, CARRIAGE_HEIGHT - 0.045)),
        material="carriage_red",
        name="upper_crossbox",
    )
    carriage.visual(
        Box((0.080, CARRIAGE_DEPTH, CARRIAGE_HEIGHT - 0.18)),
        origin=Origin(xyz=(-(CARRIAGE_WIDTH / 2.0) + 0.040, 0.0, 0.26)),
        material="carriage_red",
        name="left_cheek",
    )
    carriage.visual(
        Box((0.080, CARRIAGE_DEPTH, CARRIAGE_HEIGHT - 0.18)),
        origin=Origin(xyz=((CARRIAGE_WIDTH / 2.0) - 0.040, 0.0, 0.26)),
        material="carriage_red",
        name="right_cheek",
    )
    carriage.visual(
        Box((0.220, 0.060, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material="carriage_red",
        name="center_stiffener",
    )
    carriage.visual(
        Box((SHOE_WIDTH, SHOE_DEPTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(-SHOE_CENTER_X, SHOE_Y, LOWER_SHOE_Z)),
        material="shoe_polymer",
        name="left_lower_shoe",
    )
    carriage.visual(
        Box((SHOE_WIDTH, SHOE_DEPTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(-SHOE_CENTER_X, SHOE_Y, UPPER_SHOE_Z)),
        material="shoe_polymer",
        name="left_upper_shoe",
    )
    carriage.visual(
        Box((SHOE_WIDTH, SHOE_DEPTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(SHOE_CENTER_X, SHOE_Y, LOWER_SHOE_Z)),
        material="shoe_polymer",
        name="right_lower_shoe",
    )
    carriage.visual(
        Box((SHOE_WIDTH, SHOE_DEPTH, SHOE_HEIGHT)),
        origin=Origin(xyz=(SHOE_CENTER_X, SHOE_Y, UPPER_SHOE_Z)),
        material="shoe_polymer",
        name="right_upper_shoe",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT)),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast_frame,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_CENTER_Y, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
            effort=900.0,
            velocity=0.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    mast = object_model.get_part("mast_frame")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

    left_upright = mast.get_visual("left_upright")
    right_upright = mast.get_visual("right_upright")
    crosshead = mast.get_visual("crosshead")
    left_lower_shoe = carriage.get_visual("left_lower_shoe")
    left_upper_shoe = carriage.get_visual("left_upper_shoe")
    right_lower_shoe = carriage.get_visual("right_lower_shoe")
    right_upper_shoe = carriage.get_visual("right_upper_shoe")

    limits = lift.motion_limits
    ctx.check(
        "lift joint is vertical prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC and lift.axis == (0.0, 0.0, 1.0),
        details=f"type={lift.articulation_type}, axis={lift.axis}",
    )
    ctx.check(
        "lift travel limits are realistic",
        limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and 0.80 <= limits.upper <= 1.20,
        details=f"limits={limits}",
    )
    ctx.expect_origin_distance(
        mast,
        carriage,
        axes="x",
        max_dist=0.001,
        name="carriage stays centered between the twin masts",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=left_lower_shoe,
        elem_b=left_upright,
        contact_tol=1e-6,
        name="left lower shoe is supported by the left upright",
    )
    ctx.expect_contact(
        carriage,
        mast,
        elem_a=right_lower_shoe,
        elem_b=right_upright,
        contact_tol=1e-6,
        name="right lower shoe is supported by the right upright",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a=left_upper_shoe,
        elem_b=left_upright,
        min_overlap=0.10,
        name="left upper shoe overlaps the left upright over a meaningful guide height",
    )
    ctx.expect_overlap(
        carriage,
        mast,
        axes="z",
        elem_a=right_upper_shoe,
        elem_b=right_upright,
        min_overlap=0.10,
        name="right upper shoe overlaps the right upright over a meaningful guide height",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: CARRIAGE_TRAVEL}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem=crosshead,
            min_gap=0.03,
            max_gap=0.12,
            name="raised carriage stops just below the mast crosshead",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a=left_upper_shoe,
            elem_b=left_upright,
            min_overlap=0.10,
            name="left guide shoe remains engaged when the carriage is raised",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a=right_upper_shoe,
            elem_b=right_upright,
            min_overlap=0.10,
            name="right guide shoe remains engaged when the carriage is raised",
        )

    ctx.check(
        "carriage raises upward through the mast",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.90,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
