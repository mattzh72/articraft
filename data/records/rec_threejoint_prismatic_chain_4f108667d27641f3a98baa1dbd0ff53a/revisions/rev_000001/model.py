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


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.16
PLATE_HEIGHT = 0.22
BED_LENGTH = 0.24
BED_WIDTH = 0.10
BED_THICKNESS = 0.008
BASE_GUIDE_LENGTH = 0.20
BASE_GUIDE_WIDTH = 0.030
BASE_GUIDE_HEIGHT = 0.010
BRACE_LENGTH = 0.065
BRACE_WIDTH = 0.008
BRACE_HEIGHT = 0.055

STAGE1_LENGTH = 0.30
STAGE1_WIDTH = 0.080
STAGE1_BEAM_HEIGHT = 0.038
STAGE1_SKID_LENGTH = 0.210
STAGE1_SKID_WIDTH = 0.026
STAGE1_SKID_HEIGHT = 0.008
STAGE1_GUIDE_LENGTH = 0.170
STAGE1_GUIDE_WIDTH = 0.024
STAGE1_GUIDE_HEIGHT = 0.007
STAGE1_REAR_BLOCK_LENGTH = 0.026

STAGE2_LENGTH = 0.235
STAGE2_WIDTH = 0.064
STAGE2_BEAM_HEIGHT = 0.033
STAGE2_SKID_LENGTH = 0.165
STAGE2_SKID_WIDTH = 0.022
STAGE2_SKID_HEIGHT = 0.007
STAGE2_GUIDE_LENGTH = 0.136
STAGE2_GUIDE_WIDTH = 0.019
STAGE2_GUIDE_HEIGHT = 0.006
STAGE2_REAR_BLOCK_LENGTH = 0.022

STAGE3_LENGTH = 0.175
STAGE3_WIDTH = 0.048
STAGE3_BEAM_HEIGHT = 0.028
STAGE3_SKID_LENGTH = 0.128
STAGE3_SKID_WIDTH = 0.018
STAGE3_SKID_HEIGHT = 0.006
STAGE3_REAR_BLOCK_LENGTH = 0.018
STAGE3_TIP_PAD_LENGTH = 0.026
STAGE3_TIP_PAD_WIDTH = 0.036
STAGE3_TIP_PAD_HEIGHT = 0.010

BASE_TO_FIRST_HOME_X = 0.018
FIRST_TO_SECOND_HOME_X = 0.072
SECOND_TO_THIRD_HOME_X = 0.058

STAGE1_TRAVEL = 0.16
STAGE2_TRAVEL = 0.14
STAGE3_TRAVEL = 0.12


def _add_box(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def _stage_total_height(skid_height: float, beam_height: float, guide_height: float = 0.0) -> float:
    return skid_height + beam_height + guide_height


def _add_stage_part(
    model: ArticulatedObject,
    *,
    name: str,
    material: str,
    length: float,
    width: float,
    beam_height: float,
    skid_length: float,
    skid_width: float,
    skid_height: float,
    guide_length: float = 0.0,
    guide_width: float = 0.0,
    guide_height: float = 0.0,
    rear_block_length: float,
    mass: float,
    tip_pad_length: float = 0.0,
    tip_pad_width: float = 0.0,
    tip_pad_height: float = 0.0,
) -> None:
    part = model.part(name)

    _add_box(
        part,
        (length, width, beam_height),
        (length / 2.0, 0.0, skid_height + beam_height / 2.0),
        material,
        "main_beam",
    )
    _add_box(
        part,
        (rear_block_length, width * 0.82, beam_height * 0.72),
        (rear_block_length / 2.0, 0.0, skid_height + beam_height * 0.36),
        material,
        "rear_block",
    )
    _add_box(
        part,
        (skid_length, skid_width, skid_height),
        (0.020 + skid_length / 2.0, 0.0, skid_height / 2.0),
        material,
        "bottom_skid",
    )

    if guide_height > 0.0:
        _add_box(
            part,
            (guide_length, guide_width, guide_height),
            (length - 0.060 - guide_length / 2.0, 0.0, skid_height + beam_height + guide_height / 2.0),
            material,
            "top_guide",
        )

    total_height = _stage_total_height(skid_height, beam_height, guide_height)
    total_length = length
    total_width = width

    if tip_pad_length > 0.0 and tip_pad_height > 0.0 and tip_pad_width > 0.0:
        _add_box(
            part,
            (tip_pad_length, tip_pad_width, tip_pad_height),
            (length + tip_pad_length / 2.0, 0.0, skid_height + tip_pad_height / 2.0),
            material,
            "tip_pad",
        )
        total_length += tip_pad_length
        total_width = max(total_width, tip_pad_width)
        total_height = max(total_height, skid_height + tip_pad_height)

    part.inertial = Inertial.from_geometry(
        Box((total_length, total_width, total_height)),
        mass=mass,
        origin=Origin(xyz=(total_length / 2.0, 0.0, total_height / 2.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_triple_slide_module")

    model.material("mount_dark", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("stage_outer", rgba=(0.42, 0.46, 0.50, 1.0))
    model.material("stage_mid", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("stage_inner", rgba=(0.83, 0.84, 0.86, 1.0))

    base_support = model.part("base_support")
    _add_box(
        base_support,
        (PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT),
        (-PLATE_THICKNESS / 2.0, 0.0, PLATE_HEIGHT / 2.0),
        "mount_dark",
        "back_plate",
    )
    _add_box(
        base_support,
        (BED_LENGTH, BED_WIDTH, BED_THICKNESS),
        (BED_LENGTH / 2.0, 0.0, BED_THICKNESS / 2.0),
        "mount_dark",
        "bed_plate",
    )
    _add_box(
        base_support,
        (BRACE_LENGTH, BRACE_WIDTH, BRACE_HEIGHT),
        (BRACE_LENGTH / 2.0 - 0.004, -(BED_WIDTH / 2.0 - BRACE_WIDTH / 2.0), BRACE_HEIGHT / 2.0),
        "mount_dark",
        "left_brace",
    )
    _add_box(
        base_support,
        (BRACE_LENGTH, BRACE_WIDTH, BRACE_HEIGHT),
        (BRACE_LENGTH / 2.0 - 0.004, BED_WIDTH / 2.0 - BRACE_WIDTH / 2.0, BRACE_HEIGHT / 2.0),
        "mount_dark",
        "right_brace",
    )
    _add_box(
        base_support,
        (BASE_GUIDE_LENGTH, BASE_GUIDE_WIDTH, BASE_GUIDE_HEIGHT),
        (0.020 + BASE_GUIDE_LENGTH / 2.0, 0.0, BED_THICKNESS + BASE_GUIDE_HEIGHT / 2.0),
        "mount_dark",
        "guide_bar",
    )
    base_support.inertial = Inertial.from_geometry(
        Box((BED_LENGTH + PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=5.2,
        origin=Origin(xyz=((BED_LENGTH - PLATE_THICKNESS) / 2.0, 0.0, PLATE_HEIGHT / 2.0)),
    )

    _add_stage_part(
        model,
        name="first_stage",
        material="stage_outer",
        length=STAGE1_LENGTH,
        width=STAGE1_WIDTH,
        beam_height=STAGE1_BEAM_HEIGHT,
        skid_length=STAGE1_SKID_LENGTH,
        skid_width=STAGE1_SKID_WIDTH,
        skid_height=STAGE1_SKID_HEIGHT,
        guide_length=STAGE1_GUIDE_LENGTH,
        guide_width=STAGE1_GUIDE_WIDTH,
        guide_height=STAGE1_GUIDE_HEIGHT,
        rear_block_length=STAGE1_REAR_BLOCK_LENGTH,
        mass=2.0,
    )
    _add_stage_part(
        model,
        name="second_stage",
        material="stage_mid",
        length=STAGE2_LENGTH,
        width=STAGE2_WIDTH,
        beam_height=STAGE2_BEAM_HEIGHT,
        skid_length=STAGE2_SKID_LENGTH,
        skid_width=STAGE2_SKID_WIDTH,
        skid_height=STAGE2_SKID_HEIGHT,
        guide_length=STAGE2_GUIDE_LENGTH,
        guide_width=STAGE2_GUIDE_WIDTH,
        guide_height=STAGE2_GUIDE_HEIGHT,
        rear_block_length=STAGE2_REAR_BLOCK_LENGTH,
        mass=1.4,
    )
    _add_stage_part(
        model,
        name="third_stage",
        material="stage_inner",
        length=STAGE3_LENGTH,
        width=STAGE3_WIDTH,
        beam_height=STAGE3_BEAM_HEIGHT,
        skid_length=STAGE3_SKID_LENGTH,
        skid_width=STAGE3_SKID_WIDTH,
        skid_height=STAGE3_SKID_HEIGHT,
        rear_block_length=STAGE3_REAR_BLOCK_LENGTH,
        mass=0.9,
        tip_pad_length=STAGE3_TIP_PAD_LENGTH,
        tip_pad_width=STAGE3_TIP_PAD_WIDTH,
        tip_pad_height=STAGE3_TIP_PAD_HEIGHT,
    )

    first_stage = model.get_part("first_stage")
    second_stage = model.get_part("second_stage")
    third_stage = model.get_part("third_stage")

    model.articulation(
        "base_to_first_stage",
        ArticulationType.PRISMATIC,
        parent=base_support,
        child=first_stage,
        origin=Origin(xyz=(BASE_TO_FIRST_HOME_X, 0.0, BED_THICKNESS + BASE_GUIDE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.45,
            lower=0.0,
            upper=STAGE1_TRAVEL,
        ),
    )
    model.articulation(
        "first_to_second_stage",
        ArticulationType.PRISMATIC,
        parent=first_stage,
        child=second_stage,
        origin=Origin(
            xyz=(
                FIRST_TO_SECOND_HOME_X,
                0.0,
                _stage_total_height(STAGE1_SKID_HEIGHT, STAGE1_BEAM_HEIGHT, STAGE1_GUIDE_HEIGHT),
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.45,
            lower=0.0,
            upper=STAGE2_TRAVEL,
        ),
    )
    model.articulation(
        "second_to_third_stage",
        ArticulationType.PRISMATIC,
        parent=second_stage,
        child=third_stage,
        origin=Origin(
            xyz=(
                SECOND_TO_THIRD_HOME_X,
                0.0,
                _stage_total_height(STAGE2_SKID_HEIGHT, STAGE2_BEAM_HEIGHT, STAGE2_GUIDE_HEIGHT),
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.5,
            lower=0.0,
            upper=STAGE3_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_support = object_model.get_part("base_support")
    first_stage = object_model.get_part("first_stage")
    second_stage = object_model.get_part("second_stage")
    third_stage = object_model.get_part("third_stage")
    base_to_first = object_model.get_articulation("base_to_first_stage")
    first_to_second = object_model.get_articulation("first_to_second_stage")
    second_to_third = object_model.get_articulation("second_to_third_stage")

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
        "three_prismatic_sequence",
        all(
            articulation.articulation_type == ArticulationType.PRISMATIC
            for articulation in (base_to_first, first_to_second, second_to_third)
        ),
        "The base-to-first, first-to-second, and second-to-third joints must all be prismatic.",
    )
    ctx.check(
        "slide_axes_point_forward",
        all(articulation.axis == (1.0, 0.0, 0.0) for articulation in (base_to_first, first_to_second, second_to_third)),
        "All three slide stages should translate forward along +X.",
    )

    with ctx.pose(
        {
            base_to_first: 0.0,
            first_to_second: 0.0,
            second_to_third: 0.0,
        }
    ):
        ctx.expect_contact(
            base_support,
            first_stage,
            elem_a="guide_bar",
            elem_b="bottom_skid",
            name="first_stage_supported_by_base_guide",
        )
        ctx.expect_contact(
            first_stage,
            second_stage,
            elem_a="top_guide",
            elem_b="bottom_skid",
            name="second_stage_supported_by_first_stage_guide",
        )
        ctx.expect_contact(
            second_stage,
            third_stage,
            elem_a="top_guide",
            elem_b="bottom_skid",
            name="third_stage_supported_by_second_stage_guide",
        )
        ctx.expect_within(first_stage, base_support, axes="y", margin=0.0, name="first_stage_centered_on_base")
        ctx.expect_within(second_stage, first_stage, axes="y", margin=0.0, name="second_stage_centered_on_first_stage")
        ctx.expect_within(third_stage, second_stage, axes="y", margin=0.0, name="third_stage_centered_on_second_stage")
        ctx.expect_gap(
            first_stage,
            base_support,
            axis="z",
            min_gap=0.0,
            max_gap=0.0001,
            positive_elem="bottom_skid",
            negative_elem="guide_bar",
            name="first_stage_skid_seats_on_base_guide",
        )
        ctx.expect_gap(
            second_stage,
            first_stage,
            axis="z",
            min_gap=0.0,
            max_gap=0.0001,
            positive_elem="bottom_skid",
            negative_elem="top_guide",
            name="second_stage_skid_seats_on_first_stage_guide",
        )
        ctx.expect_gap(
            third_stage,
            second_stage,
            axis="z",
            min_gap=-0.0001,
            max_gap=0.0001,
            positive_elem="bottom_skid",
            negative_elem="top_guide",
            name="third_stage_skid_seats_on_second_stage_guide",
        )

    with ctx.pose(
        {
            base_to_first: STAGE1_TRAVEL,
            first_to_second: STAGE2_TRAVEL,
            second_to_third: STAGE3_TRAVEL,
        }
    ):
        ctx.expect_origin_gap(
            first_stage,
            base_support,
            axis="x",
            min_gap=BASE_TO_FIRST_HOME_X + STAGE1_TRAVEL - 0.001,
            max_gap=BASE_TO_FIRST_HOME_X + STAGE1_TRAVEL + 0.001,
            name="first_stage_extends_forward",
        )
        ctx.expect_origin_gap(
            second_stage,
            first_stage,
            axis="x",
            min_gap=FIRST_TO_SECOND_HOME_X + STAGE2_TRAVEL - 0.001,
            max_gap=FIRST_TO_SECOND_HOME_X + STAGE2_TRAVEL + 0.001,
            name="second_stage_extends_forward",
        )
        ctx.expect_origin_gap(
            third_stage,
            second_stage,
            axis="x",
            min_gap=SECOND_TO_THIRD_HOME_X + STAGE3_TRAVEL - 0.001,
            max_gap=SECOND_TO_THIRD_HOME_X + STAGE3_TRAVEL + 0.001,
            name="third_stage_extends_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
