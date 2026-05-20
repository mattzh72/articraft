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


BASE_TRAVEL = 0.34
BASE_GUIDE_LEN = 0.66
BASE_BEAM_W = 0.16
BASE_BEAM_H = 0.026
BASE_RAIL_W = 0.022
BASE_RAIL_H = 0.018
BASE_RAIL_Y = 0.05

FIRST_RUNNER_LEN = 0.24
FIRST_RUNNER_W = 0.018
FIRST_RUNNER_H = 0.008
STAGE1_CARRIAGE_LEN = 0.21
STAGE1_CARRIAGE_W = 0.135
STAGE1_CARRIAGE_H = 0.024

SECOND_TRAVEL = 0.26
SECOND_GUIDE_LEN = 0.46
SECOND_GUIDE_START = 0.02
SECOND_BEAM_W = 0.12
SECOND_BEAM_H = 0.022
SECOND_RAIL_W = 0.018
SECOND_RAIL_H = 0.016
SECOND_RAIL_Y = 0.038

SECOND_RUNNER_LEN = 0.18
SECOND_RUNNER_W = 0.016
SECOND_RUNNER_H = 0.007
STAGE2_CARRIAGE_LEN = 0.17
STAGE2_CARRIAGE_W = 0.112
STAGE2_CARRIAGE_H = 0.022

THIRD_TRAVEL = 0.18
THIRD_GUIDE_LEN = 0.31
THIRD_GUIDE_START = 0.015
THIRD_BEAM_W = 0.102
THIRD_BEAM_H = 0.018
THIRD_RAIL_W = 0.015
THIRD_RAIL_H = 0.013
THIRD_RAIL_Y = 0.031

END_RUNNER_LEN = 0.11
END_RUNNER_W = 0.014
END_RUNNER_H = 0.006
END_SLIDER_LEN = 0.10
END_SLIDER_W = 0.095
END_SLIDER_H = 0.02
END_PLATE_T = 0.012
END_PLATE_W = 0.145
END_PLATE_H = 0.13
BASE_RAIL_TOP_Z = BASE_BEAM_H + BASE_RAIL_H
FIRST_STAGE_TOP_Z = FIRST_RUNNER_H + STAGE1_CARRIAGE_H
SECOND_RAIL_TOP_Z = FIRST_STAGE_TOP_Z + SECOND_BEAM_H + SECOND_RAIL_H
SECOND_STAGE_TOP_Z = SECOND_RUNNER_H + STAGE2_CARRIAGE_H
THIRD_RAIL_TOP_Z = SECOND_STAGE_TOP_Z + THIRD_BEAM_H + THIRD_RAIL_H


def _box_origin(
    length: float,
    width: float,
    height: float,
    *,
    x_start: float = 0.0,
    y_center: float = 0.0,
    z_bottom: float = 0.0,
) -> Origin:
    return Origin(
        xyz=(
            x_start + (length / 2.0),
            y_center,
            z_bottom + (height / 2.0),
        )
    )


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    material,
    x_start: float = 0.0,
    y_center: float = 0.0,
    z_bottom: float = 0.0,
) -> None:
    part.visual(
        Box(size),
        origin=_box_origin(
            size[0],
            size[1],
            size[2],
            x_start=x_start,
            y_center=y_center,
            z_bottom=z_bottom,
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_linear_slide_chain")

    base_material = model.material("base_guide_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    first_material = model.material("first_stage_blue", rgba=(0.36, 0.48, 0.62, 1.0))
    second_material = model.material("second_stage_amber", rgba=(0.72, 0.55, 0.28, 1.0))
    end_material = model.material("end_stage_red", rgba=(0.70, 0.20, 0.18, 1.0))

    base = model.part("base_guide")
    _add_box_visual(
        base,
        name="base_beam",
        size=(BASE_GUIDE_LEN, BASE_BEAM_W, BASE_BEAM_H),
        material=base_material,
    )
    _add_box_visual(
        base,
        name="base_left_rail",
        size=(BASE_GUIDE_LEN, BASE_RAIL_W, BASE_RAIL_H),
        material=base_material,
        y_center=BASE_RAIL_Y,
        z_bottom=BASE_BEAM_H,
    )
    _add_box_visual(
        base,
        name="base_right_rail",
        size=(BASE_GUIDE_LEN, BASE_RAIL_W, BASE_RAIL_H),
        material=base_material,
        y_center=-BASE_RAIL_Y,
        z_bottom=BASE_BEAM_H,
    )

    first_stage = model.part("first_stage")
    _add_box_visual(
        first_stage,
        name="first_left_runner",
        size=(FIRST_RUNNER_LEN, FIRST_RUNNER_W, FIRST_RUNNER_H),
        material=first_material,
        y_center=BASE_RAIL_Y,
    )
    _add_box_visual(
        first_stage,
        name="first_right_runner",
        size=(FIRST_RUNNER_LEN, FIRST_RUNNER_W, FIRST_RUNNER_H),
        material=first_material,
        y_center=-BASE_RAIL_Y,
    )
    _add_box_visual(
        first_stage,
        name="first_carriage_body",
        size=(STAGE1_CARRIAGE_LEN, STAGE1_CARRIAGE_W, STAGE1_CARRIAGE_H),
        material=first_material,
        x_start=0.01,
        z_bottom=FIRST_RUNNER_H,
    )
    _add_box_visual(
        first_stage,
        name="second_guide_beam",
        size=(SECOND_GUIDE_LEN, SECOND_BEAM_W, SECOND_BEAM_H),
        material=first_material,
        x_start=SECOND_GUIDE_START,
        z_bottom=FIRST_STAGE_TOP_Z,
    )
    _add_box_visual(
        first_stage,
        name="second_left_rail",
        size=(SECOND_GUIDE_LEN, SECOND_RAIL_W, SECOND_RAIL_H),
        material=first_material,
        x_start=SECOND_GUIDE_START,
        y_center=SECOND_RAIL_Y,
        z_bottom=FIRST_STAGE_TOP_Z + SECOND_BEAM_H,
    )
    _add_box_visual(
        first_stage,
        name="second_right_rail",
        size=(SECOND_GUIDE_LEN, SECOND_RAIL_W, SECOND_RAIL_H),
        material=first_material,
        x_start=SECOND_GUIDE_START,
        y_center=-SECOND_RAIL_Y,
        z_bottom=FIRST_STAGE_TOP_Z + SECOND_BEAM_H,
    )

    second_stage = model.part("second_stage")
    _add_box_visual(
        second_stage,
        name="second_left_runner",
        size=(SECOND_RUNNER_LEN, SECOND_RUNNER_W, SECOND_RUNNER_H),
        material=second_material,
        y_center=SECOND_RAIL_Y,
    )
    _add_box_visual(
        second_stage,
        name="second_right_runner",
        size=(SECOND_RUNNER_LEN, SECOND_RUNNER_W, SECOND_RUNNER_H),
        material=second_material,
        y_center=-SECOND_RAIL_Y,
    )
    _add_box_visual(
        second_stage,
        name="second_carriage_body",
        size=(STAGE2_CARRIAGE_LEN, STAGE2_CARRIAGE_W, STAGE2_CARRIAGE_H),
        material=second_material,
        x_start=0.005,
        z_bottom=SECOND_RUNNER_H,
    )
    _add_box_visual(
        second_stage,
        name="third_guide_beam",
        size=(THIRD_GUIDE_LEN, THIRD_BEAM_W, THIRD_BEAM_H),
        material=second_material,
        x_start=THIRD_GUIDE_START,
        z_bottom=SECOND_STAGE_TOP_Z,
    )
    _add_box_visual(
        second_stage,
        name="third_left_rail",
        size=(THIRD_GUIDE_LEN, THIRD_RAIL_W, THIRD_RAIL_H),
        material=second_material,
        x_start=THIRD_GUIDE_START,
        y_center=THIRD_RAIL_Y,
        z_bottom=SECOND_STAGE_TOP_Z + THIRD_BEAM_H,
    )
    _add_box_visual(
        second_stage,
        name="third_right_rail",
        size=(THIRD_GUIDE_LEN, THIRD_RAIL_W, THIRD_RAIL_H),
        material=second_material,
        x_start=THIRD_GUIDE_START,
        y_center=-THIRD_RAIL_Y,
        z_bottom=SECOND_STAGE_TOP_Z + THIRD_BEAM_H,
    )

    end_stage = model.part("end_stage")
    _add_box_visual(
        end_stage,
        name="end_left_runner",
        size=(END_RUNNER_LEN, END_RUNNER_W, END_RUNNER_H),
        material=end_material,
        y_center=THIRD_RAIL_Y,
    )
    _add_box_visual(
        end_stage,
        name="end_right_runner",
        size=(END_RUNNER_LEN, END_RUNNER_W, END_RUNNER_H),
        material=end_material,
        y_center=-THIRD_RAIL_Y,
    )
    _add_box_visual(
        end_stage,
        name="end_slider_body",
        size=(END_SLIDER_LEN, END_SLIDER_W, END_SLIDER_H),
        material=end_material,
        x_start=0.005,
        z_bottom=END_RUNNER_H,
    )
    _add_box_visual(
        end_stage,
        name="end_plate",
        size=(END_PLATE_T, END_PLATE_W, END_PLATE_H),
        material=end_material,
        x_start=END_SLIDER_LEN,
        z_bottom=0.0,
    )

    model.articulation(
        "base_to_first_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.45,
            lower=0.0,
            upper=BASE_TRAVEL,
        ),
    )
    model.articulation(
        "first_to_second_stage",
        ArticulationType.PRISMATIC,
        parent=first_stage,
        child=second_stage,
        origin=Origin(xyz=(SECOND_GUIDE_START, 0.0, SECOND_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.35,
            lower=0.0,
            upper=SECOND_TRAVEL,
        ),
    )
    model.articulation(
        "second_to_end_stage",
        ArticulationType.PRISMATIC,
        parent=second_stage,
        child=end_stage,
        origin=Origin(xyz=(THIRD_GUIDE_START, 0.0, THIRD_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=THIRD_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    first_stage = object_model.get_part("first_stage")
    second_stage = object_model.get_part("second_stage")
    end_stage = object_model.get_part("end_stage")

    base_left_rail = base.get_visual("base_left_rail")
    base_right_rail = base.get_visual("base_right_rail")
    first_left_runner = first_stage.get_visual("first_left_runner")
    first_right_runner = first_stage.get_visual("first_right_runner")
    second_left_rail = first_stage.get_visual("second_left_rail")
    second_right_rail = first_stage.get_visual("second_right_rail")
    second_left_runner = second_stage.get_visual("second_left_runner")
    second_right_runner = second_stage.get_visual("second_right_runner")
    third_left_rail = second_stage.get_visual("third_left_rail")
    third_right_rail = second_stage.get_visual("third_right_rail")
    end_left_runner = end_stage.get_visual("end_left_runner")
    end_right_runner = end_stage.get_visual("end_right_runner")

    slide_1 = object_model.get_articulation("base_to_first_stage")
    slide_2 = object_model.get_articulation("first_to_second_stage")
    slide_3 = object_model.get_articulation("second_to_end_stage")

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

    ctx.fail_if_isolated_parts(max_pose_samples=12, name="sampled_pose_no_floating")
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=32,
        name="sampled_pose_no_articulation_overlap",
    )

    ctx.check(
        "all_joints_prismatic",
        all(
            joint.articulation_type == ArticulationType.PRISMATIC
            for joint in (slide_1, slide_2, slide_3)
        ),
        "Expected all three slide joints to be prismatic.",
    )
    ctx.check(
        "all_joint_axes_are_x",
        all(joint.axis == (1.0, 0.0, 0.0) for joint in (slide_1, slide_2, slide_3)),
        "Each guide should slide along its own local/world x-axis guide direction.",
    )

    def _check_limits(name: str, joint, expected_upper: float) -> None:
        limits = joint.motion_limits
        ok = (
            limits is not None
            and limits.lower == 0.0
            and limits.upper == expected_upper
            and limits.effort > 0.0
            and limits.velocity > 0.0
        )
        ctx.check(
            name,
            ok,
            f"{joint.name} should have bounded positive prismatic travel from 0.0 to {expected_upper:.3f} m.",
        )

    _check_limits("slide_1_limits", slide_1, BASE_TRAVEL)
    _check_limits("slide_2_limits", slide_2, SECOND_TRAVEL)
    _check_limits("slide_3_limits", slide_3, THIRD_TRAVEL)

    def _extent(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple(maxs[i] - mins[i] for i in range(3))

    with ctx.pose({slide_1: 0.0, slide_2: 0.0, slide_3: 0.0}):
        base_size = _extent(ctx.part_world_aabb(base))
        first_size = _extent(ctx.part_world_aabb(first_stage))
        second_size = _extent(ctx.part_world_aabb(second_stage))
        end_size = _extent(ctx.part_world_aabb(end_stage))

        ctx.check(
            "guide_lengths_descend",
            base_size is not None
            and first_size is not None
            and second_size is not None
            and end_size is not None
            and base_size[0] > first_size[0] > second_size[0] > end_size[0],
            "The telescoping guide bodies should step down in x-length from base to end stage.",
        )
        ctx.check(
            "end_plate_reads_as_plate",
            end_size is not None and end_size[1] >= 0.14 and end_size[2] >= 0.12,
            "The final stage should present a short slider with a visibly broad vertical end plate.",
        )

    def _check_pair_at_limits(
        prefix: str,
        parent,
        child,
        joint,
        *,
        left_support,
        right_support,
        left_runner,
        right_runner,
        min_x_overlap: float,
        min_y_overlap: float,
    ) -> None:
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            ctx.fail(prefix + "_missing_limits", "Bounded prismatic joint is missing lower/upper limits.")
            return
        for tag, value in (("lower", limits.lower), ("upper", limits.upper)):
            with ctx.pose({joint: value}):
                ctx.fail_if_parts_overlap_in_current_pose(name=f"{prefix}_{tag}_no_overlap")
                ctx.fail_if_isolated_parts(name=f"{prefix}_{tag}_no_floating")
                ctx.expect_contact(parent, child, name=f"{prefix}_{tag}_contact")
                ctx.expect_contact(
                    parent,
                    child,
                    elem_a=left_support,
                    elem_b=left_runner,
                    name=f"{prefix}_{tag}_left_runner_contact",
                )
                ctx.expect_contact(
                    parent,
                    child,
                    elem_a=right_support,
                    elem_b=right_runner,
                    name=f"{prefix}_{tag}_right_runner_contact",
                )
                ctx.expect_overlap(
                    parent,
                    child,
                    elem_a=left_support,
                    elem_b=left_runner,
                    axes="x",
                    min_overlap=min_x_overlap,
                    name=f"{prefix}_{tag}_left_x_overlap",
                )
                ctx.expect_overlap(
                    parent,
                    child,
                    elem_a=right_support,
                    elem_b=right_runner,
                    axes="x",
                    min_overlap=min_x_overlap,
                    name=f"{prefix}_{tag}_right_x_overlap",
                )
                ctx.expect_overlap(
                    parent,
                    child,
                    elem_a=left_support,
                    elem_b=left_runner,
                    axes="y",
                    min_overlap=min_y_overlap,
                    name=f"{prefix}_{tag}_left_y_overlap",
                )
                ctx.expect_overlap(
                    parent,
                    child,
                    elem_a=right_support,
                    elem_b=right_runner,
                    axes="y",
                    min_overlap=min_y_overlap,
                    name=f"{prefix}_{tag}_right_y_overlap",
                )

    _check_pair_at_limits(
        "base_first_slide",
        base,
        first_stage,
        slide_1,
        left_support=base_left_rail,
        right_support=base_right_rail,
        left_runner=first_left_runner,
        right_runner=first_right_runner,
        min_x_overlap=0.10,
        min_y_overlap=0.016,
    )
    _check_pair_at_limits(
        "first_second_slide",
        first_stage,
        second_stage,
        slide_2,
        left_support=second_left_rail,
        right_support=second_right_rail,
        left_runner=second_left_runner,
        right_runner=second_right_runner,
        min_x_overlap=0.08,
        min_y_overlap=0.015,
    )
    _check_pair_at_limits(
        "second_end_slide",
        second_stage,
        end_stage,
        slide_3,
        left_support=third_left_rail,
        right_support=third_right_rail,
        left_runner=end_left_runner,
        right_runner=end_right_runner,
        min_x_overlap=0.05,
        min_y_overlap=0.013,
    )

    with ctx.pose(
        {
            slide_1: BASE_TRAVEL,
            slide_2: SECOND_TRAVEL,
            slide_3: THIRD_TRAVEL,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="all_slides_extended_no_overlap")
        ctx.fail_if_isolated_parts(name="all_slides_extended_no_floating")
        ctx.expect_contact(base, first_stage, name="all_slides_extended_base_contact")
        ctx.expect_contact(first_stage, second_stage, name="all_slides_extended_first_contact")
        ctx.expect_contact(second_stage, end_stage, name="all_slides_extended_second_contact")
        ctx.expect_origin_gap(
            end_stage,
            base,
            axis="x",
            min_gap=0.75,
            name="chain_extends_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
