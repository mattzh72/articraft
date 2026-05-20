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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_slide")

    outer_finish = model.material("outer_finish", rgba=(0.32, 0.34, 0.37, 1.0))
    middle_finish = model.material("middle_finish", rgba=(0.70, 0.72, 0.74, 1.0))
    inner_finish = model.material("inner_finish", rgba=(0.18, 0.20, 0.23, 1.0))

    outer_length = 0.320
    outer_width = 0.045
    outer_height = 0.026
    wall_thickness = 0.002
    runner_height = 0.002
    runner_width_outer = 0.004
    runner_offset_outer = 0.010

    middle_length = 0.250
    middle_width = 0.033
    middle_height = 0.020
    runner_width_middle = 0.0035
    runner_offset_middle = 0.008

    inner_length = 0.170
    inner_width = 0.022
    inner_height = 0.012

    outer = model.part("outer_guide")
    outer.visual(
        Box((outer_length, outer_width, wall_thickness)),
        origin=Origin(xyz=(outer_length / 2.0, 0.0, wall_thickness / 2.0)),
        material=outer_finish,
        name="outer_base",
    )
    outer.visual(
        Box((outer_length, wall_thickness, outer_height - wall_thickness)),
        origin=Origin(
            xyz=(
                outer_length / 2.0,
                outer_width / 2.0 - wall_thickness / 2.0,
                wall_thickness + (outer_height - wall_thickness) / 2.0,
            )
        ),
        material=outer_finish,
        name="outer_left_wall",
    )
    outer.visual(
        Box((outer_length, wall_thickness, outer_height - wall_thickness)),
        origin=Origin(
            xyz=(
                outer_length / 2.0,
                -outer_width / 2.0 + wall_thickness / 2.0,
                wall_thickness + (outer_height - wall_thickness) / 2.0,
            )
        ),
        material=outer_finish,
        name="outer_right_wall",
    )
    outer.visual(
        Box((outer_length, runner_width_outer, runner_height)),
        origin=Origin(
            xyz=(outer_length / 2.0, runner_offset_outer, wall_thickness + runner_height / 2.0)
        ),
        material=outer_finish,
        name="outer_left_runner",
    )
    outer.visual(
        Box((outer_length, runner_width_outer, runner_height)),
        origin=Origin(
            xyz=(outer_length / 2.0, -runner_offset_outer, wall_thickness + runner_height / 2.0)
        ),
        material=outer_finish,
        name="outer_right_runner",
    )
    outer.visual(
        Box((0.004, outer_width - 2.0 * wall_thickness, 0.010)),
        origin=Origin(xyz=(outer_length - 0.002, 0.0, 0.007)),
        material=outer_finish,
        name="outer_front_stop",
    )
    outer.visual(
        Box((0.010, outer_width, 0.006)),
        origin=Origin(xyz=(0.005, 0.0, 0.005)),
        material=outer_finish,
        name="outer_rear_mount",
    )
    outer.visual(
        Box((outer_length, 0.005, 0.0016)),
        origin=Origin(xyz=(outer_length / 2.0, 0.0180, 0.0252)),
        material=outer_finish,
        name="outer_left_lip",
    )
    outer.visual(
        Box((outer_length, 0.005, 0.0016)),
        origin=Origin(xyz=(outer_length / 2.0, -0.0180, 0.0252)),
        material=outer_finish,
        name="outer_right_lip",
    )
    outer.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, outer_height)),
        mass=0.9,
        origin=Origin(xyz=(outer_length / 2.0, 0.0, outer_height / 2.0)),
    )

    middle = model.part("middle_carriage")
    middle.visual(
        Box((middle_length, middle_width, wall_thickness)),
        origin=Origin(xyz=(middle_length / 2.0, 0.0, wall_thickness / 2.0)),
        material=middle_finish,
        name="middle_base",
    )
    middle.visual(
        Box((middle_length, wall_thickness, middle_height - wall_thickness)),
        origin=Origin(
            xyz=(
                middle_length / 2.0,
                middle_width / 2.0 - wall_thickness / 2.0,
                wall_thickness + (middle_height - wall_thickness) / 2.0,
            )
        ),
        material=middle_finish,
        name="middle_left_wall",
    )
    middle.visual(
        Box((middle_length, wall_thickness, middle_height - wall_thickness)),
        origin=Origin(
            xyz=(
                middle_length / 2.0,
                -middle_width / 2.0 + wall_thickness / 2.0,
                wall_thickness + (middle_height - wall_thickness) / 2.0,
            )
        ),
        material=middle_finish,
        name="middle_right_wall",
    )
    middle.visual(
        Box((middle_length, runner_width_middle, runner_height)),
        origin=Origin(
            xyz=(middle_length / 2.0, runner_offset_middle, wall_thickness + runner_height / 2.0)
        ),
        material=middle_finish,
        name="middle_left_runner",
    )
    middle.visual(
        Box((middle_length, runner_width_middle, runner_height)),
        origin=Origin(
            xyz=(middle_length / 2.0, -runner_offset_middle, wall_thickness + runner_height / 2.0)
        ),
        material=middle_finish,
        name="middle_right_runner",
    )
    middle.visual(
        Box((0.004, middle_width - 2.0 * wall_thickness, 0.009)),
        origin=Origin(xyz=(middle_length - 0.002, 0.0, 0.0065)),
        material=middle_finish,
        name="middle_front_stop",
    )
    middle.visual(
        Box((0.006, middle_width, 0.006)),
        origin=Origin(xyz=(0.003, 0.0, 0.005)),
        material=middle_finish,
        name="middle_rear_stop",
    )
    middle.visual(
        Box((middle_length, 0.004, 0.0014)),
        origin=Origin(xyz=(middle_length / 2.0, 0.0125, 0.0193)),
        material=middle_finish,
        name="middle_left_lip",
    )
    middle.visual(
        Box((middle_length, 0.004, 0.0014)),
        origin=Origin(xyz=(middle_length / 2.0, -0.0125, 0.0193)),
        material=middle_finish,
        name="middle_right_lip",
    )
    middle.inertial = Inertial.from_geometry(
        Box((middle_length, middle_width, middle_height)),
        mass=0.55,
        origin=Origin(xyz=(middle_length / 2.0, 0.0, middle_height / 2.0)),
    )

    inner = model.part("inner_carriage")
    inner.visual(
        Box((inner_length, inner_width, 0.010)),
        origin=Origin(xyz=(inner_length / 2.0, 0.0, 0.005)),
        material=inner_finish,
        name="inner_body",
    )
    inner.visual(
        Box((0.110, 0.026, 0.002)),
        origin=Origin(xyz=(0.095, 0.0, 0.011)),
        material=inner_finish,
        name="inner_top_pad",
    )
    inner.visual(
        Box((0.012, 0.018, inner_height)),
        origin=Origin(xyz=(inner_length - 0.006, 0.0, inner_height / 2.0)),
        material=inner_finish,
        name="inner_nose",
    )
    inner.inertial = Inertial.from_geometry(
        Box((inner_length, inner_width, inner_height)),
        mass=0.32,
        origin=Origin(xyz=(inner_length / 2.0, 0.0, inner_height / 2.0)),
    )

    model.articulation(
        "outer_to_middle_slide",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(0.020, 0.0, wall_thickness + runner_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=0.160,
        ),
    )
    model.articulation(
        "middle_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(0.020, 0.0, wall_thickness + runner_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=0.110,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_guide")
    middle = object_model.get_part("middle_carriage")
    inner = object_model.get_part("inner_carriage")
    outer_slide = object_model.get_articulation("outer_to_middle_slide")
    inner_slide = object_model.get_articulation("middle_to_inner_slide")

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
        "serial_prismatic_layout",
        (
            outer_slide.articulation_type == ArticulationType.PRISMATIC
            and inner_slide.articulation_type == ArticulationType.PRISMATIC
            and tuple(outer_slide.axis) == (1.0, 0.0, 0.0)
            and tuple(inner_slide.axis) == (1.0, 0.0, 0.0)
            and outer_slide.parent == "outer_guide"
            and outer_slide.child == "middle_carriage"
            and inner_slide.parent == "middle_carriage"
            and inner_slide.child == "inner_carriage"
        ),
        details="Expected two serial +X prismatic joints: outer->middle and middle->inner.",
    )

    with ctx.pose({outer_slide: 0.0, inner_slide: 0.0}):
        ctx.expect_contact(
            middle,
            outer,
            elem_a="middle_base",
            elem_b="outer_left_runner",
            name="middle_left_runner_contact_closed",
        )
        ctx.expect_contact(
            middle,
            outer,
            elem_a="middle_base",
            elem_b="outer_right_runner",
            name="middle_right_runner_contact_closed",
        )
        ctx.expect_gap(
            middle,
            outer,
            axis="z",
            positive_elem="middle_base",
            negative_elem="outer_base",
            min_gap=0.002,
            max_gap=0.002,
            name="middle_base_clears_outer_floor",
        )
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle_cross_section_nested_in_outer",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.20,
            name="middle_retracted_overlap_in_outer",
        )

        ctx.expect_contact(
            inner,
            middle,
            elem_a="inner_body",
            elem_b="middle_left_runner",
            name="inner_left_runner_contact_closed",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a="inner_body",
            elem_b="middle_right_runner",
            name="inner_right_runner_contact_closed",
        )
        ctx.expect_gap(
            inner,
            middle,
            axis="z",
            positive_elem="inner_body",
            negative_elem="middle_base",
            min_gap=0.002,
            max_gap=0.002,
            name="inner_body_clears_middle_floor",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="inner_cross_section_nested_in_middle",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.15,
            name="inner_retracted_overlap_in_middle",
        )

    with ctx.pose({outer_slide: 0.160, inner_slide: 0.110}):
        ctx.fail_if_parts_overlap_in_current_pose(name="extended_pose_no_part_overlap")
        ctx.expect_contact(
            middle,
            outer,
            elem_a="middle_base",
            elem_b="outer_left_runner",
            name="middle_left_runner_contact_extended",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a="inner_body",
            elem_b="middle_left_runner",
            name="inner_left_runner_contact_extended",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            min_overlap=0.08,
            name="middle_retains_support_overlap_when_extended",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            min_overlap=0.05,
            name="inner_retains_support_overlap_when_extended",
        )
        ctx.expect_origin_gap(
            middle,
            outer,
            axis="x",
            min_gap=0.179,
            max_gap=0.181,
            name="middle_origin_moves_forward_in_extension",
        )
        ctx.expect_origin_gap(
            inner,
            middle,
            axis="x",
            min_gap=0.129,
            max_gap=0.131,
            name="inner_origin_moves_forward_from_middle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
