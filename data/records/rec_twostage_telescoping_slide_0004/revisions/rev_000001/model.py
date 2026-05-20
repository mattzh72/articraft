from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_telescoping_slide", assets=ASSETS)

    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    zinc = model.material("zinc", rgba=(0.75, 0.77, 0.80, 1.0))
    black = model.material("black_oxide", rgba=(0.17, 0.18, 0.19, 1.0))

    outer_length = 0.190
    outer_width = 0.040
    outer_height = 0.022
    outer_floor_t = 0.0025
    wall_t = 0.0020

    intermediate_length = 0.180
    intermediate_width = 0.032
    intermediate_height = 0.016
    intermediate_floor_t = 0.0020

    output_length = 0.170
    output_width = 0.024
    output_height = 0.010
    end_plate_t = 0.004
    end_plate_width = 0.030
    end_plate_height = 0.016

    travel = 0.120

    outer = model.part("outer_sleeve")
    outer.visual(
        Box((outer_length, outer_width, outer_floor_t)),
        origin=Origin(xyz=(outer_length / 2.0, 0.0, outer_floor_t / 2.0)),
        material=steel,
        name="floor",
    )
    outer.visual(
        Box((outer_length, wall_t, outer_height - outer_floor_t)),
        origin=Origin(
            xyz=(
                outer_length / 2.0,
                (outer_width - wall_t) / 2.0,
                outer_floor_t + (outer_height - outer_floor_t) / 2.0,
            )
        ),
        material=steel,
        name="left_wall",
    )
    outer.visual(
        Box((outer_length, wall_t, outer_height - outer_floor_t)),
        origin=Origin(
            xyz=(
                outer_length / 2.0,
                -(outer_width - wall_t) / 2.0,
                outer_floor_t + (outer_height - outer_floor_t) / 2.0,
            )
        ),
        material=steel,
        name="right_wall",
    )
    outer.visual(
        Box((0.010, outer_width + 0.006, 0.003)),
        origin=Origin(xyz=(0.005, 0.0, outer_height + 0.0015)),
        material=black,
        name="rear_mount_pad",
    )
    outer.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width + 0.006, outer_height + 0.003)),
        mass=0.55,
        origin=Origin(
            xyz=(
                outer_length / 2.0,
                0.0,
                (outer_height + 0.003) / 2.0,
            )
        ),
    )

    intermediate = model.part("intermediate_member")
    intermediate.visual(
        Box((intermediate_length, intermediate_width, intermediate_floor_t)),
        origin=Origin(
            xyz=(intermediate_length / 2.0, 0.0, intermediate_floor_t / 2.0)
        ),
        material=zinc,
        name="floor",
    )
    intermediate.visual(
        Box((intermediate_length, wall_t, intermediate_height - intermediate_floor_t)),
        origin=Origin(
            xyz=(
                intermediate_length / 2.0,
                (intermediate_width - wall_t) / 2.0,
                intermediate_floor_t + (intermediate_height - intermediate_floor_t) / 2.0,
            )
        ),
        material=zinc,
        name="left_wall",
    )
    intermediate.visual(
        Box((intermediate_length, wall_t, intermediate_height - intermediate_floor_t)),
        origin=Origin(
            xyz=(
                intermediate_length / 2.0,
                -(intermediate_width - wall_t) / 2.0,
                intermediate_floor_t + (intermediate_height - intermediate_floor_t) / 2.0,
            )
        ),
        material=zinc,
        name="right_wall",
    )
    intermediate.inertial = Inertial.from_geometry(
        Box((intermediate_length, intermediate_width, intermediate_height)),
        mass=0.36,
        origin=Origin(
            xyz=(
                intermediate_length / 2.0,
                0.0,
                intermediate_height / 2.0,
            )
        ),
    )

    output = model.part("output_member")
    output.visual(
        Box((output_length, output_width, output_height)),
        origin=Origin(xyz=(output_length / 2.0, 0.0, output_height / 2.0)),
        material=zinc,
        name="rail",
    )
    output.visual(
        Box((end_plate_t, end_plate_width, end_plate_height)),
        origin=Origin(
            xyz=(
                output_length + end_plate_t / 2.0,
                0.0,
                end_plate_height / 2.0,
            )
        ),
        material=black,
        name="end_plate",
    )
    output.inertial = Inertial.from_geometry(
        Box((output_length + end_plate_t, end_plate_width, end_plate_height)),
        mass=0.22,
        origin=Origin(
            xyz=(
                (output_length + end_plate_t) / 2.0,
                0.0,
                end_plate_height / 2.0,
            )
        ),
    )

    model.articulation(
        "outer_to_intermediate",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=intermediate,
        origin=Origin(xyz=(0.0, 0.0, outer_floor_t)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=travel,
        ),
    )
    model.articulation(
        "intermediate_to_output",
        ArticulationType.PRISMATIC,
        parent=intermediate,
        child=output,
        origin=Origin(xyz=(0.0, 0.0, intermediate_floor_t)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=50.0,
            velocity=0.30,
            lower=0.0,
            upper=travel,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer = object_model.get_part("outer_sleeve")
    intermediate = object_model.get_part("intermediate_member")
    output = object_model.get_part("output_member")

    stage_1 = object_model.get_articulation("outer_to_intermediate")
    stage_2 = object_model.get_articulation("intermediate_to_output")

    outer_floor = outer.get_visual("floor")
    intermediate_floor = intermediate.get_visual("floor")
    output_rail = output.get_visual("rail")
    end_plate = output.get_visual("end_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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

    ctx.check("outer sleeve exists", outer is not None)
    ctx.check("intermediate member exists", intermediate is not None)
    ctx.check("output member exists", output is not None)
    ctx.check("first stage articulation exists", stage_1 is not None)
    ctx.check("second stage articulation exists", stage_2 is not None)
    ctx.check("output end plate exists", end_plate is not None)

    ctx.expect_contact(
        intermediate,
        outer,
        elem_a=intermediate_floor,
        elem_b=outer_floor,
        name="intermediate floor rides on outer sleeve floor",
    )
    ctx.expect_within(
        intermediate,
        outer,
        axes="yz",
        margin=0.0,
        name="intermediate stays within outer sleeve section",
    )
    ctx.expect_overlap(
        intermediate,
        outer,
        axes="x",
        min_overlap=0.150,
        name="intermediate has strong engagement with outer sleeve at rest",
    )

    ctx.expect_contact(
        output,
        intermediate,
        elem_a=output_rail,
        elem_b=intermediate_floor,
        name="output rail rides on intermediate floor",
    )
    ctx.expect_within(
        output,
        intermediate,
        axes="yz",
        margin=0.0,
        inner_elem=output_rail,
        name="output rail stays within intermediate section",
    )
    ctx.expect_overlap(
        output,
        intermediate,
        axes="x",
        min_overlap=0.150,
        name="output rail has strong engagement with intermediate at rest",
    )

    with ctx.pose({stage_1: 0.120}):
        ctx.expect_origin_gap(
            intermediate,
            outer,
            axis="x",
            min_gap=0.119,
            max_gap=0.121,
            name="first stage travels about 120 mm",
        )
        ctx.expect_contact(
            intermediate,
            outer,
            elem_a=intermediate_floor,
            elem_b=outer_floor,
            name="first stage remains supported at full extension",
        )
        ctx.expect_overlap(
            intermediate,
            outer,
            axes="x",
            min_overlap=0.060,
            name="first stage retains overlap at full extension",
        )

    with ctx.pose({stage_2: 0.120}):
        ctx.expect_origin_gap(
            output,
            intermediate,
            axis="x",
            min_gap=0.119,
            max_gap=0.121,
            name="second stage travels about 120 mm",
        )
        ctx.expect_contact(
            output,
            intermediate,
            elem_a=output_rail,
            elem_b=intermediate_floor,
            name="second stage remains supported at full extension",
        )
        ctx.expect_overlap(
            output,
            intermediate,
            axes="x",
            min_overlap=0.050,
            name="second stage retains overlap at full extension",
        )

    with ctx.pose({stage_1: 0.120, stage_2: 0.120}):
        ctx.expect_origin_gap(
            output,
            outer,
            axis="x",
            min_gap=0.239,
            max_gap=0.241,
            name="combined extension stacks both 120 mm travels",
        )
        ctx.expect_gap(
            output,
            outer,
            axis="x",
            min_gap=0.200,
            positive_elem=end_plate,
            negative_elem=outer_floor,
            name="end plate projects clearly beyond the fixed sleeve when fully extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
