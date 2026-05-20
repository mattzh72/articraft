from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_BASE_LENGTH = 0.72
RAIL_BASE_WIDTH = 0.12
RAIL_BASE_THICKNESS = 0.015
RAIL_END_PAD_LENGTH = 0.10
RAIL_END_PAD_HEIGHT = 0.018
RUNNER_LENGTH = 0.60
RUNNER_WIDTH = 0.060
RUNNER_HEIGHT = 0.030

CARRIAGE_LENGTH = 0.120
CARRIAGE_WIDTH = 0.100
CARRIAGE_HEIGHT = 0.055

GUIDE_WIDTH = 0.050
GUIDE_DEPTH = 0.050
GUIDE_HEIGHT = 0.280
GUIDE_BRACE_DEPTH = 0.018
GUIDE_BRACE_HEIGHT = 0.145
GUIDE_Y_OFFSET = -0.012

LIFT_BLOCK_WIDTH = 0.082
LIFT_BLOCK_DEPTH = 0.028
LIFT_BLOCK_HEIGHT = 0.085
TOP_ARM_WIDTH = 0.045
TOP_ARM_DEPTH = 0.022
TOP_ARM_HEIGHT = 0.040
TOP_PLATE_WIDTH = 0.100
TOP_PLATE_DEPTH = 0.070
TOP_PLATE_THICKNESS = 0.012

HORIZONTAL_TRAVEL = 0.300
VERTICAL_TRAVEL = 0.120

RUNNER_CENTER_Z = RAIL_BASE_THICKNESS + (RUNNER_HEIGHT * 0.5)
RUNNER_TOP_Z = RAIL_BASE_THICKNESS + RUNNER_HEIGHT
BASE_PLATFORM_THICKNESS = 0.032
LOWER_LIFT_CENTER_Z = 0.105
GUIDE_FRONT_Y = GUIDE_Y_OFFSET + (GUIDE_DEPTH * 0.5)


def make_rail_base_shape() -> cq.Workplane:
    base_bed = cq.Workplane("XY").box(
        RAIL_BASE_LENGTH,
        RAIL_BASE_WIDTH,
        RAIL_BASE_THICKNESS,
        centered=(True, True, False),
    )
    runner = cq.Workplane("XY").box(
        RUNNER_LENGTH,
        RUNNER_WIDTH,
        RUNNER_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, 0.0, RAIL_BASE_THICKNESS))

    return base_bed.union(runner)


def make_base_carriage_shape() -> cq.Workplane:
    carriage_pad = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        BASE_PLATFORM_THICKNESS,
        centered=(True, True, False),
    )

    upright_guide = cq.Workplane("XY").box(
        GUIDE_WIDTH,
        GUIDE_DEPTH,
        GUIDE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, GUIDE_Y_OFFSET, BASE_PLATFORM_THICKNESS))
    rear_brace = cq.Workplane("XY").box(
        0.080,
        GUIDE_BRACE_DEPTH,
        GUIDE_BRACE_HEIGHT,
        centered=(True, True, False),
    ).translate(
        (
            0.0,
            GUIDE_Y_OFFSET - ((GUIDE_DEPTH * 0.5) + (GUIDE_BRACE_DEPTH * 0.5)),
            BASE_PLATFORM_THICKNESS,
        )
    )

    return carriage_pad.union(upright_guide).union(rear_brace)


def make_lift_carriage_shape() -> cq.Workplane:
    front_slider = cq.Workplane("XY").box(
        LIFT_BLOCK_WIDTH,
        LIFT_BLOCK_DEPTH,
        LIFT_BLOCK_HEIGHT,
        centered=(True, False, False),
    )
    support_post = cq.Workplane("XY").box(
        TOP_ARM_WIDTH,
        TOP_ARM_DEPTH,
        TOP_ARM_HEIGHT,
        centered=(True, False, False),
    ).translate((0.0, 0.003, LIFT_BLOCK_HEIGHT))
    neck_block = cq.Workplane("XY").box(
        0.065,
        0.040,
        0.014,
        centered=(True, False, False),
    ).translate((0.0, 0.0, LIFT_BLOCK_HEIGHT + TOP_ARM_HEIGHT))
    top_plate = cq.Workplane("XY").box(
        TOP_PLATE_WIDTH,
        TOP_PLATE_DEPTH,
        TOP_PLATE_THICKNESS,
        centered=(True, False, False),
    ).translate((0.0, 0.0, LIFT_BLOCK_HEIGHT + TOP_ARM_HEIGHT + 0.014))

    return front_slider.union(support_post).union(neck_block).union(top_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_positioning_axis")

    dark_machine_gray = model.material("dark_machine_gray", rgba=(0.22, 0.23, 0.24, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    light_plate_gray = model.material("light_plate_gray", rgba=(0.78, 0.79, 0.82, 1.0))

    rail_base = model.part("rail_base")
    rail_base.visual(
        mesh_from_cadquery(make_rail_base_shape(), "rail_base"),
        material=dark_machine_gray,
        name="rail_base_body",
    )

    base_carriage = model.part("base_carriage")
    base_carriage.visual(
        mesh_from_cadquery(make_base_carriage_shape(), "base_carriage"),
        material=anodized_aluminum,
        name="base_carriage_body",
    )

    lift_carriage = model.part("lift_carriage")
    lift_carriage.visual(
        mesh_from_cadquery(make_lift_carriage_shape(), "lift_carriage"),
        material=light_plate_gray,
        name="lift_carriage_body",
    )

    model.articulation(
        "rail_to_base_carriage",
        ArticulationType.PRISMATIC,
        parent=rail_base,
        child=base_carriage,
        origin=Origin(xyz=(-(HORIZONTAL_TRAVEL * 0.5), 0.0, RUNNER_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.25,
            lower=0.0,
            upper=HORIZONTAL_TRAVEL,
        ),
    )

    model.articulation(
        "base_carriage_to_lift_carriage",
        ArticulationType.PRISMATIC,
        parent=base_carriage,
        child=lift_carriage,
        origin=Origin(xyz=(0.0, GUIDE_FRONT_Y, LOWER_LIFT_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.18,
            lower=0.0,
            upper=VERTICAL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail_base = object_model.get_part("rail_base")
    base_carriage = object_model.get_part("base_carriage")
    lift_carriage = object_model.get_part("lift_carriage")
    rail_slide = object_model.get_articulation("rail_to_base_carriage")
    lift_slide = object_model.get_articulation("base_carriage_to_lift_carriage")

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
        "parts_present",
        all(part is not None for part in (rail_base, base_carriage, lift_carriage)),
        "Expected rail base, base carriage, and lift carriage parts.",
    )
    ctx.check(
        "prismatic_joint_axes",
        rail_slide.articulation_type == ArticulationType.PRISMATIC
        and lift_slide.articulation_type == ArticulationType.PRISMATIC
        and rail_slide.axis == (1.0, 0.0, 0.0)
        and lift_slide.axis == (0.0, 0.0, 1.0),
        "The base carriage should slide horizontally along +X and the lift carriage vertically along +Z.",
    )

    ctx.expect_contact(
        base_carriage,
        rail_base,
        contact_tol=0.002,
        name="base_carriage_supported_by_horizontal_rail",
    )
    ctx.expect_contact(
        lift_carriage,
        base_carriage,
        contact_tol=0.002,
        name="lift_carriage_supported_by_upright_guide",
    )

    base_x_at_rest = ctx.part_world_position(base_carriage)[0]
    lift_z_at_rest = ctx.part_world_position(lift_carriage)[2]

    with ctx.pose({rail_slide: HORIZONTAL_TRAVEL}):
        ctx.expect_contact(
            base_carriage,
            rail_base,
            contact_tol=0.002,
            name="base_carriage_remains_supported_at_max_horizontal_travel",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_max_horizontal_travel")
        base_x_at_max = ctx.part_world_position(base_carriage)[0]

    with ctx.pose({lift_slide: VERTICAL_TRAVEL}):
        ctx.expect_contact(
            lift_carriage,
            base_carriage,
            contact_tol=0.002,
            name="lift_carriage_remains_guided_at_max_vertical_travel",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_max_vertical_travel")
        lift_z_at_max = ctx.part_world_position(lift_carriage)[2]

    ctx.check(
        "horizontal_travel_moves_positive_x",
        base_x_at_max > base_x_at_rest + 0.25,
        f"Expected more than 0.25 m of +X travel, got {base_x_at_max - base_x_at_rest:.4f} m.",
    )
    ctx.check(
        "vertical_travel_moves_positive_z",
        lift_z_at_max > lift_z_at_rest + 0.10,
        f"Expected more than 0.10 m of +Z travel, got {lift_z_at_max - lift_z_at_rest:.4f} m.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
