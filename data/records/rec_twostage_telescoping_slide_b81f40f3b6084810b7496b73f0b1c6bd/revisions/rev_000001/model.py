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


OUTER_LENGTH = 0.300
OUTER_WIDTH = 0.040
OUTER_HEIGHT = 0.016
OUTER_WALL = 0.002

MIDDLE_LENGTH = 0.240
MIDDLE_WIDTH = 0.032
MIDDLE_HEIGHT = 0.012
MIDDLE_WALL = 0.002

INNER_LENGTH = 0.190
INNER_WIDTH = 0.024
INNER_HEIGHT = 0.010
INNER_BOTTOM_OFFSET = 0.000

PLATE_THICKNESS = 0.004
PLATE_WIDTH = 0.032
PLATE_HEIGHT = 0.024

OUTER_TO_MIDDLE_OFFSET_X = 0.018
MIDDLE_TO_INNER_OFFSET_X = 0.098
OUTER_TO_MIDDLE_TRAVEL = 0.100
MIDDLE_TO_INNER_TRAVEL = 0.085

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_drawer_slide")

    zinc_steel = model.material("zinc_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.50, 0.53, 0.57, 1.0))
    coated_plate = model.material("coated_plate", rgba=(0.30, 0.33, 0.36, 1.0))

    outer_channel = model.part("outer_channel")
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_WALL)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_WALL / 2.0)),
        material=zinc_steel,
        name="outer_bottom",
    )
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                -(OUTER_WIDTH - OUTER_WALL) / 2.0,
                OUTER_WALL + (OUTER_HEIGHT - OUTER_WALL) / 2.0,
            )
        ),
        material=zinc_steel,
        name="outer_left_wall",
    )
    outer_channel.visual(
        Box((OUTER_LENGTH, OUTER_WALL, OUTER_HEIGHT - OUTER_WALL)),
        origin=Origin(
            xyz=(
                OUTER_LENGTH / 2.0,
                (OUTER_WIDTH - OUTER_WALL) / 2.0,
                OUTER_WALL + (OUTER_HEIGHT - OUTER_WALL) / 2.0,
            )
        ),
        material=zinc_steel,
        name="outer_right_wall",
    )
    outer_channel.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=0.60,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_HEIGHT / 2.0)),
    )

    middle_slide = model.part("middle_slide")
    middle_slide.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_WALL)),
        origin=Origin(
            xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT - MIDDLE_WALL / 2.0)
        ),
        material=dark_steel,
        name="middle_top",
    )
    middle_slide.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - MIDDLE_WALL)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                -(MIDDLE_WIDTH - MIDDLE_WALL) / 2.0,
                (MIDDLE_HEIGHT - MIDDLE_WALL) / 2.0,
            )
        ),
        material=dark_steel,
        name="middle_left_runner",
    )
    middle_slide.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WALL, MIDDLE_HEIGHT - MIDDLE_WALL)),
        origin=Origin(
            xyz=(
                MIDDLE_LENGTH / 2.0,
                (MIDDLE_WIDTH - MIDDLE_WALL) / 2.0,
                (MIDDLE_HEIGHT - MIDDLE_WALL) / 2.0,
            )
        ),
        material=dark_steel,
        name="middle_right_runner",
    )
    middle_slide.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=0.36,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_HEIGHT / 2.0)),
    )

    inner_slide = model.part("inner_slide")
    inner_slide.visual(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
        material=zinc_steel,
        name="inner_slide_body",
    )
    inner_slide.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=0.20,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_HEIGHT / 2.0)),
    )

    equipment_plate = model.part("equipment_plate")
    equipment_plate.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        origin=Origin(xyz=(PLATE_THICKNESS / 2.0, 0.0, PLATE_HEIGHT / 2.0)),
        material=coated_plate,
        name="equipment_plate",
    )
    equipment_plate.inertial = Inertial.from_geometry(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        mass=0.05,
        origin=Origin(xyz=(PLATE_THICKNESS / 2.0, 0.0, PLATE_HEIGHT / 2.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=middle_slide,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_OFFSET_X, 0.0, OUTER_WALL)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_slide,
        child=inner_slide,
        origin=Origin(xyz=(MIDDLE_TO_INNER_OFFSET_X, 0.0, INNER_BOTTOM_OFFSET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.35,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_plate",
        ArticulationType.FIXED,
        parent=inner_slide,
        child=equipment_plate,
        origin=Origin(xyz=(INNER_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_channel = object_model.get_part("outer_channel")
    middle_slide = object_model.get_part("middle_slide")
    inner_slide = object_model.get_part("inner_slide")
    equipment_plate = object_model.get_part("equipment_plate")

    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "outer_to_middle_is_prismatic",
        outer_to_middle.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={outer_to_middle.articulation_type}",
    )
    ctx.check(
        "middle_to_inner_is_prismatic",
        middle_to_inner.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={middle_to_inner.articulation_type}",
    )
    ctx.check(
        "outer_to_middle_axis_is_x",
        tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0),
        details=f"axis={outer_to_middle.axis}",
    )
    ctx.check(
        "middle_to_inner_axis_is_x",
        tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        details=f"axis={middle_to_inner.axis}",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_origin_gap(
            middle_slide,
            outer_channel,
            axis="x",
            min_gap=OUTER_TO_MIDDLE_OFFSET_X - 1e-6,
            max_gap=OUTER_TO_MIDDLE_OFFSET_X + 1e-6,
            name="middle_slide_closed_offset",
        )
        ctx.expect_origin_gap(
            inner_slide,
            middle_slide,
            axis="x",
            min_gap=MIDDLE_TO_INNER_OFFSET_X - 1e-6,
            max_gap=MIDDLE_TO_INNER_OFFSET_X + 1e-6,
            name="inner_slide_closed_offset",
        )
        ctx.expect_within(
            middle_slide,
            outer_channel,
            axes="yz",
            margin=0.0,
            name="middle_slide_nested_in_outer_channel",
        )
        ctx.expect_within(
            inner_slide,
            middle_slide,
            axes="yz",
            margin=0.0,
            name="inner_slide_nested_in_middle_slide",
        )
        ctx.expect_contact(
            middle_slide,
            outer_channel,
            name="middle_slide_supported_by_outer_channel",
        )
        ctx.expect_contact(
            inner_slide,
            middle_slide,
            name="inner_slide_supported_by_middle_slide",
        )
        ctx.expect_contact(
            equipment_plate,
            inner_slide,
            name="equipment_plate_contacts_inner_slide",
        )
        ctx.expect_overlap(
            equipment_plate,
            inner_slide,
            axes="yz",
            min_overlap=0.009,
            name="equipment_plate_aligned_to_inner_front",
        )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_origin_gap(
            middle_slide,
            outer_channel,
            axis="x",
            min_gap=OUTER_TO_MIDDLE_OFFSET_X + OUTER_TO_MIDDLE_TRAVEL - 1e-6,
            max_gap=OUTER_TO_MIDDLE_OFFSET_X + OUTER_TO_MIDDLE_TRAVEL + 1e-6,
            name="middle_slide_extends_along_x",
        )
        ctx.expect_origin_gap(
            inner_slide,
            middle_slide,
            axis="x",
            min_gap=MIDDLE_TO_INNER_OFFSET_X + MIDDLE_TO_INNER_TRAVEL - 1e-6,
            max_gap=MIDDLE_TO_INNER_OFFSET_X + MIDDLE_TO_INNER_TRAVEL + 1e-6,
            name="inner_slide_extends_along_x",
        )
        ctx.expect_within(
            middle_slide,
            outer_channel,
            axes="yz",
            margin=0.0,
            name="middle_slide_stays_within_outer_profile_when_extended",
        )
        ctx.expect_within(
            inner_slide,
            middle_slide,
            axes="yz",
            margin=0.0,
            name="inner_slide_stays_within_middle_profile_when_extended",
        )
        ctx.expect_contact(
            middle_slide,
            outer_channel,
            name="middle_slide_remains_supported_when_extended",
        )
        ctx.expect_contact(
            inner_slide,
            middle_slide,
            name="inner_slide_remains_supported_when_extended",
        )
        ctx.expect_gap(
            equipment_plate,
            outer_channel,
            axis="x",
            min_gap=0.18,
            name="equipment_plate_projects_forward_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
