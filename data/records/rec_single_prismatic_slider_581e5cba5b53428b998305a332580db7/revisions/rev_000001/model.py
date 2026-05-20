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


GUIDE_LENGTH = 0.42
GUIDE_WIDTH = 0.05
GUIDE_HEIGHT = 0.028
GUIDE_BOTTOM_T = 0.004
GUIDE_WALL_T = 0.006
GUIDE_WALL_H = GUIDE_HEIGHT - GUIDE_BOTTOM_T

CARRIAGE_LENGTH = 0.24
CARRIAGE_WIDTH = 0.032
CARRIAGE_HEIGHT = 0.016
CARRIAGE_Z = -GUIDE_HEIGHT / 2.0 + GUIDE_BOTTOM_T + CARRIAGE_HEIGHT / 2.0

END_PLATE_SIZE = 0.045
END_PLATE_T = 0.006
END_PLATE_OVERLAP = 0.001
END_PLATE_Z = -GUIDE_HEIGHT / 2.0 + GUIDE_BOTTOM_T + END_PLATE_SIZE / 2.0

RETRACTED_CARRIAGE_X = 0.09
SLIDE_TRAVEL = 0.15


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_slide_unit")

    channel_finish = model.material("channel_finish", color=(0.42, 0.44, 0.46))
    carriage_finish = model.material("carriage_finish", color=(0.76, 0.78, 0.80))
    plate_finish = model.material("plate_finish", color=(0.18, 0.19, 0.21))

    guide = model.part("guide_channel")
    bottom_z = -GUIDE_HEIGHT / 2.0 + GUIDE_BOTTOM_T / 2.0
    wall_z = -GUIDE_HEIGHT / 2.0 + GUIDE_BOTTOM_T + GUIDE_WALL_H / 2.0
    wall_y = GUIDE_WIDTH / 2.0 - GUIDE_WALL_T / 2.0
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_BOTTOM_T)),
        origin=Origin(xyz=(0.0, 0.0, bottom_z)),
        material=channel_finish,
        name="channel_base",
    )
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WALL_T, GUIDE_WALL_H)),
        origin=Origin(xyz=(0.0, wall_y, wall_z)),
        material=channel_finish,
        name="left_wall",
    )
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WALL_T, GUIDE_WALL_H)),
        origin=Origin(xyz=(0.0, -wall_y, wall_z)),
        material=channel_finish,
        name="right_wall",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_Z)),
        material=carriage_finish,
        name="carriage_body",
    )
    plate_x = CARRIAGE_LENGTH / 2.0 + END_PLATE_T / 2.0 - END_PLATE_OVERLAP
    carriage.visual(
        Box((END_PLATE_T, END_PLATE_SIZE, END_PLATE_SIZE)),
        origin=Origin(xyz=(plate_x, 0.0, END_PLATE_Z)),
        material=plate_finish,
        name="end_plate",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(RETRACTED_CARRIAGE_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide_channel")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guide_to_carriage")

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
        "slide_axis_is_positive_x",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected axis (1, 0, 0), got {slide.axis}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="carriage_body",
            name="carriage_body_supported_by_channel_when_retracted",
        )
        ctx.expect_within(
            carriage,
            guide,
            axes="yz",
            inner_elem="carriage_body",
            name="carriage_body_nested_inside_channel_profile_when_retracted",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="carriage_body",
            name="carriage_body_remains_supported_at_full_extension",
        )
        ctx.expect_within(
            carriage,
            guide,
            axes="yz",
            inner_elem="carriage_body",
            name="carriage_body_stays_laterally_captured_at_full_extension",
        )

        guide_x = ctx.part_world_position(guide)[0]
        carriage_x = ctx.part_world_position(carriage)[0]
        ctx.check(
            "positive_prismatic_motion_extends_carriage_outward",
            carriage_x > guide_x + 0.18,
            details=(
                f"expected carriage origin to move well forward of the guide: "
                f"guide_x={guide_x:.4f}, carriage_x={carriage_x:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
