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


RAIL_LENGTH = 0.320
RAIL_BASE_WIDTH = 0.064
RAIL_BASE_THICK = 0.010
GUIDE_WIDTH = 0.024
GUIDE_HEIGHT = 0.012
RAIL_TOTAL_HEIGHT = RAIL_BASE_THICK + GUIDE_HEIGHT

CARRIAGE_LENGTH = 0.085
CARRIAGE_TOP_WIDTH = 0.072
CARRIAGE_TOP_HEIGHT = 0.019
CARRIAGE_RIB_WIDTH = 0.018
CARRIAGE_RIB_HEIGHT = 0.013
CARRIAGE_TOTAL_HEIGHT = CARRIAGE_RIB_HEIGHT + CARRIAGE_TOP_HEIGHT

TRAVEL_HALF_RANGE = 0.090


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_bench_slider")

    model.material("rail_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_silver", rgba=(0.73, 0.75, 0.78, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_BASE_THICK)),
        material="rail_gray",
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_THICK / 2.0)),
        name="rail_base",
    )
    rail.visual(
        Box((RAIL_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)),
        material="rail_gray",
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_THICK + GUIDE_HEIGHT / 2.0)),
        name="rail_guide",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_TOTAL_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOTAL_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_TOP_WIDTH, CARRIAGE_TOP_HEIGHT)),
        material="carriage_silver",
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_RIB_HEIGHT + CARRIAGE_TOP_HEIGHT / 2.0)
        ),
        name="carriage_top",
    )
    rib_offset_y = (GUIDE_WIDTH / 2.0) + 0.011
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_RIB_WIDTH, CARRIAGE_RIB_HEIGHT)),
        material="carriage_silver",
        origin=Origin(xyz=(0.0, rib_offset_y, CARRIAGE_RIB_HEIGHT / 2.0)),
        name="right_side_guide",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_RIB_WIDTH, CARRIAGE_RIB_HEIGHT)),
        material="carriage_silver",
        origin=Origin(xyz=(0.0, -rib_offset_y, CARRIAGE_RIB_HEIGHT / 2.0)),
        name="left_side_guide",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_TOP_WIDTH, CARRIAGE_TOTAL_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_THICK)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=150.0,
            velocity=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    slider = object_model.get_articulation("rail_to_carriage")

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

    limits = slider.motion_limits
    ctx.check(
        "slider_axis_is_linear_x",
        tuple(slider.axis) == (1.0, 0.0, 0.0),
        details=f"expected axis (1, 0, 0), got {slider.axis}",
    )
    ctx.check(
        "slider_has_symmetric_travel_limits",
        limits is not None
        and limits.lower == -TRAVEL_HALF_RANGE
        and limits.upper == TRAVEL_HALF_RANGE,
        details=f"unexpected motion limits: {limits}",
    )

    ctx.expect_contact(
        carriage,
        rail,
        name="carriage_is_supported_by_rail_at_home",
    )
    ctx.expect_within(
        carriage,
        rail,
        axes="x",
        margin=0.0,
        name="carriage_stays_on_rail_at_home",
    )

    with ctx.pose({slider: -TRAVEL_HALF_RANGE}):
        ctx.expect_contact(
            carriage,
            rail,
            name="carriage_keeps_contact_at_lower_travel",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            margin=0.0,
            name="carriage_stays_on_rail_at_lower_travel",
        )
        ctx.expect_origin_gap(
            rail,
            carriage,
            axis="x",
            min_gap=0.08,
            name="carriage_moves_negative_along_rail",
        )

    with ctx.pose({slider: TRAVEL_HALF_RANGE}):
        ctx.expect_contact(
            carriage,
            rail,
            name="carriage_keeps_contact_at_upper_travel",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            margin=0.0,
            name="carriage_stays_on_rail_at_upper_travel",
        )
        ctx.expect_origin_gap(
            carriage,
            rail,
            axis="x",
            min_gap=0.08,
            name="carriage_moves_positive_along_rail",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
