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


SUPPORT_LENGTH = 0.72
SUPPORT_WIDTH = 0.22
SUPPORT_HEIGHT = 0.09
REAR_PLATE_THICKNESS = 0.036
SUPPORT_RAIL_LENGTH = 0.648
SUPPORT_RAIL_WIDTH = 0.028
SUPPORT_RAIL_HEIGHT = 0.014
SUPPORT_RAIL_CENTER_Y = 0.078
SUPPORT_RAIL_CENTER_Z = 0.031
SUPPORT_RAIL_TOP_Z = SUPPORT_RAIL_CENTER_Z + SUPPORT_RAIL_HEIGHT / 2.0
SUPPORT_STRUT_LENGTH = 0.16
SUPPORT_STRUT_WIDTH = 0.012
SUPPORT_STRUT_HEIGHT = 0.038
SUPPORT_STRUT_CENTER_Y = 0.100

LONG_CARRIAGE_LENGTH = 0.58
LONG_CARRIAGE_SHOE_LENGTH = 0.54
LONG_CARRIAGE_SHOE_WIDTH = 0.030
LONG_CARRIAGE_SHOE_HEIGHT = 0.012
LONG_CARRIAGE_FRAME_WIDTH = 0.17
LONG_CARRIAGE_FRAME_HEIGHT = 0.044
LONG_CARRIAGE_DECK_WIDTH = 0.124
LONG_CARRIAGE_DECK_HEIGHT = 0.016
LONG_CARRIAGE_DECK_BOTTOM_Z = 0.028
LONG_CARRIAGE_TOP_RAIL_LENGTH = 0.40
LONG_CARRIAGE_TOP_RAIL_WIDTH = 0.014
LONG_CARRIAGE_TOP_RAIL_HEIGHT = 0.010
LONG_CARRIAGE_TOP_RAIL_CENTER_Y = 0.034
LONG_CARRIAGE_TOP_RAIL_CENTER_Z = 0.049
CARRIAGE_TRAVEL = 0.24

SHORT_SLIDE_LENGTH = 0.34
SHORT_SLIDE_SHOE_LENGTH = 0.28
SHORT_SLIDE_SHOE_WIDTH = 0.016
SHORT_SLIDE_SHOE_HEIGHT = 0.008
SHORT_SLIDE_FRAME_WIDTH = 0.11
SHORT_SLIDE_FRAME_HEIGHT = 0.028
SHORT_SLIDE_DECK_WIDTH = 0.10
SHORT_SLIDE_DECK_HEIGHT = 0.016
SHORT_SLIDE_DECK_BOTTOM_Z = 0.026
SHORT_SLIDE_RAIL_CENTER_Y = 0.034
SHORT_SLIDE_TRAVEL = 0.14

SUPPORT_TO_CARRIAGE_X = 0.05
CARRIAGE_TO_SLIDE_X = 0.12


def _add_box_visual(
    part,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_double_slide")

    model.material("support_gray", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("carriage_steel", rgba=(0.40, 0.43, 0.47, 1.0))
    model.material("slide_silver", rgba=(0.84, 0.85, 0.87, 1.0))

    rear_support = model.part("rear_support")
    _add_box_visual(
        rear_support,
        (REAR_PLATE_THICKNESS, SUPPORT_WIDTH, SUPPORT_HEIGHT),
        (REAR_PLATE_THICKNESS / 2.0, 0.0, SUPPORT_HEIGHT / 2.0),
        "support_gray",
        "rear_plate",
    )
    _add_box_visual(
        rear_support,
        (0.036, 0.18, 0.014),
        (0.018, 0.0, SUPPORT_HEIGHT - 0.007),
        "support_gray",
        "top_bridge",
    )
    _add_box_visual(
        rear_support,
        (SUPPORT_RAIL_LENGTH, SUPPORT_RAIL_WIDTH, SUPPORT_RAIL_HEIGHT),
        (
            REAR_PLATE_THICKNESS + SUPPORT_RAIL_LENGTH / 2.0,
            SUPPORT_RAIL_CENTER_Y,
            SUPPORT_RAIL_CENTER_Z,
        ),
        "support_gray",
        "left_support_rail",
    )
    _add_box_visual(
        rear_support,
        (SUPPORT_RAIL_LENGTH, SUPPORT_RAIL_WIDTH, SUPPORT_RAIL_HEIGHT),
        (
            REAR_PLATE_THICKNESS + SUPPORT_RAIL_LENGTH / 2.0,
            -SUPPORT_RAIL_CENTER_Y,
            SUPPORT_RAIL_CENTER_Z,
        ),
        "support_gray",
        "right_support_rail",
    )
    _add_box_visual(
        rear_support,
        (SUPPORT_STRUT_LENGTH, SUPPORT_STRUT_WIDTH, SUPPORT_STRUT_HEIGHT),
        (0.09, SUPPORT_STRUT_CENTER_Y, 0.071),
        "support_gray",
        "left_bridge_strut",
    )
    _add_box_visual(
        rear_support,
        (SUPPORT_STRUT_LENGTH, SUPPORT_STRUT_WIDTH, SUPPORT_STRUT_HEIGHT),
        (0.09, -SUPPORT_STRUT_CENTER_Y, 0.071),
        "support_gray",
        "right_bridge_strut",
    )
    _add_box_visual(
        rear_support,
        (0.06, 0.05, 0.018),
        (0.05, SUPPORT_RAIL_CENTER_Y, 0.009),
        "support_gray",
        "left_rear_pad",
    )
    _add_box_visual(
        rear_support,
        (0.06, 0.05, 0.018),
        (0.05, -SUPPORT_RAIL_CENTER_Y, 0.009),
        "support_gray",
        "right_rear_pad",
    )

    long_carriage = model.part("long_carriage")
    _add_box_visual(
        long_carriage,
        (LONG_CARRIAGE_SHOE_LENGTH, LONG_CARRIAGE_SHOE_WIDTH, LONG_CARRIAGE_SHOE_HEIGHT),
        (
            LONG_CARRIAGE_SHOE_LENGTH / 2.0,
            SUPPORT_RAIL_CENTER_Y,
            LONG_CARRIAGE_SHOE_HEIGHT / 2.0,
        ),
        "carriage_steel",
        "left_lower_shoe",
    )
    _add_box_visual(
        long_carriage,
        (LONG_CARRIAGE_SHOE_LENGTH, LONG_CARRIAGE_SHOE_WIDTH, LONG_CARRIAGE_SHOE_HEIGHT),
        (
            LONG_CARRIAGE_SHOE_LENGTH / 2.0,
            -SUPPORT_RAIL_CENTER_Y,
            LONG_CARRIAGE_SHOE_HEIGHT / 2.0,
        ),
        "carriage_steel",
        "right_lower_shoe",
    )
    _add_box_visual(
        long_carriage,
        (LONG_CARRIAGE_LENGTH, 0.012, 0.020),
        (LONG_CARRIAGE_LENGTH / 2.0, 0.060, 0.022),
        "carriage_steel",
        "left_side_beam",
    )
    _add_box_visual(
        long_carriage,
        (LONG_CARRIAGE_LENGTH, 0.012, 0.020),
        (LONG_CARRIAGE_LENGTH / 2.0, -0.060, 0.022),
        "carriage_steel",
        "right_side_beam",
    )
    _add_box_visual(
        long_carriage,
        (LONG_CARRIAGE_LENGTH, LONG_CARRIAGE_DECK_WIDTH, LONG_CARRIAGE_DECK_HEIGHT),
        (
            LONG_CARRIAGE_LENGTH / 2.0,
            0.0,
            LONG_CARRIAGE_DECK_BOTTOM_Z + LONG_CARRIAGE_DECK_HEIGHT / 2.0,
        ),
        "carriage_steel",
        "carriage_deck",
    )
    _add_box_visual(
        long_carriage,
        (0.018, LONG_CARRIAGE_FRAME_WIDTH, LONG_CARRIAGE_FRAME_HEIGHT),
        (0.009, 0.0, LONG_CARRIAGE_FRAME_HEIGHT / 2.0),
        "carriage_steel",
        "front_upright",
    )
    _add_box_visual(
        long_carriage,
        (0.016, 0.14, 0.034),
        (LONG_CARRIAGE_LENGTH - 0.008, 0.0, 0.017),
        "carriage_steel",
        "rear_stop_block",
    )
    _add_box_visual(
        long_carriage,
        (
            LONG_CARRIAGE_TOP_RAIL_LENGTH,
            LONG_CARRIAGE_TOP_RAIL_WIDTH,
            LONG_CARRIAGE_TOP_RAIL_HEIGHT,
        ),
        (
            0.26,
            LONG_CARRIAGE_TOP_RAIL_CENTER_Y,
            LONG_CARRIAGE_TOP_RAIL_CENTER_Z,
        ),
        "carriage_steel",
        "left_upper_rail",
    )
    _add_box_visual(
        long_carriage,
        (
            LONG_CARRIAGE_TOP_RAIL_LENGTH,
            LONG_CARRIAGE_TOP_RAIL_WIDTH,
            LONG_CARRIAGE_TOP_RAIL_HEIGHT,
        ),
        (
            0.26,
            -LONG_CARRIAGE_TOP_RAIL_CENTER_Y,
            LONG_CARRIAGE_TOP_RAIL_CENTER_Z,
        ),
        "carriage_steel",
        "right_upper_rail",
    )

    short_slide = model.part("short_slide")
    _add_box_visual(
        short_slide,
        (SHORT_SLIDE_SHOE_LENGTH, SHORT_SLIDE_SHOE_WIDTH, SHORT_SLIDE_SHOE_HEIGHT),
        (
            SHORT_SLIDE_SHOE_LENGTH / 2.0,
            SHORT_SLIDE_RAIL_CENTER_Y,
            SHORT_SLIDE_SHOE_HEIGHT / 2.0,
        ),
        "slide_silver",
        "left_upper_shoe",
    )
    _add_box_visual(
        short_slide,
        (SHORT_SLIDE_SHOE_LENGTH, SHORT_SLIDE_SHOE_WIDTH, SHORT_SLIDE_SHOE_HEIGHT),
        (
            SHORT_SLIDE_SHOE_LENGTH / 2.0,
            -SHORT_SLIDE_RAIL_CENTER_Y,
            SHORT_SLIDE_SHOE_HEIGHT / 2.0,
        ),
        "slide_silver",
        "right_upper_shoe",
    )
    _add_box_visual(
        short_slide,
        (SHORT_SLIDE_LENGTH, 0.010, 0.018),
        (SHORT_SLIDE_LENGTH / 2.0, 0.025, 0.017),
        "slide_silver",
        "left_side_beam",
    )
    _add_box_visual(
        short_slide,
        (SHORT_SLIDE_LENGTH, 0.010, 0.018),
        (SHORT_SLIDE_LENGTH / 2.0, -0.025, 0.017),
        "slide_silver",
        "right_side_beam",
    )
    _add_box_visual(
        short_slide,
        (SHORT_SLIDE_LENGTH, SHORT_SLIDE_DECK_WIDTH, SHORT_SLIDE_DECK_HEIGHT),
        (
            SHORT_SLIDE_LENGTH / 2.0,
            0.0,
            SHORT_SLIDE_DECK_BOTTOM_Z + SHORT_SLIDE_DECK_HEIGHT / 2.0,
        ),
        "slide_silver",
        "slide_deck",
    )
    _add_box_visual(
        short_slide,
        (0.016, SHORT_SLIDE_FRAME_WIDTH, SHORT_SLIDE_FRAME_HEIGHT),
        (0.008, 0.0, SHORT_SLIDE_FRAME_HEIGHT / 2.0),
        "slide_silver",
        "front_plate",
    )
    _add_box_visual(
        short_slide,
        (0.012, 0.08, 0.022),
        (SHORT_SLIDE_LENGTH - 0.006, 0.0, 0.011),
        "slide_silver",
        "nose_stop",
    )

    model.articulation(
        "support_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rear_support,
        child=long_carriage,
        origin=Origin(xyz=(SUPPORT_TO_CARRIAGE_X, 0.0, SUPPORT_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=0.45,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=long_carriage,
        child=short_slide,
        origin=Origin(
            xyz=(
                CARRIAGE_TO_SLIDE_X,
                0.0,
                LONG_CARRIAGE_TOP_RAIL_CENTER_Z + LONG_CARRIAGE_TOP_RAIL_HEIGHT / 2.0,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=0.50,
            lower=0.0,
            upper=SHORT_SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    long_carriage = object_model.get_part("long_carriage")
    short_slide = object_model.get_part("short_slide")
    support_to_carriage = object_model.get_articulation("support_to_carriage")
    carriage_to_slide = object_model.get_articulation("carriage_to_slide")

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
        "serial stages use +x prismatic axes",
        tuple(support_to_carriage.axis) == (1.0, 0.0, 0.0)
        and tuple(carriage_to_slide.axis) == (1.0, 0.0, 0.0),
        details=(
            f"support axis={support_to_carriage.axis}, "
            f"slide axis={carriage_to_slide.axis}"
        ),
    )

    with ctx.pose({support_to_carriage: 0.0, carriage_to_slide: 0.0}):
        ctx.expect_contact(
            long_carriage,
            rear_support,
            contact_tol=0.001,
            name="long carriage rides on rear support",
        )
        ctx.expect_contact(
            short_slide,
            long_carriage,
            contact_tol=0.001,
            name="short slide rides on long carriage",
        )
        ctx.expect_within(
            long_carriage,
            rear_support,
            axes="y",
            margin=0.001,
            name="long carriage stays between support cheeks",
        )
        ctx.expect_within(
            short_slide,
            long_carriage,
            axes="y",
            margin=0.001,
            name="short slide stays centered on carriage",
        )
        ctx.expect_overlap(
            long_carriage,
            rear_support,
            axes="x",
            min_overlap=0.45,
            name="carriage has deep support engagement when retracted",
        )
        ctx.expect_overlap(
            short_slide,
            long_carriage,
            axes="x",
            min_overlap=0.24,
            name="short slide has deep carriage engagement when retracted",
        )
        carriage_home = ctx.part_world_position(long_carriage)
        slide_home = ctx.part_world_position(short_slide)

    with ctx.pose(
        {
            support_to_carriage: CARRIAGE_TRAVEL,
            carriage_to_slide: SHORT_SLIDE_TRAVEL,
        }
    ):
        ctx.expect_contact(
            long_carriage,
            rear_support,
            contact_tol=0.001,
            name="long carriage stays guided at full extension",
        )
        ctx.expect_contact(
            short_slide,
            long_carriage,
            contact_tol=0.001,
            name="short slide stays guided at full extension",
        )
        ctx.expect_overlap(
            long_carriage,
            rear_support,
            axes="x",
            min_overlap=0.12,
            name="rear support still captures the long carriage",
        )
        ctx.expect_overlap(
            short_slide,
            long_carriage,
            axes="x",
            min_overlap=0.12,
            name="long carriage still captures the short slide",
        )
        carriage_extended = ctx.part_world_position(long_carriage)
        slide_extended = ctx.part_world_position(short_slide)

    ctx.check(
        "long carriage extends forward from the support",
        carriage_home is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_home[0] + 0.20,
        details=f"home={carriage_home}, extended={carriage_extended}",
    )
    ctx.check(
        "short slide advances beyond the long carriage travel",
        slide_home is not None
        and slide_extended is not None
        and carriage_extended is not None
        and slide_extended[0] > slide_home[0] + 0.35
        and slide_extended[0] > carriage_extended[0] + 0.10,
        details=(
            f"slide_home={slide_home}, slide_extended={slide_extended}, "
            f"carriage_extended={carriage_extended}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
