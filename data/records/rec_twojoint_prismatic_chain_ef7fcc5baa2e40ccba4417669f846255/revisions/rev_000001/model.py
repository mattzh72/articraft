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


FUSE_EPS = 0.001

BASE_LENGTH = 0.50
BASE_WIDTH = 0.16
BASE_HEIGHT = 0.018

LOWER_RAIL_LENGTH = 0.42
LOWER_RAIL_WIDTH = 0.024
LOWER_RAIL_HEIGHT = 0.014
LOWER_RAIL_Y = 0.045

CAR1_LENGTH = 0.14
CAR1_WIDTH = 0.13
CAR1_FOOT_WIDTH = 0.032
CAR1_FOOT_HEIGHT = 0.012
CAR1_WEB_WIDTH = 0.014
CAR1_DECK_Z0 = 0.028
CAR1_DECK_THICKNESS = 0.014
CAR1_STAGE_PLATE_LENGTH = 0.20
CAR1_STAGE_PLATE_WIDTH = 0.070
CAR1_STAGE_PLATE_THICKNESS = 0.008

UPPER_RAIL_LENGTH = 0.28
UPPER_RAIL_WIDTH = 0.016
UPPER_RAIL_HEIGHT = 0.012
UPPER_RAIL_Y = 0.018

CAR2_LENGTH = 0.11
CAR2_WIDTH = 0.09
CAR2_FOOT_WIDTH = 0.022
CAR2_FOOT_HEIGHT = 0.010
CAR2_WEB_WIDTH = 0.010
CAR2_DECK_Z0 = 0.021
CAR2_DECK_THICKNESS = 0.012
CAR2_TOP_PLATE_LENGTH = 0.16
CAR2_TOP_PLATE_WIDTH = 0.11
CAR2_TOP_PLATE_THICKNESS = 0.010

PRIMARY_HOME_X = -0.09
PRIMARY_TRAVEL = 0.18
SECONDARY_HOME_X = -0.055
SECONDARY_TRAVEL = 0.11


def box_at(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def solid_box_at(
    length: float,
    width: float,
    height: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
):
    return box_at(length, width, height, x=x, y=y, z=z).val()


def finalized_shape(workplane: cq.Workplane):
    return workplane.findSolid()


def make_base_shape():
    plate_thickness = 0.010
    frame_rib_height = BASE_HEIGHT - plate_thickness
    rail_z0 = BASE_HEIGHT - FUSE_EPS

    base = box_at(BASE_LENGTH, BASE_WIDTH, plate_thickness)

    for rib_y in (-0.043, 0.043):
        base = base.union(
            solid_box_at(
                BASE_LENGTH * 0.90,
                0.030,
                frame_rib_height,
                y=rib_y,
                z=plate_thickness - FUSE_EPS,
            )
        )

    for cross_x in (-BASE_LENGTH * 0.36, BASE_LENGTH * 0.36):
        base = base.union(
            solid_box_at(
                0.038,
                BASE_WIDTH * 0.74,
                frame_rib_height,
                x=cross_x,
                z=plate_thickness - FUSE_EPS,
            )
        )

    for rail_y in (-LOWER_RAIL_Y, LOWER_RAIL_Y):
        base = base.union(
            solid_box_at(
                LOWER_RAIL_LENGTH,
                LOWER_RAIL_WIDTH,
                LOWER_RAIL_HEIGHT,
                y=rail_y,
                z=rail_z0,
            )
        )

    for stop_x in (-LOWER_RAIL_LENGTH / 2 + 0.020, LOWER_RAIL_LENGTH / 2 - 0.020):
        base = base.union(
            solid_box_at(
                0.020,
                BASE_WIDTH * 0.78,
                0.008,
                x=stop_x,
                z=BASE_HEIGHT - FUSE_EPS,
            )
        )

    return finalized_shape(base)


def make_primary_carriage_shape():
    deck = box_at(
        CAR1_LENGTH,
        CAR1_WIDTH,
        CAR1_DECK_THICKNESS,
        z=CAR1_DECK_Z0 - FUSE_EPS,
    )

    body = deck
    for foot_y in (-LOWER_RAIL_Y, LOWER_RAIL_Y):
        body = body.union(
            solid_box_at(
                CAR1_LENGTH,
                CAR1_FOOT_WIDTH,
                CAR1_FOOT_HEIGHT,
                y=foot_y,
            )
        )
        body = body.union(
            solid_box_at(
                CAR1_LENGTH,
                CAR1_WEB_WIDTH,
                CAR1_DECK_Z0 - CAR1_FOOT_HEIGHT,
                y=foot_y,
                z=CAR1_FOOT_HEIGHT - FUSE_EPS,
            )
        )

    stage_plate_z0 = CAR1_DECK_Z0 + CAR1_DECK_THICKNESS - (2 * FUSE_EPS)
    body = body.union(
        solid_box_at(
            CAR1_STAGE_PLATE_LENGTH,
            CAR1_STAGE_PLATE_WIDTH,
            CAR1_STAGE_PLATE_THICKNESS,
            z=stage_plate_z0,
        )
    )

    upper_rail_z0 = stage_plate_z0 + CAR1_STAGE_PLATE_THICKNESS - FUSE_EPS
    for rail_y in (-UPPER_RAIL_Y, UPPER_RAIL_Y):
        body = body.union(
            solid_box_at(
                UPPER_RAIL_LENGTH,
                UPPER_RAIL_WIDTH,
                UPPER_RAIL_HEIGHT,
                y=rail_y,
                z=upper_rail_z0,
            )
        )

    body = body.cut(
        solid_box_at(
            CAR1_LENGTH * 0.78,
            0.050,
            CAR1_DECK_Z0 - 0.004,
            z=0.004,
        )
    )
    body = body.cut(
        solid_box_at(
            CAR1_STAGE_PLATE_LENGTH * 0.58,
            0.022,
            CAR1_STAGE_PLATE_THICKNESS + UPPER_RAIL_HEIGHT * 0.4,
            z=stage_plate_z0 + 0.002,
        )
    )

    return finalized_shape(body)


def make_secondary_carriage_shape():
    deck = box_at(
        CAR2_LENGTH,
        CAR2_WIDTH,
        CAR2_DECK_THICKNESS,
        z=CAR2_DECK_Z0 - FUSE_EPS,
    )

    body = deck
    for foot_y in (-UPPER_RAIL_Y, UPPER_RAIL_Y):
        body = body.union(
            solid_box_at(
                CAR2_LENGTH,
                CAR2_FOOT_WIDTH,
                CAR2_FOOT_HEIGHT,
                y=foot_y,
            )
        )
        body = body.union(
            solid_box_at(
                CAR2_LENGTH,
                CAR2_WEB_WIDTH,
                CAR2_DECK_Z0 - CAR2_FOOT_HEIGHT,
                y=foot_y,
                z=CAR2_FOOT_HEIGHT - FUSE_EPS,
            )
        )

    top_plate_z0 = CAR2_DECK_Z0 + CAR2_DECK_THICKNESS - (2 * FUSE_EPS)
    body = body.union(
        solid_box_at(
            CAR2_TOP_PLATE_LENGTH,
            CAR2_TOP_PLATE_WIDTH,
            CAR2_TOP_PLATE_THICKNESS,
            z=top_plate_z0,
        )
    )
    body = body.union(
        solid_box_at(
            0.065,
            0.040,
            0.010,
            z=top_plate_z0 + CAR2_TOP_PLATE_THICKNESS - FUSE_EPS,
        )
    )

    body = body.cut(
        solid_box_at(
            CAR2_LENGTH * 0.68,
            0.026,
            CAR2_DECK_Z0 - 0.003,
            z=0.003,
        )
    )

    return finalized_shape(body)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_stage_prismatic_chain")

    model.material("machine_base", rgba=(0.30, 0.33, 0.36, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("anodized_stage", rgba=(0.18, 0.28, 0.46, 1.0))

    base = model.part("base_guide")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base_guide"),
        material="machine_base",
        name="base_body",
    )

    primary = model.part("primary_carriage")
    primary.visual(
        mesh_from_cadquery(make_primary_carriage_shape(), "primary_carriage"),
        material="machined_aluminum",
        name="primary_body",
    )

    secondary = model.part("secondary_carriage")
    secondary.visual(
        mesh_from_cadquery(make_secondary_carriage_shape(), "secondary_carriage"),
        material="anodized_stage",
        name="secondary_body",
    )

    lower_contact_z = BASE_HEIGHT + LOWER_RAIL_HEIGHT - FUSE_EPS
    stage_plate_z0 = CAR1_DECK_Z0 + CAR1_DECK_THICKNESS - (2 * FUSE_EPS)
    upper_rail_z0 = stage_plate_z0 + CAR1_STAGE_PLATE_THICKNESS - FUSE_EPS
    upper_contact_z = upper_rail_z0 + UPPER_RAIL_HEIGHT

    model.articulation(
        "base_to_primary_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=primary,
        origin=Origin(xyz=(PRIMARY_HOME_X, 0.0, lower_contact_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=PRIMARY_TRAVEL,
        ),
    )

    model.articulation(
        "primary_to_secondary_slide",
        ArticulationType.PRISMATIC,
        parent=primary,
        child=secondary,
        origin=Origin(xyz=(SECONDARY_HOME_X, 0.0, upper_contact_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.30,
            lower=0.0,
            upper=SECONDARY_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    primary = object_model.get_part("primary_carriage")
    secondary = object_model.get_part("secondary_carriage")
    primary_slide = object_model.get_articulation("base_to_primary_slide")
    secondary_slide = object_model.get_articulation("primary_to_secondary_slide")

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

    ctx.expect_contact(primary, base, name="primary_carriage_supported_on_base_guide")
    ctx.expect_contact(
        secondary,
        primary,
        name="secondary_carriage_supported_on_primary_stage",
    )

    with ctx.pose(
        {
            primary_slide: primary_slide.motion_limits.upper,
            secondary_slide: secondary_slide.motion_limits.upper,
        }
    ):
        ctx.expect_contact(
            primary,
            base,
            name="primary_carriage_supported_at_full_extension",
        )
        ctx.expect_contact(
            secondary,
            primary,
            name="secondary_carriage_supported_at_full_extension",
        )
        ctx.expect_overlap(
            primary,
            base,
            axes="xy",
            min_overlap=0.06,
            name="primary_carriage_remains_over_base_guide",
        )
        ctx.expect_overlap(
            secondary,
            primary,
            axes="xy",
            min_overlap=0.035,
            name="secondary_carriage_remains_over_upper_guide",
        )

    with ctx.pose({primary_slide: 0.0, secondary_slide: 0.0}):
        primary_closed = ctx.part_world_position(primary)

    with ctx.pose({primary_slide: primary_slide.motion_limits.upper, secondary_slide: 0.0}):
        primary_open = ctx.part_world_position(primary)

    primary_dx = primary_open[0] - primary_closed[0]
    primary_dy = primary_open[1] - primary_closed[1]
    primary_dz = primary_open[2] - primary_closed[2]
    ctx.check(
        "primary_slide_moves_along_positive_x",
        (
            abs(primary_dx - PRIMARY_TRAVEL) <= 1e-6
            and abs(primary_dy) <= 1e-6
            and abs(primary_dz) <= 1e-6
        ),
        (
            f"expected +x travel of {PRIMARY_TRAVEL:.3f} m, "
            f"got dx={primary_dx:.6f}, dy={primary_dy:.6f}, dz={primary_dz:.6f}"
        ),
    )

    with ctx.pose({primary_slide: PRIMARY_TRAVEL * 0.5, secondary_slide: 0.0}):
        primary_mid_closed = ctx.part_world_position(primary)
        secondary_closed = ctx.part_world_position(secondary)

    with ctx.pose(
        {
            primary_slide: PRIMARY_TRAVEL * 0.5,
            secondary_slide: secondary_slide.motion_limits.upper,
        }
    ):
        primary_mid_open = ctx.part_world_position(primary)
        secondary_open = ctx.part_world_position(secondary)

    secondary_dx = (secondary_open[0] - primary_mid_open[0]) - (
        secondary_closed[0] - primary_mid_closed[0]
    )
    secondary_dy = (secondary_open[1] - primary_mid_open[1]) - (
        secondary_closed[1] - primary_mid_closed[1]
    )
    secondary_dz = (secondary_open[2] - primary_mid_open[2]) - (
        secondary_closed[2] - primary_mid_closed[2]
    )
    ctx.check(
        "secondary_slide_moves_along_positive_x_relative_to_primary",
        (
            abs(secondary_dx - SECONDARY_TRAVEL) <= 1e-6
            and abs(secondary_dy) <= 1e-6
            and abs(secondary_dz) <= 1e-6
        ),
        (
            f"expected relative +x travel of {SECONDARY_TRAVEL:.3f} m, "
            f"got dx={secondary_dx:.6f}, dy={secondary_dy:.6f}, dz={secondary_dz:.6f}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
