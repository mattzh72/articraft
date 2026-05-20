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


PLATE_LENGTH = 0.76
PLATE_WIDTH = 0.28
PLATE_THICKNESS = 0.018
SLOT_LENGTH = 0.60
SLOT_WIDTH = 0.09

RAIL_LENGTH = 0.64
RAIL_WIDTH = 0.038
RAIL_HEIGHT = 0.018
RAIL_Y = 0.086

CARRIAGE_TRAVEL = 0.44
VERTICAL_TRAVEL = 0.12
HOME_X = -CARRIAGE_TRAVEL / 2.0
CARRIAGE_FRAME_Z = -0.040
SLIDE_HOME_Z = 0.013


def _plate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)
        .edges("|Z")
        .fillet(0.01)
        .faces(">Z")
        .workplane()
        .slot2D(SLOT_LENGTH, SLOT_WIDTH)
        .cutThruAll()
    )


def _rail_shape() -> cq.Workplane:
    return cq.Workplane("XY").box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)


def _carriage_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(0.13, 0.082, 0.016).translate((0.0, 0.0, -0.008))
    cross_brace = cq.Workplane("XY").box(0.18, 0.038, 0.024).translate((0.0, 0.0, -0.028))
    pedestal = cq.Workplane("XY").box(0.09, 0.055, 0.022).translate((0.0, 0.0, 0.002))
    carriage = deck.union(cross_brace).union(pedestal)

    for y_center in (-0.056, 0.056):
        wing = cq.Workplane("XY").box(0.15, 0.05, 0.022).translate((0.0, y_center, -0.007))
        carriage = carriage.union(wing)

    for y_center in (-RAIL_Y, RAIL_Y):
        truck = cq.Workplane("XY").box(0.14, 0.046, 0.018).translate((0.0, y_center, 0.004))
        carriage = carriage.union(truck)

    return carriage.edges("|Z").fillet(0.004)


def _lift_post_shape() -> cq.Workplane:
    shoe = cq.Workplane("XY").box(0.11, 0.07, 0.018).translate((0.0, 0.0, 0.009))
    collar = cq.Workplane("XY").box(0.074, 0.062, 0.018).translate((0.0, 0.0, 0.027))
    mast = cq.Workplane("XY").box(0.052, 0.040, 0.028).translate((0.0, 0.0, 0.046))
    neck = cq.Workplane("XY").box(0.046, 0.036, 0.010).translate((0.0, 0.0, 0.060))
    return shoe.union(collar).union(mast).union(neck).edges("|Z").fillet(0.003)


def _output_pad_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(0.15, 0.11, 0.012)
        .translate((0.0, 0.0, 0.071))
        .edges("|Z")
        .fillet(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_plate_xz_transfer_axis")

    model.material("painted_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    model.material("rail_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("anodized_aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_plate_shape(), "top_plate"),
        material="painted_steel",
        name="plate",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape().translate((0.0, RAIL_Y, -(PLATE_THICKNESS + RAIL_HEIGHT) / 2.0)), "left_rail"),
        material="rail_steel",
        name="left_rail",
    )
    base.visual(
        mesh_from_cadquery(_rail_shape().translate((0.0, -RAIL_Y, -(PLATE_THICKNESS + RAIL_HEIGHT) / 2.0)), "right_rail"),
        material="rail_steel",
        name="right_rail",
    )

    carriage = model.part("horizontal_carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "horizontal_carriage"),
        material="carriage_dark",
        name="carriage_body",
    )

    slide = model.part("vertical_slide")
    slide.visual(
        mesh_from_cadquery(_lift_post_shape(), "lift_post"),
        material="anodized_aluminum",
        name="lift_post",
    )
    slide.visual(
        mesh_from_cadquery(_output_pad_shape(), "output_pad"),
        material="pad_black",
        name="output_pad",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(HOME_X, 0.0, CARRIAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=300.0,
            velocity=0.30,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.0, 0.0, SLIDE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.20,
            lower=0.0,
            upper=VERTICAL_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("horizontal_carriage")
    slide = object_model.get_part("vertical_slide")
    x_stage = object_model.get_articulation("base_to_carriage")
    z_stage = object_model.get_articulation("carriage_to_slide")
    plate = base.get_visual("plate")
    pad = slide.get_visual("output_pad")

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
        "horizontal stage axis is +x",
        tuple(x_stage.axis) == (1.0, 0.0, 0.0),
        f"expected +X axis, got {x_stage.axis}",
    )
    ctx.check(
        "vertical stage axis is +z",
        tuple(z_stage.axis) == (0.0, 0.0, 1.0),
        f"expected +Z axis, got {z_stage.axis}",
    )
    ctx.check(
        "prismatic limits are positive travel windows",
        (
            x_stage.motion_limits is not None
            and z_stage.motion_limits is not None
            and x_stage.motion_limits.lower == 0.0
            and z_stage.motion_limits.lower == 0.0
            and x_stage.motion_limits.upper == CARRIAGE_TRAVEL
            and z_stage.motion_limits.upper == VERTICAL_TRAVEL
        ),
        "expected zero-based positive travel on both prismatic stages",
    )

    with ctx.pose({x_stage: 0.0, z_stage: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            name="carriage_is_supported_by_under_plate_rails",
        )
        ctx.expect_contact(
            slide,
            carriage,
            name="vertical_slide_rests_on_carriage_stop_at_home",
        )
        ctx.expect_gap(
            slide,
            base,
            axis="z",
            positive_elem=pad,
            negative_elem=plate,
            min_gap=0.012,
            max_gap=0.030,
            name="output_pad_starts_above_plate_surface",
        )

    with ctx.pose({x_stage: x_stage.motion_limits.upper, z_stage: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            name="carriage_remains_supported_at_full_x_stroke",
        )
        ctx.expect_gap(
            slide,
            base,
            axis="z",
            positive_elem=pad,
            negative_elem=plate,
            min_gap=0.012,
            max_gap=0.030,
            name="pad_clears_plate_at_full_x_stroke",
        )

    with ctx.pose({x_stage: 0.0, z_stage: z_stage.motion_limits.upper}):
        ctx.expect_gap(
            slide,
            base,
            axis="z",
            positive_elem=pad,
            negative_elem=plate,
            min_gap=0.13,
            name="pad_rises_clear_of_plate_when_z_stage_extends",
        )

    with ctx.pose({x_stage: 0.0, z_stage: 0.0}):
        carriage_home = ctx.part_world_position(carriage)
        slide_home = ctx.part_world_position(slide)
    with ctx.pose({x_stage: x_stage.motion_limits.upper, z_stage: 0.0}):
        carriage_end = ctx.part_world_position(carriage)
    with ctx.pose({x_stage: 0.0, z_stage: z_stage.motion_limits.upper}):
        slide_high = ctx.part_world_position(slide)

    ctx.check(
        "horizontal carriage moves in +x",
        carriage_home is not None
        and carriage_end is not None
        and carriage_end[0] - carriage_home[0] > 0.42,
        f"expected >0.42 m +X motion, got home={carriage_home}, end={carriage_end}",
    )
    ctx.check(
        "vertical slide moves in +z",
        slide_home is not None
        and slide_high is not None
        and slide_high[2] - slide_home[2] > 0.11,
        f"expected >0.11 m +Z motion, got home={slide_home}, high={slide_high}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
