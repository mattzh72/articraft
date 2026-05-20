from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.34
BASE_W = 0.22
BASE_H = 0.026
BASE_RAIL_L = 0.28
BASE_RAIL_W = 0.022
BASE_RAIL_H = 0.010
BASE_RAIL_OFFSET_Y = 0.055

LOWER_L = 0.24
LOWER_W = 0.18
LOWER_PAD_L = 0.20
LOWER_PAD_W = 0.028
LOWER_PAD_H = 0.010
LOWER_DECK_T = 0.014
UPPER_RAIL_L = 0.14
UPPER_RAIL_W = 0.018
UPPER_RAIL_H = 0.008
UPPER_RAIL_OFFSET_X = 0.035

UPPER_FRAME_W = 0.12
UPPER_FRAME_L = 0.18
UPPER_PAD_L = 0.13
UPPER_PAD_W = 0.024
UPPER_PAD_H = 0.008
UPPER_DECK_T = 0.010
TOP_PLATE_W = 0.14
TOP_PLATE_L = 0.16
TOP_PLATE_T = 0.006
TOP_APERTURE_R = 0.022
TOP_SLOT_OFFSET_X = 0.040
TOP_SLOT_LEN = 0.052
TOP_SLOT_W = 0.008

LOWER_TRAVEL = 0.10
UPPER_TRAVEL = 0.08


def _base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_H, centered=(True, True, False))
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BASE_L - 0.060, BASE_W - 0.080)
        .cutBlind(-0.004)
    )

    rail = cq.Workplane("XY").box(
        BASE_RAIL_L, BASE_RAIL_W, BASE_RAIL_H, centered=(True, True, False)
    )
    left_rail = rail.translate((0.0, -BASE_RAIL_OFFSET_Y, BASE_H))
    right_rail = rail.translate((0.0, BASE_RAIL_OFFSET_Y, BASE_H))
    return base.union(left_rail).union(right_rail)


def _lower_carriage_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(
        LOWER_L, LOWER_W, LOWER_DECK_T, centered=(True, True, False)
    )
    deck = deck.translate((0.0, 0.0, LOWER_PAD_H))

    pad = cq.Workplane("XY").box(
        LOWER_PAD_L, LOWER_PAD_W, LOWER_PAD_H, centered=(True, True, False)
    )
    left_pad = pad.translate((0.0, -BASE_RAIL_OFFSET_Y, 0.0))
    right_pad = pad.translate((0.0, BASE_RAIL_OFFSET_Y, 0.0))

    cross_rail = cq.Workplane("XY").box(
        UPPER_RAIL_W, UPPER_RAIL_L, UPPER_RAIL_H, centered=(True, True, False)
    )
    lower_rail = cross_rail.translate(
        (-UPPER_RAIL_OFFSET_X, 0.0, LOWER_PAD_H + LOWER_DECK_T)
    )
    upper_rail = cross_rail.translate(
        (UPPER_RAIL_OFFSET_X, 0.0, LOWER_PAD_H + LOWER_DECK_T)
    )

    return deck.union(left_pad).union(right_pad).union(lower_rail).union(upper_rail)


def _upper_saddle_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(
        UPPER_FRAME_W, UPPER_FRAME_L, UPPER_DECK_T, centered=(True, True, False)
    )
    deck = deck.translate((0.0, 0.0, UPPER_PAD_H))

    pad = cq.Workplane("XY").box(
        UPPER_PAD_W, UPPER_PAD_L, UPPER_PAD_H, centered=(True, True, False)
    )
    left_pad = pad.translate((-UPPER_RAIL_OFFSET_X, 0.0, 0.0))
    right_pad = pad.translate((UPPER_RAIL_OFFSET_X, 0.0, 0.0))

    plate_z = UPPER_PAD_H + UPPER_DECK_T
    top_plate = cq.Workplane("XY").box(
        TOP_PLATE_W, TOP_PLATE_L, TOP_PLATE_T, centered=(True, True, False)
    )
    top_plate = top_plate.translate((0.0, 0.0, plate_z))

    center_cutter = (
        cq.Workplane("XY")
        .circle(TOP_APERTURE_R)
        .extrude(TOP_PLATE_T + 0.002)
        .translate((0.0, 0.0, plate_z - 0.001))
    )
    slot_cutter = (
        cq.Workplane("XY")
        .pushPoints([(-TOP_SLOT_OFFSET_X, 0.0), (TOP_SLOT_OFFSET_X, 0.0)])
        .slot2D(TOP_SLOT_LEN, TOP_SLOT_W, angle=90)
        .extrude(TOP_PLATE_T + 0.002)
        .translate((0.0, 0.0, plate_z - 0.001))
    )
    top_plate = top_plate.cut(center_cutter).cut(slot_cutter)

    return deck.union(left_pad).union(right_pad).union(top_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="microscope_xy_translation_table")

    model.material("base_charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("machined_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("stage_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "xy_table_base"),
        material="base_charcoal",
        name="base_body",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_H + BASE_RAIL_H)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + BASE_RAIL_H) / 2.0)),
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "xy_table_lower_carriage"),
        material="machined_steel",
        name="lower_stage",
    )
    lower_carriage.inertial = Inertial.from_geometry(
        Box((LOWER_L, LOWER_W, LOWER_PAD_H + LOWER_DECK_T + UPPER_RAIL_H)),
        mass=1.4,
        origin=Origin(
            xyz=(0.0, 0.0, (LOWER_PAD_H + LOWER_DECK_T + UPPER_RAIL_H) / 2.0)
        ),
    )

    upper_saddle = model.part("upper_saddle")
    upper_saddle.visual(
        mesh_from_cadquery(_upper_saddle_shape(), "xy_table_upper_saddle"),
        material="stage_black",
        name="upper_stage",
    )
    upper_saddle.inertial = Inertial.from_geometry(
        Box((TOP_PLATE_W, UPPER_FRAME_L, UPPER_PAD_H + UPPER_DECK_T + TOP_PLATE_T)),
        mass=1.0,
        origin=Origin(
            xyz=(0.0, 0.0, (UPPER_PAD_H + UPPER_DECK_T + TOP_PLATE_T) / 2.0)
        ),
    )

    model.articulation(
        "base_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_H + BASE_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-LOWER_TRAVEL / 2.0,
            upper=LOWER_TRAVEL / 2.0,
            effort=120.0,
            velocity=0.18,
        ),
    )
    model.articulation(
        "lower_carriage_to_upper_saddle",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_saddle,
        origin=Origin(xyz=(0.0, 0.0, LOWER_PAD_H + LOWER_DECK_T + UPPER_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-UPPER_TRAVEL / 2.0,
            upper=UPPER_TRAVEL / 2.0,
            effort=80.0,
            velocity=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_saddle = object_model.get_part("upper_saddle")
    lower_slide = object_model.get_articulation("base_to_lower_carriage")
    upper_slide = object_model.get_articulation("lower_carriage_to_upper_saddle")

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
        "all stage parts present",
        all(part is not None for part in (base, lower_carriage, upper_saddle)),
        "Expected base, lower_carriage, and upper_saddle parts.",
    )
    ctx.check(
        "orthogonal prismatic axes",
        tuple(lower_slide.axis) == (1.0, 0.0, 0.0)
        and tuple(upper_slide.axis) == (0.0, 1.0, 0.0),
        f"Expected X/Y slide axes, got {lower_slide.axis} and {upper_slide.axis}.",
    )

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        ctx.expect_contact(
            lower_carriage,
            base,
            name="lower carriage rests on base rails",
        )
        ctx.expect_contact(
            upper_saddle,
            lower_carriage,
            name="upper saddle rests on lower carriage rails",
        )
        ctx.expect_gap(
            upper_saddle,
            base,
            axis="z",
            min_gap=0.020,
            name="upper saddle is visibly above fixed base",
        )
        ctx.expect_overlap(
            lower_carriage,
            base,
            axes="xy",
            min_overlap=0.120,
            name="lower carriage remains broadly supported by base footprint",
        )

    with ctx.pose({lower_slide: 0.0, upper_slide: 0.0}):
        lower_rest = ctx.part_world_position(lower_carriage)
        upper_rest = ctx.part_world_position(upper_saddle)
    with ctx.pose({lower_slide: 0.030, upper_slide: 0.0}):
        lower_shifted = ctx.part_world_position(lower_carriage)
        upper_with_lower_shift = ctx.part_world_position(upper_saddle)
    with ctx.pose({lower_slide: 0.0, upper_slide: 0.025}):
        upper_shifted = ctx.part_world_position(upper_saddle)

    ctx.check(
        "lower carriage slides only along x",
        lower_rest is not None
        and lower_shifted is not None
        and lower_shifted[0] > lower_rest[0] + 0.020
        and abs(lower_shifted[1] - lower_rest[1]) < 1e-6
        and abs(lower_shifted[2] - lower_rest[2]) < 1e-6,
        f"Lower carriage positions: rest={lower_rest}, shifted={lower_shifted}.",
    )
    ctx.check(
        "upper saddle inherits x slide and adds independent y slide",
        upper_rest is not None
        and upper_with_lower_shift is not None
        and upper_shifted is not None
        and upper_with_lower_shift[0] > upper_rest[0] + 0.020
        and abs(upper_with_lower_shift[1] - upper_rest[1]) < 1e-6
        and upper_shifted[1] > upper_rest[1] + 0.015
        and abs(upper_shifted[0] - upper_rest[0]) < 1e-6,
        (
            f"Upper saddle positions: rest={upper_rest}, "
            f"lower-driven={upper_with_lower_shift}, y-shifted={upper_shifted}."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
