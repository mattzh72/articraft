from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.56
BASE_WIDTH = 0.12
BASE_BODY_HEIGHT = 0.03
LOWER_RAIL_HEIGHT = 0.012
LOWER_RAIL_LENGTH = 0.48
LOWER_RAIL_WIDTH = 0.018
LOWER_RAIL_OFFSET_Y = 0.026
END_BLOCK_LENGTH = 0.07
END_BLOCK_EXTRA_HEIGHT = 0.012

CARRIAGE_LENGTH = 0.25
CARRIAGE_WIDTH = 0.16
CARRIAGE_BODY_HEIGHT = 0.048
CARRIAGE_SKIRT_LENGTH = 0.22
CARRIAGE_SKIRT_WIDTH = 0.018
CARRIAGE_SKIRT_DROP = 0.009
UPPER_RAIL_LENGTH = 0.18
UPPER_RAIL_WIDTH = 0.016
UPPER_RAIL_HEIGHT = 0.01
UPPER_RAIL_OFFSET_X = 0.04

CROSS_SLIDE_LENGTH_X = 0.18
CROSS_SLIDE_LENGTH_Y = 0.15
CROSS_SLIDE_BODY_HEIGHT = 0.04
CROSS_RIB_WIDTH = 0.018
CROSS_RIB_DEPTH = 0.024
CROSS_RIB_HEIGHT = 0.07
CROSS_RIB_OFFSET_X = 0.045
OUTPUT_FACE_SIZE = 0.12
OUTPUT_FACE_THICKNESS = 0.012

LOWER_STAGE_TRAVEL = 0.12
UPPER_STAGE_TRAVEL = 0.05


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    return cq.Workplane("XY").box(sx, sy, sz).translate(center)


def make_lower_guide() -> cq.Workplane:
    body = _box(
        (BASE_LENGTH, BASE_WIDTH, BASE_BODY_HEIGHT),
        (0.0, 0.0, BASE_BODY_HEIGHT / 2.0),
    )
    center_relief = _box(
        (LOWER_RAIL_LENGTH * 0.88, 0.05, 0.01),
        (0.0, 0.0, BASE_BODY_HEIGHT - 0.005),
    )
    shape = body.cut(center_relief)

    for x_sign in (-1.0, 1.0):
        shape = shape.union(
            _box(
                (END_BLOCK_LENGTH, BASE_WIDTH, END_BLOCK_EXTRA_HEIGHT),
                (
                    x_sign * (BASE_LENGTH / 2.0 - END_BLOCK_LENGTH / 2.0),
                    0.0,
                    BASE_BODY_HEIGHT + END_BLOCK_EXTRA_HEIGHT / 2.0,
                ),
            )
        )

    for y_sign in (-1.0, 1.0):
        shape = shape.union(
            _box(
                (LOWER_RAIL_LENGTH, LOWER_RAIL_WIDTH, LOWER_RAIL_HEIGHT),
                (
                    0.0,
                    y_sign * LOWER_RAIL_OFFSET_Y,
                    BASE_BODY_HEIGHT + LOWER_RAIL_HEIGHT / 2.0,
                ),
            )
        )

    return shape


def make_carriage_body() -> cq.Workplane:
    deck = _box(
        (CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT),
        (0.0, 0.0, CARRIAGE_BODY_HEIGHT / 2.0),
    )
    underside_channel = _box(
        (CARRIAGE_LENGTH * 0.82, 0.08, 0.018),
        (0.0, 0.0, 0.009),
    )
    top_relief = _box(
        (0.05, UPPER_RAIL_LENGTH * 0.84, 0.009),
        (0.0, 0.0, CARRIAGE_BODY_HEIGHT - 0.0045),
    )

    shape = deck.cut(underside_channel).cut(top_relief)

    for y_sign in (-1.0, 1.0):
        shape = shape.union(
            _box(
                (CARRIAGE_SKIRT_LENGTH, CARRIAGE_SKIRT_WIDTH, 0.024),
                (
                    0.0,
                    y_sign * (CARRIAGE_WIDTH / 2.0 - CARRIAGE_SKIRT_WIDTH / 2.0),
                    0.012,
                ),
            )
        )

    for x_sign in (-1.0, 1.0):
        shape = shape.union(
            _box(
                (UPPER_RAIL_WIDTH, UPPER_RAIL_LENGTH, UPPER_RAIL_HEIGHT),
                (
                    x_sign * UPPER_RAIL_OFFSET_X,
                    0.0,
                    CARRIAGE_BODY_HEIGHT + UPPER_RAIL_HEIGHT / 2.0,
                ),
            )
        )

    return shape


def make_cross_slide_body() -> cq.Workplane:
    body = _box(
        (CROSS_SLIDE_LENGTH_X, CROSS_SLIDE_LENGTH_Y, CROSS_SLIDE_BODY_HEIGHT),
        (0.0, 0.0, CROSS_SLIDE_BODY_HEIGHT / 2.0),
    )
    top_relief = _box(
        (0.1, 0.08, 0.008),
        (0.0, -0.01, CROSS_SLIDE_BODY_HEIGHT - 0.004),
    )
    shape = body.cut(top_relief)

    for x_sign in (-1.0, 1.0):
        shape = shape.union(
            _box(
                (CROSS_RIB_WIDTH, CROSS_RIB_DEPTH, CROSS_RIB_HEIGHT),
                (
                    x_sign * CROSS_RIB_OFFSET_X,
                    CROSS_SLIDE_LENGTH_Y / 2.0 - CROSS_RIB_DEPTH / 2.0,
                    CROSS_RIB_HEIGHT / 2.0,
                ),
            )
        )

    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_xy_axis")

    lower_guide_material = model.material("lower_guide_material", rgba=(0.16, 0.17, 0.19, 1.0))
    carriage_material = model.material("carriage_material", rgba=(0.62, 0.66, 0.71, 1.0))
    cross_slide_material = model.material("cross_slide_material", rgba=(0.76, 0.79, 0.83, 1.0))
    output_face_material = model.material("output_face_material", rgba=(0.86, 0.88, 0.9, 1.0))

    lower_guide = model.part("lower_guide")
    lower_guide.visual(
        mesh_from_cadquery(make_lower_guide(), "lower_guide"),
        origin=Origin(),
        material=lower_guide_material,
        name="guide_body",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_body(), "carriage"),
        origin=Origin(),
        material=carriage_material,
        name="carriage_body",
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        mesh_from_cadquery(make_cross_slide_body(), "cross_slide_body"),
        origin=Origin(),
        material=cross_slide_material,
        name="cross_slide_body",
    )
    cross_slide.visual(
        Box((OUTPUT_FACE_SIZE, OUTPUT_FACE_THICKNESS, OUTPUT_FACE_SIZE)),
        origin=Origin(
            xyz=(
                0.0,
                CROSS_SLIDE_LENGTH_Y / 2.0 + OUTPUT_FACE_THICKNESS / 2.0,
                OUTPUT_FACE_SIZE / 2.0,
            )
        ),
        material=output_face_material,
        name="output_face",
    )

    model.articulation(
        "lower_stage",
        ArticulationType.PRISMATIC,
        parent=lower_guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_BODY_HEIGHT + LOWER_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=250.0,
            velocity=0.45,
            lower=-LOWER_STAGE_TRAVEL,
            upper=LOWER_STAGE_TRAVEL,
        ),
    )

    model.articulation(
        "upper_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_BODY_HEIGHT + UPPER_RAIL_HEIGHT)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=-UPPER_STAGE_TRAVEL,
            upper=UPPER_STAGE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_guide = object_model.get_part("lower_guide")
    carriage = object_model.get_part("carriage")
    cross_slide = object_model.get_part("cross_slide")
    lower_stage = object_model.get_articulation("lower_stage")
    upper_stage = object_model.get_articulation("upper_stage")

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
        "lower_stage_axis_is_x",
        tuple(lower_stage.axis) == (1.0, 0.0, 0.0),
        f"lower stage axis was {lower_stage.axis}",
    )
    ctx.check(
        "upper_stage_axis_is_y",
        tuple(upper_stage.axis) == (0.0, 1.0, 0.0),
        f"upper stage axis was {upper_stage.axis}",
    )

    ctx.expect_contact(
        carriage,
        lower_guide,
        contact_tol=5e-4,
        name="carriage_supported_by_lower_guide_at_rest",
    )
    ctx.expect_overlap(
        carriage,
        lower_guide,
        axes="xy",
        min_overlap=0.1,
        name="carriage_footprint_supported_on_lower_guide",
    )
    ctx.expect_contact(
        cross_slide,
        carriage,
        contact_tol=5e-4,
        name="cross_slide_supported_by_carriage_at_rest",
    )
    ctx.expect_overlap(
        cross_slide,
        carriage,
        axes="xy",
        min_overlap=0.1,
        name="cross_slide_footprint_supported_on_carriage",
    )

    output_face_aabb = ctx.part_element_world_aabb(cross_slide, elem="output_face")
    if output_face_aabb is not None:
        min_corner, max_corner = output_face_aabb
        output_dx = max_corner[0] - min_corner[0]
        output_dz = max_corner[2] - min_corner[2]
        ctx.check(
            "output_face_reads_square",
            0.11 <= output_dx <= 0.13 and abs(output_dx - output_dz) <= 0.005,
            f"output face extents were dx={output_dx:.4f}, dz={output_dz:.4f}",
        )
    else:
        ctx.fail("output_face_reads_square", "output_face visual AABB was unavailable")

    with ctx.pose({lower_stage: LOWER_STAGE_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            lower_guide,
            axis="x",
            min_gap=LOWER_STAGE_TRAVEL - 0.005,
            max_gap=LOWER_STAGE_TRAVEL + 0.005,
            name="lower_stage_translates_on_x",
        )
        ctx.expect_contact(
            carriage,
            lower_guide,
            contact_tol=5e-4,
            name="lower_stage_stays_supported_at_positive_limit",
        )

    with ctx.pose({upper_stage: UPPER_STAGE_TRAVEL}):
        ctx.expect_origin_gap(
            cross_slide,
            carriage,
            axis="y",
            min_gap=UPPER_STAGE_TRAVEL - 0.005,
            max_gap=UPPER_STAGE_TRAVEL + 0.005,
            name="upper_stage_translates_on_y",
        )
        ctx.expect_contact(
            cross_slide,
            carriage,
            contact_tol=5e-4,
            name="upper_stage_stays_supported_at_positive_limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
