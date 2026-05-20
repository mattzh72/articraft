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


GUIDE_LENGTH = 0.92
GUIDE_BASE_WIDTH = 0.26
GUIDE_BASE_THICKNESS = 0.045

RAIL_STEM_LENGTH = 0.84
RAIL_STEM_WIDTH = 0.106
RAIL_STEM_HEIGHT = 0.055

RAIL_HEAD_LENGTH = 0.82
RAIL_HEAD_WIDTH = 0.180
RAIL_HEAD_THICKNESS = 0.040

SADDLE_LENGTH = 0.42
SADDLE_WIDTH = 0.46
SADDLE_BOTTOM_Z = GUIDE_BASE_THICKNESS + 0.003
SADDLE_DECK_THICKNESS = 0.080
SADDLE_CHANNEL_TOP_Z = 0.095
SADDLE_CHEEK_HEIGHT = SADDLE_CHANNEL_TOP_Z + 0.001
SADDLE_CHANNEL_WIDTH = 0.186
SADDLE_SLOT_WIDTH = 0.114
SADDLE_GIB_HEIGHT = GUIDE_BASE_THICKNESS + RAIL_STEM_HEIGHT - SADDLE_BOTTOM_Z

SLIDE_TRAVEL = 0.18


SADDLE_CHEEK_WIDTH = (SADDLE_WIDTH - SADDLE_CHANNEL_WIDTH) / 2.0
SADDLE_GIB_WIDTH = (SADDLE_CHANNEL_WIDTH - SADDLE_SLOT_WIDTH) / 2.0
SADDLE_CHEEK_CENTER_Y = (SADDLE_CHANNEL_WIDTH / 2.0) + (SADDLE_CHEEK_WIDTH / 2.0)
SADDLE_GIB_CENTER_Y = (SADDLE_SLOT_WIDTH / 2.0) + (SADDLE_GIB_WIDTH / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saddle_type_carriage_slide")
    model.material("guide_cast_iron", rgba=(0.38, 0.40, 0.42, 1.0))
    model.material("rail_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    model.material("saddle_paint", rgba=(0.30, 0.46, 0.56, 1.0))

    guide = model.part("lower_guide")
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_BASE_WIDTH, GUIDE_BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_BASE_THICKNESS / 2.0)),
        material="guide_cast_iron",
        name="guide_base",
    )
    guide.visual(
        Box((RAIL_STEM_LENGTH, RAIL_STEM_WIDTH, RAIL_STEM_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, GUIDE_BASE_THICKNESS + (RAIL_STEM_HEIGHT / 2.0))
        ),
        material="rail_steel",
        name="guide_stem",
    )
    guide.visual(
        Box((RAIL_HEAD_LENGTH, RAIL_HEAD_WIDTH, RAIL_HEAD_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                GUIDE_BASE_THICKNESS + RAIL_STEM_HEIGHT + (RAIL_HEAD_THICKNESS / 2.0),
            )
        ),
        material="rail_steel",
        name="guide_rail",
    )
    guide.visual(
        Box((0.05, GUIDE_BASE_WIDTH * 0.82, 0.03)),
        origin=Origin(xyz=(-(GUIDE_LENGTH / 2.0) + 0.055, 0.0, GUIDE_BASE_THICKNESS + 0.015)),
        material="guide_cast_iron",
        name="left_end_stop",
    )
    guide.visual(
        Box((0.05, GUIDE_BASE_WIDTH * 0.82, 0.03)),
        origin=Origin(xyz=((GUIDE_LENGTH / 2.0) - 0.055, 0.0, GUIDE_BASE_THICKNESS + 0.015)),
        material="guide_cast_iron",
        name="right_end_stop",
    )

    saddle = model.part("moving_saddle")
    saddle.visual(
        Box((SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_DECK_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, SADDLE_CHANNEL_TOP_Z + (SADDLE_DECK_THICKNESS / 2.0))
        ),
        material="saddle_paint",
        name="saddle_bridge",
    )
    saddle.visual(
        Box((SADDLE_LENGTH, SADDLE_CHEEK_WIDTH, SADDLE_CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -SADDLE_CHEEK_CENTER_Y, SADDLE_CHEEK_HEIGHT / 2.0)
        ),
        material="saddle_paint",
        name="left_cheek",
    )
    saddle.visual(
        Box((SADDLE_LENGTH, SADDLE_CHEEK_WIDTH, SADDLE_CHEEK_HEIGHT)),
        origin=Origin(
            xyz=(0.0, SADDLE_CHEEK_CENTER_Y, SADDLE_CHEEK_HEIGHT / 2.0)
        ),
        material="saddle_paint",
        name="right_cheek",
    )
    saddle.visual(
        Box((SADDLE_LENGTH, SADDLE_GIB_WIDTH, SADDLE_GIB_HEIGHT)),
        origin=Origin(xyz=(0.0, -SADDLE_GIB_CENTER_Y, SADDLE_GIB_HEIGHT / 2.0)),
        material="saddle_paint",
        name="left_gib",
    )
    saddle.visual(
        Box((SADDLE_LENGTH, SADDLE_GIB_WIDTH, SADDLE_GIB_HEIGHT)),
        origin=Origin(xyz=(0.0, SADDLE_GIB_CENTER_Y, SADDLE_GIB_HEIGHT / 2.0)),
        material="saddle_paint",
        name="right_gib",
    )
    saddle.visual(
        Box((0.18, 0.18, 0.018)),
        origin=Origin(
            xyz=(0.0, 0.0, SADDLE_CHANNEL_TOP_Z + SADDLE_DECK_THICKNESS - 0.009)
        ),
        material="saddle_paint",
        name="top_pad",
    )

    model.articulation(
        "guide_to_saddle",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, SADDLE_BOTTOM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.25,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("lower_guide")
    saddle = object_model.get_part("moving_saddle")
    slide = object_model.get_articulation("guide_to_saddle")
    guide_base = guide.get_visual("guide_base")
    guide_rail = guide.get_visual("guide_rail")
    saddle_bridge = saddle.get_visual("saddle_bridge")
    left_gib = saddle.get_visual("left_gib")
    right_gib = saddle.get_visual("right_gib")

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
        "slide articulation is prismatic along guide axis",
        slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(float(v) for v in slide.axis) == (1.0, 0.0, 0.0),
        details=(
            f"expected prismatic axis (1, 0, 0), got "
            f"{slide.articulation_type!r} with axis {slide.axis!r}"
        ),
    )

    rest_pos = ctx.part_world_position(saddle)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        upper_pos = ctx.part_world_position(saddle)
    ctx.check(
        "upper pose moves saddle along +X only",
        rest_pos is not None
        and upper_pos is not None
        and upper_pos[0] > rest_pos[0] + 0.15
        and abs(upper_pos[1] - rest_pos[1]) < 1e-6
        and abs(upper_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, upper={upper_pos}",
    )

    ctx.expect_contact(
        guide,
        saddle,
        elem_a=guide_rail,
        elem_b=left_gib,
        contact_tol=1e-6,
        name="rail underside supports left gib in rest pose",
    )
    ctx.expect_contact(
        guide,
        saddle,
        elem_a=guide_rail,
        elem_b=right_gib,
        contact_tol=1e-6,
        name="rail underside supports right gib in rest pose",
    )
    ctx.expect_gap(
        saddle,
        guide,
        axis="z",
        positive_elem=saddle_bridge,
        negative_elem=guide_rail,
        min_gap=0.001,
        max_gap=0.004,
        name="saddle bridge clears the rail top slightly",
    )
    ctx.expect_within(
        guide,
        saddle,
        axes="y",
        inner_elem=guide_rail,
        outer_elem=saddle_bridge,
        margin=0.0,
        name="broad saddle spans wider than the rail",
    )

    with ctx.pose({slide: -SLIDE_TRAVEL}):
        ctx.expect_contact(
            guide,
            saddle,
            elem_a=guide_rail,
            elem_b=left_gib,
            contact_tol=1e-6,
            name="left gib remains supported at lower travel limit",
        )
        ctx.expect_contact(
            guide,
            saddle,
            elem_a=guide_rail,
            elem_b=right_gib,
            contact_tol=1e-6,
            name="right gib remains supported at lower travel limit",
        )
        ctx.expect_overlap(
            guide,
            saddle,
            axes="x",
            elem_a=guide_rail,
            elem_b=saddle_bridge,
            min_overlap=0.40,
            name="lower travel limit keeps the saddle well engaged on the rail",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            guide,
            saddle,
            elem_a=guide_rail,
            elem_b=left_gib,
            contact_tol=1e-6,
            name="left gib remains supported at upper travel limit",
        )
        ctx.expect_contact(
            guide,
            saddle,
            elem_a=guide_rail,
            elem_b=right_gib,
            contact_tol=1e-6,
            name="right gib remains supported at upper travel limit",
        )
        ctx.expect_overlap(
            guide,
            saddle,
            axes="x",
            elem_a=guide_rail,
            elem_b=saddle_bridge,
            min_overlap=0.40,
            name="upper travel limit keeps the saddle well engaged on the rail",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
