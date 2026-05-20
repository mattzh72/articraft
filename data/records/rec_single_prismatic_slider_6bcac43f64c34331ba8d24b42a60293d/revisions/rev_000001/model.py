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


PLATE_LENGTH = 0.24
PLATE_WIDTH = 0.12
PLATE_THICKNESS = 0.018
PLATE_CORNER_RADIUS = 0.008

RAIL_LENGTH = 0.16
RAIL_BASE_WIDTH = 0.038
RAIL_TOP_WIDTH = 0.026
RAIL_HEIGHT = 0.014

CLAMP_LENGTH = 0.06
CLAMP_WIDTH = 0.076
CLAMP_HEIGHT = 0.04
CLAMP_SLOT_WIDTH = 0.042
CLAMP_SLOT_DEPTH = 0.015

PAD_LENGTH = 0.034
PAD_WIDTH = 0.03
PAD_THICKNESS = 0.006
PAD_EMBED = 0.0006

SLIDE_TRAVEL = 0.045


def make_base_plate() -> cq.Workplane:
    slot_offset = 0.075
    slot_length = 0.034
    slot_width = 0.009

    return (
        cq.Workplane("XY")
        .box(
            PLATE_LENGTH,
            PLATE_WIDTH,
            PLATE_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(PLATE_CORNER_RADIUS)
        .faces(">Z")
        .workplane(centerOption="CenterOfBoundBox")
        .pushPoints([(-slot_offset, 0.0), (slot_offset, 0.0)])
        .slot2D(slot_length, slot_width, angle=0)
        .cutThruAll()
    )


def make_guide_rail() -> cq.Workplane:
    rail_profile = [
        (-RAIL_BASE_WIDTH / 2.0, 0.0),
        (RAIL_BASE_WIDTH / 2.0, 0.0),
        (RAIL_TOP_WIDTH / 2.0, RAIL_HEIGHT),
        (-RAIL_TOP_WIDTH / 2.0, RAIL_HEIGHT),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(rail_profile)
        .close()
        .extrude(RAIL_LENGTH, both=True)
        .translate((0.0, 0.0, PLATE_THICKNESS))
    )


def make_clamp_block() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(
            CLAMP_LENGTH,
            CLAMP_WIDTH,
            CLAMP_HEIGHT,
            centered=(True, True, False),
        )
    )

    underside_slot = (
        cq.Workplane("XY")
        .box(
            CLAMP_LENGTH + 0.002,
            CLAMP_SLOT_WIDTH,
            CLAMP_SLOT_DEPTH,
            centered=(True, True, False),
        )
    )

    top_relief = (
        cq.Workplane("XY")
        .box(
            CLAMP_LENGTH * 0.72,
            CLAMP_WIDTH * 0.42,
            0.004,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CLAMP_HEIGHT - 0.004))
    )

    return (
        body.cut(underside_slot)
        .cut(top_relief)
        .edges("|Z")
        .fillet(0.003)
        .faces(">Z")
        .edges()
        .fillet(0.0015)
    )


def make_top_pad() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(
            PAD_LENGTH,
            PAD_WIDTH,
            PAD_THICKNESS,
            centered=(True, True, False),
        )
        .edges("|Z")
        .fillet(0.0012)
        .translate((0.0, 0.0, CLAMP_HEIGHT - CLAMP_SLOT_DEPTH / 4.0 - PAD_EMBED))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_adjustment_module")

    anodized_aluminum = model.material(
        "anodized_aluminum",
        rgba=(0.72, 0.74, 0.77, 1.0),
    )
    rail_steel = model.material(
        "rail_steel",
        rgba=(0.55, 0.58, 0.61, 1.0),
    )
    clamp_finish = model.material(
        "clamp_finish",
        rgba=(0.22, 0.24, 0.28, 1.0),
    )
    rubber_pad = model.material(
        "rubber_pad",
        rgba=(0.11, 0.12, 0.13, 1.0),
    )

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_plate(), "base_plate"),
        material=anodized_aluminum,
        name="base_plate",
    )
    base.visual(
        mesh_from_cadquery(make_guide_rail(), "guide_rail"),
        material=rail_steel,
        name="guide_rail",
    )

    clamp_block = model.part("clamp_block")
    clamp_block.visual(
        mesh_from_cadquery(make_clamp_block(), "clamp_block"),
        material=clamp_finish,
        name="clamp_body",
    )
    clamp_block.visual(
        mesh_from_cadquery(make_top_pad(), "top_pad"),
        material=rubber_pad,
        name="top_pad",
    )

    model.articulation(
        "base_to_clamp_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=clamp_block,
        origin=Origin(xyz=(0.0, 0.0, PLATE_THICKNESS)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.08,
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    clamp_block = object_model.get_part("clamp_block")
    slide = object_model.get_articulation("base_to_clamp_slide")

    part_names = {part.name for part in object_model.parts}
    base_visual_names = {visual.name for visual in base.visuals}
    clamp_visual_names = {visual.name for visual in clamp_block.visuals}

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
        "expected_parts_present",
        part_names == {"base", "clamp_block"},
        details=f"found parts: {sorted(part_names)}",
    )
    ctx.check(
        "expected_visuals_present",
        base_visual_names == {"base_plate", "guide_rail"}
        and clamp_visual_names == {"clamp_body", "top_pad"},
        details=(
            f"base visuals={sorted(base_visual_names)}, "
            f"clamp visuals={sorted(clamp_visual_names)}"
        ),
    )
    ctx.check(
        "prismatic_slide_axis_is_x",
        slide.joint_type == ArticulationType.PRISMATIC and slide.axis == (1.0, 0.0, 0.0),
        details=f"joint_type={slide.joint_type}, axis={slide.axis}",
    )

    ctx.expect_contact(
        clamp_block,
        base,
        elem_b="base_plate",
        contact_tol=0.0005,
        name="clamp_block_is_supported_by_base_plate",
    )
    ctx.expect_overlap(
        clamp_block,
        base,
        axes="y",
        elem_b="guide_rail",
        min_overlap=0.02,
        name="clamp_block_tracks_over_rail_width",
    )

    lower = slide.motion_limits.lower
    upper = slide.motion_limits.upper
    assert lower is not None and upper is not None

    with ctx.pose({slide: lower}):
        lower_pos = ctx.part_world_position(clamp_block)
        ctx.expect_within(
            clamp_block,
            base,
            axes="x",
            margin=0.0,
            name="clamp_stays_on_base_at_lower_limit",
        )

    with ctx.pose({slide: upper}):
        upper_pos = ctx.part_world_position(clamp_block)
        ctx.expect_within(
            clamp_block,
            base,
            axes="x",
            margin=0.0,
            name="clamp_stays_on_base_at_upper_limit",
        )

    if lower_pos is None or upper_pos is None:
        ctx.fail("clamp_positions_resolve", "could not resolve clamp block world positions")
    else:
        travel = upper_pos[0] - lower_pos[0]
        lateral_drift = max(
            abs(upper_pos[1] - lower_pos[1]),
            abs(upper_pos[2] - lower_pos[2]),
        )
        ctx.check(
            "slide_motion_is_single_axis",
            abs(travel - (upper - lower)) <= 1e-6 and lateral_drift <= 1e-6,
            details=(
                f"travel={travel:.6f}, expected={upper - lower:.6f}, "
                f"lateral_drift={lateral_drift:.6f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
