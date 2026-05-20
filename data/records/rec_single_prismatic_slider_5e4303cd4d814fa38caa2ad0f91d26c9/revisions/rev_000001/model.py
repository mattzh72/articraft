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
)


RAIL_LENGTH = 0.42
RAIL_BODY_WIDTH = 0.074
RAIL_PAD_WIDTH = 0.09
RAIL_PAD_LENGTH = 0.076
RAIL_BASE_HEIGHT = 0.012
GUIDE_LENGTH = 0.35
GUIDE_FACE_THICKNESS = 0.004
GUIDE_WIDTH = 0.038
GUIDE_CORE_WIDTH = GUIDE_WIDTH - 2.0 * GUIDE_FACE_THICKNESS
GUIDE_HEIGHT = 0.018
GUIDE_FACE_HEIGHT = 0.014
GUIDE_FACE_CENTER_Z = 0.021
MOUNT_HOLE_DIAMETER = 0.008
MOUNT_HOLE_SPACING = 0.34

CARRIAGE_LENGTH = 0.13
CARRIAGE_WIDTH = 0.072
CARRIAGE_TOP_BRIDGE_HEIGHT = 0.018
CARRIAGE_SIDEWALL_HEIGHT = 0.024
CARRIAGE_CHANNEL_WIDTH = 0.048
CARRIAGE_TOP_PAD_LENGTH = 0.094
CARRIAGE_TOP_PAD_WIDTH = 0.070
CARRIAGE_TOP_PAD_HEIGHT = 0.010
CARRIAGE_FRAME_Z = 0.0
GUIDE_PAD_LENGTH = 0.094
GUIDE_PAD_HEIGHT = GUIDE_FACE_HEIGHT
GUIDE_PAD_CENTER_Z = GUIDE_FACE_CENTER_Z

SLIDER_TRAVEL = 0.11

GUIDE_FACE_CENTER_Y = GUIDE_WIDTH / 2.0 - GUIDE_FACE_THICKNESS / 2.0
GUIDE_PAD_THICKNESS = CARRIAGE_CHANNEL_WIDTH / 2.0 - GUIDE_WIDTH / 2.0
GUIDE_PAD_CENTER_Y = GUIDE_WIDTH / 2.0 + GUIDE_PAD_THICKNESS / 2.0
CARRIAGE_SIDEWALL_THICKNESS = (CARRIAGE_WIDTH - CARRIAGE_CHANNEL_WIDTH) / 2.0
CARRIAGE_SIDEWALL_CENTER_Y = CARRIAGE_CHANNEL_WIDTH / 2.0 + CARRIAGE_SIDEWALL_THICKNESS / 2.0
CARRIAGE_SIDEWALL_BOTTOM_Z = RAIL_BASE_HEIGHT
CARRIAGE_SIDEWALL_CENTER_Z = CARRIAGE_SIDEWALL_BOTTOM_Z + CARRIAGE_SIDEWALL_HEIGHT / 2.0
CARRIAGE_TOP_BRIDGE_BOTTOM_Z = CARRIAGE_SIDEWALL_BOTTOM_Z + CARRIAGE_SIDEWALL_HEIGHT
CARRIAGE_TOP_BRIDGE_CENTER_Z = CARRIAGE_TOP_BRIDGE_BOTTOM_Z + CARRIAGE_TOP_BRIDGE_HEIGHT / 2.0
CARRIAGE_TOP_PAD_CENTER_Z = CARRIAGE_TOP_BRIDGE_BOTTOM_Z + CARRIAGE_TOP_BRIDGE_HEIGHT + CARRIAGE_TOP_PAD_HEIGHT / 2.0
CARRIAGE_BODY_TOP_Z = CARRIAGE_TOP_BRIDGE_BOTTOM_Z + CARRIAGE_TOP_BRIDGE_HEIGHT
CARRIAGE_ENVELOPE_BOTTOM_Z = CARRIAGE_SIDEWALL_BOTTOM_Z
CARRIAGE_ENVELOPE_TOP_Z = CARRIAGE_TOP_PAD_CENTER_Z + CARRIAGE_TOP_PAD_HEIGHT / 2.0
CARRIAGE_ENVELOPE_HEIGHT = CARRIAGE_ENVELOPE_TOP_Z - CARRIAGE_ENVELOPE_BOTTOM_Z
CARRIAGE_ENVELOPE_CENTER_Z = CARRIAGE_ENVELOPE_BOTTOM_Z + CARRIAGE_ENVELOPE_HEIGHT / 2.0


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_pad_linear_slider")

    model.material("rail_finish", rgba=(0.36, 0.39, 0.42, 1.0))
    model.material("guide_face_finish", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("carriage_finish", rgba=(0.74, 0.76, 0.79, 1.0))
    model.material("pad_black", rgba=(0.10, 0.10, 0.11, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_BODY_WIDTH, RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_HEIGHT / 2.0)),
        material="rail_finish",
        name="rail_body",
    )
    rail.visual(
        Box((RAIL_PAD_LENGTH, RAIL_PAD_WIDTH, RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=(-(RAIL_LENGTH - RAIL_PAD_LENGTH) / 2.0, 0.0, RAIL_BASE_HEIGHT / 2.0)),
        material="rail_finish",
        name="mount_pad_neg_x",
    )
    rail.visual(
        Box((RAIL_PAD_LENGTH, RAIL_PAD_WIDTH, RAIL_BASE_HEIGHT)),
        origin=Origin(xyz=((RAIL_LENGTH - RAIL_PAD_LENGTH) / 2.0, 0.0, RAIL_BASE_HEIGHT / 2.0)),
        material="rail_finish",
        name="mount_pad_pos_x",
    )
    rail.visual(
        Box((GUIDE_LENGTH, GUIDE_CORE_WIDTH, GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_BASE_HEIGHT + GUIDE_HEIGHT / 2.0)),
        material="rail_finish",
        name="guide_core",
    )
    rail.visual(
        Box((GUIDE_LENGTH, GUIDE_FACE_THICKNESS, GUIDE_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, GUIDE_FACE_CENTER_Y, GUIDE_FACE_CENTER_Z)),
        material="guide_face_finish",
        name="guide_face_pos_y",
    )
    rail.visual(
        Box((GUIDE_LENGTH, GUIDE_FACE_THICKNESS, GUIDE_FACE_HEIGHT)),
        origin=Origin(xyz=(0.0, -GUIDE_FACE_CENTER_Y, GUIDE_FACE_CENTER_Z)),
        material="guide_face_finish",
        name="guide_face_neg_y",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_PAD_WIDTH, RAIL_BASE_HEIGHT + GUIDE_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, (RAIL_BASE_HEIGHT + GUIDE_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_BRIDGE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_BRIDGE_CENTER_Z)),
        material="carriage_finish",
        name="carriage_body",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_SIDEWALL_THICKNESS, CARRIAGE_SIDEWALL_HEIGHT)),
        origin=Origin(xyz=(0.0, CARRIAGE_SIDEWALL_CENTER_Y, CARRIAGE_SIDEWALL_CENTER_Z)),
        material="carriage_finish",
        name="sidewall_pos_y",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_SIDEWALL_THICKNESS, CARRIAGE_SIDEWALL_HEIGHT)),
        origin=Origin(xyz=(0.0, -CARRIAGE_SIDEWALL_CENTER_Y, CARRIAGE_SIDEWALL_CENTER_Z)),
        material="carriage_finish",
        name="sidewall_neg_y",
    )
    carriage.visual(
        Box((CARRIAGE_TOP_PAD_LENGTH, CARRIAGE_TOP_PAD_WIDTH, CARRIAGE_TOP_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_PAD_CENTER_Z)),
        material="pad_black",
        name="top_pad",
    )
    carriage.visual(
        Box((GUIDE_PAD_LENGTH, GUIDE_PAD_THICKNESS, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, GUIDE_PAD_CENTER_Y, GUIDE_PAD_CENTER_Z)),
        material="pad_black",
        name="guide_pad_pos_y",
    )
    carriage.visual(
        Box((GUIDE_PAD_LENGTH, GUIDE_PAD_THICKNESS, GUIDE_PAD_HEIGHT)),
        origin=Origin(xyz=(0.0, -GUIDE_PAD_CENTER_Y, GUIDE_PAD_CENTER_Z)),
        material="pad_black",
        name="guide_pad_neg_y",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_ENVELOPE_HEIGHT)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_ENVELOPE_CENTER_Z)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDER_TRAVEL,
            upper=SLIDER_TRAVEL,
            effort=400.0,
            velocity=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("rail_to_carriage")

    rail_body = rail.get_visual("rail_body")
    guide_face_pos = rail.get_visual("guide_face_pos_y")
    guide_face_neg = rail.get_visual("guide_face_neg_y")

    carriage_body = carriage.get_visual("carriage_body")
    top_pad = carriage.get_visual("top_pad")
    guide_pad_pos = carriage.get_visual("guide_pad_pos_y")
    guide_pad_neg = carriage.get_visual("guide_pad_neg_y")

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

    limits = slide.motion_limits
    ctx.check(
        "slider_uses_x_axis_prismatic_motion",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={slide.articulation_type} axis={slide.axis}",
    )
    ctx.check(
        "slider_travel_limits_match_rail_span",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower + SLIDER_TRAVEL) < 1e-9
        and abs(limits.upper - SLIDER_TRAVEL) < 1e-9,
        details=f"limits={limits}",
    )

    ctx.expect_contact(
        carriage,
        rail,
        elem_a=guide_pad_pos,
        elem_b=guide_face_pos,
        name="positive_side_guide_pad_contacts_rail_face",
    )
    ctx.expect_contact(
        carriage,
        rail,
        elem_a=guide_pad_neg,
        elem_b=guide_face_neg,
        name="negative_side_guide_pad_contacts_rail_face",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="xz",
        min_overlap=0.012,
        elem_a=guide_pad_pos,
        elem_b=guide_face_pos,
        name="positive_side_guide_faces_have_clear_overlap",
    )
    ctx.expect_overlap(
        carriage,
        rail,
        axes="xz",
        min_overlap=0.012,
        elem_a=guide_pad_neg,
        elem_b=guide_face_neg,
        name="negative_side_guide_faces_have_clear_overlap",
    )

    carriage_body_aabb = ctx.part_element_world_aabb(carriage, elem=carriage_body)
    top_pad_aabb = ctx.part_element_world_aabb(carriage, elem=top_pad)
    ctx.check(
        "top_pad_is_seated_on_carriage",
        carriage_body_aabb is not None
        and top_pad_aabb is not None
        and top_pad_aabb[0][2] >= carriage_body_aabb[1][2] - 1e-6,
        details=f"body_aabb={carriage_body_aabb} top_pad_aabb={top_pad_aabb}",
    )

    with ctx.pose({slide: SLIDER_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            rail,
            axis="x",
            min_gap=SLIDER_TRAVEL - 1e-6,
            max_gap=SLIDER_TRAVEL + 1e-6,
            name="carriage_reaches_positive_travel_limit",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            margin=0.0,
            name="carriage_remains_over_rail_at_positive_travel",
        )

    with ctx.pose({slide: -SLIDER_TRAVEL}):
        ctx.expect_origin_gap(
            rail,
            carriage,
            axis="x",
            min_gap=SLIDER_TRAVEL - 1e-6,
            max_gap=SLIDER_TRAVEL + 1e-6,
            name="carriage_reaches_negative_travel_limit",
        )
        ctx.expect_within(
            carriage,
            rail,
            axes="x",
            margin=0.0,
            name="carriage_remains_over_rail_at_negative_travel",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
