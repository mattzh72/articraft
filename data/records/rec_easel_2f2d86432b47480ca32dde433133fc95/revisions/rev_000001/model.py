from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TOP_Z = 1.56
FOOT_Z = 0.05
HALF_SPREAD = 0.28
BEAM_WIDTH = 0.042
BEAM_DEPTH = 0.022
BEAM_LENGTH = hypot(HALF_SPREAD, TOP_Z - FOOT_Z)
BEAM_ANGLE = atan2(HALF_SPREAD, TOP_Z - FOOT_Z)
LOWER_SLOT_Z = 0.58
LOWER_SLOT_LENGTH = 0.72
UPPER_SLOT_Z = 1.05
UPPER_SLOT_LENGTH = 0.42
REST_BAR_Z = 0.46
CLIP_BAR_Z = 1.06
SLOT_AXIS = (sin(BEAM_ANGLE), 0.0, cos(BEAM_ANGLE))
LEFT_FRONT_Y = 0.013
RIGHT_FRONT_Y = -0.013


def _beam_x_at_z(side_sign: float, z: float) -> float:
    return side_sign * HALF_SPREAD * (TOP_Z - z) / (TOP_Z - FOOT_Z)


def _beam_mount(side_sign: float, z: float, *, lateral: float = 0.0, y: float = 0.0) -> Origin:
    theta = -side_sign * BEAM_ANGLE
    x = _beam_x_at_z(side_sign, z) + lateral * cos(theta)
    z_world = z - lateral * sin(theta)
    return Origin(xyz=(x, y, z_world), rpy=(0.0, theta, 0.0))


def _beam_mount_relative(
    side_sign: float,
    z: float,
    reference_z: float,
    *,
    lateral: float = 0.0,
    y: float = 0.0,
) -> Origin:
    theta = -side_sign * BEAM_ANGLE
    x = _beam_x_at_z(side_sign, z) - _beam_x_at_z(side_sign, reference_z) + lateral * cos(theta)
    z_local = z - reference_z - lateral * sin(theta)
    return Origin(xyz=(x, y, z_local), rpy=(0.0, theta, 0.0))


def _slot_face_y() -> float:
    return BEAM_DEPTH * 0.5 - 0.001


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilting_a_frame_easel")

    beech = model.material("beech", rgba=(0.72, 0.58, 0.40, 1.0))
    walnut = model.material("walnut", rgba=(0.43, 0.30, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    brass = model.material("brass", rgba=(0.70, 0.58, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    left_upright = model.part("left_upright")
    left_upright.inertial = Inertial.from_geometry(
        Box((0.40, 0.10, 1.62)),
        mass=4.0,
        origin=Origin(xyz=(-0.10, 0.0, 0.81)),
    )
    left_upright.visual(
        Box((BEAM_WIDTH, BEAM_DEPTH, BEAM_LENGTH)),
        origin=_beam_mount(-1.0, (TOP_Z + FOOT_Z) * 0.5, y=LEFT_FRONT_Y),
        material=beech,
        name="left_beam",
    )
    left_upright.visual(
        Box((0.013, 0.002, LOWER_SLOT_LENGTH)),
        origin=_beam_mount(-1.0, LOWER_SLOT_Z, y=LEFT_FRONT_Y + _slot_face_y()),
        material=dark_steel,
        name="left_lower_slot",
    )
    left_upright.visual(
        Box((0.013, 0.002, UPPER_SLOT_LENGTH)),
        origin=_beam_mount(-1.0, UPPER_SLOT_Z, y=LEFT_FRONT_Y + _slot_face_y()),
        material=dark_steel,
        name="left_upper_slot",
    )
    left_upright.visual(
        Box((0.046, 0.026, 0.052)),
        origin=Origin(xyz=(-0.023, 0.013, TOP_Z - 0.026)),
        material=brass,
        name="left_hinge_block",
    )
    left_upright.visual(
        Box((0.050, 0.020, 0.036)),
        origin=Origin(xyz=(0.028, -0.039, 0.16)),
        material=dark_steel,
        name="rear_brace_hinge_plate",
    )
    left_upright.visual(
        Box((0.264, 0.033, 0.018)),
        origin=Origin(xyz=(-0.129, -0.0135, 0.16)),
        material=dark_steel,
        name="rear_brace_mount_rail",
    )
    left_upright.visual(
        Box((0.060, 0.028, 0.022)),
        origin=_beam_mount(-1.0, FOOT_Z + 0.020, y=LEFT_FRONT_Y),
        material=rubber,
        name="left_foot_pad",
    )

    right_upright = model.part("right_upright")
    right_upright.inertial = Inertial.from_geometry(
        Box((0.36, 0.10, 1.60)),
        mass=3.7,
        origin=Origin(xyz=(0.10, 0.0, -0.80)),
    )
    right_upright.visual(
        Box((BEAM_WIDTH, BEAM_DEPTH, BEAM_LENGTH)),
        origin=_beam_mount_relative(1.0, (TOP_Z + FOOT_Z) * 0.5, TOP_Z, y=RIGHT_FRONT_Y),
        material=beech,
        name="right_beam",
    )
    right_upright.visual(
        Box((0.013, 0.002, LOWER_SLOT_LENGTH)),
        origin=_beam_mount_relative(1.0, LOWER_SLOT_Z, TOP_Z, y=RIGHT_FRONT_Y + _slot_face_y()),
        material=dark_steel,
        name="right_lower_slot",
    )
    right_upright.visual(
        Box((0.013, 0.002, UPPER_SLOT_LENGTH)),
        origin=_beam_mount_relative(1.0, UPPER_SLOT_Z, TOP_Z, y=RIGHT_FRONT_Y + _slot_face_y()),
        material=dark_steel,
        name="right_upper_slot",
    )
    right_upright.visual(
        Box((0.046, 0.026, 0.052)),
        origin=Origin(xyz=(0.023, -0.013, -0.026)),
        material=brass,
        name="right_hinge_block",
    )
    right_upright.visual(
        Box((0.060, 0.028, 0.022)),
        origin=_beam_mount_relative(1.0, FOOT_Z + 0.020, TOP_Z, y=RIGHT_FRONT_Y),
        material=rubber,
        name="right_foot_pad",
    )

    rear_brace_leg = model.part("rear_brace_leg")
    rear_brace_leg.inertial = Inertial.from_geometry(
        Box((0.08, 0.72, 0.14)),
        mass=1.5,
        origin=Origin(xyz=(0.0, -0.36, -0.02)),
    )
    rear_brace_leg.visual(
        Box((0.046, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.001, 0.0)),
        material=dark_steel,
        name="brace_top_knuckle",
    )
    rear_brace_leg.visual(
        Box((0.020, 0.020, 0.012)),
        origin=Origin(xyz=(0.033, -0.020, -0.014)),
        material=dark_steel,
        name="brace_top_web",
    )
    rear_brace_leg.visual(
        Box((0.024, 0.020, 0.74)),
        origin=Origin(xyz=(0.042, -0.398, -0.040), rpy=(1.68, 0.0, 0.0)),
        material=beech,
        name="brace_beam",
    )
    rear_brace_leg.visual(
        Box((0.060, 0.028, 0.020)),
        origin=Origin(xyz=(0.042, -0.753, -0.080), rpy=(1.68, 0.0, 0.0)),
        material=rubber,
        name="brace_foot",
    )

    canvas_rest_bar = model.part("canvas_rest_bar")
    canvas_rest_bar.inertial = Inertial.from_geometry(
        Box((0.68, 0.06, 0.14)),
        mass=0.9,
        origin=Origin(xyz=(0.24, 0.028, -0.015)),
    )
    canvas_rest_bar.visual(
        Box((0.026, 0.004, 0.100)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="rest_shoe",
    )
    canvas_rest_bar.visual(
        Box((0.090, 0.018, 0.055)),
        origin=Origin(xyz=(0.052, 0.011, -0.026)),
        material=dark_steel,
        name="rest_bracket",
    )
    canvas_rest_bar.visual(
        Box((0.44, 0.028, 0.018)),
        origin=Origin(xyz=(0.24, 0.014, -0.040)),
        material=walnut,
        name="rest_shelf",
    )
    canvas_rest_bar.visual(
        Box((0.44, 0.012, 0.042)),
        origin=Origin(xyz=(0.24, 0.033, -0.022)),
        material=beech,
        name="rest_front_lip",
    )
    canvas_rest_bar.visual(
        Cylinder(radius=0.006, length=0.64),
        origin=Origin(xyz=(0.32, 0.016, -0.001), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="rest_cross_rod",
    )
    canvas_rest_bar.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(-0.006, 0.010, -0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="rest_thumb_knob",
    )

    upper_clip_bar = model.part("upper_clip_bar")
    upper_clip_bar.inertial = Inertial.from_geometry(
        Box((0.38, 0.06, 0.18)),
        mass=0.7,
        origin=Origin(xyz=(0.12, 0.026, -0.030)),
    )
    upper_clip_bar.visual(
        Box((0.024, 0.004, 0.084)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="clip_shoe",
    )
    upper_clip_bar.visual(
        Box((0.24, 0.024, 0.018)),
        origin=Origin(xyz=(0.12, 0.014, 0.008)),
        material=walnut,
        name="clip_crossbar",
    )
    upper_clip_bar.visual(
        Cylinder(radius=0.006, length=0.34),
        origin=Origin(xyz=(0.17, 0.014, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="clip_cross_rod",
    )
    upper_clip_bar.visual(
        Box((0.050, 0.018, 0.100)),
        origin=Origin(xyz=(0.105, 0.023, -0.050)),
        material=beech,
        name="clip_body",
    )
    upper_clip_bar.visual(
        Box((0.070, 0.014, 0.012)),
        origin=Origin(xyz=(0.105, 0.026, -0.102)),
        material=dark_steel,
        name="clip_pad",
    )
    upper_clip_bar.visual(
        Cylinder(radius=0.010, length=0.018),
        origin=Origin(xyz=(0.105, 0.038, -0.032), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="clip_knob",
    )

    model.articulation(
        "top_hinge",
        ArticulationType.REVOLUTE,
        parent=left_upright,
        child=right_upright,
        origin=Origin(xyz=(0.0, 0.0, TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.6, lower=-0.10, upper=0.18),
    )
    model.articulation(
        "rear_brace_hinge",
        ArticulationType.REVOLUTE,
        parent=left_upright,
        child=rear_brace_leg,
        origin=Origin(xyz=(0.028, -0.020, 0.16)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.8, lower=-0.08, upper=0.52),
    )
    model.articulation(
        "rest_bar_slide",
        ArticulationType.PRISMATIC,
        parent=left_upright,
        child=canvas_rest_bar,
        origin=Origin(xyz=(_beam_x_at_z(-1.0, REST_BAR_Z), 0.026, REST_BAR_Z)),
        axis=SLOT_AXIS,
        motion_limits=MotionLimits(effort=4.0, velocity=0.12, lower=-0.18, upper=0.28),
    )
    model.articulation(
        "clip_bar_slide",
        ArticulationType.PRISMATIC,
        parent=left_upright,
        child=upper_clip_bar,
        origin=Origin(xyz=(_beam_x_at_z(-1.0, CLIP_BAR_Z), 0.026, CLIP_BAR_Z)),
        axis=SLOT_AXIS,
        motion_limits=MotionLimits(effort=3.0, velocity=0.10, lower=-0.20, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    left_upright = object_model.get_part("left_upright")
    right_upright = object_model.get_part("right_upright")
    rear_brace_leg = object_model.get_part("rear_brace_leg")
    canvas_rest_bar = object_model.get_part("canvas_rest_bar")
    upper_clip_bar = object_model.get_part("upper_clip_bar")

    top_hinge = object_model.get_articulation("top_hinge")
    rear_brace_hinge = object_model.get_articulation("rear_brace_hinge")
    rest_bar_slide = object_model.get_articulation("rest_bar_slide")
    clip_bar_slide = object_model.get_articulation("clip_bar_slide")

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
        "top_hinge_axis_is_front_to_back",
        tuple(top_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"Unexpected top hinge axis: {top_hinge.axis}",
    )
    ctx.check(
        "rear_brace_axis_is_left_to_right",
        tuple(rear_brace_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"Unexpected rear brace hinge axis: {rear_brace_hinge.axis}",
    )
    ctx.check(
        "rest_slide_axis_follows_upright",
        abs(rest_bar_slide.axis[0] - SLOT_AXIS[0]) < 1e-6
        and abs(rest_bar_slide.axis[1]) < 1e-9
        and abs(rest_bar_slide.axis[2] - SLOT_AXIS[2]) < 1e-6,
        details=f"Unexpected rest slide axis: {rest_bar_slide.axis}",
    )
    ctx.check(
        "clip_slide_axis_follows_upright",
        abs(clip_bar_slide.axis[0] - SLOT_AXIS[0]) < 1e-6
        and abs(clip_bar_slide.axis[1]) < 1e-9
        and abs(clip_bar_slide.axis[2] - SLOT_AXIS[2]) < 1e-6,
        details=f"Unexpected clip slide axis: {clip_bar_slide.axis}",
    )

    ctx.expect_contact(
        left_upright,
        right_upright,
        elem_a="left_hinge_block",
        elem_b="right_hinge_block",
        name="top_hinge_blocks_touch",
    )
    ctx.expect_contact(
        rear_brace_leg,
        left_upright,
        elem_a="brace_top_knuckle",
        elem_b="rear_brace_hinge_plate",
        name="rear_brace_is_mounted",
    )
    ctx.expect_contact(
        canvas_rest_bar,
        left_upright,
        elem_a="rest_shoe",
        elem_b="left_lower_slot",
        name="rest_bar_carriage_is_mounted",
    )
    ctx.expect_contact(
        upper_clip_bar,
        left_upright,
        elem_a="clip_shoe",
        elem_b="left_upper_slot",
        name="clip_bar_carriage_is_mounted",
    )
    ctx.expect_origin_gap(
        upper_clip_bar,
        canvas_rest_bar,
        axis="z",
        min_gap=0.45,
        name="upper_clip_sits_above_canvas_rest",
    )

    left_aabb = ctx.part_world_aabb(left_upright)
    right_aabb = ctx.part_world_aabb(right_upright)
    rear_aabb = ctx.part_world_aabb(rear_brace_leg)
    left_foot_aabb = ctx.part_element_world_aabb(left_upright, elem="left_foot_pad")
    right_foot_aabb = ctx.part_element_world_aabb(right_upright, elem="right_foot_pad")
    ctx.check(
        "front_uprights_form_readable_a_frame",
        left_foot_aabb is not None
        and right_foot_aabb is not None
        and left_foot_aabb[1][0] < -0.24
        and right_foot_aabb[0][0] > 0.24,
        details=f"Unexpected front foot spread: left_foot={left_foot_aabb}, right_foot={right_foot_aabb}, left={left_aabb}, right={right_aabb}",
    )
    ctx.check(
        "rear_brace_leg_stands_behind_front_frame",
        rear_aabb is not None and rear_aabb[0][1] < -0.45,
        details=f"Rear brace leg does not project behind the frame enough: {rear_aabb}",
    )

    rest_pos = ctx.part_world_position(canvas_rest_bar)
    clip_pos = ctx.part_world_position(upper_clip_bar)
    assert rest_pos is not None
    assert clip_pos is not None

    with ctx.pose({rest_bar_slide: 0.24}):
        rest_high = ctx.part_world_position(canvas_rest_bar)
        assert rest_high is not None
        ctx.check(
            "rest_bar_moves_up_slot",
            rest_high[2] > rest_pos[2] + 0.20 and rest_high[0] > rest_pos[0] + 0.03,
            details=f"Rest bar did not translate upward along the slotted upright: rest={rest_pos}, high={rest_high}",
        )
        ctx.expect_contact(
            canvas_rest_bar,
            left_upright,
            elem_a="rest_shoe",
            elem_b="left_lower_slot",
            name="rest_bar_stays_mounted_when_raised",
        )

    with ctx.pose({clip_bar_slide: -0.16}):
        clip_low = ctx.part_world_position(upper_clip_bar)
        assert clip_low is not None
        ctx.check(
            "clip_bar_moves_down_slot",
            clip_low[2] < clip_pos[2] - 0.12 and clip_low[0] < clip_pos[0] - 0.02,
            details=f"Clip bar did not translate downward along the slotted upright: rest={clip_pos}, low={clip_low}",
        )
        ctx.expect_contact(
            upper_clip_bar,
            left_upright,
            elem_a="clip_shoe",
            elem_b="left_upper_slot",
            name="clip_bar_stays_mounted_when_lowered",
        )

    right_foot_rest = ctx.part_element_world_aabb(right_upright, elem="right_foot_pad")
    assert right_foot_rest is not None
    with ctx.pose({top_hinge: -0.10}):
        right_foot_open = ctx.part_element_world_aabb(right_upright, elem="right_foot_pad")
        assert right_foot_open is not None
        ctx.check(
            "top_hinge_opens_right_upright_outward",
            right_foot_open[1][0] > right_foot_rest[1][0] + 0.08,
            details=f"Right foot did not swing outward from the top hinge: rest={right_foot_rest}, open={right_foot_open}",
        )

    rear_foot_rest = ctx.part_element_world_aabb(rear_brace_leg, elem="brace_foot")
    assert rear_foot_rest is not None
    with ctx.pose({rear_brace_hinge: 0.40}):
        rear_folded_foot = ctx.part_element_world_aabb(rear_brace_leg, elem="brace_foot")
        assert rear_folded_foot is not None
        ctx.check(
            "rear_brace_leg_folds_toward_front_frame",
            rear_folded_foot[0][1] > rear_foot_rest[0][1] + 0.08,
            details=f"Rear brace leg did not fold forward enough: rest={rear_foot_rest}, folded={rear_folded_foot}",
        )
        ctx.expect_contact(
            rear_brace_leg,
            left_upright,
            elem_a="brace_top_knuckle",
            elem_b="rear_brace_hinge_plate",
            name="rear_brace_stays_hinged_when_folded",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
