from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_mini_easel")

    wood = model.material("warm_birch", rgba=(0.78, 0.55, 0.32, 1.0))
    end_grain = model.material("end_grain", rgba=(0.62, 0.42, 0.23, 1.0))
    dark = model.material("dark_recess", rgba=(0.03, 0.025, 0.02, 1.0))
    steel = model.material("spring_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    brass = model.material("brass_pin", rgba=(0.86, 0.64, 0.24, 1.0))
    rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    # Tabletop proportions: about 0.38 m high, 0.28 m wide and 0.25 m deep.
    apex = (0.0, 0.0, 0.0)
    left_foot = (-0.115, 0.0, -0.360)
    right_foot = (0.115, 0.0, -0.360)
    leg_len = math.sqrt((left_foot[0] - apex[0]) ** 2 + (left_foot[2] - apex[2]) ** 2)
    left_pitch = math.atan2(left_foot[0] - apex[0], left_foot[2] - apex[2])
    right_pitch = math.atan2(right_foot[0] - apex[0], right_foot[2] - apex[2])
    left_mid = ((apex[0] + left_foot[0]) * 0.5, -0.007, (apex[2] + left_foot[2]) * 0.5)
    right_mid = ((apex[0] + right_foot[0]) * 0.5, 0.007, (apex[2] + right_foot[2]) * 0.5)

    front_leg_0 = model.part("front_leg_0")
    front_leg_0.visual(
        Box((0.024, 0.014, leg_len)),
        origin=Origin(xyz=left_mid, rpy=(0.0, left_pitch, 0.0)),
        material=wood,
        name="left_leg_rail",
    )
    # A shallow base board / canvas ledge ties the root leg to the rear prop hinge.
    front_leg_0.visual(
        Box((0.095, 0.088, 0.018)),
        origin=Origin(xyz=(0.0, -0.004, -0.358)),
        material=wood,
        name="base_board",
    )
    front_leg_0.visual(
        Box((0.285, 0.024, 0.018)),
        origin=Origin(xyz=(0.0, -0.025, -0.358)),
        material=wood,
        name="front_base_rail",
    )
    front_leg_0.visual(
        Box((0.305, 0.026, 0.030)),
        origin=Origin(xyz=(0.0, -0.047, -0.336)),
        material=wood,
        name="canvas_ledge",
    )
    front_leg_0.visual(
        Box((0.305, 0.010, 0.022)),
        origin=Origin(xyz=(0.0, -0.058, -0.315)),
        material=end_grain,
        name="front_lip",
    )
    front_leg_0.visual(
        Cylinder(radius=0.007, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="apex_pin",
    )
    front_leg_0.visual(
        Box((0.032, 0.014, 0.065)),
        origin=Origin(xyz=(-0.010, -0.009, -0.030), rpy=(0.0, left_pitch, 0.0)),
        material=wood,
        name="apex_leaf",
    )
    # Visible slotted track on the front face of the left leg.
    slot_center_t = 0.55
    slot_center = (
        left_foot[0] * slot_center_t,
        -0.015,
        left_foot[2] * slot_center_t,
    )
    front_leg_0.visual(
        Box((0.014, 0.003, 0.205)),
        origin=Origin(xyz=slot_center, rpy=(0.0, left_pitch, 0.0)),
        material=dark,
        name="slot_track",
    )
    # Rear yoke for the fold-out prop strut.
    front_leg_0.visual(
        Box((0.094, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.020, -0.349)),
        material=wood,
        name="rear_hinge_block",
    )
    front_leg_0.visual(
        Box((0.014, 0.032, 0.040)),
        origin=Origin(xyz=(-0.034, 0.044, -0.344)),
        material=wood,
        name="rear_yoke_0",
    )
    front_leg_0.visual(
        Box((0.014, 0.032, 0.040)),
        origin=Origin(xyz=(0.034, 0.044, -0.344)),
        material=wood,
        name="rear_yoke_1",
    )
    front_leg_0.visual(
        Box((0.060, 0.024, 0.016)),
        origin=Origin(xyz=(-0.115, -0.007, -0.363)),
        material=rubber,
        name="foot_pad_0",
    )

    front_leg_1 = model.part("front_leg_1")
    front_leg_1.visual(
        Box((0.024, 0.014, leg_len)),
        origin=Origin(xyz=right_mid, rpy=(0.0, right_pitch, 0.0)),
        material=wood,
        name="right_leg_rail",
    )
    front_leg_1.visual(
        Box((0.040, 0.020, 0.030)),
        origin=Origin(xyz=(0.006, 0.010, -0.013), rpy=(0.0, right_pitch, 0.0)),
        material=wood,
        name="right_hinge_leaf",
    )
    front_leg_1.visual(
        Box((0.060, 0.024, 0.016)),
        origin=Origin(xyz=(0.115, 0.007, -0.363)),
        material=rubber,
        name="foot_pad_1",
    )
    model.articulation(
        "apex_hinge",
        ArticulationType.REVOLUTE,
        parent=front_leg_0,
        child=front_leg_1,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.8, lower=-0.10, upper=0.30),
    )

    prop_strut = model.part("prop_strut")
    prop_len = math.sqrt(0.225**2 + 0.070**2)
    prop_roll = math.atan2(-0.070, 0.225)
    prop_strut.visual(
        Cylinder(radius=0.0085, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="prop_hinge_barrel",
    )
    prop_strut.visual(
        Box((0.020, prop_len, 0.016)),
        origin=Origin(xyz=(0.0, 0.1125, -0.035), rpy=(prop_roll, 0.0, 0.0)),
        material=wood,
        name="rear_prop_rail",
    )
    prop_strut.visual(
        Box((0.065, 0.032, 0.012)),
        origin=Origin(xyz=(0.0, 0.226, -0.074)),
        material=rubber,
        name="rear_foot_pad",
    )
    model.articulation(
        "base_prop_hinge",
        ArticulationType.REVOLUTE,
        parent=front_leg_0,
        child=prop_strut,
        origin=Origin(xyz=(0.0, 0.044, -0.344)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=-0.35, upper=1.10),
    )

    # Sliding spring-clip carrier.  Its origin starts near the lower end of the
    # slot and translates toward the apex along the leg face.
    slide_axis = (0.304, 0.0, 0.953)
    slide_origin = (left_foot[0] * 0.70, -0.019, left_foot[2] * 0.70)
    clip_carriage = model.part("clip_carriage")
    clip_carriage.visual(
        Box((0.014, 0.008, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, left_pitch, 0.0)),
        material=steel,
        name="carriage_tongue",
    )
    clip_carriage.visual(
        Box((0.078, 0.008, 0.050)),
        origin=Origin(xyz=(0.0, -0.006, 0.0), rpy=(0.0, left_pitch, 0.0)),
        material=steel,
        name="clip_backplate",
    )
    clip_carriage.visual(
        Box((0.115, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, -0.020)),
        material=steel,
        name="fixed_canvas_lip",
    )
    clip_carriage.visual(
        Box((0.010, 0.018, 0.060)),
        origin=Origin(xyz=(-0.036, -0.020, 0.008)),
        material=steel,
        name="clip_ear_0",
    )
    clip_carriage.visual(
        Box((0.010, 0.018, 0.060)),
        origin=Origin(xyz=(0.036, -0.020, 0.008)),
        material=steel,
        name="clip_ear_1",
    )
    clip_carriage.visual(
        Cylinder(radius=0.005, length=0.080),
        origin=Origin(xyz=(0.0, -0.024, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="clip_hinge_pin",
    )
    clip_carriage.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.036, -0.024, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spring_coil",
    )
    model.articulation(
        "clip_slide",
        ArticulationType.PRISMATIC,
        parent=front_leg_0,
        child=clip_carriage,
        origin=Origin(xyz=slide_origin),
        axis=slide_axis,
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.110),
    )

    clip_jaw = model.part("clip_jaw")
    clip_jaw.visual(
        Cylinder(radius=0.0042, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="jaw_barrel",
    )
    clip_jaw.visual(
        Box((0.058, 0.012, 0.038)),
        origin=Origin(xyz=(0.0, -0.007, -0.020)),
        material=steel,
        name="jaw_plate",
    )
    clip_jaw.visual(
        Box((0.060, 0.008, 0.007)),
        origin=Origin(xyz=(0.0, -0.012, -0.041)),
        material=rubber,
        name="jaw_pad",
    )
    clip_jaw.visual(
        Box((0.040, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.006, 0.009)),
        material=steel,
        name="thumb_tab",
    )
    model.articulation(
        "clip_jaw_hinge",
        ArticulationType.REVOLUTE,
        parent=clip_carriage,
        child=clip_jaw,
        origin=Origin(xyz=(0.0, -0.024, 0.026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_leg_0 = object_model.get_part("front_leg_0")
    front_leg_1 = object_model.get_part("front_leg_1")
    prop_strut = object_model.get_part("prop_strut")
    clip_carriage = object_model.get_part("clip_carriage")
    clip_jaw = object_model.get_part("clip_jaw")

    apex_hinge = object_model.get_articulation("apex_hinge")
    prop_hinge = object_model.get_articulation("base_prop_hinge")
    clip_slide = object_model.get_articulation("clip_slide")
    jaw_hinge = object_model.get_articulation("clip_jaw_hinge")

    ctx.check(
        "required mechanisms are articulated",
        apex_hinge.articulation_type == ArticulationType.REVOLUTE
        and prop_hinge.articulation_type == ArticulationType.REVOLUTE
        and clip_slide.articulation_type == ArticulationType.PRISMATIC
        and jaw_hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Expected apex and prop revolute hinges, a prismatic clip slot, and a revolute spring jaw.",
    )

    # Intentional local captured-hinge overlaps: the authored pins occupy the
    # hinge barrels/leaves just as the real metal pins would.
    ctx.allow_overlap(
        front_leg_0,
        front_leg_1,
        elem_a="apex_pin",
        elem_b="right_hinge_leaf",
        reason="The brass apex pin is intentionally captured through the right hinge leaf.",
    )
    ctx.allow_overlap(
        front_leg_0,
        front_leg_1,
        elem_a="apex_pin",
        elem_b="right_leg_rail",
        reason="The same apex hinge pin locally passes through the upper end of the moving front leg rail.",
    )
    ctx.expect_overlap(
        front_leg_0,
        front_leg_1,
        elem_a="apex_pin",
        elem_b="right_hinge_leaf",
        axes="y",
        min_overlap=0.010,
        name="apex pin passes through the right hinge leaf",
    )
    ctx.expect_overlap(
        front_leg_0,
        front_leg_1,
        elem_a="apex_pin",
        elem_b="right_hinge_leaf",
        axes="xz",
        min_overlap=0.004,
        name="apex pin is centered in the hinge leaf",
    )
    ctx.expect_overlap(
        front_leg_0,
        front_leg_1,
        elem_a="apex_pin",
        elem_b="right_leg_rail",
        axes="xz",
        min_overlap=0.004,
        name="apex pin also captures the upper leg rail",
    )

    ctx.allow_overlap(
        clip_carriage,
        clip_jaw,
        elem_a="clip_hinge_pin",
        elem_b="jaw_barrel",
        reason="The spring clip jaw barrel rotates around the fixed hinge pin.",
    )
    ctx.expect_overlap(
        clip_carriage,
        clip_jaw,
        elem_a="clip_hinge_pin",
        elem_b="jaw_barrel",
        axes="x",
        min_overlap=0.045,
        name="clip hinge pin spans the jaw barrel",
    )
    ctx.expect_overlap(
        clip_carriage,
        clip_jaw,
        elem_a="clip_hinge_pin",
        elem_b="jaw_barrel",
        axes="yz",
        min_overlap=0.006,
        name="clip jaw barrel is concentric with the hinge pin",
    )

    # The slotted carrier remains retained in the long dark slot at both ends
    # of travel, and positive travel moves the clip up the front leg toward the apex.
    ctx.expect_within(
        clip_carriage,
        front_leg_0,
        inner_elem="carriage_tongue",
        outer_elem="slot_track",
        axes="xz",
        margin=0.004,
        name="lower clip tongue lies within the leg slot",
    )
    ctx.expect_overlap(
        clip_carriage,
        front_leg_0,
        elem_a="carriage_tongue",
        elem_b="slot_track",
        axes="y",
        min_overlap=0.0005,
        name="clip tongue is seated in the front slot",
    )

    rest_clip = ctx.part_world_position(clip_carriage)
    with ctx.pose({clip_slide: 0.110}):
        ctx.expect_within(
            clip_carriage,
            front_leg_0,
            inner_elem="carriage_tongue",
            outer_elem="slot_track",
            axes="xz",
            margin=0.004,
            name="raised clip tongue remains within the leg slot",
        )
        raised_clip = ctx.part_world_position(clip_carriage)
    ctx.check(
        "clip slide moves upward along the front leg",
        rest_clip is not None
        and raised_clip is not None
        and raised_clip[2] > rest_clip[2] + 0.09
        and raised_clip[0] > rest_clip[0] + 0.025,
        details=f"rest={rest_clip}, raised={raised_clip}",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    right_foot_rest = _center_from_aabb(ctx.part_element_world_aabb(front_leg_1, elem="foot_pad_1"))
    with ctx.pose({apex_hinge: 0.25}):
        right_foot_spread = _center_from_aabb(ctx.part_element_world_aabb(front_leg_1, elem="foot_pad_1"))
    ctx.check(
        "apex hinge spreads the second front leg",
        right_foot_rest is not None
        and right_foot_spread is not None
        and right_foot_spread[0] > right_foot_rest[0] + 0.05,
        details=f"rest={right_foot_rest}, spread={right_foot_spread}",
    )

    prop_rest = _center_from_aabb(ctx.part_element_world_aabb(prop_strut, elem="rear_foot_pad"))
    with ctx.pose({prop_hinge: 0.85}):
        prop_folded = _center_from_aabb(ctx.part_element_world_aabb(prop_strut, elem="rear_foot_pad"))
    ctx.check(
        "rear prop folds upward on its base hinge",
        prop_rest is not None
        and prop_folded is not None
        and prop_folded[2] > prop_rest[2] + 0.10,
        details=f"rest={prop_rest}, folded={prop_folded}",
    )

    jaw_rest = _center_from_aabb(ctx.part_element_world_aabb(clip_jaw, elem="jaw_pad"))
    with ctx.pose({jaw_hinge: 0.50}):
        jaw_open = _center_from_aabb(ctx.part_element_world_aabb(clip_jaw, elem="jaw_pad"))
    ctx.check(
        "spring clip jaw opens forward",
        jaw_rest is not None
        and jaw_open is not None
        and jaw_open[1] < jaw_rest[1] - 0.010,
        details=f"rest={jaw_rest}, open={jaw_open}",
    )

    return ctx.report()


object_model = build_object_model()
