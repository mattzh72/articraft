from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a_frame_display_easel")

    wood = Material("satin_beech_wood", rgba=(0.72, 0.49, 0.25, 1.0))
    end_grain = Material("darker_end_grain", rgba=(0.46, 0.29, 0.13, 1.0))
    black = Material("black_rubber", rgba=(0.015, 0.013, 0.012, 1.0))
    steel = Material("brushed_dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    brass = Material("warm_brass", rgba=(0.86, 0.62, 0.23, 1.0))

    frame = model.part("front_frame")

    # Two sloping wooden front legs form the fixed A-frame.  Local box Z is the
    # length axis; a small Y rotation leans each leg into the A silhouette.
    leg_length = 1.52
    leg_angle = math.atan2(0.34, 1.48)
    frame.visual(
        Box((0.052, 0.044, leg_length)),
        origin=Origin(xyz=(-0.245, 0.0, 0.790), rpy=(0.0, leg_angle, 0.0)),
        material=wood,
        name="front_leg_0",
    )
    frame.visual(
        Box((0.052, 0.044, leg_length)),
        origin=Origin(xyz=(0.245, 0.0, 0.790), rpy=(0.0, -leg_angle, 0.0)),
        material=wood,
        name="front_leg_1",
    )

    # Structural cross pieces and display tray tie the two legs into one rigid,
    # fixed front frame.
    frame.visual(
        Box((0.25, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 1.515)),
        material=wood,
        name="apex_block",
    )
    frame.visual(
        Box((0.52, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 1.285)),
        material=wood,
        name="upper_crossbar",
    )
    frame.visual(
        Box((0.72, 0.048, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.620)),
        material=wood,
        name="lower_crossbar",
    )
    frame.visual(
        Box((0.88, 0.100, 0.040)),
        origin=Origin(xyz=(0.0, -0.055, 0.610)),
        material=wood,
        name="display_tray",
    )
    frame.visual(
        Box((0.88, 0.030, 0.080)),
        origin=Origin(xyz=(0.0, -0.115, 0.660)),
        material=wood,
        name="tray_lip",
    )

    # Rubber feet sit at the two fixed front contact points.
    frame.visual(
        Box((0.17, 0.095, 0.050)),
        origin=Origin(xyz=(-0.405, 0.0, 0.025)),
        material=black,
        name="front_foot_0",
    )
    frame.visual(
        Box((0.17, 0.095, 0.050)),
        origin=Origin(xyz=(0.405, 0.0, 0.025)),
        material=black,
        name="front_foot_1",
    )

    # Exposed top hinge/pin detail at the crown of the A-frame.
    frame.visual(
        Cylinder(radius=0.030, length=0.32),
        origin=Origin(xyz=(0.0, -0.004, 1.555), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="top_hinge_pin",
    )
    frame.visual(
        Box((0.34, 0.020, 0.032)),
        origin=Origin(xyz=(0.0, -0.038, 1.555)),
        material=steel,
        name="top_hinge_leaf",
    )

    # The rear-side adjustment slot is modeled as two separated steel rails,
    # leaving a real open center channel for the sliding tongue.
    for x, name in ((-0.032, "slot_rail_0"), (0.032, "slot_rail_1")):
        frame.visual(
            Box((0.012, 0.018, 0.720)),
            origin=Origin(xyz=(x, 0.025, 0.950)),
            material=steel,
            name=name,
        )
    frame.visual(
        Box((0.092, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.025, 1.310)),
        material=steel,
        name="slot_end_top",
    )
    frame.visual(
        Box((0.092, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.025, 0.590)),
        material=steel,
        name="slot_end_bottom",
    )

    # Small ratchet-like witness marks make it read as a locking slot rather
    # than a plain decorative strip.
    for i, z in enumerate((0.705, 0.815, 0.925, 1.035, 1.145, 1.255)):
        frame.visual(
            Box((0.026, 0.006, 0.008)),
            origin=Origin(xyz=(-0.044, 0.037, z)),
            material=steel,
            name=f"lock_notch_{i}",
        )

    slider = model.part("slot_slider")
    slider.visual(
        Box((0.018, 0.012, 0.086)),
        origin=Origin(xyz=(0.0, -0.021, 0.0)),
        material=steel,
        name="slider_tongue",
    )
    slider.visual(
        Box((0.022, 0.018, 0.078)),
        origin=Origin(xyz=(0.0, -0.012, 0.0)),
        material=steel,
        name="slider_neck",
    )
    slider.visual(
        Box((0.125, 0.024, 0.078)),
        origin=Origin(xyz=(0.0, 0.000, 0.0)),
        material=steel,
        name="slider_plate",
    )
    for x, name in ((-0.055, "clevis_ear_0"), (0.055, "clevis_ear_1")):
        slider.visual(
            Box((0.014, 0.070, 0.055)),
            origin=Origin(xyz=(x, 0.035, 0.0)),
            material=steel,
            name=name,
        )
    slider.visual(
        Cylinder(radius=0.011, length=0.160),
        origin=Origin(xyz=(0.0, 0.048, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="brace_pin",
    )

    brace = model.part("rear_brace")
    brace.visual(
        Cylinder(radius=0.023, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="brace_eye",
    )
    brace_dy = 0.69
    brace_dz = -0.66
    brace_length = math.sqrt(brace_dy * brace_dy + brace_dz * brace_dz)
    brace_roll = math.atan2(-brace_dy, brace_dz)
    brace.visual(
        Box((0.052, 0.040, brace_length)),
        origin=Origin(xyz=(0.0, 0.365, -0.340), rpy=(brace_roll, 0.0, 0.0)),
        material=wood,
        name="rear_leg",
    )
    brace.visual(
        Box((0.240, 0.090, 0.028)),
        origin=Origin(xyz=(0.0, 0.720, -0.690)),
        material=black,
        name="rear_foot",
    )

    knob = model.part("lock_knob")
    knob.visual(
        Cylinder(radius=0.036, length=0.024),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.008, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="threaded_stem",
    )

    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(0.0, 0.053, 0.705)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.390),
        motion_properties=MotionProperties(damping=0.6, friction=1.2),
    )
    model.articulation(
        "slider_to_brace",
        ArticulationType.REVOLUTE,
        parent=slider,
        child=brace,
        origin=Origin(xyz=(0.0, 0.048, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.8, lower=-0.32, upper=0.42),
        motion_properties=MotionProperties(damping=0.3, friction=0.8),
    )
    model.articulation(
        "slider_to_knob",
        ArticulationType.CONTINUOUS,
        parent=slider,
        child=knob,
        origin=Origin(xyz=(0.0, -0.030, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=4.0),
        motion_properties=MotionProperties(damping=0.05, friction=0.2),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("front_frame")
    slider = object_model.get_part("slot_slider")
    brace = object_model.get_part("rear_brace")
    knob = object_model.get_part("lock_knob")
    slide = object_model.get_articulation("frame_to_slider")
    pivot = object_model.get_articulation("slider_to_brace")
    knob_joint = object_model.get_articulation("slider_to_knob")

    ctx.allow_overlap(
        slider,
        brace,
        elem_a="brace_pin",
        elem_b="brace_eye",
        reason="The rear brace eye is intentionally captured around the hinge pin.",
    )
    ctx.allow_overlap(
        knob,
        slider,
        elem_a="threaded_stem",
        elem_b="slider_tongue",
        reason="The locking screw stem intentionally passes through the sliding tongue to clamp it in the slot.",
    )
    ctx.allow_overlap(
        knob,
        slider,
        elem_a="threaded_stem",
        elem_b="slider_neck",
        reason="The same locking screw continues through the short neck of the captured slider carriage.",
    )
    ctx.expect_overlap(
        slider,
        brace,
        axes="xyz",
        elem_a="brace_pin",
        elem_b="brace_eye",
        min_overlap=0.018,
        name="brace eye is captured on the pin",
    )
    ctx.expect_overlap(
        knob,
        slider,
        axes="xyz",
        elem_a="threaded_stem",
        elem_b="slider_tongue",
        min_overlap=0.006,
        name="locking screw passes through slider tongue",
    )
    ctx.expect_overlap(
        knob,
        slider,
        axes="xyz",
        elem_a="threaded_stem",
        elem_b="slider_neck",
        min_overlap=0.006,
        name="locking screw passes through slider neck",
    )
    ctx.expect_within(
        slider,
        frame,
        axes="x",
        inner_elem="slider_tongue",
        outer_elem="slot_end_bottom",
        margin=0.0,
        name="slider tongue fits between slot rails",
    )
    ctx.expect_overlap(
        slider,
        frame,
        axes="z",
        elem_a="slider_tongue",
        elem_b="slot_rail_0",
        min_overlap=0.070,
        name="slider remains engaged in slot at rest",
    )
    ctx.expect_gap(
        slider,
        frame,
        axis="y",
        positive_elem="slider_plate",
        negative_elem="slot_rail_0",
        min_gap=0.0,
        max_gap=0.018,
        name="slider plate rides just behind the slot rail",
    )

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({slide: 0.390}):
        ctx.expect_overlap(
            slider,
            frame,
            axes="z",
            elem_a="slider_tongue",
            elem_b="slot_rail_0",
            min_overlap=0.070,
            name="slider remains engaged at upper lock position",
        )
        high_pos = ctx.part_world_position(slider)
    ctx.check(
        "prismatic lock carriage travels upward in the slot",
        rest_pos is not None and high_pos is not None and high_pos[2] > rest_pos[2] + 0.30,
        details=f"rest={rest_pos}, high={high_pos}",
    )

    foot_box = ctx.part_element_world_aabb(brace, elem="rear_foot")
    ctx.check(
        "rear brace foot sits on the floor plane",
        foot_box is not None and 0.0 <= foot_box[0][2] <= 0.030,
        details=f"rear_foot_aabb={foot_box}",
    )

    at_rest = ctx.part_world_aabb(brace)
    with ctx.pose({pivot: 0.30}):
        rotated = ctx.part_world_aabb(brace)
    ctx.check(
        "rear brace pivots to vary the easel angle",
        at_rest is not None
        and rotated is not None
        and rotated[1][1] > at_rest[1][1] + 0.03,
        details=f"rest={at_rest}, rotated={rotated}",
    )
    ctx.check(
        "separate locking knob is articulated",
        knob is not None and knob_joint is not None,
        details="lock_knob and slider_to_knob should both exist",
    )

    return ctx.report()


object_model = build_object_model()
