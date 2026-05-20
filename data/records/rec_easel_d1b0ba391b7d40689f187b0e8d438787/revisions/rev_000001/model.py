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
    model = ArticulatedObject(name="whiteboard_easel")

    frame_white = model.material("frame_white", rgba=(0.96, 0.97, 0.98, 1.0))
    frame_aluminum = model.material("frame_aluminum", rgba=(0.73, 0.76, 0.79, 1.0))
    plastic_gray = model.material("plastic_gray", rgba=(0.42, 0.45, 0.48, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.055, 0.030, 1.720)),
        origin=Origin(xyz=(-0.467, 0.0, 0.860)),
        material=frame_aluminum,
        name="side_rail_0",
    )
    frame.visual(
        Box((0.055, 0.030, 1.720)),
        origin=Origin(xyz=(0.467, 0.0, 0.860)),
        material=frame_aluminum,
        name="side_rail_1",
    )
    frame.visual(
        Box((0.990, 0.032, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.665)),
        material=frame_aluminum,
        name="top_rail",
    )
    frame.visual(
        Box((0.990, 0.042, 0.072)),
        origin=Origin(xyz=(0.0, 0.003, 0.575)),
        material=frame_aluminum,
        name="bottom_rail",
    )
    frame.visual(
        Box((0.900, 0.016, 1.070)),
        origin=Origin(xyz=(0.0, -0.004, 1.115)),
        material=plastic_gray,
        name="back_panel",
    )
    frame.visual(
        Box((0.845, 0.002, 1.015)),
        origin=Origin(xyz=(0.0, 0.009, 1.115)),
        material=frame_white,
        name="board_surface",
    )
    frame.visual(
        Box((0.110, 0.070, 0.026)),
        origin=Origin(xyz=(-0.467, 0.032, 0.013)),
        material=rubber_black,
        name="front_foot_0",
    )
    frame.visual(
        Box((0.110, 0.070, 0.026)),
        origin=Origin(xyz=(0.467, 0.032, 0.013)),
        material=rubber_black,
        name="front_foot_1",
    )
    frame.visual(
        Box((0.018, 0.028, 0.050)),
        origin=Origin(xyz=(-0.034, -0.027, 1.665)),
        material=plastic_gray,
        name="hinge_ear_0",
    )
    frame.visual(
        Box((0.018, 0.028, 0.050)),
        origin=Origin(xyz=(0.034, -0.027, 1.665)),
        material=plastic_gray,
        name="hinge_ear_1",
    )
    frame.visual(
        Cylinder(radius=0.009, length=0.840),
        origin=Origin(xyz=(0.0, 0.047, 0.515), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_aluminum,
        name="tray_rod",
    )
    frame.visual(
        Box((0.036, 0.050, 0.056)),
        origin=Origin(xyz=(-0.330, 0.024, 0.533)),
        material=plastic_gray,
        name="rod_standoff_0",
    )
    frame.visual(
        Box((0.036, 0.050, 0.056)),
        origin=Origin(xyz=(0.330, 0.024, 0.533)),
        material=plastic_gray,
        name="rod_standoff_1",
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="hinge_barrel",
    )
    rear_leg.visual(
        Box((0.040, 0.024, 0.180)),
        origin=Origin(xyz=(0.0, -0.032, -0.084), rpy=(0.36, 0.0, 0.0)),
        material=charcoal,
        name="hinge_neck",
    )
    rear_leg.visual(
        Box((0.050, 0.030, 1.730)),
        origin=Origin(xyz=(0.0, -0.305, -0.810), rpy=(0.36, 0.0, 0.0)),
        material=charcoal,
        name="leg",
    )

    model.articulation(
        "frame_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, -0.030, 1.665)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=0.0,
            upper=0.30,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.070, 0.048, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=plastic_gray,
        name="clip_bridge",
    )
    tray.visual(
        Box((0.070, 0.016, 0.086)),
        origin=Origin(xyz=(0.0, 0.020, -0.033)),
        material=plastic_gray,
        name="front_bracket",
    )
    tray.visual(
        Box((0.040, 0.016, 0.056)),
        origin=Origin(xyz=(0.0, -0.020, -0.018)),
        material=plastic_gray,
        name="rear_pad",
    )
    tray.visual(
        Box((0.072, 0.026, 0.066)),
        origin=Origin(xyz=(0.0, 0.036, -0.053)),
        material=plastic_gray,
        name="support_web",
    )
    tray.visual(
        Box((0.320, 0.014, 0.044)),
        origin=Origin(xyz=(0.0, 0.056, -0.058)),
        material=plastic_gray,
        name="shelf_back",
    )
    tray.visual(
        Box((0.320, 0.086, 0.012)),
        origin=Origin(xyz=(0.0, 0.098, -0.091)),
        material=plastic_gray,
        name="shelf",
    )
    tray.visual(
        Box((0.320, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.135, -0.082)),
        material=plastic_gray,
        name="front_lip",
    )
    tray.visual(
        Box((0.010, 0.086, 0.020)),
        origin=Origin(xyz=(-0.155, 0.098, -0.080)),
        material=plastic_gray,
        name="end_cap_0",
    )
    tray.visual(
        Box((0.010, 0.086, 0.020)),
        origin=Origin(xyz=(0.155, 0.098, -0.080)),
        material=plastic_gray,
        name="end_cap_1",
    )
    tray.visual(
        Cylinder(radius=0.009, length=0.018),
        origin=Origin(xyz=(0.046, 0.031, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob_boss",
    )
    tray.visual(
        Box((0.022, 0.018, 0.020)),
        origin=Origin(xyz=(0.046, 0.028, -0.010)),
        material=charcoal,
        name="knob_mount",
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=tray,
        origin=Origin(xyz=(0.0, 0.047, 0.515)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.20,
            lower=-0.260,
            upper=0.260,
        ),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.004, length=0.026),
        origin=Origin(xyz=(0.0, 0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="shaft",
    )
    clamp_knob.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob",
    )
    clamp_knob.visual(
        Box((0.030, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
        material=charcoal,
        name="wing",
    )

    model.articulation(
        "tray_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=tray,
        child=clamp_knob,
        origin=Origin(xyz=(0.046, 0.040, -0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=12.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rear_leg = object_model.get_part("rear_leg")
    tray = object_model.get_part("tray")
    clamp_knob = object_model.get_part("clamp_knob")

    rear_leg_hinge = object_model.get_articulation("frame_to_rear_leg")
    tray_slide = object_model.get_articulation("frame_to_tray")
    knob_spin = object_model.get_articulation("tray_to_clamp_knob")

    ctx.expect_gap(
        frame,
        rear_leg,
        axis="y",
        positive_elem="back_panel",
        negative_elem="leg",
        min_gap=0.004,
        name="rear leg sits behind the board panel at rest",
    )
    ctx.expect_overlap(
        tray,
        frame,
        axes="xz",
        elem_a="front_bracket",
        elem_b="tray_rod",
        min_overlap=0.018,
        name="tray bracket stays aligned to the guide rod at rest",
    )
    ctx.expect_gap(
        tray,
        frame,
        axis="y",
        positive_elem="front_bracket",
        negative_elem="tray_rod",
        min_gap=0.002,
        max_gap=0.005,
        name="tray front bracket clears the rod with a small guide gap",
    )
    ctx.expect_gap(
        frame,
        tray,
        axis="y",
        positive_elem="tray_rod",
        negative_elem="rear_pad",
        min_gap=0.002,
        max_gap=0.005,
        name="tray rear pad clears the rod with a small guide gap",
    )
    ctx.expect_contact(
        clamp_knob,
        tray,
        elem_a="shaft",
        elem_b="knob_boss",
        contact_tol=0.0005,
        name="clamp knob shaft seats against the tray boss",
    )
    ctx.check(
        "clamp knob uses continuous rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.motion_limits is not None
        and knob_spin.motion_limits.lower is None
        and knob_spin.motion_limits.upper is None,
        details=(
            f"type={knob_spin.articulation_type}, "
            f"limits={knob_spin.motion_limits}"
        ),
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.220}):
        ctx.expect_overlap(
            tray,
            frame,
            axes="xz",
            elem_a="front_bracket",
            elem_b="tray_rod",
            min_overlap=0.018,
            name="tray bracket remains aligned to the guide rod when shifted right",
        )
        shifted_tray_pos = ctx.part_world_position(tray)

    ctx.check(
        "tray slides to the right",
        rest_tray_pos is not None
        and shifted_tray_pos is not None
        and shifted_tray_pos[0] > rest_tray_pos[0] + 0.18,
        details=f"rest={rest_tray_pos}, shifted={shifted_tray_pos}",
    )

    rest_leg_aabb = ctx.part_element_world_aabb(rear_leg, elem="leg")
    with ctx.pose({rear_leg_hinge: 0.260}):
        folded_leg_aabb = ctx.part_element_world_aabb(rear_leg, elem="leg")

    ctx.check(
        "rear leg folds forward toward the frame",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[1][1] > rest_leg_aabb[1][1] + 0.25,
        details=f"rest={rest_leg_aabb}, folded={folded_leg_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
