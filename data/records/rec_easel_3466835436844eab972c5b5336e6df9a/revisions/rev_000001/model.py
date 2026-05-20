from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="classroom_whiteboard_easel")

    aluminum = Material("satin_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    dark_plastic = Material("dark_plastic", color=(0.05, 0.055, 0.06, 1.0))
    white_surface = Material("gloss_whiteboard", color=(0.96, 0.98, 0.97, 1.0))
    tray_blue = Material("blue_marker_tray", color=(0.16, 0.29, 0.50, 1.0))
    rubber = Material("black_rubber", color=(0.015, 0.015, 0.014, 1.0))

    front_frame = model.part("front_frame")

    # Front A-frame and whiteboard panel.  The display face is on the -Y side,
    # with the rear support folding behind it toward +Y.
    upright_x = 0.72
    front_upright_names = ("front_upright_0", "front_upright_1")
    for index, x in enumerate((-upright_x, upright_x)):
        front_frame.visual(
            Box((0.045, 0.050, 1.78)),
            origin=Origin(xyz=(x, 0.0, 0.93)),
            material=aluminum,
            name=front_upright_names[index],
        )
        front_frame.visual(
            Box((0.20, 0.44, 0.055)),
            origin=Origin(xyz=(x, -0.04, 0.028)),
            material=aluminum,
            name=f"front_foot_{index}",
        )
        front_frame.visual(
            Box((0.16, 0.08, 0.030)),
            origin=Origin(xyz=(x, -0.23, 0.020)),
            material=rubber,
            name=f"front_foot_pad_{index}",
        )

    front_frame.visual(
        Box((1.54, 0.052, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=aluminum,
        name="lower_crossbar",
    )
    front_frame.visual(
        Box((1.54, 0.052, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 1.79)),
        material=aluminum,
        name="top_crossbar",
    )
    front_frame.visual(
        Box((1.22, 0.030, 0.98)),
        origin=Origin(xyz=(0.0, -0.018, 1.14)),
        material=white_surface,
        name="whiteboard_panel",
    )
    front_frame.visual(
        Box((1.50, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, -0.018, 1.655)),
        material=aluminum,
        name="board_top_trim",
    )
    front_frame.visual(
        Box((1.50, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, -0.018, 0.625)),
        material=aluminum,
        name="board_bottom_trim",
    )
    for index, x in enumerate((-0.625, 0.625)):
        front_frame.visual(
            Box((0.035, 0.040, 1.02)),
            origin=Origin(xyz=(x, -0.018, 1.14)),
            material=aluminum,
            name=f"board_side_trim_{index}",
        )

    # Split hinge barrels and side brackets at the crown leave a central gap for
    # the moving rear-leg knuckle, so the hinge reads mechanically without a
    # broad interpenetrating block.
    hinge_z = 1.865
    hinge_y = 0.082
    fixed_hinge_barrel_names = ("fixed_hinge_barrel_0", "fixed_hinge_barrel_1")
    for index, x in enumerate((-0.14, 0.14)):
        cheek_x = -0.18 if x < 0.0 else 0.18
        front_frame.visual(
            Box((0.075, 0.060, 0.115)),
            origin=Origin(xyz=(cheek_x, 0.045, 1.825)),
            material=aluminum,
            name=f"hinge_cheek_{index}",
        )
        front_frame.visual(
            Cylinder(radius=0.024, length=0.090),
            origin=Origin(xyz=(x, hinge_y, hinge_z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_plastic,
            name=fixed_hinge_barrel_names[index],
        )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Cylinder(radius=0.025, length=0.190),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_plastic,
        name="hinge_knuckle",
    )
    rear_drop = 1.80
    rear_setback = 0.76
    rear_length = math.sqrt(rear_drop * rear_drop + rear_setback * rear_setback)
    rear_angle = math.pi + math.atan2(rear_setback, rear_drop)
    rear_leg.visual(
        Box((0.060, 0.060, rear_length)),
        origin=Origin(
            xyz=(0.0, rear_setback / 2.0, -rear_drop / 2.0),
            rpy=(rear_angle, 0.0, 0.0),
        ),
        material=aluminum,
        name="rear_strut",
    )
    rear_leg.visual(
        Box((0.095, 0.075, 0.145)),
        origin=Origin(xyz=(0.0, 0.030, -0.060)),
        material=aluminum,
        name="knuckle_yoke",
    )
    rear_leg.visual(
        Box((0.48, 0.115, 0.060)),
        origin=Origin(xyz=(0.0, rear_setback, -rear_drop)),
        material=aluminum,
        name="rear_foot",
    )
    rear_leg.visual(
        Box((0.38, 0.085, 0.030)),
        origin=Origin(xyz=(0.0, rear_setback, -rear_drop - 0.026)),
        material=rubber,
        name="rear_foot_pad",
    )

    marker_tray = model.part("marker_tray")
    marker_tray.visual(
        Box((1.52, 0.150, 0.035)),
        origin=Origin(xyz=(0.0, -0.135, 0.000)),
        material=tray_blue,
        name="tray_shelf",
    )
    marker_tray.visual(
        Box((1.52, 0.035, 0.075)),
        origin=Origin(xyz=(0.0, -0.212, 0.055)),
        material=tray_blue,
        name="front_lip",
    )
    marker_tray.visual(
        Box((1.52, 0.032, 0.060)),
        origin=Origin(xyz=(0.0, -0.065, 0.046)),
        material=tray_blue,
        name="back_lip",
    )
    for index, x in enumerate((-0.775, 0.775)):
        marker_tray.visual(
            Box((0.045, 0.155, 0.080)),
            origin=Origin(xyz=(x, -0.135, 0.040)),
            material=tray_blue,
            name=f"tray_end_cap_{index}",
        )
    guide_front_pad_names = ("guide_front_pad_0", "guide_front_pad_1")
    guide_cheek_inner_names = ("guide_cheek_inner_0", "guide_cheek_inner_1")
    guide_cheek_outer_names = ("guide_cheek_outer_0", "guide_cheek_outer_1")
    for index, x in enumerate((-upright_x, upright_x)):
        marker_tray.visual(
            Box((0.120, 0.025, 0.180)),
            origin=Origin(xyz=(x, -0.0475, 0.085)),
            material=dark_plastic,
            name=guide_front_pad_names[index],
        )
        marker_tray.visual(
            Box((0.025, 0.060, 0.180)),
            origin=Origin(xyz=(x - 0.035, -0.005, 0.085)),
            material=dark_plastic,
            name=guide_cheek_inner_names[index],
        )
        marker_tray.visual(
            Box((0.025, 0.060, 0.180)),
            origin=Origin(xyz=(x + 0.035, -0.005, 0.085)),
            material=dark_plastic,
            name=guide_cheek_outer_names[index],
        )

    model.articulation(
        "crown_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.15, upper=0.35),
    )
    model.articulation(
        "frame_to_marker_tray",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=marker_tray,
        origin=Origin(xyz=(0.0, 0.0, 0.76)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.25, lower=0.0, upper=0.62),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front_frame = object_model.get_part("front_frame")
    rear_leg = object_model.get_part("rear_leg")
    marker_tray = object_model.get_part("marker_tray")
    hinge = object_model.get_articulation("crown_to_rear_leg")
    slide = object_model.get_articulation("frame_to_marker_tray")

    def _axis_close(actual, expected, tol=1e-6) -> bool:
        return all(abs(a - b) <= tol for a, b in zip(actual, expected))

    ctx.check(
        "rear leg is hinged at crown",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and _axis_close(hinge.axis, (1.0, 0.0, 0.0))
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower is not None
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper - hinge.motion_limits.lower >= 0.45,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge.motion_limits}",
    )
    ctx.check(
        "marker tray slides vertically on frame",
        slide.articulation_type == ArticulationType.PRISMATIC
        and _axis_close(slide.axis, (0.0, 0.0, 1.0))
        and slide.motion_limits is not None
        and slide.motion_limits.upper is not None
        and slide.motion_limits.upper >= 0.55,
        details=f"type={slide.articulation_type}, axis={slide.axis}, limits={slide.motion_limits}",
    )

    # The guide blocks ride on both front uprights rather than floating in front
    # of the board.
    ctx.expect_contact(
        front_frame,
        marker_tray,
        elem_a="front_upright_0",
        elem_b="guide_cheek_outer_0",
        contact_tol=1e-5,
        name="tray guide bears on upright 0",
    )
    ctx.expect_contact(
        front_frame,
        marker_tray,
        elem_a="front_upright_1",
        elem_b="guide_cheek_inner_1",
        contact_tol=1e-5,
        name="tray guide bears on upright 1",
    )
    ctx.expect_overlap(
        marker_tray,
        front_frame,
        axes="z",
        elem_a="guide_front_pad_0",
        elem_b="front_upright_0",
        min_overlap=0.16,
        name="tray guide has retained rail engagement",
    )

    # The hinge knuckle is carried by the split crown barrels.
    ctx.expect_contact(
        front_frame,
        rear_leg,
        elem_a="fixed_hinge_barrel_0",
        elem_b="hinge_knuckle",
        contact_tol=1e-5,
        name="rear leg hinge knuckle seats in crown barrel",
    )

    rest_tray_pos = ctx.part_world_position(marker_tray)
    with ctx.pose({slide: slide.motion_limits.upper}):
        raised_tray_pos = ctx.part_world_position(marker_tray)
        ctx.expect_contact(
            front_frame,
            marker_tray,
            elem_a="front_upright_0",
            elem_b="guide_cheek_outer_0",
            contact_tol=1e-5,
            name="raised tray guide still rides upright",
        )
        ctx.expect_overlap(
            marker_tray,
            front_frame,
            axes="z",
            elem_a="guide_front_pad_0",
            elem_b="front_upright_0",
            min_overlap=0.16,
            name="raised tray retains rail engagement",
        )
    ctx.check(
        "tray upper stop raises shelf along uprights",
        rest_tray_pos is not None
        and raised_tray_pos is not None
        and raised_tray_pos[2] > rest_tray_pos[2] + 0.55
        and abs(raised_tray_pos[0] - rest_tray_pos[0]) < 1e-6
        and abs(raised_tray_pos[1] - rest_tray_pos[1]) < 1e-6,
        details=f"rest={rest_tray_pos}, raised={raised_tray_pos}",
    )

    rest_rear_aabb = ctx.part_world_aabb(rear_leg)
    with ctx.pose({hinge: 0.25}):
        rotated_rear_aabb = ctx.part_world_aabb(rear_leg)
    ctx.check(
        "rear support leg stands behind the board",
        rest_rear_aabb is not None
        and rest_rear_aabb[1][1] > 0.80
        and rest_rear_aabb[0][2] < 0.05,
        details=f"rear_aabb={rest_rear_aabb}",
    )
    ctx.check(
        "rear support leg rotates on crown hinge",
        rest_rear_aabb is not None
        and rotated_rear_aabb is not None
        and rotated_rear_aabb[1][1] > rest_rear_aabb[1][1] + 0.20
        and rotated_rear_aabb[0][2] > rest_rear_aabb[0][2] + 0.04,
        details=f"rest={rest_rear_aabb}, rotated={rotated_rear_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
