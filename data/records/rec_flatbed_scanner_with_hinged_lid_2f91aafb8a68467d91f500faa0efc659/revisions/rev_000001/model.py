from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    PianoHingeGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_scanning_platform")

    warm_white = model.material("warm_white_plastic", rgba=(0.86, 0.84, 0.78, 1.0))
    dark_plastic = model.material("matte_dark_plastic", rgba=(0.035, 0.038, 0.042, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    satin_metal = model.material("brushed_satin_metal", rgba=(0.65, 0.66, 0.62, 1.0))
    clear_glass = model.material("slightly_green_glass", rgba=(0.55, 0.86, 0.92, 0.36))
    smoked_glass = model.material("smoked_platen_glass", rgba=(0.08, 0.13, 0.16, 0.45))
    tray_plastic = model.material("charcoal_tray_plastic", rgba=(0.07, 0.075, 0.08, 1.0))
    photo_paper = model.material("semi_gloss_photo_paper", rgba=(0.94, 0.92, 0.86, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.62, 0.44, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=warm_white,
        name="shallow_body",
    )
    body.visual(
        Box((0.585, 0.405, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=dark_plastic,
        name="top_face",
    )
    body.visual(
        Box((0.500, 0.310, 0.005)),
        origin=Origin(xyz=(0.0, -0.010, 0.0755)),
        material=smoked_glass,
        name="glass_platen",
    )
    body.visual(
        Box((0.62, 0.030, 0.028)),
        origin=Origin(xyz=(0.0, 0.222, 0.079)),
        material=warm_white,
        name="rear_hinge_shelf",
    )
    body.visual(
        Box((0.580, 0.016, 0.027)),
        origin=Origin(xyz=(0.0, 0.244, 0.1045)),
        material=warm_white,
        name="hinge_support",
    )
    body.visual(
        Box((0.090, 0.018, 0.004)),
        origin=Origin(xyz=(-0.235, -0.206, 0.073)),
        material=black_rubber,
        name="front_grip_pad",
    )
    # Fixed guide rails sit directly on the glass platen and define the tray's
    # prismatic slide path across the platen face.
    body.visual(
        Box((0.480, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, -0.130, 0.084)),
        material=satin_metal,
        name="front_rail",
    )
    body.visual(
        Box((0.480, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.130, 0.084)),
        material=satin_metal,
        name="rear_rail",
    )

    hinge_mesh = mesh_from_geometry(
        PianoHingeGeometry(
            0.585,
            leaf_width_a=0.020,
            leaf_width_b=0.018,
            leaf_thickness=0.0018,
            pin_diameter=0.006,
            knuckle_pitch=0.030,
        ),
        "full_width_piano_hinge",
    )
    body.visual(
        hinge_mesh,
        origin=Origin(xyz=(0.0, 0.247, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="piano_hinge",
    )
    # A simple visible pin core emphasizes that the rear strip is the full-width
    # revolute axis, even if the generated hinge leaves are visually subtle.
    body.visual(
        Cylinder(radius=0.0032, length=0.590),
        origin=Origin(xyz=(0.0, 0.247, 0.112), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_pin",
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.575, 0.028, 0.025)),
        origin=Origin(xyz=(0.0, -0.031, 0.0125)),
        material=dark_plastic,
        name="rear_frame",
    )
    lid.visual(
        Box((0.575, 0.028, 0.025)),
        origin=Origin(xyz=(0.0, -0.398, 0.0125)),
        material=dark_plastic,
        name="front_frame",
    )
    for side_name, side_x in (("side_frame_0", -0.286), ("side_frame_1", 0.286)):
        lid.visual(
            Box((0.026, 0.384, 0.025)),
            origin=Origin(xyz=(side_x, -0.206, 0.0125)),
            material=dark_plastic,
            name=side_name,
        )
    lid.visual(
        Box((0.552, 0.372, 0.006)),
        origin=Origin(xyz=(0.0, -0.206, 0.007)),
        material=clear_glass,
        name="lid_glass",
    )
    lid.visual(
        Box((0.560, 0.016, 0.006)),
        origin=Origin(xyz=(0.0, -0.019, 0.003)),
        material=satin_metal,
        name="moving_hinge_leaf",
    )

    tray = model.part("print_tray")
    tray.visual(
        Box((0.282, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.130, 0.011)),
        material=tray_plastic,
        name="front_bar",
    )
    tray.visual(
        Box((0.282, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.130, 0.011)),
        material=tray_plastic,
        name="rear_bar",
    )
    for end_name, end_x in (("end_bar_0", -0.134), ("end_bar_1", 0.134)):
        tray.visual(
            Box((0.014, 0.260, 0.010)),
            origin=Origin(xyz=(end_x, 0.0, 0.011)),
            material=tray_plastic,
            name=end_name,
        )
    tray.visual(
        Box((0.260, 0.250, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=tray_plastic,
        name="support_plate",
    )
    # The rails contact these low shoes; the frame above carries the print.
    tray.visual(
        Box((0.205, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.130, 0.003)),
        material=black_rubber,
        name="front_shoe",
    )
    tray.visual(
        Box((0.205, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.130, 0.003)),
        material=black_rubber,
        name="rear_shoe",
    )
    tray.visual(
        Box((0.245, 0.235, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=photo_paper,
        name="print_card",
    )
    tray.visual(
        Box((0.060, 0.012, 0.0025)),
        origin=Origin(xyz=(-0.090, -0.104, 0.01775)),
        material=tray_plastic,
        name="front_clip_0",
    )
    tray.visual(
        Box((0.060, 0.012, 0.0025)),
        origin=Origin(xyz=(0.090, 0.104, 0.01775)),
        material=tray_plastic,
        name="rear_clip_1",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.247, 0.112)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6, lower=0.0, upper=1.35),
    )
    model.articulation(
        "body_to_print_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.22, lower=-0.120, upper=0.120),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("print_tray")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_print_tray")

    ctx.check(
        "lid uses rear revolute hinge",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.upper >= 1.2,
        details=f"type={lid_hinge.articulation_type}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "tray uses lateral prismatic slide",
        tray_slide.articulation_type == ArticulationType.PRISMATIC
        and tray_slide.motion_limits is not None
        and tray_slide.motion_limits.lower <= -0.10
        and tray_slide.motion_limits.upper >= 0.10,
        details=f"type={tray_slide.articulation_type}, limits={tray_slide.motion_limits}",
    )

    ctx.expect_contact(
        tray,
        body,
        elem_a="front_shoe",
        elem_b="front_rail",
        contact_tol=0.0005,
        name="front tray shoe rests on guide rail",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="rear_shoe",
        elem_b="rear_rail",
        contact_tol=0.0005,
        name="rear tray shoe rests on guide rail",
    )
    ctx.expect_gap(
        lid,
        tray,
        axis="z",
        min_gap=0.002,
        max_gap=0.012,
        name="closed glass lid clears print tray",
    )

    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        ctx.expect_overlap(
            tray,
            body,
            axes="xy",
            elem_a="front_shoe",
            elem_b="front_rail",
            min_overlap=0.007,
            name="extended front shoe remains on rail",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="xy",
            elem_a="rear_shoe",
            elem_b="rear_rail",
            min_overlap=0.007,
            name="extended rear shoe remains on rail",
        )
        extended_tray_pos = ctx.part_world_position(tray)
    ctx.check(
        "print tray slides across platen face",
        rest_tray_pos is not None
        and extended_tray_pos is not None
        and extended_tray_pos[0] > rest_tray_pos[0] + 0.10,
        details=f"rest={rest_tray_pos}, extended={extended_tray_pos}",
    )

    closed_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_front = ctx.part_element_world_aabb(lid, elem="front_frame")
    ctx.check(
        "lid front edge lifts upward on hinge",
        closed_front is not None
        and open_front is not None
        and open_front[0][2] > closed_front[0][2] + 0.15,
        details=f"closed={closed_front}, open={open_front}",
    )

    return ctx.report()


object_model = build_object_model()
