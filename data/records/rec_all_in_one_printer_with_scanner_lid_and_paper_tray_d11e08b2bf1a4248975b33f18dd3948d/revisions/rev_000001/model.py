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


BODY_W = 0.46
BODY_D = 0.40
BODY_H = 0.16
BODY_FRONT_Y = -(BODY_D * 0.5)
BODY_REAR_Y = BODY_D * 0.5

LID_W = 0.352
LID_D = 0.252
LID_SHELL_H = 0.032
LID_HINGE_Y = 0.108
LID_HINGE_Z = BODY_H

TRAY_W = 0.34
TRAY_D = 0.22
TRAY_H = 0.055


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multifunction_printer")

    shell = model.material("shell", rgba=(0.90, 0.91, 0.93, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    panel = model.material("panel", rgba=(0.30, 0.32, 0.35, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.70, 0.78, 0.35))
    tray_grey = model.material("tray_grey", rgba=(0.72, 0.74, 0.77, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=shell,
        name="base_plate",
    )
    for x_sign, name in ((-1.0, "left_wall"), (1.0, "right_wall")):
        body.visual(
            Box((0.012, BODY_D, 0.150)),
            origin=Origin(xyz=(x_sign * 0.224, 0.0, 0.080)),
            material=shell,
            name=name,
        )
    body.visual(
        Box((BODY_W, 0.012, 0.150)),
        origin=Origin(xyz=(0.0, BODY_REAR_Y - 0.006, 0.080)),
        material=shell,
        name="rear_wall",
    )
    body.visual(
        Box((0.050, 0.012, 0.105)),
        origin=Origin(xyz=(-0.205, BODY_FRONT_Y + 0.006, 0.0575)),
        material=shell,
        name="front_cheek_0",
    )
    body.visual(
        Box((0.050, 0.012, 0.105)),
        origin=Origin(xyz=(0.205, BODY_FRONT_Y + 0.006, 0.0575)),
        material=shell,
        name="front_cheek_1",
    )
    body.visual(
        Box((0.360, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.006, 0.015)),
        material=shell,
        name="tray_lower_lip",
    )
    body.visual(
        Box((BODY_W, 0.012, 0.055)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y + 0.006, 0.1275)),
        material=shell,
        name="front_fascia",
    )
    body.visual(
        Box((0.350, 0.168, 0.008)),
        origin=Origin(xyz=(0.0, -0.104, 0.029)),
        material=tray_grey,
        name="tray_floor",
    )
    body.visual(
        Box((0.008, 0.168, 0.035)),
        origin=Origin(xyz=(-0.150, -0.104, 0.0465)),
        material=tray_grey,
        name="tray_rail_0",
    )
    body.visual(
        Box((0.008, 0.168, 0.035)),
        origin=Origin(xyz=(0.150, -0.104, 0.0465)),
        material=tray_grey,
        name="tray_rail_1",
    )
    body.visual(
        Box((0.100, 0.220, 0.008)),
        origin=Origin(xyz=(-0.171, -0.020, BODY_H - 0.004)),
        material=shell,
        name="scanner_frame_left",
    )
    body.visual(
        Box((0.100, 0.220, 0.008)),
        origin=Origin(xyz=(0.171, -0.020, BODY_H - 0.004)),
        material=shell,
        name="scanner_frame_right",
    )
    body.visual(
        Box((0.340, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, -0.120, BODY_H - 0.004)),
        material=shell,
        name="scanner_frame_front",
    )
    body.visual(
        Box((0.340, 0.050, 0.008)),
        origin=Origin(xyz=(0.0, 0.080, BODY_H - 0.004)),
        material=shell,
        name="scanner_frame_rear",
    )
    body.visual(
        Box((0.306, 0.176, 0.003)),
        origin=Origin(xyz=(0.0, -0.020, BODY_H - 0.0015)),
        material=glass,
        name="scanner_glass",
    )
    body.visual(
        Box((0.180, 0.040, 0.040)),
        origin=Origin(xyz=(0.108, -0.178, 0.126)),
        material=panel,
        name="control_pod",
    )

    lid = model.part("lid")
    lid.visual(
        Box((LID_W, LID_D, 0.004)),
        origin=Origin(xyz=(0.0, -(LID_D * 0.5), LID_SHELL_H - 0.002)),
        material=shell,
        name="lid_top",
    )
    lid.visual(
        Box((0.010, LID_D - 0.008, 0.028)),
        origin=Origin(xyz=(-0.171, -0.122, 0.014)),
        material=shell,
        name="lid_side_0",
    )
    lid.visual(
        Box((0.010, LID_D - 0.008, 0.028)),
        origin=Origin(xyz=(0.171, -0.122, 0.014)),
        material=shell,
        name="lid_side_1",
    )
    lid.visual(
        Box((LID_W, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, -0.246, 0.014)),
        material=shell,
        name="lid_front",
    )
    lid.visual(
        Box((LID_W, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, -0.006, 0.010)),
        material=shell,
        name="lid_rear_cap",
    )
    lid.visual(
        Box((0.026, 0.220, 0.060)),
        origin=Origin(xyz=(-0.122, -0.085, 0.062)),
        material=panel,
        name="adf_side_0",
    )
    lid.visual(
        Box((0.026, 0.220, 0.060)),
        origin=Origin(xyz=(0.122, -0.085, 0.062)),
        material=panel,
        name="adf_side_1",
    )
    lid.visual(
        Box((0.270, 0.040, 0.028)),
        origin=Origin(xyz=(0.0, -0.188, 0.046)),
        material=panel,
        name="adf_front_bridge",
    )
    lid.visual(
        Box((0.270, 0.040, 0.024)),
        origin=Origin(xyz=(0.0, 0.006, 0.044)),
        material=panel,
        name="adf_rear_bridge",
    )
    lid.visual(
        Box((0.034, 0.170, 0.008)),
        origin=Origin(xyz=(-0.112, -0.076, 0.088)),
        material=charcoal,
        name="adf_top_rail_0",
    )
    lid.visual(
        Box((0.034, 0.170, 0.008)),
        origin=Origin(xyz=(0.112, -0.076, 0.088)),
        material=charcoal,
        name="adf_top_rail_1",
    )

    tray = model.part("paper_tray")
    tray.visual(
        Box((0.330, TRAY_D, 0.004)),
        origin=Origin(xyz=(0.0, -0.010, 0.002)),
        material=tray_grey,
        name="tray_panel",
    )
    tray.visual(
        Box((0.004, 0.205, 0.025)),
        origin=Origin(xyz=(-0.163, -0.004, 0.0145)),
        material=tray_grey,
        name="tray_side_0",
    )
    tray.visual(
        Box((0.004, 0.205, 0.025)),
        origin=Origin(xyz=(0.163, -0.004, 0.0145)),
        material=tray_grey,
        name="tray_side_1",
    )
    tray.visual(
        Box((TRAY_W, 0.018, TRAY_H)),
        origin=Origin(xyz=(0.0, -0.015, TRAY_H * 0.5)),
        material=shell,
        name="tray_face",
    )
    tray.visual(
        Box((0.108, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, -0.026, 0.036)),
        material=charcoal,
        name="tray_handle",
    )
    tray.visual(
        Box((0.286, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.100, 0.0055)),
        material=tray_grey,
        name="tray_stop",
    )

    feeder_flap = model.part("feeder_flap")
    feeder_flap.visual(
        Box((0.248, 0.060, 0.003)),
        origin=Origin(xyz=(0.0, 0.030, 0.0015)),
        material=tray_grey,
        name="flap_panel",
    )
    feeder_flap.visual(
        Box((0.006, 0.054, 0.018)),
        origin=Origin(xyz=(-0.121, 0.030, 0.0105)),
        material=tray_grey,
        name="flap_side_0",
    )
    feeder_flap.visual(
        Box((0.006, 0.054, 0.018)),
        origin=Origin(xyz=(0.121, 0.030, 0.0105)),
        material=tray_grey,
        name="flap_side_1",
    )
    feeder_flap.visual(
        Box((0.248, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.058, 0.0075)),
        material=tray_grey,
        name="flap_lip",
    )

    output_tray = model.part("output_tray")
    output_tray.visual(
        Box((0.220, 0.003, 0.044)),
        origin=Origin(xyz=(0.0, -0.0015, -0.022)),
        material=tray_grey,
        name="output_panel",
    )
    output_tray.visual(
        Box((0.220, 0.040, 0.003)),
        origin=Origin(xyz=(0.0, -0.020, -0.0445)),
        material=tray_grey,
        name="output_shelf",
    )
    output_tray.visual(
        Box((0.003, 0.038, 0.010)),
        origin=Origin(xyz=(-0.108, -0.020, -0.040)),
        material=tray_grey,
        name="output_side_0",
    )
    output_tray.visual(
        Box((0.003, 0.038, 0.010)),
        origin=Origin(xyz=(0.108, -0.020, -0.040)),
        material=tray_grey,
        name="output_side_1",
    )

    dial = model.part("control_dial")
    dial.visual(
        Cylinder(radius=0.031, length=0.004),
        origin=Origin(xyz=(0.0, -0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="dial_bezel",
    )
    dial.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=0.012, length=0.003),
        origin=Origin(xyz=(0.0, -0.023, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=tray_grey,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, 0.002, 0.014)),
        origin=Origin(xyz=(0.018, -0.022, 0.0)),
        material=tray_grey,
        name="dial_pointer",
    )

    button_material = model.material("button_material", rgba=(0.84, 0.86, 0.89, 1.0))
    button_positions = (0.104, 0.134, 0.164)
    for index, button_x in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.018, 0.010, 0.014)),
            origin=Origin(xyz=(0.0, -0.005, 0.007)),
            material=panel,
            name="button_stem",
        )
        button.visual(
            Box((0.022, 0.006, 0.010)),
            origin=Origin(xyz=(0.0, -0.011, 0.007)),
            material=button_material,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, -0.198, 0.128)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.06,
                lower=0.0,
                upper=0.006,
            ),
        )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, LID_HINGE_Y, LID_HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_paper_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, -0.188, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.22,
            lower=0.0,
            upper=0.100,
        ),
    )
    model.articulation(
        "lid_to_feeder_flap",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=feeder_flap,
        origin=Origin(xyz=(0.0, 0.024, 0.092)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "lid_to_output_tray",
        ArticulationType.REVOLUTE,
        parent=lid,
        child=output_tray,
        origin=Origin(xyz=(0.0, -0.210, 0.060)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_control_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.050, -0.198, 0.128)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("paper_tray")
    feeder_flap = object_model.get_part("feeder_flap")
    output_tray = object_model.get_part("output_tray")
    dial = object_model.get_part("control_dial")
    lid_hinge = object_model.get_articulation("body_to_lid")
    tray_slide = object_model.get_articulation("body_to_paper_tray")
    flap_hinge = object_model.get_articulation("lid_to_feeder_flap")
    output_hinge = object_model.get_articulation("lid_to_output_tray")
    dial_joint = object_model.get_articulation("body_to_control_dial")

    lid_limits = lid_hinge.motion_limits
    tray_limits = tray_slide.motion_limits
    flap_limits = flap_hinge.motion_limits
    output_limits = output_hinge.motion_limits
    dial_limits = dial_joint.motion_limits

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid closes onto the scanner deck",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.240,
            name="lid covers the flatbed opening",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem="lid_front",
                max_gap=None,
                min_gap=0.090,
                name="opened lid lifts the front edge clear of the body",
            )

    closed_tray_y = ctx.part_world_position(tray)
    if tray_limits is not None and tray_limits.upper is not None:
        with ctx.pose({tray_slide: 0.0}):
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                min_overlap=0.090,
                name="paper tray remains inserted when closed",
            )
        with ctx.pose({tray_slide: tray_limits.upper}):
            ctx.expect_overlap(
                tray,
                body,
                axes="y",
                min_overlap=0.010,
                name="paper tray keeps retained insertion when extended",
            )
            extended_tray_y = ctx.part_world_position(tray)
        ctx.check(
            "paper tray extends forward",
            closed_tray_y is not None
            and extended_tray_y is not None
            and extended_tray_y[1] < closed_tray_y[1] - 0.080,
            details=f"closed={closed_tray_y}, extended={extended_tray_y}",
        )

    closed_flap_aabb = ctx.part_world_aabb(feeder_flap)
    if flap_limits is not None and flap_limits.upper is not None:
        with ctx.pose({flap_hinge: flap_limits.upper}):
            open_flap_aabb = ctx.part_world_aabb(feeder_flap)
        ctx.check(
            "feeder flap lifts upward",
            closed_flap_aabb is not None
            and open_flap_aabb is not None
            and open_flap_aabb[1][2] > closed_flap_aabb[1][2] + 0.035,
            details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
        )

    closed_output_aabb = ctx.part_world_aabb(output_tray)
    closed_output_panel_aabb = ctx.part_element_world_aabb(output_tray, elem="output_panel")
    if output_limits is not None and output_limits.upper is not None:
        with ctx.pose({output_hinge: output_limits.upper}):
            open_output_aabb = ctx.part_world_aabb(output_tray)
            open_output_panel_aabb = ctx.part_element_world_aabb(output_tray, elem="output_panel")
        ctx.check(
            "output tray swings forward into a flatter support pose",
            closed_output_aabb is not None
            and open_output_aabb is not None
            and closed_output_panel_aabb is not None
            and open_output_panel_aabb is not None
            and open_output_aabb[0][1] < closed_output_aabb[0][1] - 0.010
            and (open_output_panel_aabb[1][1] - open_output_panel_aabb[0][1])
            > (closed_output_panel_aabb[1][1] - closed_output_panel_aabb[0][1]) + 0.025
            and (open_output_panel_aabb[1][2] - open_output_panel_aabb[0][2])
            < (closed_output_panel_aabb[1][2] - closed_output_panel_aabb[0][2]) - 0.015,
            details=(
                f"closed_part={closed_output_aabb}, open_part={open_output_aabb}, "
                f"closed_panel={closed_output_panel_aabb}, open_panel={open_output_panel_aabb}"
            ),
        )

    ctx.check(
        "control dial uses continuous rotation",
        dial_limits is not None and dial_limits.lower is None and dial_limits.upper is None,
        details=f"dial_limits={dial_limits}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits
        closed_button_pos = ctx.part_world_position(button)
        if button_limits is not None and button_limits.upper is not None:
            with ctx.pose({button_joint: button_limits.upper}):
                pressed_button_pos = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} presses inward",
                closed_button_pos is not None
                and pressed_button_pos is not None
                and pressed_button_pos[1] > closed_button_pos[1] + 0.004,
                details=f"closed={closed_button_pos}, pressed={pressed_button_pos}",
            )

    return ctx.report()


object_model = build_object_model()
