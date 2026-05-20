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
    model = ArticulatedObject(name="home_office_printer")

    shell_dark = model.material("shell_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    shell_light = model.material("shell_light", rgba=(0.88, 0.89, 0.90, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.12, 0.13, 0.14, 1.0))
    scanner_glass = model.material("scanner_glass", rgba=(0.56, 0.70, 0.78, 0.45))
    button_dark = model.material("button_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.30, 0.53, 0.70, 1.0))

    body = model.part("body")

    # Base shell.
    body.visual(
        Box((0.460, 0.380, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.009)),
        material=shell_dark,
        name="bottom_plate",
    )
    body.visual(
        Box((0.018, 0.380, 0.145)),
        origin=Origin(xyz=(-0.221, 0.000, 0.0905)),
        material=shell_light,
        name="left_wall",
    )
    body.visual(
        Box((0.018, 0.380, 0.145)),
        origin=Origin(xyz=(0.221, 0.000, 0.0905)),
        material=shell_light,
        name="right_wall",
    )
    body.visual(
        Box((0.424, 0.018, 0.145)),
        origin=Origin(xyz=(0.000, 0.181, 0.0905)),
        material=shell_light,
        name="rear_wall",
    )

    # Front face split to leave the cassette and output openings.
    body.visual(
        Box((0.061, 0.018, 0.145)),
        origin=Origin(xyz=(-0.1995, -0.181, 0.0905)),
        material=shell_light,
        name="front_left_column",
    )
    body.visual(
        Box((0.061, 0.018, 0.145)),
        origin=Origin(xyz=(0.1995, -0.181, 0.0905)),
        material=shell_light,
        name="front_right_column",
    )
    body.visual(
        Box((0.338, 0.018, 0.018)),
        origin=Origin(xyz=(0.000, -0.181, 0.009)),
        material=shell_light,
        name="front_bottom_lip",
    )
    body.visual(
        Box((0.338, 0.018, 0.015)),
        origin=Origin(xyz=(0.000, -0.181, 0.0845)),
        material=shell_light,
        name="front_mid_bridge",
    )
    body.visual(
        Box((0.338, 0.018, 0.050)),
        origin=Origin(xyz=(0.000, -0.181, 0.137)),
        material=shell_light,
        name="front_top_beam",
    )
    body.visual(
        Box((0.260, 0.094, 0.005)),
        origin=Origin(xyz=(0.000, -0.127, 0.0945)),
        material=trim_dark,
        name="output_floor",
    )

    # Top deck around the scanner bay and control panel.
    body.visual(
        Box((0.296, 0.070, 0.014)),
        origin=Origin(xyz=(-0.032, -0.146, 0.156)),
        material=shell_light,
        name="front_top_strip",
    )
    body.visual(
        Box((0.344, 0.080, 0.014)),
        origin=Origin(xyz=(-0.010, 0.151, 0.156)),
        material=shell_light,
        name="rear_top_strip",
    )
    body.visual(
        Box((0.050, 0.244, 0.014)),
        origin=Origin(xyz=(-0.205, 0.000, 0.156)),
        material=shell_light,
        name="left_top_strip",
    )
    body.visual(
        Box((0.110, 0.182, 0.014)),
        origin=Origin(xyz=(0.175, 0.020, 0.156)),
        material=shell_light,
        name="right_rear_top_strip",
    )
    body.visual(
        Box((0.120, 0.140, 0.014)),
        origin=Origin(xyz=(0.170, -0.110, 0.156)),
        material=shell_light,
        name="right_front_top_deck",
    )

    # Scanner recess.
    body.visual(
        Box((0.294, 0.214, 0.003)),
        origin=Origin(xyz=(-0.032, 0.000, 0.1495)),
        material=shell_dark,
        name="scanner_bed",
    )
    body.visual(
        Box((0.286, 0.206, 0.002)),
        origin=Origin(xyz=(-0.032, 0.000, 0.1520)),
        material=scanner_glass,
        name="scanner_glass",
    )
    body.visual(
        Box((0.008, 0.222, 0.012)),
        origin=Origin(xyz=(-0.183, 0.000, 0.157)),
        material=trim_dark,
        name="scanner_left_wall",
    )
    body.visual(
        Box((0.008, 0.222, 0.012)),
        origin=Origin(xyz=(0.119, 0.000, 0.157)),
        material=trim_dark,
        name="scanner_right_wall",
    )
    body.visual(
        Box((0.310, 0.008, 0.012)),
        origin=Origin(xyz=(-0.032, -0.111, 0.157)),
        material=trim_dark,
        name="scanner_front_wall",
    )
    body.visual(
        Box((0.310, 0.008, 0.012)),
        origin=Origin(xyz=(-0.032, 0.111, 0.157)),
        material=trim_dark,
        name="scanner_rear_wall",
    )

    lid = model.part("scanner_lid")
    lid.visual(
        Box((0.322, 0.244, 0.004)),
        origin=Origin(xyz=(0.000, -0.122, 0.014)),
        material=shell_light,
        name="lid_top",
    )
    lid.visual(
        Box((0.012, 0.232, 0.016)),
        origin=Origin(xyz=(-0.155, -0.116, 0.008)),
        material=shell_light,
        name="lid_left_rail",
    )
    lid.visual(
        Box((0.012, 0.232, 0.016)),
        origin=Origin(xyz=(0.155, -0.116, 0.008)),
        material=shell_light,
        name="lid_right_rail",
    )
    lid.visual(
        Box((0.298, 0.012, 0.016)),
        origin=Origin(xyz=(0.000, -0.236, 0.008)),
        material=shell_light,
        name="lid_front_rail",
    )
    lid.visual(
        Box((0.298, 0.010, 0.012)),
        origin=Origin(xyz=(0.000, -0.005, 0.006)),
        material=trim_dark,
        name="lid_hinge_rail",
    )
    lid.visual(
        Box((0.298, 0.230, 0.002)),
        origin=Origin(xyz=(0.000, -0.116, 0.001)),
        material=shell_dark,
        name="lid_backing",
    )
    model.articulation(
        "body_to_scanner_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.032, 0.121, 0.163)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.30,
        ),
    )

    cassette = model.part("paper_cassette")
    cassette.visual(
        Box((0.332, 0.280, 0.003)),
        origin=Origin(xyz=(0.000, 0.130, 0.0015)),
        material=shell_dark,
        name="cassette_floor",
    )
    cassette.visual(
        Box((0.003, 0.280, 0.048)),
        origin=Origin(xyz=(-0.1645, 0.130, 0.024)),
        material=shell_dark,
        name="cassette_left_wall",
    )
    cassette.visual(
        Box((0.003, 0.280, 0.048)),
        origin=Origin(xyz=(0.1645, 0.130, 0.024)),
        material=shell_dark,
        name="cassette_right_wall",
    )
    cassette.visual(
        Box((0.338, 0.016, 0.058)),
        origin=Origin(xyz=(0.000, -0.010, 0.029)),
        material=shell_light,
        name="cassette_front",
    )
    cassette.visual(
        Box((0.338, 0.003, 0.040)),
        origin=Origin(xyz=(0.000, 0.2715, 0.020)),
        material=shell_dark,
        name="cassette_back_wall",
    )
    cassette.visual(
        Box((0.300, 0.018, 0.010)),
        origin=Origin(xyz=(0.000, 0.262, 0.045)),
        material=trim_dark,
        name="paper_stop",
    )
    model.articulation(
        "body_to_paper_cassette",
        ArticulationType.PRISMATIC,
        parent=body,
        child=cassette,
        origin=Origin(xyz=(0.000, -0.172, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=0.25,
            lower=0.0,
            upper=0.160,
        ),
    )

    tray = model.part("output_tray")
    tray.visual(
        Box((0.262, 0.003, 0.040)),
        origin=Origin(xyz=(0.000, -0.0015, 0.020)),
        material=shell_light,
        name="tray_panel",
    )
    tray.visual(
        Box((0.236, 0.014, 0.004)),
        origin=Origin(xyz=(0.000, -0.007, 0.002)),
        material=shell_light,
        name="tray_lip",
    )
    tray.visual(
        Box((0.262, 0.008, 0.004)),
        origin=Origin(xyz=(0.000, -0.004, 0.002)),
        material=trim_dark,
        name="tray_hinge_strip",
    )
    model.articulation(
        "body_to_output_tray",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.000, -0.190, 0.092)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    control_panel = model.part("control_panel")
    control_panel.visual(
        Box((0.102, 0.044, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, 0.0015)),
        material=trim_dark,
        name="panel_floor",
    )
    control_panel.visual(
        Box((0.118, 0.008, 0.014)),
        origin=Origin(xyz=(0.000, -0.026, 0.007)),
        material=shell_dark,
        name="panel_front_rim",
    )
    control_panel.visual(
        Box((0.118, 0.008, 0.014)),
        origin=Origin(xyz=(0.000, 0.026, 0.007)),
        material=shell_dark,
        name="panel_rear_rim",
    )
    control_panel.visual(
        Box((0.008, 0.044, 0.014)),
        origin=Origin(xyz=(-0.055, 0.000, 0.007)),
        material=shell_dark,
        name="panel_left_rim",
    )
    control_panel.visual(
        Box((0.008, 0.044, 0.014)),
        origin=Origin(xyz=(0.055, 0.000, 0.007)),
        material=shell_dark,
        name="panel_right_rim",
    )
    model.articulation(
        "body_to_control_panel",
        ArticulationType.FIXED,
        parent=body,
        child=control_panel,
        origin=Origin(xyz=(0.155, -0.154, 0.163)),
    )

    dial = model.part("selector_dial")
    dial.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, -0.005)),
        material=button_dark,
        name="dial_stem",
    )
    dial.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=button_dark,
        name="dial_base",
    )
    dial.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
        material=shell_dark,
        name="dial_knob",
    )
    dial.visual(
        Box((0.010, 0.002, 0.002)),
        origin=Origin(xyz=(0.008, 0.000, 0.017)),
        material=accent_blue,
        name="dial_indicator",
    )
    model.articulation(
        "control_panel_to_selector_dial",
        ArticulationType.CONTINUOUS,
        parent=control_panel,
        child=dial,
        origin=Origin(xyz=(-0.022, 0.000, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.08,
            velocity=6.0,
        ),
    )

    button_positions = (
        ("button_0", (0.020, 0.012)),
        ("button_1", (0.040, 0.012)),
        ("button_2", (0.020, -0.012)),
        ("button_3", (0.040, -0.012)),
    )
    for button_name, (local_x, local_y) in button_positions:
        button = model.part(button_name)
        button.visual(
            Box((0.008, 0.006, 0.011)),
            origin=Origin(xyz=(0.000, 0.000, -0.0055)),
            material=button_dark,
            name="button_stem",
        )
        button.visual(
            Box((0.016, 0.012, 0.005)),
            origin=Origin(xyz=(0.000, 0.000, 0.0025)),
            material=shell_dark,
            name="button_cap",
        )
        model.articulation(
            f"control_panel_to_{button_name}",
            ArticulationType.PRISMATIC,
            parent=control_panel,
            child=button,
            origin=Origin(xyz=(local_x, local_y, 0.014)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.06,
                lower=0.0,
                upper=0.003,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("scanner_lid")
    cassette = object_model.get_part("paper_cassette")
    tray = object_model.get_part("output_tray")
    dial = object_model.get_part("selector_dial")

    lid_hinge = object_model.get_articulation("body_to_scanner_lid")
    cassette_slide = object_model.get_articulation("body_to_paper_cassette")
    tray_hinge = object_model.get_articulation("body_to_output_tray")
    dial_joint = object_model.get_articulation("control_panel_to_selector_dial")

    ctx.expect_gap(
        lid,
        body,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="scanner lid closes onto the scanner bay rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        min_overlap=0.180,
        name="scanner lid covers the scanner bay footprint",
    )
    ctx.expect_overlap(
        cassette,
        body,
        axes="xz",
        min_overlap=0.050,
        name="inserted cassette stays aligned in the lower body opening",
    )
    ctx.expect_gap(
        body,
        tray,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="closed output tray sits flush with the printer front",
    )

    lid_limits = lid_hinge.motion_limits
    cassette_limits = cassette_slide.motion_limits
    tray_limits = tray_hinge.motion_limits

    if lid_limits is not None and lid_limits.upper is not None:
        closed_lid_aabb = ctx.part_world_aabb(lid)
        with ctx.pose({lid_hinge: lid_limits.upper}):
            open_lid_aabb = ctx.part_world_aabb(lid)
        ctx.check(
            "scanner lid opens upward",
            closed_lid_aabb is not None
            and open_lid_aabb is not None
            and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.10,
            details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
        )

    if cassette_limits is not None and cassette_limits.upper is not None:
        rest_pos = ctx.part_world_position(cassette)
        with ctx.pose({cassette_slide: cassette_limits.upper}):
            extended_pos = ctx.part_world_position(cassette)
            ctx.expect_overlap(
                cassette,
                body,
                axes="x",
                min_overlap=0.250,
                name="extended cassette remains laterally captured by the body",
            )
        ctx.check(
            "paper cassette pulls forward",
            rest_pos is not None
            and extended_pos is not None
            and extended_pos[1] < rest_pos[1] - 0.12,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    if tray_limits is not None and tray_limits.upper is not None:
        closed_tray_aabb = ctx.part_world_aabb(tray)
        with ctx.pose({tray_hinge: tray_limits.upper}):
            open_tray_aabb = ctx.part_world_aabb(tray)
        ctx.check(
            "output tray folds downward",
            closed_tray_aabb is not None
            and open_tray_aabb is not None
            and open_tray_aabb[0][2] < closed_tray_aabb[0][2] - 0.01
            and open_tray_aabb[0][1] < closed_tray_aabb[0][1] - 0.015,
            details=f"closed={closed_tray_aabb}, open={open_tray_aabb}",
        )

    dial_aabb_0 = ctx.part_element_world_aabb(dial, elem="dial_indicator")
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_aabb_90 = ctx.part_element_world_aabb(dial, elem="dial_indicator")
    if dial_aabb_0 is not None and dial_aabb_90 is not None:
        indicator_center_0 = (
            (dial_aabb_0[0][0] + dial_aabb_0[1][0]) * 0.5,
            (dial_aabb_0[0][1] + dial_aabb_0[1][1]) * 0.5,
        )
        indicator_center_90 = (
            (dial_aabb_90[0][0] + dial_aabb_90[1][0]) * 0.5,
            (dial_aabb_90[0][1] + dial_aabb_90[1][1]) * 0.5,
        )
        ctx.check(
            "selector dial indicator rotates around the knob",
            abs(indicator_center_0[0] - indicator_center_90[0]) > 0.004
            and abs(indicator_center_0[1] - indicator_center_90[1]) > 0.004,
            details=f"q0={indicator_center_0}, q90={indicator_center_90}",
        )

    for button_index in range(4):
        joint = object_model.get_articulation(f"control_panel_to_button_{button_index}")
        limits = joint.motion_limits
        ctx.check(
            f"button_{button_index} has push travel",
            limits is not None and limits.upper is not None and limits.upper >= 0.002,
            details=f"limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
