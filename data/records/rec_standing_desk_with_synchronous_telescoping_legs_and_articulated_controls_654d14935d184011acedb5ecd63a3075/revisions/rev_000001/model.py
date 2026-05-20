from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_leg_standing_desk")

    wood = model.material("warm_maple_top", rgba=(0.78, 0.58, 0.34, 1.0))
    edge_wood = model.material("darker_edge_band", rgba=(0.55, 0.36, 0.18, 1.0))
    dark_metal = model.material("satin_black_metal", rgba=(0.03, 0.035, 0.04, 1.0))
    silver_metal = model.material("brushed_lift_metal", rgba=(0.55, 0.58, 0.60, 1.0))
    plastic = model.material("black_plastic", rgba=(0.01, 0.012, 0.014, 1.0))
    button_mat = model.material("rubber_buttons", rgba=(0.12, 0.13, 0.14, 1.0))

    leg_x = 0.52
    outer_x = 0.12
    outer_y = 0.10
    wall = 0.012
    outer_height = 0.60
    outer_bottom = 0.055
    outer_center_z = outer_bottom + outer_height / 2.0

    base = model.part("base")
    for index, (x, side_0_name, side_1_name) in enumerate(
        (
            (-leg_x, "column_side_0_0", "column_side_0_1"),
            (leg_x, "column_side_1_0", "column_side_1_1"),
        )
    ):
        base.visual(
            Box((0.16, 0.72, 0.055)),
            origin=Origin(xyz=(x, 0.0, 0.0275)),
            material=dark_metal,
            name=f"foot_{index}",
        )
        base.visual(
            Box((wall, outer_y, outer_height)),
            origin=Origin(xyz=(x - outer_x / 2.0 + wall / 2.0, 0.0, outer_center_z)),
            material=dark_metal,
            name=side_0_name,
        )
        base.visual(
            Box((wall, outer_y, outer_height)),
            origin=Origin(xyz=(x + outer_x / 2.0 - wall / 2.0, 0.0, outer_center_z)),
            material=dark_metal,
            name=side_1_name,
        )
        base.visual(
            Box((outer_x - 2.0 * wall, wall, outer_height)),
            origin=Origin(xyz=(x, -outer_y / 2.0 + wall / 2.0, outer_center_z)),
            material=dark_metal,
            name=f"column_front_{index}",
        )
        base.visual(
            Box((outer_x - 2.0 * wall, wall, outer_height)),
            origin=Origin(xyz=(x, outer_y / 2.0 - wall / 2.0, outer_center_z)),
            material=dark_metal,
            name=f"column_rear_{index}",
        )

    base.visual(
        Box((1.04, 0.05, 0.07)),
        origin=Origin(xyz=(0.0, 0.065, 0.48)),
        material=dark_metal,
        name="rear_crossbeam",
    )

    stage_0 = model.part("lift_stage_0")
    stage_1 = model.part("lift_stage_1")
    for stage in (stage_0, stage_1):
        stage.visual(
            Box((0.070, 0.050, 0.575)),
            origin=Origin(xyz=(0.0, 0.0, -0.2375)),
            material=silver_metal,
            name="inner_column",
        )
        stage.visual(
            Box((0.24, 0.16, 0.025)),
            origin=Origin(xyz=(0.0, 0.0, 0.0625)),
            material=dark_metal,
            name="top_cap",
        )
        for shoe_name, x in (("guide_shoe_0", -0.0415), ("guide_shoe_1", 0.0415)):
            stage.visual(
                Box((0.013, 0.045, 0.090)),
                origin=Origin(xyz=(x, 0.0, -0.250)),
                material=dark_metal,
                name=shoe_name,
            )

    lift_limits = MotionLimits(effort=900.0, velocity=0.06, lower=0.0, upper=0.48)
    model.articulation(
        "base_to_stage_0",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_0,
        origin=Origin(xyz=(-leg_x, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_stage_1",
        ArticulationType.PRISMATIC,
        parent=base,
        child=stage_1,
        origin=Origin(xyz=(leg_x, 0.0, 0.62)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="base_to_stage_0", multiplier=1.0, offset=0.0),
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box((1.45, 0.75, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=wood,
        name="top_slab",
    )
    desktop.visual(
        Box((1.45, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, -0.384, 0.016)),
        material=edge_wood,
        name="front_edge",
    )
    desktop.visual(
        Box((1.45, 0.018, 0.040)),
        origin=Origin(xyz=(0.0, 0.384, 0.016)),
        material=edge_wood,
        name="rear_edge",
    )
    desktop.visual(
        Box((1.20, 0.060, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=dark_metal,
        name="crossbeam_frame",
    )
    for index, x in enumerate((-leg_x, leg_x)):
        desktop.visual(
            Box((0.050, 0.56, 0.050)),
            origin=Origin(xyz=(x, 0.0, -0.025)),
            material=dark_metal,
            name=f"side_frame_{index}",
        )
    desktop.visual(
        Box((0.90, 0.045, 0.040)),
        origin=Origin(xyz=(0.0, -0.305, -0.020)),
        material=dark_metal,
        name="front_apron",
    )
    for index, (guide_name, x) in enumerate((("tray_guide_0", -0.3675), ("tray_guide_1", 0.3675))):
        desktop.visual(
            Box((0.035, 0.360, 0.025)),
            origin=Origin(xyz=(x, -0.205, -0.0875)),
            material=dark_metal,
            name=guide_name,
        )
        for hanger_index, y in enumerate((-0.055, -0.355)):
            desktop.visual(
                Box((0.035, 0.025, 0.075)),
                origin=Origin(xyz=(x, y, -0.0375)),
                material=dark_metal,
                name=f"guide_hanger_{index}_{hanger_index}",
            )

    model.articulation(
        "stage_to_desktop",
        ArticulationType.FIXED,
        parent=stage_0,
        child=desktop,
        origin=Origin(xyz=(leg_x, 0.0, 0.125)),
    )

    tray = model.part("keyboard_tray")
    tray.visual(
        Box((0.72, 0.30, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="tray_panel",
    )
    tray.visual(
        Box((0.64, 0.025, 0.040)),
        origin=Origin(xyz=(0.0, -0.150, 0.011)),
        material=dark_metal,
        name="front_lip",
    )
    for runner_name, x in (("runner_0", -0.350), ("runner_1", 0.350)):
        tray.visual(
            Box((0.025, 0.300, 0.025)),
            origin=Origin(xyz=(x, 0.0, 0.0125)),
            material=silver_metal,
            name=runner_name,
        )
    model.articulation(
        "desktop_to_tray",
        ArticulationType.PRISMATIC,
        parent=desktop,
        child=tray,
        origin=Origin(xyz=(0.0, -0.200, -0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.22),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.23, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=plastic,
        name="body",
    )
    handset.visual(
        Box((0.20, 0.030, 0.060)),
        origin=Origin(xyz=(0.0, 0.025, 0.045)),
        material=plastic,
        name="mount_tab",
    )
    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(0.53, -0.385, -0.075)),
    )

    for name, x in (("down_button", -0.040), ("up_button", 0.040)):
        button = model.part(name)
        button.visual(
            Box((0.038, 0.008, 0.014)),
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=button_mat,
            name="cap",
        )
        model.articulation(
            f"handset_to_{name}",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button,
            origin=Origin(xyz=(x, -0.0275, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_0 = object_model.get_part("lift_stage_0")
    lift_1 = object_model.get_part("lift_stage_1")
    desktop = object_model.get_part("desktop")
    tray = object_model.get_part("keyboard_tray")
    handset = object_model.get_part("handset")
    up_button = object_model.get_part("up_button")
    down_button = object_model.get_part("down_button")

    lift_source = object_model.get_articulation("base_to_stage_0")
    lift_follower = object_model.get_articulation("base_to_stage_1")
    tray_slide = object_model.get_articulation("desktop_to_tray")
    up_press = object_model.get_articulation("handset_to_up_button")
    down_press = object_model.get_articulation("handset_to_down_button")

    ctx.check(
        "paired lift stages share one height command",
        lift_follower.mimic is not None
        and lift_follower.mimic.joint == "base_to_stage_0"
        and abs(lift_follower.mimic.multiplier - 1.0) < 1e-9
        and abs(lift_follower.mimic.offset) < 1e-9,
        details=f"follower mimic={lift_follower.mimic}",
    )
    ctx.check(
        "standing desk lift has office height travel",
        lift_source.motion_limits is not None
        and lift_source.motion_limits.upper is not None
        and lift_source.motion_limits.upper >= 0.45,
        details=f"limits={lift_source.motion_limits}",
    )

    ctx.expect_contact(
        lift_0,
        base,
        elem_a="guide_shoe_1",
        elem_b="column_side_0_1",
        name="left lift stage is guided by its outer column",
    )
    ctx.expect_contact(
        lift_1,
        base,
        elem_a="guide_shoe_0",
        elem_b="column_side_1_0",
        name="right lift stage is guided by its outer column",
    )
    ctx.expect_contact(
        lift_0,
        desktop,
        elem_a="top_cap",
        elem_b="crossbeam_frame",
        name="left lift stage supports the desktop crossbeam",
    )
    ctx.expect_contact(
        lift_1,
        desktop,
        elem_a="top_cap",
        elem_b="crossbeam_frame",
        name="right lift stage supports the same desktop crossbeam",
    )

    rest_lift_0 = ctx.part_world_position(lift_0)
    rest_lift_1 = ctx.part_world_position(lift_1)
    rest_desktop = ctx.part_world_position(desktop)
    with ctx.pose({lift_source: 0.48}):
        raised_lift_0 = ctx.part_world_position(lift_0)
        raised_lift_1 = ctx.part_world_position(lift_1)
        raised_desktop = ctx.part_world_position(desktop)
        ctx.expect_contact(
            lift_0,
            desktop,
            elem_a="top_cap",
            elem_b="crossbeam_frame",
            name="raised left support remains under the desktop",
        )
        ctx.expect_contact(
            lift_1,
            desktop,
            elem_a="top_cap",
            elem_b="crossbeam_frame",
            name="raised right support remains under the desktop",
        )
    ctx.check(
        "lift stages stay height aligned",
        rest_lift_0 is not None
        and rest_lift_1 is not None
        and raised_lift_0 is not None
        and raised_lift_1 is not None
        and abs(rest_lift_0[2] - rest_lift_1[2]) < 1e-6
        and abs(raised_lift_0[2] - raised_lift_1[2]) < 1e-6,
        details=f"rest=({rest_lift_0}, {rest_lift_1}) raised=({raised_lift_0}, {raised_lift_1})",
    )
    ctx.check(
        "desktop rises with the synchronous lift",
        rest_desktop is not None
        and raised_desktop is not None
        and raised_desktop[2] > rest_desktop[2] + 0.45,
        details=f"rest={rest_desktop}, raised={raised_desktop}",
    )

    ctx.expect_contact(
        tray,
        desktop,
        elem_a="runner_0",
        elem_b="tray_guide_0",
        name="keyboard tray rides on the left guide",
    )
    ctx.expect_contact(
        tray,
        desktop,
        elem_a="runner_1",
        elem_b="tray_guide_1",
        name="keyboard tray rides on the right guide",
    )
    ctx.expect_overlap(
        tray,
        desktop,
        axes="y",
        elem_a="runner_1",
        elem_b="tray_guide_1",
        min_overlap=0.25,
        name="keyboard tray is well retained when stowed",
    )
    rest_tray = ctx.part_world_position(tray)
    with ctx.pose({tray_slide: 0.22}):
        extended_tray = ctx.part_world_position(tray)
        ctx.expect_contact(
            tray,
            desktop,
            elem_a="runner_1",
            elem_b="tray_guide_1",
            name="extended keyboard tray remains on a guide",
        )
        ctx.expect_overlap(
            tray,
            desktop,
            axes="y",
            elem_a="runner_1",
            elem_b="tray_guide_1",
            min_overlap=0.10,
            name="extended keyboard tray retains guide insertion",
        )
    ctx.check(
        "keyboard tray slides forward from under the front edge",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[1] < rest_tray[1] - 0.20,
        details=f"rest={rest_tray}, extended={extended_tray}",
    )

    ctx.expect_contact(
        handset,
        desktop,
        elem_a="mount_tab",
        elem_b="top_slab",
        name="handset is mounted under the front edge",
    )
    ctx.expect_contact(
        up_button,
        handset,
        elem_a="cap",
        elem_b="body",
        name="up button sits proud on the handset face",
    )
    ctx.expect_contact(
        down_button,
        handset,
        elem_a="cap",
        elem_b="body",
        name="down button sits proud on the handset face",
    )
    ctx.allow_overlap(
        handset,
        up_button,
        elem_a="body",
        elem_b="cap",
        reason="The handset button cap intentionally travels a few millimeters into its face recess when pressed.",
    )
    ctx.allow_overlap(
        handset,
        down_button,
        elem_a="body",
        elem_b="cap",
        reason="The handset button cap intentionally travels a few millimeters into its face recess when pressed.",
    )
    with ctx.pose({up_press: 0.004, down_press: 0.004}):
        ctx.expect_gap(
            handset,
            up_button,
            axis="y",
            positive_elem="body",
            negative_elem="cap",
            max_penetration=0.0045,
            name="up button press is a shallow face recess",
        )
        ctx.expect_gap(
            handset,
            down_button,
            axis="y",
            positive_elem="body",
            negative_elem="cap",
            max_penetration=0.0045,
            name="down button press is a shallow face recess",
        )

    return ctx.report()


object_model = build_object_model()
