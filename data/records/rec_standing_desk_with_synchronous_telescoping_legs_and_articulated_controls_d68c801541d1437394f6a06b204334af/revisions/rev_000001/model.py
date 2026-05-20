from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_leg_standing_desk")

    wood = Material("warm_maple_desktop", rgba=(0.78, 0.58, 0.36, 1.0))
    dark_metal = Material("satin_black_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    inner_metal = Material("brushed_inner_column", rgba=(0.58, 0.61, 0.62, 1.0))
    rubber = Material("matte_black_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    handset_plastic = Material("black_handset_plastic", rgba=(0.01, 0.012, 0.015, 1.0))
    button_plastic = Material("soft_gray_button", rgba=(0.60, 0.64, 0.66, 1.0))
    icon_white = Material("white_button_icons", rgba=(0.92, 0.94, 0.94, 1.0))

    # Object frame: +Z up, +X across the desk width, and -Y toward the user/front edge.
    base = model.part("base_frame")

    leg_x_positions = (-0.55, 0.55)
    column_outer = (0.12, 0.09)
    wall = 0.018
    outer_height = 0.62
    outer_center_z = 0.36

    for side, x in (("left", leg_x_positions[0]), ("right", leg_x_positions[1])):
        base.visual(
            Box((0.16, 0.66, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=dark_metal,
            name=f"{side}_foot_bar",
        )
        # Four walls make each lift column read as a hollow rectangular sleeve.
        base.visual(
            Box((wall, column_outer[1], outer_height)),
            origin=Origin(xyz=(x - column_outer[0] / 2 + wall / 2, 0.0, outer_center_z)),
            material=dark_metal,
            name=f"{side}_outer_wall_x0",
        )
        base.visual(
            Box((wall, column_outer[1], outer_height)),
            origin=Origin(xyz=(x + column_outer[0] / 2 - wall / 2, 0.0, outer_center_z)),
            material=dark_metal,
            name=f"{side}_outer_wall_x1",
        )
        base.visual(
            Box((column_outer[0], wall, outer_height)),
            origin=Origin(xyz=(x, -column_outer[1] / 2 + wall / 2, outer_center_z)),
            material=dark_metal,
            name=f"{side}_outer_wall_front",
        )
        base.visual(
            Box((column_outer[0], wall, outer_height)),
            origin=Origin(xyz=(x, column_outer[1] / 2 - wall / 2, outer_center_z)),
            material=dark_metal,
            name=f"{side}_outer_wall_rear",
        )
        for end_y in (-0.28, 0.28):
            base.visual(
                Cylinder(radius=0.033, length=0.012),
                origin=Origin(xyz=(x, end_y, -0.006)),
                material=rubber,
                name=f"{side}_leveler_{'front' if end_y < 0 else 'rear'}",
            )

    # A rear stretcher keeps the two static leg sleeves physically tied together.
    base.visual(
        Box((0.98, 0.045, 0.055)),
        origin=Origin(xyz=(0.0, 0.0675, 0.18)),
        material=dark_metal,
        name="rear_lower_stretcher",
    )
    base.visual(
        Box((0.98, 0.040, 0.050)),
        origin=Origin(xyz=(0.0, 0.065, 0.62)),
        material=dark_metal,
        name="rear_upper_stretcher",
    )

    lift_limits = MotionLimits(effort=900.0, velocity=0.055, lower=0.0, upper=0.48)

    left_stage = model.part("left_lift_stage")
    right_stage = model.part("right_lift_stage")
    for stage in (left_stage, right_stage):
        stage.visual(
            Box((0.070, 0.045, 0.64)),
            origin=Origin(xyz=(0.0, 0.0, 0.34)),
            material=inner_metal,
            name="inner_tube",
        )
        stage.visual(
            Box((0.19, 0.14, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.648)),
            material=dark_metal,
            name="top_plate",
        )
        stage.visual(
            Box((0.105, 0.075, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, 0.625)),
            material=dark_metal,
            name="upper_collar",
        )

    model.articulation(
        "base_to_left_lift_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=left_stage,
        origin=Origin(xyz=(leg_x_positions[0], 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
    )
    model.articulation(
        "base_to_right_lift_stage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=right_stage,
        origin=Origin(xyz=(leg_x_positions[1], 0.0, 0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=lift_limits,
        mimic=Mimic(joint="base_to_left_lift_stage"),
    )

    desktop = model.part("desktop")
    desktop.visual(
        Box((1.40, 0.75, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=wood,
        name="tabletop",
    )
    # Slim steel crossbeam frame directly under the wooden top.
    for y, name in ((-0.245, "front_crossbeam"), (0.245, "rear_crossbeam")):
        desktop.visual(
            Box((1.02, 0.045, 0.035)),
            origin=Origin(xyz=(0.0, y, -0.040)),
            material=dark_metal,
            name=name,
        )
    desktop.visual(
        Box((0.82, 0.050, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.0375)),
        material=dark_metal,
        name="center_crossbeam",
    )

    model.articulation(
        "left_lift_stage_to_desktop",
        ArticulationType.FIXED,
        parent=left_stage,
        child=desktop,
        origin=Origin(xyz=(0.55, 0.0, 0.6775)),
    )

    handset = model.part("handset")
    handset.visual(
        Box((0.22, 0.080, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=handset_plastic,
        name="body",
    )
    handset.visual(
        Box((0.205, 0.004, 0.041)),
        origin=Origin(xyz=(0.0, -0.042, 0.003)),
        material=Material("slightly_sheen_panel", rgba=(0.035, 0.038, 0.043, 1.0)),
        name="face_panel",
    )
    model.articulation(
        "desktop_to_handset",
        ArticulationType.FIXED,
        parent=desktop,
        child=handset,
        origin=Origin(xyz=(0.38, -0.405, -0.050)),
    )

    button_xs = (-0.070, -0.023, 0.023, 0.070)
    button_limits = MotionLimits(effort=1.2, velocity=0.05, lower=0.0, upper=0.005)
    for index, x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.034, 0.012, 0.020)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=button_plastic,
            name="cap",
        )
        if index in (0, 2):
            button.visual(
                Box((0.015, 0.0015, 0.003)),
                origin=Origin(xyz=(0.0, -0.0125, 0.0045)),
                material=icon_white,
                name="up_mark",
            )
        else:
            button.visual(
                Box((0.015, 0.0015, 0.003)),
                origin=Origin(xyz=(0.0, -0.0125, -0.0045)),
                material=icon_white,
                name="down_mark",
            )
        model.articulation(
            f"handset_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=handset,
            child=button,
            origin=Origin(xyz=(x, -0.040, 0.004)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=button_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    left_stage = object_model.get_part("left_lift_stage")
    right_stage = object_model.get_part("right_lift_stage")
    desktop = object_model.get_part("desktop")
    handset = object_model.get_part("handset")
    lift = object_model.get_articulation("base_to_left_lift_stage")

    ctx.expect_contact(
        left_stage,
        desktop,
        elem_a="top_plate",
        elem_b="tabletop",
        name="left lift stage bears on desktop underside",
    )
    ctx.expect_contact(
        right_stage,
        desktop,
        elem_a="top_plate",
        elem_b="tabletop",
        name="right lift stage bears on desktop underside",
    )
    ctx.expect_gap(
        desktop,
        base,
        axis="z",
        positive_elem="tabletop",
        min_gap=0.030,
        name="desktop clears static outer columns",
    )
    ctx.expect_within(
        handset,
        desktop,
        axes="x",
        inner_elem="body",
        outer_elem="tabletop",
        margin=0.0,
        name="handset sits under the front edge within the desk width",
    )

    left_rest = ctx.part_world_position(left_stage)
    right_rest = ctx.part_world_position(right_stage)
    desktop_rest = ctx.part_world_position(desktop)
    ctx.check(
        "matched lift stages start level",
        left_rest is not None
        and right_rest is not None
        and abs(left_rest[2] - right_rest[2]) < 1e-6,
        details=f"left={left_rest}, right={right_rest}",
    )

    with ctx.pose({lift: 0.48}):
        left_high = ctx.part_world_position(left_stage)
        right_high = ctx.part_world_position(right_stage)
        desktop_high = ctx.part_world_position(desktop)
        ctx.expect_contact(
            right_stage,
            desktop,
            elem_a="top_plate",
            elem_b="tabletop",
            name="mimicked right stage still supports raised desktop",
        )
        ctx.check(
            "synchronous lift stages remain level",
            left_high is not None
            and right_high is not None
            and abs(left_high[2] - right_high[2]) < 1e-6,
            details=f"left={left_high}, right={right_high}",
        )
        ctx.check(
            "desktop rises with the lift stages",
            desktop_rest is not None
            and desktop_high is not None
            and desktop_high[2] > desktop_rest[2] + 0.45,
            details=f"rest={desktop_rest}, raised={desktop_high}",
        )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"handset_to_button_{index}")
        ctx.allow_overlap(
            handset,
            button,
            elem_a="body",
            elem_b="cap",
            reason="The push-button cap intentionally travels a few millimeters into the handset face opening.",
        )
        ctx.expect_contact(
            button,
            handset,
            elem_a="cap",
            elem_b="body",
            name=f"button {index} cap is seated on handset face",
        )
        button_rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.005}):
            button_pressed = ctx.part_world_position(button)
            ctx.expect_gap(
                handset,
                button,
                axis="y",
                positive_elem="body",
                negative_elem="cap",
                max_penetration=0.006,
                max_gap=0.0,
                name=f"button {index} presses inward into handset",
            )
        ctx.check(
            f"button {index} has independent inward travel",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1] > button_rest[1] + 0.004,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
