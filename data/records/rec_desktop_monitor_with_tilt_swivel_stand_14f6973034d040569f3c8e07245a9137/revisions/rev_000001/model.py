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
    model = ArticulatedObject(name="creator_desktop_monitor")

    graphite = Material("warm_graphite", rgba=(0.05, 0.052, 0.055, 1.0))
    dark_plastic = Material("matte_black_plastic", rgba=(0.005, 0.006, 0.008, 1.0))
    soft_black = Material("soft_black", rgba=(0.015, 0.016, 0.018, 1.0))
    screen_glass = Material("dark_screen_glass", rgba=(0.015, 0.018, 0.024, 1.0))
    charcoal = Material("charcoal_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    satin_metal = Material("satin_anodized_metal", rgba=(0.45, 0.46, 0.45, 1.0))
    button_mat = Material("button_dark_gray", rgba=(0.09, 0.092, 0.095, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.58, 0.34, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Box((0.50, 0.26, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=dark_plastic,
        name="inset_top_pad",
    )
    # Four sleeve walls form a real clearance channel for the height-adjusting
    # column instead of using a solid block that the mast would slide through.
    sleeve_z = 0.1975
    sleeve_h = 0.325
    base.visual(
        Box((0.026, 0.112, sleeve_h)),
        origin=Origin(xyz=(-0.057, 0.075, sleeve_z)),
        material=graphite,
        name="sleeve_wall_0",
    )
    base.visual(
        Box((0.026, 0.112, sleeve_h)),
        origin=Origin(xyz=(0.057, 0.075, sleeve_z)),
        material=graphite,
        name="sleeve_wall_1",
    )
    base.visual(
        Box((0.140, 0.024, sleeve_h)),
        origin=Origin(xyz=(0.0, 0.031, sleeve_z)),
        material=graphite,
        name="sleeve_front_wall",
    )
    base.visual(
        Box((0.140, 0.024, sleeve_h)),
        origin=Origin(xyz=(0.0, 0.119, sleeve_z)),
        material=graphite,
        name="sleeve_rear_wall",
    )

    column = model.part("column")
    column.visual(
        Box((0.088, 0.064, 0.410)),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=satin_metal,
        name="inner_mast",
    )
    column.visual(
        Box((0.100, 0.070, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=soft_black,
        name="mast_top_cap",
    )
    column.visual(
        Box((0.170, 0.050, 0.105)),
        origin=Origin(xyz=(0.0, 0.030, 0.230)),
        material=graphite,
        name="head_block",
    )
    column.visual(
        Box((0.034, 0.065, 0.120)),
        origin=Origin(xyz=(-0.075, 0.0, 0.230)),
        material=graphite,
        name="hinge_cheek_0",
    )
    column.visual(
        Box((0.034, 0.065, 0.120)),
        origin=Origin(xyz=(0.075, 0.0, 0.230)),
        material=graphite,
        name="hinge_cheek_1",
    )

    model.articulation(
        "height_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.075, 0.360)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.160),
    )

    tilt_carriage = model.part("tilt_carriage")
    tilt_carriage.visual(
        Cylinder(radius=0.024, length=0.116),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_barrel",
    )
    tilt_carriage.visual(
        Box((0.106, 0.045, 0.048)),
        origin=Origin(xyz=(0.0, -0.0225, 0.0)),
        material=graphite,
        name="carriage_arm",
    )
    tilt_carriage.visual(
        Cylinder(radius=0.090, length=0.024),
        origin=Origin(xyz=(0.0, -0.0445, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="portrait_turntable",
    )

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=column,
        child=tilt_carriage,
        origin=Origin(xyz=(0.0, -0.020, 0.230)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2, lower=-0.30, upper=0.38),
    )

    display = model.part("display")
    display.visual(
        Box((0.660, 0.055, 0.410)),
        origin=Origin(),
        material=graphite,
        name="flat_display_shell",
    )
    display.visual(
        Box((0.600, 0.004, 0.326)),
        origin=Origin(xyz=(0.0, -0.0295, 0.030)),
        material=screen_glass,
        name="screen_glass",
    )
    display.visual(
        Box((0.624, 0.006, 0.030)),
        origin=Origin(xyz=(0.0, -0.0305, -0.176)),
        material=dark_plastic,
        name="lower_bezel_face",
    )
    display.visual(
        Box((0.160, 0.016, 0.160)),
        origin=Origin(xyz=(0.0, 0.0355, 0.0)),
        material=soft_black,
        name="rear_vesa_plate",
    )
    display.visual(
        Box((0.210, 0.012, 0.035)),
        origin=Origin(xyz=(0.0, 0.0335, -0.155)),
        material=charcoal,
        name="rear_lower_vent_band",
    )

    model.articulation(
        "portrait_pivot",
        ArticulationType.CONTINUOUS,
        parent=tilt_carriage,
        child=display,
        origin=Origin(xyz=(0.0, -0.100, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0),
    )

    for index, x in enumerate((-0.064, -0.032, 0.0, 0.032, 0.064)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.024, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"button_{index}_push",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x, -0.030, -0.205)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    tilt_carriage = object_model.get_part("tilt_carriage")
    display = object_model.get_part("display")
    height_slide = object_model.get_articulation("height_slide")
    tilt_hinge = object_model.get_articulation("tilt_hinge")
    portrait_pivot = object_model.get_articulation("portrait_pivot")

    rest_column_pos = ctx.part_world_position(column)
    with ctx.pose({height_slide: 0.160}):
        raised_column_pos = ctx.part_world_position(column)
        ctx.expect_overlap(
            column,
            base,
            axes="z",
            min_overlap=0.050,
            elem_a="inner_mast",
            elem_b="sleeve_front_wall",
            name="raised mast remains retained in sleeve height",
        )
    ctx.check(
        "height slide raises the stand head",
        rest_column_pos is not None
        and raised_column_pos is not None
        and raised_column_pos[2] > rest_column_pos[2] + 0.150,
        details=f"rest={rest_column_pos}, raised={raised_column_pos}",
    )

    ctx.expect_gap(
        tilt_carriage,
        column,
        axis="x",
        max_penetration=0.0002,
        positive_elem="hinge_barrel",
        negative_elem="hinge_cheek_0",
        name="tilt barrel clears one yoke cheek",
    )
    ctx.expect_contact(
        display,
        tilt_carriage,
        elem_a="rear_vesa_plate",
        elem_b="portrait_turntable",
        contact_tol=0.0015,
        name="pivot turntable seats on rear VESA plate",
    )

    with ctx.pose({tilt_hinge: 0.30}):
        ctx.expect_origin_distance(
            display,
            tilt_carriage,
            axes="yz",
            min_dist=0.095,
            max_dist=0.110,
            name="tilt hinge keeps panel on stout carriage radius",
        )

    with ctx.pose({portrait_pivot: math.pi / 2.0}):
        ctx.expect_origin_distance(
            display,
            tilt_carriage,
            axes="yz",
            min_dist=0.095,
            max_dist=0.110,
            name="portrait pivot rotates about display center axis",
        )

    for index in range(5):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"button_{index}_push")
        ctx.expect_contact(
            button,
            display,
            elem_a="button_cap",
            elem_b="flat_display_shell",
            contact_tol=0.001,
            name=f"button {index} rests under the lower bezel",
        )
        rest_button_pos = ctx.part_world_position(button)
        with ctx.pose({joint: 0.006}):
            pressed_button_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index} pushes inward independently",
            rest_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[2] > rest_button_pos[2] + 0.005,
            details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
        )

    return ctx.report()


object_model = build_object_model()
