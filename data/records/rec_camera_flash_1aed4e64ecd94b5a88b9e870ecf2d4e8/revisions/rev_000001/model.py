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
    model = ArticulatedObject(name="compact_speedlight_flash")

    satin_black = Material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber_black = Material("rubber_black", rgba=(0.04, 0.04, 0.038, 1.0))
    dark_plastic = Material("dark_plastic", rgba=(0.085, 0.085, 0.082, 1.0))
    button_plastic = Material("button_plastic", rgba=(0.14, 0.145, 0.15, 1.0))
    shoe_metal = Material("brushed_shoe_metal", rgba=(0.55, 0.55, 0.52, 1.0))
    diffuser = Material("warm_diffuser", rgba=(1.0, 0.88, 0.58, 0.58))
    red_window = Material("red_focus_window", rgba=(0.62, 0.04, 0.025, 1.0))

    body = model.part("battery_body")
    body.visual(
        Box((0.060, 0.050, 0.086)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=satin_black,
        name="stepped_battery_grip",
    )
    body.visual(
        Box((0.052, 0.044, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.124)),
        material=satin_black,
        name="upper_body_step",
    )
    body.visual(
        Box((0.058, 0.048, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=dark_plastic,
        name="top_deck",
    )
    body.visual(
        Box((0.004, 0.038, 0.065)),
        origin=Origin(xyz=(0.031, 0.0, 0.071)),
        material=rubber_black,
        name="front_battery_door",
    )
    body.visual(
        Box((0.024, 0.004, 0.088)),
        origin=Origin(xyz=(-0.012, -0.027, 0.090)),
        material=rubber_black,
        name="side_control_strip",
    )
    body.visual(
        Box((0.011, 0.0025, 0.0035)),
        origin=Origin(xyz=(-0.012, -0.0295, 0.132)),
        material=Material("white_mode_mark", rgba=(0.88, 0.88, 0.82, 1.0)),
        name="mode_label_mark",
    )
    body.visual(
        Box((0.044, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_plastic,
        name="shoe_boss",
    )
    body.visual(
        Box((0.038, 0.025, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=shoe_metal,
        name="hot_shoe_plate",
    )
    body.visual(
        Box((0.040, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.0145, 0.007)),
        material=shoe_metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.040, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, -0.0145, 0.007)),
        material=shoe_metal,
        name="shoe_rail_1",
    )
    body.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.0, 0.026, 0.031), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="locking_wheel",
    )
    for i, z in enumerate((0.047, 0.058, 0.069, 0.080)):
        body.visual(
            Box((0.004, 0.034, 0.0022)),
            origin=Origin(xyz=(0.033, 0.0, z)),
            material=dark_plastic,
            name=f"front_grip_rib_{i}",
        )

    swivel = model.part("aiming_neck")
    swivel.visual(
        Cylinder(radius=0.021, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_plastic,
        name="swivel_turntable",
    )
    swivel.visual(
        Box((0.023, 0.027, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=dark_plastic,
        name="neck_stem",
    )
    swivel.visual(
        Box((0.030, 0.108, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=dark_plastic,
        name="yoke_bridge",
    )
    swivel.visual(
        Box((0.024, 0.008, 0.046)),
        origin=Origin(xyz=(0.0, 0.050, 0.078)),
        material=dark_plastic,
        name="tilt_cheek_pos",
    )
    swivel.visual(
        Box((0.024, 0.008, 0.046)),
        origin=Origin(xyz=(0.0, -0.050, 0.078)),
        material=dark_plastic,
        name="tilt_cheek_neg",
    )
    swivel.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.046, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="tilt_pivot_pos",
    )
    swivel.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, -0.046, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="tilt_pivot_neg",
    )

    head = model.part("lamp_head")
    head.visual(
        Box((0.078, 0.086, 0.048)),
        origin=Origin(xyz=(0.031, 0.0, 0.010)),
        material=satin_black,
        name="head_shell",
    )
    head.visual(
        Box((0.007, 0.078, 0.038)),
        origin=Origin(xyz=(0.0735, 0.0, 0.010)),
        material=rubber_black,
        name="front_bezel",
    )
    head.visual(
        Box((0.004, 0.064, 0.027)),
        origin=Origin(xyz=(0.0790, 0.0, 0.010)),
        material=diffuser,
        name="flash_diffuser",
    )
    head.visual(
        Box((0.003, 0.026, 0.015)),
        origin=Origin(xyz=(-0.0095, 0.0, -0.004)),
        material=red_window,
        name="rear_af_window",
    )
    head.visual(
        Box((0.020, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, 0.0445, 0.0)),
        material=dark_plastic,
        name="head_trunnion_pos",
    )
    head.visual(
        Box((0.020, 0.003, 0.020)),
        origin=Origin(xyz=(0.0, -0.0445, 0.0)),
        material=dark_plastic,
        name="head_trunnion_neg",
    )
    head.visual(
        Box((0.056, 0.064, 0.003)),
        origin=Origin(xyz=(0.034, 0.0, 0.0355)),
        material=dark_plastic,
        name="top_seam_cap",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, 0.157)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=3.0, lower=-2.1, upper=2.1),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.2, lower=-0.45, upper=1.35),
    )

    button_origins = (
        (-0.012, -0.029, 0.120),
        (-0.012, -0.029, 0.102),
        (-0.012, -0.029, 0.084),
        (-0.012, -0.029, 0.066),
    )
    for i, origin_xyz in enumerate(button_origins):
        button = model.part(f"mode_button_{i}")
        button.visual(
            Box((0.014, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=button_plastic,
            name="button_cap",
        )
        button.visual(
            Box((0.009, 0.0008, 0.0015)),
            origin=Origin(xyz=(0.0, -0.0043, 0.0)),
            material=Material(f"button_glyph_{i}", rgba=(0.78, 0.80, 0.80, 1.0)),
            name="button_glyph",
        )
        model.articulation(
            f"body_to_mode_button_{i}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=origin_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.2, velocity=0.08, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("battery_body")
    neck = object_model.get_part("aiming_neck")
    head = object_model.get_part("lamp_head")
    neck_swivel = object_model.get_articulation("body_to_neck")
    head_tilt = object_model.get_articulation("neck_to_head")
    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")
    button_0_press = object_model.get_articulation("body_to_mode_button_0")

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="swivel_turntable",
        negative_elem="top_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel turntable sits on the body top deck",
    )
    ctx.expect_gap(
        neck,
        head,
        axis="y",
        positive_elem="tilt_cheek_pos",
        negative_elem="head_trunnion_pos",
        max_gap=0.001,
        max_penetration=0.0,
        name="positive yoke cheek supports the head trunnion",
    )
    ctx.expect_gap(
        head,
        neck,
        axis="y",
        positive_elem="head_trunnion_neg",
        negative_elem="tilt_cheek_neg",
        max_gap=0.001,
        max_penetration=0.0,
        name="negative yoke cheek supports the head trunnion",
    )
    ctx.expect_gap(
        body,
        button_0,
        axis="y",
        positive_elem="side_control_strip",
        negative_elem="button_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="mode button rests on the side control strip",
    )

    rest_diffuser_aabb = ctx.part_element_world_aabb(head, elem="flash_diffuser")
    with ctx.pose({neck_swivel: 0.70}):
        swivel_diffuser_aabb = ctx.part_element_world_aabb(head, elem="flash_diffuser")
    ctx.check(
        "vertical swivel turns the lamp head around the neck",
        rest_diffuser_aabb is not None
        and swivel_diffuser_aabb is not None
        and abs(swivel_diffuser_aabb[0][1] - rest_diffuser_aabb[0][1]) > 0.020,
        details=f"rest={rest_diffuser_aabb}, swivel={swivel_diffuser_aabb}",
    )

    with ctx.pose({head_tilt: 0.90}):
        tilted_diffuser_aabb = ctx.part_element_world_aabb(head, elem="flash_diffuser")
    ctx.check(
        "horizontal tilt raises the flash diffuser",
        rest_diffuser_aabb is not None
        and tilted_diffuser_aabb is not None
        and tilted_diffuser_aabb[0][2] > rest_diffuser_aabb[0][2] + 0.025,
        details=f"rest={rest_diffuser_aabb}, tilted={tilted_diffuser_aabb}",
    )

    rest_button_0 = ctx.part_world_position(button_0)
    rest_button_1 = ctx.part_world_position(button_1)
    with ctx.pose({button_0_press: 0.0025}):
        pressed_button_0 = ctx.part_world_position(button_0)
        still_button_1 = ctx.part_world_position(button_1)
    ctx.check(
        "first mode button depresses independently",
        rest_button_0 is not None
        and rest_button_1 is not None
        and pressed_button_0 is not None
        and still_button_1 is not None
        and pressed_button_0[1] > rest_button_0[1] + 0.002
        and abs(still_button_1[1] - rest_button_1[1]) < 0.0001,
        details=(
            f"rest0={rest_button_0}, pressed0={pressed_button_0}, "
            f"rest1={rest_button_1}, still1={still_button_1}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
