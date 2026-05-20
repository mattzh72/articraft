from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small helper for molded plastic housings in meter units."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mirrorless_hot_shoe_flash")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.07, 0.075, 0.08, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.03, 0.032, 0.035, 1.0))
    display_glass = model.material("display_glass", rgba=(0.08, 0.18, 0.22, 0.72))
    diffuser = model.material("milky_diffuser", rgba=(0.92, 0.90, 0.82, 0.78))
    metal = model.material("shoe_metal", rgba=(0.42, 0.43, 0.44, 1.0))
    label_white = model.material("printed_white", rgba=(0.86, 0.86, 0.82, 1.0))
    accent_blue = model.material("accent_blue", rgba=(0.08, 0.20, 0.34, 1.0))

    body = model.part("battery_body")
    body.visual(
        mesh_from_cadquery(_rounded_box((0.060, 0.038, 0.070), 0.004), "battery_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=matte_black,
        name="battery_shell",
    )
    body.visual(
        Cylinder(radius=0.020, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=satin_black,
        name="top_bearing",
    )
    body.visual(
        Box((0.002, 0.052, 0.057)),
        origin=Origin(xyz=(0.031, 0.0, 0.037)),
        material=satin_black,
        name="side_strip",
    )
    body.visual(
        Box((0.040, 0.002, 0.020)),
        origin=Origin(xyz=(0.0, -0.020, 0.050)),
        material=display_glass,
        name="rear_display",
    )
    body.visual(
        Box((0.044, 0.002, 0.019)),
        origin=Origin(xyz=(0.0, -0.020, 0.024)),
        material=satin_black,
        name="rear_control_panel",
    )
    for index, x in enumerate((-0.014, 0.0, 0.014)):
        body.visual(
            Box((0.008, 0.003, 0.005)),
            origin=Origin(xyz=(x, -0.022, 0.024)),
            material=dark_rubber,
            name=f"rear_button_{index}",
        )
    body.visual(
        Box((0.012, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, -0.022, 0.014)),
        material=label_white,
        name="rear_status_bar",
    )
    body.visual(
        Box((0.040, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.007, 0.020, 0.004)),
        origin=Origin(xyz=(-0.018, 0.0, -0.006)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.007, 0.020, 0.004)),
        origin=Origin(xyz=(0.018, 0.0, -0.006)),
        material=metal,
        name="shoe_rail_1",
    )
    body.visual(
        Cylinder(radius=0.0045, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0055)),
        material=metal,
        name="locking_pin",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=satin_black,
        name="swivel_disk",
    )
    neck.visual(
        Box((0.016, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=satin_black,
        name="short_stem",
    )
    neck.visual(
        Box((0.092, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, -0.004, 0.025)),
        material=satin_black,
        name="yoke_bridge",
    )
    neck.visual(
        Box((0.006, 0.022, 0.025)),
        origin=Origin(xyz=(-0.044, -0.004, 0.038)),
        material=satin_black,
        name="yoke_cheek_0",
    )
    neck.visual(
        Box((0.006, 0.022, 0.025)),
        origin=Origin(xyz=(0.044, -0.004, 0.038)),
        material=satin_black,
        name="yoke_cheek_1",
    )

    head = model.part("lamp_head")
    head.visual(
        Cylinder(radius=0.009, length=0.083),
        origin=Origin(xyz=(0.0, -0.004, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="tilt_barrel",
    )
    head.visual(
        mesh_from_cadquery(_rounded_box((0.078, 0.054, 0.034), 0.003), "lamp_head_shell"),
        origin=Origin(xyz=(0.0, 0.025, 0.017)),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.066, 0.003, 0.024)),
        origin=Origin(xyz=(0.0, 0.053, 0.018)),
        material=diffuser,
        name="front_diffuser",
    )
    head.visual(
        Box((0.072, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.052, 0.0315)),
        material=satin_black,
        name="front_bezel_top",
    )
    head.visual(
        Box((0.072, 0.003, 0.003)),
        origin=Origin(xyz=(0.0, 0.052, 0.0045)),
        material=satin_black,
        name="front_bezel_bottom",
    )
    head.visual(
        Box((0.003, 0.003, 0.027)),
        origin=Origin(xyz=(-0.036, 0.052, 0.018)),
        material=satin_black,
        name="front_bezel_side_0",
    )
    head.visual(
        Box((0.003, 0.003, 0.027)),
        origin=Origin(xyz=(0.036, 0.052, 0.018)),
        material=satin_black,
        name="front_bezel_side_1",
    )
    head.visual(
        Box((0.048, 0.002, 0.003)),
        origin=Origin(xyz=(0.0, -0.0015, 0.030)),
        material=label_white,
        name="bounce_card_line",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=math.radians(-10.0),
            upper=math.radians(95.0),
        ),
    )

    button_specs = (
        ("mode_button_0", 0.000, 0.053, accent_blue),
        ("mode_button_1", 0.000, 0.040, dark_rubber),
        ("mode_button_2", 0.000, 0.027, dark_rubber),
        ("mode_button_3", 0.000, 0.014, dark_rubber),
    )
    for name, y, z, material in button_specs:
        button = model.part(name)
        button.visual(
            Box((0.003, 0.011, 0.007)),
            origin=Origin(xyz=(0.0015, 0.0, 0.0)),
            material=material,
            name="button_cap",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(0.032, y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=0.06, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("battery_body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("lamp_head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")

    ctx.check(
        "neck uses vertical revolute swivel",
        swivel.articulation_type == ArticulationType.REVOLUTE and tuple(swivel.axis) == (0.0, 0.0, 1.0),
        details=f"type={swivel.articulation_type}, axis={swivel.axis}",
    )
    ctx.check(
        "head uses horizontal tilt hinge",
        tilt.articulation_type == ArticulationType.REVOLUTE and tuple(tilt.axis) == (1.0, 0.0, 0.0),
        details=f"type={tilt.articulation_type}, axis={tilt.axis}",
    )

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem="swivel_disk",
        negative_elem="top_bearing",
        name="neck swivel disk is seated on body bearing",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.025,
        name="lamp head is visibly separated above battery body",
    )
    ctx.expect_gap(
        body,
        body,
        axis="z",
        min_gap=0.004,
        max_gap=0.010,
        positive_elem="rear_display",
        negative_elem="rear_control_panel",
        name="rear control panel sits below display",
    )

    for index in range(4):
        button = object_model.get_part(f"mode_button_{index}")
        joint = object_model.get_articulation(f"body_to_mode_button_{index}")
        ctx.check(
            f"mode button {index} is independently prismatic",
            joint.articulation_type == ArticulationType.PRISMATIC and tuple(joint.axis) == (-1.0, 0.0, 0.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
        ctx.expect_gap(
            button,
            body,
            axis="x",
            max_gap=0.001,
            max_penetration=0.001,
            positive_elem="button_cap",
            negative_elem="side_strip",
            name=f"mode button {index} rests on side control strip",
        )
        ctx.expect_overlap(
            button,
            body,
            axes="yz",
            min_overlap=0.006,
            elem_a="button_cap",
            elem_b="side_strip",
            name=f"mode button {index} stays within side strip footprint",
        )
        ctx.expect_gap(
            neck,
            button,
            axis="z",
            min_gap=0.010,
            name=f"mode button {index} stays below neck geometry",
        )
        ctx.expect_gap(
            head,
            button,
            axis="z",
            min_gap=0.035,
            name=f"mode button {index} stays below lamp head geometry",
        )

    diffuser_rest = ctx.part_element_world_aabb(head, elem="front_diffuser")
    with ctx.pose({tilt: math.radians(45.0)}):
        diffuser_tilted = ctx.part_element_world_aabb(head, elem="front_diffuser")
    ctx.check(
        "head tilt raises front diffuser",
        diffuser_rest is not None
        and diffuser_tilted is not None
        and diffuser_tilted[1][2] > diffuser_rest[1][2] + 0.020,
        details=f"rest={diffuser_rest}, tilted={diffuser_tilted}",
    )

    with ctx.pose({swivel: math.radians(90.0)}):
        diffuser_swiveled = ctx.part_element_world_aabb(head, elem="front_diffuser")
    rest_center_x = (diffuser_rest[0][0] + diffuser_rest[1][0]) * 0.5 if diffuser_rest is not None else None
    swivel_center_x = (
        (diffuser_swiveled[0][0] + diffuser_swiveled[1][0]) * 0.5
        if diffuser_swiveled is not None
        else None
    )
    ctx.check(
        "vertical swivel yaws the lamp head",
        rest_center_x is not None and swivel_center_x is not None and abs(swivel_center_x - rest_center_x) > 0.030,
        details=f"rest_x={rest_center_x}, swivel_x={swivel_center_x}",
    )

    button_0 = object_model.get_part("mode_button_0")
    button_0_joint = object_model.get_articulation("body_to_mode_button_0")
    button_rest = ctx.part_world_position(button_0)
    with ctx.pose({button_0_joint: 0.0025}):
        button_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "mode button depresses inward independently",
        button_rest is not None and button_pressed is not None and button_pressed[0] < button_rest[0] - 0.002,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
