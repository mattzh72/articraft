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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_xy_section(
    width_x: float,
    width_y: float,
    z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width_x, width_y, radius)]


def _rounded_yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def _body_shell_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _rounded_xy_section(0.050, 0.064, 0.024, 0.010),
                _rounded_xy_section(0.058, 0.070, 0.060, 0.012),
                _rounded_xy_section(0.055, 0.068, 0.128, 0.011),
                _rounded_xy_section(0.046, 0.058, 0.146, 0.010),
            ]
        ),
        "flash_rear_body",
    )


def _head_shell_mesh():
    return mesh_from_geometry(
        section_loft(
            [
                _rounded_yz_section(-0.020, 0.046, 0.043, 0.010),
                _rounded_yz_section(0.018, 0.054, 0.052, 0.012),
                _rounded_yz_section(0.072, 0.052, 0.047, 0.010),
                _rounded_yz_section(0.090, 0.046, 0.038, 0.008),
            ]
        ),
        "flash_lamp_head",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.055, 0.058, 0.064, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.025, 0.027, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.125, 0.135, 1.0))
    graphite = model.material("graphite", rgba=(0.24, 0.25, 0.26, 1.0))
    shoe_metal = model.material("shoe_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    lcd_glass = model.material("lcd_glass", rgba=(0.08, 0.16, 0.20, 0.75))
    diffuser = model.material("milky_diffuser", rgba=(0.88, 0.90, 0.86, 0.62))
    lamp_warm = model.material("lamp_warm", rgba=(1.0, 0.86, 0.48, 0.85))
    blue_button = model.material("blue_button", rgba=(0.10, 0.16, 0.30, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.055, 0.080, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=shoe_metal,
        name="mounting_foot",
    )
    body.visual(
        Box((0.062, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, 0.036, 0.011)),
        material=shoe_metal,
        name="foot_rail_0",
    )
    body.visual(
        Box((0.062, 0.006, 0.006)),
        origin=Origin(xyz=(0.000, -0.036, 0.011)),
        material=shoe_metal,
        name="foot_rail_1",
    )
    body.visual(
        Box((0.034, 0.023, 0.002)),
        origin=Origin(xyz=(0.004, 0.000, 0.015)),
        material=shoe_metal,
        name="hotshoe_contacts",
    )
    body.visual(
        Box((0.042, 0.050, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.016)),
        material=matte_black,
        name="foot_pedestal",
    )
    body.visual(
        _body_shell_mesh(),
        material=satin_black,
        name="rear_control_body",
    )
    body.visual(
        Box((0.0025, 0.040, 0.027)),
        origin=Origin(xyz=(-0.0292, 0.000, 0.100)),
        material=lcd_glass,
        name="rear_lcd",
    )
    body.visual(
        Box((0.0025, 0.044, 0.006)),
        origin=Origin(xyz=(-0.0295, 0.000, 0.071)),
        material=charcoal,
        name="rear_menu_keys",
    )
    body.visual(
        Box((0.018, 0.002, 0.080)),
        origin=Origin(xyz=(-0.004, -0.035, 0.090)),
        material=charcoal,
        name="side_control_strip",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(0.000, 0.000, 0.152)),
        material=dark_rubber,
        name="swivel_socket",
    )
    body.visual(
        Box((0.042, 0.050, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.145)),
        material=matte_black,
        name="top_shoulder",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.021, length=0.007),
        origin=Origin(xyz=(0.000, 0.000, 0.0035)),
        material=dark_rubber,
        name="swivel_collar",
    )
    yoke.visual(
        Cylinder(radius=0.014, length=0.040),
        origin=Origin(xyz=(0.000, 0.000, 0.023)),
        material=matte_black,
        name="neck_post",
    )
    yoke.visual(
        Box((0.026, 0.027, 0.042)),
        origin=Origin(xyz=(-0.002, 0.000, 0.052)),
        material=matte_black,
        name="neck_bridge",
    )
    yoke.visual(
        Box((0.035, 0.080, 0.012)),
        origin=Origin(xyz=(-0.004, 0.000, 0.074)),
        material=matte_black,
        name="top_crossbar",
    )
    yoke.visual(
        Box((0.054, 0.008, 0.050)),
        origin=Origin(xyz=(0.029, 0.036, 0.052)),
        material=matte_black,
        name="yoke_arm_0",
    )
    yoke.visual(
        Box((0.054, 0.008, 0.050)),
        origin=Origin(xyz=(0.029, -0.036, 0.052)),
        material=matte_black,
        name="yoke_arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.040, 0.042, 0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_axis_cap_0",
    )
    yoke.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.040, -0.042, 0.052), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_axis_cap_1",
    )

    head = model.part("head")
    head.visual(
        _head_shell_mesh(),
        material=matte_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.045, 0.034)),
        origin=Origin(xyz=(0.093, 0.000, 0.000)),
        material=diffuser,
        name="front_diffuser",
    )
    head.visual(
        Box((0.002, 0.032, 0.022)),
        origin=Origin(xyz=(0.0955, 0.000, 0.000)),
        material=lamp_warm,
        name="flash_tube_window",
    )
    head.visual(
        Box((0.030, 0.052, 0.005)),
        origin=Origin(xyz=(0.048, 0.000, 0.028)),
        material=charcoal,
        name="top_bounce_panel",
    )
    head.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(0.000, 0.029, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pivot_disc_0",
    )
    head.visual(
        Cylinder(radius=0.0095, length=0.006),
        origin=Origin(xyz=(0.000, -0.029, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="pivot_disc_1",
    )

    mode_z = (0.060, 0.080, 0.100, 0.120)
    for index, z in enumerate(mode_z):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.008, 0.002, 0.007)),
            origin=Origin(xyz=(0.000, -0.001, 0.000)),
            material=dark_rubber,
            name="button_stem",
        )
        button.visual(
            Box((0.014, 0.004, 0.010)),
            origin=Origin(xyz=(0.000, -0.004, 0.000)),
            material=blue_button if index == 0 else graphite,
            name="button_cap",
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.004, -0.036, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.05, lower=0.0, upper=0.0025),
        )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.000, 0.000, 0.156)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.5,
            lower=-math.radians(165.0),
            upper=math.radians(165.0),
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.040, 0.000, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.9,
            velocity=1.5,
            lower=-math.radians(10.0),
            upper=math.radians(92.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    yoke = object_model.get_part("yoke")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")

    ctx.expect_gap(
        yoke,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="swivel_collar",
        negative_elem="swivel_socket",
        name="swivel collar seats on body socket",
    )
    ctx.expect_overlap(
        head,
        yoke,
        axes="xz",
        min_overlap=0.015,
        elem_a="pivot_disc_0",
        elem_b="yoke_arm_0",
        name="head pivot is captured in yoke side",
    )
    ctx.expect_within(
        head,
        yoke,
        axes="y",
        margin=0.002,
        elem_a="head_shell",
        elem_b=None,
        name="lamp head fits between yoke arms",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({swivel: math.radians(65.0)}):
        swiveled_pos = ctx.part_world_position(head)
    with ctx.pose({pitch: math.radians(70.0)}):
        pitched_head_aabb = ctx.part_world_aabb(head)

    ctx.check(
        "head swivel moves around vertical axis",
        rest_head_pos is not None
        and swiveled_pos is not None
        and abs(swiveled_pos[1] - rest_head_pos[1]) > 0.035,
        details=f"rest={rest_head_pos}, swiveled={swiveled_pos}",
    )
    ctx.check(
        "head pitch raises the forward flash face",
        rest_head_aabb is not None
        and pitched_head_aabb is not None
        and pitched_head_aabb[1][2] > rest_head_aabb[1][2] + 0.030,
        details=f"rest={rest_head_aabb}, pitched={pitched_head_aabb}",
    )

    button_joints = [
        object_model.get_articulation(f"body_to_mode_button_{index}") for index in range(4)
    ]
    for index, joint in enumerate(button_joints):
        ctx.check(
            f"mode button {index} is an independent prismatic control",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.motion_limits is not None
            and joint.motion_limits.upper >= 0.002,
            details=str(joint),
        )

    button_0 = object_model.get_part("mode_button_0")
    button_1 = object_model.get_part("mode_button_1")
    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_joints[0]: 0.0025}):
        button_0_pressed = ctx.part_world_position(button_0)
        button_1_unpressed = ctx.part_world_position(button_1)
    ctx.check(
        "pressing one side mode button does not move its neighbor",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_unpressed is not None
        and button_0_pressed[1] > button_0_rest[1] + 0.002
        and abs(button_1_unpressed[1] - button_1_rest[1]) < 0.0001,
        details=(
            f"button0 rest={button_0_rest}, pressed={button_0_pressed}; "
            f"button1 rest={button_1_rest}, unpressed={button_1_unpressed}"
        ),
    )
    for index in range(4):
        button = object_model.get_part(f"mode_button_{index}")
        ctx.expect_gap(
            yoke,
            button,
            axis="z",
            min_gap=0.020,
            positive_elem="swivel_collar",
            name=f"side button {index} stays below neck geometry",
        )

    return ctx.report()


object_model = build_object_model()
