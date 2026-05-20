from __future__ import annotations

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


BODY_DEPTH = 0.046
BODY_WIDTH = 0.078
BODY_HEIGHT = 0.118
BODY_BOTTOM_Z = 0.024
BODY_TOP_Z = BODY_BOTTOM_Z + BODY_HEIGHT
BODY_REAR_X = -BODY_DEPTH / 2.0
HEAD_PIVOT_Z = 0.028


def _body_shell() -> object:
    return (
        cq.Workplane("XY")
        .box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT)
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .edges()
        .chamfer(0.003)
        .translate((0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT / 2.0))
    )


def _head_shell() -> object:
    return (
        cq.Workplane("XY")
        .box(0.070, 0.072, 0.050)
        .translate((0.025, 0.0, 0.009))
        .faces(">X")
        .edges()
        .chamfer(0.006)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_flash")

    body_black = model.material("body_black", rgba=(0.16, 0.17, 0.18, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.26, 0.27, 0.29, 1.0))
    diffuser_white = model.material("diffuser_white", rgba=(0.95, 0.95, 0.93, 0.92))
    glass_blue = model.material("glass_blue", rgba=(0.17, 0.29, 0.35, 0.58))
    button_black = model.material("button_black", rgba=(0.12, 0.12, 0.13, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "flash_body_shell"),
        material=body_black,
        name="shell",
    )
    body.visual(
        Box((0.046, 0.030, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=foot_dark,
        name="foot_plate",
    )
    body.visual(
        Box((0.020, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=trim_black,
        name="shoe_rail",
    )
    body.visual(
        Box((0.028, 0.018, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=foot_dark,
        name="foot_stem",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z + 0.0015)),
        material=trim_black,
        name="swivel_collar",
    )
    body.visual(
        Box((0.003, 0.056, 0.038)),
        origin=Origin(xyz=(BODY_REAR_X - 0.0015, 0.0, 0.103)),
        material=dark_grey,
        name="lcd_bezel",
    )
    body.visual(
        Box((0.0015, 0.044, 0.026)),
        origin=Origin(xyz=(BODY_REAR_X - 0.0030, 0.0, 0.103)),
        material=glass_blue,
        name="lcd_glass",
    )
    body.visual(
        Box((0.0025, 0.066, 0.076)),
        origin=Origin(xyz=(BODY_REAR_X - 0.00125, 0.0, 0.076)),
        material=dark_grey,
        name="control_plate",
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_black,
        name="turntable",
    )
    yoke.visual(
        Box((0.007, 0.083, 0.016)),
        origin=Origin(xyz=(-0.0185, 0.0, 0.013)),
        material=body_black,
        name="rear_bridge",
    )
    yoke.visual(
        Box((0.018, 0.006, 0.034)),
        origin=Origin(xyz=(-0.006, -0.0420, 0.023)),
        material=body_black,
        name="arm_0",
    )
    yoke.visual(
        Box((0.018, 0.006, 0.034)),
        origin=Origin(xyz=(-0.006, 0.0420, 0.023)),
        material=body_black,
        name="arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(-0.001, -0.0420, HEAD_PIVOT_Z), rpy=(1.57079632679, 0.0, 0.0)),
        material=trim_black,
        name="pivot_cap_0",
    )
    yoke.visual(
        Cylinder(radius=0.0045, length=0.008),
        origin=Origin(xyz=(-0.001, 0.0420, HEAD_PIVOT_Z), rpy=(1.57079632679, 0.0, 0.0)),
        material=trim_black,
        name="pivot_cap_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell(), "flash_head_shell"),
        material=body_black,
        name="shell",
    )
    head.visual(
        Box((0.004, 0.062, 0.034)),
        origin=Origin(xyz=(0.056, 0.0, 0.009)),
        material=diffuser_white,
        name="diffuser",
    )
    head.visual(
        Box((0.006, 0.070, 0.042)),
        origin=Origin(xyz=(0.053, 0.0, 0.009)),
        material=trim_black,
        name="diffuser_frame",
    )
    head.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(-0.001, -0.0375, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=trim_black,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.0045, length=0.003),
        origin=Origin(xyz=(-0.001, 0.0375, 0.0), rpy=(1.57079632679, 0.0, 0.0)),
        material=trim_black,
        name="trunnion_1",
    )

    model.articulation(
        "body_to_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-1.9,
            upper=1.9,
        ),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(-0.001, 0.0, HEAD_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.018, length=0.0035),
        origin=Origin(xyz=(-0.00175, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=button_black,
        name="outer_ring",
    )
    dial.visual(
        Cylinder(radius=0.0145, length=0.005),
        origin=Origin(xyz=(-0.0025, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=dark_grey,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.0085, length=0.0055),
        origin=Origin(xyz=(-0.00275, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
        material=trim_black,
        name="hub",
    )
    dial.visual(
        Box((0.0015, 0.004, 0.010)),
        origin=Origin(xyz=(-0.0042, 0.0, 0.012)),
        material=diffuser_white,
        name="indicator",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_REAR_X - 0.0025, 0.0, 0.067)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=5.5),
    )

    key_specs = (
        ("top_key", (0.0, 0.093), (0.014, 0.018, 0.0040), (0.010, 0.014, 0.0015)),
        ("bottom_key", (0.0, 0.041), (0.014, 0.018, 0.0040), (0.010, 0.014, 0.0015)),
        ("left_key", (-0.028, 0.067), (0.014, 0.014, 0.011), (0.010, 0.010, 0.0015)),
        ("right_key", (0.028, 0.067), (0.014, 0.014, 0.011), (0.010, 0.010, 0.0015)),
    )
    for key_name, (key_y, key_z), cap_size, pad_size in key_specs:
        key = model.part(key_name)
        key.visual(
            Box(cap_size),
            origin=Origin(xyz=(-cap_size[0] / 2.0, 0.0, 0.0)),
            material=button_black,
            name="keycap",
        )
        key.visual(
            Box(pad_size),
            origin=Origin(xyz=(-cap_size[0] - pad_size[0] / 2.0 + 0.0015, 0.0, 0.0)),
            material=dark_grey,
            name="pad",
        )
        model.articulation(
            f"body_to_{key_name}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(BODY_REAR_X - 0.0025, key_y, key_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.06,
                lower=0.0,
                upper=0.0018,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    head = object_model.get_part("head")
    dial = object_model.get_part("dial")
    swivel = object_model.get_articulation("body_to_yoke")
    pitch = object_model.get_articulation("yoke_to_head")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_gap(head, body, axis="z", min_gap=0.005, name="head clears body at rest")

    diffuser_rest = ctx.part_element_world_aabb(head, elem="diffuser")
    with ctx.pose({pitch: 1.0}):
        diffuser_up = ctx.part_element_world_aabb(head, elem="diffuser")
    ctx.check(
        "head pitches upward",
        diffuser_rest is not None
        and diffuser_up is not None
        and (diffuser_up[1][2] + diffuser_up[0][2]) / 2.0 > (diffuser_rest[1][2] + diffuser_rest[0][2]) / 2.0 + 0.020,
        details=f"rest={diffuser_rest}, pitched={diffuser_up}",
    )

    with ctx.pose({swivel: 1.15}):
        diffuser_swiveled = ctx.part_element_world_aabb(head, elem="diffuser")
    ctx.check(
        "head swivels sideways",
        diffuser_rest is not None
        and diffuser_swiveled is not None
        and abs((diffuser_swiveled[1][1] + diffuser_swiveled[0][1]) / 2.0) > 0.030,
        details=f"rest={diffuser_rest}, swiveled={diffuser_swiveled}",
    )

    dial_indicator_rest = ctx.part_element_world_aabb(dial, elem="indicator")
    with ctx.pose({dial_joint: 1.0}):
        dial_indicator_turned = ctx.part_element_world_aabb(dial, elem="indicator")
    ctx.check(
        "dial rotates about rear axis",
        dial_indicator_rest is not None
        and dial_indicator_turned is not None
        and abs((dial_indicator_turned[1][1] + dial_indicator_turned[0][1]) / 2.0) > 0.007,
        details=f"rest={dial_indicator_rest}, turned={dial_indicator_turned}",
    )

    for key_name in ("top_key", "bottom_key", "left_key", "right_key"):
        key = object_model.get_part(key_name)
        key_joint = object_model.get_articulation(f"body_to_{key_name}")
        rest_pos = ctx.part_world_position(key)
        upper = key_joint.motion_limits.upper if key_joint.motion_limits is not None else None
        pressed_pos = None
        if upper is not None:
            with ctx.pose({key_joint: upper}):
                pressed_pos = ctx.part_world_position(key)
        ctx.check(
            f"{key_name} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] > rest_pos[0] + 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
