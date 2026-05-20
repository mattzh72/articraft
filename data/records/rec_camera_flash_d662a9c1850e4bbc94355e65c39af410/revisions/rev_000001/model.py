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


def _rounded_box(size: tuple[float, float, float], radius: float) -> object:
    """CadQuery rounded rectangular housing, authored in meters."""

    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_camera_flash")

    black = model.material("satin_black_polymer", rgba=(0.015, 0.014, 0.013, 1.0))
    rubber = model.material("matte_rubber_black", rgba=(0.004, 0.004, 0.004, 1.0))
    metal = model.material("dark_anodized_metal", rgba=(0.22, 0.22, 0.22, 1.0))
    glass = model.material("smoked_lcd_glass", rgba=(0.03, 0.10, 0.13, 0.92))
    lens = model.material("milky_flash_lens", rgba=(0.88, 0.86, 0.74, 0.78))
    white = model.material("white_legends", rgba=(0.86, 0.86, 0.82, 1.0))

    body = model.part("body")

    # Camera hot-shoe sized lower mount: broad enough for a DSLR/mirrorless shoe,
    # with visible metal rails and a locking collar above it.
    body.visual(
        Box((0.066, 0.064, 0.008)),
        origin=Origin(xyz=(0.000, 0.000, 0.004)),
        material=metal,
        name="mounting_foot",
    )
    body.visual(
        Box((0.060, 0.010, 0.006)),
        origin=Origin(xyz=(0.000, 0.031, 0.011)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.060, 0.010, 0.006)),
        origin=Origin(xyz=(0.000, -0.031, 0.011)),
        material=metal,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.074, 0.080, 0.018)),
        origin=Origin(xyz=(0.000, 0.000, 0.017)),
        material=black,
        name="foot_shoulder",
    )
    body.visual(
        Cylinder(radius=0.026, length=0.018),
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
        material=black,
        name="neck_post",
    )
    body.visual(
        Cylinder(radius=0.039, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.033)),
        material=rubber,
        name="locking_collar",
    )

    body.visual(
        mesh_from_cadquery(_rounded_box((0.058, 0.074, 0.089), 0.007), "control_body_shell"),
        origin=Origin(xyz=(-0.010, 0.000, 0.0805)),
        material=black,
        name="control_body_shell",
    )
    body.visual(
        Box((0.003, 0.052, 0.028)),
        origin=Origin(xyz=(-0.0402, 0.000, 0.101)),
        material=glass,
        name="rear_lcd",
    )
    body.visual(
        Box((0.0015, 0.044, 0.0020)),
        origin=Origin(xyz=(-0.0420, 0.000, 0.113)),
        material=white,
        name="lcd_top_legend",
    )
    body.visual(
        Cylinder(radius=0.035, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.130)),
        material=black,
        name="swivel_socket",
    )

    yoke = model.part("swivel_yoke")
    yoke.visual(
        Cylinder(radius=0.037, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=black,
        name="turntable_disk",
    )
    yoke.visual(
        Box((0.060, 0.112, 0.012)),
        origin=Origin(xyz=(0.025, 0.000, 0.012)),
        material=black,
        name="yoke_bridge",
    )
    yoke.visual(
        Box((0.028, 0.010, 0.046)),
        origin=Origin(xyz=(0.026, 0.048, 0.034)),
        material=black,
        name="yoke_arm_0",
    )
    yoke.visual(
        Box((0.028, 0.010, 0.046)),
        origin=Origin(xyz=(0.026, -0.048, 0.034)),
        material=black,
        name="yoke_arm_1",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.026, 0.043, 0.037), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_bushing_0",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(0.026, -0.043, 0.037), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_bushing_1",
    )

    flash_head = model.part("flash_head")
    flash_head.visual(
        mesh_from_cadquery(_rounded_box((0.112, 0.075, 0.050), 0.006), "flash_head_shell"),
        origin=Origin(xyz=(0.060, 0.000, 0.002)),
        material=black,
        name="head_shell",
    )
    flash_head.visual(
        Box((0.006, 0.061, 0.039)),
        origin=Origin(xyz=(0.119, 0.000, 0.002)),
        material=lens,
        name="flash_lens",
    )
    flash_head.visual(
        Box((0.002, 0.050, 0.0025)),
        origin=Origin(xyz=(0.123, 0.000, 0.015)),
        material=white,
        name="lens_diffuser_line_0",
    )
    flash_head.visual(
        Box((0.002, 0.050, 0.0025)),
        origin=Origin(xyz=(0.123, 0.000, -0.009)),
        material=white,
        name="lens_diffuser_line_1",
    )
    flash_head.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.000, 0.0395, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="head_pivot_0",
    )
    flash_head.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.000, -0.0395, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="head_pivot_1",
    )

    dial = model.part("rear_dial")
    dial.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(-0.004, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0014, 0.003, 0.010)),
        origin=Origin(xyz=(-0.0084, 0.000, 0.007)),
        material=white,
        name="dial_index",
    )

    for index, (y, z) in enumerate(
        ((-0.018, 0.074), (0.018, 0.074), (-0.018, 0.058), (0.018, 0.058))
    ):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.004, 0.012, 0.007)),
            origin=Origin(xyz=(-0.002, 0.000, 0.000)),
            material=rubber,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(-0.0390, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.06, lower=0.0, upper=0.003),
        )

    model.articulation(
        "body_to_rear_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(-0.0390, 0.000, 0.045)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "body_to_swivel_yoke",
        ArticulationType.REVOLUTE,
        parent=body,
        child=yoke,
        origin=Origin(xyz=(0.000, 0.000, 0.135)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-2.7, upper=2.7),
    )
    model.articulation(
        "yoke_to_flash_head",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=flash_head,
        origin=Origin(xyz=(0.026, 0.000, 0.037)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.18, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    yoke = object_model.get_part("swivel_yoke")
    head = object_model.get_part("flash_head")
    swivel = object_model.get_articulation("body_to_swivel_yoke")
    pitch = object_model.get_articulation("yoke_to_flash_head")

    foot_aabb = ctx.part_element_world_aabb(body, elem="mounting_foot")
    shell_aabb = ctx.part_element_world_aabb(body, elem="control_body_shell")
    ctx.check(
        "mounting foot is broad relative to control body",
        foot_aabb is not None
        and shell_aabb is not None
        and (foot_aabb[1][1] - foot_aabb[0][1]) > 0.80 * (shell_aabb[1][1] - shell_aabb[0][1])
        and (foot_aabb[1][0] - foot_aabb[0][0]) > (shell_aabb[1][0] - shell_aabb[0][0]),
        details=f"foot={foot_aabb}, shell={shell_aabb}",
    )

    ctx.expect_gap(
        yoke,
        head,
        axis="y",
        positive_elem="yoke_arm_0",
        negative_elem="head_shell",
        min_gap=0.001,
        max_gap=0.010,
        name="upper yoke arm clears head side",
    )
    ctx.expect_gap(
        head,
        yoke,
        axis="y",
        positive_elem="head_shell",
        negative_elem="yoke_arm_1",
        min_gap=0.001,
        max_gap=0.010,
        name="lower yoke arm clears head side",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({pitch: 1.25}):
        pitched_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "pitch hinge raises flash head",
        rest_head_aabb is not None
        and pitched_head_aabb is not None
        and pitched_head_aabb[1][2] > rest_head_aabb[1][2] + 0.035,
        details=f"rest={rest_head_aabb}, pitched={pitched_head_aabb}",
    )

    rest_head_pos = ctx.part_world_position(head)
    with ctx.pose({swivel: 0.75}):
        swivel_head_pos = ctx.part_world_position(head)
    ctx.check(
        "vertical swivel yaws head assembly",
        rest_head_pos is not None
        and swivel_head_pos is not None
        and abs(swivel_head_pos[1] - rest_head_pos[1]) > 0.010,
        details=f"rest={rest_head_pos}, swivel={swivel_head_pos}",
    )

    return ctx.report()


object_model = build_object_model()
