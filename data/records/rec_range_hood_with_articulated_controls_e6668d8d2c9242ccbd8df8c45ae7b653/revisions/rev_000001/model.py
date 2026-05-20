from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOOD_WIDTH = 0.90
BODY_HEIGHT = 0.16
GLASS_WIDTH = 0.84
GLASS_LENGTH = 0.29
GLASS_ANGLE = 0.72
BUTTON_TRAVEL = 0.004
BUTTON_CENTERS = (-0.135, -0.045, 0.045, 0.135)
BUTTON_ROW_Y = 0.126
BUTTON_FRAME_Z = 0.006


def _rotate_x(y_value: float, z_value: float, angle: float) -> tuple[float, float]:
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        y_value * cos_a - z_value * sin_a,
        y_value * sin_a + z_value * cos_a,
    )


def _tilted_origin(
    *,
    x_value: float = 0.0,
    y_value: float = 0.0,
    z_value: float = 0.0,
    angle: float,
) -> Origin:
    y_rot, z_rot = _rotate_x(y_value, z_value, angle)
    return Origin(xyz=(x_value, y_rot, z_rot), rpy=(angle, 0.0, 0.0))


def _build_body_shape() -> cq.Workplane:
    outer_profile = [
        (0.000, 0.000),
        (0.000, BODY_HEIGHT),
        (0.205, BODY_HEIGHT),
        (0.205, 0.052),
        (0.176, 0.030),
        (0.176, -0.020),
        (0.080, -0.020),
        (0.080, 0.000),
    ]
    inner_profile = [
        (0.028, -0.040),
        (0.028, 0.128),
        (0.176, 0.128),
        (0.176, 0.040),
        (0.148, 0.012),
        (0.082, 0.012),
        (0.082, -0.040),
    ]

    outer = cq.Workplane("YZ").polyline(outer_profile).close().extrude(HOOD_WIDTH / 2.0, both=True)
    inner = cq.Workplane("YZ").polyline(inner_profile).close().extrude(0.404, both=True)
    shell = outer.cut(inner)

    for x_center in BUTTON_CENTERS:
        slot = (
            cq.Workplane("XY")
            .box(0.020, 0.016, 0.028, centered=(True, True, False))
            .translate((x_center, BUTTON_ROW_Y, -0.021))
        )
        shell = shell.cut(slot)

    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="angled_glass_range_hood")

    body_color = model.material("body_charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    glass_tint = model.material("glass_smoke", rgba=(0.30, 0.36, 0.40, 0.33))
    trim_color = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    button_color = model.material("button_satin", rgba=(0.76, 0.78, 0.80, 1.0))
    button_stem_color = model.material("button_dark", rgba=(0.18, 0.19, 0.20, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "range_hood_body"),
        material=body_color,
        name="hood_shell",
    )
    for index, x_center in enumerate((-0.27, 0.27)):
        body.visual(
            Box((0.065, 0.035, 0.024)),
            origin=Origin(xyz=(x_center, 0.188, 0.024)),
            material=trim_color,
            name=f"hinge_bracket_{index}",
        )

    glass = model.part("glass")
    glass.visual(
        Box((GLASS_WIDTH, 0.008, GLASS_LENGTH)),
        origin=_tilted_origin(y_value=0.0, z_value=-GLASS_LENGTH / 2.0, angle=GLASS_ANGLE),
        material=glass_tint,
        name="glass_panel",
    )
    glass.visual(
        Box((GLASS_WIDTH, 0.022, 0.024)),
        origin=_tilted_origin(y_value=0.0, z_value=-0.012, angle=GLASS_ANGLE),
        material=trim_color,
        name="top_clamp",
    )
    glass.visual(
        Box((GLASS_WIDTH * 0.92, 0.032, 0.018)),
        origin=_tilted_origin(y_value=0.010, z_value=-(GLASS_LENGTH - 0.010), angle=GLASS_ANGLE),
        material=trim_color,
        name="bottom_handle",
    )
    side_trim_offset = GLASS_WIDTH / 2.0 - 0.008
    for index, x_center in enumerate((-side_trim_offset, side_trim_offset)):
        glass.visual(
            Box((0.016, 0.018, GLASS_LENGTH - 0.006)),
            origin=_tilted_origin(
                x_value=x_center,
                y_value=0.0,
                z_value=-(GLASS_LENGTH - 0.006) / 2.0,
                angle=GLASS_ANGLE,
            ),
            material=trim_color,
            name=f"side_trim_{index}",
        )

    model.articulation(
        "body_to_glass",
        ArticulationType.REVOLUTE,
        parent=body,
        child=glass,
        origin=Origin(xyz=(0.0, 0.2133, 0.024)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.05,
        ),
    )

    for index, x_center in enumerate(BUTTON_CENTERS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.016, 0.012, 0.026)),
            origin=Origin(xyz=(0.0, 0.0, -0.013)),
            material=button_stem_color,
            name="button_stem",
        )
        button.visual(
            Box((0.030, 0.018, 0.007)),
            origin=Origin(xyz=(0.0, 0.0, -0.0295)),
            material=button_color,
            name="button_cap",
        )

        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_center, BUTTON_ROW_Y, BUTTON_FRAME_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    glass = object_model.get_part("glass")
    glass_hinge = object_model.get_articulation("body_to_glass")

    closed_panel_aabb = ctx.part_element_world_aabb(glass, elem="glass_panel")
    with ctx.pose({glass_hinge: 1.0}):
        open_panel_aabb = ctx.part_element_world_aabb(glass, elem="glass_panel")

    ctx.check(
        "glass panel lower edge rises when opened",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[0][2] > closed_panel_aabb[0][2] + 0.09,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.check(
        "glass panel reaches above hinge height when opened",
        open_panel_aabb is not None and open_panel_aabb[1][2] > 0.03,
        details=f"open={open_panel_aabb}",
    )

    rest_caps = []
    button_parts = []
    button_joints = []
    for index in range(4):
        button_parts.append(object_model.get_part(f"button_{index}"))
        button_joints.append(object_model.get_articulation(f"body_to_button_{index}"))
        rest_caps.append(ctx.part_element_world_aabb(button_parts[index], elem="button_cap"))

    for index, (button_part, button_joint) in enumerate(zip(button_parts, button_joints)):
        other_index = (index + 1) % 4
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_cap = ctx.part_element_world_aabb(button_part, elem="button_cap")
            other_cap = ctx.part_element_world_aabb(button_parts[other_index], elem="button_cap")

        rest_cap = rest_caps[index]
        other_rest_cap = rest_caps[other_index]
        ctx.check(
            f"button_{index} travels upward when pressed",
            rest_cap is not None
            and pressed_cap is not None
            and pressed_cap[1][2] > rest_cap[1][2] + 0.0025,
            details=f"rest={rest_cap}, pressed={pressed_cap}",
        )
        ctx.check(
            f"button_{index} motion stays independent",
            other_rest_cap is not None
            and other_cap is not None
            and abs(other_cap[1][2] - other_rest_cap[1][2]) < 1e-6,
            details=f"rest={other_rest_cap}, posed={other_cap}",
        )

    return ctx.report()


object_model = build_object_model()
