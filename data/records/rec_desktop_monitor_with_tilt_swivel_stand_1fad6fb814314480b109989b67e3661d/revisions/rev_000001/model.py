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
    mesh_from_cadquery,
)
import cadquery as cq


def _rounded_box(size: tuple[float, float, float], radius: float, selector: str, name: str):
    """Return a managed mesh for a centered CadQuery box with selected rounded edges."""
    body = cq.Workplane("XY").box(*size)
    if radius > 0.0:
        body = body.edges(selector).fillet(radius) if selector else body.edges().fillet(radius)
    return mesh_from_cadquery(body, name, tolerance=0.001, angular_tolerance=0.12)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    graphite = model.material("graphite_black", rgba=(0.015, 0.017, 0.020, 1.0))
    soft_black = model.material("soft_black", rgba=(0.035, 0.038, 0.043, 1.0))
    glass = model.material("dark_glass", rgba=(0.001, 0.010, 0.016, 1.0))
    satin_metal = model.material("satin_dark_metal", rgba=(0.15, 0.155, 0.16, 1.0))
    rubber = model.material("matte_button_rubber", rgba=(0.085, 0.088, 0.092, 1.0))

    # Root: a heavy desk base, broad enough to counterbalance a professional 27-inch class screen.
    base = model.part("base_plate")
    base.visual(
        _rounded_box((0.46, 0.30, 0.045), 0.040, "|Z", "base_plate_rounded"),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=satin_metal,
        name="heavy_plate",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=soft_black,
        name="swivel_bearing_cap",
    )

    # The entire upright stand swivels on the base.
    stand = model.part("stand")
    stand.visual(
        _rounded_box((0.13, 0.085, 0.50), 0.018, "|Z", "broad_stand_column"),
        origin=Origin(xyz=(0.0, 0.035, 0.250)),
        material=satin_metal,
        name="broad_column",
    )
    stand.visual(
        _rounded_box((0.245, 0.160, 0.140), 0.018, "|Y", "deep_stand_head"),
        origin=Origin(xyz=(0.0, 0.015, 0.545)),
        material=satin_metal,
        name="deep_head",
    )

    # Tilting carrier: a real-looking horizontal hinge barrel plus a deep pivot cup behind the screen.
    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Cylinder(radius=0.018, length=0.180),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soft_black,
        name="tilt_barrel",
    )
    tilt_carrier.visual(
        _rounded_box((0.135, 0.026, 0.078), 0.008, "|Y", "tilt_neck_block"),
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
        material=satin_metal,
        name="tilt_neck",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.072, length=0.033),
        origin=Origin(xyz=(0.0, -0.0085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="pivot_cup",
    )

    # Screen housing is thin, with a raised dark glass panel and a rear VESA/pivot boss.
    screen = model.part("screen_shell")
    screen.visual(
        _rounded_box((0.640, 0.040, 0.405), 0.020, "|Y", "thin_display_shell"),
        origin=Origin(),
        material=graphite,
        name="housing_shell",
    )
    screen.visual(
        Box((0.584, 0.004, 0.325)),
        origin=Origin(xyz=(0.0, -0.022, 0.027)),
        material=glass,
        name="front_glass",
    )
    screen.visual(
        Box((0.170, 0.014, 0.170)),
        origin=Origin(xyz=(0.0, 0.027, 0.0)),
        material=soft_black,
        name="vesa_plate",
    )
    screen.visual(
        Cylinder(radius=0.064, length=0.042),
        origin=Origin(xyz=(0.0, 0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=soft_black,
        name="rear_pivot_boss",
    )
    screen.visual(
        Box((0.070, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, -0.022, -0.172)),
        material=satin_metal,
        name="lower_menu_trim",
    )

    button_mesh = _rounded_box((0.024, 0.008, 0.011), 0.002, "", "menu_button_cap")
    button_xs = (-0.056, -0.028, 0.0, 0.028, 0.056)
    buttons = []
    for i, x in enumerate(button_xs):
        button = model.part(f"button_{i}")
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, -0.004, 0.0)),
            material=rubber,
            name="button_cap",
        )
        buttons.append((button, x))

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.2),
    )
    model.articulation(
        "stand_to_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, -0.083, 0.563)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.25, upper=0.35),
    )
    model.articulation(
        "tilt_to_screen",
        ArticulationType.CONTINUOUS,
        parent=tilt_carrier,
        child=screen,
        origin=Origin(xyz=(0.0, -0.087, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5),
    )
    for button, x in buttons:
        model.articulation(
            f"screen_to_{button.name}",
            ArticulationType.PRISMATIC,
            parent=screen,
            child=button,
            origin=Origin(xyz=(x, -0.020, -0.172)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.05, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_plate")
    stand = object_model.get_part("stand")
    tilt = object_model.get_part("tilt_carrier")
    screen = object_model.get_part("screen_shell")
    swivel = object_model.get_articulation("base_to_stand")
    tilt_joint = object_model.get_articulation("stand_to_tilt")
    pivot = object_model.get_articulation("tilt_to_screen")

    ctx.expect_gap(
        stand,
        base,
        axis="z",
        positive_elem="broad_column",
        negative_elem="swivel_bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="stand column sits on swivel cap",
    )
    ctx.expect_contact(
        tilt,
        stand,
        elem_a="tilt_barrel",
        elem_b="deep_head",
        contact_tol=0.0015,
        name="tilt barrel is carried by stand head",
    )
    ctx.expect_gap(
        tilt,
        screen,
        axis="y",
        positive_elem="pivot_cup",
        negative_elem="rear_pivot_boss",
        max_gap=0.003,
        max_penetration=0.0001,
        name="screen pivot boss seats against tilt cup",
    )

    for i in range(5):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"screen_to_button_{i}")
        ctx.expect_contact(
            button,
            screen,
            elem_a="button_cap",
            elem_b="housing_shell",
            contact_tol=0.0008,
            name=f"button {i} rests on lower bezel",
        )
        ctx.allow_overlap(
            screen,
            button,
            elem_a="housing_shell",
            elem_b="button_cap",
            reason="At the pressed pose the small menu button cap intentionally travels into its bezel opening.",
        )
        with ctx.pose({joint: 0.004}):
            ctx.expect_gap(
                screen,
                button,
                axis="y",
                positive_elem="housing_shell",
                negative_elem="button_cap",
                max_penetration=0.0045,
                name=f"button {i} has short inward push travel",
            )

    rest_screen_pos = ctx.part_world_position(screen)
    with ctx.pose({tilt_joint: 0.30}):
        tilted_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "tilt raises the screen center on backward tilt",
        rest_screen_pos is not None
        and tilted_screen_pos is not None
        and tilted_screen_pos[2] > rest_screen_pos[2] + 0.010,
        details=f"rest={rest_screen_pos}, tilted={tilted_screen_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(screen, elem="housing_shell")
    with ctx.pose({pivot: math.pi / 2.0}):
        portrait_shell_aabb = ctx.part_element_world_aabb(screen, elem="housing_shell")
    if rest_shell_aabb is not None and portrait_shell_aabb is not None:
        rest_min, rest_max = rest_shell_aabb
        portrait_min, portrait_max = portrait_shell_aabb
        rest_width = rest_max[0] - rest_min[0]
        rest_height = rest_max[2] - rest_min[2]
        portrait_width = portrait_max[0] - portrait_min[0]
        portrait_height = portrait_max[2] - portrait_min[2]
        ctx.check(
            "screen rotates to portrait about viewing axis",
            portrait_height > rest_height + 0.18 and portrait_width < rest_width - 0.18,
            details=(
                f"landscape=({rest_width:.3f}, {rest_height:.3f}), "
                f"portrait=({portrait_width:.3f}, {portrait_height:.3f})"
            ),
        )
    else:
        ctx.fail("screen rotates to portrait about viewing axis", "could not measure housing_shell AABB")

    rest_after_pose = ctx.part_world_position(screen)
    with ctx.pose({swivel: 0.45}):
        swivel_screen_pos = ctx.part_world_position(screen)
    ctx.check(
        "stand swivels around vertical base axis",
        rest_after_pose is not None
        and swivel_screen_pos is not None
        and abs(swivel_screen_pos[0] - rest_after_pose[0]) > 0.050,
        details=f"rest={rest_after_pose}, swiveled={swivel_screen_pos}",
    )

    return ctx.report()


object_model = build_object_model()
