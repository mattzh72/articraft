from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """CadQuery rounded rectangular solid, authored in meters."""
    sx, sy, sz = size
    fillet = min(radius, sx * 0.45, sy * 0.45, sz * 0.45)
    return cq.Workplane("XY").box(sx, sy, sz).edges().fillet(fillet)


def _hollow_rect_tube(
    outer_size: tuple[float, float, float],
    inner_size: tuple[float, float],
    radius: float,
) -> cq.Workplane:
    """Open-ended rectangular height sleeve for the monitor lift column."""
    sx, sy, sz = outer_size
    ix, iy = inner_size
    outer = cq.Workplane("XY").box(sx, sy, sz).edges("|Z").fillet(radius)
    cutter = cq.Workplane("XY").box(ix, iy, sz + 0.02)
    return outer.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    graphite = model.material("satin_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    dark_plastic = model.material("matte_black_plastic", rgba=(0.008, 0.009, 0.010, 1.0))
    black_glass = model.material("black_glass", rgba=(0.0, 0.004, 0.009, 0.92))
    glass_sheen = model.material("cool_screen_sheen", rgba=(0.025, 0.055, 0.075, 0.72))
    brushed_metal = model.material("brushed_dark_metal", rgba=(0.28, 0.29, 0.30, 1.0))
    bearing_metal = model.material("blackened_bearing", rgba=(0.018, 0.018, 0.020, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    # Non-rotating, heavy desk plate.  The broad footprint and bevels read as a
    # professional monitor base rather than a small appliance foot.
    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_rounded_box((0.42, 0.28, 0.045), 0.018), "heavy_base_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=graphite,
        name="heavy_base_plate",
    )
    base.visual(
        Box((0.34, 0.20, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="rubber_foot_pad",
    )

    # Entire stand and display swivel continuously on a turntable at the base.
    swivel_stage = model.part("swivel_stage")
    swivel_stage.visual(
        Cylinder(radius=0.083, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=brushed_metal,
        name="swivel_turntable",
    )
    swivel_stage.visual(
        mesh_from_cadquery(
            _hollow_rect_tube((0.140, 0.082, 0.355), (0.105, 0.057), 0.010),
            "lower_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.2125)),
        material=brushed_metal,
        name="lower_sleeve",
    )
    swivel_stage.visual(
        Box((0.090, 0.006, 0.250)),
        origin=Origin(xyz=(0.0, 0.044, 0.212)),
        material=dark_plastic,
        name="rear_cable_slot",
    )

    model.articulation(
        "base_to_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=swivel_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4),
    )

    # Height-adjustable inner column.  It is long enough to remain retained in
    # the lower sleeve even at full extension.
    upper_column = model.part("upper_column")
    upper_column.visual(
        mesh_from_cadquery(_rounded_box((0.084, 0.040, 0.510), 0.006), "inner_mast"),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=brushed_metal,
        name="inner_mast",
    )
    upper_column.visual(
        mesh_from_cadquery(_rounded_box((0.220, 0.120, 0.092), 0.012), "stand_head"),
        origin=Origin(xyz=(0.0, 0.036, 0.205)),
        material=graphite,
        name="stand_head",
    )
    # Two fork cheeks flank the tilt barrel, leaving real depth for a hinge and
    # central rotation bearing behind the display.
    for x, name in ((-0.086, "hinge_cheek_0"), (0.086, "hinge_cheek_1")):
        upper_column.visual(
            mesh_from_cadquery(_rounded_box((0.034, 0.066, 0.074), 0.007), name),
            origin=Origin(xyz=(x, -0.036, 0.205)),
            material=graphite,
            name=name,
        )

    model.articulation(
        "swivel_to_column",
        ArticulationType.PRISMATIC,
        parent=swivel_stage,
        child=upper_column,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.160),
    )

    # Tilt carrier: a horizontal barrel, short neck, and rear bearing cup for the
    # portrait-rotation hub.  This assembly is intentionally deep, not a flat tab.
    tilt_carrier = model.part("tilt_carrier")
    tilt_carrier.visual(
        Cylinder(radius=0.022, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=bearing_metal,
        name="tilt_barrel",
    )
    tilt_carrier.visual(
        mesh_from_cadquery(_rounded_box((0.124, 0.039, 0.074), 0.008), "tilt_neck"),
        origin=Origin(xyz=(0.0, -0.019, 0.0)),
        material=graphite,
        name="tilt_neck",
    )
    tilt_carrier.visual(
        Cylinder(radius=0.080, length=0.010),
        origin=Origin(xyz=(0.0, -0.0435, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="rotation_cup",
    )

    model.articulation(
        "column_to_tilt",
        ArticulationType.REVOLUTE,
        parent=upper_column,
        child=tilt_carrier,
        origin=Origin(xyz=(0.0, -0.045, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=32.0, velocity=1.0, lower=-0.30, upper=0.24),
    )

    # Thin premium display shell with separate glossy glass, a small lower bezel
    # lip, and a rear rotation hub on the central viewing axis.
    display = model.part("display")
    display.visual(
        mesh_from_cadquery(_rounded_box((0.740, 0.034, 0.450), 0.018), "thin_display_shell"),
        origin=Origin(),
        material=graphite,
        name="thin_display_shell",
    )
    display.visual(
        Box((0.698, 0.004, 0.386)),
        origin=Origin(xyz=(0.0, -0.018, 0.008)),
        material=black_glass,
        name="active_glass",
    )
    display.visual(
        Box((0.642, 0.002, 0.018)),
        origin=Origin(xyz=(0.0, -0.0205, 0.170)),
        material=glass_sheen,
        name="screen_highlight",
    )
    display.visual(
        Box((0.700, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.020, -0.205)),
        material=dark_plastic,
        name="lower_bezel_lip",
    )
    display.visual(
        Cylinder(radius=0.066, length=0.014),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bearing_metal,
        name="rear_rotation_hub",
    )
    display.visual(
        Box((0.050, 0.030, 0.014)),
        origin=Origin(xyz=(0.285, -0.006, -0.232)),
        material=dark_plastic,
        name="joystick_socket",
    )

    model.articulation(
        "tilt_to_display",
        ArticulationType.CONTINUOUS,
        parent=tilt_carrier,
        child=display,
        origin=Origin(xyz=(0.0, -0.080, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=24.0, velocity=1.2),
    )

    # A two-axis gimbal gives the under-bezel joystick its real four-way feel.
    joystick_gimbal = model.part("joystick_gimbal")
    joystick_gimbal.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=bearing_metal,
        name="gimbal_collar",
    )
    model.articulation(
        "display_to_joystick",
        ArticulationType.REVOLUTE,
        parent=display,
        child=joystick_gimbal,
        origin=Origin(xyz=(0.285, -0.006, -0.239)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=-0.28, upper=0.28),
    )

    control_joystick = model.part("control_joystick")
    control_joystick.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=dark_plastic,
        name="joystick_stem",
    )
    control_joystick.visual(
        Sphere(radius=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=rubber,
        name="joystick_cap",
    )
    model.articulation(
        "joystick_to_nub",
        ArticulationType.REVOLUTE,
        parent=joystick_gimbal,
        child=control_joystick,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=-0.28, upper=0.28),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_to_swivel = object_model.get_articulation("base_to_swivel")
    height_slide = object_model.get_articulation("swivel_to_column")
    tilt_hinge = object_model.get_articulation("column_to_tilt")
    portrait_pivot = object_model.get_articulation("tilt_to_display")
    joystick_pitch = object_model.get_articulation("display_to_joystick")
    joystick_roll = object_model.get_articulation("joystick_to_nub")

    swivel_stage = object_model.get_part("swivel_stage")
    upper_column = object_model.get_part("upper_column")
    tilt_carrier = object_model.get_part("tilt_carrier")
    display = object_model.get_part("display")
    joystick_gimbal = object_model.get_part("joystick_gimbal")
    control_joystick = object_model.get_part("control_joystick")

    ctx.check(
        "monitor has all requested articulation families",
        base_to_swivel.articulation_type == ArticulationType.CONTINUOUS
        and height_slide.articulation_type == ArticulationType.PRISMATIC
        and tilt_hinge.articulation_type == ArticulationType.REVOLUTE
        and portrait_pivot.articulation_type == ArticulationType.CONTINUOUS
        and joystick_pitch.articulation_type == ArticulationType.REVOLUTE
        and joystick_roll.articulation_type == ArticulationType.REVOLUTE,
    )

    display_aabb = ctx.part_element_world_aabb(display, elem="thin_display_shell")
    if display_aabb is None:
        ctx.fail("display shell is measurable", "thin_display_shell AABB unavailable")
    else:
        shell_size = tuple(display_aabb[1][i] - display_aabb[0][i] for i in range(3))
        ctx.check(
            "thin professional-scale display shell",
            shell_size[0] > 0.70 and shell_size[2] > 0.42 and shell_size[1] < 0.045,
            details=f"shell_size={shell_size}",
        )

    ctx.expect_within(
        upper_column,
        swivel_stage,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="lower_sleeve",
        margin=0.0,
        name="height mast is centered inside sleeve",
    )
    ctx.expect_overlap(
        upper_column,
        swivel_stage,
        axes="z",
        elem_a="inner_mast",
        elem_b="lower_sleeve",
        min_overlap=0.250,
        name="lowered mast remains deeply inserted",
    )
    with ctx.pose({height_slide: 0.160}):
        ctx.expect_within(
            upper_column,
            swivel_stage,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="lower_sleeve",
            margin=0.0,
            name="raised mast remains centered in sleeve",
        )
        ctx.expect_overlap(
            upper_column,
            swivel_stage,
            axes="z",
            elem_a="inner_mast",
            elem_b="lower_sleeve",
            min_overlap=0.120,
            name="raised mast retains insertion",
        )

    rest_display_pos = ctx.part_world_position(display)
    with ctx.pose({height_slide: 0.160}):
        raised_display_pos = ctx.part_world_position(display)
    ctx.check(
        "height adjustment raises the display",
        rest_display_pos is not None
        and raised_display_pos is not None
        and raised_display_pos[2] > rest_display_pos[2] + 0.145,
        details=f"rest={rest_display_pos}, raised={raised_display_pos}",
    )

    ctx.expect_gap(
        tilt_carrier,
        display,
        axis="y",
        positive_elem="rotation_cup",
        negative_elem="rear_rotation_hub",
        min_gap=0.0,
        max_gap=0.004,
        name="portrait bearing stack sits close behind screen",
    )

    with ctx.pose({tilt_hinge: -0.25}):
        tilted_display_pos = ctx.part_world_position(display)
    ctx.check(
        "tilt hinge moves display on a real rear hinge radius",
        rest_display_pos is not None
        and tilted_display_pos is not None
        and tilted_display_pos[2] > rest_display_pos[2] + 0.015,
        details=f"rest={rest_display_pos}, tilted={tilted_display_pos}",
    )

    with ctx.pose({portrait_pivot: pi / 2.0}):
        portrait_aabb = ctx.part_element_world_aabb(display, elem="thin_display_shell")
    if portrait_aabb is None:
        ctx.fail("portrait pose is measurable", "thin_display_shell AABB unavailable in portrait pose")
    else:
        portrait_size = tuple(portrait_aabb[1][i] - portrait_aabb[0][i] for i in range(3))
        ctx.check(
            "continuous screen-axis pivot reaches portrait orientation",
            portrait_size[2] > portrait_size[0] + 0.20,
            details=f"portrait_size={portrait_size}",
        )

    ctx.expect_gap(
        display,
        joystick_gimbal,
        axis="z",
        positive_elem="joystick_socket",
        negative_elem="gimbal_collar",
        max_gap=0.001,
        max_penetration=0.0,
        name="joystick gimbal is supported by the underside socket",
    )
    ctx.expect_gap(
        joystick_gimbal,
        control_joystick,
        axis="z",
        positive_elem="gimbal_collar",
        negative_elem="joystick_stem",
        max_gap=0.001,
        max_penetration=0.0,
        name="joystick nub is carried by the gimbal collar",
    )

    return ctx.report()


object_model = build_object_model()
