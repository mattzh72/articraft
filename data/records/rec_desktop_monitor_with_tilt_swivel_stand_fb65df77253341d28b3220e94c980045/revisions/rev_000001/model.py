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


def _rounded_box(width: float, depth: float, height: float, radius: float, edge_axis: str):
    """CadQuery rounded rectangular prism in local X/Y/Z dimensions."""
    shape = cq.Workplane("XY").box(width, depth, height)
    if radius > 0.0:
        shape = shape.edges(f"|{edge_axis.upper()}").fillet(radius)
    return shape


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_desktop_monitor")

    graphite = model.material("satin_graphite", rgba=(0.055, 0.058, 0.064, 1.0))
    dark_plastic = model.material("soft_black_plastic", rgba=(0.010, 0.011, 0.013, 1.0))
    glass = model.material("glossy_black_glass", rgba=(0.015, 0.020, 0.030, 0.88))
    screen_glow = model.material("subtle_screen_blue", rgba=(0.035, 0.055, 0.080, 1.0))
    metal = model.material("dark_bushed_metal", rgba=(0.12, 0.125, 0.13, 1.0))
    control_gray = model.material("control_graphite", rgba=(0.18, 0.185, 0.19, 1.0))

    # Root: a heavy, shallow, rounded base plate sized for a professional monitor.
    base = model.part("base")
    base_plate = _rounded_box(0.46, 0.32, 0.035, 0.045, "z")
    base.visual(
        mesh_from_cadquery(base_plate, "rounded_base_plate", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.095, length=0.020),
        origin=Origin(xyz=(0.0, 0.030, 0.045)),
        material=metal,
        name="turntable",
    )

    # The swiveling upright is intentionally broad and deep, not a spindly stem.
    stand = model.part("stand")
    stand.visual(
        Box((0.13, 0.080, 0.360)),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=graphite,
        name="column",
    )
    stand.visual(
        Box((0.085, 0.014, 0.285)),
        origin=Origin(xyz=(0.0, -0.047, 0.205)),
        material=dark_plastic,
        name="front_channel",
    )
    stand.visual(
        Box((0.210, 0.100, 0.120)),
        origin=Origin(xyz=(0.0, 0.055, 0.405)),
        material=graphite,
        name="head_block",
    )
    stand.visual(
        Box((0.025, 0.060, 0.075)),
        origin=Origin(xyz=(-0.095, -0.023, 0.405)),
        material=metal,
        name="clevis_0",
    )
    stand.visual(
        Box((0.025, 0.060, 0.075)),
        origin=Origin(xyz=(0.095, -0.023, 0.405)),
        material=metal,
        name="clevis_1",
    )

    # Tilt carriage: a visible horizontal hinge barrel plus a deep central pivot
    # boss, leaving realistic depth behind the display for tilt and portrait rotation.
    tilt_carriage = model.part("tilt_carriage")
    tilt_carriage.visual(
        Cylinder(radius=0.023, length=0.165),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="hinge_barrel",
    )
    tilt_carriage.visual(
        Box((0.112, 0.029, 0.052)),
        origin=Origin(xyz=(0.0, -0.0165, 0.0)),
        material=graphite,
        name="tilt_arm",
    )
    tilt_carriage.visual(
        Cylinder(radius=0.072, length=0.012),
        origin=Origin(xyz=(0.0, -0.0325, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="pivot_ring",
    )

    # Thin display shell: 27-inch-class proportions, slim bezel, glossy inset glass,
    # and a rear turntable that is captured by the stand-side pivot ring.
    display = model.part("display_shell")
    shell_body = _rounded_box(0.640, 0.040, 0.380, 0.018, "y")
    display.visual(
        mesh_from_cadquery(shell_body, "thin_display_shell", tolerance=0.0008),
        material=dark_plastic,
        name="shell_body",
    )
    display.visual(
        Box((0.580, 0.004, 0.324)),
        origin=Origin(xyz=(0.0, -0.022, 0.006)),
        material=screen_glow,
        name="screen_panel",
    )
    display.visual(
        Box((0.640, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, -0.025, 0.176)),
        material=glass,
        name="top_bezel",
    )
    display.visual(
        Box((0.640, 0.006, 0.032)),
        origin=Origin(xyz=(0.0, -0.025, -0.174)),
        material=glass,
        name="bottom_bezel",
    )
    display.visual(
        Box((0.030, 0.006, 0.326)),
        origin=Origin(xyz=(-0.305, -0.025, 0.0)),
        material=glass,
        name="side_bezel_0",
    )
    display.visual(
        Box((0.030, 0.006, 0.326)),
        origin=Origin(xyz=(0.305, -0.025, 0.0)),
        material=glass,
        name="side_bezel_1",
    )
    display.visual(
        Cylinder(radius=0.075, length=0.012),
        origin=Origin(xyz=(0.0, 0.026, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="rear_turntable",
    )
    display.visual(
        Box((0.155, 0.006, 0.090)),
        origin=Origin(xyz=(0.0, 0.033, 0.115)),
        material=graphite,
        name="rear_mount_plate",
    )

    # Distinct underside controls.  Each has its own moving link and joint.
    power_rocker = model.part("power_rocker")
    power_rocker.visual(
        Cylinder(radius=0.005, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=control_gray,
        name="rocker_pivot",
    )
    power_rocker.visual(
        Box((0.034, 0.020, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=dark_plastic,
        name="rocker_cap",
    )

    button_positions = (0.206, 0.238, 0.270)
    buttons = []
    for index, x_pos in enumerate(button_positions):
        button = model.part(f"menu_button_{index}")
        button.visual(
            Box((0.020, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=control_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.012)),
            material=dark_plastic,
            name="finger_dimple",
        )
        buttons.append((button, x_pos))

    model.articulation(
        "base_to_stand",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=stand,
        origin=Origin(xyz=(0.0, 0.030, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=2.0),
    )
    model.articulation(
        "stand_to_tilt",
        ArticulationType.REVOLUTE,
        parent=stand,
        child=tilt_carriage,
        origin=Origin(xyz=(0.0, -0.025, 0.405)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.20, upper=0.35),
    )
    model.articulation(
        "tilt_to_display",
        ArticulationType.CONTINUOUS,
        parent=tilt_carriage,
        child=display,
        origin=Origin(xyz=(0.0, -0.060, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5),
    )
    model.articulation(
        "display_to_power_rocker",
        ArticulationType.REVOLUTE,
        parent=display,
        child=power_rocker,
        origin=Origin(xyz=(0.160, -0.006, -0.195)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0, lower=-0.28, upper=0.28),
    )
    for index, (button, x_pos) in enumerate(buttons):
        model.articulation(
            f"display_to_menu_button_{index}",
            ArticulationType.PRISMATIC,
            parent=display,
            child=button,
            origin=Origin(xyz=(x_pos, -0.006, -0.190)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.03, lower=0.0, upper=0.004),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    stand = object_model.get_part("stand")
    tilt_carriage = object_model.get_part("tilt_carriage")
    display = object_model.get_part("display_shell")
    rocker = object_model.get_part("power_rocker")
    button_0 = object_model.get_part("menu_button_0")
    button_1 = object_model.get_part("menu_button_1")

    swivel = object_model.get_articulation("base_to_stand")
    tilt = object_model.get_articulation("stand_to_tilt")
    pivot = object_model.get_articulation("tilt_to_display")
    rocker_joint = object_model.get_articulation("display_to_power_rocker")
    button_1_joint = object_model.get_articulation("display_to_menu_button_1")

    ctx.allow_overlap(
        tilt_carriage,
        display,
        elem_a="pivot_ring",
        elem_b="rear_turntable",
        reason="The rear turntable is intentionally captured in the stand-side rotation ring for the portrait pivot bearing.",
    )
    ctx.expect_overlap(
        display,
        tilt_carriage,
        axes="xz",
        elem_a="rear_turntable",
        elem_b="pivot_ring",
        min_overlap=0.11,
        name="portrait pivot bearing has broad captured overlap",
    )
    ctx.expect_gap(
        display,
        tilt_carriage,
        axis="y",
        positive_elem="rear_turntable",
        negative_elem="pivot_ring",
        max_penetration=0.016,
        max_gap=0.002,
        name="portrait pivot bearing is shallowly seated",
    )

    ctx.expect_contact(
        stand,
        base,
        elem_a="column",
        elem_b="turntable",
        contact_tol=0.0015,
        name="broad stand column sits on the swivel turntable",
    )

    ctx.check(
        "stand swivel is continuous",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={swivel.articulation_type}",
    )
    ctx.check(
        "screen portrait pivot is continuous",
        pivot.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={pivot.articulation_type}",
    )
    ctx.check(
        "tilt has realistic monitor limits",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower <= -0.18
        and tilt.motion_limits.upper >= 0.30,
        details=f"limits={tilt.motion_limits}",
    )
    ctx.check(
        "power rocker has short bidirectional rotation",
        rocker_joint.motion_limits is not None
        and rocker_joint.motion_limits.lower is not None
        and rocker_joint.motion_limits.upper is not None
        and -0.35 <= rocker_joint.motion_limits.lower < 0.0
        and 0.0 < rocker_joint.motion_limits.upper <= 0.35,
        details=f"limits={rocker_joint.motion_limits}",
    )

    rest_display = ctx.part_world_position(display)
    with ctx.pose({swivel: math.pi / 2.0}):
        swiveled_display = ctx.part_world_position(display)
    ctx.check(
        "stand swivel moves display around vertical base axis",
        rest_display is not None
        and swiveled_display is not None
        and abs(swiveled_display[0] - rest_display[0]) > 0.05,
        details=f"rest={rest_display}, swiveled={swiveled_display}",
    )

    rest_aabb = ctx.part_world_aabb(display)
    with ctx.pose({pivot: math.pi / 2.0}):
        portrait_aabb = ctx.part_world_aabb(display)
    if rest_aabb is not None and portrait_aabb is not None:
        rest_size = (
            rest_aabb[1][0] - rest_aabb[0][0],
            rest_aabb[1][2] - rest_aabb[0][2],
        )
        portrait_size = (
            portrait_aabb[1][0] - portrait_aabb[0][0],
            portrait_aabb[1][2] - portrait_aabb[0][2],
        )
    else:
        rest_size = portrait_size = None
    ctx.check(
        "screen rotates into portrait orientation",
        rest_size is not None
        and portrait_size is not None
        and rest_size[0] > rest_size[1]
        and portrait_size[1] > portrait_size[0],
        details=f"rest_size={rest_size}, portrait_size={portrait_size}",
    )

    rocker_rest = ctx.part_world_aabb(rocker)
    with ctx.pose({rocker_joint: 0.25}):
        rocker_moved = ctx.part_world_aabb(rocker)
    ctx.check(
        "power rocker visibly rotates on its local pivot",
        rocker_rest is not None
        and rocker_moved is not None
        and abs(rocker_moved[0][1] - rocker_rest[0][1]) > 0.001,
        details=f"rest={rocker_rest}, moved={rocker_moved}",
    )

    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    with ctx.pose({button_1_joint: 0.004}):
        button_0_after = ctx.part_world_position(button_0)
        button_1_after = ctx.part_world_position(button_1)
    ctx.check(
        "menu buttons depress independently",
        button_0_rest is not None
        and button_1_rest is not None
        and button_0_after is not None
        and button_1_after is not None
        and abs(button_0_after[2] - button_0_rest[2]) < 0.0005
        and button_1_after[2] > button_1_rest[2] + 0.003,
        details=f"b0_rest={button_0_rest}, b0_after={button_0_after}, b1_rest={button_1_rest}, b1_after={button_1_after}",
    )

    return ctx.report()


object_model = build_object_model()
