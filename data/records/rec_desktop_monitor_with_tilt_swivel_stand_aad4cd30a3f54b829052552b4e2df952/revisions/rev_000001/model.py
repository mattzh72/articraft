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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _rounded_box_mesh(size: tuple[float, float, float], radius: float, axis: str, name: str):
    """CadQuery rounded box used for monitor parts that need softened corners."""
    shape = cq.Workplane("XY").box(size[0], size[1], size[2]).edges(f"|{axis}").fillet(radius)
    return mesh_from_cadquery(shape, name, tolerance=0.0008, angular_tolerance=0.08)


def _pedestal_base_mesh():
    shape = (
        cq.Workplane("XY")
        .box(0.34, 0.24, 0.035)
        .edges("|Z")
        .fillet(0.055)
        .edges(">Z or <Z")
        .fillet(0.004)
    )
    return mesh_from_cadquery(shape, "pedestal_base", tolerance=0.0008, angular_tolerance=0.08)


def _outer_sleeve_mesh():
    # A real hollow rectangular sleeve, not a solid proxy, so the inner neck can
    # slide through it with a small visible clearance.
    shape = cq.Workplane("XY").rect(0.080, 0.050).rect(0.052, 0.026).extrude(0.220)
    return mesh_from_cadquery(shape, "outer_sleeve", tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="business_monitor")

    charcoal = model.material("charcoal_plastic", rgba=(0.035, 0.038, 0.042, 1.0))
    black = model.material("soft_black", rgba=(0.006, 0.007, 0.008, 1.0))
    screen_glass = model.material("dark_screen_glass", rgba=(0.015, 0.025, 0.035, 1.0))
    stand_metal = model.material("satin_black_metal", rgba=(0.10, 0.105, 0.11, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.012, 0.012, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        _pedestal_base_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=charcoal,
        name="rounded_pedestal",
    )
    base.visual(
        Cylinder(radius=0.058, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=stand_metal,
        name="bearing_cap",
    )

    outer_neck = model.part("outer_neck")
    outer_neck.visual(
        Cylinder(radius=0.050, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=stand_metal,
        name="turntable_disk",
    )
    outer_neck.visual(
        _outer_sleeve_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=stand_metal,
        name="outer_sleeve",
    )
    outer_neck.visual(
        Box((0.020, 0.003, 0.100)),
        origin=Origin(xyz=(0.0, 0.0115, 0.164)),
        material=rubber,
        name="front_guide_pad",
    )
    outer_neck.visual(
        Box((0.020, 0.003, 0.100)),
        origin=Origin(xyz=(0.0, -0.0115, 0.164)),
        material=rubber,
        name="rear_guide_pad",
    )

    inner_neck = model.part("inner_neck")
    inner_neck.visual(
        Box((0.038, 0.020, 0.290)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=stand_metal,
        name="inner_mast",
    )
    inner_neck.visual(
        Box((0.086, 0.060, 0.052)),
        origin=Origin(xyz=(0.0, 0.018, 0.135)),
        material=stand_metal,
        name="head_block",
    )
    inner_neck.visual(
        Box((0.018, 0.058, 0.070)),
        origin=Origin(xyz=(-0.050, 0.022, 0.180)),
        material=stand_metal,
        name="yoke_cheek_0",
    )
    inner_neck.visual(
        Box((0.018, 0.058, 0.070)),
        origin=Origin(xyz=(0.050, 0.022, 0.180)),
        material=stand_metal,
        name="yoke_cheek_1",
    )

    display_shell = model.part("display_shell")
    display_shell.visual(
        _rounded_box_mesh((0.550, 0.055, 0.355), 0.018, "Y", "boxy_display_shell"),
        origin=Origin(xyz=(0.0, -0.045, 0.105)),
        material=charcoal,
        name="boxy_shell",
    )
    display_shell.visual(
        Box((0.494, 0.004, 0.262)),
        origin=Origin(xyz=(0.0, -0.074, 0.125)),
        material=screen_glass,
        name="screen_panel",
    )
    display_shell.visual(
        Box((0.074, 0.024, 0.080)),
        origin=Origin(xyz=(0.0, -0.014, 0.030)),
        material=stand_metal,
        name="rear_hinge_plate",
    )
    display_shell.visual(
        Cylinder(radius=0.014, length=0.082),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stand_metal,
        name="hinge_barrel",
    )
    display_shell.visual(
        Box((0.090, 0.036, 0.035)),
        origin=Origin(xyz=(0.0, -0.056, -0.088)),
        material=charcoal,
        name="joystick_mount",
    )
    display_shell.visual(
        Cylinder(radius=0.017, length=0.006),
        origin=Origin(xyz=(0.0, -0.056, -0.108)),
        material=black,
        name="joystick_socket",
    )

    joystick_gimbal = model.part("joystick_gimbal")
    joystick_gimbal.visual(
        Sphere(radius=0.008),
        origin=Origin(),
        material=rubber,
        name="gimbal_ball",
    )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=rubber,
        name="joystick_stem",
    )
    joystick.visual(
        Sphere(radius=0.011),
        origin=Origin(xyz=(0.0, 0.0, -0.032)),
        material=rubber,
        name="joystick_nub",
    )

    model.articulation(
        "base_to_outer_neck",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=outer_neck,
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2),
    )
    model.articulation(
        "outer_neck_to_inner_neck",
        ArticulationType.PRISMATIC,
        parent=outer_neck,
        child=inner_neck,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.18, lower=0.0, upper=0.100),
    )
    model.articulation(
        "inner_neck_to_display_shell",
        ArticulationType.REVOLUTE,
        parent=inner_neck,
        child=display_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.0, lower=-0.22, upper=0.28),
    )
    model.articulation(
        "display_shell_to_joystick_gimbal",
        ArticulationType.REVOLUTE,
        parent=display_shell,
        child=joystick_gimbal,
        origin=Origin(xyz=(0.0, -0.056, -0.119)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=-0.25, upper=0.25),
    )
    model.articulation(
        "joystick_gimbal_to_joystick",
        ArticulationType.REVOLUTE,
        parent=joystick_gimbal,
        child=joystick,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=-0.25, upper=0.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer_neck = object_model.get_part("outer_neck")
    inner_neck = object_model.get_part("inner_neck")
    display_shell = object_model.get_part("display_shell")
    joystick_gimbal = object_model.get_part("joystick_gimbal")
    joystick = object_model.get_part("joystick")

    swivel = object_model.get_articulation("base_to_outer_neck")
    slide = object_model.get_articulation("outer_neck_to_inner_neck")
    tilt = object_model.get_articulation("inner_neck_to_display_shell")
    joystick_pitch = object_model.get_articulation("display_shell_to_joystick_gimbal")
    joystick_roll = object_model.get_articulation("joystick_gimbal_to_joystick")

    def _center_from_aabb(aabb, axis_index: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])

    shell_aabb = ctx.part_element_world_aabb(display_shell, elem="boxy_shell")
    if shell_aabb is not None:
        shell_size = tuple(shell_aabb[1][i] - shell_aabb[0][i] for i in range(3))
    else:
        shell_size = None
    ctx.check(
        "boxy office monitor shell scale",
        shell_size is not None
        and 0.52 <= shell_size[0] <= 0.58
        and 0.045 <= shell_size[1] <= 0.070
        and 0.33 <= shell_size[2] <= 0.38,
        details=f"shell_size={shell_size}",
    )

    ctx.check(
        "stand has continuous swivel",
        swivel.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={swivel.articulation_type}",
    )
    ctx.check(
        "neck slide has business-monitor travel",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper is not None
        and 0.08 <= slide.motion_limits.upper <= 0.12,
        details=f"limits={slide.motion_limits}",
    )
    ctx.check(
        "tilt hinge has small realistic range",
        tilt.motion_limits is not None
        and tilt.motion_limits.lower is not None
        and tilt.motion_limits.upper is not None
        and tilt.motion_limits.lower < 0.0 < tilt.motion_limits.upper
        and tilt.motion_limits.upper <= 0.35,
        details=f"limits={tilt.motion_limits}",
    )

    ctx.expect_gap(
        outer_neck,
        base,
        axis="z",
        positive_elem="turntable_disk",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="swivel disk sits on pedestal bearing",
    )
    ctx.expect_within(
        inner_neck,
        outer_neck,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.0,
        name="inner mast stays inside sleeve footprint",
    )
    ctx.expect_overlap(
        inner_neck,
        outer_neck,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.12,
        name="collapsed neck has retained insertion",
    )
    ctx.expect_gap(
        display_shell,
        inner_neck,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="yoke_cheek_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="left yoke captures display hinge",
    )
    ctx.expect_gap(
        inner_neck,
        display_shell,
        axis="x",
        positive_elem="yoke_cheek_1",
        negative_elem="hinge_barrel",
        max_gap=0.001,
        max_penetration=0.0,
        name="right yoke captures display hinge",
    )
    ctx.expect_gap(
        display_shell,
        joystick_gimbal,
        axis="z",
        positive_elem="joystick_socket",
        negative_elem="gimbal_ball",
        max_gap=0.0015,
        max_penetration=0.0,
        name="joystick ball is supported by lower bezel socket",
    )
    ctx.expect_gap(
        joystick_gimbal,
        joystick,
        axis="z",
        positive_elem="gimbal_ball",
        negative_elem="joystick_stem",
        max_gap=0.001,
        max_penetration=0.0,
        name="joystick stem hangs from gimbal ball",
    )

    rest_inner_position = ctx.part_world_position(inner_neck)
    with ctx.pose({slide: 0.100}):
        extended_inner_position = ctx.part_world_position(inner_neck)
        ctx.expect_overlap(
            inner_neck,
            outer_neck,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.050,
            name="extended neck remains inserted in sleeve",
        )
    ctx.check(
        "height slide raises the stand head",
        rest_inner_position is not None
        and extended_inner_position is not None
        and extended_inner_position[2] > rest_inner_position[2] + 0.095,
        details=f"rest={rest_inner_position}, extended={extended_inner_position}",
    )

    screen_rest = ctx.part_element_world_aabb(display_shell, elem="screen_panel")
    rest_screen_y = _center_from_aabb(screen_rest, 1)
    with ctx.pose({tilt: 0.25}):
        screen_tilted = ctx.part_element_world_aabb(display_shell, elem="screen_panel")
        tilted_screen_y = _center_from_aabb(screen_tilted, 1)
    ctx.check(
        "positive tilt leans screen back",
        rest_screen_y is not None and tilted_screen_y is not None and tilted_screen_y > rest_screen_y + 0.020,
        details=f"rest_y={rest_screen_y}, tilted_y={tilted_screen_y}",
    )

    nub_rest = ctx.part_element_world_aabb(joystick, elem="joystick_nub")
    rest_nub_x = _center_from_aabb(nub_rest, 0)
    rest_nub_y = _center_from_aabb(nub_rest, 1)
    with ctx.pose({joystick_pitch: 0.24}):
        nub_pitch = ctx.part_element_world_aabb(joystick, elem="joystick_nub")
        pitch_nub_y = _center_from_aabb(nub_pitch, 1)
    with ctx.pose({joystick_roll: 0.24}):
        nub_roll = ctx.part_element_world_aabb(joystick, elem="joystick_nub")
        roll_nub_x = _center_from_aabb(nub_roll, 0)
    ctx.check(
        "joystick pivots fore and aft",
        rest_nub_y is not None and pitch_nub_y is not None and pitch_nub_y > rest_nub_y + 0.004,
        details=f"rest_y={rest_nub_y}, pitch_y={pitch_nub_y}",
    )
    ctx.check(
        "joystick pivots side to side",
        rest_nub_x is not None and roll_nub_x is not None and roll_nub_x < rest_nub_x - 0.004,
        details=f"rest_x={rest_nub_x}, roll_x={roll_nub_x}",
    )

    return ctx.report()


object_model = build_object_model()
