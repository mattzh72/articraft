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


BASE_WIDTH = 0.260
BASE_DEPTH = 0.224
BASE_HEIGHT = 0.220
DRIVE_PLATFORM_SIZE = 0.152
DRIVE_PLATFORM_HEIGHT = 0.014
DRIVE_PLATFORM_TOP_Z = 0.224

BUTTON_XS = (-0.076, -0.038, 0.000, 0.038, 0.076)
BUTTON_TRAVEL = 0.004
BUTTON_BODY_SIZE = (0.030, 0.014, 0.019)
BUTTON_CAP_SIZE = (0.032, 0.004, 0.021)
BUTTON_PANEL_Y = -0.095
BUTTON_CENTER_Z = 0.075

JAR_COLLAR_SIZE = 0.160
JAR_COLLAR_HEIGHT = 0.022
JAR_HEIGHT = 0.300


def _make_base_shell() -> object:
    side_profile = [
        (-0.112, 0.000),
        (0.112, 0.000),
        (0.112, 0.182),
        (0.050, 0.182),
        (0.028, 0.220),
        (-0.046, 0.220),
        (-0.082, 0.145),
        (-0.095, 0.110),
        (-0.095, 0.040),
        (-0.112, 0.000),
    ]
    body = (
        cq.Workplane("YZ")
        .polyline(side_profile)
        .close()
        .extrude(BASE_WIDTH, both=True)
    )

    panel_recess = (
        cq.Workplane("XY")
        .box(0.210, 0.004, 0.072)
        .translate((0.000, -0.093, BUTTON_CENTER_Z))
    )
    body = body.cut(panel_recess)

    cavity = None
    for x_pos in BUTTON_XS:
        cutter = (
            cq.Workplane("XY")
            .box(0.034, 0.016, 0.023)
            .translate((x_pos, -0.087, BUTTON_CENTER_Z))
        )
        cavity = cutter if cavity is None else cavity.union(cutter)
    if cavity is not None:
        body = body.cut(cavity)

    drive_platform = (
        cq.Workplane("XY")
        .box(DRIVE_PLATFORM_SIZE, DRIVE_PLATFORM_SIZE, DRIVE_PLATFORM_HEIGHT)
        .translate((0.000, 0.000, DRIVE_PLATFORM_TOP_Z - DRIVE_PLATFORM_HEIGHT * 0.5))
    )
    return body.union(drive_platform)


def _make_jar_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .rect(0.148, 0.148)
        .workplane(offset=0.045)
        .rect(0.160, 0.160)
        .workplane(offset=0.120)
        .rect(0.182, 0.182)
        .workplane(offset=0.135)
        .rect(0.194, 0.194)
        .loft(combine=True)
    )
    inner = (
        cq.Workplane("XY", origin=(0.000, 0.000, 0.014))
        .rect(0.134, 0.134)
        .workplane(offset=0.036)
        .rect(0.146, 0.146)
        .workplane(offset=0.120)
        .rect(0.166, 0.166)
        .workplane(offset=0.135)
        .rect(0.180, 0.180)
        .loft(combine=True)
    )
    shell = outer.cut(inner)

    handle = (
        cq.Workplane("XY")
        .box(0.024, 0.026, 0.060)
        .translate((0.086, 0.000, 0.092))
        .union(
            cq.Workplane("XY")
            .box(0.024, 0.026, 0.052)
            .translate((0.088, 0.000, 0.224))
        )
        .union(
            cq.Workplane("XY")
            .box(0.052, 0.026, 0.024)
            .translate((0.108, 0.000, 0.109))
        )
        .union(
            cq.Workplane("XY")
            .box(0.054, 0.026, 0.024)
            .translate((0.110, 0.000, 0.238))
        )
        .union(
            cq.Workplane("XY")
            .box(0.020, 0.026, 0.164)
            .translate((0.130, 0.000, 0.171))
        )
    )
    return shell.union(handle)


def _make_jar_collar() -> object:
    collar = cq.Workplane("XY").box(JAR_COLLAR_SIZE, JAR_COLLAR_SIZE, JAR_COLLAR_HEIGHT)
    opening = cq.Workplane("XY").box(0.110, 0.110, JAR_COLLAR_HEIGHT + 0.004)
    return collar.cut(opening)


def _make_blade_assembly() -> object:
    hub = cq.Workplane("XY").circle(0.014).extrude(0.010).translate((0.000, 0.000, -0.004))
    shaft = cq.Workplane("XY").circle(0.005).extrude(0.021).translate((0.000, 0.000, -0.020))

    blade_a = (
        cq.Workplane("XY")
        .box(0.064, 0.010, 0.0018)
        .rotate((0.000, 0.000, 0.000), (1.000, 0.000, 0.000), 12.0)
        .rotate((0.000, 0.000, 0.000), (0.000, 0.000, 1.000), 22.0)
        .translate((0.000, 0.000, 0.006))
    )
    blade_b = (
        cq.Workplane("XY")
        .box(0.060, 0.010, 0.0018)
        .rotate((0.000, 0.000, 0.000), (1.000, 0.000, 0.000), -13.0)
        .rotate((0.000, 0.000, 0.000), (0.000, 0.000, 1.000), 112.0)
        .translate((0.000, 0.000, 0.004))
    )

    return hub.union(shaft).union(blade_a).union(blade_b)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_speed_blender")

    body_finish = model.material("body_finish", rgba=(0.18, 0.19, 0.21, 1.0))
    rubber_dark = model.material("rubber_dark", rgba=(0.08, 0.08, 0.09, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.76, 0.79, 1.0))
    clear_tritan = model.material("clear_tritan", rgba=(0.82, 0.88, 0.94, 0.28))
    button_finish = model.material("button_finish", rgba=(0.82, 0.84, 0.86, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.67, 0.70, 0.74, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shell(), "blender_base_shell"),
        material=body_finish,
        name="base_shell",
    )
    base.visual(
        Box((0.198, 0.004, 0.060)),
        origin=Origin(xyz=(0.000, -0.093, BUTTON_CENTER_Z)),
        material=stainless,
        name="control_strip",
    )
    base.visual(
        Box((0.062, 0.062, 0.003)),
        origin=Origin(xyz=(0.000, 0.000, DRIVE_PLATFORM_TOP_Z + 0.0015)),
        material=stainless,
        name="drive_plate",
    )
    base.visual(
        Box((DRIVE_PLATFORM_SIZE, DRIVE_PLATFORM_SIZE, DRIVE_PLATFORM_HEIGHT)),
        origin=Origin(
            xyz=(0.000, 0.000, DRIVE_PLATFORM_TOP_Z - DRIVE_PLATFORM_HEIGHT * 0.5)
        ),
        material=rubber_dark,
        name="drive_platform",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((-0.094, -0.074), (0.094, -0.074), (-0.094, 0.074), (0.094, 0.074))
    ):
        base.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=rubber_dark,
            name=f"foot_{index}",
        )

    jar = model.part("jar")
    jar.visual(
        mesh_from_cadquery(_make_jar_collar(), "blender_jar_collar"),
        origin=Origin(xyz=(0.000, 0.000, JAR_COLLAR_HEIGHT * 0.5)),
        material=stainless,
        name="jar_collar",
    )
    jar.visual(
        mesh_from_cadquery(_make_jar_shell(), "blender_jar_shell"),
        material=clear_tritan,
        name="jar_shell",
    )

    blade_assembly = model.part("blade_assembly")
    blade_assembly.visual(
        mesh_from_cadquery(_make_blade_assembly(), "blender_blade_assembly"),
        material=blade_steel,
        name="blade_span",
    )

    model.articulation(
        "base_to_jar",
        ArticulationType.FIXED,
        parent=base,
        child=jar,
        origin=Origin(xyz=(0.000, 0.000, DRIVE_PLATFORM_TOP_Z)),
    )
    model.articulation(
        "jar_to_blade_assembly",
        ArticulationType.CONTINUOUS,
        parent=jar,
        child=blade_assembly,
        origin=Origin(xyz=(0.000, 0.000, 0.034)),
        axis=(0.000, 0.000, 1.000),
        motion_limits=MotionLimits(effort=2.5, velocity=50.0),
    )

    for index, x_pos in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box(BUTTON_BODY_SIZE),
            origin=Origin(xyz=(0.000, 0.001, 0.000)),
            material=button_finish,
            name="button_body",
        )
        button.visual(
            Box(BUTTON_CAP_SIZE),
            origin=Origin(xyz=(0.000, -0.004, 0.000)),
            material=stainless,
            name="button_cap",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, BUTTON_PANEL_Y, BUTTON_CENTER_Z)),
            axis=(0.000, 1.000, 0.000),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.060,
                lower=0.000,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    jar = object_model.get_part("jar")
    blade_assembly = object_model.get_part("blade_assembly")
    blade_joint = object_model.get_articulation("jar_to_blade_assembly")

    buttons = [object_model.get_part(f"button_{index}") for index in range(5)]
    button_joints = [
        object_model.get_articulation(f"base_to_button_{index}") for index in range(5)
    ]

    ctx.check(
        "five_independent_buttons_present",
        len(buttons) == 5 and len(button_joints) == 5,
        details=f"buttons={len(buttons)}, joints={len(button_joints)}",
    )
    ctx.check(
        "buttons_use_prismatic_travel",
        all(joint.articulation_type == ArticulationType.PRISMATIC for joint in button_joints),
        details=str([joint.articulation_type for joint in button_joints]),
    )
    ctx.check(
        "blade_joint_is_continuous",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type}",
    )

    ctx.expect_contact(
        jar,
        base,
        elem_a="jar_collar",
        elem_b="drive_platform",
        name="jar collar seats on drive platform",
    )
    ctx.expect_within(
        blade_assembly,
        jar,
        axes="xy",
        inner_elem="blade_span",
        outer_elem="jar_shell",
        margin=0.010,
        name="blade assembly stays within jar footprint",
    )

    rest_button_positions = [ctx.part_world_position(button) for button in buttons]
    center_travel = button_joints[2].motion_limits.upper
    edge_travel = button_joints[0].motion_limits.upper
    rest_blade_position = ctx.part_world_position(blade_assembly)

    with ctx.pose({button_joints[2]: center_travel}):
        pressed_center_position = ctx.part_world_position(buttons[2])
        untouched_neighbor_position = ctx.part_world_position(buttons[1])

    with ctx.pose({button_joints[0]: edge_travel, blade_joint: math.pi / 2.0}):
        pressed_edge_position = ctx.part_world_position(buttons[0])
        spun_blade_position = ctx.part_world_position(blade_assembly)

    ctx.check(
        "center_button_depresses_inward",
        rest_button_positions[2] is not None
        and pressed_center_position is not None
        and pressed_center_position[1] > rest_button_positions[2][1] + 0.003,
        details=f"rest={rest_button_positions[2]}, pressed={pressed_center_position}",
    )
    ctx.check(
        "adjacent_button_remains_stationary",
        rest_button_positions[1] is not None
        and untouched_neighbor_position is not None
        and abs(untouched_neighbor_position[1] - rest_button_positions[1][1]) < 1e-6,
        details=f"rest={rest_button_positions[1]}, posed={untouched_neighbor_position}",
    )
    ctx.check(
        "edge_button_depresses_independently",
        rest_button_positions[0] is not None
        and pressed_edge_position is not None
        and pressed_edge_position[1] > rest_button_positions[0][1] + 0.003,
        details=f"rest={rest_button_positions[0]}, pressed={pressed_edge_position}",
    )
    ctx.check(
        "blade_spins_in_place",
        rest_blade_position is not None
        and spun_blade_position is not None
        and all(
            abs(spun_blade_position[axis] - rest_blade_position[axis]) < 1e-6
            for axis in range(3)
        ),
        details=f"rest={rest_blade_position}, spun={spun_blade_position}",
    )

    return ctx.report()


object_model = build_object_model()
