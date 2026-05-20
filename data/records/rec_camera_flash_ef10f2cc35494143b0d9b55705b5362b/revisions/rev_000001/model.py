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


def _body_shell_shape():
    lower = (
        cq.Workplane("XY")
        .box(0.064, 0.044, 0.082)
        .edges("|Z")
        .fillet(0.005)
        .translate((0.000, 0.000, 0.059))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.056, 0.040, 0.014)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.004, 0.000, 0.100))
    )
    neck = (
        cq.Workplane("XY")
        .box(0.026, 0.028, 0.014)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.006, 0.000, 0.107))
    )
    return lower.union(shoulder).union(neck)


def _shoe_mount_shape():
    foot = cq.Workplane("XY").box(0.020, 0.018, 0.003).translate((0.000, 0.000, 0.0015))
    left_rail = cq.Workplane("XY").box(0.020, 0.002, 0.004).translate((0.000, 0.008, 0.0035))
    right_rail = cq.Workplane("XY").box(0.020, 0.002, 0.004).translate((0.000, -0.008, 0.0035))
    riser = (
        cq.Workplane("XY")
        .box(0.015, 0.016, 0.015)
        .edges("|Z")
        .fillet(0.0015)
        .translate((0.000, 0.000, 0.0105))
    )
    lock_wheel = (
        cq.Workplane("XY")
        .cylinder(0.005, 0.005)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate((-0.006, -0.0105, 0.010))
    )
    clamp_tab = cq.Workplane("XY").box(0.004, 0.009, 0.002).translate((-0.001, -0.0095, 0.014))
    return foot.union(left_rail).union(right_rail).union(riser).union(lock_wheel).union(clamp_tab)


def _control_strip_shape():
    housing = (
        cq.Workplane("XY")
        .box(0.022, 0.014, 0.072)
        .edges("|Z")
        .fillet(0.002)
    )
    for slot_z in (-0.021, 0.0, 0.021):
        cutter = cq.Workplane("XY").box(0.016, 0.013, 0.016).translate((0.000, 0.0005, slot_z))
        housing = housing.cut(cutter)
    return housing


def _swivel_mount_shape():
    collar = cq.Workplane("XY").cylinder(0.012, 0.015)
    spine = (
        cq.Workplane("XY")
        .box(0.020, 0.028, 0.016)
        .edges("|Z")
        .fillet(0.002)
        .translate((0.004, 0.000, 0.010))
    )
    bridge = cq.Workplane("XY").box(0.012, 0.090, 0.012).translate((0.009, 0.000, 0.022))
    left_arm = (
        cq.Workplane("XY")
        .box(0.018, 0.006, 0.024)
        .edges("|Z")
        .fillet(0.001)
        .translate((0.023, 0.044, 0.022))
    )
    right_arm = (
        cq.Workplane("XY")
        .box(0.018, 0.006, 0.024)
        .edges("|Z")
        .fillet(0.001)
        .translate((0.023, -0.044, 0.022))
    )
    return collar.union(spine).union(bridge).union(left_arm).union(right_arm)


def _head_shell_shape():
    shell = (
        cq.Workplane("XY")
        .box(0.042, 0.074, 0.041)
        .edges("|Z")
        .fillet(0.004)
        .translate((0.017, 0.000, 0.008))
    )
    visor = cq.Workplane("XY").box(0.006, 0.074, 0.020).translate((-0.001, 0.000, 0.016))
    return shell.union(visor)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speedlight_flash")

    shell_black = model.material("shell_black", rgba=(0.12, 0.13, 0.14, 1.0))
    shell_dark = model.material("shell_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.08, 0.09, 0.10, 1.0))
    button_gray = model.material("button_gray", rgba=(0.30, 0.32, 0.35, 1.0))
    shoe_metal = model.material("shoe_metal", rgba=(0.57, 0.60, 0.64, 1.0))
    diffuser = model.material("diffuser", rgba=(0.92, 0.95, 0.97, 0.82))
    sensor_red = model.material("sensor_red", rgba=(0.64, 0.12, 0.10, 0.85))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "speedlight_body_shell"),
        material=shell_black,
        name="body_shell",
    )
    body.visual(
        mesh_from_cadquery(_shoe_mount_shape(), "speedlight_shoe_mount"),
        material=shoe_metal,
        name="shoe_mount",
    )
    body.visual(
        Box((0.036, 0.0015, 0.054)),
        origin=Origin(xyz=(-0.002, -0.02275, 0.060)),
        material=panel_dark,
        name="battery_door",
    )
    body.visual(
        Box((0.0014, 0.016, 0.012)),
        origin=Origin(xyz=(0.0327, 0.000, 0.056)),
        material=sensor_red,
        name="sensor_window",
    )

    control_strip = model.part("control_strip")
    control_strip.visual(
        mesh_from_cadquery(_control_strip_shape(), "speedlight_control_strip"),
        material=shell_dark,
        name="housing",
    )
    model.articulation(
        "body_to_control_strip",
        ArticulationType.FIXED,
        parent=body,
        child=control_strip,
        origin=Origin(xyz=(-0.010, 0.029, 0.066)),
    )

    button_z_positions = (0.021, 0.0, -0.021)
    for index, local_z in enumerate(button_z_positions):
        button = model.part(f"mode_button_{index}")
        button.visual(
            Box((0.016, 0.0035, 0.016)),
            origin=Origin(xyz=(0.000, 0.00175, 0.000)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.012, 0.0040, 0.012)),
            origin=Origin(xyz=(0.000, -0.0020, 0.000)),
            material=button_gray,
            name="stem",
        )
        model.articulation(
            f"control_strip_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=control_strip,
            child=button,
            origin=Origin(xyz=(0.000, 0.007, local_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0018,
            ),
        )

    swivel = model.part("swivel")
    swivel.visual(
        mesh_from_cadquery(_swivel_mount_shape(), "speedlight_swivel_mount"),
        material=shell_dark,
        name="mount",
    )
    model.articulation(
        "body_to_swivel",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel,
        origin=Origin(xyz=(0.006, 0.000, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.8,
            upper=1.8,
        ),
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "speedlight_head_shell"),
        material=shell_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.066, 0.030)),
        origin=Origin(xyz=(0.040, 0.000, 0.008)),
        material=diffuser,
        name="diffuser",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, 0.039, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_dark,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.000, -0.039, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_dark,
        name="trunnion_1",
    )
    model.articulation(
        "swivel_to_head",
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.026, 0.000, 0.026)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.2,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    control_strip = object_model.get_part("control_strip")
    swivel = object_model.get_part("swivel")
    head = object_model.get_part("head")
    swivel_joint = object_model.get_articulation("body_to_swivel")
    tilt_joint = object_model.get_articulation("swivel_to_head")

    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.010,
        name="lamp head clears the battery body at rest",
    )
    ctx.expect_gap(
        control_strip,
        body,
        axis="y",
        max_penetration=0.0005,
        name="control strip sits on the body side without deep embed",
    )
    ctx.expect_gap(
        swivel,
        body,
        axis="z",
        max_gap=0.012,
        max_penetration=1e-5,
        name="swivel collar stays seated close to the body top",
    )

    swivel_limits = swivel_joint.motion_limits
    tilt_limits = tilt_joint.motion_limits
    rest_head_pos = ctx.part_world_position(head)

    if swivel_limits is not None and swivel_limits.upper is not None:
        with ctx.pose({swivel_joint: swivel_limits.upper}):
            swivel_head_pos = ctx.part_world_position(head)
        ctx.check(
            "swivel turns the head sideways",
            rest_head_pos is not None
            and swivel_head_pos is not None
            and swivel_head_pos[1] > rest_head_pos[1] + 0.025,
            details=f"rest={rest_head_pos}, swivel={swivel_head_pos}",
        )

    rest_diffuser = ctx.part_element_world_aabb(head, elem="diffuser")
    if tilt_limits is not None and tilt_limits.upper is not None:
        with ctx.pose({tilt_joint: tilt_limits.upper}):
            raised_diffuser = ctx.part_element_world_aabb(head, elem="diffuser")
        ctx.check(
            "tilt lifts the lamp head upward",
            rest_diffuser is not None
            and raised_diffuser is not None
            and raised_diffuser[1][2] > rest_diffuser[1][2] + 0.018,
            details=f"rest={rest_diffuser}, raised={raised_diffuser}",
        )

    for index in range(3):
        button = object_model.get_part(f"mode_button_{index}")
        button_joint = object_model.get_articulation(f"control_strip_to_mode_button_{index}")
        limits = button_joint.motion_limits

        ctx.expect_gap(
            button,
            control_strip,
            axis="y",
            positive_elem="button_cap",
            negative_elem="housing",
            min_gap=0.0,
            max_gap=0.0005,
            name=f"mode button {index} rests flush to the strip face",
        )
        ctx.expect_within(
            button,
            control_strip,
            axes="xz",
            inner_elem="stem",
            outer_elem="housing",
            margin=0.0005,
            name=f"mode button {index} stem stays guided in the strip",
        )

        rest_button_pos = ctx.part_world_position(button)
        if limits is not None and limits.upper is not None:
            with ctx.pose({button_joint: limits.upper}):
                pressed_button_pos = ctx.part_world_position(button)
                ctx.expect_within(
                    button,
                    control_strip,
                    axes="xz",
                    inner_elem="stem",
                    outer_elem="housing",
                    margin=0.0005,
                    name=f"mode button {index} stays aligned when pressed",
                )
            ctx.check(
                f"mode button {index} presses inward",
                rest_button_pos is not None
                and pressed_button_pos is not None
                and pressed_button_pos[1] < rest_button_pos[1] - 0.0010,
                details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
            )

    return ctx.report()


object_model = build_object_model()
