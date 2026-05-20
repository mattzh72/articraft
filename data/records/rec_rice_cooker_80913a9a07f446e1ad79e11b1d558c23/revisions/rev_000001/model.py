from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _build_body_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .box(0.310, 0.368, 0.205, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.060)
        .edges(">Z")
        .fillet(0.026)
    )
    cooker_well = (
        cq.Workplane("XY")
        .box(0.222, 0.252, 0.134, centered=(True, True, False))
        .translate((0.005, 0.000, 0.072))
        .edges("|Z")
        .fillet(0.030)
    )
    return outer.cut(cooker_well)


def _build_lid_shell() -> object:
    outer = (
        cq.Workplane("XY")
        .box(0.262, 0.296, 0.044, centered=(False, True, False))
        .edges("|Z")
        .fillet(0.038)
        .edges(">Z")
        .fillet(0.016)
    )
    crown = (
        cq.Workplane("XY")
        .ellipse(0.082, 0.112)
        .extrude(0.014)
        .translate((0.135, 0.000, 0.036))
    )
    return outer.union(crown).faces("<Z").shell(-0.004)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="family_rice_cooker")

    shell_white = model.material("shell_white", rgba=(0.93, 0.93, 0.90, 1.0))
    lid_silver = model.material("lid_silver", rgba=(0.84, 0.85, 0.86, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.18, 0.28, 0.30, 0.55))
    warm_grey = model.material("warm_grey", rgba=(0.66, 0.67, 0.68, 1.0))
    amber = model.material("amber", rgba=(0.90, 0.60, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "rice_cooker_body_shell"),
        material=shell_white,
        name="body_shell",
    )
    body.visual(
        Box((0.288, 0.344, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=charcoal,
        name="base_skirt",
    )
    body.visual(
        Box((0.040, 0.270, 0.028)),
        origin=Origin(xyz=(-0.138, 0.000, 0.194)),
        material=charcoal,
        name="hinge_band",
    )
    body.visual(
        Box((0.026, 0.018, 0.040)),
        origin=Origin(xyz=(-0.018, 0.177, 0.214)),
        material=charcoal,
        name="handle_mount_0",
    )
    body.visual(
        Box((0.026, 0.018, 0.040)),
        origin=Origin(xyz=(-0.018, -0.177, 0.214)),
        material=charcoal,
        name="handle_mount_1",
    )
    body.visual(
        Box((0.014, 0.126, 0.094)),
        origin=Origin(xyz=(0.156, 0.000, 0.089)),
        material=charcoal,
        name="control_bezel",
    )
    body.visual(
        Box((0.004, 0.060, 0.024)),
        origin=Origin(xyz=(0.165, 0.000, 0.104)),
        material=dark_glass,
        name="display",
    )
    body.visual(
        Box((0.004, 0.016, 0.008)),
        origin=Origin(xyz=(0.165, -0.028, 0.076)),
        material=warm_grey,
        name="mode_key_0",
    )
    body.visual(
        Box((0.004, 0.016, 0.008)),
        origin=Origin(xyz=(0.165, 0.028, 0.076)),
        material=warm_grey,
        name="mode_key_1",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.165, -0.040, 0.100), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=amber,
        name="status_lamp_0",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.165, 0.040, 0.100), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=amber,
        name="status_lamp_1",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.310, 0.368, 0.220)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.110)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "rice_cooker_lid_shell"),
        material=lid_silver,
        name="lid_shell",
    )
    lid.visual(
        Box((0.032, 0.070, 0.010)),
        origin=Origin(xyz=(0.086, 0.000, 0.051)),
        material=charcoal,
        name="steam_cap",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.262, 0.296, 0.060)),
        mass=1.1,
        origin=Origin(xyz=(0.131, 0.000, 0.030)),
    )

    handle = model.part("handle")
    arm_pitch = math.atan2(0.090, 0.082)
    handle.visual(
        Cylinder(radius=0.0085, length=0.300),
        origin=Origin(xyz=(0.090, 0.000, 0.082), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=charcoal,
        name="grip_bar",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.1218),
        origin=Origin(xyz=(0.045, 0.150, 0.041), rpy=(0.000, arm_pitch, 0.000)),
        material=charcoal,
        name="arm_0",
    )
    handle.visual(
        Cylinder(radius=0.0065, length=0.1218),
        origin=Origin(xyz=(0.045, -0.150, 0.041), rpy=(0.000, arm_pitch, 0.000)),
        material=charcoal,
        name="arm_1",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.000, 0.159, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=warm_grey,
        name="pivot_0",
    )
    handle.visual(
        Cylinder(radius=0.0075, length=0.018),
        origin=Origin(xyz=(0.000, -0.159, 0.000), rpy=(math.pi / 2.0, 0.000, 0.000)),
        material=warm_grey,
        name="pivot_1",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.130, 0.320, 0.110)),
        mass=0.28,
        origin=Origin(xyz=(0.045, 0.000, 0.045)),
    )

    power_button = model.part("power_button")
    power_button.visual(
        Cylinder(radius=0.013, length=0.004),
        origin=Origin(xyz=(0.002, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
        material=amber,
        name="button_cap",
    )
    power_button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.004),
        mass=0.02,
        origin=Origin(xyz=(0.002, 0.000, 0.000), rpy=(0.000, math.pi / 2.0, 0.000)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.122, 0.000, 0.206)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(108),
        ),
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.018, 0.000, 0.225)),
        axis=(0.000, -1.000, 0.000),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(70),
        ),
    )
    model.articulation(
        "body_to_power_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=power_button,
        origin=Origin(xyz=(0.159, 0.000, 0.052)),
        axis=(-1.000, 0.000, 0.000),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.050,
            lower=0.0,
            upper=0.003,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    power_button = object_model.get_part("power_button")

    lid_hinge = object_model.get_articulation("body_to_lid")
    handle_joint = object_model.get_articulation("body_to_handle")
    button_slide = object_model.get_articulation("body_to_power_button")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.180,
            name="closed lid covers the wide body opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="body_shell",
            max_gap=0.030,
            max_penetration=0.0,
            name="closed lid rests above the body without collision",
        )

    lid_upper = lid_hinge.motion_limits.upper if lid_hinge.motion_limits else None
    if lid_upper is not None:
        with ctx.pose({lid_hinge: lid_upper}):
            body_aabb = ctx.part_world_aabb(body)
            lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
            ctx.expect_gap(
                handle,
                lid,
                axis="x",
                min_gap=0.050,
                name="opened lid clears the stowed carry handle",
            )
            ctx.check(
                "lid opens upward from the rear hinge",
                body_aabb is not None
                and lid_aabb is not None
                and lid_aabb[1][2] > body_aabb[1][2] + 0.090,
                details=f"body_aabb={body_aabb}, lid_aabb={lid_aabb}",
            )

    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits else None
    handle_rest = None
    with ctx.pose({handle_joint: 0.0}):
        body_aabb = ctx.part_world_aabb(body)
        handle_rest = ctx.part_element_world_aabb(handle, elem="grip_bar")
        ctx.check(
            "stowed handle remains clearly above the shell",
            body_aabb is not None
            and handle_rest is not None
            and handle_rest[0][2] > body_aabb[1][2] + 0.012,
            details=f"body_aabb={body_aabb}, handle_rest={handle_rest}",
        )
    if handle_upper is not None:
        with ctx.pose({handle_joint: handle_upper}):
            handle_raised = ctx.part_element_world_aabb(handle, elem="grip_bar")
        ctx.check(
            "handle lifts into a carry position",
            handle_rest is not None
            and handle_raised is not None
            and handle_raised[1][2] > handle_rest[1][2] + 0.020,
            details=f"handle_rest={handle_rest}, handle_raised={handle_raised}",
        )

    button_rest = None
    with ctx.pose({button_slide: 0.0}):
        button_rest = ctx.part_element_world_aabb(power_button, elem="button_cap")
    button_upper = button_slide.motion_limits.upper if button_slide.motion_limits else None
    if button_upper is not None:
        with ctx.pose({button_slide: button_upper}):
            button_pressed = ctx.part_element_world_aabb(power_button, elem="button_cap")
        ctx.check(
            "power button presses inward below the display",
            button_rest is not None
            and button_pressed is not None
            and button_pressed[1][0] < button_rest[1][0] - 0.002,
            details=f"button_rest={button_rest}, button_pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
