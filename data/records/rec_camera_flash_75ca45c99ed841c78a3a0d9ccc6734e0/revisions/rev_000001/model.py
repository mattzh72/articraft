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


BODY_TO_NECK_Z = 0.126
NECK_TO_HEAD_Z = 0.062
BUTTON_TRAVEL = 0.0015
TILT_LOWER = math.radians(-7.0)
TILT_UPPER = math.radians(120.0)


def _body_shell_shape() -> cq.Workplane:
    lower = cq.Workplane("XY").box(0.076, 0.080, 0.090, centered=(True, True, False))
    upper = (
        cq.Workplane("XY")
        .box(0.058, 0.074, 0.014, centered=(True, True, False))
        .translate((0.006, 0.0, 0.090))
    )
    nose = (
        cq.Workplane("XY")
        .box(0.018, 0.068, 0.010, centered=(True, True, False))
        .translate((0.016, 0.0, 0.104))
    )
    shell = lower.union(upper).union(nose)
    shell = shell.edges("|Z").fillet(0.007)
    return shell


def _neck_shape() -> cq.Workplane:
    stem = (
        cq.Workplane("XY")
        .box(0.022, 0.018, 0.050, centered=(True, True, False))
        .translate((-0.008, 0.0, 0.0))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.018, 0.038, 0.014, centered=(True, True, False))
        .translate((-0.010, 0.0, 0.038))
    )
    bridge = (
        cq.Workplane("XY")
        .box(0.008, 0.104, 0.012, centered=(True, True, False))
        .translate((-0.014, 0.0, 0.050))
    )
    arm_0 = (
        cq.Workplane("XY")
        .box(0.012, 0.006, 0.020, centered=(True, True, False))
        .translate((0.0, -0.050, 0.050))
    )
    arm_1 = (
        cq.Workplane("XY")
        .box(0.012, 0.006, 0.020, centered=(True, True, False))
        .translate((0.0, 0.050, 0.050))
    )
    neck = stem.union(shoulder).union(bridge).union(arm_0).union(arm_1)
    return neck


def _head_shell_shape() -> cq.Workplane:
    head = cq.Workplane("XY").box(0.082, 0.090, 0.050).translate((0.048, 0.0, 0.006))
    head = head.edges("|X").fillet(0.011)
    return head


def _add_button(
    model: ArticulatedObject,
    body,
    name: str,
    *,
    seat_xyz: tuple[float, float, float],
    size_yz: tuple[float, float],
    material,
) -> None:
    width_y, height_z = size_yz
    button = model.part(name)
    button.visual(
        Box((0.004, width_y, height_z)),
        origin=Origin(xyz=(-0.002, 0.0, 0.0)),
        material=material,
        name="button_cap",
    )
    button.visual(
        Box((0.0012, width_y * 0.72, height_z * 0.72)),
        origin=Origin(xyz=(-0.0041, 0.0, 0.0)),
        material=material,
        name="button_face",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=seat_xyz),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=0.05,
            lower=0.0,
            upper=BUTTON_TRAVEL,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="professional_speedlight")

    body_black = model.material("body_black", rgba=(0.12, 0.13, 0.14, 1.0))
    body_dark = model.material("body_dark", rgba=(0.18, 0.19, 0.20, 1.0))
    panel_dark = model.material("panel_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    button_grey = model.material("button_grey", rgba=(0.28, 0.29, 0.31, 1.0))
    diffuser = model.material("diffuser", rgba=(0.90, 0.92, 0.93, 0.95))
    screen = model.material("screen", rgba=(0.20, 0.43, 0.45, 0.65))
    metal = model.material("metal", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell_shape(), "speedlight_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=body_black,
        name="body_shell",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.121)),
        material=body_dark,
        name="swivel_collar",
    )
    body.visual(
        Box((0.028, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=metal,
        name="shoe_block",
    )
    body.visual(
        Box((0.036, 0.022, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=metal,
        name="shoe_rail",
    )
    body.visual(
        Box((0.004, 0.060, 0.070)),
        origin=Origin(xyz=(-0.038, 0.0, 0.077)),
        material=panel_dark,
        name="rear_panel",
    )
    body.visual(
        Box((0.003, 0.040, 0.030)),
        origin=Origin(xyz=(-0.0405, 0.0, 0.078)),
        material=body_dark,
        name="screen_bezel",
    )
    body.visual(
        Box((0.002, 0.028, 0.018)),
        origin=Origin(xyz=(-0.041, 0.0, 0.078)),
        material=screen,
        name="screen",
    )

    neck = model.part("neck")
    neck.visual(
        Box((0.022, 0.018, 0.050)),
        origin=Origin(xyz=(-0.008, 0.0, 0.025)),
        material=body_black,
        name="neck_stem",
    )
    neck.visual(
        Box((0.020, 0.044, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.045)),
        material=body_black,
        name="neck_shoulder",
    )
    neck.visual(
        Box((0.012, 0.104, 0.012)),
        origin=Origin(xyz=(-0.010, 0.0, 0.056)),
        material=body_dark,
        name="rear_bridge",
    )
    neck.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, -0.0505, 0.060)),
        material=body_black,
        name="arm_0",
    )
    neck.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.0505, 0.060)),
        material=body_black,
        name="arm_1",
    )
    neck.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.001, -0.0575, NECK_TO_HEAD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="hinge_boss_0",
    )
    neck.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(0.001, 0.0575, NECK_TO_HEAD_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="hinge_boss_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_shape(), "speedlight_head_shell"),
        material=body_black,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.072, 0.036)),
        origin=Origin(xyz=(0.091, 0.0, 0.006)),
        material=diffuser,
        name="flash_window",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.012, -0.0455, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.003),
        origin=Origin(xyz=(0.012, 0.0455, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_dark,
        name="trunnion_1",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, BODY_TO_NECK_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, NECK_TO_HEAD_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=TILT_LOWER,
            upper=TILT_UPPER,
        ),
    )

    _add_button(
        model,
        body,
        "button_0",
        seat_xyz=(-0.040, -0.018, 0.097),
        size_yz=(0.010, 0.008),
        material=button_grey,
    )
    _add_button(
        model,
        body,
        "button_1",
        seat_xyz=(-0.040, 0.018, 0.097),
        size_yz=(0.010, 0.008),
        material=button_grey,
    )
    _add_button(
        model,
        body,
        "button_2",
        seat_xyz=(-0.040, -0.026, 0.078),
        size_yz=(0.008, 0.010),
        material=button_grey,
    )
    _add_button(
        model,
        body,
        "button_3",
        seat_xyz=(-0.040, 0.026, 0.078),
        size_yz=(0.008, 0.010),
        material=button_grey,
    )
    _add_button(
        model,
        body,
        "button_4",
        seat_xyz=(-0.040, -0.015, 0.055),
        size_yz=(0.012, 0.008),
        material=button_grey,
    )
    _add_button(
        model,
        body,
        "button_5",
        seat_xyz=(-0.040, 0.015, 0.055),
        size_yz=(0.012, 0.008),
        material=button_grey,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    button = object_model.get_part("button_2")
    neck_swivel = object_model.get_articulation("body_to_neck")
    head_tilt = object_model.get_articulation("neck_to_head")
    button_joint = object_model.get_articulation("body_to_button_2")

    ctx.allow_overlap(
        head,
        neck,
        elem_a="trunnion_0",
        elem_b="arm_0",
        reason="The tilt-axis trunnion is intentionally captured by the exposed cradle arm.",
    )
    ctx.allow_overlap(
        head,
        neck,
        elem_a="trunnion_1",
        elem_b="arm_1",
        reason="The tilt-axis trunnion is intentionally captured by the exposed cradle arm.",
    )

    ctx.expect_gap(
        body,
        button,
        axis="x",
        positive_elem="rear_panel",
        negative_elem="button_cap",
        max_gap=0.0,
        max_penetration=0.0,
        name="rear button seats on panel",
    )
    ctx.expect_overlap(
        button,
        body,
        axes="yz",
        elem_a="button_cap",
        elem_b="rear_panel",
        min_overlap=0.007,
        name="rear button stays within rear panel footprint",
    )

    def _center_from_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    rest_head = _center_from_aabb(ctx.part_element_world_aabb(head, elem="flash_window"))
    with ctx.pose({head_tilt: TILT_UPPER}):
        raised_head = _center_from_aabb(ctx.part_element_world_aabb(head, elem="flash_window"))
    ctx.check(
        "head tilts upward",
        rest_head is not None
        and raised_head is not None
        and raised_head[2] > rest_head[2] + 0.03,
        details=f"rest={rest_head}, raised={raised_head}",
    )

    with ctx.pose({neck_swivel: math.pi / 2.0}):
        turned_head = _center_from_aabb(ctx.part_element_world_aabb(head, elem="flash_window"))
    ctx.check(
        "neck swivels head sideways",
        rest_head is not None
        and turned_head is not None
        and turned_head[1] > rest_head[1] + 0.025,
        details=f"rest={rest_head}, turned={turned_head}",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed_button = ctx.part_world_position(button)
        ctx.expect_overlap(
            button,
            body,
            axes="yz",
            elem_a="button_cap",
            elem_b="rear_panel",
            min_overlap=0.007,
            name="pressed button stays aligned with rear panel",
        )
    ctx.check(
        "rear button presses inward",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[0] > rest_button[0] + BUTTON_TRAVEL * 0.9,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
