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


def _arc_points(
    center_x: float,
    center_z: float,
    radius: float,
    start_deg: float,
    end_deg: float,
    *,
    segments: int = 24,
) -> list[tuple[float, float]]:
    if end_deg < start_deg:
        end_deg += 360.0
    points: list[tuple[float, float]] = []
    for index in range(segments + 1):
        t = index / segments
        angle = math.radians(start_deg + (end_deg - start_deg) * t)
        points.append(
            (
                center_x + radius * math.cos(angle),
                center_z + radius * math.sin(angle),
            )
        )
    return points


def _annular_sector(
    center_x: float,
    center_z: float,
    inner_radius: float,
    outer_radius: float,
    start_deg: float,
    end_deg: float,
    *,
    thickness: float,
) -> cq.Workplane:
    outer = _arc_points(center_x, center_z, outer_radius, start_deg, end_deg, segments=36)
    inner = list(
        reversed(
            _arc_points(center_x, center_z, inner_radius, start_deg, end_deg, segments=36)
        )
    )
    profile = outer + inner
    return cq.Workplane("XZ").polyline(profile).close().extrude(thickness / 2.0, both=True)


def _body_shell_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .rect(0.048, 0.034)
        .workplane(offset=0.060)
        .rect(0.058, 0.036)
        .workplane(offset=0.052)
        .rect(0.084, 0.040)
        .workplane(offset=0.048)
        .rect(0.068, 0.036)
        .loft(combine=True)
    )
    base_pad = cq.Workplane("XY").box(0.042, 0.034, 0.012).translate((0.0, 0.0, 0.006))
    return shell.union(base_pad)


def _fixed_jaw_shape() -> cq.Workplane:
    fixed_sector = _annular_sector(
        0.012,
        0.188,
        0.024,
        0.034,
        242.0,
        82.0,
        thickness=0.032,
    )
    bridge = cq.Workplane("XY").box(0.034, 0.034, 0.026).translate((0.018, 0.0, 0.167))
    ear = cq.Workplane("XZ").center(0.012, 0.222).circle(0.0055).extrude(0.004, both=True)
    ear_0 = ear.translate((0.0, -0.013, 0.0))
    ear_1 = ear.translate((0.0, 0.013, 0.0))
    return fixed_sector.union(bridge).union(ear_0).union(ear_1)


def _moving_jaw_shape() -> cq.Workplane:
    jaw_sector = _annular_sector(
        -0.005,
        -0.034,
        0.024,
        0.034,
        112.0,
        228.0,
        thickness=0.032,
    )
    rib = cq.Workplane("XY").box(0.030, 0.020, 0.022).translate((-0.008, 0.0, -0.005))
    return jaw_sector.union(rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="automotive_clamp_meter")

    shell_orange = model.material("shell_orange", rgba=(0.90, 0.46, 0.15, 1.0))
    shell_black = model.material("shell_black", rgba=(0.16, 0.17, 0.18, 1.0))
    grip_black = model.material("grip_black", rgba=(0.09, 0.09, 0.10, 1.0))
    dial_black = model.material("dial_black", rgba=(0.12, 0.12, 0.13, 1.0))
    button_gray = model.material("button_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    display_glass = model.material("display_glass", rgba=(0.37, 0.55, 0.48, 0.58))

    meter_body = model.part("meter_body")
    meter_body.visual(
        mesh_from_cadquery(_body_shell_shape(), "meter_body_shell"),
        material=shell_orange,
        name="body_shell",
    )
    meter_body.visual(
        mesh_from_cadquery(_fixed_jaw_shape(), "meter_fixed_jaw"),
        material=shell_black,
        name="fixed_jaw",
    )
    meter_body.visual(
        Box((0.076, 0.003, 0.108)),
        origin=Origin(xyz=(0.0, 0.0195, 0.087)),
        material=grip_black,
        name="front_panel",
    )
    meter_body.visual(
        Box((0.052, 0.004, 0.032)),
        origin=Origin(xyz=(0.0, 0.020, 0.124)),
        material=shell_black,
        name="display_bezel",
    )
    meter_body.visual(
        Box((0.042, 0.0016, 0.022)),
        origin=Origin(xyz=(0.0, 0.0228, 0.124)),
        material=display_glass,
        name="display_window",
    )
    meter_body.visual(
        Box((0.016, 0.005, 0.088)),
        origin=Origin(xyz=(-0.021, 0.0175, 0.052)),
        material=grip_black,
        name="left_grip",
    )
    meter_body.visual(
        Box((0.016, 0.005, 0.088)),
        origin=Origin(xyz=(0.021, 0.0175, 0.052)),
        material=grip_black,
        name="right_grip",
    )
    meter_body.visual(
        Box((0.028, 0.004, 0.016)),
        origin=Origin(xyz=(0.0, -0.0185, 0.022)),
        material=shell_black,
        name="stand_mount",
    )

    jaw = model.part("jaw")
    jaw.visual(
        mesh_from_cadquery(_moving_jaw_shape(), "meter_moving_jaw"),
        material=shell_black,
        name="jaw_shell",
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_skirt",
    )
    dial.visual(
        Cylinder(radius=0.0185, length=0.013),
        origin=Origin(xyz=(0.0, 0.0065, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_black,
        name="dial_cap",
    )
    dial.visual(
        Box((0.0032, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.011, 0.013)),
        material=button_gray,
        name="dial_pointer",
    )

    button_origins = (-0.019, 0.0, 0.019)
    buttons = []
    for index, x_pos in enumerate(button_origins):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.013, 0.006, 0.009)),
            origin=Origin(xyz=(0.0, 0.003, 0.0)),
            material=button_gray,
            name="button_cap",
        )
        buttons.append(button)
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=meter_body,
            child=button,
            origin=Origin(xyz=(x_pos, 0.021, 0.091)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.04,
                lower=0.0,
                upper=0.0025,
            ),
        )

    stand = model.part("stand")
    stand.visual(
        Box((0.042, 0.004, 0.048)),
        origin=Origin(xyz=(0.0, -0.002, 0.024)),
        material=shell_black,
        name="stand_panel",
    )
    stand.visual(
        Box((0.028, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.005, 0.047)),
        material=shell_black,
        name="stand_foot",
    )

    model.articulation(
        "body_to_jaw",
        ArticulationType.REVOLUTE,
        parent=meter_body,
        child=jaw,
        origin=Origin(xyz=(0.012, 0.0, 0.222)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=0.85,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=meter_body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.021, 0.049)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=8.0,
        ),
    )
    model.articulation(
        "body_to_stand",
        ArticulationType.REVOLUTE,
        parent=meter_body,
        child=stand,
        origin=Origin(xyz=(0.0, -0.019, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("meter_body")
    jaw = object_model.get_part("jaw")
    dial = object_model.get_part("dial")
    stand = object_model.get_part("stand")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    jaw_joint = object_model.get_articulation("body_to_jaw")
    dial_joint = object_model.get_articulation("body_to_dial")
    stand_joint = object_model.get_articulation("body_to_stand")
    button_0_joint = object_model.get_articulation("body_to_button_0")

    ctx.allow_overlap(
        jaw,
        body,
        reason="The top clamp hinge is represented as a tight interleaved jaw knuckle around the head pivot.",
    )

    ctx.expect_gap(
        body,
        button_1,
        axis="z",
        positive_elem="display_window",
        negative_elem="button_cap",
        min_gap=0.010,
        name="display sits above the button bank",
    )
    ctx.expect_gap(
        button_1,
        dial,
        axis="z",
        positive_elem="button_cap",
        negative_elem="dial_skirt",
        min_gap=0.012,
        name="button bank is clearly separated from the selector dial",
    )
    ctx.expect_contact(
        dial,
        body,
        elem_a="dial_skirt",
        elem_b="front_panel",
        contact_tol=0.0015,
        name="dial seats against the front control panel",
    )

    jaw_closed = ctx.part_element_world_aabb(jaw, elem="jaw_shell")
    dial_pointer_0 = ctx.part_element_world_aabb(dial, elem="dial_pointer")
    button_0_rest = ctx.part_world_position(button_0)
    button_1_rest = ctx.part_world_position(button_1)
    stand_closed = ctx.part_element_world_aabb(stand, elem="stand_panel")

    jaw_open = None
    dial_pointer_1 = None
    button_0_pressed = None
    button_1_steady = None
    stand_open = None
    jaw_upper = jaw_joint.motion_limits.upper if jaw_joint.motion_limits is not None else None
    stand_upper = stand_joint.motion_limits.upper if stand_joint.motion_limits is not None else None
    button_upper = (
        button_0_joint.motion_limits.upper if button_0_joint.motion_limits is not None else None
    )

    if jaw_upper is not None:
        with ctx.pose({jaw_joint: jaw_upper}):
            jaw_open = ctx.part_element_world_aabb(jaw, elem="jaw_shell")

    with ctx.pose({dial_joint: 1.0}):
        dial_pointer_1 = ctx.part_element_world_aabb(dial, elem="dial_pointer")

    if button_upper is not None:
        with ctx.pose({button_0_joint: button_upper}):
            button_0_pressed = ctx.part_world_position(button_0)
            button_1_steady = ctx.part_world_position(button_1)

    if stand_upper is not None:
        with ctx.pose({stand_joint: stand_upper}):
            stand_open = ctx.part_element_world_aabb(stand, elem="stand_panel")

    ctx.check(
        "jaw opens away from the clamp throat",
        jaw_closed is not None
        and jaw_open is not None
        and jaw_open[0][0] < jaw_closed[0][0] - 0.010
        and jaw_open[0][2] > jaw_closed[0][2] + 0.010,
        details=f"closed={jaw_closed}, open={jaw_open}",
    )
    ctx.check(
        "dial rotates as an independent selector",
        dial_pointer_0 is not None
        and dial_pointer_1 is not None
        and abs(dial_pointer_1[0][0] - dial_pointer_0[0][0]) > 0.001,
        details=f"rest={dial_pointer_0}, turned={dial_pointer_1}",
    )
    ctx.check(
        "front button presses inward without dragging neighbors",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_1_rest is not None
        and button_1_steady is not None
        and button_0_pressed[1] < button_0_rest[1] - 0.0015
        and abs(button_1_steady[1] - button_1_rest[1]) < 1e-6,
        details=(
            f"button_0_rest={button_0_rest}, button_0_pressed={button_0_pressed}, "
            f"button_1_rest={button_1_rest}, button_1_steady={button_1_steady}"
        ),
    )
    ctx.check(
        "rear stand folds backward from the back shell",
        stand_closed is not None
        and stand_open is not None
        and stand_open[0][1] < stand_closed[0][1] - 0.015,
        details=f"closed={stand_closed}, open={stand_open}",
    )

    return ctx.report()


object_model = build_object_model()
