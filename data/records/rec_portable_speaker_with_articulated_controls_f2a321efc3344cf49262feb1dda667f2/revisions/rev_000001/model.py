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


BODY_WIDTH = 0.290
BODY_DEPTH = 0.115
BODY_HEIGHT = 0.162
BODY_WALL = 0.008

HANDLE_PIVOT_Z = BODY_HEIGHT + 0.006
HANDLE_PIVOT_Y = 0.010
HANDLE_SPAN = BODY_WIDTH - 0.012


def _build_body_shell():
    outer = (
        cq.Workplane("XY")
        .box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.017)
    )
    cavity = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, -0.005, BODY_WALL))
        .box(
            BODY_WIDTH - (2.0 * BODY_WALL),
            BODY_DEPTH - BODY_WALL + 0.002,
            BODY_HEIGHT - (2.0 * BODY_WALL),
            centered=(True, True, False),
        )
    )
    return outer.cut(cavity)


def _center_from_aabb(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((a + b) * 0.5 for a, b in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_bluetooth_speaker")

    shell_color = model.material("shell_color", rgba=(0.15, 0.16, 0.17, 1.0))
    baffle_color = model.material("baffle_color", rgba=(0.09, 0.09, 0.10, 1.0))
    grille_color = model.material("grille_color", rgba=(0.19, 0.20, 0.21, 1.0))
    metal_color = model.material("metal_color", rgba=(0.58, 0.60, 0.63, 1.0))
    trim_color = model.material("trim_color", rgba=(0.30, 0.31, 0.33, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "speaker_shell"),
        material=shell_color,
        name="shell",
    )
    body.visual(
        Box((BODY_WIDTH - (2.0 * BODY_WALL), 0.004, BODY_HEIGHT - (2.0 * BODY_WALL))),
        origin=Origin(
            xyz=(
                0.0,
                -(BODY_DEPTH * 0.5) + 0.002,
                BODY_WALL + ((BODY_HEIGHT - (2.0 * BODY_WALL)) * 0.5),
            )
        ),
        material=baffle_color,
        name="front_baffle",
    )
    body.visual(
        Box((0.188, 0.008, 0.104)),
        origin=Origin(
            xyz=(
                -0.038,
                -(BODY_DEPTH * 0.5) + 0.004,
                0.076,
            )
        ),
        material=grille_color,
        name="grille",
    )
    body.visual(
        Box((0.074, 0.010, 0.108)),
        origin=Origin(
            xyz=(
                0.086,
                -(BODY_DEPTH * 0.5) + 0.005,
                0.078,
            )
        ),
        material=trim_color,
        name="control_face",
    )
    for x_sign, visual_name in ((-1.0, "pivot_boss_0"), (1.0, "pivot_boss_1")):
        body.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(
                xyz=(
                    x_sign * ((HANDLE_SPAN * 0.5) + 0.006),
                    HANDLE_PIVOT_Y,
                    HANDLE_PIVOT_Z,
                ),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=metal_color,
            name=visual_name,
        )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    handle = model.part("handle")
    handle_grip_length = HANDLE_SPAN - 0.034
    handle.visual(
        Box((handle_grip_length, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.034, 0.014)),
        material=trim_color,
        name="handle_grip",
    )
    arm_rise_y = 0.034
    arm_rise_z = 0.014
    arm_length = math.sqrt((arm_rise_y * arm_rise_y) + (arm_rise_z * arm_rise_z)) + 0.002
    arm_roll = -math.atan2(arm_rise_y, arm_rise_z)
    arm_center_y = arm_rise_y * 0.5
    arm_center_z = arm_rise_z * 0.5
    arm_center_x = (HANDLE_SPAN * 0.5) - 0.009
    for x_sign, visual_name in ((-1.0, "handle_arm_0"), (1.0, "handle_arm_1")):
        handle.visual(
            Box((0.030, 0.010, arm_length)),
            origin=Origin(
                xyz=(x_sign * arm_center_x, arm_center_y, arm_center_z),
                rpy=(arm_roll, 0.0, 0.0),
            ),
            material=trim_color,
            name=visual_name,
        )
    handle.inertial = Inertial.from_geometry(
        Box((HANDLE_SPAN, 0.050, 0.040)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.018, 0.018)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(0.0, HANDLE_PIVOT_Y, HANDLE_PIVOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )

    body.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(
            xyz=(0.086, -(BODY_DEPTH * 0.5) + 0.002, 0.060),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=metal_color,
        name="wheel_bezel",
    )
    for index, button_x in enumerate((0.060, 0.086, 0.112)):
        body.visual(
            Box((0.018, 0.002, 0.014)),
            origin=Origin(
                xyz=(button_x, -(BODY_DEPTH * 0.5) + 0.001, 0.104),
            ),
            material=metal_color,
            name=f"button_seat_{index}",
        )

    volume_wheel = model.part("volume_wheel")
    volume_wheel.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(
            xyz=(0.0, -0.006, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=trim_color,
        name="wheel_tire",
    )
    volume_wheel.visual(
        Cylinder(radius=0.010, length=0.013),
        origin=Origin(
            xyz=(0.0, -0.0065, 0.0),
            rpy=(math.pi * 0.5, 0.0, 0.0),
        ),
        material=metal_color,
        name="wheel_core",
    )
    volume_wheel.visual(
        Box((0.004, 0.0025, 0.009)),
        origin=Origin(xyz=(0.0, -0.012, 0.0125)),
        material=metal_color,
        name="wheel_marker",
    )
    volume_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.012),
        mass=0.035,
        origin=Origin(xyz=(0.0, -0.006, 0.0)),
    )
    model.articulation(
        "body_to_volume_wheel",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=volume_wheel,
        origin=Origin(xyz=(0.086, -(BODY_DEPTH * 0.5), 0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=12.0,
        ),
    )

    for index, button_x in enumerate((0.060, 0.086, 0.112)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.014, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
            material=metal_color,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((0.014, 0.004, 0.010)),
            mass=0.008,
            origin=Origin(xyz=(0.0, -0.002, 0.0)),
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, -(BODY_DEPTH * 0.5), 0.104)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=5.0,
                velocity=0.05,
                lower=0.0,
                upper=0.0025,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    handle_joint = object_model.get_articulation("body_to_handle")
    wheel = object_model.get_part("volume_wheel")
    wheel_joint = object_model.get_articulation("body_to_volume_wheel")
    buttons = [object_model.get_part(f"button_{index}") for index in range(3)]
    button_joints = [
        object_model.get_articulation(f"body_to_button_{index}") for index in range(3)
    ]
    limits = handle_joint.motion_limits

    if limits is not None and limits.upper is not None:
        with ctx.pose({handle_joint: 0.0}):
            ctx.expect_gap(
                handle,
                body,
                axis="z",
                positive_elem="handle_grip",
                negative_elem="shell",
                min_gap=0.002,
                name="folded handle clears the cabinet top",
            )
            folded_grip = _center_from_aabb(
                ctx.part_element_world_aabb(handle, elem="handle_grip")
            )
        with ctx.pose({handle_joint: limits.upper}):
            raised_grip = _center_from_aabb(
                ctx.part_element_world_aabb(handle, elem="handle_grip")
            )
        ctx.check(
            "handle lifts upward from the folded pose",
            folded_grip is not None
            and raised_grip is not None
            and raised_grip[2] > folded_grip[2] + 0.018,
            details=f"folded_grip={folded_grip}, raised_grip={raised_grip}",
        )

    ctx.check(
        "volume wheel uses continuous rotation",
        wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={wheel_joint.articulation_type}",
    )
    wheel_marker_rest = _center_from_aabb(
        ctx.part_element_world_aabb(wheel, elem="wheel_marker")
    )
    with ctx.pose({wheel_joint: math.pi * 0.5}):
        wheel_marker_quarter_turn = _center_from_aabb(
            ctx.part_element_world_aabb(wheel, elem="wheel_marker")
        )
    ctx.check(
        "wheel marker swings around the axle",
        wheel_marker_rest is not None
        and wheel_marker_quarter_turn is not None
        and abs(wheel_marker_quarter_turn[0] - wheel_marker_rest[0]) > 0.010,
        details=(
            f"wheel_marker_rest={wheel_marker_rest}, "
            f"wheel_marker_quarter_turn={wheel_marker_quarter_turn}"
        ),
    )

    center_button_limits = button_joints[1].motion_limits
    if center_button_limits is not None and center_button_limits.upper is not None:
        rest_positions = [ctx.part_world_position(button) for button in buttons]
        with ctx.pose({button_joints[1]: center_button_limits.upper}):
            pressed_positions = [ctx.part_world_position(button) for button in buttons]
        ctx.check(
            "center button depresses inward on the control face",
            rest_positions[1] is not None
            and pressed_positions[1] is not None
            and pressed_positions[1][1] > rest_positions[1][1] + 0.0015,
            details=f"rest_positions={rest_positions}, pressed_positions={pressed_positions}",
        )
        ctx.check(
            "buttons remain independent from one another",
            rest_positions[0] is not None
            and pressed_positions[0] is not None
            and rest_positions[2] is not None
            and pressed_positions[2] is not None
            and abs(pressed_positions[0][1] - rest_positions[0][1]) < 1e-6
            and abs(pressed_positions[2][1] - rest_positions[2][1]) < 1e-6,
            details=f"rest_positions={rest_positions}, pressed_positions={pressed_positions}",
        )

    for index, button in enumerate(buttons):
        ctx.expect_overlap(
            button,
            body,
            axes="xz",
            elem_a="button_cap",
            elem_b="control_face",
            min_overlap=0.008,
            name=f"button_{index} stays seated within the control bank",
        )

    return ctx.report()


object_model = build_object_model()
