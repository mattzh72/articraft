from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

BODY_WIDTH = 0.050
BODY_THICKNESS = 0.0175
BODY_LENGTH = 0.145
BODY_FRONT_Y = BODY_THICKNESS * 0.5

CONTROL_RECESS_WIDTH = 0.036
CONTROL_RECESS_LENGTH = 0.114
CONTROL_RECESS_DEPTH = 0.0007
CONTROL_RECESS_Z = -0.002

DISPLAY_WIDTH = 0.028
DISPLAY_LENGTH = 0.014
DISPLAY_RECESS_DEPTH = 0.0016
DISPLAY_Z = 0.044

ROCKER_WIDTH = 0.0215
ROCKER_DEPTH = 0.0024
ROCKER_HALF_LENGTH = 0.018
ROCKER_BRIDGE_LENGTH = 0.008
ROCKER_POCKET_WIDTH = 0.024
ROCKER_POCKET_LENGTH = 0.042
ROCKER_POCKET_DEPTH = 0.0036
ROCKER_Z = 0.004

BUTTON_WIDTH = 0.0122
BUTTON_DEPTH = 0.0022
BUTTON_LENGTH = 0.0095
BUTTON_POCKET_WIDTH = 0.0132
BUTTON_POCKET_LENGTH = 0.0108
BUTTON_POCKET_DEPTH = 0.0028
BUTTON_TRAVEL = 0.0012
BUTTON_Z = -0.036
BUTTON_XS = (-0.015, 0.0, 0.015)


def _rounded_prism(size_x: float, size_y: float, size_z: float, radius: float):
    prism = cq.Workplane("XY").box(size_x, size_y, size_z)
    return prism.edges("|Y").fillet(radius)


def _build_body_shape():
    shell = cq.Workplane("XY").box(BODY_WIDTH, BODY_THICKNESS, BODY_LENGTH)
    shell = shell.edges("|Z").fillet(0.006)

    recess = _rounded_prism(
        CONTROL_RECESS_WIDTH,
        CONTROL_RECESS_DEPTH,
        CONTROL_RECESS_LENGTH,
        0.004,
    ).translate((0.0, BODY_FRONT_Y - CONTROL_RECESS_DEPTH * 0.5, CONTROL_RECESS_Z))
    shell = shell.cut(recess)

    display_recess = _rounded_prism(
        DISPLAY_WIDTH + 0.006,
        DISPLAY_RECESS_DEPTH,
        DISPLAY_LENGTH + 0.004,
        0.0025,
    ).translate((0.0, BODY_FRONT_Y - DISPLAY_RECESS_DEPTH * 0.5, DISPLAY_Z))
    shell = shell.cut(display_recess)

    rocker_pocket = _rounded_prism(
        ROCKER_POCKET_WIDTH,
        ROCKER_POCKET_DEPTH,
        ROCKER_POCKET_LENGTH,
        0.004,
    ).translate((0.0, BODY_FRONT_Y - ROCKER_POCKET_DEPTH * 0.5, ROCKER_Z))
    shell = shell.cut(rocker_pocket)

    for button_x in BUTTON_XS:
        button_pocket = _rounded_prism(
            BUTTON_POCKET_WIDTH,
            BUTTON_POCKET_DEPTH,
            BUTTON_POCKET_LENGTH,
            0.002,
        ).translate((button_x, BODY_FRONT_Y - BUTTON_POCKET_DEPTH * 0.5, BUTTON_Z))
        shell = shell.cut(button_pocket)

    return shell


def _button_back_y() -> float:
    return BODY_FRONT_Y - BUTTON_POCKET_DEPTH + BUTTON_TRAVEL


def _build_button_mesh():
    return _rounded_prism(BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_LENGTH, 0.0014)


def _build_rocker_half_mesh():
    return _rounded_prism(ROCKER_WIDTH, ROCKER_DEPTH, ROCKER_HALF_LENGTH, 0.0025)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_conditioner_remote")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.92, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.82, 0.84, 0.85, 1.0))
    button_grey = model.material("button_grey", rgba=(0.70, 0.72, 0.74, 1.0))
    rocker_grey = model.material("rocker_grey", rgba=(0.78, 0.80, 0.81, 1.0))
    display_glass = model.material("display_glass", rgba=(0.18, 0.31, 0.34, 0.65))
    display_dark = model.material("display_dark", rgba=(0.11, 0.13, 0.14, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "remote_body_shell"),
        material=body_white,
        name="shell",
    )
    body.visual(
        Box((CONTROL_RECESS_WIDTH - 0.004, 0.0005, CONTROL_RECESS_LENGTH - 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_FRONT_Y - CONTROL_RECESS_DEPTH + 0.00025,
                CONTROL_RECESS_Z,
            )
        ),
        material=panel_grey,
        name="panel_insert",
    )
    body.visual(
        Box((DISPLAY_WIDTH, 0.0008, DISPLAY_LENGTH)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_FRONT_Y - DISPLAY_RECESS_DEPTH + 0.0004,
                DISPLAY_Z,
            )
        ),
        material=display_dark,
        name="display_window",
    )
    body.visual(
        Box((DISPLAY_WIDTH - 0.007, 0.00035, DISPLAY_LENGTH - 0.004)),
        origin=Origin(
            xyz=(
                0.0,
                BODY_FRONT_Y - DISPLAY_RECESS_DEPTH + 0.0008,
                DISPLAY_Z,
            )
        ),
        material=display_glass,
        name="display_glow",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_THICKNESS, BODY_LENGTH)),
        mass=0.14,
    )

    rocker_half_mesh = mesh_from_cadquery(_build_rocker_half_mesh(), "temperature_rocker_half")
    button_mesh = mesh_from_cadquery(_build_button_mesh(), "mode_button_cap")

    rocker = model.part("temperature_rocker")
    rocker.visual(
        rocker_half_mesh,
        origin=Origin(xyz=(0.0, 0.0, ROCKER_HALF_LENGTH * 0.5)),
        material=rocker_grey,
        name="rocker_top",
    )
    rocker.visual(
        rocker_half_mesh,
        origin=Origin(xyz=(0.0, 0.0, -ROCKER_HALF_LENGTH * 0.5)),
        material=rocker_grey,
        name="rocker_bottom",
    )
    rocker.visual(
        Box((0.010, ROCKER_DEPTH * 0.9, ROCKER_BRIDGE_LENGTH)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=rocker_grey,
        name="rocker_bridge",
    )
    rocker.inertial = Inertial.from_geometry(
        Box((ROCKER_WIDTH, ROCKER_DEPTH, ROCKER_HALF_LENGTH * 2.0)),
        mass=0.009,
    )
    model.articulation(
        "body_to_temperature_rocker",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rocker,
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0011, ROCKER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.12,
            upper=0.12,
        ),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"mode_button_{index}")
        button.visual(
            button_mesh,
            origin=Origin(xyz=(0.0, BUTTON_DEPTH * 0.5, 0.0)),
            material=button_grey,
            name="button_cap",
        )
        button.inertial = Inertial.from_geometry(
            Box((BUTTON_WIDTH, BUTTON_DEPTH, BUTTON_LENGTH)),
            mass=0.003,
            origin=Origin(xyz=(0.0, BUTTON_DEPTH * 0.5, 0.0)),
        )
        model.articulation(
            f"body_to_mode_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, _button_back_y(), BUTTON_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[index] + upper[index]) * 0.5 for index in range(3))


def _aabb_max_y(aabb):
    if aabb is None:
        return None
    return aabb[1][1]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    rocker = object_model.get_part("temperature_rocker")
    rocker_joint = object_model.get_articulation("body_to_temperature_rocker")
    buttons = [object_model.get_part(f"mode_button_{index}") for index in range(3)]
    button_joints = [object_model.get_articulation(f"body_to_mode_button_{index}") for index in range(3)]

    display_center = _aabb_center(ctx.part_element_world_aabb(body, elem="display_window"))
    rocker_center = _aabb_center(ctx.part_world_aabb(rocker))
    button_centers = [_aabb_center(ctx.part_world_aabb(button)) for button in buttons]
    ctx.check(
        "display sits above rocker and mode buttons",
        display_center is not None
        and rocker_center is not None
        and all(center is not None for center in button_centers)
        and display_center[2] > rocker_center[2] + 0.025
        and all(rocker_center[2] > center[2] + 0.025 for center in button_centers),
        details=(
            f"display_center={display_center}, rocker_center={rocker_center}, "
            f"button_centers={button_centers}"
        ),
    )

    button_xs = [center[0] for center in button_centers if center is not None]
    left_pitch = button_xs[1] - button_xs[0] if len(button_xs) == 3 else None
    right_pitch = button_xs[2] - button_xs[1] if len(button_xs) == 3 else None
    ctx.check(
        "mode buttons form an even front row",
        left_pitch is not None
        and right_pitch is not None
        and 0.013 < left_pitch < 0.017
        and abs(left_pitch - right_pitch) < 0.001,
        details=f"button_xs={button_xs}, left_pitch={left_pitch}, right_pitch={right_pitch}",
    )

    rest_button_positions = [ctx.part_world_position(button) for button in buttons]
    for index, joint in enumerate(button_joints):
        with ctx.pose({joint: BUTTON_TRAVEL}):
            posed_positions = [ctx.part_world_position(button) for button in buttons]
        ctx.check(
            f"mode_button_{index} depresses independently",
            rest_button_positions[index] is not None
            and posed_positions[index] is not None
            and posed_positions[index][1] < rest_button_positions[index][1] - 0.001
            and all(
                rest_button_positions[other] is not None
                and posed_positions[other] is not None
                and abs(posed_positions[other][1] - rest_button_positions[other][1]) < 1e-6
                for other in range(3)
                if other != index
            ),
            details=(
                f"rest_positions={rest_button_positions}, posed_positions={posed_positions}"
            ),
        )

    rest_top_center = _aabb_center(ctx.part_element_world_aabb(rocker, elem="rocker_top"))
    rest_bottom_center = _aabb_center(ctx.part_element_world_aabb(rocker, elem="rocker_bottom"))
    with ctx.pose({rocker_joint: rocker_joint.motion_limits.upper}):
        top_pressed_center = _aabb_center(ctx.part_element_world_aabb(rocker, elem="rocker_top"))
        bottom_released_center = _aabb_center(ctx.part_element_world_aabb(rocker, elem="rocker_bottom"))
    ctx.check(
        "positive rocker pose presses the upper end",
        rest_top_center is not None
        and rest_bottom_center is not None
        and top_pressed_center is not None
        and bottom_released_center is not None
        and top_pressed_center[1] < rest_top_center[1] - 0.0009
        and bottom_released_center[1] > rest_bottom_center[1] + 0.0009,
        details=(
            f"rest_top_center={rest_top_center}, top_pressed_center={top_pressed_center}, "
            f"rest_bottom_center={rest_bottom_center}, bottom_released_center={bottom_released_center}"
        ),
    )

    with ctx.pose({rocker_joint: rocker_joint.motion_limits.lower}):
        top_released_center = _aabb_center(ctx.part_element_world_aabb(rocker, elem="rocker_top"))
        bottom_pressed_center = _aabb_center(ctx.part_element_world_aabb(rocker, elem="rocker_bottom"))
    ctx.check(
        "negative rocker pose presses the lower end",
        rest_top_center is not None
        and rest_bottom_center is not None
        and top_released_center is not None
        and bottom_pressed_center is not None
        and top_released_center[1] > rest_top_center[1] + 0.0009
        and bottom_pressed_center[1] < rest_bottom_center[1] - 0.0009,
        details=(
            f"rest_top_center={rest_top_center}, top_released_center={top_released_center}, "
            f"rest_bottom_center={rest_bottom_center}, bottom_pressed_center={bottom_pressed_center}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
