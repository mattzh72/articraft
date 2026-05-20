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


def _head_shell_mesh():
    shell = (
        cq.Workplane("XY")
        .box(0.058, 0.082, 0.050)
        .edges("|X")
        .fillet(0.011)
        .translate((0.029, 0.0, 0.014))
    )
    return mesh_from_cadquery(shell, "speedlight_head_shell")


def _add_mode_button(
    model: ArticulatedObject,
    body,
    button_name: str,
    joint_name: str,
    *,
    x: float,
    z: float,
    material,
):
    button = model.part(button_name)
    button.visual(
        Box((0.010, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.002, 0.0)),
        material=material,
        name="button_stem",
    )
    button.visual(
        Box((0.014, 0.0045, 0.012)),
        origin=Origin(xyz=(0.0, 0.0058, 0.0)),
        material=material,
        name="button_cap",
    )

    articulation = model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(x, 0.036, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.05,
            lower=0.0,
            upper=0.002,
        ),
    )
    return button, articulation


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="speedlight")

    shell_dark = model.material("shell_dark", rgba=(0.14, 0.15, 0.17, 1.0))
    shell_mid = model.material("shell_mid", rgba=(0.20, 0.21, 0.23, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.09, 0.10, 0.11, 1.0))
    diffuser = model.material("diffuser", rgba=(0.86, 0.88, 0.90, 0.92))
    window = model.material("window", rgba=(0.18, 0.20, 0.22, 0.65))

    body = model.part("body")
    body.visual(
        Box((0.074, 0.064, 0.086)),
        origin=Origin(xyz=(-0.010, 0.0, -0.043)),
        material=shell_dark,
        name="battery_housing",
    )
    body.visual(
        Box((0.036, 0.056, 0.040)),
        origin=Origin(xyz=(0.020, 0.0, -0.066)),
        material=shell_mid,
        name="front_grip",
    )
    body.visual(
        Box((0.044, 0.060, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, -0.016)),
        material=shell_mid,
        name="top_shoulder",
    )
    body.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=trim_dark,
        name="swivel_socket",
    )
    body.visual(
        Box((0.048, 0.004, 0.056)),
        origin=Origin(xyz=(-0.004, 0.034, -0.046)),
        material=trim_dark,
        name="side_strip",
    )
    body.visual(
        Box((0.040, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
        material=shell_dark,
        name="shoe_block",
    )
    body.visual(
        Box((0.026, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.012, -0.098)),
        material=trim_dark,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.026, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.012, -0.098)),
        material=trim_dark,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.003, 0.022, 0.016)),
        origin=Origin(xyz=(0.0395, 0.0, -0.040)),
        material=window,
        name="af_window",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.0205, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=trim_dark,
        name="neck_collar",
    )
    neck.visual(
        Box((0.030, 0.034, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=shell_dark,
        name="neck_stem",
    )
    neck.visual(
        Box((0.012, 0.094, 0.018)),
        origin=Origin(xyz=(-0.010, 0.0, 0.061)),
        material=shell_mid,
        name="rear_bridge",
    )
    neck.visual(
        Box((0.020, 0.006, 0.032)),
        origin=Origin(xyz=(0.000, -0.047, 0.061)),
        material=shell_mid,
        name="yoke_arm_0",
    )
    neck.visual(
        Box((0.020, 0.006, 0.032)),
        origin=Origin(xyz=(0.000, 0.047, 0.061)),
        material=shell_mid,
        name="yoke_arm_1",
    )
    neck.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(
            xyz=(0.0, -0.050, 0.073),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="pivot_boss_0",
    )
    neck.visual(
        Cylinder(radius=0.0065, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.050, 0.073),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="pivot_boss_1",
    )

    head = model.part("head")
    head.visual(
        _head_shell_mesh(),
        material=shell_mid,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(
            xyz=(0.004, -0.0425, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="trunnion_0",
    )
    head.visual(
        Cylinder(radius=0.0065, length=0.003),
        origin=Origin(
            xyz=(0.004, 0.0425, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="trunnion_1",
    )
    head.visual(
        Box((0.003, 0.066, 0.028)),
        origin=Origin(xyz=(0.0595, 0.0, 0.014)),
        material=diffuser,
        name="flash_lens",
    )
    head.visual(
        Box((0.030, 0.060, 0.003)),
        origin=Origin(xyz=(0.028, 0.0, 0.0405)),
        material=trim_dark,
        name="top_panel",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=neck,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=5.0),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=math.radians(-7.0),
            upper=math.radians(120.0),
        ),
    )

    for index, button_z in enumerate((-0.024, -0.040, -0.056, -0.072)):
        _add_mode_button(
            model,
            body,
            f"mode_button_{index}",
            f"body_to_mode_button_{index}",
            x=-0.006,
            z=button_z,
            material=trim_dark,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    neck = object_model.get_part("neck")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")

    ctx.expect_gap(
        neck,
        body,
        axis="z",
        positive_elem="neck_collar",
        negative_elem="swivel_socket",
        max_gap=0.001,
        max_penetration=0.0,
        name="neck collar seats on swivel socket",
    )
    ctx.expect_overlap(
        neck,
        body,
        axes="xy",
        elem_a="neck_collar",
        elem_b="swivel_socket",
        min_overlap=0.035,
        name="neck collar stays centered over the body swivel",
    )

    for index in range(4):
        button = object_model.get_part(f"mode_button_{index}")
        joint = object_model.get_articulation(f"body_to_mode_button_{index}")
        limits = joint.motion_limits
        if limits is None or limits.upper is None:
            continue

        ctx.expect_gap(
            button,
            body,
            axis="y",
            positive_elem="button_stem",
            negative_elem="side_strip",
            max_gap=0.0005,
            max_penetration=1e-6,
            name=f"mode button {index} rests on the side strip",
        )
        ctx.allow_overlap(
            button,
            body,
            elem_a="button_stem",
            elem_b="side_strip",
            reason="The button stem retracts into a simplified switch pocket behind the side control strip.",
        )

        rest_pos = ctx.part_world_position(button)
        neighbor = object_model.get_part(f"mode_button_{(index + 1) % 4}")
        neighbor_rest = ctx.part_world_position(neighbor)
        with ctx.pose({joint: limits.upper}):
            pressed_pos = ctx.part_world_position(button)
            neighbor_pressed = ctx.part_world_position(neighbor)

        ctx.check(
            f"mode button {index} depresses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )
        ctx.check(
            f"mode button {index} moves independently",
            neighbor_rest is not None
            and neighbor_pressed is not None
            and abs(neighbor_pressed[1] - neighbor_rest[1]) < 1e-6,
            details=f"neighbor_rest={neighbor_rest}, neighbor_pressed={neighbor_pressed}",
        )

    tilt_limits = tilt.motion_limits
    if tilt_limits is not None and tilt_limits.upper is not None:
        rest_lens = ctx.part_element_world_aabb(head, elem="flash_lens")
        with ctx.pose({tilt: tilt_limits.upper}):
            raised_lens = ctx.part_element_world_aabb(head, elem="flash_lens")
        rest_center = _aabb_center(rest_lens)
        raised_center = _aabb_center(raised_lens)
        ctx.check(
            "head tilts the flash lens upward",
            rest_center is not None
            and raised_center is not None
            and raised_center[2] > rest_center[2] + 0.025,
            details=f"rest_center={rest_center}, raised_center={raised_center}",
        )

    rest_lens = ctx.part_element_world_aabb(head, elem="flash_lens")
    with ctx.pose({swivel: math.pi / 2.0}):
        rotated_lens = ctx.part_element_world_aabb(head, elem="flash_lens")
    rest_center = _aabb_center(rest_lens)
    rotated_center = _aabb_center(rotated_lens)
    ctx.check(
        "neck swivel turns the head sideways",
        rest_center is not None
        and rotated_center is not None
        and abs(rotated_center[1]) > 0.045
        and abs(rotated_center[0]) < abs(rest_center[0]) * 0.35,
        details=f"rest_center={rest_center}, rotated_center={rotated_center}",
    )

    return ctx.report()


object_model = build_object_model()
