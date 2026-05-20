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

BODY_DEPTH = 0.037
BODY_WIDTH = 0.068
BODY_HEIGHT = 0.082
BACK_PLANE_X = -BODY_DEPTH * 0.5
PANEL_RECESS_DEPTH = 0.0012
BUTTON_DEPTH = 0.0040
BUTTON_TRAVEL = 0.0012
BUTTON_FLOOR_X = BACK_PLANE_X + BUTTON_DEPTH

BUTTON_SPECS = (
    {"name": "top_button_0", "y": -0.013, "z": 0.068, "size": (0.010, 0.005)},
    {"name": "top_button_1", "y": 0.013, "z": 0.068, "size": (0.010, 0.005)},
    {"name": "left_button", "y": -0.018, "z": 0.050, "size": (0.008, 0.012)},
    {"name": "right_button", "y": 0.018, "z": 0.050, "size": (0.008, 0.012)},
    {"name": "center_button", "y": 0.000, "z": 0.029, "size": (0.010, 0.010)},
    {"name": "lower_button", "y": 0.000, "z": 0.014, "size": (0.014, 0.005)},
)


def _make_body_shape() -> cq.Workplane:
    base = (
        cq.Workplane("XY")
        .box(0.037, 0.068, 0.056)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.0, 0.0, 0.028))
    )
    shoulder = (
        cq.Workplane("XY")
        .box(0.034, 0.062, 0.012)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0015, 0.0, 0.052))
    )
    upper = (
        cq.Workplane("XY")
        .box(0.030, 0.058, 0.028)
        .edges("|Z")
        .fillet(0.0025)
        .translate((0.0, 0.0, 0.068))
    )
    body = base.union(shoulder).union(upper)

    panel_recess = (
        cq.Workplane("XY")
        .box(PANEL_RECESS_DEPTH, 0.050, 0.064)
        .translate((BACK_PLANE_X + PANEL_RECESS_DEPTH * 0.5, 0.0, 0.040))
    )
    body = body.cut(panel_recess)

    screen_pocket = (
        cq.Workplane("XY")
        .box(0.0020, 0.028, 0.018)
        .translate((BACK_PLANE_X + 0.0010, 0.0, 0.050))
    )
    body = body.cut(screen_pocket)

    pocket_depth = BUTTON_DEPTH + BUTTON_TRAVEL + 0.0004
    pocket_center_x = BACK_PLANE_X + pocket_depth * 0.5
    for spec in BUTTON_SPECS:
        size_y, size_z = spec["size"]
        pocket = (
            cq.Workplane("XY")
            .box(pocket_depth, size_y, size_z)
            .translate((pocket_center_x, spec["y"], spec["z"]))
        )
        body = body.cut(pocket)

    battery_door = (
        cq.Workplane("XY")
        .box(0.020, 0.0012, 0.034)
        .translate((0.002, BODY_WIDTH * 0.5 - 0.0006, 0.040))
    )
    finger_notch = (
        cq.Workplane("XY")
        .box(0.006, 0.0012, 0.004)
        .translate((0.009, BODY_WIDTH * 0.5 - 0.0006, 0.023))
    )
    return body.cut(battery_door).cut(finger_notch)


def _make_head_shape() -> cq.Workplane:
    shell = (
        cq.Workplane("XY")
        .box(0.044, 0.060, 0.036)
        .edges("|Z")
        .fillet(0.003)
        .translate((0.018, 0.0, 0.010))
    )
    lens_recess = (
        cq.Workplane("XY")
        .box(0.004, 0.050, 0.020)
        .translate((0.038, 0.0, 0.010))
    )
    return shell.cut(lens_recess)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hot_shoe_flash")

    body_plastic = model.material("body_plastic", rgba=(0.14, 0.14, 0.15, 1.0))
    trim_plastic = model.material("trim_plastic", rgba=(0.19, 0.19, 0.20, 1.0))
    button_plastic = model.material("button_plastic", rgba=(0.23, 0.23, 0.24, 1.0))
    screen_glass = model.material("screen_glass", rgba=(0.12, 0.18, 0.22, 0.85))
    shoe_metal = model.material("shoe_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.93, 0.95, 1.0, 0.90))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_make_body_shape(), "flash_body_shell"),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.028, 0.024, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=trim_plastic,
        name="foot_pedestal",
    )
    body.visual(
        Box((0.021, 0.018, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0095)),
        material=shoe_metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.021, 0.002, 0.002)),
        origin=Origin(xyz=(0.0, -0.0065, -0.0080)),
        material=shoe_metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.021, 0.002, 0.002)),
        origin=Origin(xyz=(0.0, 0.0065, -0.0080)),
        material=shoe_metal,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.0010, 0.026, 0.015)),
        origin=Origin(xyz=(BACK_PLANE_X + PANEL_RECESS_DEPTH + 0.0005, 0.0, 0.050)),
        material=screen_glass,
        name="screen",
    )

    neck = model.part("neck")
    neck.visual(
        Cylinder(radius=0.010, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=trim_plastic,
        name="swivel_collar",
    )
    neck.visual(
        Box((0.014, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim_plastic,
        name="neck_stem",
    )
    neck.visual(
        Box((0.012, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, -0.022, 0.008)),
        material=trim_plastic,
        name="yoke_web_0",
    )
    neck.visual(
        Box((0.012, 0.026, 0.004)),
        origin=Origin(xyz=(0.0, 0.022, 0.008)),
        material=trim_plastic,
        name="yoke_web_1",
    )
    neck.visual(
        Box((0.012, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, -0.033, 0.019)),
        material=trim_plastic,
        name="arm_0",
    )
    neck.visual(
        Box((0.012, 0.004, 0.022)),
        origin=Origin(xyz=(0.0, 0.033, 0.019)),
        material=trim_plastic,
        name="arm_1",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_make_head_shape(), "flash_head_shell"),
        material=body_plastic,
        name="head_shell",
    )
    head.visual(
        Box((0.004, 0.050, 0.020)),
        origin=Origin(xyz=(0.038, 0.0, 0.010)),
        material=lens_glass,
        name="flash_lens",
    )

    model.articulation(
        "body_to_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=neck,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-math.pi * 0.5,
            upper=math.pi * 0.5,
        ),
    )
    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=math.radians(-7.0),
            upper=math.radians(95.0),
        ),
    )

    for spec in BUTTON_SPECS:
        size_y, size_z = spec["size"]
        button = model.part(spec["name"])
        button.visual(
            Box((BUTTON_DEPTH, size_y, size_z)),
            origin=Origin(xyz=(-BUTTON_DEPTH * 0.5, 0.0, 0.0)),
            material=button_plastic,
            name="cap",
        )
        button.visual(
            Box((0.0004, max(size_y - 0.0020, 0.0030), max(size_z - 0.0020, 0.0030))),
            origin=Origin(xyz=(0.0002, 0.0, 0.0)),
            material=button_plastic,
            name="guide",
        )
        model.articulation(
            f"body_to_{spec['name']}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BUTTON_FLOOR_X, spec["y"], spec["z"])),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=0.05,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def _aabb_center_y(aabb) -> float | None:
    if aabb is None:
        return None
    return (aabb[0][1] + aabb[1][1]) * 0.5


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("body_to_neck")
    tilt = object_model.get_articulation("neck_to_head")

    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.012,
        name="head clears the body at rest",
    )

    rest_head_aabb = ctx.part_world_aabb(head)
    swivel_upper = swivel.motion_limits.upper if swivel.motion_limits is not None else None
    if swivel_upper is not None:
        with ctx.pose({swivel: swivel_upper}):
            swivel_aabb = ctx.part_world_aabb(head)
        rest_center_y = _aabb_center_y(rest_head_aabb)
        swivel_center_y = _aabb_center_y(swivel_aabb)
        ctx.check(
            "neck swivels the head sideways",
            rest_center_y is not None
            and swivel_center_y is not None
            and abs(swivel_center_y - rest_center_y) > 0.012,
            details=f"rest_center_y={rest_center_y}, swivel_center_y={swivel_center_y}",
        )

    tilt_upper = tilt.motion_limits.upper if tilt.motion_limits is not None else None
    if tilt_upper is not None:
        with ctx.pose({tilt: tilt_upper}):
            tilted_aabb = ctx.part_world_aabb(head)
        ctx.check(
            "head tilts upward",
            rest_head_aabb is not None
            and tilted_aabb is not None
            and tilted_aabb[1][2] > rest_head_aabb[1][2] + 0.010,
            details=f"rest={rest_head_aabb}, tilted={tilted_aabb}",
        )

    for spec in BUTTON_SPECS:
        button = object_model.get_part(spec["name"])
        joint = object_model.get_articulation(f"body_to_{spec['name']}")
        ctx.expect_within(
            button,
            body,
            axes=("y", "z"),
            margin=0.004,
            name=f"{spec['name']} stays on the rear panel footprint",
        )
        rest_pos = ctx.part_world_position(button)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        if upper is None:
            continue
        with ctx.pose({joint: upper}):
            pressed_pos = ctx.part_world_position(button)
            ctx.expect_within(
                button,
                body,
                axes=("y", "z"),
                margin=0.004,
                name=f"{spec['name']} remains aligned when pressed",
            )
        ctx.check(
            f"{spec['name']} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[0] > rest_pos[0] + 0.001,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
