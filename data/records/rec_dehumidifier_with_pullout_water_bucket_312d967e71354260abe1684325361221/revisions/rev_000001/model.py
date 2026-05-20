from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_DEPTH = 0.265
BODY_WIDTH = 0.372
BODY_HEIGHT = 0.585
SHELL_WALL = 0.004
BASE_THICKNESS = 0.008
TOP_THICKNESS = 0.018

TANK_DEPTH = 0.220
TANK_WIDTH = 0.290
TANK_HEIGHT = 0.218
TANK_WALL = 0.003
TANK_CENTER_Z = 0.183
TANK_TRAVEL = 0.125

DIAL_CENTER_Z = 0.435
DIAL_RADIUS = 0.042

BUTTON_TRAVEL = 0.003
BUTTON_DEPTH = 0.010
BUTTON_WIDTH = 0.026
BUTTON_HEIGHT = 0.014

HANDLE_PIVOT_X = -0.028
HANDLE_PIVOT_Z = BODY_HEIGHT - 0.009002


def _rounded_x_cutter(depth: float, width: float, height: float, radius: float) -> cq.Workplane:
    del radius
    return cq.Workplane("XY").box(depth, width, height)


def _build_body_shape() -> cq.Workplane:
    corner_r = 0.024
    shell_t = 0.016
    base_t = 0.016
    top_t = 0.018
    tank_open_bottom = 0.066
    tank_open_height = 0.224
    tank_open_width = 0.300
    front_x = BODY_DEPTH / 2.0 - shell_t / 2.0
    back_x = -BODY_DEPTH / 2.0 + shell_t / 2.0
    side_y = BODY_WIDTH / 2.0 - shell_t / 2.0
    panel_depth = BODY_DEPTH - 2.0 * corner_r
    panel_width = BODY_WIDTH - 2.0 * corner_r
    opening_top = tank_open_bottom + tank_open_height
    lower_jamb_width = (panel_width - tank_open_width) / 2.0

    body = cq.Workplane("XY").box(
        panel_depth,
        panel_width,
        base_t,
        centered=(True, True, False),
    )

    top_panel = cq.Workplane("XY").box(
        panel_depth,
        panel_width,
        top_t,
        centered=(True, True, False),
    ).translate((0.0, 0.0, BODY_HEIGHT - top_t))
    body = body.union(top_panel)

    left_side = cq.Workplane("XY").box(
        panel_depth,
        shell_t,
        BODY_HEIGHT - base_t - top_t,
        centered=(True, True, False),
    ).translate((0.0, -side_y, base_t))
    right_side = left_side.translate((0.0, 2.0 * side_y, 0.0))
    body = body.union(left_side).union(right_side)

    back_panel = cq.Workplane("XY").box(
        shell_t,
        panel_width,
        BODY_HEIGHT - base_t,
        centered=(True, True, False),
    ).translate((back_x, 0.0, base_t))
    body = body.union(back_panel)

    upper_front = cq.Workplane("XY").box(
        shell_t,
        panel_width,
        BODY_HEIGHT - opening_top - top_t,
        centered=(True, True, False),
    ).translate((front_x, 0.0, opening_top))
    lower_sill = cq.Workplane("XY").box(
        shell_t,
        panel_width,
        tank_open_bottom - base_t,
        centered=(True, True, False),
    ).translate((front_x, 0.0, base_t))
    left_jamb = cq.Workplane("XY").box(
        shell_t,
        lower_jamb_width,
        tank_open_height,
        centered=(True, True, False),
    ).translate((front_x, -(tank_open_width + lower_jamb_width) / 2.0, tank_open_bottom))
    right_jamb = left_jamb.translate((0.0, tank_open_width + lower_jamb_width, 0.0))
    body = body.union(upper_front).union(lower_sill).union(left_jamb).union(right_jamb)

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            corner = cq.Workplane("XY").cylinder(BODY_HEIGHT, corner_r).translate(
                (
                    x_sign * (BODY_DEPTH / 2.0 - corner_r),
                    y_sign * (BODY_WIDTH / 2.0 - corner_r),
                    BODY_HEIGHT / 2.0,
                )
            )
            body = body.union(corner)

    handle_recess = cq.Workplane("XY").box(
        0.125,
        0.286,
        0.016,
        centered=(True, True, False),
    ).translate((0.010, 0.0, BODY_HEIGHT - 0.016))
    body = body.cut(handle_recess)

    dial_pocket = cq.Workplane("YZ").circle(0.028).extrude(0.018).translate((front_x - 0.009, 0.0, DIAL_CENTER_Z))
    body = body.cut(dial_pocket)

    return body


def _build_tank_shape() -> cq.Workplane:
    outer = cq.Workplane("XY").box(TANK_DEPTH, TANK_WIDTH, TANK_HEIGHT).translate(
        (-TANK_DEPTH / 2.0, 0.0, 0.0)
    )

    inner = cq.Workplane("XY").box(
        TANK_DEPTH - 2.0 * TANK_WALL,
        TANK_WIDTH - 2.0 * TANK_WALL,
        TANK_HEIGHT - TANK_WALL,
    ).translate((-TANK_DEPTH / 2.0, 0.0, TANK_WALL / 2.0))

    tank = outer.cut(inner)

    handle_cut = _rounded_x_cutter(0.030, 0.120, 0.036, 0.010).translate(
        (-0.006, 0.0, TANK_HEIGHT / 2.0 - 0.050)
    )
    tank = tank.cut(handle_cut)

    finger_dish = cq.Workplane("YZ").circle(0.020).extrude(0.012).translate(
        (-0.006, 0.0, TANK_HEIGHT / 2.0 - 0.050)
    )
    tank = tank.cut(finger_dish)

    fascia = cq.Workplane("XY").box(0.010, 0.318, 0.228).translate((0.005, 0.0, 0.0))
    tank = tank.union(fascia)

    return tank


def _build_handle_shape() -> cq.Workplane:
    span = 0.280
    arm_y = span / 2.0 - 0.010
    barrel_radius = 0.007
    barrel_length = 0.014

    handle = cq.Workplane("XY").box(0.016, span - 0.020, 0.014).translate((0.084, 0.0, 0.008))

    for y_pos in (-arm_y, arm_y):
        arm = cq.Workplane("XY").box(0.080, 0.010, 0.010).translate((0.040, y_pos, 0.003))
        barrel = (
            cq.Workplane("XY")
            .cylinder(barrel_length, barrel_radius)
            .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
            .translate((0.0, y_pos, 0.0))
        )
        handle = handle.union(arm).union(barrel)

    return handle


def _build_dial_shape() -> cq.Workplane:
    ring = cq.Workplane("YZ").circle(DIAL_RADIUS).circle(0.024).extrude(0.012)
    bridge = cq.Workplane("YZ").circle(0.024).extrude(0.003)
    hub = cq.Workplane("YZ").circle(0.017).extrude(0.014)
    notch = cq.Workplane("XY").box(0.004, 0.007, 0.002).translate((0.012, 0.0, DIAL_RADIUS - 0.004))
    return ring.union(bridge).union(hub).union(notch)


def _build_button_shape() -> cq.Workplane:
    button = cq.Workplane("XY").box(BUTTON_DEPTH, BUTTON_WIDTH, BUTTON_HEIGHT).translate(
        (BUTTON_DEPTH / 2.0, 0.0, 0.0)
    )
    stem = cq.Workplane("XY").box(0.010, 0.012, 0.006).translate((-0.002, 0.0, 0.0))
    return button.union(stem)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_dehumidifier")

    shell_white = model.material("shell_white", rgba=(0.91, 0.92, 0.93, 1.0))
    tank_smoke = model.material("tank_smoke", rgba=(0.56, 0.62, 0.67, 0.62))
    control_dark = model.material("control_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.55, 0.57, 0.60, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shape(), "dehumidifier_body"),
        material=shell_white,
        name="body_shell",
    )

    tank = model.part("tank")
    tank.visual(
        mesh_from_cadquery(_build_tank_shape(), "dehumidifier_tank"),
        material=tank_smoke,
        name="tank_shell",
    )
    model.articulation(
        "body_to_tank",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank,
        origin=Origin(xyz=(BODY_DEPTH / 2.0, 0.0, TANK_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.25,
            lower=0.0,
            upper=TANK_TRAVEL,
        ),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_build_handle_shape(), "dehumidifier_handle"),
        material=trim_grey,
        name="handle_frame",
    )
    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.25,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_cadquery(_build_dial_shape(), "dehumidifier_dial"),
        material=control_dark,
        name="dial_ring",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(BODY_DEPTH / 2.0, 0.0, DIAL_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=8.0),
    )

    button_mesh = mesh_from_cadquery(_build_button_shape(), "dehumidifier_button")
    button_offsets = (
        (0.000, 0.070),
        (0.058, 0.036),
        (0.058, -0.034),
        (-0.058, -0.034),
        (-0.058, 0.036),
    )
    for index, (y_off, z_off) in enumerate(button_offsets):
        button = model.part(f"button_{index}")
        button.visual(
            button_mesh,
            material=control_dark,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(BODY_DEPTH / 2.0 + 0.007, y_off, DIAL_CENTER_Z + z_off)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.06,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    tank = object_model.get_part("tank")
    handle = object_model.get_part("handle")
    dial = object_model.get_part("dial")
    tank_joint = object_model.get_articulation("body_to_tank")
    handle_joint = object_model.get_articulation("body_to_handle")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.allow_overlap(
        body,
        tank,
        reason="The pullout reservoir is intentionally nested inside the appliance shell cavity at the closed pose.",
    )

    tank_upper = tank_joint.motion_limits.upper if tank_joint.motion_limits is not None else None
    handle_upper = handle_joint.motion_limits.upper if handle_joint.motion_limits is not None else None

    if tank_upper is not None:
        with ctx.pose({tank_joint: 0.0}):
            ctx.expect_overlap(
                tank,
                body,
                axes="x",
                min_overlap=0.19,
                name="closed tank remains deeply inserted",
            )
            ctx.expect_overlap(
                tank,
                body,
                axes="yz",
                min_overlap=0.20,
                name="closed tank stays centered in the lower cavity",
            )
            tank_rest = ctx.part_world_position(tank)

        with ctx.pose({tank_joint: tank_upper}):
            ctx.expect_overlap(
                tank,
                body,
                axes="x",
                min_overlap=0.09,
                name="extended tank still retains insertion",
            )
            ctx.expect_overlap(
                tank,
                body,
                axes="yz",
                min_overlap=0.20,
                name="extended tank stays aligned with the appliance opening",
            )
            tank_extended = ctx.part_world_position(tank)

        ctx.check(
            "tank slides outward from the front",
            tank_rest is not None
            and tank_extended is not None
            and tank_extended[0] > tank_rest[0] + 0.10,
            details=f"rest={tank_rest}, extended={tank_extended}",
        )

    if handle_upper is not None:
        with ctx.pose({handle_joint: 0.0}):
            closed_aabb = ctx.part_world_aabb(handle)
            ctx.expect_overlap(
                handle,
                body,
                axes="xy",
                min_overlap=0.08,
                name="closed handle nests over the top shell",
            )

        with ctx.pose({handle_joint: handle_upper}):
            open_aabb = ctx.part_world_aabb(handle)

        ctx.check(
            "handle lifts above the top shell",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][2] > BODY_HEIGHT + 0.045,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    dial_rest = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi / 2.0}):
        dial_turn = ctx.part_world_position(dial)
    ctx.check(
        "dial rotates in place on the front interface",
        dial_rest is not None
        and dial_turn is not None
        and abs(dial_rest[0] - dial_turn[0]) < 1e-6
        and abs(dial_rest[1] - dial_turn[1]) < 1e-6
        and abs(dial_rest[2] - dial_turn[2]) < 1e-6,
        details=f"rest={dial_rest}, turned={dial_turn}",
    )

    for index in range(5):
        button = object_model.get_part(f"button_{index}")
        joint = object_model.get_articulation(f"body_to_button_{index}")
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: 0.0}):
            button_rest = ctx.part_world_position(button)

        if upper is not None:
            with ctx.pose({joint: upper}):
                button_pressed = ctx.part_world_position(button)
            ctx.check(
                f"button_{index} presses inward",
                button_rest is not None
                and button_pressed is not None
                and button_pressed[0] < button_rest[0] - 0.002,
                details=f"rest={button_rest}, pressed={button_pressed}",
            )

    return ctx.report()


object_model = build_object_model()
