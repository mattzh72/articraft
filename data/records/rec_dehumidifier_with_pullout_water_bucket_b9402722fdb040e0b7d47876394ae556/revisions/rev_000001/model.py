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

BODY_W = 0.360
BODY_D = 0.240
BODY_H = 0.575

TANK_W = 0.290
TANK_D = 0.172
TANK_H = 0.198
TANK_WALL = 0.003
TANK_TRAVEL = 0.098
TANK_Z0 = 0.008

TANK_BAY_W = 0.298
TANK_BAY_D = 0.182
TANK_BAY_H = 0.208
TANK_BAY_Z0 = 0.008

CONTROL_W = 0.252
CONTROL_D = 0.013
CONTROL_H = 0.076
CONTROL_Z0 = 0.430

OUTLET_W = 0.220
OUTLET_D = 0.090
OUTLET_DEPTH = 0.011
OUTLET_Y = -0.010

BUTTON_W = 0.028
BUTTON_D = 0.010
BUTTON_H = 0.012
BUTTON_PRESS = 0.002
BUTTON_XS = (-0.082, -0.047, -0.012, 0.023)
BUTTON_Y0 = BODY_D / 2.0 - CONTROL_D + 0.0035

DIAL_X = 0.086
DIAL_Y = BODY_D / 2.0 - CONTROL_D + 0.0077
DIAL_Z = CONTROL_Z0 + CONTROL_H / 2.0
DIAL_RADIUS = 0.021
DIAL_LEN = 0.013


def _make_top_housing() -> cq.Workplane:
    lower_shell_height = TANK_BAY_Z0 + TANK_BAY_H
    housing_bottom = lower_shell_height - 0.004
    housing = (
        cq.Workplane("XY")
        .box(BODY_W, BODY_D, BODY_H - housing_bottom, centered=(True, True, False))
        .translate((0.0, 0.0, housing_bottom))
        .edges("|Z")
        .fillet(0.024)
        .edges(">Z")
        .fillet(0.012)
    )
    control_cut = (
        cq.Workplane("XY")
        .box(CONTROL_W, CONTROL_D, CONTROL_H, centered=(True, False, False))
        .edges("|Z")
        .fillet(0.006)
        .translate((0.0, BODY_D / 2.0 - CONTROL_D, CONTROL_Z0))
    )
    outlet_cut = (
        cq.Workplane("XY")
        .box(OUTLET_W, OUTLET_D, OUTLET_DEPTH, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.008)
        .translate((0.0, OUTLET_Y, BODY_H - OUTLET_DEPTH))
    )
    return housing.cut(control_cut).cut(outlet_cut)


def _make_tank_bucket() -> cq.Workplane:
    outer = (
        cq.Workplane("XY")
        .box(TANK_W, TANK_D, TANK_H, centered=(True, True, False))
        .translate((0.0, -TANK_D / 2.0, 0.0))
        .edges("|Z")
        .fillet(0.018)
    )
    inner = (
        cq.Workplane("XY")
        .box(TANK_W - 2.0 * TANK_WALL, TANK_D - 2.0 * TANK_WALL, TANK_H, centered=(True, True, False))
        .translate((0.0, -TANK_D / 2.0, TANK_WALL))
    )
    handle_cut = (
        cq.Workplane("XY")
        .box(0.112, 0.022, 0.040, centered=(True, True, False))
        .translate((0.0, -0.006, TANK_H - 0.068))
    )
    finger_cut = (
        cq.Workplane("YZ")
        .circle(0.022)
        .extrude(0.120, both=True)
        .translate((0.0, 0.001, TANK_H - 0.056))
    )

    return outer.cut(inner).cut(handle_cut).cut(finger_cut)


def _make_louver_flap() -> cq.Workplane:
    flap_width = OUTLET_W - 0.008
    flap_depth = OUTLET_D - 0.008
    flap_thickness = 0.0035
    barrel_radius = 0.004

    panel = cq.Workplane("XY").box(flap_width, flap_depth, flap_thickness, centered=(True, False, False))
    barrel = cq.Workplane("YZ").circle(barrel_radius).extrude(flap_width / 2.0, both=True).translate(
        (0.0, 0.0, barrel_radius)
    )
    front_lip = (
        cq.Workplane("XY")
        .box(flap_width, 0.008, 0.008, centered=(True, False, False))
        .translate((0.0, flap_depth - 0.004, 0.0))
    )
    return panel.union(barrel).union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_front_dehumidifier")

    body_white = model.material("body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    tank_gray = model.material("tank_gray", rgba=(0.74, 0.78, 0.82, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    button_gray = model.material("button_gray", rgba=(0.30, 0.31, 0.34, 1.0))

    lower_shell_height = TANK_BAY_Z0 + TANK_BAY_H
    side_width = (BODY_W - TANK_BAY_W) / 2.0
    lower_side_x = (BODY_W - side_width) / 2.0

    body = model.part("body")
    body.visual(
        Box((BODY_W, BODY_D, TANK_BAY_Z0)),
        origin=Origin(xyz=(0.0, 0.0, TANK_BAY_Z0 / 2.0)),
        material=body_white,
        name="base",
    )
    body.visual(
        Box((side_width, BODY_D, TANK_BAY_H)),
        origin=Origin(xyz=(-lower_side_x, 0.0, TANK_BAY_Z0 + TANK_BAY_H / 2.0)),
        material=body_white,
        name="side_0",
    )
    body.visual(
        Box((side_width, BODY_D, TANK_BAY_H)),
        origin=Origin(xyz=(lower_side_x, 0.0, TANK_BAY_Z0 + TANK_BAY_H / 2.0)),
        material=body_white,
        name="side_1",
    )
    body.visual(
        Box((TANK_BAY_W, BODY_D - TANK_BAY_D, TANK_BAY_H)),
        origin=Origin(
            xyz=(0.0, -(TANK_BAY_D / 2.0), TANK_BAY_Z0 + TANK_BAY_H / 2.0),
        ),
        material=body_white,
        name="back",
    )
    body.visual(
        mesh_from_cadquery(_make_top_housing(), "body_top"),
        material=body_white,
        name="top",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.250),
        origin=Origin(xyz=(-(BODY_W / 2.0 - 0.015), BODY_D / 2.0 - 0.015, 0.125)),
        material=body_white,
        name="front_round_0",
    )
    body.visual(
        Cylinder(radius=0.015, length=0.250),
        origin=Origin(xyz=((BODY_W / 2.0 - 0.015), BODY_D / 2.0 - 0.015, 0.125)),
        material=body_white,
        name="front_round_1",
    )
    body.visual(
        Box((TANK_BAY_W + 0.030, 0.012, 0.028)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - 0.006, lower_shell_height + 0.010)),
        material=body_white,
        name="front_bridge",
    )
    body.visual(
        Box((CONTROL_W, 0.0012, CONTROL_H)),
        origin=Origin(
            xyz=(0.0, BODY_D / 2.0 - CONTROL_D + 0.0006, CONTROL_Z0 + CONTROL_H / 2.0),
        ),
        material=charcoal,
        name="control_strip",
    )
    body.visual(
        Box((OUTLET_W, OUTLET_D, 0.0012)),
        origin=Origin(
            xyz=(0.0, OUTLET_Y, BODY_H - OUTLET_DEPTH + 0.0006),
        ),
        material=charcoal,
        name="outlet_floor",
    )
    body.visual(
        Box((CONTROL_W, 0.004, 0.002)),
        origin=Origin(
            xyz=(0.0, BUTTON_Y0 + BUTTON_D / 2.0, CONTROL_Z0 + CONTROL_H / 2.0 + BUTTON_H / 2.0 + 0.001),
        ),
        material=charcoal,
        name="button_trim_upper",
    )
    body.visual(
        Box((CONTROL_W, 0.004, 0.002)),
        origin=Origin(
            xyz=(0.0, BUTTON_Y0 + BUTTON_D / 2.0, CONTROL_Z0 + CONTROL_H / 2.0 - BUTTON_H / 2.0 - 0.001),
        ),
        material=charcoal,
        name="button_trim_lower",
    )

    tank = model.part("tank")
    tank.visual(
        mesh_from_cadquery(_make_tank_bucket(), "tank_bucket"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=tank_gray,
        name="bucket",
    )

    model.articulation(
        "body_to_tank",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank,
        origin=Origin(xyz=(0.0, BODY_D / 2.0, TANK_Z0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=TANK_TRAVEL, effort=90.0, velocity=0.25),
    )

    louver = model.part("louver")
    louver.visual(
        mesh_from_cadquery(_make_louver_flap(), "louver_flap"),
        material=charcoal,
        name="flap",
    )
    model.articulation(
        "body_to_louver",
        ArticulationType.REVOLUTE,
        parent=body,
        child=louver,
        origin=Origin(xyz=(0.0, OUTLET_Y - OUTLET_D / 2.0 + 0.004, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=4.0, velocity=2.5),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_LEN),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="knob",
    )
    dial.visual(
        Box((0.010, 0.003, 0.004)),
        origin=Origin(xyz=(0.0, DIAL_LEN / 2.0 + 0.0015, DIAL_RADIUS * 0.72)),
        material=body_white,
        name="indicator",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, DIAL_Y, DIAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=8.0),
    )

    for index, button_x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((BUTTON_W, BUTTON_D, BUTTON_H)),
            origin=Origin(xyz=(0.0, BUTTON_D / 2.0, 0.0)),
            material=button_gray,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(button_x, BUTTON_Y0, CONTROL_Z0 + CONTROL_H / 2.0)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(lower=0.0, upper=BUTTON_PRESS, effort=8.0, velocity=0.05),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tank = object_model.get_part("tank")
    tank_slide = object_model.get_articulation("body_to_tank")
    louver = object_model.get_part("louver")
    louver_hinge = object_model.get_articulation("body_to_louver")
    dial = object_model.get_part("dial")
    dial_spin = object_model.get_articulation("body_to_dial")

    limits = tank_slide.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({tank_slide: limits.lower}):
            ctx.expect_within(
                tank,
                body,
                axes="xz",
                margin=0.006,
                name="tank stays centered in the lower bay at rest",
            )
            ctx.expect_overlap(
                tank,
                body,
                axes="y",
                min_overlap=0.16,
                name="closed tank remains deeply inserted in the body",
            )
            rest_pos = ctx.part_world_position(tank)

        with ctx.pose({tank_slide: limits.upper}):
            ctx.expect_within(
                tank,
                body,
                axes="xz",
                margin=0.006,
                name="extended tank stays aligned with the body bay",
            )
            ctx.expect_overlap(
                tank,
                body,
                axes="y",
                min_overlap=0.07,
                name="extended tank still retains insertion",
            )
            extended_pos = ctx.part_world_position(tank)

        ctx.check(
            "tank pulls forward along the body depth axis",
            rest_pos is not None and extended_pos is not None and extended_pos[1] > rest_pos[1] + 0.08,
            details=f"rest={rest_pos}, extended={extended_pos}",
        )

    louver_limits = louver_hinge.motion_limits
    if louver_limits is not None and louver_limits.lower is not None and louver_limits.upper is not None:
        with ctx.pose({louver_hinge: louver_limits.lower}):
            closed_aabb = ctx.part_world_aabb(louver)
        with ctx.pose({louver_hinge: louver_limits.upper}):
            open_aabb = ctx.part_world_aabb(louver)
        ctx.check(
            "top outlet louver lifts upward from the rear hinge",
            closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.05,
            details=f"closed={closed_aabb}, open={open_aabb}",
        )

    with ctx.pose({dial_spin: 0.0}):
        indicator_rest = ctx.part_element_world_aabb(dial, elem="indicator")
    with ctx.pose({dial_spin: 1.20}):
        indicator_turned = ctx.part_element_world_aabb(dial, elem="indicator")

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        return tuple((lo + hi) / 2.0 for lo, hi in zip(aabb[0], aabb[1]))

    indicator_rest_center = _aabb_center(indicator_rest)
    indicator_turned_center = _aabb_center(indicator_turned)
    ctx.check(
        "mode dial rotates around the control strip normal",
        indicator_rest_center is not None
        and indicator_turned_center is not None
        and abs(indicator_turned_center[0] - indicator_rest_center[0]) > 0.010
        and abs(indicator_turned_center[2] - indicator_rest_center[2]) > 0.004,
        details=f"rest={indicator_rest_center}, turned={indicator_turned_center}",
    )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits
        if button_limits is None or button_limits.lower is None or button_limits.upper is None:
            continue

        with ctx.pose({button_joint: button_limits.lower}):
            ctx.expect_within(
                button,
                body,
                axes="xz",
                margin=0.006,
                name=f"button_{index} stays seated in the control strip opening",
            )
            button_rest = ctx.part_world_position(button)

        with ctx.pose({button_joint: button_limits.upper}):
            ctx.expect_within(
                button,
                body,
                axes="xz",
                margin=0.006,
                name=f"button_{index} remains guided when pressed",
            )
            button_pressed = ctx.part_world_position(button)

        ctx.check(
            f"button_{index} presses inward",
            button_rest is not None and button_pressed is not None and button_pressed[1] < button_rest[1] - 0.0015,
            details=f"rest={button_rest}, pressed={button_pressed}",
        )

    return ctx.report()


object_model = build_object_model()
