from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)

BODY_W = 0.34
BODY_D = 0.22
BODY_H = 0.52
SHELL_WALL = 0.006

TANK_W = 0.278
TANK_D = 0.178
TANK_H = 0.164
TANK_WALL = 0.004
TANK_CENTER_CLOSED = (0.0, 0.021, 0.127)
TANK_TRAVEL = 0.105

OUTLET_W = 0.182
OUTLET_D = 0.074
OUTLET_XYZ = (0.0, -0.020, BODY_H - 0.002)
BUTTON_XS = (-0.056, -0.030, -0.004, 0.022)
CONTROL_Y = BODY_D / 2.0 + 0.004
CONTROL_Z = 0.394


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_W - 0.012, 0.030, 0.270)
    shell = shell.edges("|Z").fillet(0.010)
    shell = shell.edges(">Z").fillet(0.006)
    return shell


def _build_tank_bucket() -> cq.Workplane:
    bucket = cq.Workplane("XY").box(TANK_W, TANK_D, TANK_H).translate((0.0, 0.0, TANK_H / 2.0))
    bucket = bucket.edges("|Z").fillet(0.018)
    bucket = bucket.edges(">Z").fillet(0.010)

    inner = (
        cq.Workplane("XY")
        .box(TANK_W - 2.0 * TANK_WALL, TANK_D - 2.0 * TANK_WALL, TANK_H - TANK_WALL)
        .translate((0.0, 0.0, TANK_H / 2.0 + TANK_WALL / 2.0))
    )
    bucket = bucket.cut(inner)

    handle_cut = cq.Workplane("XY").box(0.118, 0.030, 0.026).translate((0.0, TANK_D / 2.0 - 0.008, TANK_H - 0.048))
    bucket = bucket.cut(handle_cut)

    rear_rib = cq.Workplane("XY").box(0.240, 0.012, 0.020).translate((0.0, -TANK_D / 2.0 + 0.006, 0.030))
    bucket = bucket.union(rear_rib)

    return bucket


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_dehumidifier")

    body_white = model.material("body_white", rgba=(0.90, 0.91, 0.92, 1.0))
    panel_grey = model.material("panel_grey", rgba=(0.28, 0.31, 0.34, 1.0))
    tank_smoke = model.material("tank_smoke", rgba=(0.58, 0.63, 0.68, 0.82))
    louver_dark = model.material("louver_dark", rgba=(0.39, 0.43, 0.46, 1.0))
    foot_dark = model.material("foot_dark", rgba=(0.16, 0.17, 0.18, 1.0))
    button_white = model.material("button_white", rgba=(0.95, 0.96, 0.97, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.72, 0.75, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "dehumidifier_body"),
        origin=Origin(xyz=(0.0, 0.095, 0.385)),
        material=body_white,
        name="front_shell",
    )
    body.visual(
        Box((0.320, 0.208, 0.018)),
        origin=Origin(xyz=(0.0, -0.004, 0.009)),
        material=body_white,
        name="base_pan",
    )
    body.visual(
        Box((0.312, 0.006, 0.496)),
        origin=Origin(xyz=(0.0, -0.107, 0.266)),
        material=body_white,
        name="rear_panel",
    )
    body.visual(
        Box((0.006, 0.206, 0.496)),
        origin=Origin(xyz=(-0.157, -0.005, 0.266)),
        material=body_white,
        name="side_wall_0",
    )
    body.visual(
        Box((0.006, 0.206, 0.496)),
        origin=Origin(xyz=(0.157, -0.005, 0.266)),
        material=body_white,
        name="side_wall_1",
    )
    body.visual(
        Box((0.310, 0.020, 0.040)),
        origin=Origin(xyz=(0.0, 0.100, 0.230)),
        material=body_white,
        name="front_bridge",
    )
    body.visual(
        Box((0.016, 0.020, 0.212)),
        origin=Origin(xyz=(-0.147, 0.100, 0.106)),
        material=body_white,
        name="tank_jamb_0",
    )
    body.visual(
        Box((0.016, 0.020, 0.212)),
        origin=Origin(xyz=(0.147, 0.100, 0.106)),
        material=body_white,
        name="tank_jamb_1",
    )
    body.visual(
        Box((0.312, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.069, 0.517)),
        material=body_white,
        name="top_deck_front",
    )
    body.visual(
        Box((0.312, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -0.086, 0.517)),
        material=body_white,
        name="top_deck_rear",
    )
    body.visual(
        Box((0.064, 0.088, 0.006)),
        origin=Origin(xyz=(-0.124, -0.020, 0.517)),
        material=body_white,
        name="top_rail_0",
    )
    body.visual(
        Box((0.064, 0.088, 0.006)),
        origin=Origin(xyz=(0.124, -0.020, 0.517)),
        material=body_white,
        name="top_rail_1",
    )
    body.visual(
        Box((OUTLET_W - 0.006, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.057, 0.518)),
        material=panel_grey,
        name="hinge_seat",
    )
    for index, z in enumerate((0.300, 0.328, 0.356, 0.384)):
        body.visual(
            Box((0.220, 0.004, 0.010)),
            origin=Origin(xyz=(0.0, -0.111, z)),
            material=panel_grey,
            name=f"rear_vent_{index}",
        )
    body.visual(
        Box((0.010, 0.086, 0.010)),
        origin=Origin(xyz=(-0.160, 0.000, 0.346)),
        material=panel_grey,
        name="side_vent_0",
    )
    body.visual(
        Box((0.010, 0.086, 0.010)),
        origin=Origin(xyz=(0.160, 0.000, 0.346)),
        material=panel_grey,
        name="side_vent_1",
    )
    body.visual(
        Box((0.236, 0.004, 0.068)),
        origin=Origin(xyz=(0.008, BODY_D / 2.0 + 0.002, 0.394)),
        material=panel_grey,
        name="control_strip",
    )
    for suffix, x_sign, y_sign in (
        ("0", -1.0, -1.0),
        ("1", 1.0, -1.0),
        ("2", -1.0, 1.0),
        ("3", 1.0, 1.0),
    ):
        body.visual(
            Box((0.030, 0.012, 0.006)),
            origin=Origin(xyz=(x_sign * 0.115, y_sign * 0.070, 0.003)),
            material=foot_dark,
            name=f"foot_{suffix}",
        )

    tank = model.part("tank")
    tank.visual(
        mesh_from_cadquery(_build_tank_bucket(), "dehumidifier_tank"),
        origin=Origin(xyz=(0.0, 0.0, -TANK_H / 2.0)),
        material=tank_smoke,
        name="bucket_shell",
    )

    louver = model.part("louver")
    louver.visual(
        Box((OUTLET_W - 0.006, OUTLET_D - 0.006, 0.004)),
        origin=Origin(xyz=(0.0, (OUTLET_D - 0.006) / 2.0, 0.002)),
        material=louver_dark,
        name="flap",
    )
    louver.visual(
        Box((OUTLET_W - 0.010, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.004, 0.004)),
        material=louver_dark,
        name="hinge_barrel",
    )

    model.articulation(
        "body_to_tank",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tank,
        origin=Origin(xyz=TANK_CENTER_CLOSED),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.16,
            lower=0.0,
            upper=TANK_TRAVEL,
        ),
    )

    model.articulation(
        "body_to_louver",
        ArticulationType.REVOLUTE,
        parent=body,
        child=louver,
        origin=Origin(xyz=(0.0, -0.055, BODY_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=0.0,
            upper=1.15,
        ),
    )

    for index, x in enumerate(BUTTON_XS):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.016, 0.005, 0.014)),
            origin=Origin(xyz=(0.0, 0.0025, 0.007)),
            material=button_white,
            name="cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x, CONTROL_Y, CONTROL_Z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.003,
            ),
        )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.022,
                0.014,
                body_style="cylindrical",
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=14, depth=0.0007),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "mode_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dial_silver,
        name="dial_cap",
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.082, CONTROL_Y, CONTROL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    tank = object_model.get_part("tank")
    louver = object_model.get_part("louver")
    dial = object_model.get_part("dial")
    tank_slide = object_model.get_articulation("body_to_tank")
    louver_hinge = object_model.get_articulation("body_to_louver")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_overlap(tank, body, axes="xz", min_overlap=0.15, name="tank stays within body width and height")
    with ctx.pose({tank_slide: 0.0, louver_hinge: 0.0}):
        ctx.expect_gap(
            louver,
            body,
            axis="z",
            max_gap=0.006,
            max_penetration=0.0,
            name="closed louver sits on the outlet surround",
        )

    body_aabb = ctx.part_world_aabb(body)
    tank_aabb = ctx.part_world_aabb(tank)
    front_bridge_aabb = ctx.part_element_world_aabb(body, elem="front_bridge")
    ctx.check(
        "tank front aligns with body front at rest",
        front_bridge_aabb is not None
        and tank_aabb is not None
        and abs(float(front_bridge_aabb[1][1]) - float(tank_aabb[1][1])) <= 0.006,
        details=f"front_bridge_aabb={front_bridge_aabb!r}, tank_aabb={tank_aabb!r}, body_aabb={body_aabb!r}",
    )

    rest_tank_pos = ctx.part_world_position(tank)
    if tank_slide.motion_limits is not None and tank_slide.motion_limits.upper is not None:
        with ctx.pose({tank_slide: tank_slide.motion_limits.upper}):
            extended_tank_pos = ctx.part_world_position(tank)
        ctx.check(
            "tank pulls forward along depth axis",
            rest_tank_pos is not None
            and extended_tank_pos is not None
            and extended_tank_pos[1] > rest_tank_pos[1] + 0.08,
            details=f"rest={rest_tank_pos!r}, extended={extended_tank_pos!r}",
        )

    rest_louver_aabb = ctx.part_world_aabb(louver)
    if louver_hinge.motion_limits is not None and louver_hinge.motion_limits.upper is not None:
        with ctx.pose({louver_hinge: louver_hinge.motion_limits.upper}):
            open_louver_aabb = ctx.part_world_aabb(louver)
        ctx.check(
            "louver front edge rises when opened",
            rest_louver_aabb is not None
            and open_louver_aabb is not None
            and open_louver_aabb[1][2] > rest_louver_aabb[1][2] + 0.040,
            details=f"rest={rest_louver_aabb!r}, open={open_louver_aabb!r}",
        )

    ctx.expect_gap(
        dial,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        name="mode dial mounts on the control strip",
    )
    ctx.check(
        "mode dial uses continuous rotation",
        dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=f"limits={dial_joint.motion_limits!r}",
    )

    for index in range(len(BUTTON_XS)):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        ctx.expect_gap(
            button,
            body,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button {index} sits on the control strip",
        )
        rest_button_pos = ctx.part_world_position(button)
        if button_joint.motion_limits is not None and button_joint.motion_limits.upper is not None:
            with ctx.pose({button_joint: button_joint.motion_limits.upper}):
                pressed_button_pos = ctx.part_world_position(button)
            ctx.check(
                f"button {index} presses inward",
                rest_button_pos is not None
                and pressed_button_pos is not None
                and pressed_button_pos[1] < rest_button_pos[1] - 0.002,
                details=f"rest={rest_button_pos!r}, pressed={pressed_button_pos!r}",
            )

    return ctx.report()


object_model = build_object_model()
