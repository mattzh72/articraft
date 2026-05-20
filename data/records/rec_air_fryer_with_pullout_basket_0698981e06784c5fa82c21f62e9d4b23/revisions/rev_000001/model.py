from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BODY_W = 0.340
BODY_D = 0.302
BODY_H = 0.232
BODY_WALL = 0.006
BODY_FLOOR = 0.022
BODY_ROOF = 0.040
BODY_FILLET = 0.022

OPEN_W = 0.302
OPEN_H = 0.156
OPEN_BOTTOM = 0.018

BASKET_W = 0.292
BASKET_D = 0.266
BASKET_H = 0.120
BASKET_WALL = 0.004
BASKET_FLOOR = 0.006

INNER_W = 0.272
INNER_D = 0.238
INNER_H = 0.096
INNER_WALL = 0.0035
INNER_FLOOR = 0.004

BASKET_TRAVEL = 0.140
BASKET_Z = 0.026

DIAL_X = -0.067
DIAL_Z = 0.191
BUTTON_X = 0.071
BUTTON_TOP_Z = 0.202
BUTTON_BOTTOM_Z = 0.176

BUTTON_TRAVEL = 0.002
PLUNGER_TRAVEL = 0.004


def _open_top_shell(width: float, depth: float, height: float, wall: float, floor: float, *, fillet: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(width, depth, height)
    if fillet > 0.0:
        outer = outer.edges("|Z").fillet(fillet)
        outer = outer.edges(">Z").fillet(fillet * 0.5)
    inner = (
        cq.Workplane("XY")
        .box(width - 2.0 * wall, depth - 2.0 * wall, height - floor + 0.002)
        .translate((0.0, 0.0, (floor + 0.002) * 0.5))
    )
    return outer.cut(inner)


def _body_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_W, BODY_D, BODY_H)
    outer = outer.edges("|Z").fillet(BODY_FILLET)
    outer = outer.edges(">Z").fillet(0.012)

    cavity = (
        cq.Workplane("XY")
        .box(
            BODY_W - 2.0 * BODY_WALL,
            BODY_D - 2.0 * BODY_WALL,
            BODY_H - BODY_FLOOR - BODY_ROOF,
        )
        .translate((0.0, 0.0, (BODY_FLOOR - BODY_ROOF) * 0.5))
    )
    shell = outer.cut(cavity)

    front_cut_depth = BODY_WALL + 0.016
    shell = shell.cut(
        cq.Workplane("XY")
        .box(OPEN_W, front_cut_depth, OPEN_H)
        .translate(
            (
                0.0,
                -BODY_D * 0.5 + front_cut_depth * 0.5 - 0.001,
                OPEN_BOTTOM + OPEN_H * 0.5 - BODY_H * 0.5,
            )
        )
    )

    pocket_depth = 0.018
    shell = shell.cut(
        cq.Workplane("XY")
        .box(0.022, pocket_depth, 0.022)
        .translate(
            (
                DIAL_X,
                -BODY_D * 0.5 + pocket_depth * 0.5 - 0.001,
                DIAL_Z - BODY_H * 0.5,
            )
        )
    )

    for button_z in (BUTTON_TOP_Z, BUTTON_BOTTOM_Z):
        shell = shell.cut(
            cq.Workplane("XY")
            .box(0.020, pocket_depth, 0.016)
            .translate(
                (
                    BUTTON_X,
                    -BODY_D * 0.5 + pocket_depth * 0.5 - 0.001,
                    button_z - BODY_H * 0.5,
                )
            )
        )

    return shell


def _dial_mesh():
    return mesh_from_geometry(
        KnobGeometry(
            0.041,
            0.022,
            body_style="skirted",
            top_diameter=0.032,
            skirt=KnobSkirt(0.046, 0.004, flare=0.06),
            grip=KnobGrip(style="fluted", count=18, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
            center=False,
        ),
        "air_fryer_dial",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_profile_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.18, 1.0))
    panel_finish = model.material("panel_finish", rgba=(0.09, 0.10, 0.11, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.12, 0.13, 0.14, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.10, 0.10, 0.11, 1.0))
    steel_finish = model.material("steel_finish", rgba=(0.68, 0.70, 0.72, 1.0))
    control_finish = model.material("control_finish", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_body_shell(), "body_shell"),
        origin=Origin(xyz=(0.0, BODY_D * 0.5, BODY_H * 0.5)),
        material=body_finish,
        name="body_shell",
    )
    body.visual(
        Box((0.248, 0.004, 0.044)),
        origin=Origin(xyz=(-0.004, -0.002, 0.190)),
        material=panel_finish,
        name="control_strip",
    )
    body.visual(
        Box((0.006, 0.244, 0.012)),
        origin=Origin(xyz=(-0.161, 0.152, 0.056)),
        material=panel_finish,
        name="guide_rail_0",
    )
    body.visual(
        Box((0.006, 0.244, 0.012)),
        origin=Origin(xyz=(0.161, 0.152, 0.056)),
        material=panel_finish,
        name="guide_rail_1",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(
            _open_top_shell(BASKET_W, BASKET_D, BASKET_H, BASKET_WALL, BASKET_FLOOR, fillet=0.010),
            "basket_shell",
        ),
        origin=Origin(xyz=(0.0, BASKET_D * 0.5, BASKET_H * 0.5)),
        material=basket_finish,
        name="basket_shell",
    )
    basket.visual(
        Box((0.018, 0.024, 0.042)),
        origin=Origin(xyz=(-0.082, -0.012, 0.050)),
        material=handle_finish,
        name="handle_post_0",
    )
    basket.visual(
        Box((0.018, 0.024, 0.042)),
        origin=Origin(xyz=(0.082, -0.012, 0.050)),
        material=handle_finish,
        name="handle_post_1",
    )
    basket.visual(
        Box((0.168, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, -0.018, 0.040)),
        material=handle_finish,
        name="handle_grip",
    )
    basket.visual(
        Box((0.018, 0.016, 0.016)),
        origin=Origin(xyz=(-0.027, -0.017, 0.064)),
        material=handle_finish,
        name="guide_side_0",
    )
    basket.visual(
        Box((0.018, 0.016, 0.016)),
        origin=Origin(xyz=(0.027, -0.017, 0.064)),
        material=handle_finish,
        name="guide_side_1",
    )
    basket.visual(
        Box((0.072, 0.006, 0.016)),
        origin=Origin(xyz=(0.0, -0.010, 0.064)),
        material=handle_finish,
        name="guide_rear",
    )
    basket.visual(
        Box((0.072, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.012, 0.053)),
        material=handle_finish,
        name="guide_bridge",
    )
    basket.visual(
        Box((0.012, 0.230, 0.010)),
        origin=Origin(xyz=(-0.152, 0.150, 0.031)),
        material=handle_finish,
        name="runner_0",
    )
    basket.visual(
        Box((0.012, 0.230, 0.010)),
        origin=Origin(xyz=(0.152, 0.150, 0.031)),
        material=handle_finish,
        name="runner_1",
    )

    inner_basket = model.part("inner_basket")
    inner_basket.visual(
        mesh_from_cadquery(
            _open_top_shell(INNER_W, INNER_D, INNER_H, INNER_WALL, INNER_FLOOR, fillet=0.007),
            "inner_basket_shell",
        ),
        origin=Origin(xyz=(0.0, INNER_D * 0.5, INNER_H * 0.5)),
        material=steel_finish,
        name="inner_basket_shell",
    )

    dial = model.part("dial")
    dial.visual(
        _dial_mesh(),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_finish,
        name="dial_cap",
    )
    dial.visual(
        Cylinder(radius=0.0042, length=0.012),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi * 0.5, 0.0, 0.0)),
        material=control_finish,
        name="dial_shaft",
    )

    for button_name in ("mode_button_0", "mode_button_1"):
        button = model.part(button_name)
        button.visual(
            Box((0.028, 0.007, 0.016)),
            origin=Origin(xyz=(0.0, -0.0055, 0.0)),
            material=control_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.016, 0.012, 0.010)),
            origin=Origin(xyz=(0.0, 0.004, 0.0)),
            material=control_finish,
            name="button_stem",
        )

    release_plunger = model.part("release_plunger")
    release_plunger.visual(
        Box((0.032, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=control_finish,
        name="plunger_cap",
    )
    release_plunger.visual(
        Box((0.018, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=control_finish,
        name="plunger_stem",
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.PRISMATIC,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, BASKET_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=BASKET_TRAVEL,
        ),
    )
    model.articulation(
        "basket_to_inner_basket",
        ArticulationType.FIXED,
        parent=basket,
        child=inner_basket,
        origin=Origin(xyz=(0.0, 0.016, 0.006)),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(DIAL_X, 0.0, DIAL_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=10.0),
    )
    model.articulation(
        "body_to_mode_button_0",
        ArticulationType.PRISMATIC,
        parent=body,
        child=model.get_part("mode_button_0"),
        origin=Origin(xyz=(BUTTON_X, 0.0, BUTTON_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "body_to_mode_button_1",
        ArticulationType.PRISMATIC,
        parent=body,
        child=model.get_part("mode_button_1"),
        origin=Origin(xyz=(BUTTON_X, 0.0, BUTTON_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=BUTTON_TRAVEL),
    )
    model.articulation(
        "basket_to_release_plunger",
        ArticulationType.PRISMATIC,
        parent=basket,
        child=release_plunger,
        origin=Origin(xyz=(0.0, -0.017, 0.056)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.05, lower=0.0, upper=PLUNGER_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    inner_basket = object_model.get_part("inner_basket")
    release_plunger = object_model.get_part("release_plunger")
    mode_button_0 = object_model.get_part("mode_button_0")
    mode_button_1 = object_model.get_part("mode_button_1")

    basket_joint = object_model.get_articulation("body_to_basket")
    dial_joint = object_model.get_articulation("body_to_dial")
    button_joint_0 = object_model.get_articulation("body_to_mode_button_0")
    button_joint_1 = object_model.get_articulation("body_to_mode_button_1")
    plunger_joint = object_model.get_articulation("basket_to_release_plunger")

    ctx.check(
        "basket joint is prismatic",
        basket_joint.articulation_type == ArticulationType.PRISMATIC,
        details=str(basket_joint.articulation_type),
    )
    ctx.check(
        "dial joint is continuous",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=str(dial_joint.articulation_type),
    )
    ctx.check(
        "mode button joints are prismatic",
        button_joint_0.articulation_type == ArticulationType.PRISMATIC
        and button_joint_1.articulation_type == ArticulationType.PRISMATIC,
        details=f"{button_joint_0.articulation_type}, {button_joint_1.articulation_type}",
    )

    ctx.expect_within(
        basket,
        body,
        axes="xz",
        margin=0.020,
        name="basket stays centered in the body opening",
    )
    ctx.expect_within(
        inner_basket,
        basket,
        axes="xz",
        margin=0.010,
        name="inner basket nests inside the drawer shell",
    )

    basket_rest = ctx.part_world_position(basket)
    with ctx.pose({basket_joint: BASKET_TRAVEL}):
        ctx.expect_within(
            basket,
            body,
            axes="xz",
            margin=0.020,
            name="extended basket remains aligned with the body cavity",
        )
        ctx.expect_overlap(
            basket,
            body,
            axes="y",
            min_overlap=0.120,
            name="basket retains insertion at full extension",
        )
        basket_extended = ctx.part_world_position(basket)

    ctx.check(
        "basket extends outward",
        basket_rest is not None
        and basket_extended is not None
        and basket_extended[1] < basket_rest[1] - 0.10,
        details=f"rest={basket_rest}, extended={basket_extended}",
    )

    button_0_rest = ctx.part_world_position(mode_button_0)
    with ctx.pose({button_joint_0: BUTTON_TRAVEL}):
        button_0_pressed = ctx.part_world_position(mode_button_0)
    ctx.check(
        "top mode button presses inward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[1] > button_0_rest[1] + 0.0015,
        details=f"rest={button_0_rest}, pressed={button_0_pressed}",
    )

    button_1_rest = ctx.part_world_position(mode_button_1)
    with ctx.pose({button_joint_1: BUTTON_TRAVEL}):
        button_1_pressed = ctx.part_world_position(mode_button_1)
    ctx.check(
        "bottom mode button presses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[1] > button_1_rest[1] + 0.0015,
        details=f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    plunger_rest = ctx.part_world_position(release_plunger)
    with ctx.pose({plunger_joint: PLUNGER_TRAVEL}):
        plunger_pressed = ctx.part_world_position(release_plunger)
    ctx.check(
        "release plunger moves downward",
        plunger_rest is not None
        and plunger_pressed is not None
        and plunger_pressed[2] < plunger_rest[2] - 0.003,
        details=f"rest={plunger_rest}, pressed={plunger_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
