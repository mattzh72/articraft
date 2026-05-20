from __future__ import annotations

import math

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
    mesh_from_geometry,
)

BODY_W = 0.300
BODY_D = 0.320
BODY_H = 0.335
BODY_WALL = 0.0045
BODY_CORNER_R = 0.040

DRAWER_OPEN_W = 0.242
DRAWER_OPEN_H = 0.146
DRAWER_OPEN_Z = 0.050
DRAWER_FRONT_Y = -BODY_D / 2.0
DRAWER_CENTER_Z = DRAWER_OPEN_Z + DRAWER_OPEN_H / 2.0

CONTROL_BAND_W = 0.268
CONTROL_BAND_H = 0.092
CONTROL_BAND_T = BODY_WALL
CONTROL_BAND_CENTER_Y = DRAWER_FRONT_Y + CONTROL_BAND_T / 2.0
CONTROL_SURFACE_Y = DRAWER_FRONT_Y
CONTROL_BAND_CENTER_Z = 0.252

CHAMBER_W = 0.234
CHAMBER_D = 0.282
CHAMBER_H = 0.128
CHAMBER_WALL = 0.0025

DRAWER_FRONT_W = 0.252
DRAWER_FRONT_H = 0.154
DRAWER_FRONT_T = 0.010
DRAWER_PAN_W = 0.234
DRAWER_PAN_D = 0.255
DRAWER_PAN_H = 0.112
DRAWER_WALL = 0.003
DRAWER_PAN_CENTER_Z = -0.025
WINDOW_W = 0.078
WINDOW_H = 0.034
WINDOW_Z = 0.010
HANDLE_W = 0.112
HANDLE_D = 0.048
HANDLE_H = 0.026
HANDLE_Y = -0.060
HANDLE_Z = -0.018
HANDLE_ARM_X = 0.034
HANDLE_TOP_Z = HANDLE_Z + HANDLE_H / 2.0

BASKET_W = 0.218
BASKET_D = 0.240
BASKET_H = 0.088
BASKET_WALL = 0.002


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rounded_square_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.14, 0.15, 0.16, 1.0))
    band_finish = model.material("band_finish", rgba=(0.06, 0.06, 0.07, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.11, 0.12, 0.13, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    chamber_finish = model.material("chamber_finish", rgba=(0.72, 0.73, 0.74, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.22, 0.28, 0.30, 0.32))
    control_finish = model.material("control_finish", rgba=(0.09, 0.09, 0.10, 1.0))
    button_finish = model.material("button_finish", rgba=(0.14, 0.15, 0.16, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W - 2.0 * BODY_WALL, BODY_D - 2.0 * BODY_WALL, BODY_WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - BODY_WALL / 2.0)),
        material=body_finish,
        name="top_shell",
    )
    body.visual(
        Box((BODY_W - 2.0 * BODY_CORNER_R, BODY_WALL, BODY_H)),
        origin=Origin(xyz=(0.0, BODY_D / 2.0 - BODY_WALL / 2.0, BODY_H / 2.0)),
        material=band_finish,
        name="back_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_D - 2.0 * BODY_CORNER_R, BODY_H)),
        origin=Origin(xyz=(-(BODY_W / 2.0 - BODY_WALL / 2.0), 0.0, BODY_H / 2.0)),
        material=body_finish,
        name="side_wall",
    )
    body.visual(
        Box((BODY_WALL, BODY_D - 2.0 * BODY_CORNER_R, BODY_H)),
        origin=Origin(xyz=((BODY_W / 2.0 - BODY_WALL / 2.0), 0.0, BODY_H / 2.0)),
        material=body_finish,
        name="side_wall_1",
    )
    body.visual(
        Cylinder(radius=BODY_CORNER_R, length=BODY_H),
        origin=Origin(
            xyz=(-(BODY_W / 2.0 - BODY_CORNER_R), BODY_D / 2.0 - BODY_CORNER_R, BODY_H / 2.0)
        ),
        material=body_finish,
        name="rear_corner",
    )
    body.visual(
        Cylinder(radius=BODY_CORNER_R, length=BODY_H),
        origin=Origin(
            xyz=((BODY_W / 2.0 - BODY_CORNER_R), BODY_D / 2.0 - BODY_CORNER_R, BODY_H / 2.0)
        ),
        material=body_finish,
        name="rear_corner_1",
    )
    body.visual(
        Cylinder(radius=BODY_CORNER_R, length=BODY_H),
        origin=Origin(
            xyz=(-(BODY_W / 2.0 - BODY_CORNER_R), DRAWER_FRONT_Y + BODY_CORNER_R, BODY_H / 2.0)
        ),
        material=body_finish,
        name="front_corner",
    )
    body.visual(
        Cylinder(radius=BODY_CORNER_R, length=BODY_H),
        origin=Origin(
            xyz=((BODY_W / 2.0 - BODY_CORNER_R), DRAWER_FRONT_Y + BODY_CORNER_R, BODY_H / 2.0)
        ),
        material=body_finish,
        name="front_corner_1",
    )
    body.visual(
        Box(((BODY_W - DRAWER_OPEN_W) / 2.0, BODY_WALL, BODY_H)),
        origin=Origin(
            xyz=(
                -(DRAWER_OPEN_W / 2.0 + (BODY_W - DRAWER_OPEN_W) / 4.0),
                DRAWER_FRONT_Y + BODY_WALL / 2.0,
                BODY_H / 2.0,
            )
        ),
        material=body_finish,
        name="front_side",
    )
    body.visual(
        Box(((BODY_W - DRAWER_OPEN_W) / 2.0, BODY_WALL, BODY_H)),
        origin=Origin(
            xyz=(
                DRAWER_OPEN_W / 2.0 + (BODY_W - DRAWER_OPEN_W) / 4.0,
                DRAWER_FRONT_Y + BODY_WALL / 2.0,
                BODY_H / 2.0,
            )
        ),
        material=body_finish,
        name="front_side_1",
    )
    body.visual(
        Box((DRAWER_OPEN_W + 2.0 * BODY_WALL, BODY_WALL, DRAWER_OPEN_Z)),
        origin=Origin(
            xyz=(0.0, DRAWER_FRONT_Y + BODY_WALL / 2.0, DRAWER_OPEN_Z / 2.0)
        ),
        material=body_finish,
        name="front_lower_rail",
    )
    body.visual(
        Box((DRAWER_OPEN_W, BODY_WALL, BODY_H - (DRAWER_OPEN_Z + DRAWER_OPEN_H))),
        origin=Origin(
            xyz=(
                0.0,
                DRAWER_FRONT_Y + BODY_WALL / 2.0,
                DRAWER_OPEN_Z + DRAWER_OPEN_H + (BODY_H - (DRAWER_OPEN_Z + DRAWER_OPEN_H)) / 2.0,
            )
        ),
        material=band_finish,
        name="control_band",
    )

    chamber = model.part("chamber")
    chamber.visual(
        Box((CHAMBER_W, CHAMBER_D, CHAMBER_WALL)),
        origin=Origin(
            xyz=(0.0, BODY_WALL + CHAMBER_D / 2.0, -CHAMBER_H / 2.0 + CHAMBER_WALL / 2.0)
        ),
        material=chamber_finish,
        name="chamber_floor",
    )
    chamber.visual(
        Box((CHAMBER_W, CHAMBER_D, CHAMBER_WALL)),
        origin=Origin(
            xyz=(0.0, BODY_WALL + CHAMBER_D / 2.0, CHAMBER_H / 2.0 - CHAMBER_WALL / 2.0)
        ),
        material=chamber_finish,
        name="chamber_roof",
    )
    chamber.visual(
        Box((CHAMBER_WALL, CHAMBER_D, CHAMBER_H)),
        origin=Origin(
            xyz=(-(CHAMBER_W / 2.0 - CHAMBER_WALL / 2.0), BODY_WALL + CHAMBER_D / 2.0, 0.0)
        ),
        material=chamber_finish,
        name="chamber_side",
    )
    chamber.visual(
        Box((CHAMBER_WALL, CHAMBER_D, CHAMBER_H)),
        origin=Origin(
            xyz=((CHAMBER_W / 2.0 - CHAMBER_WALL / 2.0), BODY_WALL + CHAMBER_D / 2.0, 0.0)
        ),
        material=chamber_finish,
        name="chamber_side_1",
    )
    chamber.visual(
        Box((CHAMBER_W, CHAMBER_WALL, CHAMBER_H)),
        origin=Origin(xyz=(0.0, BODY_WALL + CHAMBER_D - CHAMBER_WALL / 2.0, 0.0)),
        material=chamber_finish,
        name="chamber_back",
    )
    chamber.visual(
        Box((CHAMBER_W + 0.014, CHAMBER_WALL, 0.007)),
        origin=Origin(xyz=(0.0, BODY_WALL + CHAMBER_WALL / 2.0, CHAMBER_H / 2.0 + 0.0035)),
        material=chamber_finish,
        name="flange_top",
    )
    chamber.visual(
        Box((CHAMBER_W + 0.014, CHAMBER_WALL, 0.007)),
        origin=Origin(xyz=(0.0, BODY_WALL + CHAMBER_WALL / 2.0, -CHAMBER_H / 2.0 - 0.0035)),
        material=chamber_finish,
        name="flange_bottom",
    )
    chamber.visual(
        Box((0.007, CHAMBER_WALL, CHAMBER_H + 0.014)),
        origin=Origin(
            xyz=(-(CHAMBER_W / 2.0 + 0.0035), BODY_WALL + CHAMBER_WALL / 2.0, 0.0)
        ),
        material=chamber_finish,
        name="flange_side",
    )
    chamber.visual(
        Box((0.007, CHAMBER_WALL, CHAMBER_H + 0.014)),
        origin=Origin(
            xyz=((CHAMBER_W / 2.0 + 0.0035), BODY_WALL + CHAMBER_WALL / 2.0, 0.0)
        ),
        material=chamber_finish,
        name="flange_side_1",
    )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_FRONT_W, DRAWER_FRONT_T, DRAWER_FRONT_H)),
        origin=Origin(xyz=(0.0, -DRAWER_FRONT_T / 2.0, 0.0)),
        material=drawer_finish,
        name="drawer_front",
    )
    drawer.visual(
        Box((WINDOW_W, 0.0035, WINDOW_H)),
        origin=Origin(xyz=(0.0, -DRAWER_FRONT_T + 0.00175, WINDOW_Z)),
        material=glass_finish,
        name="window_lens",
    )
    drawer.visual(
        Box((DRAWER_PAN_W, DRAWER_PAN_D, DRAWER_WALL)),
        origin=Origin(
            xyz=(0.0, DRAWER_PAN_D / 2.0, DRAWER_PAN_CENTER_Z - DRAWER_PAN_H / 2.0 + DRAWER_WALL / 2.0)
        ),
        material=drawer_finish,
        name="drawer_floor",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_PAN_D, DRAWER_PAN_H)),
        origin=Origin(
            xyz=(-(DRAWER_PAN_W / 2.0 - DRAWER_WALL / 2.0), DRAWER_PAN_D / 2.0, DRAWER_PAN_CENTER_Z)
        ),
        material=drawer_finish,
        name="drawer_side",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_PAN_D, DRAWER_PAN_H)),
        origin=Origin(
            xyz=((DRAWER_PAN_W / 2.0 - DRAWER_WALL / 2.0), DRAWER_PAN_D / 2.0, DRAWER_PAN_CENTER_Z)
        ),
        material=drawer_finish,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((DRAWER_PAN_W, DRAWER_WALL, DRAWER_PAN_H)),
        origin=Origin(xyz=(0.0, DRAWER_PAN_D - DRAWER_WALL / 2.0, DRAWER_PAN_CENTER_Z)),
        material=drawer_finish,
        name="drawer_back",
    )
    drawer.visual(
        Box((DRAWER_PAN_W, DRAWER_WALL, DRAWER_PAN_H * 0.55)),
        origin=Origin(
            xyz=(0.0, DRAWER_WALL / 2.0, DRAWER_PAN_CENTER_Z - DRAWER_PAN_H * 0.225)
        ),
        material=drawer_finish,
        name="drawer_inner_front",
    )
    drawer.visual(
        Box((HANDLE_W, HANDLE_D, HANDLE_H)),
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_Z)),
        material=band_finish,
        name="handle",
    )
    drawer.visual(
        Box((0.018, 0.032, 0.018)),
        origin=Origin(xyz=(HANDLE_ARM_X, -0.024, HANDLE_Z)),
        material=band_finish,
        name="handle_arm",
    )
    drawer.visual(
        Box((0.018, 0.032, 0.018)),
        origin=Origin(xyz=(-HANDLE_ARM_X, -0.024, HANDLE_Z)),
        material=band_finish,
        name="handle_arm_1",
    )

    basket = model.part("basket")
    basket.visual(
        Box((BASKET_W, BASKET_D, BASKET_WALL)),
        origin=Origin(xyz=(0.0, BASKET_D / 2.0, -BASKET_H / 2.0 + BASKET_WALL / 2.0)),
        material=basket_finish,
        name="basket_floor",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_D, BASKET_H)),
        origin=Origin(xyz=(-(BASKET_W / 2.0 - BASKET_WALL / 2.0), BASKET_D / 2.0, 0.0)),
        material=basket_finish,
        name="basket_side",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_D, BASKET_H)),
        origin=Origin(xyz=((BASKET_W / 2.0 - BASKET_WALL / 2.0), BASKET_D / 2.0, 0.0)),
        material=basket_finish,
        name="basket_side_1",
    )
    basket.visual(
        Box((BASKET_W, BASKET_WALL, BASKET_H)),
        origin=Origin(xyz=(0.0, BASKET_D - BASKET_WALL / 2.0, 0.0)),
        material=basket_finish,
        name="basket_back",
    )
    basket.visual(
        Box((BASKET_W, BASKET_WALL, BASKET_H * 0.50)),
        origin=Origin(xyz=(0.0, BASKET_WALL / 2.0, -BASKET_H * 0.25)),
        material=basket_finish,
        name="basket_front",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Box((0.024, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=button_finish,
        name="release_plunger",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.052,
                0.026,
                body_style="skirted",
                top_diameter=0.040,
                skirt=KnobSkirt(0.062, 0.005, flare=0.05),
                grip=KnobGrip(style="fluted", count=20, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
                center=False,
            ),
            "air_fryer_timer_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=control_finish,
        name="timer_dial",
    )

    button_positions = [
        (-0.050, 0.276),
        (0.050, 0.276),
        (-0.050, 0.220),
        (0.050, 0.220),
    ]
    for index, (x_pos, z_pos) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.030, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, -0.006, 0.0)),
            material=button_finish,
            name="button_cap",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button,
            origin=Origin(xyz=(x_pos, CONTROL_SURFACE_Y, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.05,
                lower=0.0,
                upper=0.003,
            ),
        )

    model.articulation(
        "body_to_chamber",
        ArticulationType.FIXED,
        parent=body,
        child=chamber,
        origin=Origin(xyz=(0.0, DRAWER_FRONT_Y, DRAWER_CENTER_Z)),
    )
    model.articulation(
        "body_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(0.0, DRAWER_FRONT_Y, DRAWER_CENTER_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.25,
            lower=0.0,
            upper=0.155,
        ),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(0.0, 0.008, -0.034)),
    )
    model.articulation(
        "drawer_to_plunger",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=plunger,
        origin=Origin(xyz=(0.0, HANDLE_Y, HANDLE_TOP_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )
    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.0, CONTROL_SURFACE_Y, 0.248)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    drawer = object_model.get_part("drawer")
    chamber = object_model.get_part("chamber")
    basket = object_model.get_part("basket")
    plunger = object_model.get_part("plunger")
    drawer_joint = object_model.get_articulation("body_to_drawer")
    plunger_joint = object_model.get_articulation("drawer_to_plunger")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.expect_within(
        basket,
        chamber,
        axes="xz",
        margin=0.014,
        name="basket stays within the cooking chamber footprint",
    )
    ctx.expect_overlap(
        basket,
        chamber,
        axes="y",
        min_overlap=0.180,
        name="basket sits deeply inside the chamber at rest",
    )
    ctx.expect_overlap(
        drawer,
        chamber,
        axes="xz",
        min_overlap=0.120,
        name="drawer stays aligned with the lower opening",
    )

    drawer_limits = drawer_joint.motion_limits
    if drawer_limits is not None and drawer_limits.upper is not None:
        rest_drawer_pos = ctx.part_world_position(drawer)
        with ctx.pose({drawer_joint: drawer_limits.upper}):
            ctx.expect_overlap(
                basket,
                chamber,
                axes="y",
                min_overlap=0.085,
                name="extended basket remains retained in the chamber",
            )
            extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.check(
            "drawer extends outward",
            rest_drawer_pos is not None
            and extended_drawer_pos is not None
            and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.120,
            details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
        )

    for index in range(4):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"body_to_button_{index}")
        button_limits = button_joint.motion_limits
        if button_limits is None or button_limits.upper is None:
            continue
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: button_limits.upper}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.002,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    plunger_limits = plunger_joint.motion_limits
    if plunger_limits is not None and plunger_limits.upper is not None:
        rest_plunger_pos = ctx.part_world_position(plunger)
        with ctx.pose({plunger_joint: plunger_limits.upper}):
            pressed_plunger_pos = ctx.part_world_position(plunger)
        ctx.check(
            "plunger presses downward",
            rest_plunger_pos is not None
            and pressed_plunger_pos is not None
            and pressed_plunger_pos[2] < rest_plunger_pos[2] - 0.0025,
            details=f"rest={rest_plunger_pos}, pressed={pressed_plunger_pos}",
        )

    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulation_type={dial_joint.articulation_type!r}",
    )

    return ctx.report()


object_model = build_object_model()
