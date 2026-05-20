from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
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


EPS = 0.001

BODY_DEPTH = 0.318
BODY_WIDTH = 0.286
BODY_HEIGHT = 0.274
BODY_WALL = 0.006
BODY_FRONT_X = BODY_DEPTH * 0.5

APERTURE_DEPTH = 0.088
APERTURE_WIDTH = 0.238
APERTURE_HEIGHT = 0.148
APERTURE_BOTTOM = 0.042

CAVITY_DEPTH = 0.230
CAVITY_WIDTH = 0.224
CAVITY_HEIGHT = 0.184
CAVITY_WALL = 0.0025
CAVITY_FRONT_X = BODY_FRONT_X - 0.022
CAVITY_BOTTOM = 0.050

DRAWER_DEPTH = 0.214
DRAWER_WIDTH = 0.214
DRAWER_HEIGHT = 0.103
DRAWER_WALL = 0.003
DRAWER_TRAVEL = 0.180
DRAWER_FLOOR_OFFSET = 0.010

BASKET_DEPTH = 0.192
BASKET_WIDTH = 0.191
BASKET_HEIGHT = 0.081
BASKET_WALL = 0.0022
BASKET_FRONT_OFFSET = 0.011
BASKET_FLOOR_OFFSET = 0.007

CAP_DEPTH = 0.186
CAP_WIDTH = 0.206
CAP_HEIGHT = 0.040
CAP_X = -0.008

TEMP_DIAL_Y = -0.056
TIMER_DIAL_Y = 0.056
DIAL_X = 0.018
DIAL_BOSS_HEIGHT = 0.004
POWER_BUTTON_X = 0.062


def _box_from_front(depth: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(depth, width, height, centered=(False, True, False)).translate((-depth, 0.0, 0.0))


def _open_front_shell(
    depth: float,
    width: float,
    height: float,
    wall: float,
    *,
    corner_radius: float,
) -> cq.Workplane:
    outer = _box_from_front(depth, width, height)
    if corner_radius > 0.0:
        outer = outer.edges("|Z").fillet(corner_radius)
    inner = (
        cq.Workplane("XY")
        .box(depth - wall + EPS, width - 2.0 * wall, height - 2.0 * wall, centered=(False, True, False))
        .translate((-depth + wall, 0.0, wall))
    )
    return outer.cut(inner)


def _open_top_shell(
    depth: float,
    width: float,
    height: float,
    wall: float,
    *,
    corner_radius: float,
) -> cq.Workplane:
    outer = _box_from_front(depth, width, height)
    if corner_radius > 0.0:
        outer = outer.edges("|Z").fillet(corner_radius)
    inner = (
        cq.Workplane("XY")
        .box(depth - 2.0 * wall, width - 2.0 * wall, height - wall + EPS, centered=(False, True, False))
        .translate((-depth + wall, 0.0, wall))
    )
    return outer.cut(inner)


def _build_body_shell() -> cq.Workplane:
    shell = cq.Workplane("XY").box(BODY_DEPTH, BODY_WIDTH, BODY_HEIGHT, centered=(True, True, False))
    shell = shell.edges("|Z").fillet(0.038)
    shell = shell.faces(">Z").edges().fillet(0.019)
    inner = (
        cq.Workplane("XY")
        .box(
            BODY_DEPTH - 2.0 * BODY_WALL,
            BODY_WIDTH - 2.0 * BODY_WALL,
            BODY_HEIGHT - 2.0 * BODY_WALL,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BODY_WALL))
    )
    aperture = (
        cq.Workplane("XY")
        .box(APERTURE_DEPTH, APERTURE_WIDTH, APERTURE_HEIGHT, centered=(True, True, False))
        .translate((BODY_FRONT_X - APERTURE_DEPTH * 0.5 + 0.003, 0.0, APERTURE_BOTTOM))
        .edges("|Z")
        .fillet(0.020)
    )
    return shell.cut(inner).cut(aperture).val()


def _build_cavity() -> cq.Workplane:
    cavity = _open_front_shell(CAVITY_DEPTH, CAVITY_WIDTH, CAVITY_HEIGHT, CAVITY_WALL, corner_radius=0.010)
    side_tab_width = BODY_WIDTH * 0.5 - CAVITY_WIDTH * 0.5 - BODY_WALL
    cavity = cavity.union(
        cq.Workplane("XY")
        .box(0.050, side_tab_width, 0.032, centered=(False, False, False))
        .translate((-0.050, CAVITY_WIDTH * 0.5, 0.114))
    )
    cavity = cavity.union(
        cq.Workplane("XY")
        .box(0.050, side_tab_width, 0.032, centered=(False, False, False))
        .translate((-0.050, -BODY_WIDTH * 0.5 + BODY_WALL, 0.114))
    )
    cavity = cavity.union(
        cq.Workplane("XY")
        .box(0.045, 0.096, BODY_HEIGHT - BODY_WALL - CAVITY_BOTTOM - CAVITY_HEIGHT, centered=(False, True, False))
        .translate((-0.045, 0.0, CAVITY_HEIGHT))
    )
    return cavity.val()


def _build_top_cap() -> cq.Workplane:
    cap = cq.Workplane("XY").box(CAP_DEPTH, CAP_WIDTH, CAP_HEIGHT, centered=(True, True, False))
    cap = cap.edges("|Z").fillet(0.022)
    cap = cap.faces(">Z").edges().fillet(0.012)

    for y in (TEMP_DIAL_Y, TIMER_DIAL_Y):
        cap = cap.union(
            cq.Workplane("XY")
            .circle(0.030)
            .extrude(DIAL_BOSS_HEIGHT)
            .translate((DIAL_X, y, CAP_HEIGHT))
        )

    power_pocket = (
        cq.Workplane("XY")
        .circle(0.012)
        .extrude(0.006 + EPS)
        .translate((POWER_BUTTON_X, 0.0, CAP_HEIGHT - 0.006))
    )
    return cap.cut(power_pocket).val()


def _build_drawer() -> cq.Workplane:
    pan = _open_top_shell(DRAWER_DEPTH, DRAWER_WIDTH, DRAWER_HEIGHT, DRAWER_WALL, corner_radius=0.012)

    fascia = (
        cq.Workplane("XY")
        .box(0.022, 0.232, 0.128, centered=(False, True, False))
        .translate((0.0, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.010)
    )
    handle = (
        cq.Workplane("XY")
        .box(0.060, 0.114, 0.036, centered=(False, True, False))
        .translate((0.018, 0.0, 0.032))
        .edges("|Z")
        .fillet(0.008)
    )

    drawer = pan.union(fascia).union(handle)

    grip_recess = (
        cq.Workplane("XY")
        .box(0.040, 0.080, 0.018, centered=(False, True, False))
        .translate((0.034, 0.0, 0.032))
    )
    release_pocket = (
        cq.Workplane("XY")
        .box(0.020, 0.046, 0.012 + EPS, centered=(False, True, False))
        .translate((0.040, 0.0, 0.058))
    )
    return drawer.cut(grip_recess).cut(release_pocket).val()


def _build_basket() -> cq.Workplane:
    basket = _open_top_shell(BASKET_DEPTH, BASKET_WIDTH, BASKET_HEIGHT, BASKET_WALL, corner_radius=0.009)
    basket = basket.translate((-BASKET_FRONT_OFFSET, 0.0, BASKET_FLOOR_OFFSET))

    bottom_holes: list[tuple[float, float]] = []
    x_positions = (-0.170, -0.145, -0.120, -0.095, -0.070, -0.045)
    y_positions = (-0.060, -0.036, -0.012, 0.012, 0.036, 0.060)
    for x in x_positions:
        for y in y_positions:
            bottom_holes.append((x, y))

    basket = basket.cut(
        cq.Workplane("XY")
        .workplane(offset=BASKET_FLOOR_OFFSET)
        .pushPoints(bottom_holes)
        .circle(0.0032)
        .extrude(0.010)
    )

    for slot_x in (-0.172, -0.144, -0.116, -0.088, -0.060):
        for slot_z in (0.031, 0.048):
            basket = basket.cut(
                cq.Workplane("XY")
                .box(0.014, BASKET_WIDTH + 0.010, 0.005, centered=(False, True, False))
                .translate((slot_x, 0.0, BASKET_FLOOR_OFFSET + slot_z))
            )

    rear_slots = []
    for y in (-0.054, -0.027, 0.0, 0.027, 0.054):
        rear_slots.append((y, 0.022))
        rear_slots.append((y, 0.046))
    for y, z in rear_slots:
        basket = basket.cut(
            cq.Workplane("XY")
            .box(0.006, 0.016, 0.007, centered=(False, True, False))
            .translate((-BASKET_DEPTH - BASKET_FRONT_OFFSET + 0.001, y, BASKET_FLOOR_OFFSET + z))
        )

    foot_height = BASKET_FLOOR_OFFSET - DRAWER_WALL
    for foot_x in (-0.182, -0.036):
        for foot_y in (-0.072, 0.058):
            basket = basket.union(
                cq.Workplane("XY")
                .box(0.014, 0.014, foot_height, centered=(False, False, False))
                .translate((foot_x, foot_y, DRAWER_WALL))
            )

    return basket.val()


def _build_power_button() -> cq.Workplane:
    button = cq.Workplane("XY").circle(0.0105).extrude(0.010)
    return button.faces(">Z").edges().fillet(0.002).val()


def _build_release_button() -> cq.Workplane:
    button = cq.Workplane("XY").box(0.018, 0.042, 0.009, centered=(True, True, False))
    return button.faces(">Z").edges().fillet(0.0018).val()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="manual_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.16, 0.17, 0.19, 1.0))
    cavity_metal = model.material("cavity_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    cap_finish = model.material("cap_finish", rgba=(0.20, 0.21, 0.23, 1.0))
    drawer_finish = model.material("drawer_finish", rgba=(0.15, 0.16, 0.18, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.48, 0.50, 0.53, 1.0))
    dial_finish = model.material("dial_finish", rgba=(0.09, 0.10, 0.11, 1.0))
    button_finish = model.material("button_finish", rgba=(0.84, 0.13, 0.08, 1.0))
    power_finish = model.material("power_finish", rgba=(0.80, 0.12, 0.08, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "air_fryer_body_shell"),
        material=body_finish,
        name="body_shell",
    )

    body.visual(
        mesh_from_cadquery(_build_cavity(), "air_fryer_cavity_shell"),
        origin=Origin(xyz=(CAVITY_FRONT_X, 0.0, CAVITY_BOTTOM)),
        material=cavity_metal,
        name="cavity_shell",
    )

    top_cap = model.part("top_cap")
    top_cap.visual(
        mesh_from_cadquery(_build_top_cap(), "air_fryer_top_cap"),
        material=cap_finish,
        name="cap_shell",
    )
    model.articulation(
        "body_to_top_cap",
        ArticulationType.FIXED,
        parent=body,
        child=top_cap,
        origin=Origin(xyz=(CAP_X, 0.0, BODY_HEIGHT)),
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_build_drawer(), "air_fryer_drawer"),
        material=drawer_finish,
        name="drawer_shell",
    )
    model.articulation(
        "cavity_to_drawer",
        ArticulationType.PRISMATIC,
        parent=body,
        child=drawer,
        origin=Origin(xyz=(CAVITY_FRONT_X + 0.001, 0.0, CAVITY_BOTTOM + DRAWER_FLOOR_OFFSET)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=0.45, lower=0.0, upper=DRAWER_TRAVEL),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_build_basket(), "air_fryer_basket"),
        material=basket_finish,
        name="basket_shell",
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(),
    )

    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.046,
            0.024,
            body_style="skirted",
            top_diameter=0.036,
            skirt=KnobSkirt(0.058, 0.006, flare=0.05),
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=0.0),
            center=False,
        ),
        "air_fryer_dial",
    )

    temp_dial = model.part("temp_dial")
    temp_dial.visual(dial_mesh, material=dial_finish, name="dial_shell")
    model.articulation(
        "top_cap_to_temp_dial",
        ArticulationType.CONTINUOUS,
        parent=top_cap,
        child=temp_dial,
        origin=Origin(xyz=(DIAL_X, TEMP_DIAL_Y, CAP_HEIGHT + DIAL_BOSS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    timer_dial = model.part("timer_dial")
    timer_dial.visual(dial_mesh, material=dial_finish, name="dial_shell")
    model.articulation(
        "top_cap_to_timer_dial",
        ArticulationType.CONTINUOUS,
        parent=top_cap,
        child=timer_dial,
        origin=Origin(xyz=(DIAL_X, TIMER_DIAL_Y, CAP_HEIGHT + DIAL_BOSS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0),
    )

    power_button = model.part("power_button")
    power_button.visual(
        mesh_from_cadquery(_build_power_button(), "air_fryer_power_button"),
        material=power_finish,
        name="button_cap",
    )
    model.articulation(
        "top_cap_to_power_button",
        ArticulationType.PRISMATIC,
        parent=top_cap,
        child=power_button,
        origin=Origin(xyz=(POWER_BUTTON_X, 0.0, CAP_HEIGHT - 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.10, lower=0.0, upper=0.004),
    )

    release_button = model.part("release_button")
    release_button.visual(
        mesh_from_cadquery(_build_release_button(), "air_fryer_release_button"),
        material=button_finish,
        name="button_cap",
    )
    model.articulation(
        "drawer_to_release_button",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=release_button,
        origin=Origin(xyz=(0.050, 0.0, 0.058)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.003),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    top_cap = object_model.get_part("top_cap")
    power_button = object_model.get_part("power_button")
    release_button = object_model.get_part("release_button")

    drawer_slide = object_model.get_articulation("cavity_to_drawer")
    power_joint = object_model.get_articulation("top_cap_to_power_button")
    release_joint = object_model.get_articulation("drawer_to_release_button")

    ctx.allow_overlap(
        drawer,
        basket,
        elem_a="drawer_shell",
        elem_b="basket_shell",
        reason="The perforated basket is intentionally retained inside the drawer pan, and this nested fit is represented by simplified mesh solids.",
    )
    ctx.allow_overlap(
        drawer,
        release_button,
        elem_a="drawer_shell",
        elem_b="button_cap",
        reason="The safety release button intentionally travels down into the handle pocket, which is represented by a simplified solid handle shell.",
    )
    ctx.allow_overlap(
        body,
        drawer,
        elem_a="body_shell",
        elem_b="drawer_shell",
        reason="The pullout drawer intentionally nests into the fryer body opening, and the current shell exports are represented as simplified solid meshes for that retained fit.",
    )

    ctx.expect_origin_distance(
        drawer,
        body,
        axes="y",
        max_dist=0.001,
        name="drawer remains laterally centered in the cavity",
    )
    ctx.expect_origin_gap(
        drawer,
        body,
        axis="z",
        min_gap=CAVITY_BOTTOM + DRAWER_FLOOR_OFFSET - 0.001,
        max_gap=CAVITY_BOTTOM + DRAWER_FLOOR_OFFSET + 0.001,
        name="drawer rides at the cavity entry height",
    )
    ctx.expect_overlap(
        drawer,
        body,
        axes="x",
        elem_a="drawer_shell",
        elem_b="cavity_shell",
        min_overlap=0.120,
        name="drawer remains substantially inserted at rest",
    )
    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        inner_elem="basket_shell",
        outer_elem="drawer_shell",
        margin=0.006,
        name="basket stays nested inside the drawer shell",
    )
    ctx.expect_overlap(
        basket,
        drawer,
        axes="x",
        elem_a="basket_shell",
        elem_b="drawer_shell",
        min_overlap=0.160,
        name="basket has deep retained insertion in the drawer shell",
    )
    drawer_rest = ctx.part_world_position(drawer)
    power_rest = ctx.part_world_position(power_button)
    release_rest = ctx.part_world_position(release_button)

    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        ctx.expect_origin_distance(
            drawer,
            body,
            axes="y",
            max_dist=0.001,
            name="drawer stays laterally centered when fully extended",
        )
        ctx.expect_origin_gap(
            drawer,
            body,
            axis="z",
            min_gap=CAVITY_BOTTOM + DRAWER_FLOOR_OFFSET - 0.001,
            max_gap=CAVITY_BOTTOM + DRAWER_FLOOR_OFFSET + 0.001,
            name="drawer keeps its cavity entry height when extended",
        )
        ctx.expect_overlap(
            drawer,
            body,
            axes="x",
            elem_a="drawer_shell",
            elem_b="cavity_shell",
            min_overlap=0.030,
            name="drawer keeps retained insertion at full extension",
        )
        drawer_extended = ctx.part_world_position(drawer)

    with ctx.pose({power_joint: 0.004}):
        power_pressed = ctx.part_world_position(power_button)

    with ctx.pose({release_joint: 0.003}):
        release_pressed = ctx.part_world_position(release_button)

    ctx.check(
        "drawer slides forward out of the body",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[0] > drawer_rest[0] + 0.16,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )
    ctx.check(
        "power button presses downward",
        power_rest is not None
        and power_pressed is not None
        and power_pressed[2] < power_rest[2] - 0.003,
        details=f"rest={power_rest}, pressed={power_pressed}",
    )
    ctx.check(
        "release button presses into the handle",
        release_rest is not None
        and release_pressed is not None
        and release_pressed[2] < release_rest[2] - 0.002,
        details=f"rest={release_rest}, pressed={release_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
