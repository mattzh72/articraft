from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    PerforatedPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


WIDTH = 0.52
DEPTH = 0.42
HEIGHT = 0.335
WALL = 0.012
DIVIDER_W = 0.010
CONTROL_W = 0.115
BOTTOM_LIP = 0.055
OPENING_H = 0.225
TOP_TRIM = HEIGHT - BOTTOM_LIP - OPENING_H
JAMB_W = 0.018

CAVITY_LEFT_INNER = -WIDTH / 2.0 + WALL
DIVIDER_CENTER_Y = WIDTH / 2.0 - WALL - CONTROL_W - DIVIDER_W / 2.0
CAVITY_RIGHT_INNER = DIVIDER_CENTER_Y - DIVIDER_W / 2.0
CAVITY_CENTER_Y = (CAVITY_LEFT_INNER + CAVITY_RIGHT_INNER) / 2.0
CAVITY_WIDTH = CAVITY_RIGHT_INNER - CAVITY_LEFT_INNER
FRONT_OPENING_W = CAVITY_WIDTH - 2.0 * JAMB_W
CONTROL_CENTER_Y = DIVIDER_CENTER_Y + DIVIDER_W / 2.0 + CONTROL_W / 2.0

DOOR_T = 0.022
DOOR_Y = FRONT_OPENING_W + 0.046
DOOR_Z = OPENING_H + 0.035
DOOR_GAP = 0.0
DOOR_X = -(DOOR_T / 2.0 + DOOR_GAP)
DOOR_SIDE_FRAME = 0.026
DOOR_TOP_FRAME = 0.024
DOOR_BOTTOM_FRAME = 0.030
HINGE_Z = BOTTOM_LIP - 0.006

RUNNER_X0 = 0.050
RUNNER_L = 0.270
RUNNER_W = 0.012
RUNNER_H = 0.010
RUNNER_Z = 0.088

BASKET_DEPTH = 0.285
BASKET_W = 0.347
BASKET_WALL = 0.004
BASKET_H = 0.055
BASKET_FLOOR = 0.003
BASKET_FRONT_X = 0.036
BASKET_Z = 0.082
BASKET_TRAVEL = 0.180

KNOB_DIAMETER = 0.040
KNOB_HEIGHT = 0.024
KNOB_ZS = (0.254, 0.191, 0.128)


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((float(mins[i]) + float(maxs[i])) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_toaster_oven")

    model.material("steel", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("charcoal", rgba=(0.14, 0.15, 0.16, 1.0))
    model.material("glass", rgba=(0.22, 0.28, 0.33, 0.35))
    model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    model.material("rack_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("basket_dark", rgba=(0.19, 0.20, 0.22, 1.0))
    model.material("foot_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    model.material("element_steel", rgba=(0.46, 0.39, 0.34, 1.0))

    body = model.part("body")

    _add_box(
        body,
        (DEPTH, WIDTH, WALL),
        (DEPTH / 2.0, 0.0, WALL / 2.0),
        "steel",
        "bottom_panel",
    )
    _add_box(
        body,
        (DEPTH, WIDTH, WALL),
        (DEPTH / 2.0, 0.0, HEIGHT - WALL / 2.0),
        "steel",
        "top_panel",
    )
    _add_box(
        body,
        (DEPTH, WALL, HEIGHT),
        (DEPTH / 2.0, -WIDTH / 2.0 + WALL / 2.0, HEIGHT / 2.0),
        "steel",
        "left_shell",
    )
    _add_box(
        body,
        (DEPTH, WALL, HEIGHT),
        (DEPTH / 2.0, WIDTH / 2.0 - WALL / 2.0, HEIGHT / 2.0),
        "steel",
        "right_shell",
    )
    _add_box(
        body,
        (WALL, WIDTH - 2.0 * WALL, HEIGHT - 2.0 * WALL),
        (DEPTH - WALL / 2.0, 0.0, HEIGHT / 2.0),
        "steel",
        "back_panel",
    )
    _add_box(
        body,
        (DEPTH, DIVIDER_W, HEIGHT - 2.0 * WALL),
        (DEPTH / 2.0, DIVIDER_CENTER_Y, HEIGHT / 2.0),
        "steel",
        "divider_wall",
    )
    _add_box(
        body,
        (WALL, CONTROL_W, HEIGHT - 2.0 * WALL),
        (WALL / 2.0, CONTROL_CENTER_Y, HEIGHT / 2.0),
        "charcoal",
        "control_panel",
    )
    _add_box(
        body,
        (WALL, CAVITY_WIDTH, TOP_TRIM),
        (WALL / 2.0, CAVITY_CENTER_Y, HEIGHT - TOP_TRIM / 2.0),
        "charcoal",
        "top_bezel",
    )
    _add_box(
        body,
        (WALL, CAVITY_WIDTH, BOTTOM_LIP),
        (WALL / 2.0, CAVITY_CENTER_Y, BOTTOM_LIP / 2.0),
        "charcoal",
        "bottom_bezel",
    )
    _add_box(
        body,
        (WALL, JAMB_W, OPENING_H),
        (WALL / 2.0, CAVITY_LEFT_INNER + JAMB_W / 2.0, BOTTOM_LIP + OPENING_H / 2.0),
        "charcoal",
        "left_jamb",
    )
    _add_box(
        body,
        (WALL, JAMB_W, OPENING_H),
        (WALL / 2.0, CAVITY_RIGHT_INNER - JAMB_W / 2.0, BOTTOM_LIP + OPENING_H / 2.0),
        "charcoal",
        "right_jamb",
    )

    for idx, y_sign in enumerate((-1.0, 1.0)):
        y = CAVITY_CENTER_Y + y_sign * (CAVITY_WIDTH / 2.0 - RUNNER_W / 2.0)
        _add_box(
            body,
            (RUNNER_L, RUNNER_W, RUNNER_H),
            (RUNNER_X0 + RUNNER_L / 2.0, y, RUNNER_Z),
            "rack_steel",
            f"runner_{idx}",
        )

    element_y = FRONT_OPENING_W - 0.020
    heater_left_end = CAVITY_CENTER_Y - element_y / 2.0
    heater_right_end = CAVITY_CENTER_Y + element_y / 2.0
    left_support_y = (CAVITY_LEFT_INNER + heater_left_end) / 2.0
    right_support_y = ((DIVIDER_CENTER_Y - DIVIDER_W / 2.0) + heater_right_end) / 2.0
    support_span_y = heater_left_end - CAVITY_LEFT_INNER
    for idx, z in enumerate((0.098, 0.120, 0.216, 0.238)):
        _add_cylinder(
            body,
            radius=0.003,
            length=element_y,
            xyz=(0.175, CAVITY_CENTER_Y, z),
            material="element_steel",
            name=f"heater_{idx}",
            rpy=(pi / 2.0, 0.0, 0.0),
        )
        _add_box(
            body,
            (0.010, support_span_y, 0.006),
            (0.175, left_support_y, z),
            "element_steel",
            f"heater_left_mount_{idx}",
        )
        _add_box(
            body,
            (0.010, support_span_y, 0.006),
            (0.175, right_support_y, z),
            "element_steel",
            f"heater_right_mount_{idx}",
        )

    foot_positions = (
        (0.060, -0.200),
        (0.060, 0.200),
        (0.360, -0.200),
        (0.360, 0.200),
    )
    for idx, (x, y) in enumerate(foot_positions):
        _add_cylinder(
            body,
            radius=0.012,
            length=0.006,
            xyz=(x, y, -0.003),
            material="foot_rubber",
            name=f"foot_{idx}",
        )

    door = model.part("door")
    _add_box(
        door,
        (DOOR_T, DOOR_Y, DOOR_TOP_FRAME),
        (DOOR_X, 0.0, DOOR_Z - DOOR_TOP_FRAME / 2.0),
        "charcoal",
        "top_frame",
    )
    _add_box(
        door,
        (DOOR_T, DOOR_Y, DOOR_BOTTOM_FRAME),
        (DOOR_X, 0.0, DOOR_BOTTOM_FRAME / 2.0),
        "charcoal",
        "bottom_frame",
    )
    _add_box(
        door,
        (DOOR_T, DOOR_SIDE_FRAME, DOOR_Z),
        (DOOR_X, -DOOR_Y / 2.0 + DOOR_SIDE_FRAME / 2.0, DOOR_Z / 2.0),
        "charcoal",
        "left_frame",
    )
    _add_box(
        door,
        (DOOR_T, DOOR_SIDE_FRAME, DOOR_Z),
        (DOOR_X, DOOR_Y / 2.0 - DOOR_SIDE_FRAME / 2.0, DOOR_Z / 2.0),
        "charcoal",
        "right_frame",
    )
    glass_height = DOOR_Z - DOOR_TOP_FRAME - DOOR_BOTTOM_FRAME
    door.visual(
        Box((0.006, DOOR_Y - 2.0 * DOOR_SIDE_FRAME, glass_height)),
        origin=Origin(
            xyz=(DOOR_X + 0.001, 0.0, DOOR_BOTTOM_FRAME + glass_height / 2.0),
        ),
        material="glass",
        name="glass_panel",
    )
    handle_x = DOOR_X - 0.015
    handle_z = DOOR_Z - 0.040
    for idx, y in enumerate((-0.072, 0.072)):
        _add_box(
            door,
            (0.018, 0.014, 0.040),
            ((DOOR_X + handle_x) / 2.0, y, DOOR_Z - 0.035),
            "steel",
            f"handle_post_{idx}",
        )
    _add_cylinder(
        door,
        radius=0.008,
        length=0.190,
        xyz=(handle_x, 0.0, handle_z),
        material="steel",
        name="door_handle",
        rpy=(pi / 2.0, 0.0, 0.0),
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_geometry(
            PerforatedPanelGeometry(
                (BASKET_DEPTH, BASKET_W),
                BASKET_FLOOR,
                hole_diameter=0.008,
                pitch=(0.014, 0.014),
                frame=0.008,
                stagger=True,
            ),
            "basket_floor",
        ),
        origin=Origin(
            xyz=(BASKET_DEPTH / 2.0, 0.0, BASKET_FLOOR / 2.0),
        ),
        material="basket_dark",
        name="basket_floor",
    )
    _add_box(
        basket,
        (BASKET_DEPTH, BASKET_WALL, BASKET_H),
        (BASKET_DEPTH / 2.0, -BASKET_W / 2.0 + BASKET_WALL / 2.0, BASKET_H / 2.0),
        "basket_dark",
        "left_wall",
    )
    _add_box(
        basket,
        (BASKET_DEPTH, BASKET_WALL, BASKET_H),
        (BASKET_DEPTH / 2.0, BASKET_W / 2.0 - BASKET_WALL / 2.0, BASKET_H / 2.0),
        "basket_dark",
        "right_wall",
    )
    _add_box(
        basket,
        (BASKET_WALL, BASKET_W, BASKET_H),
        (BASKET_WALL / 2.0, 0.0, BASKET_H / 2.0),
        "basket_dark",
        "front_wall",
    )
    _add_box(
        basket,
        (BASKET_WALL, BASKET_W, BASKET_H),
        (BASKET_DEPTH - BASKET_WALL / 2.0, 0.0, BASKET_H / 2.0),
        "basket_dark",
        "back_wall",
    )
    for idx, y in enumerate((-0.056, 0.056)):
        _add_box(
            basket,
            (0.018, 0.010, 0.026),
            (-0.008, y, 0.032),
            "rack_steel",
            f"handle_leg_{idx}",
        )
    _add_cylinder(
        basket,
        radius=0.006,
        length=0.140,
        xyz=(-0.020, 0.0, 0.032),
        material="rack_steel",
        name="basket_handle",
        rpy=(pi / 2.0, 0.0, 0.0),
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_HEIGHT,
            body_style="skirted",
            top_diameter=0.031,
            skirt=KnobSkirt(0.048, 0.006, flare=0.05),
            grip=KnobGrip(style="fluted", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008, angle_deg=12.0),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            center=False,
        ),
        "oven_knob",
    )
    for idx, z in enumerate(KNOB_ZS):
        knob = model.part(f"knob_{idx}")
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(0.0, -pi / 2.0, 0.0)),
            material="knob_black",
            name="knob_shell",
        )
        _add_cylinder(
            knob,
            radius=0.005,
            length=0.010,
            xyz=(-0.005, 0.0, 0.0),
            material="rack_steel",
            name="shaft",
            rpy=(0.0, pi / 2.0, 0.0),
        )
        model.articulation(
            f"knob_{idx}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.0, CONTROL_CENTER_Y, z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.5, velocity=8.0),
        )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, CAVITY_CENTER_Y, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=0.0, upper=1.75),
    )
    model.articulation(
        "basket_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=basket,
        origin=Origin(xyz=(BASKET_FRONT_X, CAVITY_CENTER_Y, BASKET_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.30, lower=0.0, upper=BASKET_TRAVEL),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    basket = object_model.get_part("basket")
    door_hinge = object_model.get_articulation("door_hinge")
    basket_slide = object_model.get_articulation("basket_slide")

    ctx.expect_gap(
        body,
        door,
        axis="x",
        min_gap=0.0,
        max_gap=0.001,
        name="closed door closes flush to the front frame",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        min_overlap=0.20,
        name="door covers the oven opening",
    )
    ctx.expect_gap(
        basket,
        body,
        axis="y",
        positive_elem="left_wall",
        negative_elem="left_shell",
        min_gap=0.010,
        max_gap=0.014,
        name="basket clears the left cavity wall",
    )
    ctx.expect_gap(
        body,
        basket,
        axis="y",
        positive_elem="divider_wall",
        negative_elem="right_wall",
        min_gap=0.010,
        max_gap=0.014,
        name="basket clears the right cavity wall",
    )
    ctx.expect_gap(
        basket,
        body,
        axis="z",
        positive_elem="basket_floor",
        negative_elem="bottom_panel",
        min_gap=0.065,
        max_gap=0.080,
        name="basket rides above the floor on side runners",
    )
    ctx.expect_overlap(
        basket,
        body,
        axes="x",
        elem_a="left_wall",
        elem_b="runner_0",
        min_overlap=0.24,
        name="closed basket remains deeply engaged on the runner",
    )

    rest_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="door_handle"))
    basket_rest = ctx.part_world_position(basket)
    with ctx.pose({door_hinge: 1.65}):
        open_handle = _aabb_center(ctx.part_element_world_aabb(door, elem="door_handle"))

    ctx.check(
        "door swings downward and outward",
        rest_handle is not None
        and open_handle is not None
        and open_handle[2] < rest_handle[2] - 0.14
        and open_handle[0] < rest_handle[0] - 0.12,
        details=f"rest_handle={rest_handle}, open_handle={open_handle}",
    )

    with ctx.pose({door_hinge: 1.65, basket_slide: BASKET_TRAVEL}):
        ctx.expect_overlap(
            basket,
            body,
            axes="x",
            elem_a="left_wall",
            elem_b="runner_0",
            min_overlap=0.08,
            name="extended basket still retains insertion on the runner",
        )
        basket_extended = ctx.part_world_position(basket)

    ctx.check(
        "basket pulls straight out of the cavity",
        basket_rest is not None
        and basket_extended is not None
        and basket_extended[0] < basket_rest[0] - 0.12,
        details=f"rest={basket_rest}, extended={basket_extended}",
    )

    for idx in range(3):
        knob_joint = object_model.get_articulation(f"knob_{idx}_spin")
        limits = knob_joint.motion_limits
        ctx.check(
            f"knob_{idx} is a continuous rotary control",
            knob_joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=(
                f"type={knob_joint.articulation_type}, "
                f"lower={None if limits is None else limits.lower}, "
                f"upper={None if limits is None else limits.upper}"
            ),
        )

    return ctx.report()


object_model = build_object_model()
