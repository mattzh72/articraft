from __future__ import annotations

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


BODY_D = 0.340
BODY_W = 0.285
BODY_H = 0.388
BODY_Z0 = 0.012
BODY_FRONT_X = BODY_D * 0.5

DRAWER_D = 0.228
DRAWER_W = 0.236
DRAWER_H = 0.178
DRAWER_FRONT_T = 0.016
DRAWER_WALL = 0.005
DRAWER_Z0 = 0.060
DRAWER_TRAVEL = 0.145

BASKET_D = 0.206
BASKET_W = 0.214
BASKET_H = 0.147
BASKET_WALL = 0.004
BASKET_FRONT_GAP = 0.026


def _build_housing_shell():
    shell = (
        cq.Workplane("XY")
        .box(BODY_D, BODY_W, BODY_H, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.020)
        .translate((0.0, 0.0, BODY_Z0))
    )

    cavity = (
        cq.Workplane("XY")
        .box(0.252, 0.244, 0.192, centered=(True, True, False))
        .translate((0.044, 0.0, 0.056))
    )

    return shell.cut(cavity)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="glass_window_air_fryer")

    body_finish = model.material("body_finish", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.19, 0.20, 0.22, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.17, 0.18, 0.19, 1.0))
    basket_finish = model.material("basket_finish", rgba=(0.23, 0.24, 0.25, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.34, 0.41, 0.46, 0.28))
    foot_finish = model.material("foot_finish", rgba=(0.07, 0.07, 0.08, 1.0))
    accent_finish = model.material("accent_finish", rgba=(0.56, 0.58, 0.61, 1.0))
    control_finish = model.material("control_finish", rgba=(0.10, 0.11, 0.12, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_build_housing_shell(), "housing_shell"),
        material=body_finish,
        name="body_shell",
    )
    housing.visual(
        Box((0.220, 0.180, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, BODY_Z0 + BODY_H - 0.005)),
        material=trim_finish,
        name="top_cap",
    )
    housing.visual(
        Box((0.146, 0.010, 0.248)),
        origin=Origin(xyz=(0.046, BODY_W * 0.5 + 0.005, 0.208)),
        material=trim_finish,
        name="control_panel",
    )
    for x_pos, z_pos, name in (
        (0.018, 0.272, "dial_ring_0"),
        (0.018, 0.190, "dial_ring_1"),
    ):
        housing.visual(
            Cylinder(radius=0.031, length=0.004),
            origin=Origin(xyz=(x_pos, BODY_W * 0.5 + 0.012, z_pos), rpy=(1.57079632679, 0.0, 0.0)),
            material=accent_finish,
            name=name,
        )
    for x_pos, z_pos, name in (
        (0.088, 0.274, "button_bezel_0"),
        (0.088, 0.224, "button_bezel_1"),
        (0.088, 0.174, "button_bezel_2"),
    ):
        housing.visual(
            Box((0.026, 0.006, 0.022)),
            origin=Origin(xyz=(x_pos, BODY_W * 0.5 + 0.008, z_pos)),
            material=accent_finish,
            name=name,
        )
    for y_pos, name in ((-0.122, "guide_rail_0"), (0.122, "guide_rail_1")):
        housing.visual(
            Box((0.248, 0.010, 0.012)),
            origin=Origin(xyz=(0.044, y_pos, DRAWER_Z0 + 0.016)),
            material=trim_finish,
            name=name,
        )
    for x_pos in (-0.120, 0.120):
        for y_pos in (-0.092, 0.092):
            housing.visual(
                Cylinder(radius=0.014, length=0.012),
                origin=Origin(xyz=(x_pos, y_pos, 0.006)),
                material=foot_finish,
                name=f"foot_{int((x_pos > 0.0))}_{int((y_pos > 0.0))}",
            )

    drawer = model.part("drawer")
    drawer.visual(
        Box((DRAWER_D - DRAWER_FRONT_T, DRAWER_W, DRAWER_WALL)),
        origin=Origin(xyz=(-(DRAWER_D + DRAWER_FRONT_T) * 0.5, 0.000, DRAWER_WALL * 0.5)),
        material=trim_finish,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((DRAWER_D - DRAWER_FRONT_T, DRAWER_WALL, DRAWER_H)),
        origin=Origin(
            xyz=(
                -(DRAWER_D + DRAWER_FRONT_T) * 0.5,
                -(DRAWER_W - DRAWER_WALL) * 0.5,
                DRAWER_H * 0.5,
            )
        ),
        material=trim_finish,
        name="drawer_side_0",
    )
    drawer.visual(
        Box((DRAWER_D - DRAWER_FRONT_T, DRAWER_WALL, DRAWER_H)),
        origin=Origin(
            xyz=(
                -(DRAWER_D + DRAWER_FRONT_T) * 0.5,
                (DRAWER_W - DRAWER_WALL) * 0.5,
                DRAWER_H * 0.5,
            )
        ),
        material=trim_finish,
        name="drawer_side_1",
    )
    drawer.visual(
        Box((DRAWER_WALL, DRAWER_W - 2.0 * DRAWER_WALL, DRAWER_H)),
        origin=Origin(xyz=(-DRAWER_D + DRAWER_WALL * 0.5, 0.000, DRAWER_H * 0.5)),
        material=trim_finish,
        name="drawer_back",
    )
    drawer.visual(
        Box((DRAWER_FRONT_T, DRAWER_W, 0.094)),
        origin=Origin(xyz=(-DRAWER_FRONT_T * 0.5, 0.000, 0.047)),
        material=body_finish,
        name="front_lower",
    )
    drawer.visual(
        Box((DRAWER_FRONT_T, DRAWER_W, DRAWER_H - 0.148)),
        origin=Origin(xyz=(-DRAWER_FRONT_T * 0.5, 0.000, 0.163)),
        material=body_finish,
        name="front_upper",
    )
    for y_pos, name in ((-0.091, "front_jamb_0"), (0.091, "front_jamb_1")):
        drawer.visual(
            Box((DRAWER_FRONT_T, 0.054, 0.054)),
            origin=Origin(xyz=(-DRAWER_FRONT_T * 0.5, y_pos, 0.121)),
            material=body_finish,
            name=name,
        )
    for y_pos, name in ((-0.114, "slider_shoe_0"), (0.114, "slider_shoe_1")):
        drawer.visual(
            Box((0.206, 0.006, 0.010)),
            origin=Origin(xyz=(-0.122, y_pos, 0.016)),
            material=trim_finish,
            name=name,
        )
    drawer.visual(
        Box((0.046, 0.014, 0.050)),
        origin=Origin(xyz=(0.021, -0.060, 0.055)),
        material=handle_finish,
        name="handle_cheek_0",
    )
    drawer.visual(
        Box((0.046, 0.014, 0.050)),
        origin=Origin(xyz=(0.021, 0.060, 0.055)),
        material=handle_finish,
        name="handle_cheek_1",
    )
    drawer.visual(
        Box((0.028, 0.140, 0.030)),
        origin=Origin(xyz=(0.030, 0.000, 0.056)),
        material=handle_finish,
        name="handle_grip",
    )
    drawer.visual(
        Box((0.018, 0.042, 0.010)),
        origin=Origin(xyz=(0.008, -0.049, 0.080)),
        material=handle_finish,
        name="handle_top_0",
    )
    drawer.visual(
        Box((0.018, 0.042, 0.010)),
        origin=Origin(xyz=(0.008, 0.049, 0.080)),
        material=handle_finish,
        name="handle_top_1",
    )
    drawer.visual(
        Box((0.003, 0.136, 0.062)),
        origin=Origin(xyz=(-0.009, 0.000, 0.121)),
        material=glass_tint,
        name="window_glass",
    )

    basket = model.part("basket")
    basket.visual(
        Box((BASKET_D, BASKET_W, BASKET_WALL)),
        origin=Origin(xyz=(-BASKET_D * 0.5, 0.000, BASKET_WALL * 0.5)),
        material=basket_finish,
        name="basket_bottom",
    )
    basket.visual(
        Box((BASKET_D, BASKET_WALL, BASKET_H)),
        origin=Origin(xyz=(-BASKET_D * 0.5, -(BASKET_W - BASKET_WALL) * 0.5, BASKET_H * 0.5)),
        material=basket_finish,
        name="basket_side_0",
    )
    basket.visual(
        Box((BASKET_D, BASKET_WALL, BASKET_H)),
        origin=Origin(xyz=(-BASKET_D * 0.5, (BASKET_W - BASKET_WALL) * 0.5, BASKET_H * 0.5)),
        material=basket_finish,
        name="basket_side_1",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_W - 2.0 * BASKET_WALL, BASKET_H)),
        origin=Origin(xyz=(-BASKET_D + BASKET_WALL * 0.5, 0.000, BASKET_H * 0.5)),
        material=basket_finish,
        name="basket_back",
    )
    basket.visual(
        Box((BASKET_WALL, 0.050, 0.064)),
        origin=Origin(xyz=(-BASKET_WALL * 0.5, -0.078, 0.032)),
        material=basket_finish,
        name="basket_front_0",
    )
    basket.visual(
        Box((BASKET_WALL, 0.050, 0.064)),
        origin=Origin(xyz=(-BASKET_WALL * 0.5, 0.078, 0.032)),
        material=basket_finish,
        name="basket_front_1",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_W, 0.032)),
        origin=Origin(xyz=(-BASKET_WALL * 0.5, 0.000, 0.016)),
        material=basket_finish,
        name="basket_front_lower",
    )
    basket.visual(
        Box((BASKET_WALL, BASKET_W, 0.035)),
        origin=Origin(xyz=(-BASKET_WALL * 0.5, 0.000, 0.1295)),
        material=basket_finish,
        name="basket_front_upper",
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.018, 0.046, 0.006)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=accent_finish,
        name="release_cap",
    )
    release_button.visual(
        Box((0.012, 0.028, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.008)),
        material=accent_finish,
        name="release_stem",
    )

    for part_name, x_pos, z_pos in (
        ("dial_0", 0.018, 0.272),
        ("dial_1", 0.018, 0.190),
    ):
        dial = model.part(part_name)
        dial.visual(
            Cylinder(radius=0.028, length=0.008),
            origin=Origin(xyz=(0.000, 0.004, 0.000), rpy=(1.57079632679, 0.0, 0.0)),
            material=control_finish,
            name="dial_skirt",
        )
        dial.visual(
            Cylinder(radius=0.021, length=0.018),
            origin=Origin(xyz=(0.000, 0.017, 0.000), rpy=(1.57079632679, 0.0, 0.0)),
            material=control_finish,
            name="dial_cap",
        )
        dial.visual(
            Box((0.004, 0.004, 0.010)),
            origin=Origin(xyz=(0.000, 0.028, 0.013)),
            material=accent_finish,
            name="dial_marker",
        )
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=dial,
            origin=Origin(xyz=(x_pos, BODY_W * 0.5 + 0.010, z_pos)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=8.0),
        )

    for index, x_pos, z_pos in (
        (0, 0.088, 0.274),
        (1, 0.088, 0.224),
        (2, 0.088, 0.174),
    ):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.020, 0.010, 0.016)),
            origin=Origin(xyz=(0.000, 0.009, 0.000)),
            material=accent_finish,
            name="button_cap",
        )
        button.visual(
            Box((0.010, 0.004, 0.010)),
            origin=Origin(xyz=(0.000, 0.002, 0.000)),
            material=accent_finish,
            name="button_stem",
        )
        model.articulation(
            f"housing_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=button,
            origin=Origin(xyz=(x_pos, BODY_W * 0.5 + 0.011, z_pos)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=6.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    model.articulation(
        "housing_to_drawer",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=drawer,
        origin=Origin(xyz=(BODY_FRONT_X, 0.000, DRAWER_Z0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=55.0,
            velocity=0.28,
            lower=0.0,
            upper=DRAWER_TRAVEL,
        ),
    )
    model.articulation(
        "drawer_to_basket",
        ArticulationType.FIXED,
        parent=drawer,
        child=basket,
        origin=Origin(xyz=(-BASKET_FRONT_GAP, 0.000, 0.012)),
    )
    model.articulation(
        "drawer_to_release_button",
        ArticulationType.PRISMATIC,
        parent=drawer,
        child=release_button,
        origin=Origin(xyz=(0.022, 0.000, 0.084)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.08,
            lower=0.0,
            upper=0.004,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    release_button = object_model.get_part("release_button")
    drawer_slide = object_model.get_articulation("housing_to_drawer")
    button_slide = object_model.get_articulation("drawer_to_release_button")

    ctx.expect_within(
        basket,
        drawer,
        axes="yz",
        margin=0.004,
        name="basket stays centered inside drawer shell",
    )
    ctx.expect_overlap(
        basket,
        drawer,
        axes="x",
        min_overlap=0.150,
        name="basket remains retained within drawer tub",
    )
    ctx.expect_overlap(
        drawer,
        housing,
        axes="yz",
        elem_a="front_lower",
        elem_b="body_shell",
        min_overlap=0.090,
        name="drawer front covers the housing opening footprint",
    )

    body_aabb = ctx.part_world_aabb(housing)
    drawer_front_aabb = ctx.part_element_world_aabb(drawer, elem="front_lower")
    ctx.check(
        "drawer_front_near_housing_front_when_closed",
        body_aabb is not None
        and drawer_front_aabb is not None
        and abs(float(drawer_front_aabb[1][0]) - float(body_aabb[1][0])) <= 0.004,
        details=f"body={body_aabb!r}, drawer_front={drawer_front_aabb!r}",
    )

    closed_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: DRAWER_TRAVEL}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            basket,
            drawer,
            axes="x",
            min_overlap=0.150,
            name="basket stays retained when drawer is fully extended",
        )
    ctx.check(
        "drawer_extends_forward",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[0] > closed_drawer_pos[0] + 0.12,
        details=f"closed={closed_drawer_pos!r}, extended={extended_drawer_pos!r}",
    )

    released_pos = ctx.part_world_position(release_button)
    with ctx.pose({button_slide: 0.004}):
        pressed_pos = ctx.part_world_position(release_button)
    ctx.check(
        "handle_release_button_presses_downward",
        released_pos is not None
        and pressed_pos is not None
        and pressed_pos[2] < released_pos[2] - 0.0025,
        details=f"rest={released_pos!r}, pressed={pressed_pos!r}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    for index in (0, 1):
        dial = object_model.get_part(f"dial_{index}")
        dial_joint = object_model.get_articulation(f"housing_to_dial_{index}")
        rest_marker = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
        with ctx.pose({dial_joint: 1.0}):
            turned_marker = _aabb_center(ctx.part_element_world_aabb(dial, elem="dial_marker"))
        ctx.check(
            f"dial_{index}_rotates",
            rest_marker is not None
            and turned_marker is not None
            and (
                abs(turned_marker[0] - rest_marker[0]) > 0.004
                or abs(turned_marker[2] - rest_marker[2]) > 0.004
            ),
            details=f"rest={rest_marker!r}, turned={turned_marker!r}",
        )

    for index in (0, 1, 2):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"housing_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index}_presses_inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] < rest_pos[1] - 0.003,
            details=f"rest={rest_pos!r}, pressed={pressed_pos!r}",
        )

    return ctx.report()


object_model = build_object_model()
