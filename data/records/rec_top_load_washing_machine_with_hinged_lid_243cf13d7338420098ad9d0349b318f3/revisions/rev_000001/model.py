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


def _lid_shape(width: float, depth: float, thickness: float, wall: float) -> object:
    shell = (
        cq.Workplane("XY")
        .box(width, depth, thickness, centered=(True, True, False))
        .translate((0.0, -depth * 0.5, 0.0))
    )
    cavity = (
        cq.Workplane("XY")
        .box(
            width - 2.0 * wall,
            depth - 2.0 * wall,
            thickness - wall,
            centered=(True, True, False),
        )
        .translate((0.0, -depth * 0.5, wall))
    )
    return shell.cut(cavity)


def _outer_tub_shape(outer_radius: float, inner_radius: float, depth: float) -> object:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(-depth)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(-(depth - 0.012))
        .translate((0.0, 0.0, -0.004))
    )
    return outer.cut(inner)


def _basket_shape(outer_radius: float, inner_radius: float, depth: float) -> object:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(-depth)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(-(depth - 0.010))
        .translate((0.0, 0.0, -0.004))
    )
    basket = outer.cut(inner)

    slot_width = 0.020
    slot_depth = 0.034
    slot_height = 0.016
    for row_z in (-0.085, -0.155, -0.225, -0.295):
        for angle_deg in range(0, 360, 30):
            cutter = (
                cq.Workplane("XY")
                .box(slot_width, slot_depth, slot_height)
                .translate((0.0, outer_radius - 0.002, row_z))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), float(angle_deg))
            )
            basket = basket.cut(cutter)
    return basket


def _annulus_shape(outer_radius: float, inner_radius: float, thickness: float) -> object:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(thickness)
    inner = cq.Workplane("XY").circle(inner_radius).extrude(thickness)
    return outer.cut(inner)


def _agitator_shape() -> object:
    cup = cq.Workplane("XY").circle(0.046).extrude(-0.032)
    cup_recess = cq.Workplane("XY").circle(0.034).extrude(-0.021)
    upper = (
        cq.Workplane("XY")
        .circle(0.043)
        .workplane(offset=-0.085)
        .circle(0.080)
        .loft(combine=True)
    )
    lower = (
        cq.Workplane("XY")
        .workplane(offset=-0.085)
        .circle(0.080)
        .workplane(offset=-0.185)
        .circle(0.060)
        .loft(combine=True)
    )
    base = (
        cq.Workplane("XY")
        .workplane(offset=-0.270)
        .circle(0.052)
        .extrude(-0.208)
    )
    agitator = cup.cut(cup_recess).union(upper).union(lower).union(base)
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        fin = (
            cq.Workplane("XY")
            .box(0.070, 0.008, 0.250, centered=(True, True, False))
            .translate((0.0, 0.026, -0.250))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        agitator = agitator.union(fin)
    return agitator


def _softener_lid_shape(radius: float, thickness: float) -> object:
    lid = (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(thickness)
        .translate((0.0, -radius, 0.0))
    )
    tab = (
        cq.Workplane("XY")
        .box(radius * 0.7, 0.010, thickness, centered=(True, True, False))
        .translate((0.0, -2.0 * radius + 0.005, 0.0))
    )
    return lid.union(tab)


def _control_plate_shape(
    width: float,
    height: float,
    thickness: float,
    knob_center_x: float,
    button_centers_x: tuple[float, float, float],
    control_center_z: float,
) -> object:
    plate = cq.Workplane("XZ").rect(width, height).extrude(thickness)
    knob_cut = (
        cq.Workplane("XZ")
        .circle(0.030)
        .extrude(thickness * 2.0)
        .translate((knob_center_x, -thickness * 0.5, control_center_z))
    )
    plate = plate.cut(knob_cut)
    for center_x in button_centers_x:
        button_cut = (
            cq.Workplane("XZ")
            .box(0.030, 0.018, thickness * 2.0)
            .translate((center_x, -thickness * 0.5, control_center_z))
        )
        plate = plate.cut(button_cut)
    return plate


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((low[i] + high[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_top_load_washer")

    enamel = model.material("enamel_white", rgba=(0.95, 0.96, 0.97, 1.0))
    trim = model.material("trim_grey", rgba=(0.72, 0.74, 0.77, 1.0))
    pod_dark = model.material("pod_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    glass = model.material("smoked_lid", rgba=(0.30, 0.34, 0.38, 0.55))
    basket_metal = model.material("basket_metal", rgba=(0.79, 0.82, 0.85, 1.0))
    tub_grey = model.material("tub_grey", rgba=(0.83, 0.85, 0.88, 1.0))
    agitator_grey = model.material("agitator_grey", rgba=(0.88, 0.89, 0.90, 1.0))
    control_black = model.material("control_black", rgba=(0.12, 0.13, 0.15, 1.0))
    button_silver = model.material("button_silver", rgba=(0.86, 0.88, 0.90, 1.0))

    cabinet_w = 0.55
    cabinet_d = 0.60
    cabinet_h = 0.97
    wall = 0.022
    top_thickness = 0.028
    top_z = cabinet_h - 0.035

    opening_w = 0.420
    opening_d = 0.390
    opening_center_y = -0.060
    opening_rear_y = opening_center_y + opening_d * 0.5

    lid_width = 0.468
    lid_depth = 0.408
    lid_thickness = 0.026
    lid_hinge_y = opening_rear_y + 0.010
    lid_hinge_z = top_z + 0.003

    pod_w = 0.420
    pod_d = 0.145
    pod_h = 0.085
    pod_wall = 0.014
    pod_front_y = cabinet_d * 0.5 - pod_d
    pod_center_y = pod_front_y + pod_d * 0.5
    pod_rear_y = cabinet_d * 0.5
    pod_bottom_z = top_z - 0.001
    control_panel_height = 0.062
    control_center_z = pod_bottom_z + 0.054
    control_plate = _control_plate_shape(
        width=pod_w - 2.0 * pod_wall,
        height=control_panel_height,
        thickness=0.004,
        knob_center_x=-0.112,
        button_centers_x=(0.048, 0.092, 0.136),
        control_center_z=0.0,
    )

    cabinet = model.part("cabinet")
    body_height = top_z - top_thickness * 0.5
    cabinet.visual(
        Box((wall, cabinet_d, body_height)),
        origin=Origin(xyz=(-cabinet_w * 0.5 + wall * 0.5, 0.0, body_height * 0.5)),
        material=enamel,
        name="left_side",
    )
    cabinet.visual(
        Box((wall, cabinet_d, body_height)),
        origin=Origin(xyz=(cabinet_w * 0.5 - wall * 0.5, 0.0, body_height * 0.5)),
        material=enamel,
        name="right_side",
    )
    cabinet.visual(
        Box((cabinet_w - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, -cabinet_d * 0.5 + wall * 0.5, body_height * 0.5)),
        material=enamel,
        name="front_panel",
    )
    cabinet.visual(
        Box((cabinet_w - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, cabinet_d * 0.5 - wall * 0.5, body_height * 0.5)),
        material=enamel,
        name="back_panel",
    )
    cabinet.visual(
        Box((cabinet_w - 2.0 * wall, cabinet_d - 2.0 * wall, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=trim,
        name="base_pan",
    )

    front_rim_depth = 0.048
    side_rim_width = (cabinet_w - opening_w) * 0.5 + 0.002
    side_rim_depth = opening_d + 0.026
    rear_deck_depth = cabinet_d * 0.5 - opening_rear_y
    cabinet.visual(
        Box((cabinet_w, front_rim_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -cabinet_d * 0.5 + front_rim_depth * 0.5,
                top_z - top_thickness * 0.5,
            )
        ),
        material=enamel,
        name="front_rim",
    )
    cabinet.visual(
        Box((side_rim_width, side_rim_depth, top_thickness)),
        origin=Origin(
            xyz=(
                -opening_w * 0.5 - side_rim_width * 0.5 + 0.001,
                opening_center_y,
                top_z - top_thickness * 0.5,
            )
        ),
        material=enamel,
        name="left_rim",
    )
    cabinet.visual(
        Box((side_rim_width, side_rim_depth, top_thickness)),
        origin=Origin(
            xyz=(
                opening_w * 0.5 + side_rim_width * 0.5 - 0.001,
                opening_center_y,
                top_z - top_thickness * 0.5,
            )
        ),
        material=enamel,
        name="right_rim",
    )
    cabinet.visual(
        Box((cabinet_w, rear_deck_depth, top_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                opening_rear_y + rear_deck_depth * 0.5,
                top_z - top_thickness * 0.5,
            )
        ),
        material=enamel,
        name="rear_deck",
    )

    cabinet.visual(
        Box((pod_wall, pod_d, pod_h)),
        origin=Origin(
            xyz=(
                -pod_w * 0.5 + pod_wall * 0.5,
                pod_center_y,
                pod_bottom_z + pod_h * 0.5,
            )
        ),
        material=pod_dark,
        name="pod_left",
    )
    cabinet.visual(
        Box((pod_wall, pod_d, pod_h)),
        origin=Origin(
            xyz=(
                pod_w * 0.5 - pod_wall * 0.5,
                pod_center_y,
                pod_bottom_z + pod_h * 0.5,
            )
        ),
        material=pod_dark,
        name="pod_right",
    )
    cabinet.visual(
        Box((pod_w, pod_d, pod_wall)),
        origin=Origin(
            xyz=(0.0, pod_center_y, pod_bottom_z + pod_h - pod_wall * 0.5)
        ),
        material=pod_dark,
        name="pod_top",
    )
    cabinet.visual(
        Box((pod_w - 2.0 * pod_wall, pod_wall, pod_h - pod_wall)),
        origin=Origin(
            xyz=(
                0.0,
                pod_rear_y - pod_wall * 0.5,
                pod_bottom_z + (pod_h - pod_wall) * 0.5,
            )
        ),
        material=pod_dark,
        name="pod_back",
    )
    cabinet.visual(
        mesh_from_cadquery(control_plate, "washer_control_plate"),
        origin=Origin(xyz=(0.0, pod_front_y, control_center_z)),
        material=pod_dark,
        name="control_plate",
    )

    tub = model.part("tub")
    tub_wall_radius = 0.184
    tub_wall_height = 0.500
    tub_wall_count = 16
    tub_wall_width = 0.076
    tub_wall_thickness = 0.010
    for index in range(tub_wall_count):
        angle = 2.0 * math.pi * index / tub_wall_count
        yaw = angle + math.pi * 0.5
        x = tub_wall_radius * math.cos(angle)
        y = tub_wall_radius * math.sin(angle)
        tub.visual(
            Box((tub_wall_width, tub_wall_thickness, tub_wall_height)),
            origin=Origin(xyz=(x, y, -tub_wall_height * 0.5), rpy=(0.0, 0.0, yaw)),
            material=tub_grey,
            name=f"tub_wall_{index}",
        )
    for index in range(3):
        angle = 2.0 * math.pi * index / 3.0
        yaw = angle + math.pi * 0.5
        x = 0.1765 * math.cos(angle)
        y = 0.1765 * math.sin(angle)
        tub.visual(
            Box((0.030, 0.005, 0.018)),
            origin=Origin(xyz=(x, y, -0.010), rpy=(0.0, 0.0, yaw)),
            material=tub_grey,
            name=f"basket_guide_{index}",
        )
    tub.visual(
        mesh_from_cadquery(_annulus_shape(0.194, 0.178, 0.016), "washer_tub_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=tub_grey,
        name="tub_ring",
    )
    tub.visual(
        Cylinder(radius=0.188, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.493)),
        material=tub_grey,
        name="tub_floor",
    )
    model.articulation(
        "cabinet_to_tub",
        ArticulationType.FIXED,
        parent=cabinet,
        child=tub,
        origin=Origin(xyz=(0.0, opening_center_y, top_z)),
    )

    basket = model.part("basket")
    basket_slat_radius = 0.164
    basket_slat_height = 0.374
    basket_slat_count = 20
    for index in range(basket_slat_count):
        angle = 2.0 * math.pi * index / basket_slat_count
        yaw = angle + math.pi * 0.5
        x = basket_slat_radius * math.cos(angle)
        y = basket_slat_radius * math.sin(angle)
        basket.visual(
            Box((0.018, 0.006, basket_slat_height)),
            origin=Origin(xyz=(x, y, -0.191), rpy=(0.0, 0.0, yaw)),
            material=basket_metal,
            name=f"basket_slat_{index}",
        )
    basket.visual(
        mesh_from_cadquery(_annulus_shape(0.174, 0.146, 0.016), "washer_basket_top_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=basket_metal,
        name="basket_top_ring",
    )
    basket.visual(
        mesh_from_cadquery(_annulus_shape(0.170, 0.146, 0.012), "washer_basket_bottom_ring"),
        origin=Origin(xyz=(0.0, 0.0, -0.390)),
        material=basket_metal,
        name="basket_bottom_ring",
    )
    basket.visual(
        Box((0.012, 0.006, 0.060)),
        origin=Origin(xyz=(0.164, 0.0, -0.165)),
        material=basket_metal,
        name="basket_rib",
    )
    model.articulation(
        "tub_to_basket",
        ArticulationType.CONTINUOUS,
        parent=tub,
        child=basket,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=18.0),
    )

    agitator = model.part("agitator")
    agitator.visual(
        mesh_from_cadquery(_agitator_shape(), "washer_agitator"),
        material=agitator_grey,
        name="agitator_shell",
    )
    model.articulation(
        "tub_to_agitator",
        ArticulationType.FIXED,
        parent=tub,
        child=agitator,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )

    softener_lid = model.part("softener_lid")
    softener_lid.visual(
        mesh_from_cadquery(_softener_lid_shape(0.034, 0.005), "washer_softener_lid"),
        material=agitator_grey,
        name="softener_lid",
    )
    model.articulation(
        "agitator_to_softener_lid",
        ArticulationType.REVOLUTE,
        parent=agitator,
        child=softener_lid,
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=1.35,
        ),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_shape(lid_width, lid_depth, lid_thickness, 0.003), "washer_lid"),
        material=glass,
        name="lid_shell",
    )
    lid.visual(
        Box((0.030, 0.012, 0.004)),
        origin=Origin(xyz=(-0.155, 0.006, -0.001)),
        material=trim,
        name="hinge_foot_0",
    )
    lid.visual(
        Box((0.030, 0.012, 0.004)),
        origin=Origin(xyz=(0.155, 0.006, -0.001)),
        material=trim,
        name="hinge_foot_1",
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, lid_hinge_y, lid_hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.38,
        ),
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.029, length=0.006),
        origin=Origin(xyz=(0.0, -0.003, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="knob_skirt",
    )
    knob.visual(
        Cylinder(radius=0.022, length=0.022),
        origin=Origin(xyz=(0.0, -0.017, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=control_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.003, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.028, 0.016)),
        material=trim,
        name="pointer",
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(-0.112, pod_front_y, control_center_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=6.0),
    )

    button_xs = (0.048, 0.092, 0.136)
    for index, center_x in enumerate(button_xs):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.026, 0.009, 0.014)),
            origin=Origin(xyz=(0.0, -0.0045, 0.0)),
            material=button_silver,
            name="button_cap",
        )
        button.visual(
            Box((0.038, 0.004, 0.022)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=control_black,
            name="button_guide",
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(center_x, pod_front_y, control_center_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=0.08,
                lower=0.0,
                upper=0.004,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    tub = object_model.get_part("tub")
    basket = object_model.get_part("basket")
    knob = object_model.get_part("selector_knob")
    softener_lid = object_model.get_part("softener_lid")

    lid_joint = object_model.get_articulation("cabinet_to_lid")
    basket_joint = object_model.get_articulation("tub_to_basket")
    knob_joint = object_model.get_articulation("cabinet_to_selector_knob")
    softener_joint = object_model.get_articulation("agitator_to_softener_lid")

    ctx.expect_gap(
        lid,
        cabinet,
        axis="z",
        positive_elem="lid_shell",
        negative_elem="front_rim",
        min_gap=0.0,
        max_gap=0.010,
        name="lid seats just above the front rim",
    )
    ctx.expect_within(
        basket,
        tub,
        axes="xy",
        margin=0.030,
        name="basket stays centered within the tub opening",
    )
    ctx.expect_overlap(
        basket,
        tub,
        axes="z",
        min_overlap=0.34,
        name="basket remains deeply inserted in the tub",
    )
    ctx.expect_overlap(
        knob,
        cabinet,
        axes="xz",
        elem_a="knob_body",
        elem_b="control_plate",
        min_overlap=0.020,
        name="selector knob is mounted within the control pod panel area",
    )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    with ctx.pose({lid_joint: 1.25}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="lid_shell")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.20
        and open_lid_aabb[0][1] > closed_lid_aabb[0][1] + 0.14,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    basket_rib_rest = _aabb_center(ctx.part_element_world_aabb(basket, elem="basket_rib"))
    with ctx.pose({basket_joint: math.pi * 0.5}):
        basket_rib_turned = _aabb_center(ctx.part_element_world_aabb(basket, elem="basket_rib"))
    ctx.check(
        "basket articulation rotates the basket around the vertical axis",
        basket_rib_rest is not None
        and basket_rib_turned is not None
        and abs(basket_rib_rest[0] - basket_rib_turned[0]) > 0.10
        and abs(basket_rib_rest[1] - basket_rib_turned[1]) > 0.10,
        details=f"rest={basket_rib_rest}, turned={basket_rib_turned}",
    )

    pointer_rest = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    with ctx.pose({knob_joint: math.pi * 0.5}):
        pointer_turned = _aabb_center(ctx.part_element_world_aabb(knob, elem="pointer"))
    ctx.check(
        "selector knob rotates continuously on the pod",
        pointer_rest is not None
        and pointer_turned is not None
        and abs(pointer_rest[0] - pointer_turned[0]) > 0.010,
        details=f"rest={pointer_rest}, turned={pointer_turned}",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        button_joint = object_model.get_articulation(f"cabinet_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({button_joint: 0.004}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button_{index} presses inward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[1] > rest_pos[1] + 0.003,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    softener_closed = ctx.part_element_world_aabb(softener_lid, elem="softener_lid")
    with ctx.pose({softener_joint: 1.20}):
        softener_open = ctx.part_element_world_aabb(softener_lid, elem="softener_lid")
    ctx.check(
        "softener cup lid flips upward on the agitator top",
        softener_closed is not None
        and softener_open is not None
        and softener_open[1][2] > softener_closed[1][2] + 0.020
        and softener_open[0][1] > softener_closed[0][1] + 0.015,
        details=f"closed={softener_closed}, open={softener_open}",
    )

    return ctx.report()


object_model = build_object_model()
