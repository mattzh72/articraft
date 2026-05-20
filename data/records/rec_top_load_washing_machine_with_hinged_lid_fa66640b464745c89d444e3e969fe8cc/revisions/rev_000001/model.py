from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


CABINET_WIDTH = 0.685
CABINET_DEPTH = 0.715
DECK_HEIGHT = 0.935
CONSOLE_HEIGHT = 0.155
OPENING_CENTER_Y = -0.04
OPENING_RADIUS = 0.205
DRAWER_CENTER_X = -0.274
DRAWER_CENTER_Y = 0.020


def _cabinet_shape() -> cq.Workplane:
    wall = 0.038
    console_depth = 0.130
    console_width = 0.630

    shell = cq.Workplane("XY").box(
        CABINET_WIDTH,
        CABINET_DEPTH,
        DECK_HEIGHT,
        centered=(True, True, False),
    )
    shell = shell.edges("|Z").fillet(0.028)

    cavity = cq.Workplane("XY").box(
        CABINET_WIDTH - 2.0 * wall,
        CABINET_DEPTH - 2.0 * wall,
        DECK_HEIGHT - 0.055,
        centered=(True, True, False),
    ).translate((0.0, 0.0, 0.030))

    tub_opening = (
        cq.Workplane("XY")
        .center(0.0, OPENING_CENTER_Y)
        .circle(OPENING_RADIUS)
        .extrude(0.085)
        .translate((0.0, 0.0, DECK_HEIGHT - 0.060))
    )
    drawer_slot = (
        cq.Workplane("XY")
        .center(DRAWER_CENTER_X, DRAWER_CENTER_Y)
        .rect(0.096, 0.205)
        .extrude(0.085)
        .translate((0.0, 0.0, DECK_HEIGHT - 0.060))
    )

    cabinet = shell.cut(cavity).cut(tub_opening).cut(drawer_slot)

    console = cq.Workplane("XY").box(
        console_width,
        console_depth,
        CONSOLE_HEIGHT,
        centered=(True, True, False),
    ).translate((0.0, CABINET_DEPTH * 0.5 - console_depth * 0.5 - 0.012, DECK_HEIGHT))
    console = console.edges("|Z").fillet(0.016)

    tub_well = (
        cq.Workplane("XY")
        .center(0.0, OPENING_CENTER_Y)
        .circle(0.225)
        .circle(0.213)
        .extrude(0.640)
        .translate((0.0, 0.0, 0.295))
    )

    return cabinet.union(console).union(tub_well)


def _tub_ring_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .center(0.0, OPENING_CENTER_Y)
        .circle(0.219)
        .circle(OPENING_RADIUS)
        .extrude(0.010)
        .translate((0.0, 0.0, DECK_HEIGHT - 0.008))
    )


def _lid_frame_shape() -> cq.Workplane:
    width = 0.620
    depth = 0.548
    thickness = 0.026

    frame = cq.Workplane("XY").box(
        width,
        depth,
        thickness,
        centered=(True, True, False),
    ).translate((0.0, -depth * 0.5, -0.010))
    window = cq.Workplane("XY").box(
        width - 0.116,
        depth - 0.114,
        thickness + 0.014,
        centered=(True, True, False),
    ).translate((0.0, -depth * 0.5, -0.008))
    frame = frame.cut(window)

    rear_rib = cq.Workplane("XY").box(
        width * 0.82,
        0.048,
        0.018,
        centered=(True, True, False),
    ).translate((0.0, -0.024, -0.012))
    front_pull = cq.Workplane("XY").box(
        0.170,
        0.030,
        0.010,
        centered=(True, True, False),
    ).translate((0.0, -depth + 0.018, -0.004))

    return frame.union(rear_rib).union(front_pull)


def _drawer_shape() -> cq.Workplane:
    width = 0.088
    depth = 0.190
    body_height = 0.028

    body = cq.Workplane("XY").box(
        width,
        depth,
        body_height,
        centered=(True, True, False),
    ).translate((0.0, 0.0, -body_height))
    top_pad = cq.Workplane("XY").box(
        width + 0.006,
        depth - 0.016,
        0.005,
        centered=(True, True, False),
    ).translate((0.0, 0.0, -0.001))
    pull = cq.Workplane("XY").box(
        0.040,
        0.012,
        0.008,
        centered=(True, True, False),
    ).translate((0.0, -depth * 0.5 + 0.014, 0.0))

    return body.union(top_pad).union(pull)


def _basket_shape() -> cq.Workplane:
    outer_radius = 0.186
    inner_radius = 0.176
    height = 0.540

    shell = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
        .translate((0.0, 0.0, -height))
    )
    rim = (
        cq.Workplane("XY")
        .circle(outer_radius + 0.010)
        .circle(outer_radius - 0.004)
        .extrude(0.016)
        .translate((0.0, 0.0, -0.010))
    )
    base = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(0.008)
        .translate((0.0, 0.0, -height))
    )
    spindle = (
        cq.Workplane("XY")
        .circle(0.018)
        .extrude(0.070)
        .translate((0.0, 0.0, -height + 0.008))
    )
    wash_plate = (
        cq.Workplane("XY")
        .circle(0.152)
        .extrude(0.016)
        .translate((0.0, 0.0, -height + 0.056))
    )

    basket = shell.union(rim).union(base).union(spindle).union(wash_plate)

    for angle_deg in (0.0, 72.0, 144.0, 216.0, 288.0):
        vane = cq.Workplane("XY").box(
            0.020,
            0.095,
            0.015,
            centered=(True, True, False),
        ).translate((0.060, 0.0, -height + 0.072))
        basket = basket.union(vane.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))

    return basket


def _softener_lid_shape() -> cq.Workplane:
    lid = cq.Workplane("XY").box(
        0.070,
        0.052,
        0.006,
        centered=(True, True, False),
    ).translate((0.0, -0.026, -0.002))
    tab = cq.Workplane("XY").box(
        0.018,
        0.012,
        0.010,
        centered=(True, True, False),
    ).translate((0.0, -0.046, -0.001))
    return lid.union(tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="top_load_impeller_washer")

    cabinet_white = model.material("cabinet_white", rgba=(0.96, 0.96, 0.95, 1.0))
    trim_grey = model.material("trim_grey", rgba=(0.25, 0.27, 0.29, 1.0))
    console_black = model.material("console_black", rgba=(0.12, 0.13, 0.14, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.42, 0.55, 0.60, 0.30))
    basket_metal = model.material("basket_metal", rgba=(0.78, 0.80, 0.82, 1.0))
    drawer_blue = model.material("drawer_blue", rgba=(0.78, 0.86, 0.94, 0.92))
    dial_dark = model.material("dial_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    button_white = model.material("button_white", rgba=(0.95, 0.95, 0.96, 1.0))
    softener_blue = model.material("softener_blue", rgba=(0.78, 0.88, 0.98, 0.92))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shape(), "washer_cabinet"),
        material=cabinet_white,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_tub_ring_shape(), "washer_tub_ring"),
        material=trim_grey,
        name="tub_ring",
    )
    cabinet.visual(
        Box((0.470, 0.004, 0.092)),
        origin=Origin(xyz=(0.055, 0.214, 1.004)),
        material=console_black,
        name="control_panel",
    )
    cabinet.visual(
        Cylinder(radius=0.016, length=0.355),
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, 0.1775)),
        material=trim_grey,
        name="drive_post",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_lid_frame_shape(), "washer_lid_frame"),
        material=trim_grey,
        name="lid_frame",
    )
    lid.visual(
        Box((0.512, 0.442, 0.006)),
        origin=Origin(xyz=(0.0, -0.274, -0.001)),
        material=glass_tint,
        name="glass",
    )

    drawer = model.part("drawer")
    drawer.visual(
        mesh_from_cadquery(_drawer_shape(), "washer_drawer"),
        material=drawer_blue,
        name="drawer_tray",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shape(), "washer_basket"),
        material=basket_metal,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.032, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, -0.452)),
        material=softener_blue,
        name="softener_cup",
    )

    softener_lid = model.part("softener_lid")
    softener_lid.visual(
        mesh_from_cadquery(_softener_lid_shape(), "washer_softener_lid"),
        material=softener_blue,
        name="softener_panel",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.058,
                0.026,
                body_style="skirted",
                top_diameter=0.044,
                center=False,
            ),
            "washer_selector_dial",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dial_dark,
        name="dial_knob",
    )
    dial.visual(
        Box((0.003, 0.004, 0.013)),
        origin=Origin(xyz=(0.0, -0.026, 0.016)),
        material=button_white,
        name="dial_pointer",
    )
    dial.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_grey,
        name="dial_shaft",
    )

    button_positions = (0.055, 0.096, 0.137)
    for index, x_pos in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.024, 0.012, 0.015)),
            origin=Origin(xyz=(0.0, -0.006, 0.0075)),
            material=button_white,
            name="button_cap",
        )
        button.visual(
            Box((0.014, 0.008, 0.009)),
            origin=Origin(xyz=(0.0, 0.000, 0.0045)),
            material=trim_grey,
            name="button_stem",
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x_pos, 0.210, 1.000)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=0.08,
                lower=0.0,
                upper=0.006,
            ),
        )

    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.198, 0.948)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=0.0,
            upper=1.42,
        ),
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(DRAWER_CENTER_X, DRAWER_CENTER_Y, DECK_HEIGHT - 0.003)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.20,
            lower=0.0,
            upper=0.130,
        ),
    )
    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, OPENING_CENTER_Y, 0.895)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=25.0),
    )
    model.articulation(
        "basket_to_softener_lid",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=softener_lid,
        origin=Origin(xyz=(0.0, 0.012, -0.429)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(0.187, 0.210, 1.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lid = object_model.get_part("lid")
    drawer = object_model.get_part("drawer")
    basket = object_model.get_part("basket")
    softener_lid = object_model.get_part("softener_lid")
    button_0 = object_model.get_part("button_0")

    lid_joint = object_model.get_articulation("cabinet_to_lid")
    drawer_joint = object_model.get_articulation("cabinet_to_drawer")
    softener_joint = object_model.get_articulation("basket_to_softener_lid")
    button_joint = object_model.get_articulation("cabinet_to_button_0")

    with ctx.pose({lid_joint: 0.0}):
        ctx.expect_gap(
            lid,
            "cabinet",
            axis="z",
            positive_elem="glass",
            negative_elem="tub_ring",
            min_gap=0.006,
            max_gap=0.030,
            name="glass lid clears the tub ring when closed",
        )
        closed_lid_aabb = ctx.part_world_aabb(lid)

    lid_upper = lid_joint.motion_limits.upper if lid_joint.motion_limits is not None else None
    with ctx.pose({lid_joint: lid_upper or 1.2}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "main lid opens upward",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.22,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    drawer_upper = drawer_joint.motion_limits.upper if drawer_joint.motion_limits is not None else None
    with ctx.pose({drawer_joint: drawer_upper or 0.1}):
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "detergent drawer slides forward",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.10,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    with ctx.pose({softener_joint: 0.0}):
        ctx.expect_gap(
            softener_lid,
            basket,
            axis="z",
            positive_elem="softener_panel",
            negative_elem="softener_cup",
            min_gap=0.0,
            max_gap=0.012,
            name="softener lid sits over the center cup",
        )
        closed_softener_aabb = ctx.part_world_aabb(softener_lid)

    softener_upper = softener_joint.motion_limits.upper if softener_joint.motion_limits is not None else None
    with ctx.pose({softener_joint: softener_upper or 1.0}):
        open_softener_aabb = ctx.part_world_aabb(softener_lid)

    ctx.check(
        "softener lid flips upward",
        closed_softener_aabb is not None
        and open_softener_aabb is not None
        and open_softener_aabb[1][2] > closed_softener_aabb[1][2] + 0.025,
        details=f"closed={closed_softener_aabb}, open={open_softener_aabb}",
    )

    button_rest = ctx.part_world_position(button_0)
    button_upper = button_joint.motion_limits.upper if button_joint.motion_limits is not None else None
    with ctx.pose({button_joint: button_upper or 0.004}):
        button_pressed = ctx.part_world_position(button_0)
    ctx.check(
        "console button presses inward",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[1] > button_rest[1] + 0.004,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
