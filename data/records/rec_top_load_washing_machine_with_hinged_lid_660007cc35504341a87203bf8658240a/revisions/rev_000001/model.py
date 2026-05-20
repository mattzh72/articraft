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
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


WHITE = Material("white_enamel", color=(0.92, 0.94, 0.93, 1.0))
SHADOW = Material("deep_shadow", color=(0.02, 0.025, 0.03, 1.0))
BLACK = Material("black_plastic", color=(0.015, 0.016, 0.018, 1.0))
GLASS = Material("smoked_glass", color=(0.10, 0.16, 0.20, 0.45))
STEEL = Material("brushed_stainless", color=(0.72, 0.75, 0.76, 1.0))
LIGHT_GRAY = Material("warm_gray_plastic", color=(0.72, 0.72, 0.69, 1.0))
BLUE = Material("softener_blue", color=(0.42, 0.58, 0.74, 1.0))


def _cabinet_body() -> cq.Workplane:
    """One connected static shell with a deep blind tub well and rear console."""
    width = 0.68
    depth = 0.76
    body_h = 0.90
    opening_r = 0.315
    tub_y = -0.04

    body = cq.Workplane("XY").box(width, depth, body_h).translate((0.0, 0.0, body_h / 2.0))
    # A blind cylindrical well keeps the top-load tub visibly hollow and deep.
    well_cut = (
        cq.Workplane("XY")
        .circle(opening_r)
        .extrude(0.78)
        .translate((0.0, tub_y, 0.18))
    )
    body = body.cut(well_cut)

    # A drive pedestal rises from the bottom of the blind well to support the basket axis.
    pedestal = cq.Workplane("XY").circle(0.045).extrude(0.18).translate((0.0, tub_y, 0.18))
    body = body.union(pedestal)

    # Raised rear console: a low riser clears the dispenser drawer while keeping the
    # console physically connected to the enamel top deck.
    riser = cq.Workplane("XY").box(0.62, 0.045, 0.055).translate((0.0, 0.405, 0.9275))
    console = cq.Workplane("XY").box(0.62, 0.12, 0.18).translate((0.0, 0.39, 1.04))
    body = body.union(riser).union(console)

    return body


def _basket_shell() -> cq.Workplane:
    """Perforated stainless spin basket with connected rim and bottom."""
    outer = 0.245
    inner = 0.224
    height = 0.43
    side = cq.Workplane("XY").circle(outer).circle(inner).extrude(height)
    bottom = cq.Workplane("XY").circle(inner).extrude(0.026)
    rim = (
        cq.Workplane("XY")
        .circle(outer + 0.010)
        .circle(inner - 0.006)
        .extrude(0.026)
        .translate((0.0, 0.0, height - 0.006))
    )
    basket = side.union(bottom).union(rim)

    for i in range(24):
        angle = i * 360.0 / 24.0
        slot = (
            cq.Workplane("XY")
            .box(0.070, 0.018, 0.245)
            .translate((outer, 0.0, 0.205))
            .rotate((0, 0, 0), (0, 0, 1), angle)
        )
        basket = basket.cut(slot)

    return basket


def _wash_plate() -> cq.Workplane:
    """Low impeller wash plate with three raised vanes and a center hub."""
    plate = cq.Workplane("XY").circle(0.178).extrude(0.018).translate((0.0, 0.0, 0.026))
    hub = cq.Workplane("XY").circle(0.067).extrude(0.050).translate((0.0, 0.0, 0.044))
    result = plate.union(hub)
    for i in range(3):
        vane = (
            cq.Workplane("XY")
            .box(0.142, 0.030, 0.026)
            .translate((0.082, 0.0, 0.064))
            .rotate((0, 0, 0), (0, 0, 1), i * 120.0 + 18.0)
        )
        result = result.union(vane)
    return result


def _top_opening_lip() -> cq.Workplane:
    """Thin circular enamel lip that makes the broad tub opening read round."""
    return (
        cq.Workplane("XY")
        .circle(0.325)
        .circle(0.292)
        .extrude(0.018)
        .translate((0.0, -0.04, 0.884))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="impeller_top_load_washer")

    cabinet = model.part("cabinet")
    cabinet.visual(Box((0.68, 0.040, 0.88)), origin=Origin(xyz=(0.0, -0.360, 0.44)), material=WHITE, name="front_panel")
    cabinet.visual(Box((0.68, 0.040, 0.88)), origin=Origin(xyz=(0.0, 0.360, 0.44)), material=WHITE, name="rear_panel")
    cabinet.visual(Box((0.040, 0.76, 0.88)), origin=Origin(xyz=(-0.320, 0.0, 0.44)), material=WHITE, name="side_panel_0")
    cabinet.visual(Box((0.040, 0.76, 0.88)), origin=Origin(xyz=(0.320, 0.0, 0.44)), material=WHITE, name="side_panel_1")
    cabinet.visual(Box((0.68, 0.76, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.09)), material=WHITE, name="base_floor")
    cabinet.visual(Box((0.68, 0.18, 0.020)), origin=Origin(xyz=(0.0, -0.290, 0.890)), material=WHITE, name="front_deck")
    cabinet.visual(Box((0.68, 0.090, 0.020)), origin=Origin(xyz=(0.0, 0.335, 0.890)), material=WHITE, name="rear_deck")
    cabinet.visual(Box((0.105, 0.500, 0.020)), origin=Origin(xyz=(-0.287, 0.045, 0.890)), material=WHITE, name="side_deck_0")
    cabinet.visual(Box((0.105, 0.500, 0.020)), origin=Origin(xyz=(0.287, 0.045, 0.890)), material=WHITE, name="side_deck_1")
    cabinet.visual(Box((0.62, 0.12, 0.17)), origin=Origin(xyz=(0.0, 0.390, 1.025)), material=WHITE, name="rear_console")
    cabinet.visual(Box((0.040, 0.120, 0.050)), origin=Origin(xyz=(-0.150, 0.390, 0.925)), material=WHITE, name="console_riser_0")
    cabinet.visual(Box((0.040, 0.120, 0.050)), origin=Origin(xyz=(0.250, 0.390, 0.925)), material=WHITE, name="console_riser_1")
    cabinet.visual(
        mesh_from_cadquery(_top_opening_lip(), "round_top_opening_lip", tolerance=0.002),
        material=WHITE,
        name="top_opening_lip",
    )
    cabinet.visual(Cylinder(radius=0.045, length=0.18), origin=Origin(xyz=(0.0, -0.04, 0.27)), material=WHITE, name="drive_pedestal")
    for x, y, sx, sy, n in (
        (-0.304, -0.04, 0.012, 0.56, "inner_wall_0"),
        (0.304, -0.04, 0.012, 0.56, "inner_wall_1"),
        (0.0, -0.318, 0.56, 0.012, "inner_wall_2"),
        (0.0, 0.238, 0.56, 0.012, "inner_wall_3"),
    ):
        cabinet.visual(Box((sx, sy, 0.67)), origin=Origin(xyz=(x, y, 0.545)), material=SHADOW, name=n)
    cabinet.visual(
        Box((0.50, 0.006, 0.115)),
        origin=Origin(xyz=(0.0, 0.330, 1.045)),
        material=BLACK,
        name="console_panel",
    )
    cabinet.visual(
        Box((0.115, 0.084, 0.004)),
        origin=Origin(xyz=(-0.300, 0.346, 0.902)),
        material=SHADOW,
        name="drawer_recess",
    )
    cabinet.visual(
        Box((0.42, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.272, 0.906)),
        material=BLACK,
        name="rear_hinge_gasket",
    )
    cabinet.visual(
        Box((0.50, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.382, 0.53)),
        material=SHADOW,
        name="front_panel_seam",
    )

    basket = model.part("wash_basket")
    basket.visual(
        mesh_from_cadquery(_basket_shell(), "perforated_wash_basket", tolerance=0.002),
        material=STEEL,
        name="basket_shell",
    )
    basket.visual(
        mesh_from_cadquery(_wash_plate(), "impeller_wash_plate", tolerance=0.002),
        material=LIGHT_GRAY,
        name="wash_plate",
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, -0.04, 0.36)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=8.0),
    )

    lid = model.part("glass_lid")
    lid.visual(
        Box((0.585, 0.530, 0.016)),
        origin=Origin(xyz=(0.0, -0.275, 0.018)),
        material=GLASS,
        name="glass_panel",
    )
    lid.visual(
        Box((0.625, 0.030, 0.030)),
        origin=Origin(xyz=(0.0, -0.010, 0.020)),
        material=BLACK,
        name="rear_frame",
    )
    lid.visual(
        Box((0.625, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, -0.540, 0.020)),
        material=BLACK,
        name="front_frame",
    )
    for x in (-0.307, 0.307):
        lid.visual(
            Box((0.032, 0.530, 0.030)),
            origin=Origin(xyz=(x, -0.275, 0.020)),
            material=BLACK,
            name=f"side_frame_{0 if x < 0 else 1}",
        )
    lid.visual(
        Cylinder(radius=0.012, length=0.46),
        origin=Origin(xyz=(0.0, 0.002, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLACK,
        name="hinge_barrel",
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.272, 0.913)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=0.0, upper=1.30),
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        Box((0.070, 0.080, 0.026)),
        origin=Origin(xyz=(0.0, -0.040, 0.013)),
        material=LIGHT_GRAY,
        name="drawer_tray",
    )
    drawer.visual(
        Box((0.056, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.079, 0.026)),
        material=BLUE,
        name="drawer_pull_lip",
    )
    model.articulation(
        "drawer_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(-0.300, 0.386, 0.902)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.12),
    )

    dial = model.part("selector_dial")
    dial_mesh = mesh_from_geometry(
        KnobGeometry(
            0.088,
            0.036,
            body_style="skirted",
            top_diameter=0.068,
            skirt=KnobSkirt(0.098, 0.006, flare=0.04, chamfer=0.001),
            grip=KnobGrip(style="ribbed", count=24, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        ),
        "selector_dial_cap",
    )
    dial.visual(
        dial_mesh,
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=BLACK,
        name="dial_cap",
    )
    model.articulation(
        "dial_turn",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=dial,
        origin=Origin(xyz=(0.170, 0.323, 1.050)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=3.0),
    )

    for i, x in enumerate((-0.125, -0.070, -0.015)):
        button = model.part(f"button_{i}")
        button.visual(
            Box((0.038, 0.018, 0.026)),
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=LIGHT_GRAY,
            name="button_cap",
        )
        model.articulation(
            f"button_{i}_push",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.327, 1.050)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.08, lower=0.0, upper=0.010),
        )

    softener_lid = model.part("softener_lid")
    softener_lid.visual(
        Cylinder(radius=0.052, length=0.012),
        origin=Origin(xyz=(0.0, -0.052, 0.006)),
        material=BLUE,
        name="cup_cap",
    )
    softener_lid.visual(
        Box((0.075, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.006, 0.006)),
        material=BLUE,
        name="hinge_tab",
    )
    softener_lid.visual(
        Cylinder(radius=0.008, length=0.078),
        origin=Origin(xyz=(0.0, 0.020, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=BLUE,
        name="cup_hinge_barrel",
    )
    model.articulation(
        "softener_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=basket,
        child=softener_lid,
        origin=Origin(xyz=(0.0, 0.058, 0.094)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    basket = object_model.get_part("wash_basket")
    lid = object_model.get_part("glass_lid")
    drawer = object_model.get_part("detergent_drawer")
    softener_lid = object_model.get_part("softener_lid")

    lid_hinge = object_model.get_articulation("lid_hinge")
    drawer_slide = object_model.get_articulation("drawer_slide")
    softener_hinge = object_model.get_articulation("softener_lid_hinge")

    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        elem_a="basket_shell",
        elem_b="top_opening_lip",
        margin=0.0,
        name="basket sits within the broad hollow tub opening",
    )
    ctx.expect_gap(
        basket,
        cabinet,
        axis="z",
        positive_elem="basket_shell",
        negative_elem="drive_pedestal",
        max_gap=0.01,
        max_penetration=0.002,
        name="basket is seated on the central drive pedestal",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.15}):
        open_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "glass lid rotates upward on rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    drawer_rest = ctx.part_world_position(drawer)
    with ctx.pose({drawer_slide: 0.10}):
        drawer_extended = ctx.part_world_position(drawer)
    ctx.check(
        "detergent drawer slides forward from top deck",
        drawer_rest is not None
        and drawer_extended is not None
        and drawer_extended[1] < drawer_rest[1] - 0.08,
        details=f"rest={drawer_rest}, extended={drawer_extended}",
    )

    cap_closed = ctx.part_world_aabb(softener_lid)
    with ctx.pose({softener_hinge: 1.20}):
        cap_open = ctx.part_world_aabb(softener_lid)
    ctx.check(
        "softener cup lid flips up from center hub",
        cap_closed is not None
        and cap_open is not None
        and cap_open[1][2] > cap_closed[1][2] + 0.025,
        details=f"closed={cap_closed}, open={cap_open}",
    )

    for i in range(3):
        button = object_model.get_part(f"button_{i}")
        joint = object_model.get_articulation(f"button_{i}_push")
        rest = ctx.part_world_position(button)
        with ctx.pose({joint: 0.008}):
            pressed = ctx.part_world_position(button)
        ctx.check(
            f"button {i} is a separate prismatic push control",
            rest is not None and pressed is not None and pressed[1] > rest[1] + 0.006,
            details=f"rest={rest}, pressed={pressed}",
        )

    return ctx.report()


object_model = build_object_model()
