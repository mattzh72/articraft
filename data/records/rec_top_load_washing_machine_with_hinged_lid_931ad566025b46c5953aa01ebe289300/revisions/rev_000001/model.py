from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
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


WIDTH = 0.68
DEPTH = 0.78
TOP_Z = 0.84
CAVITY_Y = -0.05
CAVITY_R = 0.285
BANK_Y = 0.312
BANK_TOP_Z = TOP_Z + 0.012
HINGE_Y = 0.24
LID_Z = TOP_Z + 0.010
LOCK_Y = -0.377


def _ring(outer_radius: float, inner_radius: float, height: float, z_bottom: float) -> cq.Workplane:
    outer = cq.Workplane("XY").circle(outer_radius).extrude(height).translate((0.0, 0.0, z_bottom))
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(height + 0.020)
        .translate((0.0, 0.0, z_bottom - 0.010))
    )
    return outer.cut(inner)


def _cabinet_shell() -> cq.Workplane:
    body = cq.Workplane("XY").box(WIDTH, DEPTH, TOP_Z, centered=(True, True, False))
    body = body.edges("|Z").fillet(0.018)

    cavity_cut = (
        cq.Workplane("XY")
        .circle(CAVITY_R + 0.015)
        .extrude(TOP_Z + 0.16)
        .translate((0.0, CAVITY_Y, 0.14))
    )
    body = body.cut(cavity_cut)

    latch_slot = (
        cq.Workplane("XY")
        .box(0.064, 0.018, 0.105, centered=(True, True, False))
        .translate((0.0, LOCK_Y, TOP_Z - 0.075))
    )
    return body.cut(latch_slot)


def _tub_liner() -> cq.Workplane:
    wall = _ring(0.292, 0.268, 0.555, 0.245)
    top_flange = _ring(0.313, 0.268, 0.018, 0.790)
    return wall.union(top_flange).translate((0.0, CAVITY_Y, 0.0))


def _basket_shell() -> cq.Workplane:
    height = 0.430
    z_bottom = -height / 2.0
    shell = (
        cq.Workplane("XY")
        .circle(0.235)
        .extrude(height)
        .translate((0.0, 0.0, z_bottom))
    )
    inner_cut = (
        cq.Workplane("XY")
        .circle(0.204)
        .extrude(height + 0.065)
        .translate((0.0, 0.0, z_bottom + 0.032))
    )
    basket = shell.cut(inner_cut)
    basket = basket.union(_ring(0.254, 0.214, 0.034, z_bottom + height - 0.026))

    for row, z in enumerate((-0.115, -0.015, 0.085)):
        for index in range(12):
            angle = index * 30.0 + (15.0 if row % 2 else 0.0)
            slot = (
                cq.Workplane("XY")
                .box(0.082, 0.014, 0.026)
                .translate((0.230, 0.0, z))
                .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
            )
            basket = basket.cut(slot)
    return basket


def _rounded_button() -> cq.Workplane:
    cap = cq.Workplane("XY").box(0.052, 0.028, 0.006, centered=(True, True, False))
    return cap.edges("|Z").fillet(0.004).edges(">Z").fillet(0.0015)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_efficiency_top_load_washer")

    white = model.material("warm_white_plastic", rgba=(0.93, 0.94, 0.91, 1.0))
    light_gray = model.material("soft_gray_trim", rgba=(0.70, 0.72, 0.72, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_glass = model.material("smoked_glass", rgba=(0.08, 0.12, 0.14, 0.36))
    glossy_black = model.material("glossy_black", rgba=(0.01, 0.012, 0.014, 1.0))
    button_black = model.material("button_black", rgba=(0.035, 0.038, 0.042, 1.0))
    icon_blue = model.material("cool_white_icon", rgba=(0.70, 0.90, 1.00, 1.0))
    rubber = model.material("black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.02, 0.022, 0.025, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell", tolerance=0.0015),
        material=white,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_tub_liner(), "tub_liner", tolerance=0.0015),
        material=shadow,
        name="tub_liner",
    )
    cabinet.visual(
        Cylinder(radius=0.055, length=0.150),
        origin=Origin(xyz=(0.0, CAVITY_Y, 0.185)),
        material=light_gray,
        name="drive_bearing",
    )
    cabinet.visual(
        Box((0.610, 0.120, 0.012)),
        origin=Origin(xyz=(0.0, BANK_Y, TOP_Z + 0.006)),
        material=glossy_black,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.540, 0.004, 0.430)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 - 0.002, 0.405)),
        material=Material("front_panel_sheen", rgba=(0.86, 0.87, 0.84, 1.0)),
        name="front_panel",
    )
    cabinet.visual(
        Box((0.560, 0.030, 0.055)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 + 0.014, 0.040)),
        material=shadow,
        name="toe_kick",
    )
    for x in (-0.318, 0.318):
        cabinet.visual(
            Box((0.030, 0.050, 0.035)),
            origin=Origin(xyz=(x, HINGE_Y + 0.006, TOP_Z + 0.0175)),
            material=light_gray,
            name=f"hinge_pedestal_{'neg' if x < 0 else 'pos'}",
        )
    cabinet.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.040, 0.012),
                (0.074, 0.026),
                0.008,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.004,
                outer_corner_radius=0.008,
                center=False,
            ),
            "lock_guide",
        ),
        origin=Origin(xyz=(0.0, LOCK_Y, TOP_Z)),
        material=glossy_black,
        name="lock_guide",
    )

    basket = model.part("basket")
    basket.visual(
        mesh_from_cadquery(_basket_shell(), "basket_shell", tolerance=0.0012),
        material=stainless,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.102, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.171)),
        material=light_gray,
        name="low_impeller",
    )
    basket.visual(
        Cylinder(radius=0.035, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        material=light_gray,
        name="drive_shaft",
    )
    model.articulation(
        "basket_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(0.0, CAVITY_Y, 0.535)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=18.0),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.458, 0.478),
                (0.580, 0.600),
                0.026,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.050,
                outer_corner_radius=0.070,
                center=False,
            ),
            "lid_frame",
        ),
        origin=Origin(xyz=(0.0, -0.300, 0.0)),
        material=white,
        name="lid_frame",
    )
    lid.visual(
        Box((0.486, 0.506, 0.006)),
        origin=Origin(xyz=(0.0, -0.300, 0.014)),
        material=dark_glass,
        name="glass_pane",
    )
    lid.visual(
        Cylinder(radius=0.014, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=light_gray,
        name="hinge_sleeve",
    )
    bumper_specs = (
        ("bumper_0", -0.265, -0.525),
        ("bumper_1", 0.265, -0.525),
        ("bumper_2", -0.265, -0.100),
        ("bumper_3", 0.265, -0.100),
    )
    for bumper_name, x, y in bumper_specs:
        lid.visual(
            Box((0.045, 0.026, 0.010)),
            origin=Origin(xyz=(x, y, -0.005)),
            material=rubber,
            name=bumper_name,
        )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, LID_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=1.22),
    )

    knob_geo = KnobGeometry(
        0.086,
        0.038,
        body_style="skirted",
        top_diameter=0.070,
        skirt=KnobSkirt(0.104, 0.006, flare=0.06, chamfer=0.001),
        grip=KnobGrip(style="ribbed", count=28, depth=0.0012, width=0.0020),
        indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
        center=False,
    )
    selector_knob = model.part("selector_knob")
    selector_knob.visual(
        mesh_from_geometry(knob_geo, "selector_knob"),
        material=light_gray,
        name="knob_body",
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=selector_knob,
        origin=Origin(xyz=(0.0, BANK_Y, BANK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=9.0),
    )

    button_mesh = mesh_from_cadquery(_rounded_button(), "touch_button", tolerance=0.0008)
    button_positions = []
    for x in (-0.245, -0.175, -0.105, 0.105, 0.175, 0.245):
        button_positions.append((x, BANK_Y - 0.026))
        button_positions.append((x, BANK_Y + 0.026))
    for index, (x, y) in enumerate(button_positions):
        button = model.part(f"button_{index}")
        button.visual(button_mesh, material=button_black, name="button_cap")
        button.visual(
            Box((0.024, 0.003, 0.001)),
            origin=Origin(xyz=(0.0, 0.0, 0.0065)),
            material=icon_blue,
            name="button_icon",
        )
        model.articulation(
            f"button_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, y, BANK_TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.06, lower=0.0, upper=0.006),
        )

    plunger = model.part("lock_plunger")
    plunger.visual(
        Box((0.058, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=glossy_black,
        name="plunger_cap",
    )
    plunger.visual(
        Box((0.030, 0.008, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=light_gray,
        name="plunger_tongue",
    )
    model.articulation(
        "plunger_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=plunger,
        origin=Origin(xyz=(0.0, LOCK_Y, TOP_Z + 0.008)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.032),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cabinet = object_model.get_part("cabinet")
    lid = object_model.get_part("lid")
    basket = object_model.get_part("basket")
    knob = object_model.get_part("selector_knob")
    button_0 = object_model.get_part("button_0")
    plunger = object_model.get_part("lock_plunger")
    lid_hinge = object_model.get_articulation("lid_hinge")
    basket_spin = object_model.get_articulation("basket_spin")
    knob_spin = object_model.get_articulation("knob_spin")
    button_slide = object_model.get_articulation("button_slide_0")
    plunger_slide = object_model.get_articulation("plunger_slide")

    ctx.allow_overlap(
        cabinet,
        basket,
        elem_a="drive_bearing",
        elem_b="drive_shaft",
        reason="The rotating basket drive shaft is intentionally captured inside the fixed bearing proxy.",
    )
    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        inner_elem="drive_shaft",
        outer_elem="drive_bearing",
        margin=0.0,
        name="basket drive shaft is centered in bearing",
    )
    ctx.expect_overlap(
        basket,
        cabinet,
        axes="z",
        elem_a="drive_shaft",
        elem_b="drive_bearing",
        min_overlap=0.080,
        name="basket drive shaft remains inserted in bearing",
    )

    ctx.check(
        "primary mechanisms are articulated",
        lid_hinge.articulation_type == ArticulationType.REVOLUTE
        and basket_spin.articulation_type == ArticulationType.CONTINUOUS
        and knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and button_slide.articulation_type == ArticulationType.PRISMATIC
        and plunger_slide.articulation_type == ArticulationType.PRISMATIC,
        details="Expected revolute lid, continuous basket/knob, and prismatic controls.",
    )
    ctx.check(
        "touch bank has separate buttons",
        sum(1 for part in object_model.parts if part.name.startswith("button_")) == 12,
        details="The touch-style bank should be split into individual push-button parts.",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="bumper_0",
            negative_elem="cabinet_shell",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed lid front bumper seats on deck",
        )
        ctx.expect_gap(
            lid,
            cabinet,
            axis="z",
            positive_elem="bumper_2",
            negative_elem="cabinet_shell",
            max_gap=0.002,
            max_penetration=0.0,
            name="closed lid rear bumper seats on deck",
        )

    closed_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.10}):
        open_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid rotates upward on rear hinge",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.16
        and open_aabb[0][1] > closed_aabb[0][1] + 0.10,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="tub_liner",
        margin=0.0,
        name="rotating basket stays inside fixed tub liner",
    )
    liner_aabb = ctx.part_element_world_aabb(cabinet, elem="tub_liner")
    basket_aabb = ctx.part_element_world_aabb(basket, elem="basket_shell")
    ctx.check(
        "tub opening has visible laundry depth",
        liner_aabb is not None
        and basket_aabb is not None
        and (liner_aabb[1][2] - liner_aabb[0][2]) > 0.50
        and (basket_aabb[1][2] - basket_aabb[0][2]) > 0.40,
        details=f"liner_aabb={liner_aabb}, basket_aabb={basket_aabb}",
    )

    ctx.expect_gap(
        knob,
        cabinet,
        axis="z",
        positive_elem="knob_body",
        negative_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="selector knob seats on control bank",
    )
    ctx.expect_gap(
        button_0,
        cabinet,
        axis="z",
        positive_elem="button_cap",
        negative_elem="control_panel",
        max_gap=0.001,
        max_penetration=0.0,
        name="touch button cap rests on panel",
    )
    rest_button = ctx.part_world_position(button_0)
    with ctx.pose({button_slide: 0.006}):
        pressed_button = ctx.part_world_position(button_0)
    ctx.check(
        "touch button depresses downward",
        rest_button is not None and pressed_button is not None and pressed_button[2] < rest_button[2] - 0.004,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    ctx.expect_gap(
        plunger,
        cabinet,
        axis="z",
        positive_elem="plunger_cap",
        negative_elem="lock_guide",
        max_gap=0.001,
        max_penetration=0.0,
        name="lid lock plunger cap seats on guide",
    )
    rest_plunger = ctx.part_world_position(plunger)
    with ctx.pose({plunger_slide: 0.028}):
        down_plunger = ctx.part_world_position(plunger)
    ctx.check(
        "lid lock plunger slides into top deck",
        rest_plunger is not None and down_plunger is not None and down_plunger[2] < rest_plunger[2] - 0.020,
        details=f"rest={rest_plunger}, down={down_plunger}",
    )

    return ctx.report()


object_model = build_object_model()
