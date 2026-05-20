from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


WIDTH = 0.76
DEPTH = 0.74
BODY_H = 0.90
OPEN_X = -0.055
OPEN_Y = -0.080
OPEN_R = 0.220


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """A centered box with softly rounded vertical plan corners."""
    return cq.Workplane("XY").box(*size).edges("|Z").fillet(radius)


def _cabinet_shell() -> cq.Workplane:
    body = (
        _rounded_box((WIDTH, DEPTH, BODY_H), 0.035)
        .translate((0.0, 0.0, BODY_H / 2.0))
    )

    # Rear control tower fused into the top deck.
    tower = (
        _rounded_box((WIDTH * 0.95, 0.165, 0.255), 0.020)
        .translate((0.0, 0.305, BODY_H + 0.110))
    )
    body = body.union(tower)

    # A deep cylindrical well through the top deck: the opening remains visibly
    # hollow, with a floor well below the suspended stainless basket.
    wash_well = (
        cq.Workplane("XY")
        .center(OPEN_X, OPEN_Y)
        .circle(OPEN_R)
        .extrude(0.79)
        .translate((0.0, 0.0, 0.16))
    )
    body = body.cut(wash_well)

    return body


def _opening_gasket() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(OPEN_R + 0.028).extrude(0.018)
    inner = (
        cq.Workplane("XY")
        .circle(OPEN_R + 0.002)
        .extrude(0.024)
        .translate((0.0, 0.0, -0.003))
    )
    return outer.cut(inner).translate((OPEN_X, OPEN_Y, BODY_H))


def _drawer_runner() -> cq.Workplane:
    rail_a = cq.Workplane("XY").box(0.012, 0.270, 0.020).translate((0.239, 0.010, BODY_H + 0.010))
    rail_b = cq.Workplane("XY").box(0.012, 0.270, 0.020).translate((0.371, 0.010, BODY_H + 0.010))
    bridge = cq.Workplane("XY").box(0.144, 0.014, 0.020).translate((0.305, 0.150, BODY_H + 0.010))
    return rail_a.union(rail_b).union(bridge)


def _hollow_basket() -> cq.Workplane:
    height = 0.500
    outer_r = 0.202
    inner_r = 0.170
    outer = cq.Workplane("XY").circle(outer_r).extrude(height)
    inner = (
        cq.Workplane("XY")
        .circle(inner_r)
        .extrude(height + 0.030)
        .translate((0.0, 0.0, 0.045))
    )
    cup = outer.cut(inner)

    # A small lower spindle and a raised impeller disk make the rotating basket
    # read as a mounted wash assembly without filling the clothing volume.
    spindle = (
        cq.Workplane("XY")
        .circle(0.040)
        .extrude(0.110)
        .translate((0.0, 0.0, -0.085))
    )
    impeller = (
        cq.Workplane("XY")
        .circle(0.090)
        .extrude(0.018)
        .translate((0.0, 0.0, 0.043))
    )
    return cup.union(spindle).union(impeller).translate((0.0, 0.0, -height / 2.0))


def _lid_panel() -> cq.Workplane:
    # The lid's local frame is the rear hinge line.  The rounded panel extends
    # toward local -Y so a -X hinge axis opens it upward.
    panel = (
        _rounded_box((0.565, 0.485, 0.034), 0.060)
        .translate((OPEN_X, -0.245, 0.021))
    )
    glass = (
        _rounded_box((0.435, 0.335, 0.010), 0.050)
        .translate((OPEN_X, -0.245, 0.043))
    )
    return panel, glass


def _detergent_tray() -> cq.Workplane:
    outer = cq.Workplane("XY").box(0.140, 0.242, 0.047).translate((0.0, 0.0, -0.011))
    inner = cq.Workplane("XY").box(0.106, 0.192, 0.040).translate((0.0, -0.004, 0.006))
    tray = outer.cut(inner)
    front_lip = cq.Workplane("XY").box(0.134, 0.030, 0.034).translate((0.0, -0.136, -0.001))
    return tray.union(front_lip)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_top_load_washer")

    enamel = model.material("white_enamel", rgba=(0.93, 0.94, 0.91, 1.0))
    shadow = model.material("deep_shadow", rgba=(0.03, 0.035, 0.040, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.006, 0.008, 1.0))
    glass = model.material("smoked_glass", rgba=(0.12, 0.18, 0.22, 0.42))
    steel = model.material("brushed_steel", rgba=(0.68, 0.70, 0.69, 1.0))
    panel = model.material("charcoal_panel", rgba=(0.08, 0.085, 0.09, 1.0))
    blue = model.material("detergent_blue", rgba=(0.35, 0.62, 0.86, 1.0))
    grey = model.material("soft_grey", rgba=(0.55, 0.57, 0.58, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        mesh_from_cadquery(_cabinet_shell(), "cabinet_shell", tolerance=0.002),
        material=enamel,
        name="cabinet_shell",
    )
    cabinet.visual(
        mesh_from_cadquery(_opening_gasket(), "opening_gasket", tolerance=0.0015),
        material=black,
        name="opening_gasket",
    )
    cabinet.visual(
        mesh_from_cadquery(_drawer_runner(), "drawer_runner", tolerance=0.0015),
        material=grey,
        name="drawer_runner",
    )
    cabinet.visual(
        Box((0.560, 0.010, 0.125)),
        origin=Origin(xyz=(0.0, 0.2175, 1.030)),
        material=panel,
        name="control_panel",
    )
    cabinet.visual(
        Box((0.420, 0.009, 0.105)),
        origin=Origin(xyz=(0.0, -DEPTH / 2.0 - 0.0045, 0.500)),
        material=Material("front_shadow", rgba=(0.80, 0.82, 0.80, 1.0)),
        name="front_inset",
    )
    cabinet.visual(
        Cylinder(radius=0.018, length=0.430),
        origin=Origin(xyz=(OPEN_X, 0.196, BODY_H + 0.005), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="hinge_support",
    )
    cabinet.visual(
        Cylinder(radius=0.038, length=0.105),
        origin=Origin(xyz=(OPEN_X, OPEN_Y, 0.2125)),
        material=black,
        name="basket_bearing",
    )

    basket = model.part("wash_basket")
    basket.visual(
        mesh_from_cadquery(_hollow_basket(), "wash_basket", tolerance=0.0015),
        origin=Origin(),
        material=steel,
        name="basket_shell",
    )

    lid = model.part("lid")
    lid_panel, lid_glass = _lid_panel()
    lid.visual(
        mesh_from_cadquery(lid_panel, "lid_panel", tolerance=0.0015),
        material=enamel,
        name="lid_panel",
    )
    lid.visual(
        mesh_from_cadquery(lid_glass, "lid_glass", tolerance=0.0015),
        material=glass,
        name="lid_glass",
    )
    lid.visual(
        Cylinder(radius=0.018, length=0.430),
        origin=Origin(xyz=(OPEN_X, 0.006, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grey,
        name="hinge_barrel",
    )
    lid.visual(
        Box((0.230, 0.028, 0.018)),
        origin=Origin(xyz=(OPEN_X, -0.493, 0.040)),
        material=grey,
        name="front_handle",
    )

    drawer = model.part("detergent_drawer")
    drawer.visual(
        mesh_from_cadquery(_detergent_tray(), "detergent_drawer", tolerance=0.0015),
        material=blue,
        name="drawer_tray",
    )

    knob = model.part("selector_knob")
    knob.visual(
        Cylinder(radius=0.055, length=0.036),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grey,
        name="knob_cap",
    )
    knob.visual(
        Box((0.010, 0.006, 0.070)),
        origin=Origin(xyz=(0.0, -0.038, 0.018)),
        material=black,
        name="pointer_mark",
    )

    for index, x in enumerate((0.155, 0.225, 0.295)):
        button = model.part(f"button_{index}")
        button.visual(
            Cylinder(radius=0.024, length=0.020),
            origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=grey,
            name="button_cap",
        )
        model.articulation(
            f"cabinet_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=cabinet,
            child=button,
            origin=Origin(xyz=(x, 0.2125, 1.018)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=0.15, lower=0.0, upper=0.010),
        )

    model.articulation(
        "cabinet_to_basket",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=basket,
        origin=Origin(xyz=(OPEN_X, OPEN_Y, 0.600)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=20.0),
    )
    model.articulation(
        "cabinet_to_lid",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lid,
        origin=Origin(xyz=(0.0, 0.190, BODY_H + 0.020)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=0.0, upper=1.25),
    )
    model.articulation(
        "cabinet_to_detergent_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.305, 0.025, BODY_H + 0.0465)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.180),
    )
    model.articulation(
        "cabinet_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=(-0.185, 0.2125, 1.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    basket = object_model.get_part("wash_basket")
    lid = object_model.get_part("lid")
    drawer = object_model.get_part("detergent_drawer")
    knob = object_model.get_part("selector_knob")
    lid_hinge = object_model.get_articulation("cabinet_to_lid")
    drawer_slide = object_model.get_articulation("cabinet_to_detergent_drawer")
    knob_spin = object_model.get_articulation("cabinet_to_selector_knob")

    ctx.allow_overlap(
        cabinet,
        drawer,
        elem_a="drawer_runner",
        elem_b="drawer_tray",
        reason="The detergent drawer tray is intentionally captured by shallow molded slide runners in the top deck.",
    )

    ctx.expect_within(
        basket,
        cabinet,
        axes="xy",
        margin=0.0,
        name="wash basket sits within cabinet footprint",
    )

    rest_lid_aabb = ctx.part_world_aabb(lid)
    with ctx.pose({lid_hinge: 1.05}):
        raised_lid_aabb = ctx.part_world_aabb(lid)
    ctx.check(
        "lid opens upward on rear hinge",
        rest_lid_aabb is not None
        and raised_lid_aabb is not None
        and raised_lid_aabb[1][2] > rest_lid_aabb[1][2] + 0.18,
        details=f"closed={rest_lid_aabb}, raised={raised_lid_aabb}",
    )

    rest_drawer = ctx.part_world_position(drawer)
    ctx.expect_gap(
        drawer,
        cabinet,
        axis="z",
        positive_elem="drawer_tray",
        negative_elem="drawer_runner",
        max_penetration=0.010,
        name="drawer tray is lightly captured on deck runners",
    )
    with ctx.pose({drawer_slide: 0.160}):
        extended_drawer = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_tray",
            elem_b="drawer_runner",
            min_overlap=0.040,
            name="detergent drawer remains retained in its deck pocket",
        )
    ctx.check(
        "detergent drawer slides toward the user",
        rest_drawer is not None
        and extended_drawer is not None
        and extended_drawer[1] < rest_drawer[1] - 0.12,
        details=f"rest={rest_drawer}, extended={extended_drawer}",
    )

    ctx.check(
        "selector knob is continuous",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_spin.articulation_type}",
    )
    ctx.expect_contact(
        knob,
        cabinet,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.006,
        name="selector knob is mounted on the control tower",
    )

    for index in range(3):
        button = object_model.get_part(f"button_{index}")
        push = object_model.get_articulation(f"cabinet_to_button_{index}")
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({push: 0.008}):
            pushed_pos = ctx.part_world_position(button)
        ctx.check(
            f"button {index} pushes into tower",
            rest_pos is not None and pushed_pos is not None and pushed_pos[1] > rest_pos[1] + 0.006,
            details=f"rest={rest_pos}, pushed={pushed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
